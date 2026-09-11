"""Deterministic DSP branch: *when* should motion occur?

Consumes AudioFrames and produces timing events - onsets and (once a tempo
locks) beats - plus per-band energy. No learning here; this branch exists to
be accurate about time.

Method, deliberately simple for bring-up:

  1. Hann-window each block, take the magnitude FFT.
  2. Spectral flux = sum of positive bin-to-bin increases vs the last block.
  3. Adaptive threshold from a rolling window of recent flux
     (mean + k*std). A local maximum above threshold, past a refractory
     gap, is an onset.
  4. A phase-locked loop over committed onsets tracks tempo + phase;
     ``predict_next_beat`` reads the next beat time off it.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from host.audio.capture import AudioFrame

MIN_BPM = 60.0
MAX_BPM = 200.0
REFRACTORY_S = 0.12                    # ignore onsets closer than this (>= 500 BPM noise)
FLUX_HISTORY_S = 1.5                   # rolling window for the adaptive threshold
THRESHOLD_K = 1.6                      # onset if flux > mean + K*std of the window

_BANDS = {"low": (20, 250), "mid": (250, 2000), "high": (2000, 8000)}


@dataclass
class BeatEvent:
    t: float                          # seconds (monotonic), estimated event time
    kind: str                         # "onset" | "beat"
    strength: float                   # 0..1, normalised flux at the peak
    bands: dict[str, float] = field(default_factory=dict)  # low/mid/high energy


class BeatDetector:
    def __init__(self, sample_rate: int, block_size: int) -> None:
        self.sample_rate = sample_rate
        self.block_size = block_size
        self.bpm: float | None = None

        self._window = np.hanning(block_size).astype(np.float32)
        self._freqs = np.fft.rfftfreq(block_size, 1.0 / sample_rate)
        self._prev_mag: np.ndarray | None = None

        maxlen = max(4, int(FLUX_HISTORY_S * sample_rate / block_size))
        self._flux_hist: deque[float] = deque(maxlen=maxlen)
        self._flux_ceiling = 1e-9      # running max, for normalising strength

        self._last_onset_t: float | None = None
        self._iois: deque[float] = deque(maxlen=8)
        self._pending_peak: tuple[float, float, dict] | None = None  # (t, flux, bands)

        # the detector reports an onset ~0.8 blocks after it actually happened
        # (block quantisation + the one-block peak-pick confirm). The PLL
        # anchors on the true onset by subtracting this. Tune against measured
        # detection lag (analysis/latency_analysis.py prints it).
        self.report_latency_s = 0.8 * block_size / sample_rate

        # phase-locked loop over committed onsets: integrates jitter, rejects
        # outliers, tracks tempo drift.
        self._pll_period: float | None = None
        self._pll_ref: float = 0.0        # true-onset time of some beat index

    # -- main entry ------------------------------------------------------
    def push(self, frame: AudioFrame) -> list[BeatEvent]:
        """Feed one frame, get back any events that fired in it."""
        mag = np.abs(np.fft.rfft(frame.samples * self._window))
        bands = self._band_energy(mag)

        events: list[BeatEvent] = []
        if self._prev_mag is not None:
            flux = float(np.sum(np.maximum(mag - self._prev_mag, 0.0)))
            self._flux_ceiling = max(self._flux_ceiling * 0.999, flux)

            if len(self._flux_hist) >= self._flux_hist.maxlen:
                hist = np.fromiter(self._flux_hist, dtype=np.float64)
                thresh = hist.mean() + THRESHOLD_K * hist.std()
                events = self._peak_pick(frame.t_capture, flux, thresh, bands)

            self._flux_hist.append(flux)
        self._prev_mag = mag
        return events

    # -- tempo ----------------------------------------------------------
    def predict_next_beat(self, now: float | None = None, min_lead: float = 0.0) -> float | None:
        """First beat at least `min_lead` seconds after `now`, from the PLL.

        The PLL (updated on every committed onset) has already integrated out
        per-beat detection jitter and outliers, so consecutive calls for the
        same upcoming beat return a stable target. `min_lead` skips a beat
        that's too soon to schedule for and takes the next one instead. This
        is the predictive-sync target: schedule the command at
        ``predict_next_beat() - L``.
        """
        if self._pll_period is None:
            return None
        ref = (float(now) if now is not None else (self._last_onset_t or self._pll_ref)) + min_lead
        n = math.floor((ref - self._pll_ref) / self._pll_period) + 1.0
        return self._pll_ref + n * self._pll_period

    def _pll_update(self, t_detected: float) -> None:
        t_true = t_detected - self.report_latency_s
        if self._pll_period is None:
            if len(self._iois) >= 3:
                self._pll_period = float(np.median(self._iois))
                self._pll_ref = t_true
            return
        k = round((t_true - self._pll_ref) / self._pll_period)
        if k <= 0:
            return
        predicted = self._pll_ref + k * self._pll_period
        err = t_true - predicted
        if abs(err) > 0.35 * self._pll_period:            # outlier onset - ignore
            return
        self._pll_ref += 0.12 * err                       # phase pull
        self._pll_period += 0.02 * err / k                # gentle tempo adaptation
        self._pll_period = float(np.clip(self._pll_period, 60.0 / MAX_BPM, 60.0 / MIN_BPM))
        self.bpm = 60.0 / self._pll_period
        if k > 32:                                        # keep the anchor recent
            self._pll_ref += (k - 8) * self._pll_period

    # -- internals ----------------------------------------------------------
    def _peak_pick(self, t: float, flux: float, thresh: float, bands: dict) -> list[BeatEvent]:
        # Wait one block after crossing the threshold so we can confirm a local max.
        events: list[BeatEvent] = []
        if self._pending_peak is not None:
            pt, pflux, pbands = self._pending_peak
            if flux < pflux:                      # previous block was the peak
                events.append(self._commit_onset(pt, pflux, pbands))
                self._pending_peak = None
            elif flux >= pflux:                   # still rising; move the candidate
                self._pending_peak = (t, flux, bands)
            return events

        if flux > thresh:
            if self._last_onset_t is None or (t - self._last_onset_t) >= REFRACTORY_S:
                self._pending_peak = (t, flux, bands)
        return events

    def _commit_onset(self, t: float, flux: float, bands: dict) -> BeatEvent:
        if self._last_onset_t is not None:
            ioi = t - self._last_onset_t
            if 60.0 / MAX_BPM <= ioi <= 60.0 / MIN_BPM:
                self._iois.append(ioi)
                if self._pll_period is None:
                    self.bpm = float(np.clip(60.0 / np.median(self._iois), MIN_BPM, MAX_BPM))
        self._last_onset_t = t
        self._pll_update(t)
        strength = float(np.clip(flux / self._flux_ceiling, 0.0, 1.0))
        kind = "beat" if len(self._iois) >= 4 else "onset"
        return BeatEvent(t=t, kind=kind, strength=strength, bands=bands)

    def _band_energy(self, mag: np.ndarray) -> dict[str, float]:
        power = mag.astype(np.float64) ** 2
        out: dict[str, float] = {}
        for name, (lo, hi) in _BANDS.items():
            sel = (self._freqs >= lo) & (self._freqs < hi)
            out[name] = float(power[sel].sum())
        return out
