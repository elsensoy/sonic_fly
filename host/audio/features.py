"""Per-frame audio feature vector - the input contract for the translation engine.

`FeatureExtractor` wraps `BeatDetector` (timing) and adds a smoothed, adaptively
normalised loudness envelope per band, an energy trend, and a high-frequency
transient flag. It emits one `AudioFeatures` per `AudioFrame`, so everything
downstream (`MusicStateEstimator`, `MotionPolicy`) depends only on this
dataclass, never on the DSP internals.

Design notes
------------
* `energy`/`bass`/`mid`/`high` are a *smoothed envelope* (~0.4 s), not the raw
  per-frame power, so they represent "how loud is this passage" and do not drop
  to zero between beats.
* Normalisation is min-max against rolling percentiles of a ~6 s history
  (20th = floor, 95th = reference), so the numbers are stable and self-
  calibrating once the track has been "heard" for a second or two (`warmup`
  is True until then).
* `beat`/`onset_strength` carry the spiky per-event signal from `BeatDetector`.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np

from host.audio.beat_detector import BeatDetector
from host.audio.capture import AudioFrame

_BANDS = {"bass": (20, 250), "mid": (250, 2000), "high": (2000, 8000)}
_TREND_WINDOW_S = 2.0
_HISTORY_S = 6.0
_WARMUP_FRAMES = 40


@dataclass
class AudioFeatures:
    t: float                 # monotonic seconds (frame capture time)
    beat: bool               # a beat/onset fired in this frame
    onset_strength: float    # 0..1, strongest event this frame (0 if none)
    kind: str                # "beat" | "onset" | ""
    bpm: float | None        # current tempo estimate
    energy: float            # 0..1 normalised broadband loudness envelope
    bass: float              # 0..1
    mid: float               # 0..1
    high: float              # 0..1
    building: float          # -1..1 energy trend over ~2 s (rising positive)
    transient: bool          # sharp high-frequency transient this frame
    warmup: bool = False     # True until the normaliser has calibrated
    raw: dict[str, float] = field(default_factory=dict)  # unnormalised, for debugging


class FeatureExtractor:
    def __init__(self, sample_rate: int, block_size: int) -> None:
        self.sample_rate = sample_rate
        self.block_size = block_size
        self.detector = BeatDetector(sample_rate, block_size)

        self._dt = block_size / sample_rate
        self._env_alpha = 1.0 - np.exp(-self._dt / 0.4)     # ~0.4 s envelope
        self._window = np.hanning(block_size).astype(np.float32)
        self._freqs = np.fft.rfftfreq(block_size, 1.0 / sample_rate)
        self._band_sel = {
            name: (self._freqs >= lo) & (self._freqs < hi)
            for name, (lo, hi) in _BANDS.items()
        }
        self._keys = ("total", "bass", "mid", "high")
        self._env = {k: 0.0 for k in self._keys}
        hist_len = max(16, int(_HISTORY_S / self._dt))
        self._hist: dict[str, deque[float]] = {k: deque(maxlen=hist_len) for k in self._keys}
        self._high_frac_avg = 0.0
        self._trend: deque[tuple[float, float]] = deque()
        self._n = 0

    def push(self, frame: AudioFrame) -> AudioFeatures:
        self._n += 1
        events = self.detector.push(frame)
        beat = bool(events)
        strength = max((e.strength for e in events), default=0.0)
        kind = max(events, key=lambda e: e.strength).kind if events else ""

        mag = np.abs(np.fft.rfft(frame.samples * self._window))
        power = mag.astype(np.float64) ** 2
        raw = {name: float(power[sel].sum()) for name, sel in self._band_sel.items()}
        raw["total"] = float(power.sum())

        norm = {}
        a = self._env_alpha
        for k in self._keys:
            self._env[k] += (raw[k] - self._env[k]) * a
            self._hist[k].append(self._env[k])
            if len(self._hist[k]) >= 8:
                arr = np.fromiter(self._hist[k], dtype=np.float64)
                lo, hi = np.percentile(arr, (20.0, 95.0))
                span = hi - lo
                norm[k] = float(np.clip((self._env[k] - lo) / span, 0.0, 1.0)) if span > 1e-15 else 0.0
            else:
                norm[k] = 0.0

        # transient: highs briefly *dominate* the spectrum and jump vs recent
        high_frac = raw["high"] / (raw["total"] + 1e-12)
        transient = (
            high_frac > 0.45
            and high_frac > 2.5 * self._high_frac_avg
            and norm["high"] > 0.35
        )
        self._high_frac_avg += (high_frac - self._high_frac_avg) * 0.1

        building = self._update_trend(frame.t_capture, norm["total"])

        return AudioFeatures(
            t=frame.t_capture,
            beat=beat,
            onset_strength=strength,
            kind=kind,
            bpm=self.detector.bpm,
            energy=norm["total"],
            bass=norm["bass"],
            mid=norm["mid"],
            high=norm["high"],
            building=building,
            transient=transient,
            warmup=self._n < _WARMUP_FRAMES,
            raw=raw,
        )

    def _update_trend(self, t: float, energy: float) -> float:
        self._trend.append((t, energy))
        while self._trend and t - self._trend[0][0] > _TREND_WINDOW_S:
            self._trend.popleft()
        if len(self._trend) < 4:
            return 0.0
        ts = np.array([p[0] for p in self._trend])
        es = np.array([p[1] for p in self._trend])
        ts -= ts[0]
        denom = float(((ts - ts.mean()) ** 2).sum())
        if denom == 0:
            return 0.0
        slope = float(((ts - ts.mean()) * (es - es.mean())).sum() / denom)  # energy/s
        return float(np.clip(slope * _TREND_WINDOW_S, -1.0, 1.0))
