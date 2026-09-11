"""Audio frame sources - one place `response_test.py` and the visualizer share.

`open_source(spec, ...)` returns `(frame_iterator, sample_rate, description)` for:

  * ``"mic"``          - live capture via AudioCapture (sounddevice or arecord)
  * ``"synth"``        - a synthetic track (see `profile`), no hardware
  * ``"<path>.wav"``   - a recorded clip, played back in real time
"""

from __future__ import annotations

import time
import wave
from typing import Iterator

import numpy as np

from host.audio.capture import AudioCapture, AudioFrame


# ---- synthetic ---------------------------------------------------------


def synth_signal(sr: int, seconds: float, bpm: float = 120.0, profile: str = "arc") -> np.ndarray:
    """profile 'click' = steady click track; 'arc' = quiet -> groove -> peak -> out."""
    n = int(seconds * sr)
    rng = np.random.default_rng(0)
    sig = (rng.standard_normal(n).astype(np.float32) * 0.003)
    period = 60.0 / bpm

    click = _env_tone(sr, 1800, 0.02)
    kick = _env_tone(sr, 70, 0.06)
    hat = (rng.standard_normal(int(0.012 * sr)).astype(np.float32) * np.hanning(int(0.012 * sr)))

    def place(t: float, w: np.ndarray, g: float) -> None:
        i = int(t * sr)
        if 0 <= i < n:
            end = min(n, i + w.size)
            sig[i:end] += w[: end - i] * g

    t = period
    while t < seconds - 0.1:
        if profile == "click":
            place(t, click, 0.5)
        else:  # arc
            frac = t / seconds
            if frac < 0.25:                         # quiet intro
                place(t, click, 0.25)
            elif frac < 0.55:                       # groove
                place(t, kick, 0.7); place(t, click, 0.4)
            elif frac < 0.85:                       # peak
                place(t, kick, 1.0); place(t, click, 0.9); place(t + period / 2, hat, 0.6)
            # last 15%: let it fall out
        t += period
    return np.clip(sig, -1.0, 1.0)


def _env_tone(sr: int, hz: float, dur_s: float) -> np.ndarray:
    k = int(dur_s * sr)
    return (np.sin(2 * np.pi * hz * np.arange(k) / sr) * np.hanning(k)).astype(np.float32)


def frames_from_signal(sig: np.ndarray, sr: int, block: int, realtime: bool = True) -> Iterator[AudioFrame]:
    t0 = time.monotonic()
    for i in range(0, len(sig) - block, block):
        yield AudioFrame(sig[i:i + block].copy(), t0 + i / sr)
        if realtime:
            target = t0 + (i + block) / sr
            slack = target - time.monotonic()
            if slack > 0:
                time.sleep(slack)


# ---- wav -------------------------------------------------------------


def wav_signal(path: str) -> tuple[np.ndarray, int]:
    with wave.open(path, "rb") as wf:
        sr, ch = wf.getframerate(), wf.getnchannels()
        raw = wf.readframes(wf.getnframes())
    a = np.frombuffer(raw, dtype="<i2").astype(np.float32) / 32768.0
    if ch > 1:
        a = a.reshape(-1, ch).mean(axis=1)
    return a, sr


# ---- unified entry -------------------------------------------------


def open_source(
    spec: str,
    *,
    rate: int,
    block: int,
    backend: str = "auto",
    device: int | str | None = None,
    bpm: float = 120.0,
    seconds: float = 60.0,
    profile: str = "arc",
    realtime: bool = True,
) -> tuple[Iterator[AudioFrame], int, str]:
    if spec == "mic":
        cap = AudioCapture(rate, block, device, backend)
        return cap.frames(), cap.sample_rate, f"mic via {cap.backend}"
    if spec == "synth":
        sig = synth_signal(rate, seconds, bpm, profile)
        return frames_from_signal(sig, rate, block, realtime), rate, f"synth/{profile} @ {bpm:.0f} BPM"
    sig, sr = wav_signal(spec)
    return frames_from_signal(sig, sr, block, realtime), sr, f"wav {spec} @ {sr} Hz"
