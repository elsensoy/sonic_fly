"""Learned audio branch: *what kind* of musical event is happening?

Runs on a rolling ~2 s window at ~4 Hz - slower and lower-rate than the DSP
timing branch, and off the timing-critical path. On the RTX box the mel
front-end is DALI+CUDA and everything stays GPU-resident
(`dali_pipeline.build_mel`).

V0 has no trained checkpoint, so `_encode` is a deterministic GPU function:
interpretable spectral statistics (intensity, brightness, flux, density) plus a
fixed random-projection embedding whose drift drives a novelty / section-change
signal. The interpretable scalars are solid; `texture` and `section_change` are
threshold heuristics — rough, meant to be tuned against real music in the
visualizer or replaced outright. The `MusicalState` contract is real, so
swapping `_encode` for a trained `nn.Module` (or an open audio encoder like a
CLAP / music-tagging model) touches nothing downstream.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from host.audio.capture import AudioFrame
from host.audio.dali_pipeline import MelConfig, build_mel
from host.audio.encoders import AudioEncoder, RandomProjectionEncoder

def _unit(v: np.ndarray) -> np.ndarray:
    return v / (np.linalg.norm(v) + 1e-9)


@dataclass
class MusicalState:
    """Deliberately tiny - what the motion policy actually consumes is the
    categorical fields (`energy`, `texture`, `section_change`, `intensity`).
    `embedding` is kept for novelty / future retrieval; the controller never
    touches it.
    """

    t: float                       # window-end time (monotonic seconds)
    intensity: float               # 0..1, normalised loudness
    texture: str                   # "percussive"|"melodic"|"sparse"|"unknown" (smoothed)
    section_change: bool           # novelty spike vs recent history
    energy: str = "mid"            # "low"|"mid"|"high"|"unknown"
    density_class: str = ""        # "sparse"|"mid"|"dense"|"unknown" (CLAP only)
    mood: str = ""                 # unused by default prompts; kept for other encoders
    confidence: float = 1.0        # top1 - top2 label margin (< ~0.1 ⇒ "unknown")
    brightness: float = 0.5        # 0..1, hf / (hf+lf)
    flux: float = 0.0              # 0..1, frame-to-frame spectral change
    density: float = 0.0           # 0..1, fraction of mel bands active (DSP)
    novelty: float = 0.0           # 0..1, distance from recent history
    seq: int = 0                   # inference counter - dedupe `section_change` on this
    backend: str = ""              # "<mel>/<encoder>"
    embedding: np.ndarray | None = None
    extras: dict = field(default_factory=dict)


class AudioModel:
    def __init__(
        self,
        sample_rate: int = 16_000,
        window_seconds: float = 4.0,       # CLAP semantics want context; beat path is separate
        hop_seconds: float = 0.5,          # overlapping windows -> smooth cadence
        device: str = "auto",
        backend: str = "auto",
        encoder: AudioEncoder | None = None,
        inference_delay_s: float = 0.0,
        semantic_alpha: float = 0.4,        # EMA weight on each new label distribution
        section_w_s: float = 4.0,           # half-width of the past/present novelty blocks
        section_threshold: float = 0.10,    # tune from `semantic_trace` novelty percentiles
        section_cooldown_s: float = 5.0,
    ) -> None:
        self.sr = sample_rate
        self.window = int(window_seconds * sample_rate)
        self.hop = int(hop_seconds * sample_rate)
        self.device = device
        self.backend = backend
        self.encoder = encoder
        self.inference_delay_s = inference_delay_s   # simulate a slow model (decoupling proof)
        self.semantic_alpha = semantic_alpha
        self._sem_ema: dict[str, dict[str, float]] = {}
        self.section_w_s = section_w_s
        self.section_threshold = section_threshold
        self.section_cooldown_s = section_cooldown_s

        self._buf = np.zeros(0, dtype=np.float32)
        self._since_infer = 0
        self._t_last_frame = 0.0
        self._seq = 0
        self._loaded = False
        self._emb_ring: deque[tuple[float, np.ndarray]] = deque()   # (t, embedding), ~2·W s
        self._level_hist: deque[float] = deque(maxlen=40)
        self._last_section_t = -1e9
        self.mel = None

    # -- lifecycle -------------------------------------------------
    def load(self) -> None:
        self.mel = build_mel(MelConfig(sample_rate=self.sr), backend=self.backend, device=self.device)
        if self.encoder is None:
            self.encoder = RandomProjectionEncoder(mel=self.mel, device=self.device)
        self._loaded = True

    # -- streaming ------------------------------------------------
    def feed(self, samples: np.ndarray, t: float | None = None) -> None:
        """Append audio. Cheap - no inference. Safe to call at frame rate."""
        if t is not None:
            self._t_last_frame = t
        self._buf = np.concatenate([self._buf, samples.astype(np.float32)])
        if self._buf.size > self.window * 2:
            self._buf = self._buf[-self.window * 2:]
        self._since_infer += samples.size

    def snapshot_window(self) -> tuple[np.ndarray, float] | None:
        """The current window + its end time, if a fresh one is due; else None."""
        if self._buf.size < self.window or self._since_infer < self.hop:
            return None
        self._since_infer = 0
        return self._buf[-self.window:].copy(), self._t_last_frame

    def push(self, frame: AudioFrame) -> MusicalState | None:
        """Synchronous feed + infer. Use ThreadedAudioModel to keep GPU work
        off the caller's thread."""
        if not self._loaded:
            self.load()
        self.feed(frame.samples, frame.t_capture)
        w = self.snapshot_window()
        return None if w is None else self.infer_window(*w)

    # -- the "model" ---------------------------------------------
    def infer_window(self, wave: np.ndarray, t: float | None = None) -> MusicalState:
        if not self._loaded:
            self.load()
        return self._encode(wave, self._t_last_frame if t is None else t)

    def _encode(self, wave: np.ndarray, t: float) -> MusicalState:
        import torch

        if self.inference_delay_s:                           # simulate a slow model
            time.sleep(self.inference_delay_s)               # (runs in the worker thread)
        self._seq += 1

        # --- interpretable spectral stats (model-agnostic post-processing) ---
        logmel = self.mel.process(wave)                      # [n_mels, T] in [-top_db, 0]
        m = (logmel + MelConfig().top_db) / MelConfig().top_db   # -> ~[0, 1]
        band_raw = m.mean(dim=1)
        n = band_raw.numel()
        lf = band_raw[: n // 3].mean()
        hf = band_raw[2 * n // 3:].mean()
        level = m.mean()
        flux = (m[:, 1:] - m[:, :-1]).clamp(min=0).mean()
        gmean = torch.exp(torch.log(band_raw.clamp(min=1e-4)).mean())
        flatness = (gmean / band_raw.mean().clamp(min=1e-4)).clamp(0, 1)
        density = (band_raw > band_raw.mean()).float().mean()

        # --- embedding (+ optional zero-shot labels): whatever encoder is plugged in ---
        emb_np, labels = self.encoder.analyze(wave, self.sr)

        brightness = float((hf / (hf + lf).clamp(min=1e-6)).item())
        flux_f = float(flux.item())
        density_f = float(density.item())
        flatness_f = float(flatness.item())
        level_f = float(level.item())

        self._level_hist.append(level_f)
        intensity = _range_norm(level_f, self._level_hist)

        # section novelty: cosine distance between the mean embedding of the
        # *previous* ~W seconds and the *current* ~W seconds - a local
        # self-similarity check, not adjacent 0.5 s windows.
        self._emb_ring.append((t, emb_np))
        cutoff = t - 2.0 * self.section_w_s - 1.0
        while self._emb_ring and self._emb_ring[0][0] < cutoff:
            self._emb_ring.popleft()
        w = self.section_w_s
        past = [e for (tt, e) in self._emb_ring if t - 2 * w <= tt < t - w]
        pres = [e for (tt, e) in self._emb_ring if t - w <= tt <= t]
        novelty = 0.0
        if len(past) >= 3 and len(pres) >= 3:
            pm = _unit(np.mean(past, axis=0))
            cm = _unit(np.mean(pres, axis=0))
            novelty = float(np.clip(1.0 - float(pm @ cm), 0.0, 1.0))

        section = False
        if (novelty > self.section_threshold
                and t - self._last_section_t > self.section_cooldown_s):
            section = True
            self._last_section_t = t

        # --- categorical semantic state, temporally smoothed ---
        # Beat timing stays sharp (DSP branch); the "vibe" should not flip
        # window-to-window. EMA the label distributions, then take the argmax
        # only when the top1-top2 margin is decisive - else "unknown".
        energy_dsp = "low" if intensity < 0.33 else ("high" if intensity > 0.66 else "mid")
        if labels:                                          # trained encoder: real zero-shot dists
            raw = labels
        else:                                               # dummy: heuristic -> one-hot-ish
            tex_h, tex_c = _classify_texture(flux_f, flatness_f, density_f,
                                             intensity, brightness, level_f)
            raw = {"texture": _spread(tex_h, tex_c, TEXTURES),
                   "energy": _spread(energy_dsp, 0.6, ENERGIES)}
        sm = self._smooth_labels(raw)
        texture, conf = _pick(sm.get("texture"))
        energy, _ = _pick(sm.get("energy"), default=energy_dsp)
        density_cls, _ = _pick(sm.get("density"))
        mood, _ = _pick(sm.get("mood"))

        return MusicalState(
            t=t,
            seq=self._seq,
            intensity=intensity,
            energy=energy,
            density_class=density_cls,
            mood=mood,
            texture=texture,
            section_change=section,
            confidence=conf,
            brightness=brightness,
            flux=min(1.0, flux_f * 4.0),
            density=density_f,
            novelty=novelty,
            backend=f"{self.mel.backend}/{self.encoder.name}",
            embedding=emb_np,
            extras={"flatness": flatness_f, "level": level_f,
                    "labels_raw": raw, "labels_smoothed": sm},
        )

    def _smooth_labels(self, raw: dict[str, dict[str, float]]) -> dict[str, dict[str, float]]:
        a = self.semantic_alpha
        out: dict[str, dict[str, float]] = {}
        for cat, scores in raw.items():
            prev = self._sem_ema.get(cat)
            if prev is None:
                sm = dict(scores)
            else:
                keys = set(scores) | set(prev)
                sm = {k: a * scores.get(k, 0.0) + (1.0 - a) * prev.get(k, 0.0) for k in keys}
            self._sem_ema[cat] = sm
            out[cat] = sm
        return out


ENERGIES = ("low", "mid", "high")
MIN_MARGIN = 0.10          # top1 - top2 below this -> "unknown"


def _pick(scores: dict[str, float] | None, default: str = "",
          min_margin: float = MIN_MARGIN) -> tuple[str, float]:
    """(label, margin). `label` is the argmax unless the top1-top2 margin is
    below `min_margin`, in which case it's "unknown" - the model can't
    separate the classes on this window. `margin` doubles as confidence."""
    if not scores:
        return default, 0.0
    ranked = sorted(scores.values(), reverse=True)
    margin = ranked[0] - (ranked[1] if len(ranked) > 1 else 0.0)
    if margin < min_margin:
        return "unknown", margin
    return max(scores, key=scores.get), float(margin)


def _spread(label: str, p: float, classes: tuple[str, ...]) -> dict[str, float]:
    """A heuristic pick -> a soft distribution, for uniform smoothing."""
    rest = (1.0 - p) / max(1, len(classes) - 1)
    return {c: (p if c == label else rest) for c in classes}


def _range_norm(x: float, hist: deque) -> float:
    if len(hist) < 4:
        return 0.5
    arr = np.fromiter(hist, dtype=np.float64)
    lo, hi = np.percentile(arr, (10, 90))
    return float(np.clip((x - lo) / (hi - lo), 0.0, 1.0)) if hi > lo else 0.5


TEXTURES = ("percussive", "melodic", "sparse")


def _classify_texture(flux: float, flatness: float, density: float, intensity: float,
                      brightness: float, level: float) -> tuple[str, float]:
    """-> (texture, confidence) in the same 3-class vocabulary CLAP's texture
    prompts use. Heuristic distance-from-threshold confidence for the dummy path."""
    if level < 0.14 or intensity < 0.18:
        return "sparse", _conf(0.18 - min(level, intensity), 0.12)
    if flux > 0.09 and (flatness > 0.3 or brightness > 0.55):
        return "percussive", _conf(flux - 0.09, 0.08)
    return "melodic", 0.5


def _conf(margin: float, scale: float) -> float:
    return float(min(1.0, 0.5 + max(0.0, margin) / scale * 0.5))


class ThreadedAudioModel:
    """`AudioModel` on a worker thread with its own CUDA stream.

    `push(frame)` is non-blocking - it only appends samples. The worker pulls a
    fresh window whenever it's free, runs inference on a dedicated
    `torch.cuda.Stream` (so a slow forward pass never serialises behind, or
    stalls, the timing path), and publishes the result. `latest()` returns the
    most recent `MusicalState`; it may lag real time by up to one inference,
    which is fine for the semantic branch.
    """

    def __init__(self, **audio_model_kwargs) -> None:
        self.model = AudioModel(**audio_model_kwargs)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._wake = threading.Event()
        self._thread: threading.Thread | None = None
        self._latest: MusicalState | None = None
        self.stream = None
        self.infer_ms = 0.0        # wall time of the last inference, for telemetry
        self.infer_log: deque[float] = deque(maxlen=512)   # recent inference wall times
        self.backend = ""

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="audio-model", daemon=True)
        self._thread.start()

    def push(self, frame: AudioFrame) -> None:
        with self._lock:
            self.model.feed(frame.samples, frame.t_capture)
        self._wake.set()

    def latest(self) -> MusicalState | None:
        with self._lock:
            return self._latest

    def stop(self) -> None:
        self._stop.set()
        self._wake.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def _run(self) -> None:
        import torch

        self.model.load()
        self.backend = self.model.mel.backend
        if torch.cuda.is_available():
            self.stream = torch.cuda.Stream()

        while not self._stop.is_set():
            self._wake.wait(timeout=0.5)
            self._wake.clear()
            if self._stop.is_set():
                break
            with self._lock:
                w = self.model.snapshot_window()
            if w is None:
                continue
            wave, t = w
            t0 = time.monotonic()
            if self.stream is not None:
                with torch.cuda.stream(self.stream):
                    state = self.model.infer_window(wave, t)
                self.stream.synchronize()
            else:
                state = self.model.infer_window(wave, t)
            self.infer_ms = (time.monotonic() - t0) * 1000.0
            self.infer_log.append(self.infer_ms)
            with self._lock:
                self._latest = state
