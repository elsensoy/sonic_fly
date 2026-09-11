"""Pluggable audio encoders.

`AudioModel` owns one `AudioEncoder` and never cares which - swapping the
dummy for a trained model touches nothing in the motion pipeline.

    AudioEncoder
      ├── RandomProjectionEncoder   ← today (deterministic, not semantic)
      ├── ClapEncoder               ← next
      └── ...

`encode()` takes raw mono audio and returns an L2-normalised embedding.
`labels()` is optional zero-shot category scoring - a trained model can fill
it in; the dummy returns None and `AudioModel` falls back to its heuristics.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np

from host.audio.dali_pipeline import MelConfig, build_mel

_T_REF = 60          # log-mel window resampled to this many frames


class AudioEncoder(ABC):
    name: str = "abstract"
    dim: int = 0

    @abstractmethod
    def encode(self, wave: np.ndarray, sr: int) -> np.ndarray:
        """Raw mono audio -> float32 embedding of shape (dim,), L2-normalised."""

    def labels(self, wave: np.ndarray, sr: int) -> dict[str, dict[str, float]] | None:
        """Optional zero-shot {category: {label: prob}}. None if unsupported."""
        return None

    def analyze(self, wave: np.ndarray, sr: int) -> tuple[np.ndarray, dict | None]:
        """Embedding + labels in one call. Override to share a forward pass."""
        return self.encode(wave, sr), self.labels(wave, sr)


class RandomProjectionEncoder(AudioEncoder):
    """V0 stand-in: a fixed random projection of a z-scored log-mel window.

    Not semantic, but a stable feature space whose drift tracks real spectral
    change - enough to drive novelty / section detection until a trained model
    lands. Obeys the same interface, so the swap is a one-liner.
    """

    name = "randproj"

    def __init__(self, dim: int = 64, seed: int = 0, mel=None, device: str = "auto") -> None:
        self.dim = dim
        self.seed = seed
        self._mel = mel                       # share AudioModel's mel pipeline if given
        self._device_pref = device
        self._proj = None

    def _ensure(self, sr: int):
        import torch

        if self._mel is None:
            self._mel = build_mel(MelConfig(sample_rate=sr), device=self._device_pref)
        if self._proj is None:
            n_mels = MelConfig().n_mels
            dev = "cuda" if self._mel.backend == "dali" else getattr(self._mel, "device", "cpu")
            g = torch.Generator(device="cpu").manual_seed(self.seed)
            self._proj = (torch.randn(self.dim, n_mels * _T_REF, generator=g)
                          .to(dev) / np.sqrt(n_mels * _T_REF))
        return torch

    def encode(self, wave: np.ndarray, sr: int) -> np.ndarray:
        torch = self._ensure(sr)
        import torch.nn.functional as F

        logmel = self._mel.process(wave)                       # [n_mels, T], <= 0 dB
        m = (logmel + MelConfig().top_db) / MelConfig().top_db
        mz = (m - m.mean()) / (m.std() + 1e-6)
        fixed = F.interpolate(mz[None, None], size=(mz.shape[0], _T_REF),
                              mode="bilinear", align_corners=False).reshape(-1)
        emb = torch.tanh(self._proj @ fixed)
        emb = emb / emb.norm().clamp(min=1e-6)
        return emb.detach().cpu().numpy().astype(np.float32)


# ------------------------------------------------------------------------


# Concrete, contrastive prompts - 3 classes per axis. Chosen so the *movement*
# differs and so CLAP actually separates them (abstract words like "bright" /
# "dark" barely move the cosine on short windows). Tune against real tracks.
_CLAP_PROMPTS: dict[str, dict[str, str]] = {
    "texture": {
        "percussive": "music dominated by drums and percussion",
        "melodic":    "music dominated by sustained melodic instruments",
        "sparse":     "music with sparse instrumentation and long pauses",
    },
    "energy": {
        "low":  "quiet soft low-energy music",
        "mid":  "moderately energetic music",
        "high": "loud intense high-energy music",
    },
    "density": {
        "sparse": "sparse music with few simultaneous sounds",
        "mid":    "moderately dense music",
        "dense":  "dense layered music with many simultaneous sounds",
    },
}


class ClapEncoder(AudioEncoder):
    """LAION CLAP audio encoder via HuggingFace transformers.

    512-d contrastive embedding + zero-shot category scores from text prompts.
    CLAP wants 48 kHz and brings its own mel front-end, so it ignores the
    shared pipeline - `encode(wave, sr)` resamples and hands off to the
    processor. ~30-50 ms/window on the RTX 5060; the threaded architecture
    absorbs that (see docs/audio_model.md).

    Default checkpoint is `clap-htsat-unfused` - `laion/larger_clap_music` is
    broken under transformers >= 5.16 (text embeddings collapse, `logit_scale`
    fails to load), which forces every zero-shot softmax to ~uniform.

    Pinned to an exact commit (`_CLAP_REVISION`) so the frozen policy sees the
    same weights every run - HF `main` can move under you otherwise.
    """

    name = "clap"
    dim = 512
    _CLAP_SR = 48_000
    _CLAP_REVISION = "8fa0f1c6d0433df6e97c127f64b2a1d6c0dcda8a"   # laion/clap-htsat-unfused @ main, 2024

    def __init__(self, model_id: str = "laion/clap-htsat-unfused", device: str = "auto",
                 prompts: dict[str, dict[str, str]] | None = None,
                 revision: str | None = None) -> None:
        self.model_id = model_id
        self.revision = revision or (self._CLAP_REVISION
                                     if model_id == "laion/clap-htsat-unfused" else None)
        self._device_pref = device
        self.prompts = prompts or _CLAP_PROMPTS
        self.model = None
        self.proc = None
        self._logit_scale = 1.0
        self._text_embs: dict[str, "object"] = {}   # category -> [K, dim] tensor
        self._labels: dict[str, list[str]] = {c: list(p) for c, p in self.prompts.items()}

    def _ensure(self):
        if self.model is not None:
            return
        import torch
        from transformers import ClapModel, ClapProcessor

        dev = self._device_pref
        if dev == "auto":
            dev = "cuda" if torch.cuda.is_available() else "cpu"
        self._dev = dev
        self.model = ClapModel.from_pretrained(self.model_id, revision=self.revision).to(dev).eval()
        self.proc = ClapProcessor.from_pretrained(self.model_id, revision=self.revision)
        self._logit_scale = float(self.model.logit_scale_a.detach().exp())
        with torch.no_grad():
            for cat, mapping in self.prompts.items():
                ti = self.proc(text=list(mapping.values()), return_tensors="pt",
                               padding=True).to(dev)
                te = self.model.get_text_features(**ti).pooler_output
                emb = torch.nn.functional.normalize(te, dim=-1)
                if len(emb) > 1 and float((emb @ emb.T).triu(1).abs().max()) > 0.98:
                    raise RuntimeError(
                        f"{self.model_id}: text prompts collapsed to one embedding "
                        "- checkpoint is broken under this transformers version")
                self._text_embs[cat] = emb

    def _audio_embed(self, wave: np.ndarray, sr: int):
        import torch
        import torchaudio

        self._ensure()
        x = torch.as_tensor(np.ascontiguousarray(wave, dtype=np.float32), device=self._dev)
        if sr != self._CLAP_SR:
            x = torchaudio.functional.resample(x, sr, self._CLAP_SR)
        inp = self.proc(audio=x.detach().cpu().numpy(), sampling_rate=self._CLAP_SR,
                        return_tensors="pt").to(self._dev)
        with torch.no_grad():
            emb = self.model.get_audio_features(**inp).pooler_output
        return torch.nn.functional.normalize(emb, dim=-1)[0]     # [dim], on device

    def encode(self, wave: np.ndarray, sr: int) -> np.ndarray:
        return self._audio_embed(wave, sr).detach().cpu().numpy().astype(np.float32)

    def analyze(self, wave: np.ndarray, sr: int) -> tuple[np.ndarray, dict]:
        e = self._audio_embed(wave, sr)
        labels: dict[str, dict[str, float]] = {}
        for cat, temb in self._text_embs.items():
            probs = (self._logit_scale * (e @ temb.T)).softmax(-1)   # CLAP's own temperature
            labels[cat] = {name: float(probs[i]) for i, name in enumerate(self._labels[cat])}
        return e.detach().cpu().numpy().astype(np.float32), labels

    def labels(self, wave: np.ndarray, sr: int) -> dict[str, dict[str, float]]:
        return self.analyze(wave, sr)[1]
