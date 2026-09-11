"""Mel-spectrogram front-end for the learned audio branch.

Two backends, same output: a `torch` tensor ``[n_mels, frames]`` on the target
device (kept on the GPU so it feeds the model with no CPU bounce).

  * `DaliMel`  - NVIDIA DALI GPU pipeline (spectrogram -> mel -> dB). Preferred
                 on the RTX box. Needs `nvidia-dali-cuda120` + a cuFFT wheel
                 (`nvidia-cufft-cu12`); this module preloads the wheel libs.
  * `TorchMel` - `torch.stft` + a mel filterbank matrix. Runs on CUDA or CPU,
                 has no extra deps, and is what the tests use.

`build_mel(...)` picks one.
"""

from __future__ import annotations

import ctypes
import glob
import os
import site
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class MelConfig:
    sample_rate: int = 16_000
    n_fft: int = 1024
    hop: int = 512
    n_mels: int = 64
    fmin: float = 20.0
    fmax: float = 8_000.0
    top_db: float = 80.0


# ---- CUDA wheel libs (DALI needs cuFFT etc. that torch bundles privately) ----

_PRELOADED = False


def _preload_cuda_libs() -> None:
    global _PRELOADED
    if _PRELOADED:
        return
    roots = list(site.getsitepackages())
    if hasattr(site, "getusersitepackages"):
        roots.append(site.getusersitepackages())
    for root in roots:
        for lib in glob.glob(os.path.join(root, "nvidia", "*", "lib", "*.so*")):
            try:
                ctypes.CDLL(lib, mode=ctypes.RTLD_GLOBAL)
            except OSError:
                pass
    _PRELOADED = True


# ---- torch backend -------------------------------------------------


def _mel_matrix(cfg: MelConfig) -> np.ndarray:
    """Slaney-style triangular mel filterbank, [n_mels, n_fft//2+1]."""
    def hz_to_mel(f):
        return 2595.0 * np.log10(1.0 + f / 700.0)

    def mel_to_hz(m):
        return 700.0 * (10.0 ** (m / 2595.0) - 1.0)

    n_bins = cfg.n_fft // 2 + 1
    fft_freqs = np.linspace(0, cfg.sample_rate / 2, n_bins)
    mel_pts = np.linspace(hz_to_mel(cfg.fmin), hz_to_mel(cfg.fmax), cfg.n_mels + 2)
    hz_pts = mel_to_hz(mel_pts)
    fb = np.zeros((cfg.n_mels, n_bins), dtype=np.float32)
    for m in range(cfg.n_mels):
        lo, ctr, hi = hz_pts[m], hz_pts[m + 1], hz_pts[m + 2]
        left = (fft_freqs - lo) / max(ctr - lo, 1e-9)
        right = (hi - fft_freqs) / max(hi - ctr, 1e-9)
        fb[m] = np.clip(np.minimum(left, right), 0.0, None)
    return fb


def _finalize(mel_power, top_db: float):
    """Linear mel power -> per-window-max-referenced log-mel in [-top_db, 0]."""
    import torch

    ref = torch.clamp(mel_power.amax(), min=1e-10)
    db = 10.0 * torch.log10(torch.clamp(mel_power, min=1e-10) / ref)
    return torch.clamp(db, min=-top_db)


class TorchMel:
    backend = "torch"

    def __init__(self, cfg: MelConfig, device: str = "cpu") -> None:
        import torch

        self.cfg = cfg
        self.device = device
        self._t = torch
        self._window = torch.hann_window(cfg.n_fft, device=device)
        self._fb = torch.from_numpy(_mel_matrix(cfg)).to(device)

    def process(self, wave: np.ndarray):
        torch = self._t
        x = torch.as_tensor(np.ascontiguousarray(wave, dtype=np.float32), device=self.device)
        spec = torch.stft(x, n_fft=self.cfg.n_fft, hop_length=self.cfg.hop,
                          window=self._window, return_complex=True, center=True)
        mel = self._fb @ (spec.abs() ** 2)           # [n_mels, frames]
        return _finalize(mel, self.cfg.top_db)


# ---- DALI backend -------------------------------------------------


class DaliMel:
    backend = "dali"

    def __init__(self, cfg: MelConfig, device_id: int = 0) -> None:
        _preload_cuda_libs()
        import torch
        from nvidia.dali import fn, pipeline_def, types

        self.cfg = cfg
        self._t = torch

        @pipeline_def(batch_size=1, num_threads=2, device_id=device_id,
                      prefetch_queue_depth=1, exec_pipelined=False, exec_async=False)
        def _pipe():
            wav = fn.external_source(name="wave", dtype=types.FLOAT, ndim=1)
            spec = fn.spectrogram(wav.gpu(), nfft=cfg.n_fft, power=2,
                                  window_length=cfg.n_fft, window_step=cfg.hop)
            # linear mel power; HTK formula + unnormalised triangles to match
            # _mel_matrix(). The log + max-normalisation is done in torch so the
            # two backends produce near-identical log-mel.
            return fn.mel_filter_bank(spec, sample_rate=cfg.sample_rate, nfilter=cfg.n_mels,
                                      freq_low=cfg.fmin, freq_high=cfg.fmax,
                                      mel_formula="htk", normalize=False)

        self._pipe = _pipe()
        self._pipe.build()

    def process(self, wave: np.ndarray):
        import nvidia.dali.plugin.pytorch as dpt

        self._pipe.feed_input("wave", [np.ascontiguousarray(wave, dtype=np.float32)])
        (out,) = self._pipe.run()
        g = out.as_tensor()                          # [1, n_mels, frames] on GPU
        t = self._t.empty(tuple(g.shape()), dtype=self._t.float32, device="cuda")
        dpt.feed_ndarray(g, t)
        return _finalize(t.squeeze(0), self.cfg.top_db)   # -> [n_mels, frames]


# ---- selection ---------------------------------------------------


def build_mel(cfg: MelConfig | None = None, *, backend: str = "auto", device: str = "auto"):
    cfg = cfg or MelConfig()
    try:
        import torch
        has_cuda = torch.cuda.is_available()
    except Exception:
        has_cuda = False

    if device == "auto":
        device = "cuda" if has_cuda else "cpu"

    if backend in ("auto", "dali") and device == "cuda":
        try:
            return DaliMel(cfg)
        except Exception as e:
            if backend == "dali":
                raise
            print(f"[dali_pipeline] DALI unavailable ({e.__class__.__name__}); using torch")

    return TorchMel(cfg, device=device)
