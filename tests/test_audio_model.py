"""GPU/DALI audio-model branch: mel front-end, streaming encoder, policy hook."""

import numpy as np
import pytest

import time

from host.audio.audio_model import (
    AudioModel,
    MusicalState,
    ThreadedAudioModel,
    _classify_texture,
)
from host.audio.capture import AudioFrame
from host.audio.dali_pipeline import MelConfig, TorchMel, build_mel
from host.audio.sources import synth_signal

torch = pytest.importorskip("torch")
_HAS_CUDA = torch.cuda.is_available()


def _wave(seconds=2.0, sr=16000):
    t = np.arange(int(seconds * sr)) / sr
    w = 0.05 * np.random.default_rng(0).standard_normal(t.size)
    w += 0.3 * np.sin(2 * np.pi * 200 * t) + 0.15 * np.sin(2 * np.pi * 1500 * t)
    return w.astype(np.float32)


# ---- mel front-end ------------------------------------------------


def test_torchmel_shape_and_range():
    cfg = MelConfig()
    m = TorchMel(cfg, device="cpu").process(_wave())
    assert m.shape[0] == cfg.n_mels
    assert float(m.max()) <= 0.001 and float(m.min()) >= -cfg.top_db - 0.001


@pytest.mark.skipif(not _HAS_CUDA, reason="needs CUDA")
def test_dali_matches_torch():
    from host.audio.dali_pipeline import DaliMel

    w = _wave(3.0)
    a = TorchMel(MelConfig(), device="cpu").process(w).numpy()
    try:
        c = DaliMel(MelConfig()).process(w).cpu().numpy()
    except Exception as e:                       # DALI present but cuFFT wheel missing
        pytest.skip(f"DALI unavailable: {e}")
    assert a.shape == c.shape
    assert np.corrcoef(a.ravel(), c.ravel())[0, 1] > 0.98
    assert np.abs(a - c).mean() < 1.0


def test_build_mel_falls_back_to_torch_on_cpu():
    mel = build_mel(backend="auto", device="cpu")
    assert mel.backend == "torch"
    assert mel.process(_wave()).shape[0] == MelConfig().n_mels


# ---- streaming encoder -----------------------------------------


def test_audiomodel_streaming_contract():
    sr, block = 16000, 512
    m = AudioModel(sample_rate=sr, window_seconds=2.0, hop_seconds=0.5,
                   device="cpu", backend="torch")
    sig = synth_signal(sr, 8, bpm=120, profile="arc")
    states = []
    for i in range(0, len(sig) - block, block):
        s = m.push(AudioFrame(sig[i:i + block], i / sr))
        if s is not None:
            states.append(s)
    # nothing until the 2 s window is full, then ~one per 0.5 s hop
    assert 6 <= len(states) <= 14
    s = states[-1]
    assert isinstance(s, MusicalState)
    assert s.embedding.shape == (64,) and abs(np.linalg.norm(s.embedding) - 1.0) < 1e-3
    assert 0.0 <= s.intensity <= 1.0 and 0.0 <= s.brightness <= 1.0
    assert s.texture in ("percussive", "melodic", "sparse", "unknown")
    assert s.energy in ("low", "mid", "high", "unknown")
    assert s.backend == "torch/randproj"


def test_audiomodel_flags_a_hard_section_cut():
    sr, block = 16000, 512
    t = np.arange(sr * 24) / sr
    sig = (0.02 * np.random.default_rng(1).standard_normal(sr * 24)).astype(np.float32)
    sig[: sr * 12] += (0.25 * np.sin(2 * np.pi * 180 * t[: sr * 12])).astype(np.float32)   # low pad
    sig[sr * 12:] += (0.25 * np.sin(2 * np.pi * 2600 * t[sr * 12:])).astype(np.float32)    # bright tone
    m = AudioModel(sample_rate=sr, device="cpu", backend="torch",
                   section_threshold=0.4, section_cooldown_s=5.0)
    cuts = []
    for i in range(0, len(sig) - block, block):
        s = m.push(AudioFrame(sig[i:i + block], i / sr))
        if s and s.section_change:
            cuts.append(s.t)
    assert any(10.0 < c < 18.0 for c in cuts)      # detected the 12 s cut


# ---- threaded / CUDA-stream separation ------------------------


def test_threaded_model_push_is_nonblocking_and_publishes():
    sr, block = 16000, 512
    m = ThreadedAudioModel(sample_rate=sr, device="cpu", backend="torch",
                           window_seconds=1.5, hop_seconds=0.4)
    m.start()
    time.sleep(0.2)
    sig = synth_signal(sr, 6, bpm=120, profile="arc")
    worst_us = 0.0
    for i in range(0, len(sig) - block, block):
        t0 = time.perf_counter()
        m.push(AudioFrame(sig[i:i + block], i / sr))
        worst_us = max(worst_us, (time.perf_counter() - t0) * 1e6)
        time.sleep(0.001)
    deadline = time.monotonic() + 2.0
    while m.latest() is None and time.monotonic() < deadline:
        time.sleep(0.02)
    try:
        assert worst_us < 3000                       # push() never blocks on inference
        s = m.latest()
        assert isinstance(s, MusicalState) and s.seq >= 1
        s2 = m.latest()
        assert s2.seq == s.seq or s2.seq > s.seq      # monotonic, dedupe-able
    finally:
        m.stop()


# ---- encoder labels flow through to MusicalState --------------


def test_zero_shot_labels_override_heuristics():
    from host.audio.encoders import AudioEncoder

    class StubEncoder(AudioEncoder):
        name = "stub"
        dim = 8

        def encode(self, wave, sr):
            return np.ones(8, dtype=np.float32) / np.sqrt(8)

        def analyze(self, wave, sr):
            return self.encode(wave, sr), {
                "texture": {"percussive": 0.7, "melodic": 0.2, "sparse": 0.1},
                "energy": {"low": 0.1, "mid": 0.2, "high": 0.7},
                "mood": {"dark": 0.8, "bright": 0.2},
            }

    sr, block = 16000, 512
    m = AudioModel(sample_rate=sr, device="cpu", backend="torch", encoder=StubEncoder())
    sig = synth_signal(sr, 6, bpm=120, profile="arc")
    s = None
    for i in range(0, len(sig) - block, block):
        s = m.push(AudioFrame(sig[i:i + block], i / sr)) or s
    assert s.texture == "percussive" and s.energy == "high" and s.mood == "dark"
    assert abs(s.confidence - 0.5) < 0.08          # confidence is now top1-top2 margin
    assert s.backend == "torch/stub"


# ---- classifier + policy hook --------------------------------


def test_classify_texture_extremes():
    tex, conf = _classify_texture(0.02, 0.9, 0.3, 0.05, 0.5, 0.05)
    assert tex == "sparse" and 0.0 <= conf <= 1.0
    assert _classify_texture(0.20, 0.5, 0.5, 0.7, 0.7, 0.5)[0] == "percussive"


def test_pick_margin_and_unknown():
    from host.audio.audio_model import _pick
    label, conf = _pick({"a": 0.7, "b": 0.15, "c": 0.15})
    assert label == "a" and abs(conf - 0.55) < 1e-6
    label, conf = _pick({"a": 0.36, "b": 0.34, "c": 0.30})   # margin 0.02 < 0.10
    assert label == "unknown"
    assert _pick(None, default="mid")[0] == "mid"


def test_policy_reacts_to_section_change():
    from host.control.motion_policy import MotionPolicy
    from host.control.music_state import MusicState
    from tests.test_translation import feat

    p = MotionPolicy()
    v0 = p._variant
    ms = MusicalState(t=1.0, embedding=np.zeros(64), intensity=0.7,
                      texture="melodic", section_change=True, seq=7)
    cmd = p.decide(feat(1.0, beat=True, energy=0.6), MusicState.RHYTHMIC, musical=ms)
    assert p._variant != v0                          # choreography flipped
    assert cmd is not None and cmd.primitive == "RISE"   # section accent

    # the *same* state, polled again, must not re-flip the choreography
    v1 = p._variant
    p.decide(feat(1.6, beat=True, energy=0.6), MusicState.RHYTHMIC, musical=ms)
    assert p._variant == v1


def test_policy_without_musical_is_unchanged():
    from host.control.motion_policy import MotionPolicy
    from host.control.music_state import MusicState
    from tests.test_translation import feat

    a = MotionPolicy().decide(feat(0.0, beat=True, energy=0.5), MusicState.RHYTHMIC)
    b = MotionPolicy().decide(feat(0.0, beat=True, energy=0.5), MusicState.RHYTHMIC, musical=None)
    assert a == b
