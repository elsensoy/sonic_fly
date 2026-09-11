"""AudioEncoder interface + the V0 random-projection encoder."""

import numpy as np
import pytest

from host.audio.encoders import AudioEncoder, RandomProjectionEncoder

pytest.importorskip("torch")


def _wave(seconds=2.0, sr=16000, seed=0):
    t = np.arange(int(seconds * sr)) / sr
    rng = np.random.default_rng(seed)
    return (0.05 * rng.standard_normal(t.size)
            + 0.3 * np.sin(2 * np.pi * 220 * t)).astype(np.float32)


def test_abstract_encoder_cannot_be_instantiated():
    with pytest.raises(TypeError):
        AudioEncoder()


def test_randproj_encode_shape_and_norm():
    enc = RandomProjectionEncoder(dim=48, device="cpu")
    e = enc.encode(_wave(), 16000)
    assert e.shape == (48,) and e.dtype == np.float32
    assert abs(np.linalg.norm(e) - 1.0) < 1e-3
    assert enc.name == "randproj" and enc.dim == 48


def test_randproj_is_deterministic_and_seed_sensitive():
    w = _wave()
    a = RandomProjectionEncoder(seed=0, device="cpu").encode(w, 16000)
    b = RandomProjectionEncoder(seed=0, device="cpu").encode(w, 16000)
    c = RandomProjectionEncoder(seed=1, device="cpu").encode(w, 16000)
    assert np.allclose(a, b)
    assert not np.allclose(a, c)


def test_randproj_tracks_spectral_change():
    lo = _wave(seed=1)                                    # 220 Hz tone
    t = np.arange(2 * 16000) / 16000
    hi = (0.05 * np.random.default_rng(1).standard_normal(t.size)
          + 0.3 * np.sin(2 * np.pi * 3000 * t)).astype(np.float32)
    enc = RandomProjectionEncoder(device="cpu")
    e_lo, e_hi = enc.encode(lo, 16000), enc.encode(hi, 16000)
    e_lo2 = enc.encode(_wave(seed=2), 16000)              # another 220 Hz-ish tone
    assert float(e_lo @ e_hi) < float(e_lo @ e_lo2)       # low↔high less similar than low↔low


def test_labels_default_none_and_analyze_default():
    enc = RandomProjectionEncoder(device="cpu")
    w = _wave()
    assert enc.labels(w, 16000) is None
    emb, labels = enc.analyze(w, 16000)
    assert emb.shape == (enc.dim,) and labels is None


# ---- CLAP (integration; self-skips without transformers / network / disk) ----


@pytest.fixture(scope="module")
def clap():
    pytest.importorskip("transformers")
    pytest.importorskip("torchaudio")
    from host.audio.encoders import ClapEncoder

    enc = ClapEncoder(device="cpu")
    try:
        enc._ensure()
    except Exception as e:                       # no network / not enough disk / etc.
        pytest.skip(f"CLAP unavailable: {e}")
    return enc


def test_clap_encode_shape_and_norm(clap):
    e = clap.encode(_wave(2.0), 16000)
    assert e.shape == (512,) and e.dtype == np.float32
    assert abs(np.linalg.norm(e) - 1.0) < 1e-3


def test_clap_analyze_returns_category_scores(clap):
    emb, labels = clap.analyze(_wave(2.0), 16000)
    assert emb.shape == (512,)
    assert set(labels) >= {"texture", "energy"}
    for cat, scores in labels.items():
        assert abs(sum(scores.values()) - 1.0) < 1e-3      # softmax within category
        assert all(0.0 <= v <= 1.0 for v in scores.values())


def test_clap_embedding_varies_with_audio(clap):
    a = clap.encode(_wave(2.0, seed=1), 16000)
    b = clap.encode(_wave(2.0, seed=2), 16000)
    t = np.arange(2 * 16000) / 16000
    noisy = (0.3 * np.random.default_rng(3).standard_normal(t.size)).astype(np.float32)
    c = clap.encode(noisy, 16000)
    assert float(a @ c) < 0.999 and float(a @ b) < 1.0     # not collapsed


def test_clap_default_checkpoint_is_pinned():
    from host.audio.encoders import ClapEncoder

    assert len(ClapEncoder().revision) == 40                     # frozen to an exact commit
    assert ClapEncoder(model_id="some/other-clap").revision is None   # only the default is pinned
    assert ClapEncoder(revision="deadbeef").revision == "deadbeef"    # explicit override wins
