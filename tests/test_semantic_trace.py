"""Semantic trace harness + the label-stability helpers."""

import pytest

from analysis.semantic_trace import _runs, summarise, trace

pytest.importorskip("torch")


def test_runs_helper():
    assert _runs([]) == (0, 0.0, 0)
    assert _runs(["a", "a", "a"]) == (0, 3.0, 3)
    sw, mean_run, longest = _runs(["a", "a", "b", "a", "a", "a"])
    assert sw == 2 and longest == 3


def test_trace_on_synth_produces_evolving_rows():
    rows = trace("synth", encoder="randproj", seconds=16, bpm=126)
    assert len(rows) >= 8
    assert rows[0]["t"] < 5.0                       # time is relative to the start
    for r in rows:
        assert r["texture"] in ("percussive", "melodic", "sparse", "unknown")
        assert r["energy"] in ("low", "mid", "high", "unknown")
        assert 0.0 <= r["conf"] <= 1.0
    # over a quiet->groove->peak->out arc *something* should change
    assert len({r["energy"] for r in rows}) >= 2
    assert "windows over" in summarise(rows)


def test_smoothing_suppresses_decisive_label_flips():
    import numpy as np

    from host.audio.audio_model import AudioModel, _pick

    def decisive_flips(alpha):
        m = AudioModel.__new__(AudioModel)
        m.semantic_alpha = alpha
        m._sem_ema = {}
        rng = np.random.default_rng(0)
        picks = []
        for _ in range(60):                        # jittery raw distribution
            base = {"a": 0.5, "b": 0.35, "c": 0.15}
            noisy = {k: max(0.0, v + rng.normal(0, 0.22)) for k, v in base.items()}
            tot = sum(noisy.values())
            noisy = {k: v / tot for k, v in noisy.items()}
            lbl, _ = _pick(m._smooth_labels({"x": noisy})["x"])
            picks.append(lbl)
        real = [p for p in picks if p != "unknown"]
        return sum(a != b for a, b in zip(real, real[1:]))

    assert decisive_flips(0.15) <= decisive_flips(0.9)
