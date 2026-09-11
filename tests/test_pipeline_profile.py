"""The decoupling proof: injected inference latency must not move timing-path jitter."""

import pytest

from analysis.pipeline_profile import run

pytest.importorskip("torch")


def test_profile_records_stage_latencies():
    p = run(0.0, bpm=128, seconds=7, profile="click")
    assert p.frames and all(f.dsp_ms >= 0 for f in p.frames)
    assert any(f.send_ms is not None for f in p.frames)
    assert any(f.model_staleness_ms is not None for f in p.frames)


def test_semantic_age_at_command_is_reported():
    p = run(0.0, bpm=128, seconds=8, profile="click")
    sa = p.semantic_age_at_command(threshold_ms=1000)
    assert sa["n"] >= 3
    assert sa["mean_ms"] > 0                        # a real staleness figure
    assert sa["frac_under"] == 1.0                  # all well under 1 s with no injected delay

    slow = run(600.0, bpm=128, seconds=8, profile="click")
    ss = slow.semantic_age_at_command(threshold_ms=350)
    assert ss["mean_ms"] > sa["mean_ms"]           # injected delay ages the semantic state
    assert ss["frac_under"] < 1.0


def test_injected_inference_delay_does_not_touch_the_timing_path():
    fast = run(0.0, bpm=128, seconds=10, profile="click")
    slow = run(300.0, bpm=128, seconds=10, profile="click")   # 300 ms per inference

    # the model got much slower...
    assert max(slow.model_infer_ms) > 250 and max(fast.model_infer_ms) < 120

    # ...but the fire-at-T path (schedule -> FIRE edge) did not move
    sf, ss = fast.scheduling_error(), slow.scheduling_error()
    assert sf["n"] >= 3 and ss["n"] >= 3
    assert abs(sf["p95_abs_ms"] - ss["p95_abs_ms"]) < 8

    # nor did DSP / decision
    def p95(xs):
        xs = sorted(xs)
        return xs[int(0.95 * (len(xs) - 1))]
    assert p95([f.dsp_ms for f in slow.frames]) < 10
    assert p95([f.decision_ms for f in slow.frames]) < 5

    # semantic state IS more stale, though (that's expected)
    assert (slow.semantic_age_at_command()["mean_ms"]
            > fast.semantic_age_at_command()["mean_ms"] + 100)
