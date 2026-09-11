"""Beat prediction (PLL) + the latency experiment harness."""

import numpy as np

from analysis.latency_analysis import Fire, RunResult, estimate_L, run_pipeline
from host.audio.beat_detector import BeatDetector


# ---- PLL beat prediction ------------------------------------------


def _feed_onsets(det: BeatDetector, period: float, n: int, jitter: float, t0: float = 10.0, seed=0):
    rng = np.random.default_rng(seed)
    for _ in range(3):
        det._iois.append(period)
    det._pll_update(t0)                       # init
    last_true = t0
    for k in range(1, n):
        true_t = t0 + k * period + rng.normal(0, jitter)
        det._pll_update(true_t + det.report_latency_s)   # detector sees it late
        last_true = true_t
    return last_true


def test_pll_predicts_next_beat_through_jitter():
    det = BeatDetector(16000, 512)
    period = 0.48
    last = _feed_onsets(det, period, 40, jitter=0.010)
    now = last + 0.12
    pred = det.predict_next_beat(now)
    true_next = 10.0 + np.ceil((now - 10.0) / period) * period
    assert abs(pred - true_next) < 0.015            # <15 ms despite 10 ms onset jitter
    assert abs(det.bpm - 60.0 / period) < 3


def test_pll_rejects_a_single_outlier():
    det = BeatDetector(16000, 512)
    period = 0.5
    _feed_onsets(det, period, 20, jitter=0.003)
    ref_before = det._pll_ref
    det._pll_update(10.0 + 20 * period + 0.22 + det.report_latency_s)   # 220 ms off
    assert abs(det._pll_ref - ref_before) < 0.02     # barely moved


def test_predict_none_before_lock():
    det = BeatDetector(16000, 512)
    assert det.predict_next_beat(0.0) is None


# ---- RunResult bookkeeping -------------------------------------


def test_runresult_matches_and_summarises():
    beats = [1.0, 2.0, 3.0, 4.0]
    r = RunResult(mode="x", bpm=60.0, true_beats=beats)
    r.fires = [
        Fire(0, 0, fired=1.05, primitive="BOUNCE"),   # +50 ms
        Fire(0, 0, fired=2.03, primitive="BOUNCE"),   # +30 ms
        Fire(0, 0, fired=5.60, primitive="DIP"),      # 1.6 s from any beat -> unmatched
        Fire(0, 0, fired=None, primitive="DIP", rejected="late"),
    ]
    errs = r.motion_errors_ms()
    assert len(errs) == 2 and abs(errs[0] - 50) < 1e-6
    s = r.summary()
    assert s["n"] == 2 and abs(s["mean_signed_ms"] - 40) < 1e-6
    L = estimate_L(r)
    assert abs(L["default"] - 0.040) < 1e-6 and "BOUNCE" in L


# ---- end-to-end harness smoke ---------------------------------


def test_run_pipeline_both_modes():
    react = run_pipeline("reactive", bpm=128, seconds=8, profile="click")
    pred = run_pipeline("predictive", bpm=128, seconds=8, profile="click")
    assert react.counts()["issued"] > 0 and react.counts()["fired"] > 0
    assert pred.counts()["fired"] > 0
    assert pred.summary()["n"] >= 1
