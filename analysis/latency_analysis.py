"""Beat-to-motion latency: reactive vs. predictive scheduling.

Runs the host stack (features -> state -> policy -> ScheduledController ->
FakeArduino) against a synthetic track whose beat times are known exactly, in
two modes, and reports the README > Temporal Alignment metric:

    mean |beat - motion| timing error, ms   (+ std, + signed bias)

  reactive    - issue the command as soon as the beat is detected
                (at = now + a small transport margin)
  predictive  - issue at  predict_next_beat() - L(primitive)

"motion" here is the Arduino's FIRE edge (when the MOSFET would close),
recovered from the fire-at-T telemetry and converted back to host time. The
physical tail (RF + stabilisation + motor spin-up) needs an external sensor -
pass its event timestamps to `estimate_L(..., motion_times=...)` and it drops
straight in; the comparison logic is unchanged.

    python -m analysis.latency_analysis --bpm 128 --seconds 30
    python -m analysis.latency_analysis --bpm 128 --seconds 30 --calibrate
"""

from __future__ import annotations

import argparse
import statistics
import time
from dataclasses import dataclass, field
from typing import Iterator

import numpy as np

from host.audio.capture import DEFAULT_BLOCK_SIZE, DEFAULT_SAMPLE_RATE, AudioFrame
from host.audio.features import FeatureExtractor
from host.audio.sources import synth_signal
from host.control.latency_model import LatencyModel
from host.control.link import SerialLink
from host.control.motion_policy import MotionPolicy
from host.control.music_state import MusicStateEstimator
from host.control.scheduled_controller import ScheduledController
from host.control.scheduler import DroneScheduler
from host.sim.fake_arduino import FakeArduino

REACT_MARGIN_S = 0.035      # transport headroom for the reactive path (else NAK late)
PRED_MIN_LEAD_S = 0.08      # predictive: skip a beat too soon to schedule for
MATCH_TOL_FRAC = 0.5        # a fire/beat pair matches within this fraction of a beat


# ---- synthetic source with ground-truth beats ------------------------


def _synth_source(sr: int, block: int, bpm: float, seconds: float, profile: str
                  ) -> tuple[Iterator[AudioFrame], list[float]]:
    sig = synth_signal(sr, seconds, bpm, profile)
    t0 = time.monotonic()
    period = 60.0 / bpm
    beats = [t0 + k * period for k in range(1, int(seconds / period))]

    def gen() -> Iterator[AudioFrame]:
        for i in range(0, len(sig) - block, block):
            yield AudioFrame(sig[i:i + block].copy(), t0 + i / sr)
            slack = (t0 + (i + block) / sr) - time.monotonic()
            if slack > 0:
                time.sleep(slack)

    return gen(), beats


# ---- run records --------------------------------------------------


@dataclass
class Fire:
    issued: float                     # perform() call time (host clock)
    target: float                     # requested motion time (host clock)
    fired: float | None = None        # FIRE edge, host clock
    primitive: str = ""
    rejected: str | None = None

    @property
    def lead_ms(self) -> float:
        return 1000.0 * (self.target - self.issued)


@dataclass
class RunResult:
    mode: str
    bpm: float
    true_beats: list[float]
    detected_beats: list[float] = field(default_factory=list)
    fires: list[Fire] = field(default_factory=list)
    _pending: list = field(default_factory=list, repr=False)   # (Fire, ScheduledEvent)

    @property
    def _period(self) -> float:
        return 60.0 / self.bpm

    def _match(self, times: list[float]) -> list[float]:
        """Signed (event - nearest true beat) in seconds, for matched events."""
        tol = self._period * MATCH_TOL_FRAC
        beats = np.array(self.true_beats)
        out = []
        for t in times:
            if beats.size == 0:
                continue
            d = t - beats[np.argmin(np.abs(beats - t))]
            if abs(d) <= tol:
                out.append(d)
        return out

    def motion_errors_ms(self, motion_times: list[float] | None = None) -> list[float]:
        mt = motion_times if motion_times is not None else [f.fired for f in self.fires if f.fired]
        return [1000.0 * d for d in self._match(mt)]

    def detection_lag_ms(self) -> list[float]:
        return [1000.0 * d for d in self._match(self.detected_beats)]

    def counts(self) -> dict[str, int]:
        return {
            "issued": len(self.fires),
            "fired": sum(f.fired is not None for f in self.fires),
            "rejected": sum(f.rejected is not None for f in self.fires),
            "true_beats": len(self.true_beats),
        }

    def reject_reasons(self) -> dict[str, int]:
        out: dict[str, int] = {}
        for f in self.fires:
            if f.rejected:
                out[f.rejected] = out.get(f.rejected, 0) + 1
        return out

    def summary(self, motion_times: list[float] | None = None) -> dict[str, float]:
        e = self.motion_errors_ms(motion_times)
        if not e:
            return {"n": 0}
        return {
            "n": len(e),
            "mean_abs_ms": statistics.fmean(abs(x) for x in e),
            "mean_signed_ms": statistics.fmean(e),
            "std_ms": statistics.pstdev(e) if len(e) > 1 else 0.0,
            "p90_abs_ms": float(np.percentile(np.abs(e), 90)),
        }


# ---- the run --------------------------------------------------------


def run_pipeline(
    mode: str,
    *,
    bpm: float = 128.0,
    seconds: float = 30.0,
    sr: int = DEFAULT_SAMPLE_RATE,
    block: int = DEFAULT_BLOCK_SIZE,
    profile: str = "click",
    latency: LatencyModel | None = None,
) -> RunResult:
    if mode not in ("reactive", "predictive"):
        raise ValueError(mode)

    fa = FakeArduino()
    port = fa.start()
    ctl = ScheduledController(
        DroneScheduler(SerialLink(port), resync_interval=0.35),
        latency=latency or LatencyModel(default_s=0.0),
    )
    ctl.open()
    ctl.scheduler.link.drain()
    ctl.handshake(sync_probes=8)
    ctl.arm()

    fx = FeatureExtractor(sr, block)
    est = MusicStateEstimator()
    pol = MotionPolicy()
    frames, beats = _synth_source(sr, block, bpm, seconds, profile)
    res = RunResult(mode=mode, bpm=bpm, true_beats=beats)

    try:
        for frame in frames:
            f = fx.push(frame)
            st = est.update(f)
            if f.beat:
                res.detected_beats.append(f.t)
            cmd = pol.select(f, st)
            now = time.monotonic()
            if cmd and ctl.clock is not None and not ctl.safe_hold:
                if mode == "reactive":
                    target = now + REACT_MARGIN_S
                else:
                    nb = fx.detector.predict_next_beat(now, min_lead=PRED_MIN_LEAD_S)
                    if nb is None:
                        ctl.service()
                        continue
                    target = nb
                plan = ctl.perform(cmd, at=target)
                ev = plan.events[0] if plan.events else None
                fire = Fire(now, target, primitive=cmd.primitive)
                res.fires.append(fire)
                res._pending.append((fire, ev))
            ctl.service()

        # let the last scheduled events fire
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            ctl.service()
            time.sleep(0.02)

        for fire, ev in res._pending:
            if ev is None:
                continue
            if ev.reason:
                fire.rejected = ev.reason
            if ev.fired_ms is not None and ctl.clock is not None:
                fire.fired = ctl.clock.to_host(ev.fired_ms)
    finally:
        ctl.close()
        fa.stop()
    return res


# ---- L estimation + reporting -----------------------------------


def estimate_L(result: RunResult, motion_times: list[float] | None = None) -> dict[str, float]:
    """Per-primitive L = mean(motion - true beat). Feeds LatencyModel.observe()."""
    tol = result._period * MATCH_TOL_FRAC
    beats = np.array(result.true_beats)
    by_prim: dict[str, list[float]] = {}
    for f in result.fires:
        t = f.fired
        if t is None or beats.size == 0:
            continue
        d = t - beats[np.argmin(np.abs(beats - t))]
        if abs(d) <= tol:
            by_prim.setdefault(f.primitive, []).append(d)
    out = {p: statistics.fmean(v) for p, v in by_prim.items() if v}
    alld = [d for v in by_prim.values() for d in v]
    if alld:
        out["default"] = statistics.fmean(alld)
    return out


def _fmt(s: dict) -> str:
    if s.get("n", 0) == 0:
        return "  (no matched fires)"
    return (f"  n={s['n']:<3d}  |err| mean={s['mean_abs_ms']:6.1f} ms   "
            f"bias={s['mean_signed_ms']:+6.1f} ms   std={s['std_ms']:5.1f} ms   "
            f"p90={s['p90_abs_ms']:6.1f} ms")


def compare(reactive: RunResult, predictive: RunResult) -> str:
    lag = reactive.detection_lag_ms()
    lines = [
        "beat-to-motion timing error   (motion = Arduino FIRE edge)",
        "",
        f"detection lag        mean={statistics.fmean(lag):+6.1f} ms  "
        f"std={statistics.pstdev(lag):5.1f} ms   (n={len(lag)})" if lag else "detection lag: n/a",
        "",
        f"reactive   {reactive.counts()}  rejects={reactive.reject_reasons()}",
        _fmt(reactive.summary()),
        "",
        f"predictive {predictive.counts()}  rejects={predictive.reject_reasons()}",
        _fmt(predictive.summary()),
        f"           lead time issued->target: "
        + (lambda L: f"min={min(L):.0f} mean={statistics.fmean(L):.0f} max={max(L):.0f} ms"
           if L else "n/a")([f.lead_ms for f in predictive.fires]),
    ]
    rs, ps = reactive.summary(), predictive.summary()
    if rs.get("n") and ps.get("n"):
        d_abs = rs["mean_abs_ms"] - ps["mean_abs_ms"]
        d_std = rs["std_ms"] - ps["std_ms"]
        lines += ["", f"predictive improves |err| by {d_abs:+.1f} ms and std by {d_std:+.1f} ms"]
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bpm", type=float, default=128.0)
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--profile", default="click", choices=["click", "arc"])
    ap.add_argument("--calibrate", action="store_true",
                    help="measure L from the reactive run, feed it back, re-run predictive")
    args = ap.parse_args(argv)

    print(f"running reactive   ({args.seconds:.0f}s @ {args.bpm:.0f} BPM) ...")
    reactive = run_pipeline("reactive", bpm=args.bpm, seconds=args.seconds, profile=args.profile)
    print(f"running predictive ({args.seconds:.0f}s @ {args.bpm:.0f} BPM) ...")
    predictive = run_pipeline("predictive", bpm=args.bpm, seconds=args.seconds, profile=args.profile)

    print("\n" + compare(reactive, predictive))

    if args.calibrate:
        L = estimate_L(reactive)
        print(f"\nmeasured L (reactive): { {k: round(v*1000,1) for k,v in L.items()} } ms")
        lm = LatencyModel(default_s=L.get("default", 0.0),
                          per_primitive={k: v for k, v in L.items() if k != "default"})
        print("re-running predictive with calibrated L ...")
        cal = run_pipeline("predictive", bpm=args.bpm, seconds=args.seconds,
                           profile=args.profile, latency=lm)
        print("\ncalibrated predictive")
        print(_fmt(cal.summary()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
