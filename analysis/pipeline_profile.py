"""Per-stage latency profile of the full pipeline, and the decoupling proof.

Runs  audio -> DSP -> (threaded GPU model) -> policy -> ScheduledController ->
FakeArduino  in real time over a synthetic track with known beats, and logs
per frame:

    stage latencies   : dsp_ms, decision_ms, send_ms, |scheduling error|
                        (+ model infer_ms from the worker)
    timestamps        : t_audio, t_semantic (+ staleness at command), t_command, t_fired
    feasibility       : host PhysicalScheduler drop tally + Arduino NAK tally
                        (candidate vs emitted vs dropped, by reason)

The point (README > Temporal Alignment, and the NVIDIA systems claim):

    model inference  = tens to hundreds of ms  (and you can inject 500 ms)
    BUT
    |scheduling error| (FIRE edge vs requested time) + DSP latency = unchanged

i.e. asynchronous GPU semantic inference on its own CUDA stream + a stale-safe
semantic-state cache isolates perception latency from the physical control
path. `beat-to-command` also shifts with the semantic state, but only because
the *policy picks different primitives* - the choreography question, not a
timing leak.

    python -m analysis.pipeline_profile --bpm 128 --seconds 30
    python -m analysis.pipeline_profile --bpm 128 --seconds 30 --inject-delay-ms 500
    python -m analysis.pipeline_profile --bpm 128 --seconds 24 --prove      # 0 vs 500 ms
"""

from __future__ import annotations

import argparse
import statistics
import time
from dataclasses import dataclass, field

import numpy as np

from analysis.latency_analysis import MATCH_TOL_FRAC, PRED_MIN_LEAD_S, _synth_source
from host.audio.capture import DEFAULT_BLOCK_SIZE, DEFAULT_SAMPLE_RATE
from host.audio.audio_model import ThreadedAudioModel
from host.audio.features import FeatureExtractor
from host.control.link import SerialLink
from host.control.motion_policy import MotionPolicy
from host.control.music_state import MusicStateEstimator
from host.control.scheduled_controller import ScheduledController
from host.control.scheduler import DroneScheduler
from host.sim.fake_arduino import FakeArduino


@dataclass
class Frame:
    t_audio: float
    dsp_ms: float
    decision_ms: float
    send_ms: float | None = None
    beat: bool = False
    t_command: float | None = None      # perform() target (host clock)
    t_fired: float | None = None        # FIRE edge (host clock), filled post-hoc
    model_seq: int | None = None
    t_semantic: float | None = None     # window-end time of the semantic state in use
    model_staleness_ms: float | None = None    # now - t_semantic at this frame
    sched_error_ms: float | None = None        # FIRE edge - requested time (fire-at-T accuracy)


@dataclass
class Profile:
    bpm: float
    inject_delay_ms: float
    frames: list[Frame] = field(default_factory=list)
    true_beats: list[float] = field(default_factory=list)
    model_infer_ms: list[float] = field(default_factory=list)
    host_sched: object = None                          # PhysicalScheduler (host-side gate)
    fw_rejects: dict = field(default_factory=dict)     # Arduino NAK reason -> count
    _pending: list = field(default_factory=list, repr=False)

    @property
    def _period(self) -> float:
        return 60.0 / self.bpm

    def _p(self, xs, q):
        return float(np.percentile(xs, q)) if xs else 0.0

    def stage_table(self) -> str:
        def row(name, xs, unit="ms"):
            if not xs:
                return f"  {name:24s} (none)"
            return (f"  {name:24s} mean {statistics.fmean(xs):7.2f}  "
                    f"p50 {self._p(xs,50):7.2f}  p95 {self._p(xs,95):7.2f}  "
                    f"max {max(xs):7.2f}  {unit}")
        dsp = [f.dsp_ms for f in self.frames]
        dec = [f.decision_ms for f in self.frames]
        snd = [f.send_ms for f in self.frames if f.send_ms is not None]
        stale = [f.model_staleness_ms for f in self.frames if f.model_staleness_ms is not None]
        sperr = [abs(f.sched_error_ms) for f in self.frames if f.sched_error_ms is not None]
        return "\n".join([
            row("DSP latency", dsp),
            row("decision latency", dec),
            row("serial-send latency", snd),
            row("|scheduling error|", sperr),          # FIRE vs requested - the fire-at-T path
            row("model inference latency", self.model_infer_ms),
            row("model-state staleness", stale),
        ])

    def scheduling_error(self) -> dict:
        xs = [f.sched_error_ms for f in self.frames if f.sched_error_ms is not None]
        if not xs:
            return {"n": 0}
        return {"n": len(xs), "mean_ms": statistics.fmean(xs),
                "std_ms": statistics.pstdev(xs) if len(xs) > 1 else 0.0,
                "p95_abs_ms": self._p([abs(x) for x in xs], 95)}

    def semantic_age_at_command(self, threshold_ms: float = 350.0) -> dict:
        """How stale was the semantic state when a motion command was issued?"""
        ages = [f.model_staleness_ms for f in self.frames
                if f.t_command is not None and f.model_staleness_ms is not None]
        if not ages:
            return {"n": 0}
        under = sum(a < threshold_ms for a in ages) / len(ages)
        return {"n": len(ages), "mean_ms": statistics.fmean(ages),
                "p95_ms": self._p(ages, 95), "max_ms": max(ages),
                "frac_under": under, "threshold_ms": threshold_ms}

    def beat_to_command(self) -> dict:
        """Signed (FIRE - nearest true beat), matched within half a beat."""
        beats = np.array(self.true_beats)
        tol = self._period * MATCH_TOL_FRAC
        errs = []
        for f in self.frames:
            if f.t_fired is None or beats.size == 0:
                continue
            d = f.t_fired - beats[np.argmin(np.abs(beats - f.t_fired))]
            if abs(d) <= tol:
                errs.append(d * 1000.0)
        if not errs:
            return {"n": 0}
        return {"n": len(errs), "mean_ms": statistics.fmean(errs),
                "std_ms": statistics.pstdev(errs) if len(errs) > 1 else 0.0,
                "p90_abs_ms": float(np.percentile(np.abs(errs), 90))}


def run(inject_delay_ms: float = 0.0, *, bpm: float = 128.0, seconds: float = 30.0,
        sr: int = DEFAULT_SAMPLE_RATE, block: int = DEFAULT_BLOCK_SIZE,
        profile: str = "arc", encoder: str = "randproj") -> Profile:
    fa = FakeArduino()
    port = fa.start()
    ctl = ScheduledController(DroneScheduler(SerialLink(port), resync_interval=0.35))
    ctl.open()
    ctl.scheduler.link.drain()
    ctl.handshake(sync_probes=8)
    ctl.arm()

    fx = FeatureExtractor(sr, block)
    est = MusicStateEstimator()
    pol = MotionPolicy()
    enc = None
    if encoder == "clap":
        from host.audio.encoders import ClapEncoder
        enc = ClapEncoder()
    model = ThreadedAudioModel(sample_rate=sr, encoder=enc,
                               inference_delay_s=inject_delay_ms / 1000.0)
    model.start()
    time.sleep(0.3)                                  # let the worker load

    frames, beats = _synth_source(sr, block, bpm, seconds, profile)
    prof = Profile(bpm=bpm, inject_delay_ms=inject_delay_ms, true_beats=beats)

    try:
        for af in frames:
            t0 = time.perf_counter()
            f = fx.push(af)
            st = est.update(f)
            model.push(af)                           # non-blocking
            musical = model.latest()
            t1 = time.perf_counter()

            cmd = pol.select(f, st, musical)
            t2 = time.perf_counter()

            rec = Frame(
                t_audio=af.t_capture,
                dsp_ms=(t1 - t0) * 1000.0,
                decision_ms=(t2 - t1) * 1000.0,
                beat=f.beat,
                model_seq=(musical.seq if musical else None),
                t_semantic=(musical.t if musical else None),
                model_staleness_ms=((time.monotonic() - musical.t) * 1000.0 if musical else None),
            )

            if cmd and ctl.clock is not None and not ctl.safe_hold:
                now = time.monotonic()
                nb = fx.detector.predict_next_beat(now, min_lead=PRED_MIN_LEAD_S)
                target = nb if nb else now + 0.12
                t3 = time.perf_counter()
                plan = ctl.perform(cmd, at=target)
                rec.send_ms = (time.perf_counter() - t3) * 1000.0
                rec.t_command = target
                prof._pending.append((rec, plan.events[0] if plan.events else None))
            prof.frames.append(rec)
            ctl.service()

        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            ctl.service()
            time.sleep(0.02)

        for rec, ev in prof._pending:
            if ev and ev.fired_ms is not None and ctl.clock is not None:
                rec.t_fired = ctl.clock.to_host(ev.fired_ms)
                if ev.schedule_error_ms is not None:
                    rec.sched_error_ms = float(ev.schedule_error_ms)
        prof.model_infer_ms = list(model.infer_log)
        prof.host_sched = pol.scheduler
        prof.fw_rejects = _tally(e.reason for e in ctl.scheduler.events.values() if e.reason)
    finally:
        model.stop()
        ctl.close()
        fa.stop()
    return prof


def _tally(reasons):
    out: dict[str, int] = {}
    for r in reasons:
        out[r] = out.get(r, 0) + 1
    return out


def _fmt_bc(d: dict) -> str:
    if d.get("n", 0) == 0:
        return "  (no matched commands)"
    return (f"  n={d['n']:<3d}  mean {d['mean_ms']:+6.1f} ms   "
            f"std {d['std_ms']:5.1f} ms   p90|err| {d['p90_abs_ms']:5.1f} ms")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bpm", type=float, default=128.0)
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--profile", default="arc", choices=["click", "arc"])
    ap.add_argument("--encoder", default="randproj", choices=["randproj", "clap"])
    ap.add_argument("--inject-delay-ms", type=float, default=0.0)
    ap.add_argument("--prove", action="store_true",
                    help="run with 0 ms and 500 ms injected inference delay, compare")
    args = ap.parse_args(argv)

    if args.prove:
        runs = [("no delay", 0.0), ("+500 ms inference", 500.0)]
    else:
        runs = [(f"+{args.inject_delay_ms:.0f} ms" if args.inject_delay_ms else "baseline",
                 args.inject_delay_ms)]

    results = []
    for label, d in runs:
        print(f"running [{label}] ...")
        p = run(d, bpm=args.bpm, seconds=args.seconds, profile=args.profile, encoder=args.encoder)
        results.append((label, p))

    for label, p in results:
        print(f"\n=== {label} ===")
        print(p.stage_table())
        print("  beat-to-command:")
        print(_fmt_bc(p.beat_to_command()))
        sa = p.semantic_age_at_command()
        if sa.get("n"):
            print(f"  semantic age at command:  mean {sa['mean_ms']:.0f} ms  "
                  f"p95 {sa['p95_ms']:.0f} ms  max {sa['max_ms']:.0f} ms")
            print(f"  {sa['frac_under']:.0%} of motion decisions used semantic context "
                  f"< {sa['threshold_ms']:.0f} ms old")
        if p.host_sched is not None and p.host_sched.candidate:
            print(f"  host scheduler:  {p.host_sched.report()}")
        if p.fw_rejects:
            print(f"  arduino NAKs:    {p.fw_rejects}")

    if len(results) == 2:
        (la, pa), (lb, pb) = results
        sa, sb = pa.scheduling_error(), pb.scheduling_error()
        da = [f.dsp_ms for f in pa.frames]
        db = [f.dsp_ms for f in pb.frames]
        print(f"\ninference {la} -> {lb}:")
        if sa.get("n") and sb.get("n"):
            print(f"  |scheduling error|  p95 {sa['p95_abs_ms']:.1f} -> {sb['p95_abs_ms']:.1f} ms   "
                  f"(std {sa['std_ms']:.1f} -> {sb['std_ms']:.1f})")
        print(f"  DSP latency         p95 {pa._p(da,95):.2f} -> {pb._p(db,95):.2f} ms")
        ok = (sa.get("n") and sb.get("n")
              and abs(sa["p95_abs_ms"] - sb["p95_abs_ms"]) < 5
              and abs(pa._p(da, 95) - pb._p(db, 95)) < 3)
        print("  -> the physical timing path (schedule + fire) is unmoved by "
              "model inference latency." if ok else
              "  -> WARNING: a timing-path stage moved.")
        print("  (beat-to-command also depends on which primitives the policy chose,\n"
              "   which *does* change with the semantic state - that's the choreography\n"
              "   question, separate from timing.)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
