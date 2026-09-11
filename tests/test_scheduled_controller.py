"""ScheduledController: MotionCommand + target time -> scheduled channel events."""

import time

from host.control.latency_model import LatencyModel
from host.control.motion_primitives import MotionCommand
from host.control.scheduled_controller import ScheduledController
from tests.conftest import pump_until


def _ctl(sched, latency=None):
    s, fa = sched
    ctl = ScheduledController(s, latency=latency)
    ctl.handshake(sync_probes=6)
    ctl.arm()
    time.sleep(0.05)
    return ctl, fa


# ---- latency model -------------------------------------------------


def test_latency_model_observe_is_ema():
    m = LatencyModel(default_s=0.05, ema=0.5)
    m.observe("BOUNCE", 0.09)
    assert abs(m.get("BOUNCE") - 0.07) < 1e-9
    m.observe("BOUNCE", 0.09)
    assert abs(m.get("BOUNCE") - 0.08) < 1e-9
    assert m.get("RISE") == 0.05                 # untouched primitives keep the default


# ---- perform ------------------------------------------------------


def test_perform_schedules_all_ops_and_they_fire(sched):
    ctl, fa = _ctl(sched)
    plan = ctl.perform(MotionCommand("BOUNCE", intensity=0.7), at=time.monotonic() + 0.25)
    assert len(plan.events) == 2                 # U then D
    assert pump_until(ctl.scheduler, lambda: plan.status == "released", timeout=2.0)
    assert plan.schedule_error_ms is not None and abs(plan.schedule_error_ms) < 20
    assert fa.pins["U"] is False and fa.pins["D"] is False


def test_latency_compensation_shifts_target_earlier(sched):
    s, fa = sched
    ctl, _ = _ctl((s, fa), latency=LatencyModel(default_s=0.15))
    at = time.monotonic() + 0.5
    plan = ctl.perform(MotionCommand("RISE", intensity=0.5), at=at)
    want = s.clock.to_arduino_ms(at)
    assert plan.events[0].at_ms <= want - 140    # pulled ~150 ms earlier (minus rounding)


def test_perform_in_the_past_is_rejected_not_fired(sched):
    ctl, fa = _ctl(sched)
    plan = ctl.perform(MotionCommand("BOUNCE"), at=time.monotonic() - 0.2)
    assert pump_until(ctl.scheduler, lambda: plan.status == "rejected", timeout=1.0)
    assert all(e.reason == "late" for e in plan.events)
    assert all(e.fired_ms is None for e in plan.events)


def test_disarmed_perform_is_rejected(sched):
    s, fa = sched
    ctl = ScheduledController(s)
    ctl.handshake(sync_probes=6)                 # no arm()
    plan = ctl.perform(MotionCommand("SWAY_LEFT"), at=time.monotonic() + 0.2)
    assert pump_until(s, lambda: plan.status == "rejected", timeout=1.0)
    assert all(e.reason == "disarmed" for e in plan.events)


def test_conflicting_primitives_back_to_back(sched):
    ctl, fa = _ctl(sched)
    t = time.monotonic() + 0.3
    left = ctl.perform(MotionCommand("SWAY_LEFT", 0.6), at=t)
    right = ctl.perform(MotionCommand("SWAY_RIGHT", 0.6), at=t + 0.02)   # overlaps opposite yaw
    assert pump_until(ctl.scheduler, lambda: left.status in ("pending", "firing", "released"))
    assert pump_until(ctl.scheduler, lambda: right.status == "rejected", timeout=1.0)
    assert right.events[0].reason == "conflict"


def test_cancel_plan_retracts_pending(sched):
    ctl, fa = _ctl(sched)
    plan = ctl.perform(MotionCommand("RISE", 0.5), at=time.monotonic() + 1.0)
    assert pump_until(ctl.scheduler, lambda: plan.events[0].status == "acked")
    ctl.cancel_plan(plan)
    assert pump_until(ctl.scheduler, lambda: plan.events[0].status == "released", timeout=1.0)
    assert plan.events[0].fired_ms is None       # cancelled before it fired


def test_reflex_fires_promptly(sched):
    ctl, fa = _ctl(sched)
    t0 = fa.millis()
    plan = ctl.reflex(MotionCommand("YAW_TWITCH", 0.6))
    assert pump_until(ctl.scheduler, lambda: plan.status == "released", timeout=1.0)
    assert plan.events[0].fired_ms is not None
    assert plan.events[0].fired_ms - t0 < 150    # "soon"
