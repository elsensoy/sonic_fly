"""ScheduledController - MotionCommand + target time -> fire-at-T SCHED lines.

The one place the primitive -> channel-op expansion meets the wire. The same
object drives a real Arduino or `host/sim/fake_arduino.py` - it's only a port.

    ctl = ScheduledController(DroneScheduler(SerialLink(port)))
    ctl.open(); ctl.handshake(); ctl.arm()
    ...
    cmd = policy.select(feats, state)
    if cmd:
        ctl.perform(cmd, at=beat_detector.predict_next_beat())   # host monotonic seconds
    ctl.service()                                                # pump replies, keep clock fresh
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

from host.control.latency_model import LatencyModel
from host.control.motion_primitives import MotionCommand, expand
from host.control.scheduler import DroneScheduler, ScheduledEvent

REFLEX_LEAD_MS = 20   # enough for serial TX + parse so the first edge isn't NAK'd late


@dataclass
class Plan:
    """One performed MotionCommand = a group of scheduled channel events."""

    grp: int
    primitive: str
    at: float | None                 # host monotonic target (None for reflex)
    events: list[ScheduledEvent] = field(default_factory=list)
    t_issued: float = field(default_factory=time.monotonic)

    @property
    def status(self) -> str:
        if not self.events:
            return "empty"
        st = {e.status for e in self.events}
        if st == {"released"}:
            return "released"
        if st <= {"rejected"}:
            return "rejected"
        if st & {"fired", "released"}:
            return "firing"
        if "rejected" in st:
            return "partial"
        return "pending"

    @property
    def schedule_error_ms(self) -> int | None:
        for e in self.events:
            if e.schedule_error_ms is not None:
                return e.schedule_error_ms
        return None


class ScheduledController:
    def __init__(
        self,
        scheduler: DroneScheduler,
        latency: LatencyModel | None = None,
        keep_plans: int = 64,
    ) -> None:
        self.scheduler = scheduler
        self.latency = latency or LatencyModel()
        self.keep_plans = keep_plans
        self.armed = False
        self._grp = 1
        self.plans: dict[int, Plan] = {}

    # -- lifecycle ---------------------------------------------------
    def open(self) -> None:
        self.scheduler.open()

    def close(self) -> None:
        try:
            self.scheduler.abort()
        finally:
            self.scheduler.close()

    def __enter__(self) -> "ScheduledController":
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def handshake(self, sync_probes: int = 8) -> None:
        self.scheduler.hello()
        self.scheduler.sync(n=sync_probes)

    def arm(self) -> None:
        self.scheduler.arm()
        self.armed = True

    def disarm(self) -> None:
        self.scheduler.disarm()
        self.armed = False

    def service(self) -> None:
        self.scheduler.service()

    @property
    def clock(self):
        return self.scheduler.clock

    @property
    def safe_hold(self) -> bool:
        return self.scheduler.safe_hold

    # -- performing -------------------------------------------------
    def perform(self, cmd: MotionCommand, at: float) -> Plan:
        """Schedule `cmd` so its motion lands at host-clock time `at` (seconds).

        Each channel op is placed at ``at - L(primitive) + op.at_ms``. An op that
        works out to the past is NAK'd `late` by the Arduino and shows up as a
        rejected event - a skipped beat, never a lag.
        """
        if self.clock is None:
            raise RuntimeError("call handshake() before perform()")
        cmd = cmd.clamp()
        ops, _ = expand(cmd)
        grp = self._next_grp()
        lead = self.latency.get(cmd.primitive)
        plan = Plan(grp, cmd.primitive, at)
        for op in ops:
            t_host = at - lead + op.at_ms / 1000.0
            plan.events.append(self.scheduler.schedule(op.act, t_host, op.dur_ms, grp=grp))
        self._store(plan)
        return plan

    def reflex(self, cmd: MotionCommand) -> Plan:
        """Fire `cmd` as soon as possible - the low-latency path for reflexes."""
        cmd = cmd.clamp()
        ops, _ = expand(cmd)
        grp = self._next_grp()
        plan = Plan(grp, cmd.primitive, None)
        if self.clock is not None:
            base = self.clock.now_arduino_ms() + REFLEX_LEAD_MS
            for op in ops:
                plan.events.append(
                    self.scheduler.schedule_at(op.act, base + op.at_ms, op.dur_ms, grp=grp)
                )
        else:
            for op in ops:                       # no clock yet: best-effort immediate
                self.scheduler.now(op.act, op.dur_ms)
        self._store(plan)
        return plan

    def cancel_plan(self, plan: Plan) -> None:
        """Retract a plan's not-yet-fired events (firmware has no grp store yet)."""
        for ev in plan.events:
            if ev.status in ("pending", "acked"):
                self.scheduler.cancel(ev.id)

    def hold(self) -> None:
        """Cancel everything still pending; let in-flight pulses finish."""
        self.scheduler.cancel_all()

    def stop(self) -> None:
        """Emergency: release every output now, drop the schedule."""
        self.scheduler.abort()

    # -- internal --------------------------------------------------
    def _next_grp(self) -> int:
        g = self._grp
        self._grp = (self._grp % 0xFFFF) + 1
        return g

    def _store(self, plan: Plan) -> None:
        self.plans[plan.grp] = plan
        if len(self.plans) > self.keep_plans:
            for k in sorted(self.plans, key=lambda k: self.plans[k].t_issued)[: len(self.plans) - self.keep_plans]:
                del self.plans[k]
