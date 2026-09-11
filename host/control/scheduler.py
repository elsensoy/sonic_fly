"""DroneScheduler - host side of the fire-at-T protocol.

Owns the serial link + clock tracker, allocates event ids, converts musical
target times into Arduino-ms `SCHED` lines, and tracks each event's lifecycle
(ACK/NAK -> FIRE -> REL) from the inbound stream.

    sched = DroneScheduler(SerialLink("/dev/ttyUSB0"))
    sched.open()
    sched.hello(); sched.sync(); sched.arm()
    ev = sched.schedule("U", t_host=beat_time - L, dur_ms=90)
    while running:
        sched.service()               # pump replies + keep the clock fresh
        ...
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable

from host.control.clock_sync import Clock, ClockTracker, Probe
from host.control.link import SerialLink
from host.control.protocol import Ack, Boot, Fire, Hello, Pong, Rel, Safe

EventCb = Callable[["ScheduledEvent"], None]

# an Arduino millis() jump this far backwards (while host time moves forward)
# means the board reset under us.
RESET_BACKSTEP_MS = 2000


@dataclass
class ScheduledEvent:
    id: int
    act: str
    at_ms: int
    dur_ms: int
    grp: int = 0
    t_sched: float = field(default_factory=time.monotonic)
    status: str = "pending"           # pending -> acked | rejected -> fired -> released
    reason: str = ""
    fired_ms: int | None = None
    released_ms: int | None = None

    @property
    def schedule_error_ms(self) -> int | None:
        return None if self.fired_ms is None else self.fired_ms - self.at_ms


class DroneScheduler:
    def __init__(self, link: SerialLink, resync_interval: float = 2.0) -> None:
        self.link = link
        self.tracker = ClockTracker(resync_interval=resync_interval)
        self.caps: Hello | None = None
        self.safe_hold = False
        self.needs_resync = False
        self._next_id = 1
        self._probe_n = 0
        self._probe_t0: dict[str, float] = {}
        self._last_arduino_ms = 0
        self.events: dict[int, ScheduledEvent] = {}
        self.on_fire: EventCb | None = None
        self.on_reject: EventCb | None = None
        self.on_safe: Callable[[str], None] | None = None
        self.on_reset: Callable[[str], None] | None = None

    @property
    def clock(self) -> Clock | None:
        return self.tracker.model

    # -- lifecycle ---------------------------------------------------
    def open(self) -> None:
        self.link.open()

    def close(self) -> None:
        try:
            self.abort()
        finally:
            self.link.close()

    def __enter__(self) -> "DroneScheduler":
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # -- handshake -------------------------------------------------------
    def hello(self, timeout: float = 1.0) -> Hello:
        self.link.send("HELLO")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self.link.get(timeout=deadline - time.monotonic())
            if isinstance(msg, Hello):
                self.caps = msg
                return msg
        raise TimeoutError("no HELLO reply")

    def sync(self, n: int = 8, gap: float = 0.02, timeout: float = 1.0) -> Clock:
        """Blocking clock-sync burst, routed through pump()."""
        for _ in range(n):
            self._send_probe()
            time.sleep(gap)
            self.pump()
        deadline = time.monotonic() + timeout
        while self.clock is None and time.monotonic() < deadline:
            self.pump()
            time.sleep(0.01)
        if self.clock is None:
            raise TimeoutError("no PONG during sync")
        self.needs_resync = False
        return self.clock

    def arm(self) -> None:
        self.link.send("ARM")

    def disarm(self) -> None:
        self.link.send("DISARM")

    # -- scheduling -----------------------------------------------------
    def _alloc_id(self) -> int:
        eid = self._next_id
        self._next_id = (self._next_id % 0xFFFF) + 1
        return eid

    def schedule(self, act: str, t_host: float, dur_ms: int, grp: int = 0) -> ScheduledEvent:
        """Schedule `act` to assert at host-clock time `t_host` (monotonic seconds)."""
        if self.clock is None:
            raise RuntimeError("call sync() before schedule()")
        return self.schedule_at(act, self.clock.to_arduino_ms(t_host), dur_ms, grp)

    def schedule_at(self, act: str, at_ms: int, dur_ms: int, grp: int = 0) -> ScheduledEvent:
        eid = self._alloc_id()
        ev = ScheduledEvent(eid, act, at_ms & 0xFFFFFFFF, dur_ms, grp)
        self.events[eid] = ev
        self.link.send(f"SCHED {eid} {ev.at_ms} {act} {dur_ms} {grp}")
        return ev

    def now(self, act: str, dur_ms: int) -> None:
        self.link.send(f"NOW {act} {dur_ms}")

    def cancel(self, eid: int) -> None:
        self.link.send(f"CANCEL {eid}")

    def cancel_all(self) -> None:
        self.link.send("CANCEL all")

    def abort(self) -> None:
        self.link.send("ABORT")

    # -- clock upkeep -------------------------------------------------
    def _send_probe(self) -> None:
        seq = f"s{self._probe_n}"
        self._probe_n += 1
        self._probe_t0[seq] = time.monotonic()
        if len(self._probe_t0) > 32:
            for k in list(self._probe_t0)[:16]:
                self._probe_t0.pop(k, None)
        self.link.send(f"PING {seq}")
        self.tracker.mark_sent()

    def maybe_resync(self) -> None:
        if self.clock is not None and self.tracker.due():
            self._send_probe()

    def service(self) -> None:
        """Call this regularly from your loop: process replies, keep clock fresh."""
        self.pump()
        if self.needs_resync:
            try:
                self.sync()
            except TimeoutError:
                pass
        else:
            self.maybe_resync()

    # -- inbound processing -------------------------------------------
    def pump(self) -> None:
        for msg in self.link.drain():
            self._handle(msg)

    def _reset(self, reason: str) -> None:
        self.tracker.reset()
        self.events.clear()
        self.safe_hold = False
        self._last_arduino_ms = 0
        self._probe_t0.clear()
        self.needs_resync = True
        if self.on_reset:
            self.on_reset(reason)

    def _handle(self, msg) -> None:
        if isinstance(msg, Pong):
            t0 = self._probe_t0.pop(msg.seq, None)
            if t0 is None:
                return
            if self._last_arduino_ms and msg.millis + RESET_BACKSTEP_MS < self._last_arduino_ms:
                self._reset("clock-backwards")
                return
            self._last_arduino_ms = max(self._last_arduino_ms, msg.millis)
            self.tracker.add(Probe(t0, time.monotonic(), msg.millis))
            self.safe_hold = False           # a PONG means the link is live again
        elif isinstance(msg, Ack):
            ev = self.events.get(msg.id)
            if ev and ev.status == "pending":
                ev.status = "acked" if msg.ok else "rejected"
                ev.reason = msg.reason
                if not msg.ok and self.on_reject:
                    self.on_reject(ev)
        elif isinstance(msg, Fire):
            ev = self.events.get(msg.id)
            if ev:
                ev.status = "fired"
                ev.fired_ms = msg.millis
                if self.on_fire:
                    self.on_fire(ev)
        elif isinstance(msg, Rel):
            ev = self.events.get(msg.id)
            if ev:
                ev.status = "released"
                ev.released_ms = msg.millis
        elif isinstance(msg, Safe):
            self.safe_hold = True
            if self.on_safe:
                self.on_safe(msg.reason)
        elif isinstance(msg, Boot):
            self._reset("boot")
