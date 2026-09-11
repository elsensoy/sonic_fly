"""Host stack <-> FakeArduino over a real pty (phase 1 behaviour)."""

import time

from tests.conftest import pump_until


def test_hello_and_sync(sched):
    s, _ = sched
    caps = s.hello()
    assert caps.slots == 16 and caps.proto == 1
    model = s.sync(n=6)
    assert model.rtt_ms < 100


def test_scheduled_event_fires_near_target(sched):
    s, fa = sched
    s.hello(); s.sync(n=6); s.arm()
    time.sleep(0.05)

    target = fa.millis() + 120
    ev = s.schedule_at("U", target, dur_ms=80)

    assert pump_until(s, lambda: ev.status == "released")
    assert ev.fired_ms is not None
    assert abs(ev.schedule_error_ms) < 15
    assert abs((ev.released_ms - ev.fired_ms) - 80) < 15
    assert fa.pins["U"] is False


def test_late_event_is_rejected_not_fired(sched):
    s, fa = sched
    s.hello(); s.sync(n=6); s.arm()
    ev = s.schedule_at("U", fa.millis() - 50, dur_ms=80)
    assert pump_until(s, lambda: ev.status == "rejected")
    assert ev.reason == "late"
    assert ev.fired_ms is None


def test_disarmed_flight_command_rejected(sched):
    s, fa = sched
    s.hello(); s.sync(n=6)
    ev = s.schedule_at("L", fa.millis() + 100, dur_ms=50)
    assert pump_until(s, lambda: ev.status == "rejected")
    assert ev.reason == "disarmed"


def test_mutual_exclusion_conflict(sched):
    s, fa = sched
    s.hello(); s.sync(n=6); s.arm()
    t = fa.millis() + 150
    a = s.schedule_at("L", t, dur_ms=100)
    b = s.schedule_at("R", t + 30, dur_ms=100)
    assert pump_until(s, lambda: a.status in ("acked", "fired", "released"))
    assert pump_until(s, lambda: b.status == "rejected")
    assert b.reason == "conflict"


def test_link_watchdog_safe_hold(sched):
    s, fa = sched
    s.hello(); s.sync(n=6); s.arm()
    got = []
    s.on_safe = got.append
    assert pump_until(s, lambda: bool(got), timeout=1.5)
    assert got[0] == "linkloss"
