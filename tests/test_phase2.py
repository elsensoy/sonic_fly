"""Phase 2: cooldown, concurrency cap, checksum enforcement, drift, BOOT resync."""

import time

from host.control.clock_sync import (
    ClockModel,
    ClockTracker,
    LinearClockModel,
    Probe,
    fit_linear,
)
from host.control.link import SerialLink
from host.control.scheduler import DroneScheduler
from host.sim.fake_arduino import FakeArduino
from tests.conftest import pump_until

# ---- clock / drift ------------------------------------------------------


def test_fit_linear_recovers_drift():
    # arduino clock runs 200 ppm fast:  arduino_ms = 1000.2 * t_host_s + 5000
    # (1 ms integer rounding on each PONG limits precision, so use a long
    #  baseline like a real session's re-sync history would give)
    probes = []
    for k in range(40):
        t0 = 100.0 + k
        t1 = t0 + 0.003
        probes.append(Probe(t0, t1, round(1000.2 * (0.5 * (t0 + t1)) + 5000.0)))
    m = fit_linear(probes)
    assert isinstance(m, LinearClockModel)
    assert abs(m.drift_ppm - 200) < 15
    t = 180.0
    assert abs(m.to_arduino_ms(t) - (1000.2 * t + 5000.0)) < 3


def test_tracker_offset_then_linear():
    tr = ClockTracker(min_span_for_drift=5.0)
    now = time.monotonic()
    tr.add(Probe(now, now + 0.003, 1000))
    tr.add(Probe(now + 0.02, now + 0.023, 1020))
    assert isinstance(tr.model, ClockModel)          # span too short for drift
    tr.probes.appendleft(Probe(now - 8, now - 8 + 0.003, -7000))
    assert isinstance(tr.model, LinearClockModel)    # now spans > 5 s


# ---- cooldown ---------------------------------------------------------


def test_cooldown_rejects_close_repeat(sched):
    s, fa = sched
    s.hello(); s.sync(n=6); s.arm()
    t = fa.millis() + 120
    a = s.schedule_at("U", t, dur_ms=30)
    b = s.schedule_at("U", t + 40, dur_ms=30)        # 40 ms < 60 ms cooldown
    assert pump_until(s, lambda: a.status in ("acked", "fired", "released"))
    assert pump_until(s, lambda: b.status == "rejected")
    assert b.reason == "cooldown"


def test_cooldown_allows_spaced_repeat(sched):
    s, fa = sched
    s.hello(); s.sync(n=6); s.arm()
    t = fa.millis() + 120
    a = s.schedule_at("U", t, dur_ms=30)
    c = s.schedule_at("U", t + 100, dur_ms=30)       # 100 ms > 60 ms cooldown
    assert pump_until(s, lambda: a.status in ("acked", "fired", "released"))
    assert pump_until(s, lambda: c.status in ("acked", "fired", "released"))


# ---- concurrency cap -------------------------------------------------


def test_concurrency_cap(sched):
    s, fa = sched
    s.hello(); s.sync(n=6); s.arm()
    t = fa.millis() + 150
    evs = [s.schedule_at(a, t, dur_ms=120) for a in ("U", "L", "F")]  # 3 non-conflicting
    for e in evs:
        assert pump_until(s, lambda e=e: e.status in ("acked", "fired", "released"))
    over = s.schedule_at("P", t + 10, dur_ms=120)                     # 4th overlapping
    assert pump_until(s, lambda: over.status == "rejected")
    assert over.reason == "conflict"


# ---- checksum enforcement ------------------------------------------


def test_arduino_requires_checksum():
    fa = FakeArduino(require_checksum=True)
    port = fa.start()
    s = DroneScheduler(SerialLink(port, checksum=False))     # host sends no *HH
    s.link.open(settle=False)
    time.sleep(0.05); s.link.drain()
    try:
        with_err = []
        s.link.send("HELLO")
        assert not pump_until(s, lambda: s.caps is not None, timeout=0.4)
    finally:
        s.link.close(); fa.stop()


def test_checksummed_link_still_works():
    fa = FakeArduino(require_checksum=True)
    port = fa.start()
    s = DroneScheduler(SerialLink(port, checksum=True))
    s.link.open(settle=False)
    time.sleep(0.05); s.link.drain()
    try:
        assert s.hello().proto == 1
    finally:
        s.link.close(); fa.stop()


def test_host_requires_checksum_drops_unchecksummed():
    fa = FakeArduino(emit_checksum=False)
    port = fa.start()
    link = SerialLink(port, checksum=True, require_checksum=True)
    s = DroneScheduler(link)
    s.link.open(settle=False)
    time.sleep(0.05); s.link.drain()
    try:
        s.link.send("HELLO")
        assert not pump_until(s, lambda: s.caps is not None, timeout=0.4)
        assert link.bad_checksums > 0
    finally:
        s.link.close(); fa.stop()


# ---- BOOT auto-resync ----------------------------------------------


def test_reboot_triggers_resync(sched):
    s, fa = sched
    s.hello(); s.sync(n=6); s.arm()
    assert s.clock is not None
    resets = []
    s.on_reset = resets.append

    fa.reboot()
    assert pump_until(s, lambda: bool(resets), timeout=1.0)
    assert s.needs_resync and s.clock is None

    s.service()                                  # should re-handshake
    assert pump_until(s, lambda: s.clock is not None, timeout=1.0)
    assert not s.needs_resync
