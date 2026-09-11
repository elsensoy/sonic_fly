"""PhysicalScheduler - the feasibility layer between policy and wire."""

import pytest

from host.control.motion_primitives import MotionCommand
from host.control.physical_scheduler import PhysicalScheduler


def test_min_interval_drops_ordinary_beats():
    ps = PhysicalScheduler(min_interval_ms=350)
    assert ps.submit(MotionCommand("SWAY_LEFT", 0.5), now=0.0).emit
    d = ps.submit(MotionCommand("SWAY_RIGHT", 0.5), now=0.2)      # 200 ms < 350
    assert not d.emit and d.reason == "min_interval"
    assert ps.submit(MotionCommand("SWAY_RIGHT", 0.5), now=0.4).emit


def test_slam_family_gets_a_wider_interval():
    ps = PhysicalScheduler(min_interval_ms=400, slam_interval_ms=650)
    assert ps.submit(MotionCommand("DIP", 0.5), now=0.0, family="SLAM").emit
    assert not ps.submit(MotionCommand("DIP", 0.5), now=0.5, family="SLAM").emit   # 500 < 650
    assert ps.submit(MotionCommand("DIP", 0.5), now=0.7, family="SLAM").emit
    # a SWAY-family command only needs the ordinary 400 ms gap
    ps2 = PhysicalScheduler(min_interval_ms=400, slam_interval_ms=650)
    assert ps2.submit(MotionCommand("BOUNCE", 0.5), now=0.0, family="SWAY").emit
    assert ps2.submit(MotionCommand("BOUNCE", 0.5), now=0.45, family="SWAY").emit


def test_accent_and_section_events_bypass_the_gap():
    ps = PhysicalScheduler(min_interval_ms=400)
    ps.submit(MotionCommand("DIP", 0.5), now=0.0)
    assert not ps.submit(MotionCommand("DIP", 0.5), now=0.2, strength=0.1).emit   # ordinary, too soon
    assert ps.submit(MotionCommand("YAW_TWITCH", 0.5), now=0.2, strength=0.9).emit  # accent bypasses
    assert not ps.submit(MotionCommand("DROP", 0.5), now=0.24).emit                 # 40 ms even for a section evt
    assert ps.submit(MotionCommand("DROP", 0.5), now=0.30).emit                     # 100 ms: section event through


def test_per_primitive_cooldown():
    ps = PhysicalScheduler(min_interval_ms=0)
    assert ps.submit(MotionCommand("YAW_TWITCH", 0.6), now=0.0).emit
    d = ps.submit(MotionCommand("YAW_TWITCH", 0.6), now=0.1)     # YAW cooldown is 260 ms
    assert not d.emit and d.reason == "cooldown"
    assert ps.submit(MotionCommand("YAW_TWITCH", 0.6), now=0.5).emit


def test_conflict_with_still_running_primitive():
    ps = PhysicalScheduler(min_interval_ms=0)
    assert ps.submit(MotionCommand("SWAY_LEFT", 0.6), now=0.0).emit
    d = ps.submit(MotionCommand("SWAY_RIGHT", 0.6), now=0.02)   # opposite direction, still running
    assert not d.emit and d.reason == "conflict"


def test_max_duration_rejects_overlong_envelope():
    ps = PhysicalScheduler(min_interval_ms=0, max_duration_ms=100)
    d = ps.submit(MotionCommand("RISE", 1.0), now=0.0)      # _rise at i=1 -> 220 ms envelope
    assert not d.emit and d.reason == "too_long"


def test_max_run_caps_one_direction():
    ps = PhysicalScheduler(min_interval_ms=0, max_run=3)
    for k in range(3):
        assert ps.submit(MotionCommand("SWAY_LEFT", 0.4), now=k).emit   # cooldown 200 ms, spaced 1 s
    d = ps.submit(MotionCommand("SWAY_LEFT", 0.4), now=3)
    assert not d.emit and d.reason == "repeat"
    assert ps.submit(MotionCommand("SWAY_RIGHT", 0.4), now=4).emit      # other axis resets the run


def test_cooldown_and_gate_introspection():
    ps = PhysicalScheduler(min_interval_ms=300)
    ps.submit(MotionCommand("YAW_TWITCH", 0.6), now=0.0)                 # YAW cooldown 260 ms
    assert ps.gate_remaining_ms(0.1) == pytest.approx(200.0, abs=1)
    assert ps.cooldown_remaining_ms("YAW_TWITCH", 0.1) == pytest.approx(160.0, abs=1)
    assert not ps.is_neutral(0.05) and ps.is_neutral(1.0)
    assert ps.last_emitted_primitive == "YAW_TWITCH"


def test_stats_and_report():
    ps = PhysicalScheduler(min_interval_ms=300)
    for k in range(12):
        ps.submit(MotionCommand("BOUNCE", 0.5), now=k * 0.1)     # every 100 ms
    assert ps.candidate == 12
    assert ps.emitted < 12 and ps.dropped == ps.candidate - ps.emitted
    assert "min_interval" in ps.drop_reasons()
    assert "candidate" in ps.report() and "dropped" in ps.report()
