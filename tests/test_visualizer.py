"""Drone2D kinematics, Timeline data model, and a headless panel-render smoke."""

import os

import pytest

from host.sim.drone_2d import Drone2D
from host.sim.panels import Timeline

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")


def _run(d: Drone2D, ch: dict, secs: float, dt: float = 1 / 120):
    for _ in range(int(secs / dt)):
        d.step(dt, ch)


# ---- Drone2D --------------------------------------------------------


def test_idle_settles_at_hover():
    d = Drone2D()
    d.s.y, d.s.x = 0.4, -0.3
    _run(d, {}, 3.0)
    assert abs(d.s.y) < 0.02 and abs(d.s.x) < 0.02


def test_U_rises_D_dips():
    up = Drone2D(); _run(up, {"U": True}, 0.4)
    dn = Drone2D(); _run(dn, {"D": True}, 0.4)
    assert up.s.y > 0.15
    assert dn.s.y < -0.15


def test_sway_returns_to_centre():
    d = Drone2D()
    _run(d, {"L": True}, 0.25)
    assert d.s.x < -0.03 and d.s.vx < 0   # moving left, displaced left
    _run(d, {}, 3.0)
    assert abs(d.s.x) < 0.04              # sprang back


def test_position_stays_bounded():
    d = Drone2D()
    _run(d, {"U": True, "R": True}, 10.0)
    assert -1.0 <= d.s.y <= 1.0 and -1.2 <= d.s.x <= 1.2


def test_thrust_tracks_up():
    d = Drone2D()
    _run(d, {"U": True}, 1.0)
    hi = d.s.thrust
    _run(d, {}, 1.0)
    assert hi > d.s.thrust > 0.2         # boosted, then back to the hover baseline


# ---- Timeline ------------------------------------------------------


def test_timeline_prunes_old_samples():
    tl = Timeline(window_s=4.0)
    for k in range(200):
        tl.push_sample(k * 0.1, 0.5)
        tl.push_mark(k * 0.1, "beat")
    tl._prune(now=20.0)
    assert all(t >= 20.0 - 4.0 - 0.5 for t, _ in tl.samples)
    assert all(t >= 20.0 - 4.0 - 0.5 for t, *_ in tl.marks)
    assert len(tl.samples) < 60


# ---- headless render smoke ---------------------------------------


def test_panels_render_to_surface(tmp_path):
    pygame = pytest.importorskip("pygame")
    from host.audio.features import AudioFeatures
    from host.control.motion_primitives import MotionCommand, expand
    from host.sim import panels

    from host.audio.audio_model import MusicalState
    from host.control.motion_policy import MotionPolicy
    from host.control.music_state import MusicState

    pygame.init()
    surf = pygame.Surface((980, 660))
    surf.fill(panels.BG)

    drone_rect = pygame.Rect(0, 0, 440, 430)
    tele_rect = pygame.Rect(440, 0, 540, 430)
    time_rect = pygame.Rect(0, 430, 980, 140)
    log_rect = pygame.Rect(0, 570, 980, 90)

    d = Drone2D()
    _run(d, {"U": True}, 0.3)
    ch = {"U": True}
    pygame.draw.rect(surf, panels.PANEL, drone_rect)
    d.draw(surf, drone_rect, ch)

    f = AudioFeatures(t=1.0, beat=True, onset_strength=0.8, kind="beat", bpm=124.0,
                      energy=0.7, bass=0.6, mid=0.4, high=0.3, building=0.2, transient=False)
    musical = MusicalState(t=0.9, intensity=0.7, texture="percussive", section_change=False,
                           energy="high", density_class="mid", confidence=0.81, novelty=0.07)
    pol = MotionPolicy()
    pol.select(f, MusicState.ENERGETIC, musical)
    cand = pol.last_candidate or MotionCommand("BOUNCE", 0.8)
    ops, env = expand(cand)
    panels.draw_telemetry(surf, tele_rect, t=1.0, bpm=124.0, beat=True,
                          musical=musical, family="SLAM", primitive=cand.primitive,
                          intensity=cand.intensity, decision=("emit", ops, env), dec_age=0.1,
                          sched=pol.scheduler, now=1.0, model_ms=48.0, model_stale_ms=110.0)
    panels.draw_command_log(surf, log_rect, [
        ("  1.0  SLAM/YAW_TWITCH  ->  YAW_RIGHT 54  YAW_LEFT 54", panels.INK),
        ("  1.3  DIP  REJECTED  (cooldown)", panels.REJECT),
    ])

    tl = Timeline(window_s=8.0)
    for k in range(60):
        tl.push_sample(k * 0.1, 0.3 + 0.4 * (k % 5 == 0))
        if k % 5 == 0:
            tl.push_mark(k * 0.1, "beat")
            tl.push_mark(k * 0.1 + 0.05, "fire", "BOUN")
    tl.push_mark(3.0, "rej", "cool")
    tl.draw(surf, time_rect, now=6.0)

    out = tmp_path / "viz.png"
    pygame.image.save(surf, str(out))
    assert out.stat().st_size > 3000        # a real image, not a blank frame
    pygame.quit()
