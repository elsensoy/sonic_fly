"""A toy 2D drone driven by transmitter-channel state.

Side view: x = lateral, y = altitude. This is *not* flight physics - it is a
readable caricature whose only job is "does the music -> command mapping look
coherent in real time." It consumes the same channel state (`P T U D L R F B`)
that `FakeArduino` / the real transmitter sees, so what you watch here is what
the drone would receive.

Model: the real drone's own stabilisation holds it at hover; our pulses are
*accents* on top. So each axis is a damped spring back to hover (0), and a
channel pulse is a push away from it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# per-channel influence (accel units are arbitrary, tuned to look right)
_THRUST = 6.0          # U / D  -> vertical accel
_LATERAL = 6.5         # L / R  -> lateral accel
_DEPTH_TILT = 0.5      # F / B  -> pitch-tilt for the sprite
_CENTER_SPRING = 5.0   # pull x back toward centre so sways return
_Y_SPRING = 5.0        # pull altitude back toward the hover line
_DRAG = 3.3


@dataclass
class DroneState:
    x: float = 0.0        # -1..1 lateral (0 = centre)
    y: float = 0.0        # -1..1 altitude offset from hover (0 = hover line)
    vx: float = 0.0
    vy: float = 0.0
    tilt: float = 0.0     # radians, visual pitch from F/B and lateral motion
    yaw: float = 0.0      # radians, accumulated from L/R imbalance
    thrust: float = 0.0   # 0..1 smoothed, for rotor-blur


class Drone2D:
    def __init__(self) -> None:
        self.s = DroneState()

    def step(self, dt: float, ch: dict[str, bool]) -> DroneState:
        dt = min(dt, 0.05)
        s = self.s

        up = _THRUST * (bool(ch.get("U")) - bool(ch.get("D")))
        lat = _LATERAL * (bool(ch.get("R")) - bool(ch.get("L")))
        depth = bool(ch.get("F")) - bool(ch.get("B"))

        ay = up - _Y_SPRING * s.y - _DRAG * s.vy
        ax = lat - _CENTER_SPRING * s.x - _DRAG * s.vx

        s.vy += ay * dt
        s.vx += ax * dt
        s.y = _clamp(s.y + s.vy * dt, -1.0, 1.0)
        s.x = _clamp(s.x + s.vx * dt, -1.2, 1.2)

        target_tilt = -0.35 * s.vx + _DEPTH_TILT * depth
        s.tilt += (target_tilt - s.tilt) * min(1.0, 12 * dt)
        s.yaw += 0.9 * (bool(ch.get("R")) - bool(ch.get("L"))) * dt
        s.yaw *= math.exp(-3.0 * dt)          # yaw wobble decays back

        base = 0.45 if not ch.get("P") else 0.25
        want = min(1.0, base + 0.55 * bool(ch.get("U")) + 0.3 * bool(ch.get("T")))
        s.thrust += (want - s.thrust) * min(1.0, 8 * dt)
        return s

    # -- rendering ---------------------------------------------------
    def draw(self, surf, rect, active: dict[str, bool]) -> None:
        import pygame

        cx = rect.x + rect.w * (0.5 + 0.32 * self.s.x)
        cy = rect.y + rect.h * (0.55 - 0.42 * self.s.y)
        span = rect.h * 0.14
        col = (90, 200, 235)
        accent = (250, 210, 90)

        # hover reference line
        hy = rect.y + rect.h * 0.55
        pygame.draw.line(surf, (60, 66, 84), (rect.x + 12, hy), (rect.right - 12, hy), 1)

        cos, sin = math.cos(self.s.tilt), math.sin(self.s.tilt)

        def rot(dx, dy):
            return (cx + dx * cos - dy * sin, cy + dx * sin + dy * cos)

        # arms
        l, r = rot(-span, 0), rot(span, 0)
        pygame.draw.line(surf, col, l, r, 4)
        # body
        pygame.draw.circle(surf, col, (int(cx), int(cy)), max(4, int(span * 0.28)))
        # rotors (blur grows with thrust)
        blur = int(6 + 14 * self.s.thrust)
        for px, py in (l, r):
            pygame.draw.ellipse(surf, accent if self.s.thrust > 0.05 else (70, 78, 96),
                                pygame.Rect(px - blur, py - 3, blur * 2, 6), 2)

        # direction arrows, lit when the channel is asserted
        self._arrows(surf, pygame, rect, active)

    def _arrows(self, surf, pygame, rect, active) -> None:
        cx, cy = rect.centerx, rect.centery
        d = rect.h * 0.40
        specs = {
            "U": ((cx, cy - d), "^"), "D": ((cx, cy + d), "v"),
            "L": ((cx - d, cy), "<"), "R": ((cx + d, cy), ">"),
        }
        font = _font(int(rect.h * 0.09))
        for k, (pos, glyph) in specs.items():
            on = bool(active.get(k))
            c = (250, 210, 90) if on else (70, 78, 96)
            surf.blit(font.render(glyph, True, c), font.render(glyph, True, c).get_rect(center=pos))


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


_FONTS: dict[int, object] = {}


def _font(size: int):
    import pygame

    if size not in _FONTS:
        _FONTS[size] = pygame.font.SysFont("monospace", size, bold=True)
    return _FONTS[size]
