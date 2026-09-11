"""Motion primitive library.

Perception never commands channels directly - it emits a `MotionCommand`
(a primitive name + an intensity). `expand()` turns that into a bounded list
of `ChannelOp`s: timed pulses on the transmitter channels (`P T U D L R F B`,
see docs/fire_at_t_protocol.md), each relative to the command's start time.

A controller - simulated or serial - is what actually schedules those ops.
Keeping primitives as data (not controller calls) means the same command
drives the 2D sim and the real drone with no branching.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable


@dataclass(frozen=True)
class ChannelOp:
    act: str          # one of P T U D L R F B
    at_ms: int        # offset from command start
    dur_ms: int       # pulse width


@dataclass(frozen=True)
class MotionCommand:
    primitive: str
    intensity: float = 0.6      # 0..1, scales pulse widths
    duration_ms: int = 0        # overall envelope; filled in by expand() if 0

    def clamp(self) -> "MotionCommand":
        return MotionCommand(self.primitive, min(max(self.intensity, 0.0), 1.0),
                             self.duration_ms)


# intensity -> pulse width helper
def _w(intensity: float, lo: int, hi: int) -> int:
    return int(round(lo + (hi - lo) * min(max(intensity, 0.0), 1.0)))


# each builder: intensity -> list[ChannelOp]
Builder = Callable[[float], list[ChannelOp]]


def _hold(i: float) -> list[ChannelOp]:
    return []


def _bounce(i: float) -> list[ChannelOp]:
    """Rounded up-down bob - the SWAY-family beat response."""
    up = _w(i, 55, 130)
    return [ChannelOp("U", 0, up), ChannelOp("D", up + 25, _w(i, 35, 80))]


def _bounce_hard(i: float) -> list[ChannelOp]:
    """Short punchy up-hit - the SLAM/PULSE beat response. Brief, high, no
    counter-pulse (the drone's own stabilisation recovers)."""
    return [ChannelOp("U", 0, _w(i, 30, 55))]


def _rise(i: float) -> list[ChannelOp]:
    return [ChannelOp("U", 0, _w(i, 90, 220))]


def _drop(i: float) -> list[ChannelOp]:
    return [ChannelOp("D", 0, _w(i, 80, 200))]


def _sway_left(i: float) -> list[ChannelOp]:
    return [ChannelOp("L", 0, _w(i, 50, 130))]


def _sway_right(i: float) -> list[ChannelOp]:
    return [ChannelOp("R", 0, _w(i, 50, 130))]


def _yaw_twitch(i: float) -> list[ChannelOp]:
    d = _w(i, 30, 70)
    return [ChannelOp("R", 0, d), ChannelOp("L", d + 15, d)]


def _dip(i: float) -> list[ChannelOp]:
    # quick down-accent then recover: a bass hit
    dn = _w(i, 50, 120)
    return [ChannelOp("D", 0, dn), ChannelOp("U", dn + 15, _w(i, 30, 80))]


@dataclass(frozen=True)
class PrimitiveSpec:
    name: str
    build: Builder
    cooldown_ms: int
    nominal_ms: int
    conflicts: frozenset[str] = field(default_factory=frozenset)  # other primitives it excludes


_UD = frozenset({"BOUNCE", "BOUNCE_HARD", "RISE", "DROP", "DIP"})   # all share U/D

PRIMITIVES: dict[str, PrimitiveSpec] = {
    "HOLD":        PrimitiveSpec("HOLD",        _hold,          0,   0),
    "BOUNCE":      PrimitiveSpec("BOUNCE",      _bounce,      140, 200, _UD),
    "BOUNCE_HARD": PrimitiveSpec("BOUNCE_HARD", _bounce_hard,  90,  60, _UD),
    "RISE":        PrimitiveSpec("RISE",        _rise,        220, 220, _UD),
    "DROP":        PrimitiveSpec("DROP",        _drop,        260, 220, _UD),
    "DIP":         PrimitiveSpec("DIP",         _dip,         150, 170, _UD),
    "SWAY_LEFT":   PrimitiveSpec("SWAY_LEFT",   _sway_left,   200, 130, frozenset({"SWAY_RIGHT", "YAW_TWITCH"})),
    "SWAY_RIGHT":  PrimitiveSpec("SWAY_RIGHT",  _sway_right,  200, 130, frozenset({"SWAY_LEFT", "YAW_TWITCH"})),
    "YAW_TWITCH":  PrimitiveSpec("YAW_TWITCH",  _yaw_twitch,  260, 120, frozenset({"SWAY_LEFT", "SWAY_RIGHT"})),
}

PRIMITIVE_NAMES = tuple(PRIMITIVES)

# transmitter-contact names, for the rehearsal readout (channels are P T U D L R F B)
ACT_NAMES: dict[str, str] = {
    "P": "POWER",       "T": "TAKEOFF",
    "U": "THROTTLE_UP", "D": "THROTTLE_DOWN",
    "L": "YAW_LEFT",    "R": "YAW_RIGHT",
    "F": "PITCH_FWD",   "B": "PITCH_BACK",
}

# coarse directional axis a primitive drifts on, for the "no long run in one
# direction" check. Primitives that self-cancel (bob / dip) are not tracked.
DIR_AXIS: dict[str, str] = {
    "SWAY_LEFT": "yaw_l", "SWAY_RIGHT": "yaw_r", "YAW_TWITCH": "yaw",
    "RISE": "up",
}


def expand(cmd: MotionCommand) -> tuple[list[ChannelOp], int]:
    """MotionCommand -> (channel ops, envelope duration in ms)."""
    spec = PRIMITIVES.get(cmd.primitive)
    if spec is None:
        raise KeyError(f"unknown primitive {cmd.primitive!r}")
    ops = spec.build(min(max(cmd.intensity, 0.0), 1.0))
    envelope = cmd.duration_ms or max((o.at_ms + o.dur_ms for o in ops), default=spec.nominal_ms)
    return ops, envelope
