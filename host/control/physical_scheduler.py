"""Physical feasibility layer between the policy and the wire.

    MotionPolicy.candidate()  ->  MotionCommand           (what the music suggests)
                                    │
                              PhysicalScheduler.submit()   (what the drone can do)
                              ├─ minimum inter-command interval  (accent / priority shrinks it)
                              ├─ per-primitive cooldown
                              ├─ mutually-exclusive / still-running directions
                              ├─ maximum command duration  (stuck-stick guard)
                              ├─ maximum consecutive emits on one axis  (anti-drift)
                              └─ priority (section RISE/DROP always try first)
                                    │
                              ScheduledController -> Arduino

A toy drone driven through discrete transmitter contacts cannot act on every
beat. This drops the ones it can't, keeps a per-reason tally, and lets strong
onsets and section events through.

The firmware runs its *own* gate (`tryArm` in transmitter_controller.ino); this
is the host-side pre-filter so most drops never hit the wire, and so we can
account for them (`stats`, `report()`).
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, field

from host.control.motion_primitives import DIR_AXIS, PRIMITIVES, MotionCommand, expand

# priority: 0 = ordinary beat, 1 = strong accent, 2 = section event (RISE/DROP)
PRIO_SECTION = 2
PRIO_ACCENT = 1
PRIO_ORDINARY = 0

_FORCE_PRIMS = ("RISE", "DROP")


@dataclass
class Decision:
    emit: bool
    reason: str = ""            # "" when emitted, else the drop reason


@dataclass
class PhysicalScheduler:
    min_interval_ms: int = 400          # ~150 cmd/min ceiling for the SWAY/PULSE families
    slam_interval_ms: int = 650         # SLAM is punchier per hit, so pace it wider (~92 cmd/min)
    accent_strength: float = 0.55       # onset strength that counts as an accent
    priority_relief: float = 0.4        # accents/section events may fire this fraction of min_interval
    max_duration_ms: int = 320          # reject a primitive whose envelope runs longer (stuck-stick guard)
    max_run: int = 4                    # max consecutive emits on one directional axis (0 = off)

    _last_emit: float = float("-inf")
    _last_emit_primitive: str = ""
    _last_primitive: dict[str, float] = field(default_factory=dict)
    _running_until: dict[str, float] = field(default_factory=dict)   # primitive -> release time
    _run_axis: str = ""
    _run_len: int = 0
    stats: Counter = field(default_factory=Counter)

    def priority_of(self, cmd: MotionCommand, strength: float) -> int:
        if cmd.primitive in _FORCE_PRIMS:
            return PRIO_SECTION
        return PRIO_ACCENT if strength >= self.accent_strength else PRIO_ORDINARY

    def submit(self, cmd: MotionCommand, now: float, *, strength: float = 0.0,
               family: str = "") -> Decision:
        """Try to emit `cmd` at time `now`. Records the outcome in `stats`.

        `family` (the motion family the command came from) selects the base
        inter-command interval - SLAM gets a wider one so a percussive track
        doesn't machine-gun the transmitter.
        """
        self.stats["candidate"] += 1
        spec = PRIMITIVES.get(cmd.primitive)
        if spec is None or spec.name == "HOLD":
            return self._drop("invalid")

        prio = self.priority_of(cmd, strength)
        base_ms = self.slam_interval_ms if family == "SLAM" else self.min_interval_ms
        eff_min = base_ms / 1000.0
        if prio == PRIO_ACCENT:
            eff_min *= self.priority_relief
        elif prio == PRIO_SECTION:                       # near-guaranteed, tiny hard floor
            eff_min = min(0.08, eff_min * self.priority_relief)
        if now - self._last_emit < eff_min:
            return self._drop("min_interval")

        last = self._last_primitive.get(cmd.primitive)
        if last is not None and (now - last) * 1000.0 < spec.cooldown_ms:
            return self._drop("cooldown")

        for other, until in self._running_until.items():
            if until > now and (other in spec.conflicts or cmd.primitive in PRIMITIVES[other].conflicts):
                return self._drop("conflict")

        envelope = expand(cmd)[1]
        if envelope > self.max_duration_ms:
            return self._drop("too_long")

        axis = DIR_AXIS.get(cmd.primitive, "")
        if (self.max_run and axis and axis == self._run_axis
                and self._run_len >= self.max_run and prio < PRIO_SECTION):
            return self._drop("repeat")

        self._last_emit = now
        self._last_emit_primitive = cmd.primitive
        self._last_primitive[cmd.primitive] = now
        self._running_until[cmd.primitive] = now + envelope / 1000.0
        if axis and axis == self._run_axis:
            self._run_len += 1
        elif axis:
            self._run_axis, self._run_len = axis, 1
        else:                                            # self-cancelling move: run broken
            self._run_axis, self._run_len = "", 0
        self.stats["emitted"] += 1
        return Decision(True)

    def _drop(self, reason: str) -> Decision:
        self.stats[reason] += 1
        return Decision(False, reason)

    # -- live introspection (rehearsal readout) --------------------
    def cooldown_remaining_ms(self, primitive: str, now: float) -> float:
        """ms left on `primitive`'s own cooldown (0 if ready or unknown)."""
        last = self._last_primitive.get(primitive)
        spec = PRIMITIVES.get(primitive)
        if last is None or spec is None:
            return 0.0
        return max(0.0, spec.cooldown_ms - (now - last) * 1000.0)

    def gate_remaining_ms(self, now: float) -> float:
        """ms until an ordinary command could next clear the min-interval gate."""
        return max(0.0, self.min_interval_ms - (now - self._last_emit) * 1000.0)

    @property
    def last_emitted_primitive(self) -> str:
        return self._last_emit_primitive

    def running(self, now: float) -> list[str]:
        """primitives whose channel pulses are still in flight."""
        return [p for p, until in list(self._running_until.items()) if until > now]

    def is_neutral(self, now: float) -> bool:
        """True when nothing is driving a stick - the safe default state."""
        return not self.running(now)

    # -- reporting -------------------------------------------------
    @property
    def candidate(self) -> int:
        return self.stats["candidate"]

    @property
    def emitted(self) -> int:
        return self.stats["emitted"]

    @property
    def dropped(self) -> int:
        return self.candidate - self.emitted

    def drop_reasons(self) -> dict[str, int]:
        return {k: v for k, v in self.stats.items()
                if k not in ("candidate", "emitted") and v}

    def report(self, per_min: float | None = None) -> str:
        c, e = self.candidate, self.emitted
        rate = f"  ({per_min:.0f}/min in)" if per_min else ""
        pct = f" ({e / c:.0%})" if c else ""
        reasons = "  ".join(f"{k} {v}" for k, v in sorted(self.drop_reasons().items(),
                                                          key=lambda kv: -kv[1]))
        return (f"candidate {c}{rate}   emitted {e}{pct}   dropped {self.dropped}"
                + (f"   ({reasons})" if reasons else ""))
