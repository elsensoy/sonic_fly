"""Per-primitive actuation latency L (docs/fire_at_t_protocol.md, README > Temporal Alignment).

L is everything between "the beat should land now" and observable motion:
decision + serial + Arduino parse + MOSFET + RF + stabilisation + mechanical
spin-up. The scheduler compensates by asking the Arduino to fire at
``target - L``.

V0 is a single guessed constant. `observe()` is the hook for
`analysis/latency_analysis.py` to fold in measured values (mic near the drone,
or camera pose) once that experiment runs - each primitive can then carry its
own L, since a throttle pulse and a yaw twitch have different RF/mechanical
response.
"""

from __future__ import annotations

from dataclasses import dataclass, field

DEFAULT_L_S = 0.05


@dataclass
class LatencyModel:
    default_s: float = DEFAULT_L_S
    per_primitive: dict[str, float] = field(default_factory=dict)
    ema: float = 0.2                       # weight on each new observation

    def get(self, primitive: str) -> float:
        return self.per_primitive.get(primitive, self.default_s)

    def observe(self, primitive: str, measured_s: float) -> None:
        """Fold one measured beat-to-motion delay into this primitive's estimate."""
        cur = self.per_primitive.get(primitive, self.default_s)
        self.per_primitive[primitive] = cur + (measured_s - cur) * self.ema

    def as_dict(self) -> dict[str, float]:
        return {"default": self.default_s, **self.per_primitive}
