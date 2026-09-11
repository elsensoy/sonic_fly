"""Host <-> Arduino clock synchronisation (docs/fire_at_t_protocol.md section 5).

SNTP-style: send PING, get PONG with the Arduino's millis(). Two models:

  * ClockModel        - offset only, from the lowest-RTT probe. Good immediately.
  * LinearClockModel  - arduino_ms = a + b * t_host, least-squares fit once
                        probes span enough time to estimate oscillator drift.

`ClockTracker` accumulates probes over a session and hands back whichever model
the data supports.
"""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field

from host.control.link import SerialLink
from host.control.protocol import Pong

MIN_SPAN_FOR_DRIFT_S = 5.0
DEFAULT_WINDOW_S = 120.0
DEFAULT_RESYNC_S = 2.0


@dataclass
class Probe:
    t0: float          # host monotonic seconds, PING sent
    t1: float          # host monotonic seconds, PONG received
    arduino_ms: int    # millis() reported in the PONG

    @property
    def rtt(self) -> float:
        return self.t1 - self.t0

    @property
    def t_mid(self) -> float:
        return 0.5 * (self.t0 + self.t1)

    @property
    def offset_ms(self) -> float:
        # symmetric-latency assumption: arduino clock == arduino_ms at t_mid
        return self.arduino_ms - self.t_mid * 1000.0


@dataclass
class ClockModel:
    offset_ms: float
    rtt_ms: float
    t_estimated: float
    n: int = 1

    def to_arduino_ms(self, t_host: float) -> int:
        return int(round(t_host * 1000.0 + self.offset_ms)) & 0xFFFFFFFF

    def to_host(self, arduino_ms: int) -> float:
        return (arduino_ms - self.offset_ms) / 1000.0

    def now_arduino_ms(self) -> int:
        return self.to_arduino_ms(time.monotonic())

    @property
    def drift_ppm(self) -> float:
        return 0.0            # offset-only model can't estimate drift


@dataclass
class LinearClockModel:
    a_ms: float             # arduino_ms at t_host == 0
    b: float                # arduino ms per host second (~1000)
    rtt_ms: float
    t_estimated: float
    n: int

    def to_arduino_ms(self, t_host: float) -> int:
        return int(round(self.a_ms + self.b * t_host)) & 0xFFFFFFFF

    def to_host(self, arduino_ms: int) -> float:
        return (arduino_ms - self.a_ms) / self.b

    def now_arduino_ms(self) -> int:
        return self.to_arduino_ms(time.monotonic())

    @property
    def offset_ms(self) -> float:
        return self.a_ms + (self.b - 1000.0) * time.monotonic()

    @property
    def drift_ppm(self) -> float:
        return (self.b / 1000.0 - 1.0) * 1e6


Clock = ClockModel | LinearClockModel


def estimate(probes: list[Probe]) -> ClockModel:
    if not probes:
        raise ValueError("no probes")
    best = min(probes, key=lambda p: p.rtt)
    return ClockModel(best.offset_ms, best.rtt * 1000.0, time.monotonic(), len(probes))


def fit_linear(probes: list[Probe]) -> Clock:
    """Least-squares arduino_ms = a + b*t_host over the lowest-RTT probes."""
    if len(probes) < 2:
        return estimate(probes)
    best_rtt = min(p.rtt for p in probes)
    good = [p for p in probes if p.rtt <= best_rtt + 0.010] or list(probes)
    if len(good) < 2:
        good = list(probes)

    x0 = good[0].t_mid
    xs = [p.t_mid - x0 for p in good]
    ys = [float(p.arduino_ms) for p in good]
    n = len(good)
    sx, sy = sum(xs), sum(ys)
    sxx = sum(x * x for x in xs)
    sxy = sum(x * y for x, y in zip(xs, ys))
    denom = n * sxx - sx * sx
    if denom == 0:
        return estimate(probes)
    b = (n * sxy - sx * sy) / denom          # ms per second
    a_at_x0 = (sy - b * sx) / n              # arduino_ms at t_mid == x0
    a_ms = a_at_x0 - b * x0                  # arduino_ms at t_host == 0
    return LinearClockModel(a_ms, b, best_rtt * 1000.0, time.monotonic(), n)


@dataclass
class ClockTracker:
    resync_interval: float = DEFAULT_RESYNC_S
    window: float = DEFAULT_WINDOW_S
    min_span_for_drift: float = MIN_SPAN_FOR_DRIFT_S
    probes: deque[Probe] = field(default_factory=deque)
    _last_sent: float = 0.0

    def mark_sent(self) -> None:
        self._last_sent = time.monotonic()

    def due(self) -> bool:
        return time.monotonic() - self._last_sent >= self.resync_interval

    def add(self, p: Probe) -> None:
        self.probes.append(p)
        cutoff = time.monotonic() - self.window
        while len(self.probes) > 4 and self.probes[0].t_mid < cutoff:
            self.probes.popleft()
        self._last_sent = time.monotonic()

    def reset(self) -> None:
        self.probes.clear()
        self._last_sent = 0.0

    @property
    def span(self) -> float:
        if len(self.probes) < 2:
            return 0.0
        return self.probes[-1].t_mid - self.probes[0].t_mid

    @property
    def model(self) -> Clock | None:
        if not self.probes:
            return None
        if len(self.probes) >= 3 and self.span >= self.min_span_for_drift:
            return fit_linear(list(self.probes))
        return estimate(list(self.probes))


# ---- blocking helpers (fine before anything else is in flight) -----------


def probe_once(link: SerialLink, seq: str, timeout: float = 0.5) -> Probe | None:
    t0 = time.monotonic()
    link.send(f"PING {seq}")
    deadline = t0 + timeout
    while time.monotonic() < deadline:
        msg = link.get(timeout=max(0.0, deadline - time.monotonic()))
        if isinstance(msg, Pong) and msg.seq == seq:
            return Probe(t0, time.monotonic(), msg.millis)
    return None


def sync(link: SerialLink, n: int = 8, gap: float = 0.02, timeout: float = 0.5) -> ClockModel:
    probes: list[Probe] = []
    for i in range(n):
        p = probe_once(link, str(i), timeout)
        if p:
            probes.append(p)
        time.sleep(gap)
    if not probes:
        raise TimeoutError("no PONG received during sync")
    return estimate(probes)
