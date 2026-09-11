"""Telemetry + scrolling-timeline panels for the visualizer.

`draw_telemetry` is a stateless render of the rehearsal readout: model state,
timing, the policy's candidate, and - after the PhysicalScheduler - the exact
transmitter-contact stream (or the rejection + reason).  `draw_command_log`
scrolls the accepted / rejected decisions.  `Timeline` keeps a rolling history
of energy + event marks so you can see perception, prediction and fire line up.
"""

from __future__ import annotations

from collections import deque

from host.control.motion_primitives import ACT_NAMES

BG = (18, 20, 28)
PANEL = (24, 27, 38)
GRID = (44, 49, 66)
INK = (210, 216, 230)
DIM = (120, 128, 148)

STATE_COLOR = {
    "calm": (110, 140, 200),
    "building": (90, 200, 190),
    "rhythmic": (120, 210, 120),
    "energetic": (245, 170, 70),
    "transient": (230, 120, 220),
}
FIRE = (245, 210, 90)
REJECT = (210, 90, 90)
BEATC = (150, 160, 190)


def _font(size, bold=False, _c={}):
    import pygame

    key = (size, bold)
    if key not in _c:
        _c[key] = pygame.font.SysFont("monospace", size, bold=bold)
    return _c[key]


def _bar(surf, pygame, rect, frac, color, label=""):
    frac = 0.0 if frac != frac else max(0.0, min(1.0, frac))
    pygame.draw.rect(surf, GRID, rect, 1)
    inner = pygame.Rect(rect.x + 1, rect.y + 1, int((rect.w - 2) * frac), rect.h - 2)
    pygame.draw.rect(surf, color, inner)
    if label:
        surf.blit(_font(rect.h - 4).render(label, True, INK), (rect.x + 4, rect.y - 1))


HEAD = (150, 160, 190)
OKC = (90, 200, 140)
WARN = (235, 170, 90)


def draw_telemetry(surf, rect, *, t, bpm, beat, musical, family, primitive,
                   intensity, decision, dec_age, sched, now,
                   model_ms=None, model_stale_ms=None) -> None:
    """The rehearsal readout: model / timing / policy / hardware command / cooldown.

    `decision` is the last scheduler outcome to display: `("emit", ops, env)`,
    `("reject", reason)`, or None. `dec_age` is seconds since it happened.
    `now` / `t` are on the audio-feature clock (the scheduler's timebase).
    """
    import pygame

    pygame.draw.rect(surf, PANEL, rect)
    x, y = rect.x + 14, rect.y + 10
    sm = _font(13)
    lh = 16

    def head(label: str) -> None:
        nonlocal y
        surf.blit(_font(13, True).render(label, True, HEAD), (x, y))
        y += lh + 2

    def kv(key: str, val: str, vc=INK) -> None:
        nonlocal y
        surf.blit(sm.render(key, True, DIM), (x, y))
        surf.blit(sm.render(val, True, vc), (x + 118, y))
        y += lh

    surf.blit(_font(22, True).render(f"{t:.1f} s", True, INK), (x, y))
    y += 30

    m = musical
    head("MODEL")
    if m is not None:
        conf_c = INK if m.confidence >= 0.5 else WARN
        kv("texture", m.texture or "-")
        kv("energy", m.energy or "-")
        kv("density", m.density_class or "-")
        kv("confidence", f"{m.confidence:.2f}", conf_c)
        if model_ms is not None:
            s = f"infer {model_ms:.0f} ms"
            if model_stale_ms is not None:
                s += f"   stale {model_stale_ms:.0f} ms"
            surf.blit(sm.render(s, True, DIM), (x, y)); y += lh
    else:
        kv("(model off)", "")
    y += 4

    head("TIMING")
    kv("beat", "YES" if beat else "no", FIRE if beat else DIM)
    kv("BPM", f"{bpm:.0f}" if bpm else "--")
    kv("novelty", f"{m.novelty:.2f}" if m else "-")
    y += 4

    head("POLICY")
    kv("family", family)
    kv("primitive", primitive or "-")
    kv("intensity", f"{intensity:.2f}" if intensity is not None else "-")
    y += 4

    head("HARDWARE COMMAND")
    fresh = decision is not None and dec_age is not None and dec_age < 1.5
    if fresh and decision[0] == "reject":
        surf.blit(sm.render(f"candidate  {primitive}", True, DIM), (x, y)); y += lh
        surf.blit(_font(13, True).render(f"REJECTED   {decision[1]}", True, REJECT), (x, y)); y += lh + 2
    elif fresh and decision[0] == "emit":
        for op in decision[1][:4]:
            surf.blit(sm.render(ACT_NAMES.get(op.act, op.act), True, INK), (x + 6, y))
            surf.blit(sm.render(f"{op.dur_ms} ms", True, DIM), (x + 150, y))
            y += lh
        surf.blit(sm.render(f"envelope   {decision[2]} ms", True, DIM), (x, y)); y += lh
    else:
        running = sched.running(now)
        if running:
            surf.blit(sm.render(f"running    {' '.join(running)}", True, FIRE), (x, y)); y += lh
        else:
            surf.blit(sm.render("(neutral)", True, DIM), (x, y)); y += lh
    y += 4

    head("COOLDOWN")
    g = sched.gate_remaining_ms(now)
    kv("gate", f"{g:.0f} ms remaining" if g > 1 else "ready", REJECT if g > 1 else OKC)
    lp = sched.last_emitted_primitive
    if lp:
        c = sched.cooldown_remaining_ms(lp, now)
        kv(lp.lower(), f"{c:.0f} ms remaining" if c > 1 else "ready", REJECT if c > 1 else OKC)
    kv("sticks", "neutral" if sched.is_neutral(now) else "driving",
       OKC if sched.is_neutral(now) else FIRE)


def draw_command_log(surf, rect, entries) -> None:
    """Scrolling list of scheduler outcomes: `(text, color)` newest last."""
    import pygame

    pygame.draw.rect(surf, PANEL, rect)
    x, y = rect.x + 14, rect.y + 8
    surf.blit(_font(13, True).render("COMMAND STREAM  (after PhysicalScheduler)", True, HEAD), (x, y))
    y += 18
    sm = _font(13)
    rows = max(1, (rect.bottom - y - 4) // 15)
    for text, col in list(entries)[-rows:]:
        surf.blit(sm.render(text, True, col), (x, y))
        y += 15


class Timeline:
    def __init__(self, window_s: float = 8.0) -> None:
        self.window_s = window_s
        self.samples: deque[tuple[float, float]] = deque()
        self.marks: deque[tuple[float, str, str]] = deque()   # (t, kind, text)

    def push_sample(self, t: float, energy: float) -> None:
        self.samples.append((t, energy))

    def push_mark(self, t: float, kind: str, text: str = "") -> None:
        self.marks.append((t, kind, text))

    def _prune(self, now: float) -> None:
        lo = now - self.window_s - 0.5
        while self.samples and self.samples[0][0] < lo:
            self.samples.popleft()
        while self.marks and self.marks[0][0] < lo:
            self.marks.popleft()

    def draw(self, surf, rect, now: float) -> None:
        import pygame

        self._prune(now)
        pygame.draw.rect(surf, PANEL, rect)
        left, right = rect.x + 8, rect.right - 8
        span = right - left
        t0 = now - self.window_s

        def px(t: float) -> int:
            return int(left + span * (t - t0) / self.window_s)

        # lanes
        energy_h = rect.h * 0.5
        e_top = rect.y + 8
        e_bot = e_top + energy_h
        lane_beat = e_bot + 16
        lane_prim = lane_beat + 22
        for gy in (e_bot, lane_beat, lane_prim):
            pygame.draw.line(surf, GRID, (left, gy), (right, gy), 1)
        surf.blit(_font(12).render("energy", True, DIM), (left, e_top - 2))
        surf.blit(_font(12).render("beat", True, DIM), (left, lane_beat - 12))
        surf.blit(_font(12).render("fire", True, DIM), (left, lane_prim - 12))

        # energy trace
        pts = [(px(t), e_bot - e * energy_h) for t, e in self.samples if t >= t0]
        if len(pts) > 1:
            pygame.draw.lines(surf, (90, 200, 190), False, pts, 2)

        last_label_x = -999
        row = 0
        for t, kind, text in self.marks:
            if t < t0:
                continue
            X = px(t)
            if kind == "beat":
                pygame.draw.line(surf, BEATC, (X, lane_beat - 8), (X, lane_beat + 4), 2)
            elif kind == "section":
                pygame.draw.line(surf, (230, 120, 220), (X, rect.y + 2), (X, rect.bottom - 2), 2)
                if text:
                    surf.blit(_font(11).render(text, True, (230, 120, 220)), (X + 3, rect.y + 2))
            elif kind == "predict":
                pygame.draw.line(surf, (80, 105, 140), (X, e_top), (X, lane_beat), 1)
            elif kind in ("fire", "rej"):
                c = FIRE if kind == "fire" else REJECT
                pygame.draw.circle(surf, c, (X, lane_prim), 4)
                if text:
                    row = (row + 1) % 2 if X - last_label_x < 44 else 0
                    surf.blit(_font(11).render(text[:3], True, c), (X + 5, lane_prim - 7 + row * 11))
                    last_label_x = X

        # "now" cursor
        pygame.draw.line(surf, (80, 88, 110), (right, rect.y + 4), (right, rect.bottom - 4), 1)
