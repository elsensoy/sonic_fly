"""Command translation engine: (AudioFeatures, MusicState[, MusicalState]) -> MotionCommand.

Rule-based for V0. The mapping lives in `decide()` as a small readable table.
Swapping the rules for a learned policy later means replacing `decide()` only -
the scheduler, the primitives, and everything below (controllers, sim, Arduino)
are untouched.

`musical=None` -> `_decide_dsp` (the beat-only DSP `MusicState` table, the
comparison baseline). `musical` present -> `_decide_semantic`: `_family(texture,
energy)` picks one of five families (HOLD / DRIFT / SWAY / PULSE / SLAM), each
with its own primitive palette and beat density; beat/onset stays the clock.

`select()` then runs the candidate through `PhysicalScheduler`: minimum
inter-command interval (strong accents + section events relax it), per-primitive
cooldown, still-running conflicts, max envelope, max run on one axis. It records
`last_candidate` / `last_decision` so a caller can show what was dropped and why.
"""

from __future__ import annotations

from host.audio.audio_model import MusicalState
from host.audio.features import AudioFeatures
from host.control.motion_primitives import MotionCommand
from host.control.music_state import MusicState
from host.control.physical_scheduler import Decision, PhysicalScheduler


class MotionPolicy:
    def __init__(self, scheduler: PhysicalScheduler | None = None) -> None:
        self.scheduler = scheduler or PhysicalScheduler()
        self._beat_n = 0
        self._sway_left_next = True
        self._energy_s = 0.0        # fast smoothed energy (silence gate)
        self._energy_slow = 0.0    # ~1.5 s smoothed energy (DROP detection)
        self._drop_armed = False    # True once energy has been sustained-high
        self._variant = 0          # choreography variant, flipped on section_change
        self._pending_accent = False  # emit one distinctive move after a section change
        self._seen_section_seq = -1   # dedupe section_change when musical is polled
        # last select() outcome, for the rehearsal readout
        self.last_candidate: MotionCommand | None = None
        self.last_decision: Decision | None = None

    # -- rules ---------------------------------------------------------
    def decide(self, f: AudioFeatures, state: MusicState,
               musical: MusicalState | None = None) -> MotionCommand | None:
        """The mapping, pre-safety-gate. Returns None for 'do nothing'."""
        self._energy_s += (f.energy - self._energy_s) * 0.25
        self._energy_slow += (f.energy - self._energy_slow) * 0.02
        if self._energy_slow > 0.5:
            self._drop_armed = True

        if (musical is not None and musical.section_change
                and musical.seq != self._seen_section_seq):
            self._seen_section_seq = musical.seq
            self._variant ^= 1
            self._beat_n = 0
            self._pending_accent = True

        if state is MusicState.TRANSIENT:
            return MotionCommand("YAW_TWITCH", intensity=_clip(0.4 + 0.6 * f.high))

        if self._drop_armed and self._energy_slow < 0.22:      # a real section fell out
            self._drop_armed = False
            loud = musical is not None and musical.energy == "high"
            return MotionCommand("DROP", intensity=0.85 if loud else 0.5)

        if self._energy_s < 0.12:                              # essentially silence
            return None

        if not f.beat:
            return None
        self._beat_n += 1

        if self._pending_accent:                               # mark the new section
            self._pending_accent = False
            amp = musical.intensity if musical else f.energy
            return MotionCommand("RISE", intensity=_clip(0.5 + 0.4 * amp))

        if musical is None:
            return self._decide_dsp(f, state)                  # beat-only baseline
        return self._decide_semantic(f, musical)

    # -- beat-only (DSP MusicState) --------------------------------
    def _decide_dsp(self, f: AudioFeatures, state: MusicState) -> MotionCommand | None:
        i = _clip(0.25 + 0.7 * f.energy)
        bassy = f.bass > 0.6 and f.bass >= f.high
        off = self._variant

        if state is MusicState.CALM:
            return MotionCommand("BOUNCE", _clip(0.2 + 0.2 * f.energy)) if self._beat_n % 4 == 0 else None
        if state is MusicState.BUILDING:
            if self._beat_n % 8 == 0:
                return MotionCommand("RISE", intensity=_clip(0.4 + 0.5 * f.building))
            return MotionCommand("DIP" if bassy else "BOUNCE", _clip(i * (0.7 + 0.5 * f.building)))
        if state is MusicState.RHYTHMIC:
            if (self._beat_n + off) % 2 == 0:
                return self._next_sway(i)
            return MotionCommand("DIP" if bassy else "BOUNCE", intensity=i)
        if state is MusicState.ENERGETIC:
            if (self._beat_n + off) % 2 == 0:
                return self._next_sway(_clip(i + 0.2))
            return MotionCommand("BOUNCE", intensity=_clip(i + 0.2))
        return None

    # -- semantic (CLAP): family from texture x energy, beat from DSP --------
    # texture -> WHAT kind of movement, energy -> HOW big / how often,
    # beat/onset -> exactly WHEN. Each family has a distinct primitive palette
    # so the choreography reads differently, not just "BOUNCE everywhere".
    # `density` is not trusted yet (see docs).
    def _decide_semantic(self, f: AudioFeatures, m: MusicalState) -> MotionCommand | None:
        fam = _family(m.texture, m.energy)
        e_num = f.energy if m.energy in ("unknown", "") else 0.4 * f.energy + 0.6 * _energy_num(m.energy)
        amp = _clip(0.18 + 0.72 * e_num)
        n, off = self._beat_n, self._variant

        if fam == "HOLD":                                      # near still
            return self._next_sway(_clip(amp * 0.45)) if n % 8 == 0 else None

        if fam == "DRIFT":                                     # gentle, long
            return self._next_sway(_clip(amp * 0.65)) if n % 4 == 0 else None

        if fam == "SWAY":                                      # lateral, occasional soft rise
            if n % 8 == 0:
                return MotionCommand("RISE", intensity=_clip(amp * 0.6))
            if (n + off) % 2 == 0:
                return self._next_sway(amp)
            return MotionCommand("BOUNCE", intensity=_clip(0.35 + 0.25 * e_num))   # soft, rounded

        if fam == "PULSE":                                     # DIP-led, some sway
            if (n + off) % 3 == 0:
                return self._next_sway(_clip(amp + 0.05))
            return MotionCommand("DIP", intensity=amp)

        # SLAM - percussive + high: a punchy hit every beat, twitch accents, no soft bounce
        if n % 4 == 3:
            return MotionCommand("YAW_TWITCH", intensity=_clip(amp + 0.15))
        if n % 2 == 0:
            return MotionCommand("DIP", intensity=_clip(amp + 0.1))
        return MotionCommand("BOUNCE_HARD", intensity=_clip(amp + 0.15))

    def _next_sway(self, intensity: float) -> MotionCommand:
        name = "SWAY_LEFT" if self._sway_left_next else "SWAY_RIGHT"
        self._sway_left_next = not self._sway_left_next
        return MotionCommand(name, intensity=intensity)

    candidate = decide          # alias: decide() is the pre-feasibility candidate

    # -- physically-gated selection --------------------------------
    def select(self, f: AudioFeatures, state: MusicState,
               musical: MusicalState | None = None) -> MotionCommand | None:
        self.last_candidate = None
        self.last_decision = None
        if f.warmup:
            return None
        cmd = self.decide(f, state, musical)
        if cmd is None or cmd.primitive == "HOLD":
            return None
        self.last_candidate = cmd
        fam = _family(musical.texture, musical.energy) if musical is not None else ""
        self.last_decision = self.scheduler.submit(cmd, f.t, strength=f.onset_strength, family=fam)
        return cmd if self.last_decision.emit else None


def _clip(x: float) -> float:
    return float(min(max(x, 0.0), 1.0))


def _energy_num(energy: str) -> float:
    return {"low": 0.2, "mid": 0.55, "high": 0.9}.get(energy, 0.55)


_FAMILIES = ("HOLD", "DRIFT", "SWAY", "PULSE", "SLAM")

# texture x energy -> motion family. 'unknown' on an axis reads as the
# middle/neutral row or column.
_FAMILY_GRID: dict[tuple[str, str], str] = {
    ("percussive", "low"): "PULSE",  ("percussive", "mid"): "PULSE",  ("percussive", "high"): "SLAM",
    ("melodic",    "low"): "DRIFT",  ("melodic",    "mid"): "SWAY",   ("melodic",    "high"): "SWAY",
    ("sparse",     "low"): "HOLD",   ("sparse",     "mid"): "DRIFT",  ("sparse",     "high"): "DRIFT",
}


def _family(texture: str, energy: str) -> str:
    tex = texture if texture in ("percussive", "melodic", "sparse") else "melodic"
    ene = energy if energy in ("low", "mid", "high") else "mid"
    return _FAMILY_GRID[(tex, ene)]
