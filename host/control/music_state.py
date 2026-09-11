"""Musical-state estimator: AudioFeatures -> one of a few coarse states.

Sustained classification (CALM / BUILDING / RHYTHMIC / ENERGETIC) with
hysteresis so it doesn't flicker frame-to-frame, plus an edge-triggered
TRANSIENT that overrides for a single frame on a sharp hit.

This is the deterministic stand-in for the eventual GPU semantic model
(host/audio/audio_model.py) - same output contract, so MotionPolicy doesn't
care which produced it.
"""

from __future__ import annotations

from collections import deque
from enum import Enum

from host.audio.features import AudioFeatures

_BEAT_RATE_WINDOW_S = 4.0


class MusicState(str, Enum):
    CALM = "calm"
    BUILDING = "building"
    RHYTHMIC = "rhythmic"
    ENERGETIC = "energetic"
    TRANSIENT = "transient"


class MusicStateEstimator:
    def __init__(
        self,
        energy_release: float = 0.94,   # per-frame decay of the peak-follower
        switch_frames: int = 4,
        calm_energy: float = 0.30,
        energetic_energy: float = 0.68,
        leave_margin: float = 0.12,     # Schmitt band around the thresholds
    ) -> None:
        self.energy_release = energy_release
        self.switch_frames = switch_frames
        self.calm_energy = calm_energy
        self.energetic_energy = energetic_energy
        self.leave_margin = leave_margin

        self._energy = 0.0
        self._beats: deque[float] = deque()
        self._state = MusicState.CALM
        self._candidate = MusicState.CALM
        self._candidate_count = 0
        self._last_transient = float("-inf")

    @property
    def state(self) -> MusicState:
        return self._state

    def beat_rate(self) -> float:
        return len(self._beats) / _BEAT_RATE_WINDOW_S

    def update(self, f: AudioFeatures) -> MusicState:
        if f.warmup:
            return MusicState.CALM
        # peak-decay follower: sits near the top of the beat spikes, so it
        # tracks "how loud is this passage" rather than the per-beat flicker
        self._energy = max(f.energy, self._energy * self.energy_release)
        if f.beat and f.kind == "beat":
            self._beats.append(f.t)
        while self._beats and f.t - self._beats[0] > _BEAT_RATE_WINDOW_S:
            self._beats.popleft()

        target = self._classify(f)
        if target == self._candidate:
            self._candidate_count += 1
        else:
            self._candidate = target
            self._candidate_count = 1
        if self._candidate_count >= self.switch_frames:
            self._state = self._candidate

        # transient is a brief override, not a state change; rate-limited so a
        # busy hi-hat pattern doesn't mask the sustained state
        if (f.transient and self._state != MusicState.CALM
                and f.t - self._last_transient > 0.4):
            self._last_transient = f.t
            return MusicState.TRANSIENT
        return self._state

    def _classify(self, f: AudioFeatures) -> MusicState:
        e, m = self._energy, self.leave_margin
        in_calm = self._state is MusicState.CALM
        in_energetic = self._state is MusicState.ENERGETIC

        if e < self.calm_energy - (0 if in_calm else m):
            return MusicState.CALM
        if e >= self.energetic_energy - (m if in_energetic else 0):
            return MusicState.ENERGETIC
        if f.building > 0.22:
            return MusicState.BUILDING
        if self.beat_rate() >= 0.8:
            return MusicState.RHYTHMIC
        return MusicState.BUILDING
