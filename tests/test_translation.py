"""Command translation engine: features -> music state -> motion command."""

import numpy as np
import pytest

from host.audio.capture import AudioFrame
from host.audio.features import AudioFeatures, FeatureExtractor
from host.control.motion_policy import MotionPolicy
from host.control.motion_primitives import MotionCommand, expand
from host.control.music_state import MusicState, MusicStateEstimator


def feat(t, *, beat=False, kind="", energy=0.1, bass=0.1, mid=0.1, high=0.1,
         building=0.0, transient=False, onset=0.0, bpm=120.0) -> AudioFeatures:
    if beat and not kind:
        kind = "beat"
    return AudioFeatures(t=t, beat=beat, onset_strength=onset, kind=kind, bpm=bpm,
                         energy=energy, bass=bass, mid=mid, high=high,
                         building=building, transient=transient)


# ---- primitives ------------------------------------------------------


def test_expand_bounce_is_bounded_and_ordered():
    ops, env = expand(MotionCommand("BOUNCE", intensity=0.7))
    assert [o.act for o in ops] == ["U", "D"]
    assert ops[1].at_ms >= ops[0].at_ms + ops[0].dur_ms
    assert 0 < env <= 400


def test_expand_hold_is_empty():
    ops, env = expand(MotionCommand("HOLD"))
    assert ops == [] and env >= 0


def test_intensity_scales_pulse_width():
    soft, _ = expand(MotionCommand("RISE", intensity=0.0))
    hard, _ = expand(MotionCommand("RISE", intensity=1.0))
    assert hard[0].dur_ms > soft[0].dur_ms


def test_expand_unknown_primitive():
    with pytest.raises(KeyError):
        expand(MotionCommand("MOONWALK"))


# ---- music state ----------------------------------------------------


def test_calm_when_quiet_and_beatless():
    est = MusicStateEstimator()
    for k in range(10):
        s = est.update(feat(k * 0.1, energy=0.05))
    assert s is MusicState.CALM


def test_energetic_when_loud():
    est = MusicStateEstimator(switch_frames=3)
    s = None
    for k in range(30):
        s = est.update(feat(k * 0.1, energy=0.9, beat=(k % 4 == 0)))
    assert s is MusicState.ENERGETIC


def test_building_on_rising_energy():
    est = MusicStateEstimator()
    s = None
    for k in range(25):
        s = est.update(feat(k * 0.1, energy=min(0.5, 0.05 + k * 0.02), building=0.5))
    assert s is MusicState.BUILDING


def test_rhythmic_with_steady_beats_mid_energy():
    est = MusicStateEstimator()
    s = None
    for k in range(40):
        s = est.update(feat(k * 0.1, energy=0.45, beat=(k % 3 == 0), building=0.0))
    assert s is MusicState.RHYTHMIC


def test_transient_overrides_without_changing_held_state():
    est = MusicStateEstimator()
    for k in range(20):
        est.update(feat(k * 0.1, energy=0.45, beat=(k % 3 == 0)))
    held = est.state
    out = est.update(feat(2.1, energy=0.45, transient=True))
    assert out is MusicState.TRANSIENT
    assert est.state is held           # held classification unchanged


def test_hysteresis_needs_repeats_to_switch():
    est = MusicStateEstimator(switch_frames=3)
    for k in range(10):
        est.update(feat(k * 0.1, energy=0.05))
    assert est.state is MusicState.CALM
    est.update(feat(1.1, energy=0.9))          # one loud frame
    assert est.state is MusicState.CALM        # not enough to switch yet


# ---- policy --------------------------------------------------------


def test_semantic_family_grid():
    from host.control.motion_policy import _FAMILIES, _family
    assert _family("percussive", "high") == "SLAM"
    assert _family("percussive", "mid") == "PULSE"
    assert _family("melodic", "mid") == "SWAY"
    assert _family("melodic", "low") == "DRIFT"
    assert _family("sparse", "low") == "HOLD"
    assert _family("unknown", "unknown") == "SWAY"      # neutral row/col
    assert all(_family(t, e) in _FAMILIES
               for t in ("percussive", "melodic", "sparse", "unknown")
               for e in ("low", "mid", "high", "unknown"))


def test_semantic_policy_families_produce_different_motion():
    from host.audio.audio_model import MusicalState
    from host.control.motion_policy import MotionPolicy

    def run_state(texture, energy, mood=""):
        p = MotionPolicy()
        ms = MusicalState(t=0.0, intensity=0.6, texture=texture, section_change=False,
                          energy=energy, mood=mood, seq=1)
        return [
            (p.decide(feat(k * 0.5, beat=True, energy=0.6), MusicState.RHYTHMIC, musical=ms) or
             MotionCommand("NONE")).primitive
            for k in range(8)
        ]

    slam = run_state("percussive", "high")
    hold = run_state("sparse", "low")
    assert any(p in ("DIP", "BOUNCE_HARD", "YAW_TWITCH") for p in slam)
    assert "SWAY_LEFT" not in slam and "BOUNCE" not in slam   # SLAM has its own palette
    assert hold.count("NONE") >= 5                             # HOLD family barely moves
    assert slam != hold


def test_transient_maps_to_yaw_twitch():
    p = MotionPolicy()
    cmd = p.decide(feat(0.0, high=0.8, transient=True), MusicState.TRANSIENT)
    assert cmd and cmd.primitive == "YAW_TWITCH"


def test_energy_drop_maps_to_drop():
    p = MotionPolicy()
    for k in range(80):
        p.decide(feat(k * 0.03, energy=0.9), MusicState.ENERGETIC)    # sustained high -> armed
    cmds = [p.decide(feat(3.0 + k * 0.03, energy=0.05), MusicState.RHYTHMIC) for k in range(150)]
    drops = [c for c in cmds if c and c.primitive == "DROP"]
    assert len(drops) == 1                                            # fires once, on the fall
    # does not re-fire on per-beat flicker without a real recovery first
    more = [p.decide(feat(8.0 + k * 0.03, energy=0.05), MusicState.RHYTHMIC) for k in range(100)]
    assert not any(c and c.primitive == "DROP" for c in more)


def test_rhythmic_alternates_sway():
    p = MotionPolicy()
    got = []
    for k in range(8):
        c = p.decide(feat(k * 0.5, beat=True, energy=0.5), MusicState.RHYTHMIC)
        got.append(c.primitive if c else None)
    sways = [g for g in got if g and g.startswith("SWAY")]
    assert "SWAY_LEFT" in sways and "SWAY_RIGHT" in sways


def test_calm_is_restrained():
    p = MotionPolicy()
    # a quiet-but-audible passage (above the silence gate, below calm_energy)
    fired = [p.decide(feat(k * 0.5, beat=True, energy=0.18), MusicState.CALM) for k in range(8)]
    non_none = [c for c in fired if c is not None]
    assert 0 < len(non_none) <= 2               # only occasional


def test_calm_silence_produces_nothing():
    p = MotionPolicy()
    fired = [p.decide(feat(k * 0.5, beat=True, energy=0.02), MusicState.CALM) for k in range(8)]
    assert all(c is None for c in fired)


def test_energetic_intensity_exceeds_building():
    p1, p2 = MotionPolicy(), MotionPolicy()
    b = p1.decide(feat(0.0, beat=True, energy=0.5), MusicState.BUILDING)
    e = p2.decide(feat(0.0, beat=True, energy=0.5), MusicState.ENERGETIC)
    assert e.intensity > b.intensity


def test_select_throttles_rapid_beats():
    p = MotionPolicy()
    out = [p.select(feat(k * 0.02, beat=True, energy=0.6), MusicState.ENERGETIC) for k in range(20)]
    fired = [c for c in out if c is not None]
    assert 0 < len(fired) < 20                        # the physical scheduler thinned the stream
    assert p.scheduler.candidate >= p.scheduler.emitted + 5


# ---- feature extractor smoke ------------------------------------


def _frames(sig: np.ndarray, sr: int, block: int):
    for i in range(0, len(sig) - block, block):
        yield AudioFrame(sig[i:i + block].astype(np.float32), i / sr)


def test_feature_extractor_on_click_track():
    sr, block = 16000, 512
    n = sr * 4
    sig = (0.003 * np.random.randn(n)).astype(np.float32)
    click = (np.sin(2 * np.pi * 1800 * np.arange(int(0.02 * sr)) / sr)
             * np.hanning(int(0.02 * sr))).astype(np.float32)
    for k in range(8):
        sig[int(k * 0.5 * sr):int(k * 0.5 * sr) + click.size] += click * 0.6

    fx = FeatureExtractor(sr, block)
    feats = [fx.push(fr) for fr in _frames(sig, sr, block)]

    assert any(f.beat for f in feats)
    assert max(f.energy for f in feats) > 0.5
    assert all(0.0 <= f.energy <= 1.0 for f in feats)
