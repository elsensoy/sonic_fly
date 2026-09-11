"""Choreography comparison harness + windowed section novelty."""

import numpy as np
import pytest

from analysis.choreo_compare import run_track, summarise
from host.audio.audio_model import AudioModel
from host.audio.capture import AudioFrame

pytest.importorskip("torch")


def test_run_track_produces_family_and_command_streams():
    fams = {"HOLD", "DRIFT", "SWAY", "PULSE", "SLAM"}
    r = run_track("synth", encoder="randproj", seconds=18)
    assert r["fam_windows"] and set(r["fam_windows"]) <= fams
    assert r["sem_cmds"]
    for t, fam, prim, inten in r["sem_cmds"]:
        assert 0.0 <= inten <= 1.0 and fam in fams and t >= 0.0
    assert "family" in summarise(r, baseline=False)


def test_semantic_and_beat_only_policies_differ():
    r = run_track("synth", encoder="randproj", seconds=20, baseline=True)
    sem = [p for _, _, p, _ in r["sem_cmds"]]
    dsp = r["dsp_cmds"]
    assert sem and dsp
    assert sem != dsp                                       # different choreography


def test_windowed_novelty_spikes_at_a_real_change_not_within_a_section():
    sr, block = 16000, 512
    t = np.arange(sr * 24) / sr
    sig = (0.02 * np.random.default_rng(0).standard_normal(sr * 24)).astype(np.float32)
    # section 1: steady low tone;  section 2 (from 12 s): busy broadband noise bursts
    sig[: sr * 12] += (0.3 * np.sin(2 * np.pi * 150 * t[: sr * 12])).astype(np.float32)
    rng = np.random.default_rng(1)
    for k in range(48, 96):
        i = int(k * 0.25 * sr)
        sig[i:i + 1500] += (rng.standard_normal(1500) * np.hanning(1500) * 0.8).astype(np.float32)

    m = AudioModel(sample_rate=sr, device="cpu", backend="torch",
                   section_w_s=3.0, section_threshold=0.15, section_cooldown_s=4.0)
    nov_by_t = []
    for i in range(0, len(sig) - block, block):
        s = m.push(AudioFrame(sig[i:i + block], i / sr))
        if s:
            nov_by_t.append((s.t, s.novelty, s.section_change))

    early = [n for tt, n, _ in nov_by_t if 6 < tt < 11]          # mid section 1
    at_cut = [n for tt, n, _ in nov_by_t if 12 < tt < 17]        # just after the change
    assert max(at_cut) > max(early) + 0.1                        # the change stands out
    cuts = [tt for tt, _, sec in nov_by_t if sec]
    assert any(11 < c < 20 for c in cuts)                        # detected it
    assert not any(c < 10 for c in cuts)                         # no spurious mid-section fire
