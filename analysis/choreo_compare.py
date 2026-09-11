"""Run perception -> motion policy over several audio files and compare the
command timelines - the check before flying anything.

    python -m analysis.choreo_compare media/track.wav media/track_2.wav
    python -m analysis.choreo_compare a.wav b.wav c.wav --baseline

The question: does the drone respond *differently* to different music?

    metal            -> SLAM / PULSE, big amplitude, a hit every beat
    ukulele          -> DRIFT / SWAY, moderate, few accents
    quiet->buildup   -> restrained, then stronger as energy confidence rises

If the three command traces are clearly different here, the perception->motion
layer is doing its job.
"""

from __future__ import annotations

import argparse
import statistics
from collections import Counter

from host.audio.audio_model import AudioModel
from host.audio.capture import DEFAULT_BLOCK_SIZE, DEFAULT_SAMPLE_RATE
from host.audio.features import FeatureExtractor
from host.audio.sources import open_source
from host.control.motion_policy import MotionPolicy, _family
from host.control.music_state import MusicStateEstimator

_FAM_CHAR = {"HOLD": ".", "DRIFT": "-", "SWAY": "~", "PULSE": "+", "SLAM": "#"}


def run_track(path: str, *, encoder: str = "clap", seconds: float = 180.0,
              baseline: bool = False) -> dict:
    frames, sr, _ = open_source(path, rate=DEFAULT_SAMPLE_RATE, block=DEFAULT_BLOCK_SIZE,
                                seconds=seconds, realtime=False)
    fx = FeatureExtractor(sr, DEFAULT_BLOCK_SIZE)
    est = MusicStateEstimator()
    enc = None
    if encoder == "clap":
        from host.audio.encoders import ClapEncoder
        enc = ClapEncoder()
    model = AudioModel(sample_rate=sr, encoder=enc)
    sem_pol = MotionPolicy()
    dsp_pol = MotionPolicy() if baseline else None

    musical = None
    fam_windows: list[str] = []            # family per model window
    sem_cmds: list[tuple[float, str, str, float]] = []   # (t, family, primitive, intensity)
    dsp_cmds: list[str] = []
    t0 = None
    for af in frames:
        if t0 is None:
            t0 = af.t_capture
        f = fx.push(af)
        st = est.update(f)
        s = model.push(af)
        if s is not None:
            musical = s
            fam_windows.append(_family(s.texture, s.energy))
        c = sem_pol.select(f, st, musical)
        if c and musical is not None:
            sem_cmds.append((af.t_capture - t0, _family(musical.texture, musical.energy),
                             c.primitive, c.intensity))
        if dsp_pol is not None:
            d = dsp_pol.select(f, st, None)
            if d:
                dsp_cmds.append(d.primitive)

    span = max(1e-6, (frames_span(sem_cmds, fam_windows)) or seconds)
    return {
        "path": path, "span_s": span, "fam_windows": fam_windows,
        "sem_cmds": sem_cmds, "dsp_cmds": dsp_cmds,
        "sched": sem_pol.scheduler,
    }


def frames_span(cmds, fam_windows) -> float:
    if cmds:
        return cmds[-1][0]
    return len(fam_windows) * 0.5


def _hist(items) -> str:
    c = Counter(items)
    n = sum(c.values()) or 1
    return "  ".join(f"{k} {v/n:.0%}" for k, v in c.most_common())


def summarise(r: dict, baseline: bool) -> str:
    span_min = max(1e-6, r["span_s"] / 60.0)
    cmds = r["sem_cmds"]
    out = [f"=== {r['path']}  ({r['span_s']:.0f} s) ==="]
    if r["fam_windows"]:
        out.append(f"  family     {_hist(r['fam_windows'])}")
    if cmds:
        out.append(f"  primitives {_hist(p for _, _, p, _ in cmds)}")
        out.append(f"  rate       {len(cmds)/span_min:.1f} cmd/min    "
                   f"mean intensity {statistics.fmean(i for *_, i in cmds):.2f}")
        # coarse family timeline, one char per ~2 s
        step = max(1, len(r["fam_windows"]) // 80)
        line = "".join(_FAM_CHAR.get(fw, "?") for fw in r["fam_windows"][::step])
        out.append(f"  timeline   {line}")
    else:
        out.append("  (no commands)")
    sched = r.get("sched")
    if sched is not None and sched.candidate:
        out.append(f"  scheduler  {sched.report(per_min=sched.candidate / span_min)}")
    if baseline and r["dsp_cmds"]:
        out.append(f"  beat-only  {_hist(r['dsp_cmds'])}   ({len(r['dsp_cmds'])/span_min:.1f}/min)")
    return "\n".join(out)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("tracks", nargs="+", help="audio files (or 'synth')")
    ap.add_argument("--encoder", default="clap", choices=["clap", "randproj"])
    ap.add_argument("--seconds", type=float, default=180.0)
    ap.add_argument("--baseline", action="store_true", help="also show the beat-only policy")
    args = ap.parse_args(argv)

    results = [run_track(t, encoder=args.encoder, seconds=args.seconds, baseline=args.baseline)
               for t in args.tracks]
    for r in results:
        print(summarise(r, args.baseline))
        print()
    print("legend:  . HOLD   - DRIFT   ~ SWAY   + PULSE   # SLAM")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
