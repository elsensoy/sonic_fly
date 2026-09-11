"""Semantic-label trace for an audio file (or synth / mic).

    python -m analysis.semantic_trace path/to/track.wav
    python -m analysis.semantic_trace track.wav --encoder randproj
    python -m analysis.semantic_trace synth --seconds 30

Runs the DSP beat detector and the (smoothed) audio model over the audio and
prints, per model window:

    time   beats  texture      energy  density  conf   nov
    2.0      4    percussive   mid     bright   .58

plus a stability summary - how often each label, longest run, switches/min.
The question this answers: *do the semantic labels evolve sensibly over the
track, or do they thrash / stick?* Run it on 5-8 very different clips before
touching the microphone.
"""

from __future__ import annotations

import argparse
import statistics
from collections import Counter

import numpy as np

from host.audio.audio_model import AudioModel
from host.audio.capture import DEFAULT_BLOCK_SIZE, DEFAULT_SAMPLE_RATE
from host.audio.features import FeatureExtractor
from host.audio.sources import open_source


def trace(source: str, *, encoder: str = "clap", rate: int = DEFAULT_SAMPLE_RATE,
          block: int = DEFAULT_BLOCK_SIZE, seconds: float = 60.0, bpm: float = 120.0,
          semantic_alpha: float = 0.4) -> list[dict]:
    frames, sr, desc = open_source(source, rate=rate, block=block, seconds=seconds,
                                   bpm=bpm, realtime=(source == "mic"))
    fx = FeatureExtractor(sr, block)
    enc = None
    if encoder == "clap":
        from host.audio.encoders import ClapEncoder
        enc = ClapEncoder()
    model = AudioModel(sample_rate=sr, encoder=enc, semantic_alpha=semantic_alpha)

    rows: list[dict] = []
    beats_since = 0
    t0: float | None = None
    for af in frames:
        if t0 is None:
            t0 = af.t_capture
        f = fx.push(af)
        if f.beat:
            beats_since += 1
        s = model.push(af)
        if s is None:
            continue
        rows.append({"t": s.t - t0, "beats": beats_since, "texture": s.texture,
                     "energy": s.energy, "density": s.density_class or "-",
                     "conf": s.confidence, "novelty": s.novelty,
                     "section": s.section_change,
                     "raw": s.extras.get("labels_raw", {})})
        beats_since = 0
    return rows


def _fmt_scores(d: dict[str, float]) -> str:
    return "  ".join(f"{k} {v:.2f}" for k, v in sorted(d.items(), key=lambda kv: -kv[1]))


def _runs(seq: list[str]) -> tuple[int, float, int]:
    """(#switches, mean run length, longest run) for a label sequence."""
    if not seq:
        return 0, 0.0, 0
    lengths, cur = [], 1
    for a, b in zip(seq, seq[1:]):
        if a == b:
            cur += 1
        else:
            lengths.append(cur)
            cur = 1
    lengths.append(cur)
    return len(lengths) - 1, statistics.fmean(lengths), max(lengths)


def summarise(rows: list[dict], hop_s: float = 0.5) -> str:
    if not rows:
        return "  (no windows - source too short?)"
    span_min = max(1e-6, (rows[-1]["t"] - rows[0]["t"]) / 60.0)
    out = [f"  {len(rows)} windows over {span_min*60:.0f} s",
           f"  section changes: {sum(r['section'] for r in rows)}"]
    for cat in ("texture", "energy", "density"):
        seq = [r[cat] for r in rows]
        sw, mean_run, longest = _runs(seq)
        dist = ", ".join(f"{k} {v/len(seq):.0%}" for k, v in Counter(seq).most_common())
        out.append(f"  {cat:8s} {dist}")
        out.append(f"           {sw} switches ({sw/span_min:.1f}/min), "
                   f"mean run {mean_run*hop_s:.1f} s, longest {longest*hop_s:.1f} s")
    confs = [r["conf"] for r in rows]
    out.append(f"  confidence  mean {statistics.fmean(confs):.2f}  "
               f"min {min(confs):.2f}  max {max(confs):.2f}")

    nov = np.array([r["novelty"] for r in rows])
    q = np.percentile(nov, [50, 75, 90, 95, 99])
    out.append(f"  novelty  p50 {q[0]:.3f}  p75 {q[1]:.3f}  p90 {q[2]:.3f}  "
               f"p95 {q[3]:.3f}  p99 {q[4]:.3f}  max {nov.max():.3f}")
    top = sorted(rows, key=lambda r: -r["novelty"])[:10]
    out.append("  novelty peaks:  " +
               "   ".join(f"{r['t']:.0f}s {r['novelty']:.3f}" for r in top))
    return "\n".join(out)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("source", nargs="?", default="synth", help="a .wav path, 'synth', or 'mic'")
    ap.add_argument("--encoder", default="clap", choices=["clap", "randproj"])
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--bpm", type=float, default=120.0, help="for --source synth")
    ap.add_argument("--alpha", type=float, default=0.4, help="semantic EMA weight (lower = smoother)")
    ap.add_argument("--raw", action="store_true", help="print raw per-category cosine scores")
    ap.add_argument("--quiet", action="store_true", help="summary only")
    args = ap.parse_args(argv)

    rows = trace(args.source, encoder=args.encoder, seconds=args.seconds,
                 bpm=args.bpm, semantic_alpha=args.alpha)

    if args.raw:
        for r in rows[::4]:
            print(f"t={r['t']:6.1f}")
            for cat, scores in r["raw"].items():
                print(f"  {cat:8s} {_fmt_scores(scores)}")
        print()
    if not args.quiet:
        print(f"{'time':>6}  {'beats':>5}  {'texture':<11} {'energy':<8} {'density':<8} {'conf':>4}  nov")
        for r in rows:
            flag = "  <<< SECTION" if r["section"] else ""
            print(f"{r['t']:6.1f}  {r['beats']:5d}  {r['texture']:<11} {r['energy']:<8} "
                  f"{r['density']:<8} {r['conf']:4.2f}  {r['novelty']:.2f}{flag}")
        print()
    print(summarise(rows))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
