"""End-to-end bring-up test for the laptop side: capture -> detect -> respond.

    mic / wav / synth  -->  BeatDetector  -->  response
                                                 |
                                    terminal meter + ONSET/BEAT markers
                                    (optional) serial command to the Arduino

Examples
--------
Listen on the default mic, print what would be sent (no drone needed):

    python -m host.response_test

Same, but actually pulse the drone's throttle on every beat:

    python -m host.response_test --port /dev/ttyUSB0 --respond throttle

No microphone handy? Feed a synthetic click track and watch it lock tempo:

    python -m host.response_test --source synth --bpm 120

Run a recorded clip through the detector:

    python -m host.response_test --source path/to/clip.wav

List audio devices:

    python -m host.response_test --list-devices
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np

from host.audio.beat_detector import BeatDetector
from host.audio.capture import DEFAULT_BLOCK_SIZE, DEFAULT_SAMPLE_RATE, AudioCapture
from host.audio.sources import open_source

# -- response --------------------------------------------------------------


class Responder:
    """Turns detector events into a visible (and optionally physical) response."""

    def __init__(self, mode: str, port: str | None, min_gap_s: float) -> None:
        self.mode = mode
        self.min_gap_s = min_gap_s
        self._last = 0.0
        self.drone = None
        if port:
            from host.control.drone_controller import DroneController

            self.drone = DroneController(port)
            self.drone.open()
            print(f"[serial] opened {port}")

    def close(self) -> None:
        if self.drone:
            self.drone.close()

    def on_event(self, ev, bpm: float | None) -> None:
        now = time.monotonic()
        if now - self._last < self.min_gap_s:
            return
        self._last = now
        tag = "BEAT" if ev.kind == "beat" else "onset"
        bpm_s = f"{bpm:5.1f} BPM" if bpm else "  -- BPM"
        cmd = {"throttle": "U", "yaw": "L", "none": "-"}[self.mode]
        line = f"  ♪ {tag:5s}  str={ev.strength:0.2f}  {bpm_s}  -> {cmd}"
        print(line, flush=True)
        if self.drone and self.mode == "throttle":
            self.drone.throttle_up(90)
        elif self.drone and self.mode == "yaw":
            self.drone.yaw_left(70)


def _meter(rms: float, width: int = 40) -> str:
    db = 20 * np.log10(rms + 1e-9)
    frac = float(np.clip((db + 60) / 60, 0.0, 1.0))   # -60 dB .. 0 dB
    n = int(frac * width)
    return "[" + "#" * n + "-" * (width - n) + f"] {db:6.1f} dBFS"


# -- main -----------------------------------------------------------------


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--source", default="mic", help="'mic' (default), 'synth', or a .wav path")
    p.add_argument("--backend", default="auto", choices=["auto", "sounddevice", "arecord"])
    p.add_argument("--device", default=None, help="capture device (index or ALSA name)")
    p.add_argument("--rate", type=int, default=DEFAULT_SAMPLE_RATE)
    p.add_argument("--block", type=int, default=DEFAULT_BLOCK_SIZE)
    p.add_argument("--bpm", type=float, default=120.0, help="click tempo for --source synth")
    p.add_argument("--seconds", type=float, default=30.0, help="duration for synth/limit for mic")
    p.add_argument("--port", default=None, help="Arduino serial port; omit for a dry run")
    p.add_argument("--respond", default="throttle", choices=["throttle", "yaw", "none"])
    p.add_argument("--min-gap", type=float, default=0.15, help="min seconds between responses")
    p.add_argument("--list-devices", action="store_true")
    args = p.parse_args(argv)

    if args.list_devices:
        print(AudioCapture.list_devices())
        return 0

    frames, sr, src_desc = open_source(
        args.source, rate=args.rate, block=args.block, backend=args.backend,
        device=args.device, bpm=args.bpm, seconds=args.seconds, profile="click",
    )

    det = BeatDetector(sr, args.block)
    responder = Responder(args.respond, args.port, args.min_gap)

    print(f"source : {src_desc}")
    print(f"rate   : {sr} Hz    block: {args.block} ({1000*args.block/sr:.0f} ms)")
    print(f"respond: {args.respond}{'  (dry run)' if not args.port else ''}")
    print("Ctrl-C to stop.\n")

    t_start = time.monotonic()
    last_draw = 0.0
    try:
        for fr in frames:
            events = det.push(fr)
            for ev in events:
                responder.on_event(ev, det.bpm)
            now = time.monotonic()
            if now - last_draw > 0.05:                      # ~20 fps meter
                bpm_s = f"{det.bpm:5.1f} BPM" if det.bpm else "  --  BPM"
                sys.stdout.write("\r" + _meter(fr.rms) + "  " + bpm_s + "  ")
                sys.stdout.flush()
                last_draw = now
            if args.source == "mic" and now - t_start > args.seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        responder.close()
        print("\ndone.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
