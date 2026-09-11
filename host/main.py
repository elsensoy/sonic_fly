r"""Entry point: wire the pipeline together and run the music-response loop.

    audio capture --> beat detector (timing)      --\
                  \-> audio model  (musical state) --> motion policy --> drone

Run:
    python -m host.main --port /dev/ttyUSB0
"""

from __future__ import annotations

import argparse

from host.audio.audio_model import AudioModel
from host.audio.beat_detector import BeatDetector
from host.audio.capture import AudioCapture
from host.control.drone_controller import DroneController
from host.control.motion_policy import MotionPolicy


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Acoustic physical-AI micro-drone host")
    p.add_argument("--port", required=True, help="Arduino serial port, e.g. /dev/ttyUSB0")
    p.add_argument("--input-device", default=None, help="audio input device index or name")
    p.add_argument("--no-drone", action="store_true", help="run perception only, no serial")
    p.add_argument("--gpu", default="cuda", help="torch device for the audio model")
    return p.parse_args(argv)


def run(args: argparse.Namespace) -> None:
    capture = AudioCapture(device=args.input_device)
    beats = BeatDetector(capture.sample_rate)
    model = AudioModel(device=args.gpu)
    policy = MotionPolicy()

    drone = None if args.no_drone else DroneController(args.port)

    raise NotImplementedError("main loop: pull frames, fan out, select, actuate")


if __name__ == "__main__":
    run(parse_args())
