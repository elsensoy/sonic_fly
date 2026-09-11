"""Characterise the embedded acoustic detector from its CSV telemetry.

Input: a capture produced by
    arduino-cli monitor -p /dev/ttyUSB0 -c baudrate=115200 --timestamp > run.csv

Telemetry columns (see README > Telemetry):
    ms,energy,r1600,r2000,r2400,r2800,best,tone,armed,move,dropped,noise,gate

What this script is for:
  - noise-floor / MIN_ENERGY sizing from quiet sections
  - true analysis period from median(diff(ms)) and the `dropped` slope
  - per-tone detection rate and false-trigger rate against a labelled run
  - ratio-threshold (REL_THRESHOLD) sweeps
"""

from __future__ import annotations

import argparse
from pathlib import Path

COLUMNS = [
    "ms", "energy", "r1600", "r2000", "r2400", "r2800",
    "best", "tone", "armed", "move", "dropped", "noise", "gate",
]


def load(path: Path):
    """Return the telemetry as a structured table (pandas if available)."""
    raise NotImplementedError


def summarize(path: Path) -> None:
    raise NotImplementedError


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", type=Path, default=Path("../run.csv"), nargs="?")
    summarize(ap.parse_args().csv)
