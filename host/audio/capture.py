"""Microphone acquisition.

Yields fixed-size, mono, float32 frames (range roughly [-1, 1]) to the rest of
the pipeline. This is the only module that knows about the sound backend.

Two backends:

  * ``sounddevice`` - low latency, preferred. Needs the PortAudio system lib
    (``sudo apt install libportaudio2``) plus ``pip install sounddevice``.
  * ``arecord``     - shells out to ALSA's ``arecord``. No extra install on a
    normal Linux desktop. Slightly higher latency, fine for bring-up.

``backend="auto"`` (the default) uses sounddevice if it imports, else arecord.
"""

from __future__ import annotations

import queue
import shutil
import subprocess
import time
from dataclasses import dataclass
from typing import Iterator

import numpy as np

DEFAULT_SAMPLE_RATE = 16_000          # plenty for beat/onset work; matches the drone's DMIC16kHz
DEFAULT_BLOCK_SIZE = 512              # 32 ms at 16 kHz


@dataclass
class AudioFrame:
    """One block of mono audio plus the monotonic time its capture started."""

    samples: np.ndarray  # float32, shape (block_size,), ~[-1, 1]
    t_capture: float      # seconds, time.monotonic() at block start

    @property
    def rms(self) -> float:
        return float(np.sqrt(np.mean(self.samples.astype(np.float64) ** 2)) + 1e-12)


class AudioCapture:
    def __init__(
        self,
        sample_rate: int = DEFAULT_SAMPLE_RATE,
        block_size: int = DEFAULT_BLOCK_SIZE,
        device: int | str | None = None,
        backend: str = "auto",
    ) -> None:
        self.sample_rate = sample_rate
        self.block_size = block_size
        self.device = device
        self.backend = _resolve_backend(backend)

    # -- public ----------------------------------------------------------
    def frames(self) -> Iterator[AudioFrame]:
        """Yield AudioFrames until the caller stops iterating (or Ctrl-C)."""
        if self.backend == "sounddevice":
            yield from self._frames_sounddevice()
        else:
            yield from self._frames_arecord()

    @staticmethod
    def list_devices() -> str:
        try:
            import sounddevice as sd

            return str(sd.query_devices())
        except Exception:
            out = subprocess.run(["arecord", "-l"], capture_output=True, text=True)
            return out.stdout + out.stderr

    # -- sounddevice ---------------------------------------------------------
    def _frames_sounddevice(self) -> Iterator[AudioFrame]:
        import sounddevice as sd

        q: "queue.Queue[tuple[np.ndarray, float]]" = queue.Queue(maxsize=32)

        def cb(indata, frames, time_info, status):  # noqa: ARG001 - sd signature
            q.put((indata[:, 0].copy(), time.monotonic()))

        with sd.InputStream(
            samplerate=self.sample_rate,
            blocksize=self.block_size,
            channels=1,
            dtype="float32",
            device=self.device,
            callback=cb,
        ):
            while True:
                samples, t = q.get()
                yield AudioFrame(samples=samples, t_capture=t)

    # -- arecord -----------------------------------------------------------
    def _frames_arecord(self) -> Iterator[AudioFrame]:
        dev = str(self.device) if self.device is not None else "default"
        cmd = [
            "arecord", "-q",
            "-D", dev,
            "-f", "S16_LE",
            "-c", "1",
            "-r", str(self.sample_rate),
            "-t", "raw",
        ]
        nbytes = self.block_size * 2
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
        try:
            assert proc.stdout is not None
            while True:
                buf = proc.stdout.read(nbytes)
                t = time.monotonic()
                if not buf or len(buf) < nbytes:
                    break
                samples = np.frombuffer(buf, dtype="<i2").astype(np.float32) / 32768.0
                yield AudioFrame(samples=samples, t_capture=t)
        finally:
            proc.terminate()
            try:
                proc.wait(timeout=1)
            except subprocess.TimeoutExpired:
                proc.kill()


def _resolve_backend(backend: str) -> str:
    if backend == "auto":
        try:
            import sounddevice  # noqa: F401

            return "sounddevice"
        except Exception:
            if shutil.which("arecord"):
                return "arecord"
            raise RuntimeError(
                "no audio backend: install PortAudio+sounddevice, or ALSA's arecord"
            )
    if backend not in ("sounddevice", "arecord"):
        raise ValueError(f"unknown backend {backend!r}")
    return backend


if __name__ == "__main__":
    print(AudioCapture.list_devices())
