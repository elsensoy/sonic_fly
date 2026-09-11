"""Host-side drone abstraction.

The rest of the code should call methods here, never write serial bytes
directly. Each method maps to a single-character command in the
host-to-Arduino protocol (see README > Host-to-Arduino Protocol):

    P power/wake   T takeoff      U throttle up   D throttle down
    L left         R right        F forward       B backward
    S stop/release

`duration_ms` is honoured host-side for now: send the command, wait, send S.
Timing enforcement will move onto the Arduino as the state machine grows.
"""

from __future__ import annotations

import time

try:
    import serial  # pyserial
except ImportError:  # keep import-time failure friendly for the scaffold
    serial = None

BAUD = 115_200
BOOT_DELAY_S = 2.0  # Arduino resets on port open


class DroneController:
    def __init__(self, port: str, baud: int = BAUD) -> None:
        if serial is None:
            raise ImportError("pyserial not installed; `pip install -r requirements.txt`")
        self.port = port
        self.baud = baud
        self._ser: "serial.Serial | None" = None

    def __enter__(self) -> "DroneController":
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def open(self) -> None:
        self._ser = serial.Serial(self.port, self.baud, timeout=1)
        time.sleep(BOOT_DELAY_S)

    def close(self) -> None:
        if self._ser is not None:
            self.stop()
            self._ser.close()
            self._ser = None

    # -- low level --------------------------------------------------------
    def _send(self, cmd: str) -> None:
        assert self._ser is not None, "call open() first"
        assert len(cmd) == 1
        self._ser.write(cmd.encode("ascii"))

    def _pulse(self, cmd: str, duration_ms: int) -> None:
        self._send(cmd)
        time.sleep(duration_ms / 1000)
        self._send("S")

    # -- protocol --------------------------------------------------------
    def power(self) -> None:            self._send("P")
    def takeoff(self) -> None:          self._send("T")
    def stop(self) -> None:             self._send("S")
    def hover(self) -> None:            self._send("S")

    def throttle_up(self, duration_ms: int = 100) -> None:   self._pulse("U", duration_ms)
    def throttle_down(self, duration_ms: int = 100) -> None: self._pulse("D", duration_ms)
    def yaw_left(self, duration_ms: int = 80) -> None:       self._pulse("L", duration_ms)
    def yaw_right(self, duration_ms: int = 80) -> None:      self._pulse("R", duration_ms)
    def forward(self, duration_ms: int = 80) -> None:        self._pulse("F", duration_ms)
    def backward(self, duration_ms: int = 80) -> None:       self._pulse("B", duration_ms)
