"""Serial transport for the fire-at-T protocol.

A background thread reads lines off the port, verifies the checksum, parses
each into a typed record (protocol.parse_inbound), and pushes it onto a queue.
``send`` is synchronous and thread-safe.
"""

from __future__ import annotations

import queue
import threading
import time

import serial  # pyserial

from host.control import protocol
from host.control.protocol import Inbound

BAUD = 115_200
BOOT_SETTLE_S = 2.0   # the Nano resets when the port opens


class SerialLink:
    def __init__(
        self,
        port: str,
        baud: int = BAUD,
        checksum: bool = True,
        require_checksum: bool = False,
    ) -> None:
        self.port = port
        self.baud = baud
        self.checksum = checksum            # send lines with a *HH suffix
        self.require_checksum = require_checksum  # drop inbound lines that lack one
        self._ser: serial.Serial | None = None
        self._rx: "queue.Queue[Inbound]" = queue.Queue()
        self._reader: threading.Thread | None = None
        self._stop = threading.Event()
        self._wlock = threading.Lock()
        self.bad_checksums = 0

    # -- lifecycle -----------------------------------------------------
    def open(self, settle: bool = True) -> None:
        self._ser = serial.Serial(self.port, self.baud, timeout=0.1)
        if settle:
            time.sleep(BOOT_SETTLE_S)
            self._ser.reset_input_buffer()
        self._stop.clear()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def close(self) -> None:
        self._stop.set()
        if self._reader:
            self._reader.join(timeout=1)
        if self._ser:
            self._ser.close()
            self._ser = None

    def __enter__(self) -> "SerialLink":
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # -- io ----------------------------------------------------------
    def send(self, msg: str) -> None:
        assert self._ser is not None, "open() first"
        data = protocol.encode(msg, with_checksum=self.checksum)
        with self._wlock:
            self._ser.write(data)

    def get(self, timeout: float | None = None) -> Inbound | None:
        try:
            return self._rx.get(timeout=timeout)
        except queue.Empty:
            return None

    def drain(self) -> list[Inbound]:
        out: list[Inbound] = []
        while True:
            try:
                out.append(self._rx.get_nowait())
            except queue.Empty:
                return out

    # -- internal -----------------------------------------------------
    def _read_loop(self) -> None:
        buf = bytearray()
        while not self._stop.is_set():
            try:
                chunk = self._ser.read(256)  # type: ignore[union-attr]
            except (serial.SerialException, OSError):
                break
            if not chunk:
                continue
            buf.extend(chunk)
            while b"\n" in buf:
                raw, _, buf = buf.partition(b"\n")
                text = raw.decode("ascii", "replace").strip()
                if not text:
                    continue
                body, ok = protocol.verify_and_strip(text)
                if not ok or (self.require_checksum and "*" not in text):
                    self.bad_checksums += 1
                    continue
                self._rx.put(protocol.parse_inbound(body))
