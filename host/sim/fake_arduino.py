"""A software stand-in for transmitter_controller.ino.

Speaks the fire-at-T protocol over a pseudo-terminal so the real
``host.control`` stack (pyserial and all) can be exercised with no hardware:

    fa = FakeArduino()
    port = fa.start()                       # e.g. "/dev/pts/7"
    sched = DroneScheduler(SerialLink(port, checksum=True))
    sched.open()  ...
    fa.stop()

It mirrors the firmware's logic and constants; it is a behavioural model, not
a cycle-accurate one (Python timing means edge times are good to a few ms).
"""

from __future__ import annotations

import os
import pty
import threading
import time
import tty

from host.control import protocol

FW_VERSION = "0.2.0-sim"
PROTO_VERSION = 1
SLOTS = 16
MAX_PULSE_MS = 500
MAX_HORIZON_MS = 3000
MIN_LEAD_MS = 5
MAX_CONCURRENT = 3
LINK_TIMEOUT_MS = 500

CHANNELS = {
    "P": None, "T": None,
    "U": "D", "D": "U", "L": "R", "R": "L", "F": "B", "B": "F",
}
COOLDOWN_MS = {
    "P": 1000, "T": 2000,
    "U": 60, "D": 60, "L": 60, "R": 60, "F": 60, "B": 60,
}
LATCHING = ("P", "T")


class _Event:
    __slots__ = ("id", "act", "fire_at", "release_at", "state")

    def __init__(self, eid, act, fire_at, release_at):
        self.id = eid
        self.act = act
        self.fire_at = fire_at
        self.release_at = release_at
        self.state = "armed"          # armed -> asserted -> (freed)


class FakeArduino:
    def __init__(self, emit_checksum: bool = True, require_checksum: bool = False) -> None:
        self.emit_checksum = emit_checksum
        self.require_checksum = require_checksum
        self._boot = time.monotonic()
        self.armed = False
        self.safe_hold = False
        self.events: list[_Event] = []
        self.pins: dict[str, bool] = {a: False for a in CHANNELS}
        self.last_fire: dict[str, int] = {a: 0 for a in CHANNELS}
        self.last_valid_ms = 0
        self._mfd = -1
        self._sfd = -1
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        self._wlock = threading.Lock()
        self.log: list[str] = []       # every line we emit, for test assertions

    # -- lifecycle -------------------------------------------------------
    def millis(self) -> int:
        return int((time.monotonic() - self._boot) * 1000) & 0xFFFFFFFF

    def start(self) -> str:
        self._mfd, self._sfd = pty.openpty()
        tty.setraw(self._sfd)
        self.last_valid_ms = self.millis()
        self._emit(f"BOOT {FW_VERSION} {PROTO_VERSION}")
        for target in (self._read_loop, self._tick_loop):
            t = threading.Thread(target=target, daemon=True)
            t.start()
            self._threads.append(t)
        return os.ttyname(self._sfd)

    def stop(self) -> None:
        self._stop.set()
        for t in self._threads:
            t.join(timeout=1)
        for fd in (self._mfd, self._sfd):
            try:
                os.close(fd)
            except OSError:
                pass

    # -- io -------------------------------------------------------------
    def _emit(self, msg: str) -> None:
        line = protocol.encode(msg, with_checksum=self.emit_checksum)
        self.log.append(msg)
        with self._wlock:
            try:
                os.write(self._mfd, line)
            except OSError:
                pass

    def _read_loop(self) -> None:
        buf = bytearray()
        while not self._stop.is_set():
            try:
                chunk = os.read(self._mfd, 256)
            except OSError:
                break
            if not chunk:
                break
            buf.extend(chunk)
            while b"\n" in buf:
                raw, _, buf = buf.partition(b"\n")
                text = raw.decode("ascii", "replace").strip()
                if text:
                    self._dispatch(text)

    # -- scheduler tick -------------------------------------------------
    def _tick_loop(self) -> None:
        while not self._stop.is_set():
            now = self.millis()
            for e in list(self.events):
                if e.state == "armed" and _reached(now, e.fire_at):
                    self.pins[e.act] = True
                    self.last_fire[e.act] = now
                    e.state = "asserted"
                    self._emit(f"FIRE {e.id} {now}")
                if e.state == "asserted" and _reached(now, e.release_at):
                    self.pins[e.act] = False
                    self.events.remove(e)
                    self._emit(f"REL {e.id} {now}")
            if not self.safe_hold and now - self.last_valid_ms > LINK_TIMEOUT_MS:
                self._release_transient()
                self.safe_hold = True
                self._emit("SAFE linkloss")
            time.sleep(0.0003)

    # -- command handlers (mirror the .ino) ---------------------------
    def _dispatch(self, text: str) -> None:
        body, ok = protocol.verify_and_strip(text)
        if not ok or (self.require_checksum and "*" not in text):
            self._emit("ERR checksum")
            return
        self.last_valid_ms = self.millis()
        self.safe_hold = False
        toks = body.split()
        if not toks:
            return
        kw, args = toks[0], toks[1:]
        {
            "HELLO": self._h_hello, "PING": self._h_ping, "ARM": self._h_arm,
            "DISARM": self._h_disarm, "SCHED": self._h_sched, "NOW": self._h_now,
            "CANCEL": self._h_cancel, "ABORT": self._h_abort,
        }.get(kw, lambda a: self._emit(f"ERR unknown {kw}"))(args)

    def _h_hello(self, args) -> None:
        self._emit(
            f"HELLO {FW_VERSION} {PROTO_VERSION} {SLOTS} "
            f"{MAX_PULSE_MS} {MAX_HORIZON_MS} 1000"
        )

    def _h_ping(self, args) -> None:
        self._emit(f"PONG {args[0] if args else '0'} {self.millis()}")

    def _h_arm(self, args) -> None:
        self.armed = True
        self._emit("ACK 0 OK")

    def _h_disarm(self, args) -> None:
        self.armed = False
        self._release_transient()
        self._emit("ACK 0 OK")

    def _h_sched(self, args) -> None:
        if len(args) < 4:
            return self._emit("ERR parse SCHED")
        eid = int(args[0]); at = int(args[1]); act = args[2][:1]; dur = int(args[3])
        if act not in CHANNELS:
            return self._emit(f"NAK {eid} range")
        if not self.armed and act not in LATCHING:
            return self._emit(f"NAK {eid} disarmed")
        if self._try_arm(act, at, min(dur, MAX_PULSE_MS), eid):
            self._emit(f"ACK {eid} OK")

    def _h_now(self, args) -> None:
        if len(args) < 2:
            return self._emit("ERR parse NOW")
        act = args[0][:1]; dur = min(int(args[1]), MAX_PULSE_MS)
        if act not in CHANNELS:
            return self._emit("ERR range NOW")
        if not self.armed and act not in LATCHING:
            return self._emit("ERR disarmed NOW")
        at = (self.millis() + MIN_LEAD_MS) & 0xFFFFFFFF
        self._try_arm(act, at, dur, 0)

    def _try_arm(self, act: str, at: int, dur: int, eid: int) -> bool:
        lead = _sdiff(at, self.millis())
        if lead < MIN_LEAD_MS:
            return bool(self._emit(f"NAK {eid} late"))
        if lead > MAX_HORIZON_MS:
            return bool(self._emit(f"NAK {eid} horizon"))
        release_at = (at + dur) & 0xFFFFFFFF

        ref = self.last_fire[act]
        has_ref = ref != 0
        for e in self.events:
            if e.act == act and (not has_ref or _sdiff(e.fire_at, ref) > 0):
                ref, has_ref = e.fire_at, True
        if has_ref and _sdiff(at, ref) < COOLDOWN_MS[act]:
            return bool(self._emit(f"NAK {eid} cooldown"))

        opp = CHANNELS[act]
        overlap = 0
        for e in self.events:
            if not (_sdiff(at, e.release_at) < 0 and _sdiff(e.fire_at, release_at) < 0):
                continue
            if e.act == act or e.act == opp:
                return bool(self._emit(f"NAK {eid} conflict"))
            overlap += 1
        if overlap + 1 > MAX_CONCURRENT:
            return bool(self._emit(f"NAK {eid} conflict"))
        if len(self.events) >= SLOTS:
            return bool(self._emit(f"NAK {eid} full"))

        self.events.append(_Event(eid, act, at, release_at))
        return True

    def _h_cancel(self, args) -> None:
        if not args:
            return self._emit("ERR parse CANCEL")
        target = args[0]
        for e in list(self.events):
            if e.state != "armed":
                continue
            if target == "all" or (target != "grp" and e.id == int(target)):
                self.events.remove(e)
                self._emit(f"REL {e.id} {self.millis()}")

    def _h_abort(self, args) -> None:
        self.events.clear()
        self._release_transient()
        self._emit("SAFE abort")

    # -- helpers -----------------------------------------------------
    def _release_transient(self) -> None:
        for a in self.pins:
            if a not in LATCHING:
                self.pins[a] = False

    def reboot(self) -> None:
        """Simulate a hardware reset: millis() restarts, state clears, BOOT emitted."""
        self._boot = time.monotonic()
        self.events.clear()
        self.armed = False
        self.safe_hold = False
        self.last_fire = {a: 0 for a in CHANNELS}
        self._release_transient()
        self.last_valid_ms = self.millis()
        self._emit(f"BOOT {FW_VERSION} {PROTO_VERSION}")


def _sdiff(a: int, b: int) -> int:
    """Signed 32-bit (a - b), for wrap-safe time comparison."""
    d = (a - b) & 0xFFFFFFFF
    return d - 0x100000000 if d >= 0x80000000 else d


def _reached(now: int, t: int) -> bool:
    return _sdiff(now, t) >= 0
