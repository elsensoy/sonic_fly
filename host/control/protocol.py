"""Wire codec for the fire-at-T protocol (docs/fire_at_t_protocol.md).

Line-oriented ASCII, one message per line, space-separated fields, optional
NMEA-style ``*HH`` XOR checksum. This module is transport-agnostic: it turns
strings <-> bytes and parses inbound lines into small typed records.
"""

from __future__ import annotations

from dataclasses import dataclass, field

LINE_MAX = 64


def checksum(body: str) -> str:
    x = 0
    for b in body.encode("ascii"):
        x ^= b
    return f"{x:02X}"


def encode(msg: str, with_checksum: bool = True) -> bytes:
    """`'PING 1'` -> `b'PING 1*<HH>\\n'` (or without the checksum)."""
    if with_checksum:
        msg = f"{msg}*{checksum(msg)}"
    line = msg + "\n"
    if len(line) > LINE_MAX:
        raise ValueError(f"line exceeds {LINE_MAX} bytes: {msg!r}")
    return line.encode("ascii")


def verify_and_strip(line: str) -> tuple[str, bool]:
    """Return (body_without_checksum, checksum_ok). No checksum -> ok=True."""
    line = line.strip()
    if "*" in line:
        body, _, hh = line.rpartition("*")
        return body, checksum(body) == hh.strip().upper()
    return line, True


# ---- inbound records (Arduino -> host) -----------------------------------


@dataclass
class Boot:
    fw: str
    proto: int


@dataclass
class Hello:
    fw: str
    proto: int
    slots: int
    maxpulse: int
    horizon: int
    tick: int


@dataclass
class Pong:
    seq: str
    millis: int


@dataclass
class Ack:
    id: int
    ok: bool
    reason: str = ""


@dataclass
class Fire:
    id: int
    millis: int


@dataclass
class Rel:
    id: int
    millis: int


@dataclass
class Safe:
    reason: str


@dataclass
class Err:
    text: str


@dataclass
class Unknown:
    kind: str
    fields: list[str] = field(default_factory=list)
    raw: str = ""


Inbound = Boot | Hello | Pong | Ack | Fire | Rel | Safe | Err | Unknown


def parse_inbound(body: str) -> Inbound:
    toks = body.split()
    if not toks:
        return Unknown("", [], body)
    kw, rest = toks[0], toks[1:]
    try:
        if kw == "BOOT":
            # BOOT <fw> <proto>
            return Boot(rest[0], int(rest[1]))
        if kw == "HELLO":
            # HELLO <fw> <proto> <slots> <maxpulse> <horizon> <tick>
            return Hello(rest[0], int(rest[1]), int(rest[2]),
                         int(rest[3]), int(rest[4]), int(rest[5]))
        if kw == "PONG":
            return Pong(rest[0], int(rest[1]))
        if kw == "ACK":
            return Ack(int(rest[0]), True)
        if kw == "NAK":
            return Ack(int(rest[0]), False, rest[1] if len(rest) > 1 else "")
        if kw == "FIRE":
            return Fire(int(rest[0]), int(rest[1]))
        if kw == "REL":
            return Rel(int(rest[0]), int(rest[1]))
        if kw == "SAFE":
            return Safe(rest[0] if rest else "")
        if kw == "ERR":
            return Err(" ".join(rest))
    except (IndexError, ValueError):
        return Unknown(kw, rest, body)
    return Unknown(kw, rest, body)
