from host.control import protocol as p
from host.control.protocol import Ack, Boot, Fire, Hello, Pong, Rel, Safe, Unknown


def test_checksum_roundtrip():
    body = "SCHED 17 40940 U 90 4"
    line = p.encode(body).decode()
    assert line.endswith("\n")
    stripped, ok = p.verify_and_strip(line)
    assert stripped == body and ok


def test_checksum_detects_corruption():
    line = p.encode("PING 1").decode().replace("PING 1", "PING 2")
    _, ok = p.verify_and_strip(line)
    assert not ok


def test_no_checksum_is_accepted():
    body, ok = p.verify_and_strip("FIRE 3 12345")
    assert ok and body == "FIRE 3 12345"


def test_encode_rejects_overlong_line():
    try:
        p.encode("X" * 70)
        assert False, "expected ValueError"
    except ValueError:
        pass


def test_parse_inbound_variants():
    assert parse("BOOT 0.2.0 1") == Boot("0.2.0", 1)
    h = parse("HELLO 0.2.0 1 16 500 3000 1000")
    assert isinstance(h, Hello) and h.slots == 16 and h.horizon == 3000
    assert parse("PONG 5 40232") == Pong("5", 40232)
    assert parse("ACK 17 OK") == Ack(17, True, "")
    assert parse("NAK 18 late") == Ack(18, False, "late")
    assert parse("FIRE 17 40941") == Fire(17, 40941)
    assert parse("REL 17 41031") == Rel(17, 41031)
    assert parse("SAFE linkloss") == Safe("linkloss")
    assert isinstance(parse("WAT is this"), Unknown)


def parse(s: str):
    body, ok = p.verify_and_strip(s)
    assert ok
    return p.parse_inbound(body)
