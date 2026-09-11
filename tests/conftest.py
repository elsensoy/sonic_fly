import time

import pytest

from host.control.link import SerialLink
from host.control.scheduler import DroneScheduler
from host.sim.fake_arduino import FakeArduino


@pytest.fixture
def sched():
    """DroneScheduler wired to a FakeArduino over a pty. Yields (scheduler, fake)."""
    fa = FakeArduino()
    port = fa.start()
    s = DroneScheduler(SerialLink(port, checksum=True))
    s.link.open(settle=False)
    time.sleep(0.05)
    s.link.drain()                       # discard BOOT
    yield s, fa
    s.link.close()
    fa.stop()


def pump_until(s: DroneScheduler, pred, timeout: float = 2.0) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        s.pump()
        if pred():
            return True
        time.sleep(0.005)
    return False
