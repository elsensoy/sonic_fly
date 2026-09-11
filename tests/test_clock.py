from host.control.clock_sync import ClockModel, Probe, estimate
from host.sim.fake_arduino import _reached, _sdiff


def test_sdiff_wraps():
    assert _sdiff(10, 5) == 5
    assert _sdiff(5, 10) == -5
    # near the uint32 wrap point
    assert _sdiff(2, 0xFFFFFFFF) == 3
    assert _sdiff(0xFFFFFFFF, 2) == -3
    assert _reached(2, 0xFFFFFFFF) is True
    assert _reached(0xFFFFFFFE, 0xFFFFFFFF) is False


def test_estimate_picks_lowest_rtt():
    # true offset: arduino is 1000.0 ms ahead of host*1000
    def probe(t0, rtt):
        t1 = t0 + rtt
        arduino_ms = 0.5 * (t0 + t1) * 1000.0 + 1000.0
        return Probe(t0, t1, int(round(arduino_ms)))

    probes = [probe(100.0, 0.05), probe(100.1, 0.004), probe(100.2, 0.03)]
    m = estimate(probes)
    assert abs(m.offset_ms - 1000.0) < 1.0
    assert m.rtt_ms < 5.0


def test_clock_model_maps_both_ways():
    m = ClockModel(offset_ms=1234.0, rtt_ms=2.0, t_estimated=0.0)
    a = m.to_arduino_ms(50.0)
    assert abs(m.to_host(a) - 50.0) < 0.002
