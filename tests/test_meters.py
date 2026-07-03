"""RTFMeter unit test -- the arithmetic of evals 3/9, no Isaac needed."""
import time

from deltahil.plant.meters import RTFMeter


def test_empty_meter_is_zero():
    s = RTFMeter().summary()
    assert s["rtf"] == 0.0 and s["fps"] == 0.0 and s["n"] == 0


def test_faster_than_realtime_gives_rtf_above_one():
    m = RTFMeter()
    # advance 1 s of sim-time across 250 ticks, spending far less wall-time
    for _ in range(250):
        m.tick(0.004)
    s = m.summary()
    assert s["n"] == 250
    assert abs(s["sim_s"] - 1.0) < 1e-9
    assert s["rtf"] > 1.0            # a mock loop trivially outruns real time
    assert s["fps"] > 30.0


def test_rtf_drops_below_one_when_wall_exceeds_sim():
    m = RTFMeter()
    for _ in range(5):
        m.tick(0.001)               # 1 ms sim per tick...
        time.sleep(0.01)            # ...but 10 ms wall spent
    s = m.summary()
    assert s["rtf"] < 1.0           # sim fell behind real time
