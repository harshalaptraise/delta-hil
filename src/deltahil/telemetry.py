"""Telemetry -- taps the tag map, runs closed-loop picks, prints the scorecard."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .bridge import Bridge
from .plant.mock_plant import MockPlant
from .plc.mock_plc import MockPLC
from .scenario import Scenario


@dataclass
class LoopStats:
    attempts: int
    successes: int
    mean_latency_ms: float
    jitter_ms: float

    @property
    def success_rate(self):
        return self.successes / self.attempts if self.attempts else 0.0


def run_closed_loop(scenario: Scenario, n_picks: int, calib_offset_xyz=None,
                    grip_tol_mm: float = 0.5) -> LoopStats:
    """Run n_picks full cycles through the real bridge with the mock DUT+plant.

    This is the actual closed loop (P1/P2): the PLC only ever sees sensor tags,
    the plant only ever sees command tags. ``calib_offset_xyz`` is the slow-tier
    correction the PLC applies; None means uncalibrated.
    """
    successes = 0
    fast_meter = None
    for _ in range(n_picks):
        true_xyz, reported, jammed = scenario.sample_part()
        plc = MockPLC()
        plant = MockPlant(grip_tol_mm=grip_tol_mm)
        if calib_offset_xyz is not None:
            plc.set_calibration(calib_offset_xyz)
        plant.set_part(true_xyz, present=not jammed)
        plc.set_target_from_vision(reported)
        bridge = Bridge(plc, plant)
        bridge.run(max_scans=40, until=lambda: plc.done)
        if plc.done:
            successes += 1
        fast_meter = bridge.fast_meter
    lat = fast_meter.summary_ms() if fast_meter else {"mean_ms": 0.0, "jitter_ms": 0.0}
    return LoopStats(n_picks, successes, lat["mean_ms"], lat["jitter_ms"])


def calibration_offset(scenario: Scenario, n_calib: int = 25):
    """Derive the slow-tier correction the PLC should apply: the mean systematic
    displacement Kabsch identifies, expressed as an additive base-frame offset."""
    from .calibration import calibrate
    true_pts, rep_pts = scenario.fixture_points(n_calib)
    res = calibrate(true_pts, rep_pts)
    corrected = res.transform.invert_point(rep_pts)
    return np.mean(corrected - rep_pts, axis=0), res
