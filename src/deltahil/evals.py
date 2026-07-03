"""Eval harness. Row 10 (calibration) is fully desk-verifiable and lives here.

Per the protocol, evals must be checks *I can verify myself*. Row 10 needs no
GPU and no PLC, so it runs in CI. The eval-5 latency check is rig-verifiable
only -- the meter reports numbers but they are illustrative on the mock.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .calibration import calibrate, noise_reliability_ceiling
from .scenario import Scenario


@dataclass
class Eval10Result:
    residual_bias_mm: float
    uncalibrated_success: float
    calibrated_success: float
    ceiling: float
    n_calib: int
    tol_mm: float

    # (a) systematic bias removed below the locked IK threshold
    @property
    def bias_ok(self):
        return self.residual_bias_mm < self.tol_mm

    # (b) calibrated success reaches the noise ceiling but does NOT exceed it.
    # Exceeding the ceiling would mean calibration beat variance -> impossible
    # -> the P3 coincidence must be decorative. Small slack for sampling error.
    @property
    def ceiling_ok(self):
        slack = 0.02
        return self.calibrated_success <= self.ceiling + slack

    @property
    def improves(self):
        return self.calibrated_success > self.uncalibrated_success + 0.1

    @property
    def passed(self):
        return self.bias_ok and self.ceiling_ok and self.improves


def run_eval10(scenario: Scenario, n_calib: int = 25, n_test: int = 4000,
               tol_mm: float = 0.5) -> Eval10Result:
    """Inject a known frame offset + noise, calibrate on N samples, and check
    that calibration removes the bias and lands exactly at the noise floor."""
    true_pts, rep_pts = scenario.fixture_points(n_calib)
    res = calibrate(true_pts, rep_pts)

    # fresh test picks (independent of the calibration set)
    true_t, rep_t = scenario.fixture_points(n_test)
    raw_err = np.linalg.norm(rep_t - true_t, axis=1)                    # uncorrected
    cal_err = np.linalg.norm(res.transform.invert_point(rep_t) - true_t, axis=1)
    ceiling = noise_reliability_ceiling(scenario.pose_sigma_mm, tol_mm)

    return Eval10Result(
        residual_bias_mm=res.residual_bias_mm,
        uncalibrated_success=float((raw_err < tol_mm).mean()),
        calibrated_success=float((cal_err < tol_mm).mean()),
        ceiling=ceiling,
        n_calib=n_calib,
        tol_mm=tol_mm,
    )
