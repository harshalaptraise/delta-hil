"""Eval 10 (P5): calibration removes bias, cannot beat the noise floor."""
import numpy as np

from deltahil.calibration import calibrate, noise_reliability_ceiling
from deltahil.evals import run_eval10
from deltahil.scenario import Scenario, default_offset


def _scn(seed):
    return Scenario(vision_offset=default_offset(deg=0.4), pose_sigma_mm=0.15, seed=seed)


def test_bias_driven_below_ik_threshold():
    e = run_eval10(_scn(0))
    assert e.residual_bias_mm < 0.5, e.residual_bias_mm


def test_calibrated_success_reaches_but_not_exceeds_ceiling():
    e = run_eval10(_scn(1))
    assert e.calibrated_success <= e.ceiling + 0.02   # cannot beat variance (P5)
    assert e.calibrated_success >= e.ceiling - 0.05   # but does reach the floor


def test_calibration_actually_helps():
    e = run_eval10(_scn(2))
    assert e.uncalibrated_success < 0.05              # bias dominates uncorrected
    assert e.improves


def test_eval10_passes_across_seeds():
    # P6: statistical, so require it to hold over several seeds, not one trace.
    assert all(run_eval10(_scn(s)).passed for s in range(6))


def test_ceiling_is_a_real_floor_not_one():
    # honesty check: with non-zero noise the ceiling must be < 1.0
    assert noise_reliability_ceiling(0.15, 0.5) < 0.999


def test_zero_noise_recovers_transform_exactly():
    scn = Scenario(vision_offset=default_offset(deg=0.4), pose_sigma_mm=0.0, seed=5)
    true_pts, rep_pts = scn.fixture_points(25)
    res = calibrate(true_pts, rep_pts)
    assert res.residual_bias_mm < 1e-6
    assert np.allclose(res.transform.R, default_offset(deg=0.4).R, atol=1e-6)
