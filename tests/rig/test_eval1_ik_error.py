"""Eval 1 (P3, P4) -- 0.5 mm IK error on the rigged PhysX Delta.

Rig-only. Commands a grid of known Cartesian targets, lets the rigged
articulation settle, reads back the *physical* TCP, and asserts the P95 error is
under 0.5 mm. A mis-rigged loop (loose parallelogram, too few solver iterations)
shows up here as > 0.5 mm and fails the gate -- which is the whole point: this
gate must pass before any downstream result is trusted.

The IK *math* is proven separately in ``tests/test_delta_ik.py`` (CI); this test
measures only what the *physics* adds on top.
"""
import math
import os

import numpy as np
import pytest

pytestmark = pytest.mark.rig

from deltahil.plant.delta_ik import DEFAULT_DELTA_GEOM, Unreachable, fk
from deltahil.plant.isaac_plant import IsaacPlant


def _reachable_targets(geom, n_per_axis=6):
    """Cartesian targets known-reachable (generated via fk from a joint grid)."""
    targets = []
    for a in np.linspace(math.radians(0), math.radians(45), n_per_axis):
        for b in np.linspace(math.radians(0), math.radians(45), n_per_axis):
            for c in np.linspace(math.radians(0), math.radians(45), n_per_axis):
                try:
                    targets.append(fk(geom, (a, b, c)))
                except Unreachable:
                    continue
    return targets


def test_ik_error_p95_under_half_mm(delta_usd):
    geom = DEFAULT_DELTA_GEOM
    plant = IsaacPlant(delta_usd, headless=True, geom=geom, grasp_mode="ideal")
    errs = []
    for tgt in _reachable_targets(geom):
        plant.apply_commands({"cmd.target_xyz": tuple(tgt), "cmd.grip": False})
        for _ in range(250):                       # ~1 s of settling at 250 Hz
            plant.step(0.004)
        tcp = plant.read_sensors()["sensor.tcp_xyz"]
        errs.append(float(np.linalg.norm(np.asarray(tcp) - np.asarray(tgt))))
    p95 = float(np.percentile(errs, 95))
    print(f"eval1: n={len(errs)} p95={p95:.4f} mm max={max(errs):.4f} mm")
    assert p95 < 0.5, f"eval 1 FAIL: P95 IK error {p95:.4f} mm >= 0.5 mm"
