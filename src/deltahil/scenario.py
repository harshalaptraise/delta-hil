"""Fault injection -- the 'challenging' in challenging pick-and-place.

Constitutional role
-------------------
Splits every disturbance into the two categories P5 cares about:

  systematic  -- ``vision_offset`` (a fixed frame bias). Identifiable, therefore
                 *removable* by calibration.
  stochastic  -- ``pose_sigma`` Gaussian jitter, plus discrete faults (``jam``,
                 ``misfeed``). Unidentifiable, therefore *not* removable. These
                 set the reliability floor no matter how good calibration is.

Reproducibility here is seeded, but per **P6** the closed loop's timing jitter
means run-to-run results are only statistically -- not bit-exactly -- repeatable.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .calibration import FrameTransform


@dataclass
class Scenario:
    vision_offset: FrameTransform  # SYSTEMATIC: removable by calibration
    pose_sigma_mm: float = 0.15    # STOCHASTIC: irreducible noise floor
    jam_rate: float = 0.0          # STOCHASTIC: no part arrives
    misfeed_rate: float = 0.0      # STOCHASTIC: part grossly mislocated
    misfeed_mm: float = 8.0
    workspace_mm: float = 150.0
    seed: int = 0

    def __post_init__(self):
        self._rng = np.random.default_rng(self.seed)

    def sample_part(self):
        """Return (true_xyz, reported_xyz, jammed) for one cycle.

        ``true_xyz``     -- where the part actually is (plant ground truth).
        ``reported_xyz`` -- what the vision system tells the PLC: true pose put
                            through the systematic offset + stochastic noise.
        ``jammed``       -- no part present this cycle.
        """
        true_xyz = self._rng.uniform(-self.workspace_mm, self.workspace_mm, 3)
        if self._rng.random() < self.jam_rate:
            return true_xyz, None, True
        reported = self.vision_offset.apply(true_xyz)
        reported = reported + self._rng.normal(0, self.pose_sigma_mm, 3)
        if self._rng.random() < self.misfeed_rate:
            reported = reported + self._rng.normal(0, self.misfeed_mm, 3)
        return true_xyz, reported, False

    def fixture_points(self, n: int):
        """Known calibration-fixture correspondences (true, reported).

        A fixture has *known* geometry, so the PLC-side sees true points and the
        vision system's reported points -- the input to Kabsch. Noise still
        applies (that is the whole point: calibration sees through bias, not
        noise)."""
        true_pts, rep_pts = [], []
        for _ in range(n):
            p = self._rng.uniform(-self.workspace_mm, self.workspace_mm, 3)
            q = self.vision_offset.apply(p) + self._rng.normal(0, self.pose_sigma_mm, 3)
            true_pts.append(p)
            rep_pts.append(q)
        return np.array(true_pts), np.array(rep_pts)


def default_offset(deg: float = 0.4, t=(3.0, -2.0, 1.5)) -> FrameTransform:
    th = np.radians(deg)
    R = np.array([[np.cos(th), -np.sin(th), 0.0],
                  [np.sin(th), np.cos(th), 0.0],
                  [0.0, 0.0, 1.0]])
    return FrameTransform(R, np.array(t, float))
