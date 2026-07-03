"""Analytic Delta kinematics -- pure numpy, no Isaac dependency.

Standard 3-DOF parallel Delta (the Olsson / Trossen derivation): three actuated
revolute joints at the base drive three upper arms; the parallelogram forearms
constrain the moving platform to *pure translation* (no rotation). That is P4's
closed-chain structure reduced to its solvable closed form:

  - ``ik`` : Cartesian TCP (base frame, mm) -> the three actuated joint angles
    (rad). Each arm is solved independently in its own plane, rotated 0/120/240
    deg, by intersecting the driven-arm circle with the forearm sphere.
  - ``fk`` : the three joint angles -> Cartesian TCP (mm). The moving platform
    centre is the common intersection of three spheres (one per forearm).

``ik`` and ``fk`` are exact inverses (to floating-point), which is why this
module is the *desk-verifiable* half of eval 1: the kinematic **math** is proven
here in CI, and ``isaac_plant.py`` measures the residual the **physics** adds on
the rigged PhysX articulation. If the rig ever disagrees with ``fk`` by more than
the eval-1 tolerance, the fault is in the rigging (loose parallelogram, too few
solver iterations), not the math.

Units: geometry and Cartesian coordinates in **mm**; joint angles in **radians**.
No Isaac / omni import here on purpose -- this must import and test on the laptop.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

# geometry constants (fixed trigonometry of the 120-deg-symmetric arms)
_SQRT3 = math.sqrt(3.0)
_SIN30 = 0.5
_TAN30 = 1.0 / _SQRT3
_TAN60 = _SQRT3
_SIN120 = _SQRT3 / 2.0
_COS120 = -0.5


class Unreachable(ValueError):
    """Raised when a Cartesian target lies outside the Delta workspace (the
    driven-arm circle and forearm sphere do not intersect), or a joint triple
    does not close onto a platform pose. Callers on the rig should clamp to the
    workspace before commanding; this mirrors a real controller rejecting an
    out-of-envelope move rather than driving into a singularity."""


@dataclass(frozen=True)
class DeltaGeom:
    """Delta geometry, all in mm.

    f  : side length of the fixed (base) equilateral triangle
    e  : side length of the moving (platform) equilateral triangle
    rf : upper-arm length (the driven link, motor -> elbow)
    re : forearm length (the parallelogram link, elbow -> platform)
    """
    f: float
    e: float
    rf: float
    re: float


# A plausible ABB IRB 360-class FlexPicker footprint. Only the *ratios* matter
# for the IK/FK inverse proof; the rig replaces these with the asset's real
# geometry (kept in sync so delta_ik.fk stays a valid oracle -- eval 1).
DEFAULT_DELTA_GEOM = DeltaGeom(f=567.0, e=76.0, rf=270.0, re=800.0)


def _angle_yz(geom: DeltaGeom, x0: float, y0: float, z0: float) -> float:
    """Solve one arm lying in the YZ plane: intersect the driven-arm circle
    (radius rf about the base hinge) with the forearm sphere (radius re about
    the platform attach point). Returns the actuated angle in radians."""
    f, e, rf, re = geom.f, geom.e, geom.rf, geom.re
    y1 = -0.5 * _TAN30 * f          # base hinge y (this arm)
    y0 = y0 - 0.5 * _TAN30 * e      # shift platform centre to this arm's attach
    # z = a + b*y is the line of forearm-sphere centres consistent with re
    a = (x0 * x0 + y0 * y0 + z0 * z0 + rf * rf - re * re - y1 * y1) / (2.0 * z0)
    b = (y1 - y0) / z0
    disc = -(a + b * y1) * (a + b * y1) + rf * (b * b * rf + rf)
    if disc < 0.0:
        raise Unreachable("target outside workspace (no arm/forearm intersection)")
    yj = (y1 - a * b - math.sqrt(disc)) / (b * b + 1.0)   # outer (elbow-out) branch
    zj = a + b * yj
    return math.atan2(-zj, y1 - yj)


def ik(geom: DeltaGeom, xyz_mm) -> tuple[float, float, float]:
    """Inverse kinematics: TCP (x, y, z) in mm -> (theta1, theta2, theta3) rad.

    The three arms sit 120 deg apart, so arms 2 and 3 are solved by rotating the
    target into each arm's plane. Raises ``Unreachable`` if the point is outside
    the workspace."""
    x0, y0, z0 = (float(v) for v in xyz_mm)
    t1 = _angle_yz(geom, x0, y0, z0)
    t2 = _angle_yz(geom, x0 * _COS120 + y0 * _SIN120, y0 * _COS120 - x0 * _SIN120, z0)
    t3 = _angle_yz(geom, x0 * _COS120 - y0 * _SIN120, y0 * _COS120 + x0 * _SIN120, z0)
    return (t1, t2, t3)


def fk(geom: DeltaGeom, thetas) -> tuple[float, float, float]:
    """Forward kinematics: (theta1, theta2, theta3) rad -> TCP (x, y, z) mm.

    The platform centre is the point at forearm distance ``re`` from all three
    elbow positions -- a three-sphere intersection, taken on the lower (z<0)
    solution. Raises ``Unreachable`` if the joint triple does not close."""
    t1, t2, t3 = (float(v) for v in thetas)
    f, e, rf, re = geom.f, geom.e, geom.rf, geom.re
    t = (f - e) * _TAN30 / 2.0

    y1 = -(t + rf * math.cos(t1))
    z1 = -rf * math.sin(t1)

    y2 = (t + rf * math.cos(t2)) * _SIN30
    x2 = y2 * _TAN60
    z2 = -rf * math.sin(t2)

    y3 = (t + rf * math.cos(t3)) * _SIN30
    x3 = -y3 * _TAN60
    z3 = -rf * math.sin(t3)

    dnm = (y2 - y1) * x3 - (y3 - y1) * x2

    w1 = y1 * y1 + z1 * z1
    w2 = x2 * x2 + y2 * y2 + z2 * z2
    w3 = x3 * x3 + y3 * y3 + z3 * z3

    a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
    b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0

    a2 = -(z2 - z1) * x3 + (z3 - z1) * x2
    b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0

    a = a1 * a1 + a2 * a2 + dnm * dnm
    b = 2.0 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm)
    c = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - re * re)

    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        raise Unreachable("joint triple does not close onto a platform pose")

    z0 = -0.5 * (b + math.sqrt(disc)) / a
    x0 = (a1 * z0 + b1) / dnm
    y0 = (a2 * z0 + b2) / dnm
    return (x0, y0, z0)
