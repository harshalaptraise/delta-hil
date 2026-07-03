"""Procedurally rig a closed-chain Delta as a PhysX articulation (P4).

This is the *fallback* asset source: when no vendor USD (e.g. an ABB IRB 360) is
available, build one from ``DeltaGeom`` so the geometry stays identical to
``delta_ik`` -- which is what lets ``delta_ik.fk`` serve as the eval-1 oracle.

The whole point of this file is the rigging discipline P4 demands. A PhysX
articulation is solved as a reduced-coordinate **tree** and cannot represent a
kinematic loop directly; an ABB-style Delta has three closed parallelogram
chains. So per the official "Rig Closed-Loop Structures" workflow, for each arm:

  base --revolute(MOTOR)--> upper_arm --spherical(guide)--> forearm  ==LOOP==> platform

  * the MOTOR joint is actuated (DriveAPI, angular);
  * the elbow is a passive spherical *guide* joint (no drive, no limits) so it
    adds no resistance to the loop;
  * the forearm->platform joint is the loop-closer, marked
    ``physxArticulation:excludeFromArticulation = True`` -- PhysX then keeps it
    as a maximal-coordinate constraint that closes the loop while the tree solver
    still sees only a tree;
  * on the articulation root we raise ``solverPositionIterationCount`` well above
    the default (4 is far too low for closed loops) for stability.

Gate the result on eval 1 (``tests/rig/test_eval1_ik_error.py``): if the
parallelograms are loose or the solver under-iterates, the readback TCP drifts
past 0.5 mm and the gate fails -- exactly the "gate before trusting downstream"
rule in the checklist.

Runs only inside the Isaac Sim Python runtime (pxr is imported lazily). Nothing
here is imported at module top level beyond stdlib + numpy + delta_ik, so the
laptop can import this module (it just cannot execute ``build_delta``).
"""
from __future__ import annotations

import math

import numpy as np

from ..delta_ik import DEFAULT_DELTA_GEOM, DeltaGeom

# solver iterations for the closed-loop articulation root (default is 4 -- far
# too low for three parallelograms). Raise position iterations most; velocity a
# little. Tune upward on the rig if the loop drifts under eval 1.
SOLVER_POSITION_ITERATIONS = 48
SOLVER_VELOCITY_ITERATIONS = 4

# radius of the actuated links' upward drive at home, used to seed poses (mm)
_ARM_ANGLE_DEG = (0.0, 120.0, 240.0)


def _circumradius(side: float) -> float:
    """Circumradius of an equilateral triangle from its side length."""
    return side / math.sqrt(3.0)


def build_delta(
    stage,
    geom: DeltaGeom = DEFAULT_DELTA_GEOM,
    root_path: str = "/World/Delta",
    *,
    part_path: str = "/World/Part",
    units_per_mm: float = 1e-3,
):
    """Author the Delta articulation + gripper + part onto ``stage``.

    ``stage`` is a ``pxr.Usd.Stage``; ``geom`` is in mm; ``units_per_mm`` scales
    mm -> stage units (1e-3 for a metres stage). Returns the root prim path.

    The joint/DOF names authored here match ``IsaacPlant``'s defaults
    (``motor_0/1/2``, ``/World/Delta/platform/tcp``, ``/World/Part``).
    """
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics

    s = units_per_mm
    base_r = _circumradius(geom.f) * s
    plat_r = _circumradius(geom.e) * s
    rf = geom.rf * s
    re = geom.re * s

    # -- articulation root ---------------------------------------------------
    root = UsdGeom.Xform.Define(stage, root_path)
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
    art = PhysxSchema.PhysxArticulationAPI.Apply(root.GetPrim())
    art.CreateSolverPositionIterationCountAttr(SOLVER_POSITION_ITERATIONS)
    art.CreateSolverVelocityIterationCountAttr(SOLVER_VELOCITY_ITERATIONS)

    # fixed base body (grounded to the world)
    base = _rigid_box(stage, f"{root_path}/base", size=0.06,
                      pos=Gf.Vec3f(0, 0, 0), UsdPhysics=UsdPhysics, UsdGeom=UsdGeom)
    UsdPhysics.FixedJoint.Define(stage, f"{root_path}/base/ground")

    # moving platform (pure translation), hangs one forearm below the base
    plat_z = -math.sqrt(max(re * re - (base_r - plat_r) ** 2, 0.0)) - rf
    platform = _rigid_box(stage, f"{root_path}/platform", size=0.05,
                          pos=Gf.Vec3f(0, 0, plat_z),
                          UsdPhysics=UsdPhysics, UsdGeom=UsdGeom)
    # the TCP frame the controller commands and IsaacPlant reads back
    UsdGeom.Xform.Define(stage, f"{root_path}/platform/tcp")
    # gripper body carried by the platform (holds the contact geometry)
    _rigid_box(stage, f"{root_path}/platform/gripper", size=0.03,
               pos=Gf.Vec3f(0, 0, plat_z - 0.02),
               UsdPhysics=UsdPhysics, UsdGeom=UsdGeom)

    for i, deg in enumerate(_ARM_ANGLE_DEG):
        _build_arm(
            stage, root_path, i, deg, base_r, plat_r, rf, re, plat_z,
            Gf=Gf, PhysxSchema=PhysxSchema, UsdGeom=UsdGeom, UsdPhysics=UsdPhysics,
        )

    # -- the part to be picked (free rigid body, contact reporting on) --------
    part = _rigid_box(stage, part_path, size=0.02, pos=Gf.Vec3f(0, 0, plat_z - 0.1),
                      UsdPhysics=UsdPhysics, UsdGeom=UsdGeom)
    PhysxSchema.PhysxContactReportAPI.Apply(part.GetPrim())
    return root_path


def _build_arm(stage, root_path, i, deg, base_r, plat_r, rf, re, plat_z,
               *, Gf, PhysxSchema, UsdGeom, UsdPhysics):
    """One of the three 120-deg arms: actuated motor + guide elbow + loop-closer."""
    ca, sa = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    hinge = Gf.Vec3f(base_r * ca, base_r * sa, 0.0)         # base hinge
    attach = Gf.Vec3f(plat_r * ca, plat_r * sa, plat_z)     # platform attach
    elbow = Gf.Vec3f((base_r + rf) * ca, (base_r + rf) * sa, 0.0)  # arm tip (home)

    arm_axis = Gf.Vec3f(-sa, ca, 0.0)  # motor rotates in this arm's vertical plane

    upper = _rigid_box(stage, f"{root_path}/upper_{i}", size=0.03,
                       pos=Gf.Vec3f(*((np.asarray(hinge) + np.asarray(elbow)) / 2)),
                       UsdPhysics=UsdPhysics, UsdGeom=UsdGeom)
    forearm = _rigid_box(stage, f"{root_path}/forearm_{i}", size=0.025,
                         pos=Gf.Vec3f(*((np.asarray(elbow) + np.asarray(attach)) / 2)),
                         UsdPhysics=UsdPhysics, UsdGeom=UsdGeom)

    # 1) MOTOR: actuated revolute, base -> upper arm. This is DOF "motor_i".
    motor = UsdPhysics.RevoluteJoint.Define(stage, f"{root_path}/base/motor_{i}")
    motor.CreateBody0Rel().SetTargets([f"{root_path}/base"])
    motor.CreateBody1Rel().SetTargets([f"{root_path}/upper_{i}"])
    motor.CreateAxisAttr("X")  # local axis; align via local rotation on the rig
    motor.GetPrim().SetInstanceable(False)
    drive = UsdPhysics.DriveAPI.Apply(motor.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(1.0e6)
    drive.CreateDampingAttr(1.0e5)
    drive.CreateMaxForceAttr(1.0e7)
    # The DOF name Articulation.get_dof_index("motor_i") resolves is the joint
    # prim's own name (".../base/motor_i" -> "motor_i") -- no extra tagging needed.

    # 2) GUIDE: passive spherical elbow, upper arm -> forearm (no drive, no limit)
    elbow_j = UsdPhysics.SphericalJoint.Define(stage, f"{root_path}/upper_{i}/elbow_{i}")
    elbow_j.CreateBody0Rel().SetTargets([f"{root_path}/upper_{i}"])
    elbow_j.CreateBody1Rel().SetTargets([f"{root_path}/forearm_{i}"])

    # 3) LOOP-CLOSER: forearm -> platform, EXCLUDED from the articulation so the
    #    tree solver stays a tree while PhysX closes the loop as a constraint.
    loop = UsdPhysics.SphericalJoint.Define(stage, f"{root_path}/forearm_{i}/loop_{i}")
    loop.CreateBody0Rel().SetTargets([f"{root_path}/forearm_{i}"])
    loop.CreateBody1Rel().SetTargets([f"{root_path}/platform"])
    lp = PhysxSchema.PhysxJointAPI.Apply(loop.GetPrim())
    lp.CreateExcludeFromArticulationAttr(True)


def _rigid_box(stage, path, *, size, pos, UsdPhysics, UsdGeom):
    from pxr import Gf
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(size)
    cube.AddTranslateOp().Set(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))
    UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    UsdPhysics.MassAPI.Apply(cube.GetPrim())
    return cube


def main(out_path: str = "delta.usd", geom: DeltaGeom = DEFAULT_DELTA_GEOM) -> str:
    """Build a standalone Delta USD and save it. Run inside the Isaac runtime:

        python -m deltahil.plant.rig.build_delta   # writes delta.usd

    Then point IsaacPlant at it: ``IsaacPlant(usd_stage="delta.usd")``.
    """
    from pxr import Usd, UsdGeom
    stage = Usd.Stage.CreateNew(out_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    build_delta(stage, geom)
    stage.GetRootLayer().Save()
    print(f"wrote {out_path}  (geom f={geom.f} e={geom.e} rf={geom.rf} re={geom.re} mm)")
    return out_path


if __name__ == "__main__":  # pragma: no cover -- rig only
    import sys
    main(sys.argv[1] if len(sys.argv) > 1 else "delta.usd")
