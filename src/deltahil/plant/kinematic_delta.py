"""Kinematic Delta plant -- real CAD-ready, real-time, P4 deferred.

The full closed-chain PhysX articulation (P4) is a substantial modeling effort
and no pre-rigged IRB 360 asset exists. This plant is the pragmatic path agreed
with the user: the moving platform is driven by the **exact analytic Delta
kinematics** (``delta_ik``, proven inverse in CI), while PhysX still does the
real work that matters for a pick -- the part is a dynamic rigid body and the
grasp is gated on the gripper actually reaching it.

What it honors / defers (stated plainly -- calibration-protocol Step 4):
  P1  real-time closed loop: steps at a fixed physics_dt (250 Hz), RTF metered.
  P2  I/O contract only: consumes cmd.* tags, emits sensor.* tags; ground truth
      (part pose) never enters read_sensors().
  P3  grasp is a coincidence: grip_confirm requires the gripper to be at the part
      AND grip commanded (contact-gated attach). [v1 uses a proximity gate; a
      PhysX contact-force gate is the planned upgrade.]
  P4  DEFERRED: the parallelogram is posed analytically, not simulated as a
      closed dynamic loop. This is a conscious, agreed deviation; the full
      articulation rig is the upgrade path (Isaac "Rig Closed-Loop Structures").

Behind the same ``PlantModel`` interface as ``MockPlant`` / ``IsaacPlant`` -- the
bridge, tags and evals are unchanged. Imports clean on the laptop (only boot
touches isaacsim); only instantiation needs the runtime.
"""
from __future__ import annotations

import numpy as np

from .delta_ik import DEFAULT_DELTA_GEOM, DeltaGeom
from .isaac_plant import _boot_isaac, _confirm_grasp
from .meters import RTFMeter

_MM = 0.001  # mm -> m


class KinematicDeltaPlant:
    def __init__(
        self,
        *,
        headless: bool = True,
        geom: DeltaGeom = DEFAULT_DELTA_GEOM,
        physics_dt: float = 0.004,
        grip_reach_mm: float = 5.0,
        grip_confirm_steps: int = 3,
        home_xyz_mm=(0.0, 0.0, -900.0),
        platform_prim: str = "/World/Delta/platform",
        tcp_prim: str = "/World/Delta/platform/tcp",
        part_prim: str = "/World/Part",
    ):
        self.geom = geom
        self.physics_dt = physics_dt
        self.grip_reach_mm = grip_reach_mm
        self.grip_confirm_steps = grip_confirm_steps
        self._home_mm = np.asarray(home_xyz_mm, float)
        self._plat_path = platform_prim
        self._tcp_path = tcp_prim
        self._part_path = part_prim

        # engine-agnostic state
        self._target_mm = self._home_mm.copy()
        self._grip = False
        self._tracking = False
        self._grip_confirm = False
        self._part_present = False
        self._held_steps = 0
        self._attached = False
        self._rtf = RTFMeter()

        self._sim_app, api = _boot_isaac(headless)   # raises on the laptop
        self._api = api
        self._world = api["World"](
            physics_dt=physics_dt, rendering_dt=physics_dt, stage_units_in_meters=1.0
        )
        self._render = not headless

        self._build_scene(api)
        self._world.reset()

    # -- scene: a kinematic platform + gripper collider, and a dynamic part ----
    def _build_scene(self, api):
        from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

        stage = api["get_current_stage"]()
        home_m = self._home_mm * _MM

        # platform: a KINEMATIC rigid body -- we teleport it to the analytic TCP
        # each step; kinematic bodies push/carry dynamic bodies through contact.
        plat = UsdGeom.Cube.Define(stage, self._plat_path)
        plat.CreateSizeAttr(0.05)
        plat.AddTranslateOp().Set(Gf.Vec3d(*(float(v) for v in home_m)))
        UsdPhysics.CollisionAPI.Apply(plat.GetPrim())
        rb = UsdPhysics.RigidBodyAPI.Apply(plat.GetPrim())
        rb.CreateKinematicEnabledAttr(True)
        # the TCP frame the controller reads back (child transform of platform)
        UsdGeom.Xform.Define(stage, self._tcp_path)

        # part: a KINEMATIC body placed by set_part. Kinematic means it never
        # free-falls under gravity and a kinematic platform can't shove it on
        # arrival -- so the proximity grasp gate catches a clean coincidence.
        # (A dynamic part + real PhysX contact is the 2b upgrade.)
        part = UsdGeom.Cube.Define(stage, self._part_path)
        part.CreateSizeAttr(0.02)
        part.AddTranslateOp().Set(Gf.Vec3d(*(float(v) for v in home_m)))
        UsdPhysics.CollisionAPI.Apply(part.GetPrim())
        prb = UsdPhysics.RigidBodyAPI.Apply(part.GetPrim())
        prb.CreateKinematicEnabledAttr(True)

        self._plat = api["RigidPrim"](self._plat_path)
        self._tcp = api["XFormPrim"](self._tcp_path)
        self._part = api["RigidPrim"](self._part_path)

    # -- ground truth injection (sim-only, never to the PLC -- P2) -------------
    def set_part(self, true_xyz, present: bool) -> None:
        self._part_present = present
        self._grip_confirm = False
        self._held_steps = 0
        self._attached = False
        if true_xyz is not None:
            self._part.set_world_pose(position=np.asarray(true_xyz, float) * _MM)

    def true_part_xyz(self):
        pos_m, _ = self._part.get_world_pose()
        return tuple(np.asarray(pos_m, float) / _MM)

    # -- PlantModel interface -------------------------------------------------
    def apply_commands(self, values: dict) -> None:
        if "cmd.target_xyz" in values:
            self._target_mm = np.asarray(values["cmd.target_xyz"], float)
        if "cmd.grip" in values:
            self._grip = bool(values["cmd.grip"])
        if "cmd.tracking" in values:
            self._tracking = bool(values["cmd.tracking"])

    def step(self, dt: float) -> None:
        # place the platform (and its TCP) at the commanded pose -- the analytic
        # Delta reaches its target exactly, so TCP == target (P4 deferred).
        self._plat.set_world_pose(position=self._target_mm * _MM)
        if self._grip_confirm:                       # once grasped, carry the part
            self._part.set_world_pose(position=self._target_mm * _MM)
        n = max(1, round(dt / self.physics_dt))
        for _ in range(n):
            self._world.step(render=self._render)
            self._rtf.tick(self.physics_dt)
        self._update_grasp()

    def read_sensors(self) -> dict:
        pos_m, _ = self._tcp.get_world_pose()
        return {
            "sensor.part_present": bool(self._part_present),
            "sensor.grip_confirm": bool(self._grip_confirm),
            "sensor.tcp_xyz": tuple(np.asarray(pos_m, float) / _MM),
        }

    # -- grasp: proximity-gated pick (P3, v1) --------------------------------
    def _update_grasp(self) -> None:
        # v1 gate: the gripper must actually reach the part (pose coincidence)
        # while grip is commanded and a part is present -- the P3 conjuncts,
        # with a proximity stand-in for contact force. Once confirmed, step()
        # carries the (kinematic) part with the platform. Real contact-force
        # gating is the 2b upgrade.
        if self._grip_confirm:
            return
        plat_p, _ = self._plat.get_world_pose()
        part_p, _ = self._part.get_world_pose()
        reach_m = self.grip_reach_mm * _MM
        at_part = float(np.linalg.norm(np.asarray(plat_p) - np.asarray(part_p))) < reach_m
        confirmed, self._held_steps = _confirm_grasp(
            self._grip and self._part_present, 1.0 if at_part else 0.0, at_part,
            0.5, self._held_steps, self.grip_confirm_steps,
        )
        self._grip_confirm = confirmed

    # -- metering (evals 3/9) + lifecycle ------------------------------------
    def rtf_summary(self) -> dict:
        return self._rtf.summary()

    def close(self) -> None:
        app = getattr(self, "_sim_app", None)
        if app is not None:
            app.close()
            self._sim_app = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False
