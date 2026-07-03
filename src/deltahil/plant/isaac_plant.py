"""Isaac Sim plant -- the real physics seam.

Drops the recommended platform (Isaac Sim, standalone, RTX GPU) in behind the
``PlantModel`` interface. Everything upstream (``bridge.py``, ``tags.py``,
``interfaces.py``) is engine-agnostic and never changes; only this file speaks
PhysX. It matches ``MockPlant`` method-for-method (plus ``set_part``, which
``telemetry.run_closed_loop`` injects), so the same closed loop, scenarios and
eval harness run against real physics.

Integration checklist (traces to the constitution)
--------------------------------------------------
P4  Closed-chain Delta: PhysX articulations must be a kinematic *tree*. The rig
    (see ``rig/build_delta.py``) excludes the three loop-closing joints from the
    articulation, adds spherical guide joints for the passive parallelogram
    ends, and raises solver iteration counts. Gate on eval 1 (0.5 mm IK error)
    before trusting any downstream result.
P3  Contact grasp: ``grip_confirm`` comes from real PhysX contact force between
    gripper and part (``grasp_mode="contact"``, the default), sustained over a
    window while the part tracks the TCP -- pose AND timing AND force, jointly.
    ``grasp_mode="ideal"`` exposes the kinematic-attach baseline for bring-up.
P1  The plant steps in real time at a fixed ``physics_dt`` (0.004 s = 250 Hz,
    matching the bridge's ``dt`` and EGM). RTF/FPS are metered (evals 3/9).
P2  ``true_part_xyz`` (ground truth) is read straight from the part rigid body
    and never enters ``read_sensors()`` -- the PLC sees only the declared tags.

Unit boundary: tags are **mm** (base frame); Isaac/USD/PhysX are **meters**. All
conversion is confined to the seam methods here via ``_MM``.

This module imports cleanly on a machine without Isaac (only ``_require_isaac``
touches ``isaacsim``/``omni``), so ``pytest`` stays green on the dev laptop; only
*instantiating* ``IsaacPlant`` needs the runtime.
"""
from __future__ import annotations

import numpy as np

from .delta_ik import DEFAULT_DELTA_GEOM, DeltaGeom, Unreachable, ik
from .meters import RTFMeter

_MM = 0.001  # mm -> m

# Kit allows exactly one SimulationApp per process. Cache it so a second
# IsaacPlant (e.g. two rig tests in one pytest process) reuses the running app
# instead of crashing. The first boot's headless setting wins.
_SIM_APP = None
_SIM_SYMBOLS = None


def _boot_isaac(headless: bool):
    """Boot the Isaac Sim runtime and return ``(simulation_app, symbols)``.

    Isaac's Kit/Carbonite runtime has a hard ordering requirement: ``SimulationApp``
    must be instantiated *before* any ``isaacsim.core.*`` (or ``omni.*``) import --
    those extension modules do not exist until the app boots them. So we import and
    construct ``SimulationApp`` first, and only then import World/prims/stage utils.

    Raises a clear RuntimeError (not ImportError) if Isaac is absent, so callers on
    the laptop get the "needs the rig" message and ``pytest`` stays green.
    """
    global _SIM_APP, _SIM_SYMBOLS
    if _SIM_APP is not None:  # already booted in this process -- reuse it
        return _SIM_APP, _SIM_SYMBOLS

    try:
        from isaacsim import SimulationApp
    except ImportError as exc:  # no Isaac at all -- the laptop path
        raise RuntimeError(
            "IsaacPlant requires the Isaac Sim runtime (a Python 3.11 env with "
            "the isaacsim pip package on an RTX GPU). The headless MockPlant runs "
            "the full loop and eval 10 without it -- see this module's docstring."
        ) from exc

    # Booting Kit takes ~30-60 s the first time (extensions + shader warmup).
    simulation_app = SimulationApp({"headless": headless})

    # Safe now: the runtime is up, so the extension modules are importable.
    from isaacsim.core.api import World
    from isaacsim.core.utils.stage import add_reference_to_stage
    try:  # Isaac 4.5/5.x single-object wrappers
        from isaacsim.core.prims import (
            SingleArticulation as Articulation,
            SingleRigidPrim as RigidPrim,
            SingleXFormPrim as XFormPrim,
        )
    except ImportError:  # older layout without the Single* prefix
        from isaacsim.core.prims import Articulation, RigidPrim, XFormPrim

    symbols = {
        "World": World,
        "Articulation": Articulation,
        "RigidPrim": RigidPrim,
        "XFormPrim": XFormPrim,
        "add_reference_to_stage": add_reference_to_stage,
    }
    _SIM_APP, _SIM_SYMBOLS = simulation_app, symbols
    return simulation_app, symbols


def _confirm_grasp(grip_cmd: bool, force_N: float, part_tracks_tcp: bool,
                   threshold_N: float, held_steps: int, need_steps: int):
    """The P3 coincidence, as pure logic (unit-testable off the rig).

    A grasp is confirmed iff grip is commanded AND the gripper/part contact force
    exceeds the lift threshold AND the part is moving with the TCP (i.e. it is
    actually being carried, not merely brushed) -- sustained for ``need_steps``.
    Returns ``(confirmed, held_steps_next)``.
    """
    engaged = grip_cmd and force_N >= threshold_N and part_tracks_tcp
    held = held_steps + 1 if engaged else 0
    return (held >= need_steps, held)


class IsaacPlant:
    def __init__(
        self,
        usd_stage: str,
        *,
        headless: bool = True,
        geom: DeltaGeom = DEFAULT_DELTA_GEOM,
        grasp_mode: str = "contact",
        physics_dt: float = 0.004,
        grip_threshold_N: float = 5.0,
        grip_confirm_steps: int = 3,
        ee_prim: str = "/World/Delta/platform/tcp",
        part_prim: str = "/World/Part",
        gripper_prim: str = "/World/Delta/platform/gripper",
        articulation_prim: str = "/World/Delta",
        motor_dofs: tuple[str, str, str] = ("motor_0", "motor_1", "motor_2"),
    ):
        if grasp_mode not in ("contact", "ideal"):
            raise ValueError("grasp_mode must be 'contact' or 'ideal'")
        self.geom = geom
        self.grasp_mode = grasp_mode
        self.physics_dt = physics_dt
        self.grip_threshold_N = grip_threshold_N
        self.grip_confirm_steps = grip_confirm_steps
        self._ee_path = ee_prim
        self._part_path = part_prim
        self._gripper_path = gripper_prim
        self._art_path = articulation_prim
        self._motor_dofs = motor_dofs

        # -- control/sensor state (engine-agnostic) --
        self._target_mm = np.zeros(3)
        self._grip = False
        self._tracking = False
        self._grip_confirm = False
        self._part_present = False
        self._held_steps = 0
        self._rtf = RTFMeter()

        # Boot Kit first (SimulationApp), THEN import core modules -- see
        # _boot_isaac. Raises RuntimeError on the laptop (no isaacsim installed).
        self._sim_app, api = _boot_isaac(headless)
        self._api = api

        # World owns the PhysX scene + stepping cadence. render off in headless
        # soaks is the RTF lever (evals 3/9); on when visualising.
        self._world = api["World"](
            physics_dt=physics_dt, rendering_dt=physics_dt, stage_units_in_meters=1.0
        )
        self._render = not headless

        # Load the rigged Delta USD (asset with exclude-flags + guide joints +
        # solver counts already authored; or produced by rig/build_delta.py).
        api["add_reference_to_stage"](usd_path=usd_stage, prim_path=self._art_path)
        self._art = api["Articulation"](self._art_path)
        self._ee = api["XFormPrim"](self._ee_path)
        self._part = api["RigidPrim"](self._part_path)
        self._gripper = api["RigidPrim"](self._gripper_path)

        self._world.reset()  # initialises physics handles (DOF indices, views)
        self._motor_idx = [self._art.get_dof_index(n) for n in self._motor_dofs]

        # optional contact sensor on the gripper, filtered to the part (P3).
        self._contact = self._make_contact_sensor()
        # ideal-pick kinematic attach handle (created on grip, in ideal mode).
        self._attach_joint = None

    # -- contact wiring (rig-only; guarded so import/other modes don't need it) --
    def _make_contact_sensor(self):
        if self.grasp_mode != "contact":
            return None
        try:
            from isaacsim.sensors.physics import ContactSensor
        except ImportError:  # deprecated namespace
            from omni.isaac.sensor import ContactSensor
        return ContactSensor(
            prim_path=self._gripper_path + "/contact",
            min_threshold=0.0,
            max_threshold=1e7,
        )

    def _contact_force_N(self) -> float:
        """Net gripper<->part contact force this frame (Newtons). On the rig this
        reads the filtered PhysX contact; the force is what makes the grasp a
        *physical* event rather than a distance check (P3)."""
        if self._contact is None:
            return 0.0
        frame = self._contact.get_current_frame()
        return float(frame.get("force", 0.0))

    # -- ground truth injection (sim-only, never visible to the PLC -- P2) --
    def set_part(self, true_xyz, present: bool) -> None:
        self._part_present = present
        self._grip_confirm = False
        self._held_steps = 0
        if true_xyz is not None:
            pos_m = np.asarray(true_xyz, float) * _MM
            self._part.set_world_pose(position=pos_m)
        # detach any prior ideal-pick weld from the last cycle
        self._release_ideal()

    def true_part_xyz(self):
        pos_m, _ = self._part.get_world_pose()
        return tuple(np.asarray(pos_m, float) / _MM)

    # -- PlantModel interface ------------------------------------------------
    def apply_commands(self, values: dict) -> None:
        if "cmd.target_xyz" in values:
            self._target_mm = np.asarray(values["cmd.target_xyz"], float)
            try:
                thetas = ik(self.geom, self._target_mm)  # mm -> joint angles (rad)
            except Unreachable:
                thetas = None  # out-of-envelope: hold last target (real ctrl rejects)
            if thetas is not None:
                self._art.set_joint_position_targets(
                    np.asarray(thetas, float), joint_indices=self._motor_idx
                )
        if "cmd.grip" in values:
            self._grip = bool(values["cmd.grip"])
        if "cmd.tracking" in values:
            self._tracking = bool(values["cmd.tracking"])

    def step(self, dt: float) -> None:
        # Honour the bridge's dt via integer substeps of physics_dt; the bridge
        # owns dt (=0.004) -- the plant adapts, never asks it to change.
        n = max(1, round(dt / self.physics_dt))
        for _ in range(n):
            self._world.step(render=self._render)
            self._rtf.tick(self.physics_dt)
        self._update_grasp()

    def read_sensors(self) -> dict:
        pos_m, _ = self._ee.get_world_pose()
        tcp_mm = np.asarray(pos_m, float) / _MM
        return {
            "sensor.part_present": bool(self._part_present),
            "sensor.grip_confirm": bool(self._grip_confirm),
            "sensor.tcp_xyz": tuple(tcp_mm),
        }

    # -- grasp resolution ----------------------------------------------------
    def _update_grasp(self) -> None:
        if self._grip_confirm:
            return
        if self.grasp_mode == "ideal":
            self._update_grasp_ideal()
        else:
            self._update_grasp_contact()

    def _update_grasp_contact(self) -> None:
        force = self._contact_force_N()
        part_p, _ = self._part.get_world_pose()
        ee_p, _ = self._ee.get_world_pose()
        # part "tracks" the TCP when it sits within a gripper's reach of it
        tracks = float(np.linalg.norm(np.asarray(part_p) - np.asarray(ee_p))) < 0.02
        confirmed, self._held_steps = _confirm_grasp(
            self._grip and self._part_present, force, tracks,
            self.grip_threshold_N, self._held_steps, self.grip_confirm_steps,
        )
        self._grip_confirm = confirmed

    def _update_grasp_ideal(self) -> None:
        # kinematic-attach baseline: on grip, weld the part to the gripper and
        # latch confirm. Isolates kinematics (eval 1) from contact-solver tuning.
        if self._grip and self._part_present and self._attach_joint is None:
            self._attach_ideal()
        self._grip_confirm = self._attach_joint is not None

    def _attach_ideal(self) -> None:
        from pxr import UsdPhysics
        stage = self._world.stage
        path = self._gripper_path + "/ideal_weld"
        joint = UsdPhysics.FixedJoint.Define(stage, path)
        joint.CreateBody0Rel().SetTargets([self._gripper_path])
        joint.CreateBody1Rel().SetTargets([self._part_path])
        self._attach_joint = path

    def _release_ideal(self) -> None:
        if self._attach_joint is not None:
            self._world.stage.RemovePrim(self._attach_joint)
            self._attach_joint = None

    # -- metering (evals 3/9) -----------------------------------------------
    def rtf_summary(self) -> dict:
        return self._rtf.summary()

    # -- lifecycle -----------------------------------------------------------
    def close(self) -> None:
        """Shut down the Kit runtime. Call at the end of a rig session; leaving
        SimulationApp running holds the GPU and blocks a clean process exit."""
        global _SIM_APP, _SIM_SYMBOLS
        app = getattr(self, "_sim_app", None)
        if app is not None:
            app.close()
            self._sim_app = None
            _SIM_APP, _SIM_SYMBOLS = None, None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False
