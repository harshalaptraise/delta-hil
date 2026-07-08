"""MuJoCo physics-plant evals: the SAME controller, real contact dynamics.

Skipped if `mujoco` isn't installed (it's an optional extra). What these pin: the
belt carries by friction (not kinematics), the grasp still needs velocity
coincidence (not pose alone), the closed loop conserves with 0 reach violations,
placed tortillas physically land/pile in the totes, and the physics plant tracks
the kinematic reference under the same controller (cross-backend agreement).
"""
import pytest

pytest.importorskip("mujoco")

from deltahil.plant.cell_plant import CellPlant
from deltahil.plant.mujoco_cell_plant import MuJoCoCellPlant
from deltahil.plc.cell_controller import MockCellController

DT = 0.01


def _run(plant, ctrl, steps):
    for _ in range(steps):
        sensors = plant.read_sensors()
        plant.apply_commands(ctrl.decide(sensors, DT))
        plant.step(DT)
        assert plant.conserved()          # conservation holds EVERY step
    return plant


def test_belt_carries_items_by_contact_friction():
    # the belt is a frictional slab, not a kinematic conveyor -> emergent carry speed
    p = MuJoCoCellPlant(seed=7)
    p._spawn_tort()
    for _ in range(80):
        p.step(DT)                        # settle onto the belt
    x0 = p.parts[0]["x"]
    for _ in range(60):
        p.step(DT)
    v = (p.parts[0]["x"] - x0) / 0.6
    assert abs(v - p.vs) < 0.02, f"belt-carry {v:.3f} != {p.vs}"


def test_grasp_still_needs_velocity_coincidence():
    # gripper parked exactly on a moving tortilla but STATIONARY (vx=0) must not latch
    p = MuJoCoCellPlant(seed=7)
    p._spawn_tort()
    for _ in range(220):
        p.step(DT)                        # ride to mid-belt
    tort = next(q for q in p.parts if q["state"] == "belt")
    hold = {"Robot_B": {"tcp": (0.7, 0.0, 0.55), "grip": False, "vel": (0, 0, 0)}}
    for _ in range(6):
        g = (tort["x"], tort["y"], tort["z"] + 0.02)      # gripper on it, but not moving
        p.apply_commands({"Robot_A": {"tcp": g, "grip": True, "vel": (0, 0, 0)}, **hold})
        p.step(DT)
        tort = next((q for q in p.parts if q["id"] == tort["id"]), tort)
    assert p.robots["Robot_A"]["carry"] is None, "grabbed pose-only (no velocity match)"
    assert p.ledger["picked"] == 0


def test_closed_loop_picks_places_and_conserves():
    p = _run(MuJoCoCellPlant(seed=7), MockCellController(), steps=3500)
    assert p.ledger["picked"] >= 4
    assert p.ledger["placed"] >= 1
    assert p.reach_violations == 0
    assert p.conserved()


def test_placed_tortillas_physically_land_in_totes():
    p = _run(MuJoCoCellPlant(seed=7), MockCellController(), steps=3000)
    placed = [q for q in p.parts if q["state"] == "placed"]
    assert placed, "nothing placed"
    for q in placed:
        assert 0.33 <= q["z"] <= 0.44, f"placed tortilla z={q['z']} not inside a tote"
        assert abs(q["x"] - q["box"]["x"]) < 0.13, "placed tortilla not over its tote"


def test_cross_backend_agreement_with_kinematic():
    # same controller, same seed -> the physics plant tracks the kinematic reference
    kin = _run(CellPlant(seed=7), MockCellController(), steps=3000)
    mjc = _run(MuJoCoCellPlant(seed=7), MockCellController(), steps=3000)
    assert abs(kin.ledger["placed"] - mjc.ledger["placed"]) <= 4, \
        f"placed diverged: kinematic {kin.ledger['placed']} vs mujoco {mjc.ledger['placed']}"
