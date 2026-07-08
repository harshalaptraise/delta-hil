"""Rapier physics-plant evals: the SAME controller, a Rapier rigid-body world.

Skipped if `node` isn't on PATH (Rapier runs in a node worker; the WASM engine is
vendored, so no npm install is needed). What these pin: the belt carries at belt
speed (so pose AND velocity coincidence can both hold), the grasp still needs
velocity coincidence, the closed loop conserves with 0 reach violations, placed
tortillas physically land/pile in the totes, and the Rapier plant tracks the
kinematic reference under the same controller (cross-backend agreement).
"""
import shutil

import pytest

if shutil.which("node") is None:
    pytest.skip("node not on PATH (Rapier worker needs it)", allow_module_level=True)

from deltahil.plant.cell_plant import CellPlant
from deltahil.plant.rapier_cell_plant import RapierCellPlant
from deltahil.plc.cell_controller import MockCellController

DT = 0.01


def _run(plant, ctrl, steps):
    for _ in range(steps):
        sensors = plant.read_sensors()
        plant.apply_commands(ctrl.decide(sensors, DT))
        plant.step(DT)
        assert plant.conserved()
    return plant


def test_belt_carries_items_at_belt_speed():
    p = RapierCellPlant(seed=7)
    try:
        p._spawn_tort()
        for _ in range(90):
            p.step(DT)
        x0 = p.parts[0]["x"]
        for _ in range(60):
            p.step(DT)
        v = (p.parts[0]["x"] - x0) / 0.6
        assert abs(v - p.vs) < 0.02, f"belt-carry {v:.3f} != {p.vs}"
    finally:
        p.close()


def test_grasp_still_needs_velocity_coincidence():
    p = RapierCellPlant(seed=7)
    try:
        p._spawn_tort()
        for _ in range(220):
            p.step(DT)
        tort = next(q for q in p.parts if q["state"] == "belt")
        hold = {"Robot_B": {"tcp": (0.7, 0.0, 0.55), "grip": False, "vel": (0, 0, 0)}}
        for _ in range(6):
            g = (tort["x"], tort["y"], tort["z"] + 0.02)      # on it, but stationary
            p.apply_commands({"Robot_A": {"tcp": g, "grip": True, "vel": (0, 0, 0)}, **hold})
            p.step(DT)
            tort = next((q for q in p.parts if q["id"] == tort["id"]), tort)
        assert p.robots["Robot_A"]["carry"] is None, "grabbed pose-only (no velocity match)"
        assert p.ledger["picked"] == 0
    finally:
        p.close()


def test_closed_loop_picks_places_and_conserves():
    p = RapierCellPlant(seed=7)
    try:
        _run(p, MockCellController(), steps=2800)
        assert p.ledger["picked"] >= 4
        assert p.ledger["placed"] >= 1
        assert p.reach_violations == 0
        assert p.conserved()
    finally:
        p.close()


def test_placed_tortillas_physically_land_in_totes():
    p = RapierCellPlant(seed=7)
    try:
        _run(p, MockCellController(), steps=2800)
        placed = [q for q in p.parts if q["state"] == "placed"]
        assert placed, "nothing placed"
        for q in placed:
            assert 0.32 <= q["z"] <= 0.44, f"placed tortilla z={q['z']} not inside a tote"
            assert abs(q["x"] - q["box"]["x"]) < 0.13, "placed tortilla not over its tote"
    finally:
        p.close()


def test_cross_backend_agreement_with_kinematic():
    kin = _run(CellPlant(seed=7), MockCellController(), steps=2800)
    rap = RapierCellPlant(seed=7)
    try:
        _run(rap, MockCellController(), steps=2800)
        assert abs(kin.ledger["placed"] - rap.ledger["placed"]) <= 4, \
            f"placed diverged: kinematic {kin.ledger['placed']} vs rapier {rap.ledger['placed']}"
    finally:
        rap.close()
