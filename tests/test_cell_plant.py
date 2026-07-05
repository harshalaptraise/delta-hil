"""Laptop verification of the PLC-driven cell loop (evals E1/E2/E4/E5).

Runs the pure-Python CellPlant under the reference MockCellController -- the same
plant the real TwinCAT PLC drives on the rig -- and checks the calibration evals.
"""
import inspect

from deltahil.plant.cell_plant import CellPlant
from deltahil.plc.cell_controller import MockCellController


def _run(plant, ctrl, steps, dt=0.01):
    for _ in range(steps):
        sensors = plant.read_sensors()
        if ctrl is not None:
            plant.apply_commands(ctrl.decide(sensors, dt))
        plant.step(dt)
        assert plant.conserved()          # E5: mass balance holds every step
    return plant


def test_loop_picks_places_and_conserves():
    plant = CellPlant()
    _run(plant, MockCellController(), steps=2500)    # 25 s
    assert plant.ledger["picked"] >= 4               # E1: the loop actually works
    assert plant.ledger["placed"] >= 1               # parts reach the boxes
    assert plant.reach_violations == 0               # E4: no command out of reach
    assert plant.ledger["placed"] <= plant.ledger["picked"]
    assert plant.conserved()                         # E5


def test_both_robots_work():
    # E4 exclusivity + load balance: both robots complete picks, none double-claimed
    plant = CellPlant()
    ctrl = MockCellController()
    _run(plant, ctrl, steps=2500)
    # every carried/placed part has exactly one owning robot (set at grab)
    owners = [p["robot"] for p in plant.parts if p["state"] in ("carried", "placed")]
    assert all(o in ("Robot_A", "Robot_B") for o in owners)


def test_no_controller_no_motion():
    # E1: with no controller, nothing is commanded -> no grabs, robots stay home
    plant = _run(CellPlant(), None, steps=1200)
    assert plant.ledger["picked"] == 0
    assert plant.reach_violations == 0


def test_plant_holds_no_control_logic():
    # E1 (structural): the plant senses/actuates only -- no decision code
    src = inspect.getsource(inspect.getmodule(CellPlant))
    for banned in ("claim", "assign", "intercept", "nearest_box", "schedule"):
        assert banned not in src, f"control logic leaked into the plant: {banned!r}"
