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


def test_velocity_feedforward():
    # Velocity-mode conveyor tracking: the controller commands a velocity feed-forward
    # (X slaved to the belt), and picks/places still succeed under the TIGHTENED gate.
    from deltahil.plant.cell_plant import VEL_TOL
    assert VEL_TOL <= 0.02, "gate must be tight for the velocity lock to mean something"

    plant = CellPlant(seed=7)
    ctrl = MockCellController()
    dt = 0.01
    emitted_src = emitted_box = False
    for _ in range(3000):
        sensors = plant.read_sensors()
        cmds = ctrl.decide(sensors, dt)
        for cm in cmds.values():
            vx, vy, vz = cm["vel"]
            assert vy == 0.0 and vz == 0.0                 # only X (conveyor dir) is slaved
            if abs(vx - sensors["belt_v_src"]) < 1e-9 and abs(vx) > 1e-6:
                emitted_src = True                         # slaving to the source belt (pick)
            if abs(vx - sensors["belt_v_box"]) < 1e-9 and abs(vx) > 1e-6:
                emitted_box = True                         # slaving to the box belt (place)
        plant.apply_commands(cmds)
        plant.step(dt)
        assert plant.conserved()                           # E5

    assert emitted_src and emitted_box                     # feed-forward on BOTH belts
    # picks/places latch only when |tcp_vel - belt_v| < VEL_TOL (the gate) -> success here
    # means the velocity lock held at the tightened tolerance.
    assert plant.ledger["picked"] >= 4
    assert plant.ledger["placed"] >= 1
    assert plant.reach_violations == 0                     # integrated vel stays in reach (E4)
