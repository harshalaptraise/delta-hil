"""Option 2a on the rig: the kinematic Delta closes the pick loop with the mock
PLC, in real time. Honors P1/P2/P3; P4 deferred (see kinematic_delta.py).

This replaces the articulation-blocked eval-1 path for now: instead of the 0.5 mm
physics gate, we verify the real-time loop closes end-to-end (PLC drives the
platform to the part, grasp confirms) and RTF >= 1.0 (evals 3/9).
"""
import pytest

pytestmark = pytest.mark.rig

from deltahil.bridge import Bridge
from deltahil.plant.kinematic_delta import KinematicDeltaPlant
from deltahil.plc.mock_plc import MockPLC


def test_kinematic_loop_closes_in_realtime():
    # NB: do NOT use `with`/plant.close() here. SimulationApp.close() hard-exits
    # the process, which would kill pytest before it prints the asserts/summary.
    # Do all asserts first; let the interpreter's natural exit tear Kit down.
    pick_mm = (25.0, -15.0, -900.0)
    plant = KinematicDeltaPlant(headless=True, home_xyz_mm=(0.0, 0.0, -900.0))
    plant.set_part(pick_mm, present=True)         # ground truth (never to the PLC)
    plc = MockPLC()
    plc.set_target_from_vision(pick_mm)           # clean report: vision == truth
    bridge = Bridge(plc, plant)
    scans = bridge.run(max_scans=400, until=lambda: plc.done)
    sensors = plant.read_sensors()
    s = plant.rtf_summary()
    print(f"\nkin: closed={plc.done} in {scans} scans  "
          f"grip_confirm={sensors['sensor.grip_confirm']}  "
          f"tcp={tuple(round(v,2) for v in sensors['sensor.tcp_xyz'])}  "
          f"rtf={s['rtf']:.3f} fps={s['fps']:.1f}")
    assert plc.done, "loop did not close (grip never confirmed)"
    assert s["rtf"] >= 1.0, f"eval 3 FAIL: RTF {s['rtf']:.3f} < 1.0"
