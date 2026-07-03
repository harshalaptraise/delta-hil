"""The loop closes (P1) and control flows only through the tag map (P2)."""
import numpy as np

from deltahil.bridge import Bridge
from deltahil.plant.mock_plant import MockPlant
from deltahil.plc.mock_plc import MockPLC
from deltahil.scenario import Scenario, default_offset
from deltahil.telemetry import calibration_offset, run_closed_loop


def test_single_calibrated_pick_confirms():
    plc = MockPLC()
    plant = MockPlant()
    true_xyz = np.array([50.0, -30.0, 20.0])
    plc.set_calibration(np.zeros(3))          # perfect report, no offset
    plant.set_part(true_xyz, present=True)
    plc.set_target_from_vision(true_xyz)       # vision == truth here
    bridge = Bridge(plc, plant)
    bridge.run(max_scans=40, until=lambda: plc.done)
    assert plc.done
    sensors = plant.read_sensors()
    assert sensors["sensor.grip_confirm"] is True   # P3 coincidence achieved


def test_grip_confirm_requires_part_present():
    # P2: with no part present, the sensor never confirms -> PLC never finishes.
    plc = MockPLC()
    plant = MockPlant()
    plc.set_calibration(np.zeros(3))
    plant.set_part(np.array([0.0, 0.0, 0.0]), present=False)  # jam
    plc.set_target_from_vision(np.array([0.0, 0.0, 0.0]))
    Bridge(plc, plant).run(max_scans=40, until=lambda: plc.done)
    assert not plc.done


def test_calibration_lifts_closed_loop_success():
    scn = Scenario(vision_offset=default_offset(deg=0.0), pose_sigma_mm=0.15, seed=7)
    off, _ = calibration_offset(scn, n_calib=25)
    uncal = run_closed_loop(scn, n_picks=120)
    cal = run_closed_loop(scn, n_picks=120, calib_offset_xyz=off)
    assert uncal.success_rate < 0.05
    assert cal.success_rate > 0.85


def test_faults_cap_success_below_clean(  ):
    # P5: jams+misfeeds the calibration cannot touch pull success down.
    clean = Scenario(vision_offset=default_offset(deg=0.0), pose_sigma_mm=0.15, seed=4)
    hard = Scenario(vision_offset=default_offset(deg=0.0), pose_sigma_mm=0.15,
                    jam_rate=0.1, misfeed_rate=0.05, seed=4)
    off_c, _ = calibration_offset(clean, n_calib=25)
    off_h, _ = calibration_offset(hard, n_calib=25)
    sc = run_closed_loop(clean, n_picks=200, calib_offset_xyz=off_c)
    sh = run_closed_loop(hard, n_picks=200, calib_offset_xyz=off_h)
    assert sh.success_rate < sc.success_rate
