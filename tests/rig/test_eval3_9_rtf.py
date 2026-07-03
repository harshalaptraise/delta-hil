"""Evals 3/9 (P4, P7) -- RTF >= 1.0 and >= 30 FPS on the rig.

Rig-only. Runs the full closed loop (mock PLC + real IsaacPlant) headless for a
few seconds and checks the sim keeps up with real time. RTF < 1.0 would mean the
sim is silently lying to the free-running PLC about how fast the world moved
(violates P1); < 30 FPS misses the interactive-visualisation bar.
"""
import os

import pytest

pytestmark = pytest.mark.rig

from deltahil.bridge import Bridge
from deltahil.plant.isaac_plant import IsaacPlant
from deltahil.plc.mock_plc import MockPLC


def test_rtf_and_fps_meet_realtime(delta_usd):
    plant = IsaacPlant(delta_usd, headless=True, grasp_mode="contact")
    plc = MockPLC()
    plc.set_target_from_vision((0.0, 0.0, plant.read_sensors()["sensor.tcp_xyz"][2]))
    bridge = Bridge(plc, plant)
    bridge.run(max_scans=int(round(5.0 / bridge.dt)))   # ~5 s of sim
    s = plant.rtf_summary()
    print(f"eval3/9: rtf={s['rtf']:.3f} fps={s['fps']:.1f} "
          f"sim={s['sim_s']:.2f}s wall={s['wall_s']:.2f}s n={s['n']}")
    assert s["rtf"] >= 1.0, f"eval 3 FAIL: RTF {s['rtf']:.3f} < 1.0"
    assert s["fps"] >= 30.0, f"eval 9 FAIL: FPS {s['fps']:.1f} < 30"
