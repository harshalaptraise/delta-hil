"""Evals 3/9 soak -- run the closed loop against the real Isaac Delta and report
real-time factor + FPS over a sustained run.

Rig-only. Run inside the Isaac Sim Python runtime on the RTX box:

    python scripts/soak_rtf.py                       # 60 s, builds a Delta USD
    python scripts/soak_rtf.py 120 /path/delta.usd   # 120 s, given asset

Prints a PASS/FAIL against the eval-3/9 bar (RTF >= 1.0, FPS >= 30). This is the
plant-side twin of the bridge's eval-5 latency meter: same "the acceptance test
has a home" idea, now with real numbers.
"""
from __future__ import annotations

import sys


def main(seconds: float = 60.0, usd: str | None = None) -> int:
    from deltahil.bridge import Bridge
    from deltahil.plant.isaac_plant import IsaacPlant
    from deltahil.plc.mock_plc import MockPLC

    if usd is None:
        # build a throwaway rigged Delta from geometry (needs the Isaac runtime)
        from pxr import Usd, UsdGeom
        from deltahil.plant.rig.build_delta import build_delta
        usd = "delta_soak.usd"
        stage = Usd.Stage.CreateNew(usd)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        build_delta(stage)
        stage.GetRootLayer().Save()

    plant = IsaacPlant(usd, headless=True, grasp_mode="contact")
    try:
        plc = MockPLC()
        plc.set_target_from_vision((0.0, 0.0, plant.read_sensors()["sensor.tcp_xyz"][2]))
        bridge = Bridge(plc, plant)

        n = int(round(seconds / bridge.dt))
        print(f"soaking {seconds:.0f} s ({n} scans at dt={bridge.dt}) ...")
        bridge.run(max_scans=n)

        s = plant.rtf_summary()
        lat = bridge.fast_meter.summary_ms()
    finally:
        plant.close()
    print("-" * 56)
    print(f"RTF  {s['rtf']:.3f}   (sim {s['sim_s']:.1f}s / wall {s['wall_s']:.1f}s)")
    print(f"FPS  {s['fps']:.1f}   over {s['n']} physics frames")
    print(f"fast-tier scan  mean {lat['mean_ms']:.3f} ms  jitter {lat['jitter_ms']:.3f} ms")
    ok = s["rtf"] >= 1.0 and s["fps"] >= 30.0
    print(f"EVAL 3/9: {'PASS' if ok else 'FAIL'}  (need RTF>=1.0 and FPS>=30)")
    print("-" * 56)
    return 0 if ok else 1


if __name__ == "__main__":
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    path = sys.argv[2] if len(sys.argv) > 2 else None
    raise SystemExit(main(secs, path))
