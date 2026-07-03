"""Inspect the converted IRB 360 USD parts: units, up-axis, default prim, and
world bounding box. Tells us the scale and whether parts share an assembly frame
(so we know how to place them on the kinematic rig).

Run on the rig (inside isaacenv):  python scripts/inspect_usd.py
"""
from __future__ import annotations

import glob
import os

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom  # noqa: E402 (after SimulationApp)

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(REPO, "assets", "usd")


def short(name: str) -> str:
    # IRB360_..._rev03_UA1_CAD -> UA1 ; ..._rev04_BASE_CAD -> BASE
    b = os.path.splitext(os.path.basename(name))[0]
    for tag in ("rev03_", "rev04_", "rev05_"):
        if tag in b:
            b = b.split(tag, 1)[1]
    return b.replace("_CAD", "").replace(".step1", "")


print("\n================ USD PART INSPECTION ================")
print(f"{'part':16s} {'m/unit':>7s} {'up':>3s}  {'size (mm, sorted)':>26s}   center(mm)")
for path in sorted(glob.glob(os.path.join(OUT, "*.usd"))):
    stage = Usd.Stage.Open(path)
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    up = UsdGeom.GetStageUpAxis(stage)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    rng = cache.ComputeWorldBound(stage.GetPseudoRoot()).ComputeAlignedRange()
    mn, mx = rng.GetMin(), rng.GetMax()
    to_mm = mpu * 1000.0
    size = sorted([(mx[i] - mn[i]) * to_mm for i in range(3)], reverse=True)
    ctr = [((mx[i] + mn[i]) / 2.0) * to_mm for i in range(3)]
    print(f"{short(path):16s} {mpu:7.4f} {str(up):>3s}  "
          f"{size[0]:8.1f} x{size[1]:7.1f} x{size[2]:7.1f}   "
          f"({ctr[0]:7.0f},{ctr[1]:7.0f},{ctr[2]:7.0f})")
print("====================================================\n")

app.close()
