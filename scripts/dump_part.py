"""Dump the prim structure + transforms of a few converted parts, so we can see
where the assembly-placement transform lives relative to the defaultPrim (and
therefore how to reference the parts so they assemble correctly).

Run on the rig (inside isaacenv):  python scripts/dump_part.py
"""
from __future__ import annotations

import glob
import os

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
USD_DIR = os.path.join(REPO, "assets", "usd")


def dump(path: str, max_depth: int = 3):
    s = Usd.Stage.Open(path)
    dp = s.GetDefaultPrim()
    print(f"\n=== {os.path.basename(path)} ===")
    print(f"defaultPrim: {dp.GetPath() if (dp and dp.IsValid()) else '(none)'}")
    xc = UsdGeom.XformCache(Usd.TimeCode.Default())
    for prim in s.Traverse():
        depth = len(prim.GetPath().pathString.strip("/").split("/"))
        if depth > max_depth:
            continue
        info = ""
        if prim.IsA(UsdGeom.Xformable):
            tr = xc.GetLocalToWorldTransform(prim).ExtractTranslation()
            ops = [o.GetOpName() for o in UsdGeom.Xformable(prim).GetOrderedXformOps()]
            info = f"   worldT=({tr[0]:.0f},{tr[1]:.0f},{tr[2]:.0f})  ops={ops}"
        indent = "  " * (depth - 1)
        print(f"  {indent}{prim.GetName() or '/'} [{prim.GetTypeName()}]{info}")


for key in ("BASE", "MovingPlate", "UA1"):
    hits = glob.glob(os.path.join(USD_DIR, f"*{key}*.usd"))
    if hits:
        dump(hits[0])
    else:
        print(f"(no file matching {key})")

app.close()
