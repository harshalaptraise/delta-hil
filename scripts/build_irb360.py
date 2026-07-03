"""Assemble the converted IRB 360 parts into one USD (they share an assembly
frame, so identity references reassemble the whole robot). Saves
assets/irb360.usd, which you can open in the Isaac Sim GUI to see the real
FlexPicker, and which the kinematic plant will skin onto next.

Run on the rig (inside isaacenv):  python scripts/build_irb360.py
"""
from __future__ import annotations

import glob
import os

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from pxr import Sdf, Usd, UsdGeom  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
USD_DIR = os.path.join(REPO, "assets", "usd")
OUT = os.path.join(REPO, "assets", "irb360.usd")


def prim_name(path: str) -> str:
    b = os.path.splitext(os.path.basename(path))[0]
    for tag in ("rev03_", "rev04_", "rev05_"):
        if tag in b:
            b = b.split(tag, 1)[1]
    b = b.replace("_CAD", "").replace(".step1", "")
    return "".join(c if (c.isalnum() or c == "_") else "_" for c in b)


def ref_target(part_usd: str):
    """Prim path to reference: defaultPrim if set, else the first root child."""
    s = Usd.Stage.Open(part_usd)
    dp = s.GetDefaultPrim()
    if dp and dp.IsValid():
        return None  # empty path -> reference the defaultPrim
    for child in s.GetPseudoRoot().GetChildren():
        return child.GetPath()
    return None


parts = sorted(glob.glob(os.path.join(USD_DIR, "*.usd")))
stage = Usd.Stage.CreateNew(OUT)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 0.001)          # mm, matching the parts
UsdGeom.Xform.Define(stage, "/World")
UsdGeom.Xform.Define(stage, "/World/IRB360")

print(f"\n[build_irb360] assembling {len(parts)} parts -> {OUT}")
for p in parts:
    name = prim_name(p)
    prim = stage.DefinePrim(f"/World/IRB360/{name}", "Xform")
    tgt = ref_target(p)
    rel = os.path.relpath(p, os.path.dirname(OUT)).replace("\\", "/")
    if tgt is None:
        prim.GetReferences().AddReference(rel)
    else:
        prim.GetReferences().AddReference(rel, tgt)
    print(f"  + {name:16s} <- {os.path.basename(p)}  (ref {'defaultPrim' if tgt is None else tgt})")

stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
stage.Save()
print(f"\n[build_irb360] wrote {OUT}")
print("  open it in the Isaac Sim GUI (File > Open) to see the assembled IRB 360.\n")

app.close()
