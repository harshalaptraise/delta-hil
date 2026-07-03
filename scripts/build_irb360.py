"""Assemble the converted IRB 360 parts into one USD (they share an assembly
frame, so identity references reassemble the whole robot). Writes
assets/irb360.usd -- openable in the Isaac Sim GUI and the skin source for the
kinematic plant next.

Run on the rig (inside isaacenv):  python scripts/build_irb360.py
"""
from __future__ import annotations

import glob
import os

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
USD_DIR = os.path.join(REPO, "assets", "usd").replace("\\", "/")
OUT = os.path.join(REPO, "assets", "irb360.usd").replace("\\", "/")


def prim_name(path: str) -> str:
    b = os.path.splitext(os.path.basename(path))[0]
    for tag in ("rev03_", "rev04_", "rev05_"):
        if tag in b:
            b = b.split(tag, 1)[1]
    b = b.replace("_CAD", "").replace(".step1", "")
    return "".join(c if (c.isalnum() or c == "_") else "_" for c in b)


def ref_target(part_usd: str):
    s = Usd.Stage.Open(part_usd)
    dp = s.GetDefaultPrim()
    if dp and dp.IsValid():
        return None
    for child in s.GetPseudoRoot().GetChildren():
        return child.GetPath()
    return None


print(f"\n[build_irb360] source dir: {USD_DIR}")
print(f"[build_irb360] output:     {OUT}")
parts = sorted(glob.glob(os.path.join(USD_DIR, "*.usd")))
print(f"[build_irb360] found {len(parts)} part USDs")
if not parts:
    print("  ERROR: no part USDs found -- run scripts/step_to_usd.py first.")
    app.close()
    raise SystemExit(1)

if os.path.exists(OUT):
    os.remove(OUT)                      # CreateNew fails if the layer exists

stage = Usd.Stage.CreateNew(OUT)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 0.001)
UsdGeom.Xform.Define(stage, "/World")
UsdGeom.Xform.Define(stage, "/World/IRB360")

added = 0
for p in parts:
    name = prim_name(p)
    rel = os.path.relpath(p, os.path.dirname(OUT)).replace("\\", "/")
    try:
        tgt = ref_target(p)
        prim = stage.DefinePrim(f"/World/IRB360/{name}", "Xform")
        if tgt is None:
            prim.GetReferences().AddReference(rel)
        else:
            prim.GetReferences().AddReference(rel, tgt)
        added += 1
        print(f"  + {name:16s} <- {os.path.basename(p)}  (ref {'defaultPrim' if tgt is None else tgt})")
    except Exception as exc:
        print(f"  ! {name:16s} FAILED: {type(exc).__name__}: {exc}")

stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
stage.Save()

ok = os.path.exists(OUT)
size = os.path.getsize(OUT) if ok else 0
print(f"\n[build_irb360] referenced {added}/{len(parts)} parts")
print(f"[build_irb360] wrote {OUT}  exists={ok}  size={size} bytes")
print("  open it in the Isaac Sim GUI (File > Open) to see the assembled IRB 360.\n")

app.close()
