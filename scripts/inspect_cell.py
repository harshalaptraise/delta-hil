"""Inspect the cell layout FB000224.usd so we can drive the two robots.

Run on the rig (inside isaacenv):  python scripts/inspect_cell.py

Reports: units + up-axis, the prim tree (to a depth), and for every Xformable
prim its world translation + rotation. Flags candidate robots (prims whose
subtree contains our part leaf names UA1/LA1_*/MovingPlate/RevoluteLink*) and
candidate conveyors/boxes (name contains 'conv'/'belt'/'box'/'pick'/'place').
That tells us whether the cell robots are articulable in place (named like ours)
or whether we instance our own irb360.usd at each robot's base frame, plus the
conveyor/box world positions to aim the pick/place at.
"""
from __future__ import annotations

import os

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import omni.usd  # noqa: E402
from pxr import Usd, UsdGeom  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
USD = os.path.join(REPO, "assets", "FB000224.usd").replace("\\", "/")

# leaf names our pose() drives; a subtree containing these is an articulable robot
ROBOT_LEAVES = {"UA1", "UA2", "UA3", "MovingPlate", "RevoluteLinkPlate",
                "LA1_1", "LA1_CU"}
CELL_HINTS = ("conv", "belt", "box", "pick", "place", "infeed", "outfeed", "tote")


def _composition(prim):
    """Return list of (arc_type, asset_path) for references/payloads on a prim."""
    arcs = []
    refs = prim.GetMetadata("references")
    if refs:
        for r in getattr(refs, "prependedItems", []) + getattr(refs, "appendedItems", []) \
                + getattr(refs, "explicitItems", []):
            arcs.append(("reference", r.assetPath))
    pay = prim.GetMetadata("payload")
    if pay:
        for p in getattr(pay, "prependedItems", []) + getattr(pay, "appendedItems", []) \
                + getattr(pay, "explicitItems", []):
            arcs.append(("payload", p.assetPath))
    return arcs


def main():
    # open directly with pxr so we control payload loading + can read arcs
    stage = Usd.Stage.Open(USD, load=Usd.Stage.LoadAll)
    stage.Load()  # force-load every payload
    for _ in range(30):
        app.update()

    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    up = UsdGeom.GetStageUpAxis(stage)
    dp = stage.GetDefaultPrim()
    print("\n================ CELL: FB000224.usd ================")
    print(f"metersPerUnit={mpu}  upAxis={up}  defaultPrim={dp.GetPath() if dp else None}")

    xc = UsdGeom.XformCache(Usd.TimeCode.Default())
    all_prims = list(stage.Traverse())
    all_incl = list(stage.TraverseAll())
    print(f"prims: loaded={len(all_prims)}  including-unloaded={len(all_incl)}")

    print("\n-- composition (references / payloads) --")
    found_arc = False
    for prim in all_incl:
        arcs = _composition(prim)
        for kind, path in arcs:
            found_arc = True
            print(f"  {prim.GetPath()}  {kind} -> {path}")
    if not found_arc:
        print("  (no authored references/payloads found)")

    print("\n-- ALL Xform/mesh prims (name + world T, first 60) --")
    shown = 0
    for prim in all_prims:
        if shown >= 60:
            print("  ...(truncated)")
            break
        if prim.IsA(UsdGeom.Xformable):
            m = xc.GetLocalToWorldTransform(prim)
            t = m.ExtractTranslation()
            print(f"  {prim.GetPath()} [{prim.GetTypeName()}]  T=({t[0]:.2f},{t[1]:.2f},{t[2]:.2f})")
            shown += 1

    print("\n-- prim tree (depth<=3, Xformables show world T + rot) --")
    for prim in all_prims:
        depth = len(prim.GetPath().pathString.strip("/").split("/"))
        if depth > 3:
            continue
        info = ""
        if prim.IsA(UsdGeom.Xformable):
            m = xc.GetLocalToWorldTransform(prim)
            t = m.ExtractTranslation()
            rot = m.ExtractRotation().GetAngle()
            info = f"  T=({t[0]:.0f},{t[1]:.0f},{t[2]:.0f}) rot={rot:.0f}deg"
        indent = "  " * (depth - 1)
        print(f"  {indent}{prim.GetName()} [{prim.GetTypeName()}]{info}")

    # robot subtrees: the shallowest ancestor whose subtree holds our leaves
    print("\n-- candidate ROBOTS (subtrees containing our part leaf names) --")
    robot_roots = {}
    for prim in all_prims:
        if prim.GetName() in ROBOT_LEAVES:
            # walk up to the child-of-defaultPrim-level robot root
            anc = prim
            while anc.GetParent() and anc.GetParent().GetName() not in (
                    "", (dp.GetName() if dp else "")):
                anc = anc.GetParent()
            robot_roots.setdefault(anc.GetPath().pathString, anc)
    for path, prim in robot_roots.items():
        m = xc.GetLocalToWorldTransform(prim)
        t = m.ExtractTranslation()
        r = m.ExtractRotation()
        print(f"  {path}  baseT=({t[0]:.0f},{t[1]:.0f},{t[2]:.0f}) "
              f"rot={r.GetAngle():.0f}deg axis={tuple(round(a,2) for a in r.GetAxis())}")
    if not robot_roots:
        print("  (none found with our leaf names -> cell robots are NOT our "
              "separated parts; we'll instance irb360.usd at each base frame)")

    print("\n-- candidate CONVEYORS / BOXES (name hints) --")
    for prim in all_prims:
        nm = prim.GetName().lower()
        if any(h in nm for h in CELL_HINTS) and prim.IsA(UsdGeom.Xformable):
            m = xc.GetLocalToWorldTransform(prim)
            t = m.ExtractTranslation()
            print(f"  {prim.GetPath()}  T=({t[0]:.0f},{t[1]:.0f},{t[2]:.0f})")
    print("====================================================\n")
    app.close()


if __name__ == "__main__":
    main()
