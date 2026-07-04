"""Build a two-robot IRB 360 cell from our articulable irb360.usd and render a
static layout to validate placement / scale / reach before animating.

FB000224.usd self-payloads (broken), so we construct the cell: two robots over
two conveyors (source + box), each posed reaching its source pick point. Cell is
in METRES (Z-up); our robot is mm, so each robot sits under a parent Xform with
scale 0.001 + a base translation. Targets are converted world->local via
irb360_pose.world_to_local.

Run on the rig (inside isaacenv):  python scripts/cell_layout.py
Output: assets/render/cell_layout.png
"""
from __future__ import annotations

import os

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdLux  # noqa: E402

from deltahil.plant.irb360_pose import pose, world_to_local  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IRB360 = os.path.join(REPO, "assets", "irb360.usd").replace("\\", "/")
RENDER_DIR = os.path.join(REPO, "assets", "render").replace("\\", "/")
OUT_PNG = os.path.join(RENDER_DIR, "cell_layout.png").replace("\\", "/")
os.makedirs(RENDER_DIR, exist_ok=True)

MM = 0.001  # our robot is mm; cell is metres

# --- cell layout (metres) ---------------------------------------------------
ROBOT_X = 0.70                       # robot bases at x = +/- ROBOT_X (1.4 m apart)
BASE_Z = 1.50                        # robot mount height
ROBOTS = {
    "Robot_A": {"base": (-ROBOT_X, 0.0, BASE_Z)},
    "Robot_B": {"base": (+ROBOT_X, 0.0, BASE_Z)},
}
SRC_Y, BOX_Y = -0.22, 0.22           # source & box conveyor centre-lines (world y)
BOX_TOP = 0.15                       # box conveyor belt-top height
SRC_TOP = BOX_TOP + 0.30             # product conveyor ~1 ft higher (taller boxes)
PART_Z = SRC_TOP + 0.03              # part sits on the source belt
BELT_LEN = 2.6                       # conveyors run along X


def box(stage, path, size, pos, color=(0.6, 0.6, 0.6)):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)         # unit cube (±0.5); scale to size
    M = Gf.Matrix4d().SetScale(Gf.Vec3d(*size)) * Gf.Matrix4d().SetTranslate(Gf.Vec3d(*pos))
    cube.AddTransformOp().Set(M)
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    return cube


def add_robot(stage, name, base):
    """Reference irb360.usd (/World/IRB360) under a scaled+translated parent."""
    prim = stage.DefinePrim(f"/World/Cell/{name}", "Xform")
    M = Gf.Matrix4d().SetScale(Gf.Vec3d(MM, MM, MM)) \
        * Gf.Matrix4d().SetTranslate(Gf.Vec3d(*base))
    UsdGeom.Xformable(prim).AddTransformOp().Set(M)
    prim.GetReferences().AddReference(IRB360, Sdf.Path("/World/IRB360"))
    return prim


# --- build the stage --------------------------------------------------------
omni.usd.get_context().new_stage()
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdGeom.Xform.Define(stage, "/World")
UsdGeom.Xform.Define(stage, "/World/Cell")

import math as _m  # noqa: E402

STEEL = (0.10, 0.11, 0.13)      # black machine-frame steel
GREEN = (0.20, 0.55, 0.25)      # belt surface
DECK_Z = BASE_Z                 # robots mount on the top deck
FR_L, FR_W = 2.9, 1.5           # enclosure length (X) x width (Y); wide -> arms clear
c = 0.09                        # structural member thickness

box(stage, "/World/Floor", (3.6, 2.4, 0.02), (0.0, 0.0, 0.0), color=(0.32, 0.33, 0.36))

# conveyors: green belts, box belt low + product belt one foot higher
BELT_H = 0.14
box(stage, "/World/BoxConveyor", (BELT_LEN, 0.42, BELT_H),
    (0.0, BOX_Y, BOX_TOP - BELT_H / 2), color=GREEN)
box(stage, "/World/SrcConveyor", (BELT_LEN, 0.34, BELT_H),
    (0.0, SRC_Y, SRC_TOP - BELT_H / 2), color=GREEN)
for i, sx in enumerate((-1.1, 0.0, 1.1)):   # stands under the elevated product belt
    box(stage, f"/World/SrcStand_{i}", (0.08, 0.08, SRC_TOP - BELT_H),
        (sx, SRC_Y, (SRC_TOP - BELT_H) / 2), color=STEEL)

# --- machine enclosure (black steel cage; robots on the top deck) ----------
COL_X = (-FR_L / 2, 0.0, FR_L / 2)   # columns at ends + centre; robots (+/-0.7) sit between, clear
COL_Y = (-FR_W / 2, FR_W / 2)
k = 0
for cx in COL_X:                     # vertical columns floor -> deck
    for cy in COL_Y:
        box(stage, f"/World/Frame_col_{k}", (c, c, DECK_Z), (cx, cy, DECK_Z / 2), color=STEEL)
        k += 1
for i, cy in enumerate(COL_Y):       # top deck perimeter (X beams) -- open under each robot
    box(stage, f"/World/Frame_topX_{i}", (FR_L, c, c), (0.0, cy, DECK_Z), color=STEEL)
for i, cx in enumerate(COL_X):       # top deck cross members (Y beams) at the columns
    box(stage, f"/World/Frame_topY_{i}", (c, FR_W, c), (cx, 0.0, DECK_Z), color=STEEL)
for j, rz in enumerate((0.55, 1.05)):   # side rails at two heights (cage look)
    for i, cy in enumerate(COL_Y):
        box(stage, f"/World/Frame_rail_{j}_{i}", (FR_L, c, c), (0.0, cy, rz), color=STEEL)

# --- 3-point robot mounts on the deck (delta bolts via its 3 motor axes) ----
MOUNT_R = 0.20
for name, cfg in ROBOTS.items():
    rx = cfg["base"][0]
    for a, phi in enumerate((0.0, 120.0, 240.0)):     # pads at the 3 arm azimuths
        px = rx + MOUNT_R * _m.cos(_m.radians(phi))
        py = MOUNT_R * _m.sin(_m.radians(phi))
        box(stage, f"/World/Mount_{name}_{a}", (0.15, 0.12, 0.05),
            (px, py, DECK_Z + 0.025), color=(0.16, 0.17, 0.19))

# a few parts on the product belt + boxes on the box belt (visual scale)
for i, x in enumerate((-ROBOT_X, 0.0, ROBOT_X)):
    box(stage, f"/World/Part_{i}", (0.06, 0.06, 0.06), (x, SRC_Y, PART_Z), color=(0.85, 0.5, 0.2))
for i, x in enumerate((-ROBOT_X, ROBOT_X)):
    box(stage, f"/World/Box_{i}", (0.20, 0.20, 0.16), (x, BOX_Y, BOX_TOP + 0.08), color=(0.5, 0.35, 0.2))

bases = {}
for name, cfg in ROBOTS.items():
    add_robot(stage, name, cfg["base"])
    bm = UsdGeom.Xformable(stage.GetPrimAtPath(f"/World/Cell/{name}")) \
        .ComputeLocalToWorldTransform(0)
    bases[name] = bm

for _ in range(60):
    app.update()

# pose robots at both reach extremes: A picks from the product belt, B places
# over a box on the box belt (world target -> robot local mm)
for name, cfg in ROBOTS.items():
    bx = cfg["base"][0]
    if name == "Robot_A":
        tgt = (bx, SRC_Y, PART_Z)             # pick from product belt (higher)
    else:
        tgt = (bx, BOX_Y, BOX_TOP + 0.18)     # place over a box (lower)
    T_local = world_to_local(bases[name], tgt)
    pose(stage, f"/World/Cell/{name}", T_local)
    print(f"  {name}: tgt_world=({tgt[0]:.2f},{tgt[1]:.2f},{tgt[2]:.2f})  "
          f"T_local(mm)=({T_local[0]:.0f},{T_local[1]:.0f},{T_local[2]:.0f})")

# lights + render -----------------------------------------------------------
UsdLux.DomeLight.Define(stage, "/World/Light_Dome").CreateIntensityAttr(700.0)
UsdLux.DistantLight.Define(stage, "/World/Light_Key").CreateIntensityAttr(2500.0)

import omni.replicator.core as rep  # noqa: E402

cam = rep.create.camera(position=(3.9, -4.3, 2.7), look_at=(0.0, 0.0, 0.8))
rp = rep.create.render_product(cam, (1280, 720))
rgb = rep.AnnotatorRegistry.get_annotator("rgb")
rgb.attach([rp])
for _ in range(6):
    rep.orchestrator.step(rt_subframes=16)
arr = np.asarray(rgb.get_data())
try:
    from PIL import Image
    Image.fromarray(arr[:, :, :3].astype("uint8")).save(OUT_PNG)
except Exception as exc:
    print(f"[render] save failed: {exc}")
print(f"\n[cell] wrote {OUT_PNG}  exists={os.path.exists(OUT_PNG)}  "
      f"shape={getattr(arr,'shape',None)}\n")

app.close()
