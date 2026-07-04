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

box(stage, "/World/Floor", (3.4, 2.4, 0.02), (0.0, 0.0, 0.0), color=(0.35, 0.35, 0.38))

# conveyors (belt-top at *_TOP; a 0.15 m-tall belt body sits below it)
BELT_H = 0.15
box(stage, "/World/BoxConveyor", (BELT_LEN, 0.40, BELT_H),
    (0.0, BOX_Y, BOX_TOP - BELT_H / 2), color=(0.25, 0.28, 0.32))
box(stage, "/World/SrcConveyor", (BELT_LEN, 0.34, BELT_H),
    (0.0, SRC_Y, SRC_TOP - BELT_H / 2), color=(0.25, 0.28, 0.32))
# stands under the elevated source conveyor
for i, sx in enumerate((-1.0, 0.0, 1.0)):
    box(stage, f"/World/SrcStand_{i}", (0.08, 0.08, SRC_TOP - BELT_H),
        (sx, SRC_Y, (SRC_TOP - BELT_H) / 2), color=(0.22, 0.24, 0.27))

# portal gantry: 4 legs + top perimeter beams + a central mount beam
GY, GX, GTZ = 0.5, 0.95, BASE_Z + 0.12
for i, (lx, ly) in enumerate([(-GX, -GY), (-GX, GY), (GX, -GY), (GX, GY)]):
    box(stage, f"/World/Gantry_leg_{i}", (0.07, 0.07, GTZ),
        (lx, ly, GTZ / 2), color=(0.3, 0.32, 0.36))
for i, ly in enumerate((-GY, GY)):
    box(stage, f"/World/Gantry_beamX_{i}", (2 * GX, 0.09, 0.09), (0.0, ly, GTZ),
        color=(0.3, 0.32, 0.36))
for i, lx in enumerate((-GX, GX)):
    box(stage, f"/World/Gantry_beamY_{i}", (0.09, 2 * GY, 0.09), (lx, 0.0, GTZ),
        color=(0.3, 0.32, 0.36))
box(stage, "/World/Gantry_mount", (2 * GX, 0.14, 0.12), (0.0, 0.0, BASE_Z + 0.05),
    color=(0.28, 0.30, 0.34))

# a few parts on the source belt + boxes on the box belt (visual scale)
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

# pose each robot reaching the source pick point under it (world -> local mm)
for name, cfg in ROBOTS.items():
    bx = cfg["base"][0]
    pick_world = (bx, SRC_Y, PART_Z)
    T_local = world_to_local(bases[name], pick_world)
    pose(stage, f"/World/Cell/{name}", T_local)
    print(f"  {name}: pick_world={pick_world}  T_local(mm)="
          f"({T_local[0]:.0f},{T_local[1]:.0f},{T_local[2]:.0f})")

# lights + render -----------------------------------------------------------
UsdLux.DomeLight.Define(stage, "/World/Light_Dome").CreateIntensityAttr(700.0)
UsdLux.DistantLight.Define(stage, "/World/Light_Key").CreateIntensityAttr(2500.0)

import omni.replicator.core as rep  # noqa: E402

cam = rep.create.camera(position=(3.4, -3.8, 2.5), look_at=(0.0, 0.0, 0.85))
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
