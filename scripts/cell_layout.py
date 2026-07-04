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
ROBOTS = {
    "Robot_A": {"base": (-0.45, 0.0, 1.35)},
    "Robot_B": {"base": (0.45, 0.0, 1.35)},
}
SRC_Y, BOX_Y = -0.20, 0.20          # source & box conveyor centre-lines (world y)
CONV_Z = 0.10                        # conveyor top surface height
PART_Z = CONV_Z + 0.03               # part sits on the source conveyor


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

box(stage, "/World/Floor", (3.0, 2.0, 0.02), (0.0, 0.0, 0.0), color=(0.35, 0.35, 0.38))
box(stage, "/World/SrcConveyor", (2.4, 0.3, 0.2), (0.0, SRC_Y, CONV_Z - 0.1), color=(0.25, 0.28, 0.32))
box(stage, "/World/BoxConveyor", (2.4, 0.35, 0.2), (0.0, BOX_Y, CONV_Z - 0.1), color=(0.25, 0.28, 0.32))

# a few parts on the source conveyor + boxes on the box conveyor (visual scale)
for i, x in enumerate((-0.45, 0.0, 0.45)):
    box(stage, f"/World/Part_{i}", (0.06, 0.06, 0.06), (x, SRC_Y, PART_Z), color=(0.8, 0.5, 0.2))
for i, x in enumerate((-0.45, 0.45)):
    box(stage, f"/World/Box_{i}", (0.18, 0.18, 0.12), (x, BOX_Y, CONV_Z + 0.06), color=(0.5, 0.35, 0.2))

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

cam = rep.create.camera(position=(2.6, -3.0, 2.2), look_at=(0.0, 0.0, 0.6))
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
