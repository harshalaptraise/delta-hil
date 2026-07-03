"""Articulation step 1: rotate the three arms about their motor pivots and render.

Validates the motor pivot positions + axes + per-arm part grouping before we add
forearm tracking and plate motion. Each 'leg' (upper arm + its forearm rods) is
rotated rigidly about the estimated motor pivot by TEST_ANGLE_DEG. If the legs
pivot cleanly at the base and tilt symmetrically, the pivots/axes are right.

Run on the rig (inside isaacenv):  python scripts/pose_irb360.py
Output: assets/render/pose.png
"""
from __future__ import annotations

import math
import os

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
USD = os.path.join(REPO, "assets", "irb360.usd").replace("\\", "/")
RENDER_DIR = os.path.join(REPO, "assets", "render").replace("\\", "/")
OUT_PNG = os.path.join(RENDER_DIR, "pose.png").replace("\\", "/")
os.makedirs(RENDER_DIR, exist_ok=True)

# --- estimated motor geometry (mm), from the CAD part bounding boxes ---------
R_MOTOR = 175.0      # base-hinge radius from the central axis
Z_MOTOR = -263.0     # hinge height (upper-arm center z)
TEST_ANGLE_DEG = 30.0  # +ve should tilt the arms UP (plate rises)

# arm angle (deg) around Z, and the top-level prim names belonging to each leg
ARMS = {
    1: {"phi": 0.0,   "prims": ["UA1", "LA1_CL", "LA1_CU", "LA1_1", "LA1_2"]},
    2: {"phi": 240.0, "prims": ["UA2", "LA2_CL", "LA2_CU", "LA2_1", "LA2_2"]},
    3: {"phi": 120.0, "prims": ["UA3", "LA3_CL", "LA3_CU", "LA3_1", "LA3_2"]},
}


def rot_about(pivot, axis, deg) -> Gf.Matrix4d:
    """Row-vector matrix for rotating about `axis` through `pivot` (v' = v*M)."""
    Tn = Gf.Matrix4d().SetTranslate(Gf.Vec3d(-pivot[0], -pivot[1], -pivot[2]))
    Rm = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(*axis), deg))
    Tp = Gf.Matrix4d().SetTranslate(Gf.Vec3d(*pivot))
    return Tn * Rm * Tp


omni.usd.get_context().open_stage(USD)
for _ in range(60):
    app.update()
stage = omni.usd.get_context().get_stage()

for i, arm in ARMS.items():
    phi = math.radians(arm["phi"])
    pivot = (R_MOTOR * math.cos(phi), R_MOTOR * math.sin(phi), Z_MOTOR)
    axis = (-math.sin(phi), math.cos(phi), 0.0)     # tangent to the base circle
    M = rot_about(pivot, axis, TEST_ANGLE_DEG)
    for name in arm["prims"]:
        prim = stage.GetPrimAtPath(f"/World/IRB360/{name}")
        if not prim or not prim.IsValid():
            print(f"  (missing prim {name})")
            continue
        UsdGeom.Xformable(prim).ClearXformOpOrder()
        UsdGeom.Xformable(prim).AddTransformOp().Set(M)
    print(f"  arm {i}: pivot=({pivot[0]:.0f},{pivot[1]:.0f},{pivot[2]:.0f}) "
          f"axis=({axis[0]:.2f},{axis[1]:.2f},0) rot={TEST_ANGLE_DEG}deg")

# lights + render -----------------------------------------------------------
UsdLux.DomeLight.Define(stage, "/World/Light_Dome").CreateIntensityAttr(700.0)
UsdLux.DistantLight.Define(stage, "/World/Light_Key").CreateIntensityAttr(2500.0)

import omni.replicator.core as rep  # noqa: E402

cam = rep.create.camera(position=(2600, -3000, 400), look_at=(150, 0, -580))
rp = rep.create.render_product(cam, (1280, 720))
rgb = rep.AnnotatorRegistry.get_annotator("rgb")
rgb.attach([rp])
for _ in range(5):
    rep.orchestrator.step(rt_subframes=16)
arr = np.asarray(rgb.get_data())
try:
    from PIL import Image
    Image.fromarray(arr[:, :, :3].astype("uint8")).save(OUT_PNG)
except Exception as exc:
    print(f"[render] save failed: {exc}")
print(f"[render] wrote {OUT_PNG}  exists={os.path.exists(OUT_PNG)}\n")

app.close()
