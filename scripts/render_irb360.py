"""Verify the IRB 360 assembly and render a clean lit PNG we can look at.

Run on the rig (inside isaacenv):  python scripts/render_irb360.py
Output: assets/render/irb360.png  (open it / screenshot it to review).
"""
from __future__ import annotations

import os

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Usd, UsdGeom, UsdLux  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
USD = os.path.join(REPO, "assets", "irb360.usd").replace("\\", "/")
RENDER_DIR = os.path.join(REPO, "assets", "render").replace("\\", "/")
OUT_PNG = os.path.join(RENDER_DIR, "irb360.png").replace("\\", "/")
os.makedirs(RENDER_DIR, exist_ok=True)

omni.usd.get_context().open_stage(USD)
for _ in range(60):
    app.update()
stage = omni.usd.get_context().get_stage()

# assembled bbox (sanity) ---------------------------------------------------
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
rng = cache.ComputeWorldBound(stage.GetPrimAtPath("/World/IRB360")).ComputeAlignedRange()
mn, mx = rng.GetMin(), rng.GetMax()
ctr = [(mx[i] + mn[i]) / 2 for i in range(3)]
print(f"\n[render] assembled size = "
      f"({mx[0]-mn[0]:.0f}, {mx[1]-mn[1]:.0f}, {mx[2]-mn[2]:.0f}) mm  "
      f"center=({ctr[0]:.0f},{ctr[1]:.0f},{ctr[2]:.0f})")

# lights (empty scene renders black otherwise) ------------------------------
dome = UsdLux.DomeLight.Define(stage, "/World/Light_Dome")
dome.CreateIntensityAttr(700.0)
key = UsdLux.DistantLight.Define(stage, "/World/Light_Key")
key.CreateIntensityAttr(2500.0)

# render a 3/4 front view via a Replicator annotator ------------------------
import omni.replicator.core as rep  # noqa: E402

cam = rep.create.camera(position=(2600, -3000, 400), look_at=(ctr[0], ctr[1], ctr[2]))
rp = rep.create.render_product(cam, (1280, 720))
rgb = rep.AnnotatorRegistry.get_annotator("rgb")
rgb.attach([rp])

for _ in range(5):
    rep.orchestrator.step(rt_subframes=16)   # converge RTX, drive the writer graph
data = rgb.get_data()
arr = np.asarray(data)
print(f"[render] annotator data shape={getattr(arr,'shape',None)} dtype={getattr(arr,'dtype',None)}")

saved = False
try:
    from PIL import Image
    if arr.ndim == 3 and arr.shape[2] >= 3:
        Image.fromarray(arr[:, :, :3].astype("uint8")).save(OUT_PNG)
        saved = True
except Exception as exc:
    print(f"[render] PIL save failed: {type(exc).__name__}: {exc}")

print(f"[render] wrote {OUT_PNG}  exists={os.path.exists(OUT_PNG)}  (saved={saved})\n")

app.close()
