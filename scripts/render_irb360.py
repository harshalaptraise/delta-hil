"""Verify the IRB 360 assembly and render a clean front-view PNG.

Run on the rig (inside isaacenv):  python scripts/render_irb360.py

1. Opens assets/irb360.usd and prints the assembled bounding box (mm). If the Z
   extent is ~1200-1300 mm and X/Y ~1000-1800 mm, the parts assembled correctly
   (and the earlier flat-looking view was just a top-down camera angle).
2. Renders a front view to assets/render/rgb_*.png via Replicator (best-effort).
"""
from __future__ import annotations

import os

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import omni.usd  # noqa: E402
from pxr import Usd, UsdGeom  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
USD = os.path.join(REPO, "assets", "irb360.usd").replace("\\", "/")
RENDER_DIR = os.path.join(REPO, "assets", "render").replace("\\", "/")

omni.usd.get_context().open_stage(USD)
for _ in range(60):
    app.update()
stage = omni.usd.get_context().get_stage()

# 1) assembled bounding box ------------------------------------------------
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
rng = cache.ComputeWorldBound(stage.GetPrimAtPath("/World/IRB360")).ComputeAlignedRange()
mn, mx = rng.GetMin(), rng.GetMax()
size = [round(mx[i] - mn[i]) for i in range(3)]
ctr = [round((mx[i] + mn[i]) / 2) for i in range(3)]
print("\n================ ASSEMBLED IRB 360 ================")
print(f"  bbox min = ({mn[0]:.0f}, {mn[1]:.0f}, {mn[2]:.0f}) mm")
print(f"  bbox max = ({mx[0]:.0f}, {mx[1]:.0f}, {mx[2]:.0f}) mm")
print(f"  size     = {tuple(size)} mm   center = {tuple(ctr)} mm")
tall = size[2] > 800
print(f"  verdict  = {'ASSEMBLED (tall Delta)' if tall else 'COLLAPSED (flat)'}")
print("===================================================\n")

# 2) render a front view ---------------------------------------------------
try:
    import omni.replicator.core as rep
    cam = rep.create.camera(position=(0, -3200, -300), look_at=(0, 0, -560))
    rp = rep.create.render_product(cam, (1280, 720))
    writer = rep.WriterRegistry.get("BasicWriter")
    os.makedirs(RENDER_DIR, exist_ok=True)
    writer.initialize(output_dir=RENDER_DIR, rgb=True)
    writer.attach([rp])
    for _ in range(20):
        app.update()
    rep.orchestrator.step()
    for _ in range(20):
        app.update()
    pngs = [f for f in os.listdir(RENDER_DIR) if f.lower().endswith(".png")]
    print(f"[render] wrote {len(pngs)} PNG(s) to {RENDER_DIR}: {pngs}")
except Exception as exc:
    print(f"[render] skipped/failed: {type(exc).__name__}: {exc}")

app.close()
