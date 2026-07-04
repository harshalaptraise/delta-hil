"""Animate the articulated IRB 360 through a pick-and-place cycle -> GIF.

Reuses the solved kinematics (arms IK + forearm frame-tracking + 4th-axis shaft)
and drives the plate along a pick/place trajectory, rendering each frame and
assembling assets/render/irb360_pick.gif.

Run on the rig (inside isaacenv):  python scripts/animate_irb360.py
"""
from __future__ import annotations

import os

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from pxr import UsdLux  # noqa: E402

from deltahil.plant.irb360_pose import pose  # noqa: E402  (kinematics, shared)

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
USD = os.path.join(REPO, "assets", "irb360.usd").replace("\\", "/")
RENDER_DIR = os.path.join(REPO, "assets", "render").replace("\\", "/")
OUT_GIF = os.path.join(RENDER_DIR, "irb360_pick.gif").replace("\\", "/")
os.makedirs(RENDER_DIR, exist_ok=True)

def trajectory():
    """Pick-and-place waypoints (plate centre, mm), linearly interpolated."""
    z_hi, z_lo = -1050.0, -1180.0
    pick, place = np.array([230.0, -140.0, 0]), np.array([-230.0, 150.0, 0])
    home = np.array([-10.0, 0.0, z_lo])
    wp = [home,
          pick + [0, 0, z_hi], pick + [0, 0, z_lo], pick + [0, 0, z_hi],   # pick
          place + [0, 0, z_hi], place + [0, 0, z_lo], place + [0, 0, z_hi],  # place
          home]
    frames = []
    for a, b in zip(wp[:-1], wp[1:]):
        for k in range(6):
            frames.append(a + (b - a) * (k / 6.0))
    frames.append(wp[-1])
    return frames


# open + lights + camera + render product (once) ---------------------------
omni.usd.get_context().open_stage(USD)
for _ in range(60):
    app.update()
stage = omni.usd.get_context().get_stage()
UsdLux.DomeLight.Define(stage, "/World/Light_Dome").CreateIntensityAttr(700.0)
UsdLux.DistantLight.Define(stage, "/World/Light_Key").CreateIntensityAttr(2500.0)

import omni.replicator.core as rep  # noqa: E402

cam = rep.create.camera(position=(2600, -3000, 400), look_at=(0, 0, -650))
rp = rep.create.render_product(cam, (900, 600))
rgb = rep.AnnotatorRegistry.get_annotator("rgb")
rgb.attach([rp])

from PIL import Image  # noqa: E402

# warm up the render graph so the first get_data() returns a real frame
for _ in range(12):
    rep.orchestrator.step(rt_subframes=8)


def capture():
    for _ in range(6):
        rep.orchestrator.step(rt_subframes=8)
        a = np.asarray(rgb.get_data())
        if a.ndim == 3 and a.size and a.shape[2] >= 3:
            return a[:, :, :3].astype("uint8")
    return None


frames = trajectory()
print(f"[animate] rendering {len(frames)} frames ...")
imgs = []
for idx, T in enumerate(frames):
    pose(stage, "/World/IRB360", T)
    arr = capture()
    if arr is None:
        print(f"  frame {idx+1} skipped (no data)")
        continue
    imgs.append(Image.fromarray(arr))
    if idx % 6 == 0:
        print(f"  frame {idx+1}/{len(frames)}  plate=({T[0]:.0f},{T[1]:.0f},{T[2]:.0f})")

if imgs:
    imgs[0].save(OUT_GIF, save_all=True, append_images=imgs[1:], duration=80, loop=0)
    print(f"\n[animate] wrote {OUT_GIF}  exists={os.path.exists(OUT_GIF)}  frames={len(imgs)}\n")
else:
    print("\n[animate] no frames captured\n")

app.close()
