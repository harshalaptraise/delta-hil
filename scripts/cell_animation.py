"""Two-robot IRB 360 cell: both robots pick tortillas off the product belt and
place them into boxes on the box belt -> assets/render/cell_pick.gif.

Reuses the shared cell (deltahil.plant.cell_scene) and the solved kinematics
(irb360_pose.pose, per robot, in each robot's local frame). Tortillas are thin
discs that queue on the product belt, get carried on grasp, and stack in the box.

Run on the rig (inside isaacenv):  python scripts/cell_animation.py
"""
from __future__ import annotations

import os

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from pxr import UsdLux  # noqa: E402

from deltahil.plant import cell_scene as cs  # noqa: E402
from deltahil.plant.irb360_pose import pose, world_to_local  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IRB360 = os.path.join(REPO, "assets", "irb360.usd").replace("\\", "/")
RENDER_DIR = os.path.join(REPO, "assets", "render").replace("\\", "/")
OUT_GIF = os.path.join(RENDER_DIR, "cell_pick.gif").replace("\\", "/")
os.makedirs(RENDER_DIR, exist_ok=True)

# --- animation params (metres) ---------------------------------------------
N_CYCLES = 3            # tortillas each robot picks
FPS = 3                 # interpolation frames per motion segment
THICK = 0.014          # tortilla stack thickness
HOME_Z = 0.40          # plate rest height (mid)
PICK_Z = cs.PART_Z + 0.02
PICK_HI = cs.PART_Z + 0.20
PLACE_HI = cs.BOX_TOP + 0.34
STACK0 = cs.BOX_TOP + 0.09   # first tortilla height in the box


def cycle_points(rx, stack_z):
    return [(rx, 0.0, HOME_Z),
            (rx, cs.SRC_Y, PICK_HI), (rx, cs.SRC_Y, PICK_Z),   # to pick
            (rx, cs.SRC_Y, PICK_HI), (rx, cs.BOX_Y, PLACE_HI),  # carry over
            (rx, cs.BOX_Y, stack_z), (rx, 0.0, HOME_Z)]          # place, home


def robot_state(rx, n_torts, frame_i):
    """(plate_target_world, {tortilla_index: world_pos}) for this robot/frame."""
    per_cycle = 6 * FPS
    cyc = min(frame_i // per_cycle, n_torts - 1)
    lf = frame_i - cyc * per_cycle
    si, k = lf // FPS, lf % FPS
    stack_z = STACK0 + cyc * THICK
    wps = cycle_points(rx, stack_z)
    a, b = np.array(wps[si]), np.array(wps[min(si + 1, 6)])
    target = a + (b - a) * (k / FPS)
    carried = 2 <= si <= 4                      # holding this cycle's tortilla
    pos = {}
    for j in range(n_torts):
        if j < cyc:
            pos[j] = (rx, cs.BOX_Y, STACK0 + j * THICK)         # already boxed
        elif j == cyc:
            pos[j] = tuple(target) if carried else (rx, cs.SRC_Y, cs.PART_Z + 0.006)
        else:
            pos[j] = (rx - 0.14 * (j - cyc), cs.SRC_Y, cs.PART_Z + 0.006)  # queued
    return target, pos


# --- build the stage --------------------------------------------------------
omni.usd.get_context().new_stage()
stage = omni.usd.get_context().get_stage()
bases = cs.build_cell(stage, IRB360)

TORT = {name: [f"/World/Tortilla_{name}_{j}" for j in range(N_CYCLES)] for name in cs.ROBOTS}
for name, (rx, _, _) in cs.ROBOTS.items():
    cs.spawn_box(stage, f"/World/Box_{name}", (rx, cs.BOX_Y, cs.BOX_TOP + 0.07))
    for j, tp in enumerate(TORT[name]):
        cs.spawn_tortilla(stage, tp, (rx - 0.14 * j, cs.SRC_Y, cs.PART_Z + 0.006))

for _ in range(60):
    app.update()

UsdLux.DomeLight.Define(stage, "/World/Light_Dome").CreateIntensityAttr(700.0)
UsdLux.DistantLight.Define(stage, "/World/Light_Key").CreateIntensityAttr(2500.0)

import omni.replicator.core as rep  # noqa: E402
from PIL import Image  # noqa: E402

cam = rep.create.camera(position=(3.9, -4.3, 2.7), look_at=(0.0, 0.0, 0.8))
rp = rep.create.render_product(cam, (1000, 640))
rgb = rep.AnnotatorRegistry.get_annotator("rgb")
rgb.attach([rp])
for _ in range(12):
    rep.orchestrator.step(rt_subframes=8)


def capture():
    for _ in range(6):
        rep.orchestrator.step(rt_subframes=8)
        a = np.asarray(rgb.get_data())
        if a.ndim == 3 and a.size and a.shape[2] >= 3:
            return a[:, :, :3].astype("uint8")
    return None


n_frames = N_CYCLES * 6 * FPS
print(f"[cell] rendering {n_frames} frames ...")
imgs = []
for f in range(n_frames):
    for name, (rx, _, _) in cs.ROBOTS.items():
        tgt_world, tort_pos = robot_state(rx, N_CYCLES, f)
        T_local = world_to_local(bases[name], tgt_world)
        pose(stage, f"/World/Cell/{name}", T_local)
        for j, p in tort_pos.items():
            cs.move_prim(stage, TORT[name][j], p)
    arr = capture()
    if arr is None:
        print(f"  frame {f+1} skipped")
        continue
    imgs.append(Image.fromarray(arr))
    if f % 6 == 0:
        print(f"  frame {f+1}/{n_frames}")

if imgs:
    imgs[0].save(OUT_GIF, save_all=True, append_images=imgs[1:], duration=90, loop=0)
    print(f"\n[cell] wrote {OUT_GIF}  exists={os.path.exists(OUT_GIF)}  frames={len(imgs)}\n")
else:
    print("\n[cell] no frames captured\n")

app.close()
