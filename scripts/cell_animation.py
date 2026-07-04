"""Two-robot IRB 360 cell with CONVEYOR TRACKING.

Both belts move; each robot picks a moving tortilla off the product belt and
places it into a moving box on the box belt, synchronising its TCP to the belt
velocity (pick-on-the-fly / place-on-the-fly). Motion profile per pick:
  rest -> match Y -> match X-velocity (track) -> descend & grab -> release track
  -> transfer -> match box velocity -> descend & place -> return.
Robots run staggered (out of phase); tortillas arrive at random lanes; extra
product flows on the belt for realism.

Reuses deltahil.plant.cell_scene (frame/belts/robots) and irb360_pose.pose (per
robot, local frame). Output: assets/render/cell_pick.gif.

Run on the rig (inside isaacenv):  python scripts/cell_animation.py
"""
from __future__ import annotations

import os
import random

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
random.seed(11)

# --- timing + belts (metres, frames) ---------------------------------------
Tc = 24                 # frames per pick-place cycle (more -> smoother)
N = 2                   # cycles per robot
VS = 0.010              # product belt velocity (+X, m/frame)
VB = 0.007              # box belt velocity (+X, m/frame)
STAGGER = Tc // 2       # robot B is half a cycle out of phase

HOME_Z = 0.42
PICK_Z = cs.PART_Z + 0.01
PICK_HI = cs.PART_Z + 0.10
PLACE_HI = cs.BOX_TOP + 0.30
STACK0 = cs.BOX_TOP + 0.03     # first tortilla just above the tote floor
THICK = 0.014

# phase boundaries as fractions of Tc (grab at end of 'pick', release end of 'place')
PH = [("rest", 0.05), ("approach", 0.34), ("pick", 0.46), ("lift", 0.55),
      ("transfer", 0.72), ("matchbox", 0.82), ("place", 0.93), ("home", 1.0)]
F_PICK, F_PLACE = 0.46, 0.93


def phase_of(tau):
    prev = 0.0
    for name, end in PH:
        if tau <= end:
            return name, (tau - prev) / max(end - prev, 1e-9)
        prev = end
    return "home", 1.0


def box_x(rb, f):
    return rb["box0"] + VB * f


def tcp_for(rb, cy, f):
    rx = rb["rx"]
    home = np.array([rx, 0.0, HOME_Z])
    name, u = phase_of((f - cy["s"]) / Tc)
    tx = rx + VS * (f - cy["tp"])         # tortilla x now (== rx at grab time)
    bx = box_x(rb, f)                     # box x now
    yj, sz = cy["yj"], STACK0 + cy["k"] * THICK
    if name == "rest":
        return home
    if name == "approach":                # home -> above & alongside the tortilla
        b = np.array([tx, yj, PICK_HI]); return home + (b - home) * u
    if name == "pick":                    # track x, descend to grab
        return np.array([tx, yj, PICK_HI + (PICK_Z - PICK_HI) * u])
    if name == "lift":                    # tracking released; lift from grab point
        return np.array([rx, yj, PICK_Z + (PICK_HI - PICK_Z) * u])
    if name == "transfer":                # carry over to the box belt
        a = np.array([rx, yj, PICK_HI]); b = np.array([bx, cs.BOX_Y, PLACE_HI])
        return a + (b - a) * u
    if name == "matchbox":                # sync to box velocity
        return np.array([bx, cs.BOX_Y, PLACE_HI])
    if name == "place":                   # track box, descend, release
        return np.array([bx, cs.BOX_Y, PLACE_HI + (sz - PLACE_HI) * u])
    a = np.array([bx, cs.BOX_Y, PLACE_HI]); return a + (home - a) * u   # home


def eval_robot(rb, f):
    rx = rb["rx"]
    cur = next((cy for cy in rb["cycles"] if cy["s"] <= f < cy["s"] + Tc), None)
    tcp = tcp_for(rb, cur, f) if cur else np.array([rx, 0.0, HOME_Z])
    pos = {}
    for cy in rb["cycles"]:
        if f < cy["tp"]:                                   # travelling to the pick
            pos[cy["tort"]] = (rx + VS * (f - cy["tp"]), cy["yj"], cs.PART_Z + 0.006)
        elif f < cy["tpl"]:                                # carried (tracks the TCP)
            pos[cy["tort"]] = tuple(tcp + np.array([0, 0, -0.02]))
        else:                                              # placed, riding the box
            pos[cy["tort"]] = (box_x(rb, f), cs.BOX_Y, STACK0 + cy["k"] * THICK)
    return tcp, pos


def make_robot(rx, start, tpaths):
    cycles = []
    for k in range(N):
        s = start + k * Tc
        cycles.append({"k": k, "s": s, "tp": s + F_PICK * Tc, "tpl": s + F_PLACE * Tc,
                       "tort": tpaths[k], "yj": cs.SRC_Y + random.uniform(-0.05, 0.05)})
    return {"rx": rx, "cycles": cycles, "box0": rx - 0.10 - VB * cycles[0]["tpl"]}


# --- build the stage --------------------------------------------------------
omni.usd.get_context().new_stage()
stage = omni.usd.get_context().get_stage()
bases = cs.build_cell(stage, IRB360)

robots, TORT = {}, {}
for i, (name, (rx, _, _)) in enumerate(cs.ROBOTS.items()):
    TORT[name] = [f"/World/Tortilla_{name}_{j}" for j in range(N)]
    robots[name] = make_robot(rx, i * STAGGER, TORT[name])
    cs.spawn_box(stage, f"/World/Box_{name}", (robots[name]["box0"], cs.BOX_Y, cs.BOX_TOP))
    for tp in TORT[name]:
        cs.spawn_tortilla(stage, tp, (-cs.BELT_LEN, cs.SRC_Y, cs.PART_Z + 0.006))

# background product flowing on the belt (looping)
BG = [{"path": f"/World/BG_{i}", "off": random.uniform(0, cs.BELT_LEN),
       "y": cs.SRC_Y + random.uniform(-0.06, 0.06)} for i in range(7)]
for bg in BG:
    cs.spawn_tortilla(stage, bg["path"], (0, bg["y"], cs.PART_Z + 0.006))
LEFT = -cs.BELT_LEN / 2

for _ in range(60):
    app.update()

UsdLux.DomeLight.Define(stage, "/World/Light_Dome").CreateIntensityAttr(700.0)
UsdLux.DistantLight.Define(stage, "/World/Light_Key").CreateIntensityAttr(2500.0)

import omni.replicator.core as rep  # noqa: E402
from PIL import Image  # noqa: E402

# view from the +Y (box-belt) side, elevated, so the boxes are nearest the
# observer and the belts run left-right -> conveyor tracking is clearly visible
cam = rep.create.camera(position=(2.0, 3.8, 2.6), look_at=(0.0, 0.0, 0.5))
rp = rep.create.render_product(cam, (1000, 640))
rgb = rep.AnnotatorRegistry.get_annotator("rgb")
rgb.attach([rp])
for _ in range(12):
    rep.orchestrator.step(rt_subframes=16)


def capture():
    for _ in range(6):
        rep.orchestrator.step(rt_subframes=16)
        a = np.asarray(rgb.get_data())
        if a.ndim == 3 and a.size and a.shape[2] >= 3:
            return a[:, :, :3].astype("uint8")
    return None


F = STAGGER + N * Tc + 4
print(f"[cell] rendering {F} frames (conveyor tracking) ...")
imgs = []
for f in range(F):
    for name in cs.ROBOTS:
        rb = robots[name]
        tgt, tort_pos = eval_robot(rb, f)
        pose(stage, f"/World/Cell/{name}", world_to_local(bases[name], tgt))
        cs.move_prim(stage, f"/World/Box_{name}", (box_x(rb, f), cs.BOX_Y, cs.BOX_TOP))
        for tp, p in tort_pos.items():
            cs.move_prim(stage, tp, p)
    for bg in BG:                              # looping background product
        x = LEFT + ((bg["off"] + VS * f - LEFT) % cs.BELT_LEN)
        cs.move_prim(stage, bg["path"], (x, bg["y"], cs.PART_Z + 0.006))
    arr = capture()
    if arr is None:
        print(f"  frame {f+1} skipped")
        continue
    imgs.append(Image.fromarray(arr))
    if f % 6 == 0:
        print(f"  frame {f+1}/{F}")

if imgs:
    # one shared palette + no dithering -> no per-frame colour shimmer (flicker)
    try:
        pal = imgs[len(imgs) // 2].convert("P", palette=Image.ADAPTIVE, colors=128)
        fp = [im.quantize(palette=pal, dither=Image.Dither.NONE) for im in imgs]
        fp[0].save(OUT_GIF, save_all=True, append_images=fp[1:], duration=70, loop=0, disposal=2)
    except Exception as exc:
        print(f"[cell] palette quantize failed ({exc}); saving RGB gif")
        imgs[0].save(OUT_GIF, save_all=True, append_images=imgs[1:], duration=70, loop=0)
    print(f"\n[cell] wrote {OUT_GIF}  exists={os.path.exists(OUT_GIF)}  frames={len(imgs)}\n")
else:
    print("\n[cell] no frames captured\n")

app.close()
