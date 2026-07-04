"""Two-robot IRB 360 cell -- continuous production with conveyor tracking.

Tortillas and boxes stream in from the left, travel the belts, and exit the
right. Each robot reactively picks an arriving tortilla (velocity-matched, on the
fly) and places it into a passing box (also velocity-matched). A picked tortilla
vanishes from the belt at the pick; an un-picked one rides off the end. Boxes
fill as they pass and exit full. No scene reset -- it just keeps running.

Reuses deltahil.plant.cell_scene + irb360_pose.pose. Output: cell_pick.gif.
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
random.seed(7)

# --- run + belts (metres, frames) ------------------------------------------
F = 500                 # total frames (raise for more boxes; ~linear render cost)
Tc = 24                 # frames per pick-place cycle
VS = 0.011              # product belt velocity (+X, m/frame)
VB = 0.0075             # box belt velocity (+X, m/frame)
DT_TORT = 20            # tortilla spawn interval -> ~0.5-diameter gap between products
DT_BOX = 42             # box spawn interval -> ~0.32 m spacing > 0.26 m tote (no overlap)
XL = -cs.BELT_LEN / 2 - 0.15
XR = cs.BELT_LEN / 2 + 0.05
HIDE = (5.0, 3.0, -3.0)   # park unused/exited prims off-camera

HOME_Z = 0.42
PICK_Z = cs.PART_Z + 0.005
PICK_HI = cs.PART_Z + 0.10
PLACE_HI = cs.BOX_TOP + 0.30
STACK0 = cs.BOX_TOP + 0.02
THICK = 0.014
DESC = 0.6
PH = [("rest", 0.05), ("approach", 0.22), ("track_pick", 0.46), ("lift", 0.56),
      ("transfer", 0.70), ("track_place", 0.92), ("home", 1.0)]
F_PICK = 0.22 + DESC * (0.46 - 0.22)
F_PLACE = 0.70 + DESC * (0.92 - 0.70)
PICK_TOL = 0.09          # start a cycle when a tortilla is within this of the ideal x


def phase_of(tau):
    prev = 0.0
    for name, end in PH:
        if tau <= end:
            return name, (tau - prev) / max(end - prev, 1e-9)
        prev = end
    return "home", 1.0


def tort_x(t, f):
    return XL + VS * (f - t["spawn"])


def box_x(b, f):
    return XL + VB * (f - b["spawn"])


def tcp_for(rb, f):
    rx = rb["rx"]
    home = np.array([rx, 0.0, HOME_Z])
    name, u = phase_of((f - rb["s"]) / Tc)
    tx = tort_x(rb["tort"], f)            # track the tortilla's ACTUAL belt position
    bx = box_x(rb["box"], f) if rb["box"] else rx
    yj, sz = rb["yj"], STACK0 + rb["slot"] * THICK
    if name == "rest":
        return home
    if name == "approach":
        b = np.array([tx, yj, PICK_HI]); return home + (b - home) * u
    if name == "track_pick":
        return np.array([tx, yj, PICK_HI + (PICK_Z - PICK_HI) * min(u / DESC, 1.0)])
    if name == "lift":                   # release tracking; lift from the grab point
        return np.array([rb["grab_x"], yj, PICK_Z + (PICK_HI - PICK_Z) * u])
    if name == "transfer":
        a = np.array([rb["grab_x"], yj, PICK_HI]); b = np.array([bx, cs.BOX_Y, PLACE_HI])
        return a + (b - a) * u
    if name == "track_place":
        return np.array([bx, cs.BOX_Y, PLACE_HI + (sz - PLACE_HI) * min(u / DESC, 1.0)])
    a = np.array([bx, cs.BOX_Y, sz]); mid = np.array([bx, cs.BOX_Y, PLACE_HI])
    return a + (mid - a) * (u / 0.4) if u < 0.4 else mid + (home - mid) * ((u - 0.4) / 0.6)


# --- build the stage + prim pools ------------------------------------------
omni.usd.get_context().new_stage()
stage = omni.usd.get_context().get_stage()
bases = cs.build_cell(stage, IRB360)

N_TORT = F // DT_TORT + 25   # extra: placed tortillas ride boxes until the box exits
N_BOX = F // DT_BOX + 6
for i in range(N_TORT):
    cs.spawn_tortilla(stage, f"/World/T_{i}", HIDE)
for i in range(N_BOX):
    cs.spawn_box(stage, f"/World/B_{i}", HIDE)
free_t, free_b = list(range(N_TORT)), list(range(N_BOX))
torts, boxes = [], []       # active items
robots = {name: {"rx": rx, "state": "idle", "box": None}
          for name, (rx, _, _) in cs.ROBOTS.items()}

for _ in range(60):
    app.update()
UsdLux.DomeLight.Define(stage, "/World/Light_Dome").CreateIntensityAttr(700.0)
UsdLux.DistantLight.Define(stage, "/World/Light_Key").CreateIntensityAttr(2500.0)

import omni.replicator.core as rep  # noqa: E402
from PIL import Image  # noqa: E402

cam = rep.create.camera(position=(2.0, 3.8, 2.6), look_at=(0.0, 0.0, 0.5))
rp = rep.create.render_product(cam, (860, 540))   # smaller -> manageable 500-frame gif
rgb = rep.AnnotatorRegistry.get_annotator("rgb")
rgb.attach([rp])
for _ in range(12):
    rep.orchestrator.step(rt_subframes=12)


def capture():
    for _ in range(6):
        rep.orchestrator.step(rt_subframes=12)
        a = np.asarray(rgb.get_data())
        if a.ndim == 3 and a.size and a.shape[2] >= 3:
            return a[:, :, :3].astype("uint8")
    return None


print(f"[cell] streaming {F} frames ...")
imgs = []
spawn_count = 0
for f in range(F):
    # spawn: alternate which robot 'prefers' each tortilla (A upstream / B downstream)
    if f % DT_TORT == 0 and free_t:
        pref = "Robot_A" if spawn_count % 2 == 0 else "Robot_B"
        torts.append({"idx": free_t.pop(0), "spawn": f, "state": "belt", "pref": pref,
                      "lane": cs.SRC_Y + random.uniform(-0.05, 0.05)})
        spawn_count += 1
    if f % DT_BOX == 0 and free_b:
        boxes.append({"idx": free_b.pop(0), "spawn": f, "fill": 0})

    # robot state machines
    for name, rb in robots.items():
        rx = rb["rx"]
        if rb["state"] == "busy":
            dt = f - rb["s"]
            if not rb["grabbed"] and dt >= F_PICK * Tc:
                rb["grabbed"] = True; rb["tort"]["state"] = "carried"
            if not rb["placed"] and dt >= F_PLACE * Tc:
                rb["placed"] = True
                bx = rb["box"]
                rb["tort"]["state"] = "placed"; rb["tort"]["box"] = bx
                rb["tort"]["slot"] = rb["slot"]; bx["torts"] = bx.get("torts", []) + [rb["tort"]]
            if dt >= Tc:
                rb["state"] = "idle"; rb["box"] = None
        if rb["state"] == "idle":
            want = rx - VS * F_PICK * Tc               # tortilla should be here to start now
            # A takes its own share; B (downstream) takes its own + anything A missed
            pool = [t for t in torts if t["state"] == "belt"
                    and abs(tort_x(t, f) - want) < PICK_TOL
                    and (name == "Robot_B" or t["pref"] == "Robot_A")]
            cand = min(pool, key=lambda t: abs(tort_x(t, f) - want), default=None)
            # a box that will be near rx at place time
            place_f = f + F_PLACE * Tc
            bx = min((b for b in boxes if box_x(b, place_f) < XR),
                     key=lambda b: abs(box_x(b, place_f) - rx), default=None)
            if cand and bx and abs(box_x(bx, place_f) - rx) < 0.36:
                rb.update({"state": "busy", "s": f, "tp": f + F_PICK * Tc,
                           "tpl": f + F_PLACE * Tc, "yj": cand["lane"],
                           "tort": cand, "box": bx, "slot": bx["fill"],
                           "grab_x": tort_x(cand, f + F_PICK * Tc),  # tortilla x at grab
                           "grabbed": False, "placed": False})
                cand["state"] = "assigned"; bx["fill"] += 1

    # poses + carried-tortilla tracking
    for name, rb in robots.items():
        if rb["state"] == "busy":
            tcp = tcp_for(rb, f)
            pose(stage, f"/World/Cell/{name}", world_to_local(bases[name], tcp))
            if rb["tort"]["state"] == "carried":
                rb["tort"]["cpos"] = tuple(tcp + np.array([0, 0, -0.02]))
        else:
            pose(stage, f"/World/Cell/{name}",
                 world_to_local(bases[name], (rb["rx"], 0.0, HOME_Z)))

    # advance + place items; set prim positions; retire exited
    for b in list(boxes):
        bx = box_x(b, f)
        if bx > XR:
            cs.move_prim(stage, f"/World/B_{b['idx']}", HIDE)
            for t in b.get("torts", []):
                cs.move_prim(stage, f"/World/T_{t['idx']}", HIDE); free_t.append(t["idx"]); (t in torts) and torts.remove(t)
            free_b.append(b["idx"]); boxes.remove(b)
        else:
            cs.move_prim(stage, f"/World/B_{b['idx']}", (bx, cs.BOX_Y, cs.BOX_TOP))
    for t in list(torts):
        st = t["state"]
        if st in ("belt", "assigned"):
            x = tort_x(t, f)
            if st == "belt" and x > XR:            # rode off the end un-picked
                cs.move_prim(stage, f"/World/T_{t['idx']}", HIDE); free_t.append(t["idx"]); torts.remove(t); continue
            cs.move_prim(stage, f"/World/T_{t['idx']}", (x, t["lane"], cs.PART_Z + 0.006))
        elif st == "carried":
            cs.move_prim(stage, f"/World/T_{t['idx']}", t.get("cpos", HIDE))
        elif st == "placed":
            cs.move_prim(stage, f"/World/T_{t['idx']}",
                         (box_x(t["box"], f), cs.BOX_Y, STACK0 + t["slot"] * THICK))

    arr = capture()
    if arr is None:
        print(f"  frame {f+1} skipped"); continue
    imgs.append(Image.fromarray(arr))
    if f % 20 == 0:
        print(f"  frame {f+1}/{F}  torts={len(torts)} boxes={len(boxes)}")

if imgs:
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
