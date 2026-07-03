"""Animate the articulated IRB 360 through a pick-and-place cycle -> GIF.

Reuses the solved kinematics (arms IK + forearm frame-tracking + 4th-axis shaft)
and drives the plate along a pick/place trajectory, rendering each frame and
assembling assets/render/irb360_pick.gif.

Run on the rig (inside isaacenv):  python scripts/animate_irb360.py
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
OUT_GIF = os.path.join(RENDER_DIR, "irb360_pick.gif").replace("\\", "/")
os.makedirs(RENDER_DIR, exist_ok=True)

# geometry (mm) from the CAD
R_MOTOR, Z_MOTOR = 175.0, -263.0
HOME_E1 = np.array([682.0, 0.0, -299.0])
HOME_ATTACH1 = np.array([66.0, 0.0, -1144.0])
HOME_PLATE = np.array([-10.0, 0.0, -1187.0])
ATTACH_OFF1 = HOME_ATTACH1 - HOME_PLATE
RF = float(np.linalg.norm(HOME_E1 - np.array([R_MOTOR, 0, Z_MOTOR])))
RE = float(np.linalg.norm(HOME_ATTACH1 - HOME_E1))
P_TOP = np.array([0.0, 0.0, -148.0])
P_BOT_HOME = np.array([0.0, 0.0, -1178.0])
ARMS = {1: 0.0, 2: 240.0, 3: 120.0}
ARM_LA = {1: ["LA1_CL", "LA1_CU", "LA1_1", "LA1_2"],
          2: ["LA2_CL", "LA2_CU", "LA2_1", "LA2_2"],
          3: ["LA3_CL", "LA3_CU", "LA3_1", "LA3_2"]}
CENTRAL = {"RevoluteLinkUpper": -577.0, "RevoluteLinkLower": -745.0,
           "RevoluteLinkPlate": -1178.0}


def rotz(p, phi_deg):
    c, s = math.cos(math.radians(phi_deg)), math.sin(math.radians(phi_deg))
    return np.array([p[0] * c - p[1] * s, p[0] * s + p[1] * c, p[2]])


def _frame(d, w):
    d = d / np.linalg.norm(d)
    w = w - (w @ d) * d
    w = w / np.linalg.norm(w)
    return np.array([d, w, np.cross(d, w)])


def solve_elbow(P, u_r, attach, home_E):
    z = np.array([0, 0, 1.0])
    d = attach - P
    a, b = float(d @ u_r), float(d @ z)
    Rr = math.hypot(a, b)
    c = max(-1.0, min(1.0, (RF * RF + a * a + b * b - RE * RE) / (2 * RF * Rr)))
    psi = math.atan2(b, a)
    best, bestd = None, 1e18
    for sign in (+1, -1):
        theta = -psi + sign * math.acos(c)
        E = P + RF * (math.cos(theta) * u_r - math.sin(theta) * z)
        dd = float(np.linalg.norm(E - home_E))
        if dd < bestd:
            best, bestd = E, dd
    return best


def m_rot_map(pivot, v_from, v_to):
    r = Gf.Rotation()
    r.SetRotateInto(Gf.Vec3d(*v_from.tolist()), Gf.Vec3d(*v_to.tolist()))
    P = Gf.Vec3d(*pivot.tolist())
    return Gf.Matrix4d().SetTranslate(-P) * Gf.Matrix4d().SetRotate(r) \
        * Gf.Matrix4d().SetTranslate(P)


def m_frame_map(p0f, p1f, p0t, p1t, width):
    R = _frame(p1f - p0f, width).T @ _frame(p1t - p0t, width)
    Rm = Gf.Matrix4d(Gf.Matrix3d(*R.flatten().tolist()), Gf.Vec3d(0, 0, 0))
    return Gf.Matrix4d().SetTranslate(Gf.Vec3d(*(-p0f).tolist())) * Rm \
        * Gf.Matrix4d().SetTranslate(Gf.Vec3d(*p0t.tolist()))


def set_xform(stage, name, M):
    prim = stage.GetPrimAtPath(f"/World/IRB360/{name}")
    if prim and prim.IsValid():
        UsdGeom.Xformable(prim).ClearXformOpOrder()
        UsdGeom.Xformable(prim).AddTransformOp().Set(M)


def pose(stage, T):
    for i, phi in ARMS.items():
        u_r = rotz(np.array([1.0, 0, 0]), phi)
        P = np.array([R_MOTOR * u_r[0], R_MOTOR * u_r[1], Z_MOTOR])
        A = np.array([-math.sin(math.radians(phi)), math.cos(math.radians(phi)), 0.0])
        home_E, home_attach = rotz(HOME_E1, phi), rotz(HOME_ATTACH1, phi)
        attach = T + rotz(ATTACH_OFF1, phi)
        E = solve_elbow(P, u_r, attach, home_E)
        set_xform(stage, f"UA{i}", m_rot_map(P, home_E - P, E - P))
        Mf = m_frame_map(home_E, home_attach, E, attach, A)
        for la in ARM_LA[i]:
            set_xform(stage, la, Mf)
    set_xform(stage, "MovingPlate", Gf.Matrix4d().SetTranslate(
        Gf.Vec3d(*(T - HOME_PLATE).tolist())))
    rs = Gf.Rotation()
    rs.SetRotateInto(Gf.Vec3d(*(P_BOT_HOME - P_TOP).tolist()),
                     Gf.Vec3d(*(T - P_TOP).tolist()))
    Rm = Gf.Matrix4d().SetRotate(rs)
    for name, z_home in CENTRAL.items():
        t = (z_home - P_TOP[2]) / (P_BOT_HOME[2] - P_TOP[2])
        Ch = np.array([0.0, 0.0, z_home])
        Cn = P_TOP + t * (T - P_TOP)
        set_xform(stage, name, Gf.Matrix4d().SetTranslate(Gf.Vec3d(*(-Ch).tolist()))
                  * Rm * Gf.Matrix4d().SetTranslate(Gf.Vec3d(*Cn.tolist())))


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
    pose(stage, T)
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
