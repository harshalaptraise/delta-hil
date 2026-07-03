"""Full kinematic articulation of the IRB 360, rendered at a target plate pose.

For a target plate-centre T (mm): each arm rotates about its motor by the IK
angle, its forearm rods pivot at the elbow to reach the plate, and the plate
(+ its RevoluteLinkPlate) translates to T. Geometry is measured from the CAD
(arm 1 elbow/attach points; others by 120deg symmetry). Renders to
assets/render/pose.png.

Run on the rig (inside isaacenv):  python scripts/pose_irb360.py
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

# --- geometry (mm), from the CAD ------------------------------------------
R_MOTOR, Z_MOTOR = 175.0, -263.0
HOME_E1 = np.array([682.0, 0.0, -299.0])       # arm-1 elbow (LA1_CU centre)
HOME_ATTACH1 = np.array([66.0, 0.0, -1144.0])  # arm-1 plate attach (LA1_CL centre)
HOME_PLATE = np.array([-10.0, 0.0, -1187.0])   # MovingPlate centre
ATTACH_OFF1 = HOME_ATTACH1 - HOME_PLATE        # plate->attach offset for arm 1
RF = float(np.linalg.norm(HOME_E1 - np.array([R_MOTOR, 0, Z_MOTOR])))  # ~508
RE = float(np.linalg.norm(HOME_ATTACH1 - HOME_E1))                     # ~1046

# target plate centre (move it up + out to show the reach)
TARGET = HOME_PLATE + np.array([100.0, 0.0, 120.0])

ARMS = {
    1: {"phi": 0.0,   "ua": "UA1", "la": ["LA1_CL", "LA1_CU", "LA1_1", "LA1_2"]},
    2: {"phi": 240.0, "ua": "UA2", "la": ["LA2_CL", "LA2_CU", "LA2_1", "LA2_2"]},
    3: {"phi": 120.0, "ua": "UA3", "la": ["LA3_CL", "LA3_CU", "LA3_1", "LA3_2"]},
}


def rotz(p, phi_deg):
    c, s = math.cos(math.radians(phi_deg)), math.sin(math.radians(phi_deg))
    return np.array([p[0] * c - p[1] * s, p[0] * s + p[1] * c, p[2]])


def solve_elbow(P, u_r, attach, home_E):
    """Delta IK in the arm plane: elbow at distance RF from P (rotating about the
    motor) and RE from the plate attach. Returns the branch nearest home_E."""
    z = np.array([0, 0, 1.0])
    d = attach - P
    a, b = float(d @ u_r), float(d @ z)
    Rr = math.hypot(a, b)
    K = (RF * RF + a * a + b * b - RE * RE) / (2 * RF)
    c = max(-1.0, min(1.0, K / Rr))
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


def _frame(d, w):
    """Orthonormal frame (rows) from a primary direction d and a width hint w."""
    d = d / np.linalg.norm(d)
    w = w - (w @ d) * d
    w = w / np.linalg.norm(w)
    u = np.cross(d, w)
    return np.array([d, w, u])


def m_frame_map(p0_from, p1_from, p0_to, p1_to, width):
    """Rigid transform mapping segment (p0_from->p1_from) onto (p0_to->p1_to)
    while preserving the `width` axis -- so the forearm parallelogram doesn't roll
    about its own length when it swings out of the arm's plane."""
    R = _frame(p1_from - p0_from, width).T @ _frame(p1_to - p0_to, width)
    Rm = Gf.Matrix4d(Gf.Matrix3d(*R.flatten().tolist()), Gf.Vec3d(0, 0, 0))
    return Gf.Matrix4d().SetTranslate(Gf.Vec3d(*(-p0_from).tolist())) * Rm \
        * Gf.Matrix4d().SetTranslate(Gf.Vec3d(*p0_to.tolist()))


def set_xform(stage, name, M):
    prim = stage.GetPrimAtPath(f"/World/IRB360/{name}")
    if not prim or not prim.IsValid():
        print(f"  (missing {name})")
        return
    UsdGeom.Xformable(prim).ClearXformOpOrder()
    UsdGeom.Xformable(prim).AddTransformOp().Set(M)


omni.usd.get_context().open_stage(USD)
for _ in range(60):
    app.update()
stage = omni.usd.get_context().get_stage()

for i, arm in ARMS.items():
    phi = arm["phi"]
    u_r = rotz(np.array([1.0, 0, 0]), phi)
    P = np.array([R_MOTOR * u_r[0], R_MOTOR * u_r[1], Z_MOTOR])
    A = np.array([-math.sin(math.radians(phi)), math.cos(math.radians(phi)), 0.0])
    home_E = rotz(HOME_E1, phi)
    home_attach = rotz(HOME_ATTACH1, phi)
    attach = TARGET + rotz(ATTACH_OFF1, phi)
    E = solve_elbow(P, u_r, attach, home_E)
    set_xform(stage, arm["ua"], m_rot_map(P, home_E - P, E - P))       # upper arm
    Mf = m_frame_map(home_E, home_attach, E, attach, A)                # forearm
    for la in arm["la"]:
        set_xform(stage, la, Mf)
    print(f"  arm {i}: E=({E[0]:.0f},{E[1]:.0f},{E[2]:.0f}) "
          f"reach_err={np.linalg.norm(E-attach)-RE:+.1f}mm")

# plate translation
set_xform(stage, "MovingPlate", Gf.Matrix4d().SetTranslate(
    Gf.Vec3d(*(TARGET - HOME_PLATE).tolist())))
print(f"  plate -> ({TARGET[0]:.0f},{TARGET[1]:.0f},{TARGET[2]:.0f})")

# central 4th-axis telescoping shaft: tilt to point at the plate and slide each
# section along the (now shorter) axis to fake the telescope. Base joint fixed.
P_TOP = np.array([0.0, 0.0, -148.0])       # shaft pivot at the base
P_BOT_HOME = np.array([0.0, 0.0, -1178.0])  # plate-side joint at home
rshaft = Gf.Rotation()
rshaft.SetRotateInto(Gf.Vec3d(*(P_BOT_HOME - P_TOP).tolist()),
                     Gf.Vec3d(*(TARGET - P_TOP).tolist()))
Rm_shaft = Gf.Matrix4d().SetRotate(rshaft)
CENTRAL = {"RevoluteLinkUpper": -577.0, "RevoluteLinkLower": -745.0,
           "RevoluteLinkPlate": -1178.0}
for name, z_home in CENTRAL.items():
    t = (z_home - P_TOP[2]) / (P_BOT_HOME[2] - P_TOP[2])   # 0=base .. 1=plate
    Chome = np.array([0.0, 0.0, z_home])
    Cnew = P_TOP + t * (TARGET - P_TOP)
    M = Gf.Matrix4d().SetTranslate(Gf.Vec3d(*(-Chome).tolist())) * Rm_shaft \
        * Gf.Matrix4d().SetTranslate(Gf.Vec3d(*Cnew.tolist()))
    set_xform(stage, name, M)
print("  central 4th-axis shaft: tilted + telescoped to the plate")

# lights + render -----------------------------------------------------------
UsdLux.DomeLight.Define(stage, "/World/Light_Dome").CreateIntensityAttr(700.0)
UsdLux.DistantLight.Define(stage, "/World/Light_Key").CreateIntensityAttr(2500.0)

import omni.replicator.core as rep  # noqa: E402

cam = rep.create.camera(position=(2600, -3000, 400), look_at=(150, 0, -600))
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
