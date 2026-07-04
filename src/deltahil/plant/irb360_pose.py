"""IRB 360-4D kinematic articulation, reusable for one or many robots.

The math (arm IK + parallelogram forearm frame-mapping + 4th-axis telescoping
shaft) was solved against the real CAD and validated in scripts/animate_irb360.py.
Here it is factored so it can drive any robot subtree: `pose(stage, prefix, T)`
sets the LOCAL transforms of the parts under `prefix` (e.g. "/World/IRB360" or a
per-robot "/World/Cell/Robot_A") to place the plate at target `T`, expressed in
that robot's LOCAL frame (base at origin, mm). A robot placed elsewhere in a cell
sits under a parent Xform holding its base pose; convert a shared cell target to
the robot's local frame with `world_to_local` before calling.

Pure-numpy geometry + helpers are module-level (import-clean off the rig, unit
tested). `pose()` and `world_to_local` import pxr lazily (rig only).
"""
from __future__ import annotations

import math

import numpy as np

# --- geometry (mm), from the CAD (robot local frame, base at origin) --------
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


# --- pure-numpy helpers (laptop-safe) --------------------------------------
def rotz(p, phi_deg):
    c, s = math.cos(math.radians(phi_deg)), math.sin(math.radians(phi_deg))
    return np.array([p[0] * c - p[1] * s, p[0] * s + p[1] * c, p[2]])


def _frame(d, w):
    d = d / np.linalg.norm(d)
    w = w - (w @ d) * d
    w = w / np.linalg.norm(w)
    return np.array([d, w, np.cross(d, w)])


def solve_elbow(P, u_r, attach, home_E):
    """Planar delta IK: elbow at RF from motor P and RE from plate attach;
    returns the branch nearest home_E."""
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


# --- USD-side articulation (rig only; pxr imported lazily) ------------------
def pose(stage, prefix, T):
    """Set local transforms of the parts under `prefix` so the plate reaches T
    (local-frame mm). `T` may be a numpy array or 3-tuple."""
    from pxr import Gf, UsdGeom

    T = np.asarray(T, float)

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

    def set_xform(name, M):
        prim = stage.GetPrimAtPath(f"{prefix}/{name}")
        if prim and prim.IsValid():
            UsdGeom.Xformable(prim).ClearXformOpOrder()
            UsdGeom.Xformable(prim).AddTransformOp().Set(M)

    for i, phi in ARMS.items():
        u_r = rotz(np.array([1.0, 0, 0]), phi)
        P = np.array([R_MOTOR * u_r[0], R_MOTOR * u_r[1], Z_MOTOR])
        A = np.array([-math.sin(math.radians(phi)), math.cos(math.radians(phi)), 0.0])
        home_E, home_attach = rotz(HOME_E1, phi), rotz(HOME_ATTACH1, phi)
        attach = T + rotz(ATTACH_OFF1, phi)
        E = solve_elbow(P, u_r, attach, home_E)
        set_xform(f"UA{i}", m_rot_map(P, home_E - P, E - P))
        Mf = m_frame_map(home_E, home_attach, E, attach, A)
        for la in ARM_LA[i]:
            set_xform(la, Mf)

    set_xform("MovingPlate", Gf.Matrix4d().SetTranslate(
        Gf.Vec3d(*(T - HOME_PLATE).tolist())))

    rs = Gf.Rotation()
    rs.SetRotateInto(Gf.Vec3d(*(P_BOT_HOME - P_TOP).tolist()),
                     Gf.Vec3d(*(T - P_TOP).tolist()))
    Rm = Gf.Matrix4d().SetRotate(rs)
    for name, z_home in CENTRAL.items():
        t = (z_home - P_TOP[2]) / (P_BOT_HOME[2] - P_TOP[2])
        Ch = np.array([0.0, 0.0, z_home])
        Cn = P_TOP + t * (T - P_TOP)
        set_xform(name, Gf.Matrix4d().SetTranslate(Gf.Vec3d(*(-Ch).tolist()))
                  * Rm * Gf.Matrix4d().SetTranslate(Gf.Vec3d(*Cn.tolist())))


def world_to_local(base_matrix, world_pt):
    """Convert a world-frame point (mm) into a robot's local frame, given the
    robot parent Xform's local-to-world Gf.Matrix4d. Row-vector convention."""
    from pxr import Gf
    inv = base_matrix.GetInverse()
    p = Gf.Vec3d(float(world_pt[0]), float(world_pt[1]), float(world_pt[2]))
    q = inv.Transform(p)
    return np.array([q[0], q[1], q[2]])
