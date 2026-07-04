"""Shared two-robot IRB 360 cell scene: black-steel enclosure + green belts +
two robots mounted on the top deck (3-point mounts). Config in METRES; the robot
USD is mm, so each robot sits under a parent Xform scaled 0.001 + translated.

build_cell() returns {robot_name: base Gf.Matrix4d} for world<->local target
conversion (irb360_pose.world_to_local). pxr is imported lazily so this module
imports clean off the rig (config constants only).
"""
from __future__ import annotations

import math

# --- layout (metres) --------------------------------------------------------
MM = 0.001
ROBOT_X, BASE_Z = 0.70, 1.50
ROBOTS = {"Robot_A": (-ROBOT_X, 0.0, BASE_Z), "Robot_B": (ROBOT_X, 0.0, BASE_Z)}
# belts kept within the robot's usable lateral reach (~+/-0.15 m; the CAD-derived
# workspace is narrow -- forearms over-stretch beyond that)
SRC_Y, BOX_Y = -0.15, 0.15          # product & box belt centre-lines
BOX_TOP = 0.16                       # box belt top height
SRC_TOP = BOX_TOP + 0.30             # product belt ~1 ft higher (taller boxes)
PART_Z = SRC_TOP + 0.02              # a part rests on the product belt
BELT_LEN = 3.0
DECK_Z = BASE_Z
FR_L, FR_W = 3.2, 1.8                # enclosure length x width (broadened)
_STEEL = (0.10, 0.11, 0.13)
_GREEN = (0.20, 0.55, 0.25)
_C = 0.09


def _box(stage, path, size, pos, color):
    from pxr import Gf, UsdGeom
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    M = Gf.Matrix4d().SetScale(Gf.Vec3d(*size)) * Gf.Matrix4d().SetTranslate(Gf.Vec3d(*pos))
    cube.AddTransformOp().Set(M)
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    return cube


def build_cell(stage, irb360_path):
    from pxr import Gf, Sdf, UsdGeom
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    UsdGeom.Xform.Define(stage, "/World/Cell")

    _box(stage, "/World/Floor", (3.6, 2.4, 0.02), (0, 0, 0), (0.32, 0.33, 0.36))

    belt_h = 0.14
    _box(stage, "/World/BoxConveyor", (BELT_LEN, 0.42, belt_h),
         (0, BOX_Y, BOX_TOP - belt_h / 2), _GREEN)
    _box(stage, "/World/SrcConveyor", (BELT_LEN, 0.34, belt_h),
         (0, SRC_Y, SRC_TOP - belt_h / 2), _GREEN)
    for i, sx in enumerate((-1.1, 0.0, 1.1)):
        _box(stage, f"/World/SrcStand_{i}", (0.08, 0.08, SRC_TOP - belt_h),
             (sx, SRC_Y, (SRC_TOP - belt_h) / 2), _STEEL)

    col_x = (-FR_L / 2, 0.0, FR_L / 2)
    col_y = (-FR_W / 2, FR_W / 2)
    k = 0
    for cx in col_x:
        for cy in col_y:
            _box(stage, f"/World/Frame_col_{k}", (_C, _C, DECK_Z), (cx, cy, DECK_Z / 2), _STEEL)
            k += 1
    for i, cy in enumerate(col_y):
        _box(stage, f"/World/Frame_topX_{i}", (FR_L, _C, _C), (0, cy, DECK_Z), _STEEL)
    for i, cx in enumerate(col_x):
        _box(stage, f"/World/Frame_topY_{i}", (_C, FR_W, _C), (cx, 0, DECK_Z), _STEEL)
    for j, rz in enumerate((0.55, 1.05)):
        for i, cy in enumerate(col_y):
            _box(stage, f"/World/Frame_rail_{j}_{i}", (FR_L, _C, _C), (0, cy, rz), _STEEL)

    mount_r = 0.20
    for name, (rx, _, _) in ROBOTS.items():
        for a, phi in enumerate((0.0, 120.0, 240.0)):
            px = rx + mount_r * math.cos(math.radians(phi))
            py = mount_r * math.sin(math.radians(phi))
            _box(stage, f"/World/Mount_{name}_{a}", (0.15, 0.12, 0.05),
                 (px, py, DECK_Z + 0.025), (0.16, 0.17, 0.19))

    bases = {}
    for name, base in ROBOTS.items():
        prim = stage.DefinePrim(f"/World/Cell/{name}", "Xform")
        M = Gf.Matrix4d().SetScale(Gf.Vec3d(MM, MM, MM)) \
            * Gf.Matrix4d().SetTranslate(Gf.Vec3d(*base))
        UsdGeom.Xformable(prim).AddTransformOp().Set(M)
        prim.GetReferences().AddReference(irb360_path, Sdf.Path("/World/IRB360"))
        bases[name] = M
    return bases


def spawn_tortilla(stage, path, pos):
    """A tortilla: a thin flat disc (Cylinder), ~15 cm dia, ~1 cm thick."""
    from pxr import Gf, UsdGeom
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.CreateRadiusAttr(0.075)
    cyl.CreateHeightAttr(0.010)
    cyl.CreateAxisAttr("Z")
    cyl.AddTranslateOp().Set(Gf.Vec3d(*pos))
    cyl.CreateDisplayColorAttr([Gf.Vec3f(0.86, 0.72, 0.48)])
    return cyl


def spawn_box(stage, path, pos):
    _box(stage, path, (0.24, 0.24, 0.14), pos, (0.55, 0.4, 0.25))


def move_prim(stage, path, pos):
    from pxr import Gf, UsdGeom
    prim = stage.GetPrimAtPath(path)
    if prim and prim.IsValid():
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
