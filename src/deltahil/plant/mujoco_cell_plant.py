"""MuJoCoCellPlant -- the SAME plant contract as CellPlant, backed by MuJoCo dynamics.

What changes vs the kinematic `cell_plant`: tortillas are free rigid cylinders the
**source belt drags by contact friction** (the standard MuJoCo conveyor idiom -- hold
the slide-jointed slab's velocity, re-zero its position each substep); the grasp is a
**weld** toggled on the SAME P3 coincidence gate; placed tortillas **physically fall and
pile** in the (kinematically-moving) totes. What stays identical: the geometry
(`cell_scene`), the tolerances, the integrate-ff-then-trim TCP law, the bounded reach
envelope, and the conservation ledger -- so `MockCellController` / `CellAdsLink` / the
TwinCAT program (and the web snapshot + tests) drive it **unchanged**.

The TCP is a mocap body (kinematic ideal servo, non-colliding) -- adjudication, not
contact force, decides the pick, exactly as in the kinematic plant.
"""
from __future__ import annotations

import numpy as np

from . import cell_scene as cs
from .cell_plant import (BOX_TOP, BOX_Y, GRIP_OFFSET, GRIP_TOL, HOME_Z, K_BOXES,
                         K_PARTS, PICK_Z, PLACE_POS_TOL, REACH_XY, VEL_TOL, XL,
                         XR, Z_MAX, Z_MIN, _home)

N_TORT = 28                     # tortilla body pool (recycled)
N_TOTE = 12                     # tote body pool (recycled)
TR, THH = 0.075, 0.006          # tortilla radius, half-height
TW, TH, TWALL = 0.13, 0.05, 0.008   # tote half-width, wall half-height, wall half-thick
PARK = (6.0, 4.0)               # off-cell park row
_SUBSTEP = 0.002


def _model_xml() -> str:
    src_z = cs.SRC_TOP - 0.02                     # belt slab: top at SRC_TOP (0.46)
    torts = "\n".join(
        f"""    <body name="t{i}" pos="{PARK[0] + 0.3 * i:.2f} {PARK[1]} 0.05">
      <freejoint name="t{i}"/>
      <geom name="t{i}g" type="cylinder" size="{TR} {THH}" mass="0.05"
            friction="1.0 0.01 0.001" condim="4" rgba=".93 .91 .86 1"/>
    </body>""" for i in range(N_TORT))
    grips = "\n".join(
        f"""    <body name="g{r}" mocap="true" pos="{rx} 0 {HOME_Z}">
      <geom type="sphere" size="0.03" contype="0" conaffinity="0" rgba=".8 .84 .88 .6"/>
    </body>""" for r, (rx, _, _) in enumerate(cs.ROBOTS.values()))
    totes = []
    for k in range(N_TOTE):
        px = PARK[0] + 0.4 * k
        walls = (f'<geom type="box" pos="0 0 0" size="{TW} {TW} {TWALL}" rgba=".72 .58 .40 1"/>'
                 f'<geom type="box" pos="{-TW} 0 {TH}" size="{TWALL} {TW} {TH}" rgba=".72 .58 .40 1"/>'
                 f'<geom type="box" pos="{TW} 0 {TH}" size="{TWALL} {TW} {TH}" rgba=".72 .58 .40 1"/>'
                 f'<geom type="box" pos="0 {-TW} {TH}" size="{TW} {TWALL} {TH}" rgba=".72 .58 .40 1"/>'
                 f'<geom type="box" pos="0 {TW} {TH}" size="{TW} {TWALL} {TH}" rgba=".72 .58 .40 1"/>')
        totes.append(f'    <body name="b{k}" mocap="true" pos="{px:.2f} {PARK[1] + 1.0} {BOX_TOP}">{walls}</body>')
    welds = "\n".join(
        f'    <weld name="w{r}_{i}" body1="g{r}" body2="t{i}" active="false" '
        f'relpose="0 0 {-GRIP_OFFSET - THH:.4f} 1 0 0 0" torquescale="1"/>'
        for r in range(len(cs.ROBOTS)) for i in range(N_TORT))
    return f"""
<mujoco model="delta_cell">
  <option timestep="{_SUBSTEP}" integrator="implicitfast"/>
  <worldbody>
    <geom name="floor" type="plane" pos="0 0 0" size="5 4 .1" friction="1 .01 .001"/>
    <body name="srcbelt" pos="0 {cs.SRC_Y} {src_z:.3f}">
      <joint name="belt_x" type="slide" axis="1 0 0"/>
      <geom name="srcbelt_g" type="box" size="{cs.BELT_LEN / 2 + 0.25:.2f} 0.14 0.02" mass="80"
            friction="1.4 0.01 0.001" rgba=".42 .30 .19 1"/>
    </body>
{grips}
{torts}
{chr(10).join(totes)}
  </worldbody>
  <equality>
{welds}
  </equality>
</mujoco>"""


class MuJoCoCellPlant:
    def __init__(self, belt_v_src=0.22, belt_v_box=0.15,
                 spawn_dt_s=3.4, box_dt_s=1.9, v_tcp=1.3, seed=7):
        import mujoco
        self._mj = mujoco
        self.vs, self.vb, self.v_tcp = float(belt_v_src), float(belt_v_box), float(v_tcp)
        self.spawn_dt, self.box_dt = float(spawn_dt_s), float(box_dt_s)
        self._rng = np.random.default_rng(seed)
        self.t = 0.0
        self._next_part_t = self._next_box_t = 0.0
        self._pid = self._bid = 0

        self.m = mujoco.MjModel.from_xml_string(_model_xml())
        self.d = mujoco.MjData(self.m)
        self._belt_q = self.m.joint("belt_x").qposadr[0]
        self._belt_v = self.m.joint("belt_x").dofadr[0]
        self._tq = [self.m.joint(f"t{i}").qposadr[0] for i in range(N_TORT)]
        self._tv = [self.m.joint(f"t{i}").dofadr[0] for i in range(N_TORT)]
        self._gmid = [self.m.body(f"g{r}").mocapid[0] for r in range(len(cs.ROBOTS))]
        self._bmid = [self.m.body(f"b{k}").mocapid[0] for k in range(N_TOTE)]
        self._weld = {(r, i): self.m.equality(f"w{r}_{i}").id
                      for r in range(len(cs.ROBOTS)) for i in range(N_TORT)}

        self.robots = {
            name: {"rx": rx, "cmd_tcp": _home(rx), "cmd_grip": False,
                   "cmd_vel": np.zeros(3), "tcp": _home(rx), "tcp_prev": _home(rx),
                   "carry": None, "grip_confirm": False}
            for name, (rx, _, _) in cs.ROBOTS.items()}
        self._names = list(self.robots)
        self.parts: list[dict] = []          # {id,x,y,z,quat,state,robot,box,slot,_ts}
        self.boxes: list[dict] = []          # {id,x,fill,_bs}
        self.ledger = {"spawned": 0, "picked": 0, "passed": 0, "placed": 0}
        self.reach_violations = 0
        self._free_t = list(range(N_TORT))
        self._free_b = list(range(N_TOTE))
        self._settle: dict[int, float] = {}  # tortilla slot -> recycle time

        for r, name in enumerate(self._names):
            self.d.mocap_pos[self._gmid[r]] = self.robots[name]["tcp"]
        mujoco.mj_forward(self.m, self.d)

    # -- mujoco slot helpers -------------------------------------------------
    def _tort_pose(self, ts):
        q = self._tq[ts]
        return self.d.qpos[q:q + 3].copy(), self.d.qpos[q + 3:q + 7].copy()

    def _set_tort(self, ts, x, y, z, quat=(1, 0, 0, 0)):
        q, v = self._tq[ts], self._tv[ts]
        self.d.qpos[q:q + 7] = [x, y, z, *quat]
        self.d.qvel[v:v + 6] = 0

    def _park_tort(self, ts):
        self._set_tort(ts, PARK[0] + 0.3 * ts, PARK[1], 0.05)
        self._free_t.append(ts)

    # -- controller -> plant (identical to CellPlant) ------------------------
    def apply_commands(self, cmds: dict) -> None:
        for name, c in cmds.items():
            rb = self.robots.get(name)
            if rb is None:
                continue
            tcp = np.asarray(c["tcp"], float)
            lat = float(np.hypot(tcp[0] - rb["rx"], tcp[1]))
            if lat > REACH_XY + 1e-6 or not (Z_MIN - 1e-6 <= tcp[2] <= Z_MAX + 1e-6):
                self.reach_violations += 1
                tcp = tcp.copy()
                tcp[2] = min(max(tcp[2], Z_MIN), Z_MAX)
            rb["cmd_tcp"] = tcp
            rb["cmd_grip"] = bool(c.get("grip", False))
            rb["cmd_vel"] = np.asarray(c.get("vel", (0.0, 0.0, 0.0)), float)

    # -- physics -------------------------------------------------------------
    def step(self, dt: float) -> None:
        mj = self._mj
        self.t += dt
        while self.t >= self._next_part_t:
            self._spawn_tort()
            self._next_part_t += self.spawn_dt
        while self.t >= self._next_box_t:
            self._spawn_tote()
            self._next_box_t += self.box_dt

        # TCP law -> mocap gripper (integrate ff, trim, reach clamp) ; same as CellPlant
        max_step = self.v_tcp * dt
        tcp_vel = {}
        for r, name in enumerate(self._names):
            rb = self.robots[name]
            rb["tcp_prev"] = rb["tcp"].copy()
            tcp = rb["tcp"] + rb["cmd_vel"] * dt
            to = rb["cmd_tcp"] - tcp
            d = float(np.linalg.norm(to))
            tcp = rb["cmd_tcp"].copy() if (d <= max_step or d < 1e-9) else tcp + to * (max_step / d)
            lat = float(np.hypot(tcp[0] - rb["rx"], tcp[1]))
            if lat > REACH_XY:                                  # re-clamp integrated ff (E4)
                s = REACH_XY / lat
                tcp[0] = rb["rx"] + (tcp[0] - rb["rx"]) * s
                tcp[1] = tcp[1] * s
            tcp[2] = min(max(tcp[2], Z_MIN), Z_MAX)
            rb["tcp"] = tcp
            tcp_vel[name] = (tcp - rb["tcp_prev"]) / dt if dt > 0 else np.zeros(3)
            self.d.mocap_pos[self._gmid[r]] = tcp

        # totes advance kinematically at vb (the robot tracks a moving tote)
        for b in self.boxes:
            b["x"] += self.vb * dt
            self.d.mocap_pos[self._bmid[b["_bs"]]] = [b["x"], BOX_Y, BOX_TOP]

        # conveyor idiom + substep
        self.d.qpos[self._belt_q] = 0.0
        self.d.qvel[self._belt_v] = self.vs
        for _ in range(max(1, round(dt / _SUBSTEP))):
            mj.mj_step(self.m, self.d)
            self.d.qpos[self._belt_q] = 0.0
            self.d.qvel[self._belt_v] = self.vs

        # sync tortilla poses from physics
        for p in self.parts:
            if p["_ts"] is None:
                continue
            pos, quat = self._tort_pose(p["_ts"])
            p["x"], p["y"], p["z"], p["quat"] = float(pos[0]), float(pos[1]), float(pos[2]), [float(q) for q in quat]

        # grasp / release adjudication (SAME P3 gate as CellPlant)
        for name in self._names:
            rb = self.robots[name]
            gripper = rb["tcp"] - np.array([0.0, 0.0, GRIP_OFFSET])
            if rb["carry"] is None and rb["cmd_grip"]:
                self._try_grab(name, rb, gripper, tcp_vel[name])
            elif rb["carry"] is not None and not rb["cmd_grip"]:
                self._release(name, rb)

        # belt tortillas that ran off the end -> passed
        for p in self.parts:
            if p["state"] == "belt" and p["x"] > XR:
                p["state"] = "exit"
                self.ledger["passed"] += 1
                if p["_ts"] is not None:
                    self._park_tort(p["_ts"]); p["_ts"] = None
        # totes that exit take their (placed) parts
        for b in list(self.boxes):
            if b["x"] > XR:
                self._free_b.append(b["_bs"])
                self.d.mocap_pos[self._bmid[b["_bs"]]] = [PARK[0] + 0.4 * b["_bs"], PARK[1] + 1.0, BOX_TOP]
                self.boxes.remove(b)
                for p in self.parts:
                    if p["box"] is b and p["state"] == "placed":
                        p["state"] = "exit"
                        if p["_ts"] is not None:
                            self._park_tort(p["_ts"]); p["_ts"] = None
        # recycle settled placed tortillas after a dwell (pile, then free the slot)
        for ts, when in list(self._settle.items()):
            if self.t >= when:
                self._settle.pop(ts)
        self.parts = [p for p in self.parts if p["state"] != "exit"]

    def _spawn_tort(self) -> None:
        if not self._free_t:
            return
        ts = self._free_t.pop(0)
        y = cs.SRC_Y + float(self._rng.uniform(-0.018, 0.018))
        self._set_tort(ts, XL, y, cs.SRC_TOP + THH + 0.003)      # drop onto the belt
        self.parts.append({"id": self._pid, "x": XL, "y": y, "z": cs.SRC_TOP + THH,
                           "quat": [1, 0, 0, 0], "state": "belt", "robot": None,
                           "box": None, "slot": 0, "_ts": ts})
        self._pid += 1
        self.ledger["spawned"] += 1

    def _spawn_tote(self) -> None:
        if not self._free_b:
            return
        bs = self._free_b.pop(0)
        self.boxes.append({"id": self._bid, "x": XL, "fill": 0, "_bs": bs})
        self.d.mocap_pos[self._bmid[bs]] = [XL, BOX_Y, BOX_TOP]
        self._bid += 1

    def _try_grab(self, name, rb, gripper, tcp_vel) -> None:
        belt_v = np.array([self.vs, 0.0, 0.0])
        for p in self.parts:
            if p["state"] != "belt" or p["_ts"] is None:
                continue
            ppos = np.array([p["x"], p["y"], p["z"]])
            if np.linalg.norm(gripper - ppos) < GRIP_TOL and np.linalg.norm(tcp_vel - belt_v) < VEL_TOL:
                ts = p["_ts"]
                self._set_tort(ts, gripper[0], gripper[1], gripper[2])   # cup self-centres
                self.d.eq_active[self._weld[(self._names.index(name), ts)]] = 1
                p["state"], p["robot"] = "carried", name
                rb["carry"], rb["grip_confirm"] = p, True
                self.ledger["picked"] += 1
                return

    def _release(self, name, rb) -> None:
        p = rb["carry"]
        best, bd = None, 1e9
        for b in self.boxes:
            dd = abs(b["x"] - rb["tcp"][0])
            if dd < bd:
                bd, best = dd, b
        ts = p["_ts"]
        self.d.eq_active[self._weld[(self._names.index(name), ts)]] = 0   # let it fall
        if best is not None and bd < PLACE_POS_TOL:
            p["state"], p["box"], p["slot"] = "placed", best, best["fill"]
            best["fill"] += 1
            self.ledger["placed"] += 1
            rb["carry"], rb["grip_confirm"] = None, False
            self._settle[ts] = self.t + 8.0
        else:
            # no tote under it -> re-weld and keep carrying (never abandon)
            self.d.eq_active[self._weld[(self._names.index(name), ts)]] = 1

    # -- plant -> controller (identical structure to CellPlant) --------------
    def read_sensors(self) -> dict:
        rxs = [rb["rx"] for rb in self.robots.values()]

        def dmin(x):
            return min(abs(x - rx) for rx in rxs)

        belt = sorted((p for p in self.parts if p["state"] == "belt"), key=lambda p: dmin(p["x"]))[:K_PARTS]
        parts = [(belt[k]["id"], belt[k]["x"], belt[k]["y"], True) if k < len(belt)
                 else (0, 0.0, 0.0, False) for k in range(K_PARTS)]
        bs = sorted(self.boxes, key=lambda b: dmin(b["x"]))[:K_BOXES]
        boxes = [(bs[m]["id"], bs[m]["x"], bs[m]["fill"], True) if m < len(bs)
                 else (0, 0.0, 0, False) for m in range(K_BOXES)]
        robots = {name: {"tcp": tuple(rb["tcp"]), "grip_confirm": rb["grip_confirm"]}
                  for name, rb in self.robots.items()}
        return {"parts": parts, "boxes": boxes, "robots": robots,
                "belt_v_src": self.vs, "belt_v_box": self.vb}

    def conserved(self) -> bool:
        belt_active = sum(1 for p in self.parts if p["state"] == "belt")
        return self.ledger["spawned"] == self.ledger["passed"] + self.ledger["picked"] + belt_active
