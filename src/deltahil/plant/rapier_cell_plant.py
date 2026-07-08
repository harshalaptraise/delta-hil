"""RapierCellPlant -- the SAME plant contract as CellPlant, backed by a Rapier world.

Architecturally identical to the MuJoCo plant: a Python plant seam behind the same
server + `cell_controller` (or live TwinCAT over ADS), with the browser as a passive
renderer. The only difference is where the rigid-body physics runs: Rapier
(Rust/WASM) has no Python binding, so the physics world lives in a small node worker
(`render/rapier/rapier_worker.mjs`) that this class drives over a one-line-JSON stdio
protocol. ALL cell logic -- spawn, the integrate-ff-then-trim TCP law, the P3 grasp
coincidence gate, place/pile, the reach envelope, and the conservation ledger -- stays
here in Python (one source of truth, shared shape with `mujoco_cell_plant`). The node
side only owns the Rapier bodies: kinematic grippers/totes, dynamic tortillas dragged
by an explicit belt Coulomb term, and a fixed-joint weld on grasp.
"""
from __future__ import annotations

import json
import os
import subprocess
import sys

import numpy as np

from . import cell_scene as cs
from .cell_plant import (BOX_TOP, BOX_Y, GRIP_OFFSET, GRIP_TOL, HOME_Z, K_BOXES,
                         K_PARTS, PLACE_POS_TOL, REACH_XY, VEL_TOL, XL, XR, Z_MAX,
                         Z_MIN, _home)

N_ITEMS = 24
N_TOTES = 12
TR, THH = 0.075, 0.006


class RapierCellPlant:
    def __init__(self, belt_v_src=0.22, belt_v_box=0.15,
                 spawn_dt_s=3.4, box_dt_s=1.9, v_tcp=1.3, seed=7):
        self.vs, self.vb, self.v_tcp = float(belt_v_src), float(belt_v_box), float(v_tcp)
        self.spawn_dt, self.box_dt = float(spawn_dt_s), float(box_dt_s)
        self._rng = np.random.default_rng(seed)
        self.t = 0.0
        self._next_part_t = self._next_box_t = 0.0
        self._pid = self._bid = 0

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
        self._free_t = list(range(N_ITEMS))
        self._free_b = list(range(N_TOTES))
        # decisions from this step's gate/exit, applied on the NEXT worker request
        self._pend_weld: list = []
        self._pend_unweld: list = []
        self._pend_park: list = []

        rdir = os.path.join(os.path.dirname(__file__), "..", "render", "rapier")
        self._proc = subprocess.Popen(
            ["node", "rapier_worker.mjs"], cwd=rdir, text=True, bufsize=1,
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=sys.stderr)
        r = self._ex({"geom": {
            "tr": TR, "thh": THH, "n_items": N_ITEMS, "n_totes": N_TOTES,
            "tote_w": 0.13, "tote_h": 0.05, "tote_wall": 0.008,
            "belt_len": cs.BELT_LEN, "src_y": cs.SRC_Y, "src_top": cs.SRC_TOP,
            "n_robots": len(self._names), "robot_x": [self.robots[n]["rx"] for n in self._names]}})
        if not r.get("ready"):
            raise RuntimeError(f"rapier worker failed to init: {r}")

    # -- worker IPC ----------------------------------------------------------
    def _ex(self, req: dict) -> dict:
        self._proc.stdin.write(json.dumps(req) + "\n")
        self._proc.stdin.flush()
        line = self._proc.stdout.readline()
        if not line:
            raise RuntimeError("rapier worker closed")
        return json.loads(line)

    def close(self) -> None:
        if getattr(self, "_proc", None) is not None:
            try:
                self._proc.stdin.close()
                self._proc.terminate()
            except Exception:
                pass
            self._proc = None

    def __del__(self):
        self.close()

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
        self.t += dt
        spawn = []
        while self.t >= self._next_part_t:
            spawn.append(self._spawn_tort())
            self._next_part_t += self.spawn_dt
        while self.t >= self._next_box_t:
            self._spawn_tote()
            self._next_box_t += self.box_dt

        # TCP law -> kinematic gripper targets (same integrate-ff-then-trim as CellPlant)
        max_step = self.v_tcp * dt
        tcp_vel = {}
        grippers = []
        for name in self._names:
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
            grippers.append([float(tcp[0]), float(tcp[1]), float(tcp[2])])

        for b in self.boxes:                                    # totes advance kinematically
            b["x"] += self.vb * dt

        drag = [p["_ts"] for p in self.parts if p["state"] == "belt" and p["_ts"] is not None]
        req = {
            "dt": dt, "belt_v": self.vs, "grippers": grippers,
            "totes": [[b["_bs"], b["x"], BOX_Y, BOX_TOP] for b in self.boxes],
            "spawn": [s for s in spawn if s is not None],
            "park": self._pend_park, "weld": self._pend_weld, "unweld": self._pend_unweld,
            "drag": drag,
        }
        resp = self._ex(req)
        self._pend_weld, self._pend_unweld, self._pend_park = [], [], []
        pos = {row[0]: row for row in resp.get("items", [])}
        for p in self.parts:
            if p["_ts"] is not None and p["_ts"] in pos:
                _, x, y, z, qw, qx, qy, qz = pos[p["_ts"]]
                p["x"], p["y"], p["z"], p["quat"] = x, y, z, [qw, qx, qy, qz]

        # grasp / release adjudication (SAME P3 gate as CellPlant); weld applies next step
        belt_v = np.array([self.vs, 0.0, 0.0])
        for name in self._names:
            rb = self.robots[name]
            gripper = rb["tcp"] - np.array([0.0, 0.0, GRIP_OFFSET])
            if rb["carry"] is None and rb["cmd_grip"]:
                for p in self.parts:
                    if p["state"] != "belt" or p["_ts"] is None:
                        continue
                    if (np.linalg.norm(gripper - np.array([p["x"], p["y"], p["z"]])) < GRIP_TOL
                            and np.linalg.norm(tcp_vel[name] - belt_v) < VEL_TOL):
                        p["state"], p["robot"] = "carried", name
                        rb["carry"], rb["grip_confirm"] = p, True
                        self.ledger["picked"] += 1
                        self._pend_weld.append([self._names.index(name), p["_ts"]])
                        break
            elif rb["carry"] is not None and not rb["cmd_grip"]:
                p = rb["carry"]
                self._pend_unweld.append([self._names.index(name), p["_ts"]])
                best, bd = None, 1e9
                for b in self.boxes:
                    dd = abs(b["x"] - rb["tcp"][0])
                    if dd < bd:
                        bd, best = dd, b
                if best is not None and bd < PLACE_POS_TOL:
                    p["state"], p["box"], p["slot"] = "placed", best, best["fill"]
                    best["fill"] += 1
                    self.ledger["placed"] += 1
                    rb["carry"], rb["grip_confirm"] = None, False
                else:                                            # nothing under it -> keep carrying
                    self._pend_weld.append([self._names.index(name), p["_ts"]])
                    self._pend_unweld.pop()

        for p in self.parts:                                     # ran off the belt end
            if p["state"] == "belt" and p["x"] > XR:
                p["state"] = "exit"
                self.ledger["passed"] += 1
                if p["_ts"] is not None:
                    self._pend_park.append(p["_ts"]); self._free_t.append(p["_ts"]); p["_ts"] = None
        for b in list(self.boxes):                               # totes that exit take their parts
            if b["x"] > XR:
                self._free_b.append(b["_bs"])
                self.boxes.remove(b)
                for p in self.parts:
                    if p["box"] is b and p["state"] == "placed":
                        p["state"] = "exit"
                        if p["_ts"] is not None:
                            self._pend_park.append(p["_ts"]); self._free_t.append(p["_ts"]); p["_ts"] = None
        self.parts = [p for p in self.parts if p["state"] != "exit"]

    def _spawn_tort(self):
        if not self._free_t:
            return None
        ts = self._free_t.pop(0)
        y = cs.SRC_Y + float(self._rng.uniform(-0.018, 0.018))
        z = cs.SRC_TOP + THH + 0.003
        self.parts.append({"id": self._pid, "x": XL, "y": y, "z": z, "quat": [1, 0, 0, 0],
                           "state": "belt", "robot": None, "box": None, "slot": 0, "_ts": ts})
        self._pid += 1
        self.ledger["spawned"] += 1
        return [ts, XL, y, z]

    def _spawn_tote(self):
        if not self._free_b:
            return
        bs = self._free_b.pop(0)
        self.boxes.append({"id": self._bid, "x": XL, "fill": 0, "_bs": bs})
        self._bid += 1

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
