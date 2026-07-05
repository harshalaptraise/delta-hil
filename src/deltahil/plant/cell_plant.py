"""CellPlant -- the two-robot tortilla cell as a *pure plant* (no control logic).

The controller (TwinCAT PLC on the rig, or MockCellController on the laptop) reads
this plant's sensors -- live part/box positions, belt velocities, per-robot TCP +
grip_confirm -- and writes per-robot commands -- TCP pose + grip. This plant only
senses and actuates:

  * belts advance; tortillas + boxes stream in and exit (P5 conservation),
  * each robot's TCP eases toward its commanded pose,
  * a grasp is adjudicated by P3 coincidence -- position AND belt-velocity match
    at the grip instant (P2 conveyor tracking); a mismatch is a rejected/sheared
    grab, never a phantom pick,
  * commands outside the measured reach envelope are counted as violations (P4).

Pure Python, metres, cell/world frame -- no Isaac. The render layer
(scripts/run_twincat_cell.py) reads this state and draws it with the same USD
artifacts. Geometry constants come from cell_scene (imported for values only; its
pxr use is lazy, so this stays laptop-importable).
"""
from __future__ import annotations

import numpy as np

from . import cell_scene as cs

# --- reported slots + coincidence/reach tolerances -------------------------
K_PARTS = 6                     # nearest belt parts reported to the controller
K_BOXES = 4                     # nearest boxes reported
GRIP_TOL = 0.02                 # m   position coincidence for a grasp (pick: strict)
VEL_TOL = 0.06                  # m/s velocity coincidence (belt-velocity match, pick)
PLACE_POS_TOL = 0.10            # m   a release over a 0.26 m tote lands in it (never drop)
PLACE_VEL_TOL = 0.40            # m/s (you drop it in; no tight velocity lock needed)
REACH_XY = 0.20                 # m   lateral reach from a robot's axis (wider pick/place window)
Z_MIN, Z_MAX = 0.10, 0.60       # m   vertical reach envelope (world)
PICK_Z = cs.PART_Z              # tortilla top on the product belt
STACK0 = cs.BOX_TOP + 0.02      # first tortilla height inside a tote
THICK = 0.014
HOME_Z = 0.42
XL = -cs.BELT_LEN / 2 - 0.15    # belt entry (m)
XR = cs.BELT_LEN / 2 + 0.05     # belt exit (m)


def _home(rx):
    return np.array([rx, 0.0, HOME_Z])


class CellPlant:
    def __init__(self, belt_v_src=0.22, belt_v_box=0.10,
                 spawn_dt_s=2.6, box_dt_s=3.2, v_tcp=1.3, seed=7):
        self.vs = float(belt_v_src)          # product belt velocity (m/s, +X)
        self.vb = float(belt_v_box)          # box belt velocity
        self.v_tcp = float(v_tcp)            # max TCP speed (m/s) -> smooth, fast motion
        self.spawn_dt = float(spawn_dt_s)
        self.box_dt = float(box_dt_s)
        self._rng = np.random.default_rng(seed)
        self.t = 0.0
        self._next_part_t = 0.0
        self._next_box_t = 0.0
        self._pid = 0
        self._bid = 0
        self.parts = []                      # active: state in belt|carried|placed
        self.boxes = []                      # active totes
        self.robots = {
            name: {"rx": rx, "cmd_tcp": _home(rx), "cmd_grip": False,
                   "tcp": _home(rx), "tcp_prev": _home(rx),
                   "carry": None, "grip_confirm": False}
            for name, (rx, _, _) in cs.ROBOTS.items()
        }
        # monotonic ledger -> conservation invariant (E5)
        self.ledger = {"spawned": 0, "picked": 0, "passed": 0, "placed": 0}
        self.reach_violations = 0            # E4
        self.bad_grasps = 0                  # E2 (grip asserted w/o coincidence)

    # -- controller -> plant -------------------------------------------------
    def apply_commands(self, cmds: dict) -> None:
        """cmds: {robot_name: {"tcp": (x,y,z) metres world, "grip": bool}}."""
        for name, c in cmds.items():
            rb = self.robots.get(name)
            if rb is None:
                continue
            tcp = np.asarray(c["tcp"], float)
            lat = float(np.hypot(tcp[0] - rb["rx"], tcp[1]))
            if lat > REACH_XY + 1e-6 or not (Z_MIN - 1e-6 <= tcp[2] <= Z_MAX + 1e-6):
                self.reach_violations += 1          # count, then clamp so sim is stable
                tcp = tcp.copy()
                tcp[2] = min(max(tcp[2], Z_MIN), Z_MAX)
            rb["cmd_tcp"] = tcp
            rb["cmd_grip"] = bool(c.get("grip", False))

    # -- physics -------------------------------------------------------------
    def step(self, dt: float) -> None:
        self.t += dt
        while self.t >= self._next_part_t:                  # spawn tortillas
            y = cs.SRC_Y + float(self._rng.uniform(-0.05, 0.05))
            self.parts.append({"id": self._pid, "x": XL, "y": y, "z": cs.PART_Z,
                               "state": "belt", "robot": None, "box": None, "slot": 0})
            self._pid += 1
            self.ledger["spawned"] += 1
            self._next_part_t += self.spawn_dt
        while self.t >= self._next_box_t:                   # steady tote feed
            self.boxes.append({"id": self._bid, "x": XL, "fill": 0})
            self._bid += 1
            self._next_box_t += self.box_dt

        for p in self.parts:                                # product belt carries belt-parts
            if p["state"] == "belt":
                p["x"] += self.vs * dt
        for b in self.boxes:                                # tote belt runs continuously
            b["x"] += self.vb * dt                          #   (the robot TRACKS a moving tote)

        belt_v = np.array([self.vs, 0.0, 0.0])
        box_v = np.array([self.vb, 0.0, 0.0])
        max_step = self.v_tcp * dt
        for name, rb in self.robots.items():
            rb["tcp_prev"] = rb["tcp"].copy()
            to = rb["cmd_tcp"] - rb["tcp"]            # constant-speed move -> smooth motion
            d = float(np.linalg.norm(to))
            if d <= max_step or d < 1e-9:
                rb["tcp"] = rb["cmd_tcp"].copy()
            else:
                rb["tcp"] = rb["tcp"] + to * (max_step / d)
            tcp_vel = (rb["tcp"] - rb["tcp_prev"]) / dt if dt > 0 else np.zeros(3)

            if rb["carry"] is None:
                if rb["cmd_grip"]:
                    self._try_grab(name, rb, tcp_vel, belt_v)
            else:                                # carrying -> the part rides the TCP
                p = rb["carry"]
                p["x"], p["y"], p["z"] = rb["tcp"][0], rb["tcp"][1], rb["tcp"][2]
                if not rb["cmd_grip"]:           # release -> place if over a tracked box
                    self._release(rb, tcp_vel, box_v)

        for p in self.parts:                                # placed parts ride their box
            if p["state"] == "placed" and p["box"] is not None:
                b = p["box"]
                p["x"], p["y"], p["z"] = b["x"], cs.BOX_Y, STACK0 + p["slot"] * THICK

        for p in list(self.parts):                          # belt-parts that ran off the end
            if p["state"] == "belt" and p["x"] > XR:
                p["state"] = "exit"
                self.ledger["passed"] += 1
        for b in list(self.boxes):                          # totes that exit take their parts
            if b["x"] > XR:
                self.boxes.remove(b)
                for p in self.parts:
                    if p["box"] is b:
                        p["state"] = "exit"
        self.parts = [p for p in self.parts if p["state"] != "exit"]

    def _try_grab(self, name, rb, tcp_vel, belt_v) -> bool:
        for p in self.parts:
            if p["state"] != "belt":
                continue
            ppos = np.array([p["x"], p["y"], cs.PART_Z])
            pos_ok = np.linalg.norm(rb["tcp"] - ppos) < GRIP_TOL
            vel_ok = np.linalg.norm(tcp_vel - belt_v) < VEL_TOL     # P2 velocity match
            if pos_ok and vel_ok:
                p["state"] = "carried"
                p["robot"] = name
                rb["carry"] = p
                rb["grip_confirm"] = True
                self.ledger["picked"] += 1
                return True
        return False

    def _release(self, rb, tcp_vel, box_v) -> None:
        p = rb["carry"]
        best, bd = None, 1e9
        for b in self.boxes:                          # nearest tote under the gripper (x)
            d = abs(b["x"] - rb["tcp"][0])
            if d < bd:
                bd, best = d, b
        if best is not None and bd < PLACE_POS_TOL:   # a tote is under it -> lands inside
            p["state"] = "placed"
            p["box"] = best
            p["slot"] = best["fill"]
            best["fill"] += 1
            self.ledger["placed"] += 1
            rb["carry"] = None
            rb["grip_confirm"] = False
        # else: no tote under the gripper -> do NOT drop it; keep it in hand (never
        # abandon). The controller only releases with a tote in the window, so this
        # is just a safety net -- the part stays carried until a tote is there.

    # -- plant -> controller -------------------------------------------------
    def read_sensors(self) -> dict:
        # report the slots NEAREST the robots (not the left-most) so a dense belt
        # doesn't hide the reachable parts/totes behind far-upstream ones
        rxs = [rb["rx"] for rb in self.robots.values()]

        def dmin(x):
            return min(abs(x - rx) for rx in rxs)

        belt = sorted((p for p in self.parts if p["state"] == "belt"),
                      key=lambda p: dmin(p["x"]))[:K_PARTS]
        parts = []
        for k in range(K_PARTS):
            if k < len(belt):
                p = belt[k]
                parts.append((p["id"], p["x"], p["y"], True))
            else:
                parts.append((0, 0.0, 0.0, False))
        boxes = []
        bsorted = sorted(self.boxes, key=lambda b: dmin(b["x"]))[:K_BOXES]
        for m in range(K_BOXES):
            if m < len(bsorted):
                b = bsorted[m]
                boxes.append((b["id"], b["x"], b["fill"], True))
            else:
                boxes.append((0, 0.0, 0, False))
        robots = {name: {"tcp": tuple(rb["tcp"]), "grip_confirm": rb["grip_confirm"]}
                  for name, rb in self.robots.items()}
        return {"parts": parts, "boxes": boxes, "robots": robots,
                "belt_v_src": self.vs, "belt_v_box": self.vb}

    # -- E5 conservation check (callable each step) --------------------------
    def conserved(self) -> bool:
        belt_active = sum(1 for p in self.parts if p["state"] == "belt")
        return self.ledger["spawned"] == self.ledger["passed"] + self.ledger["picked"] + belt_active
