"""MockCellController -- the reference cell controller (the PLC's Python twin).

Reads CellPlant sensors (live part/box slots, per-robot TCP + grip_confirm, belt
velocities) and returns per-robot commands (TCP pose + grip). It does ALL the
control -- claim a part, track it (command the TCP to the part's live position so
the velocity matches the belt), grip on coincidence, transfer, track a box, place
-- and clamps every command into the measured reach envelope so P4 is never
violated. The TwinCAT ST program in docs/twincat_cell_program.md mirrors this.

Pure Python; the plant is identical whether driven by this or the real PLC (P1).
"""
from __future__ import annotations

import numpy as np

from ..plant import cell_scene as cs
from ..plant.cell_plant import (GRIP_OFFSET, HOME_Z, PICK_Z, REACH_XY, STACK0,
                                 THICK, Z_MAX, Z_MIN)

PICK_HI = cs.PART_Z + 0.10
PLACE_HI = cs.BOX_TOP + 0.30
WIN = 0.08                 # reachable half-window in x -> the robot tracks a part/tote
                           # only within its clean reach (no over-stretch to its limits)
CLAIM_LO, CLAIM_HI = 0.30, 0.02
PHASE_TIMEOUT = 4.0        # s -- abort a stuck pick so nothing deadlocks
PLACE_PATIENCE = 2.5       # s -- wait briefly for a tote, but don't starve picking
ROBOT_X = {name: rx for name, (rx, _, _) in cs.ROBOTS.items()}


def _pref(pid):
    # alternate each product's preferred robot so the upstream one doesn't hog
    return "Robot_A" if pid % 2 == 0 else "Robot_B"


def _clamp(tcp, rx):
    x, y, z = float(tcp[0]), float(tcp[1]), float(tcp[2])
    dx, dy = x - rx, y
    lat = np.hypot(dx, dy)
    if lat > REACH_XY:
        s = REACH_XY / lat
        dx, dy = dx * s, dy * s
    return (rx + dx, dy, min(max(z, Z_MIN), Z_MAX))


class MockCellController:
    def __init__(self, lock_time=0.3):
        self.st = {}
        self.lock_time = lock_time      # s the TCP rides the moving part before gripping

    def _state(self, name, rx):
        if name not in self.st:
            self.st[name] = {"phase": "idle", "part": None, "box": None,
                             "t": 0.0, "gx": rx, "gy": 0.0}
        return self.st[name]

    def _claimed(self):
        return {s["part"] for s in self.st.values() if s.get("part") is not None}

    def _nearest_box(self, rx, bmap):
        best, bd = None, 1e9
        for bid, (bx, _fill) in bmap.items():
            if bx > rx + WIN + 0.5:            # place into the nearest tote in reach
                continue
            d = abs(bx - rx)
            if d < bd:
                bd, best = d, bid
        return best

    def decide(self, sensors, dt):
        pmap = {pid: (x, y) for (pid, x, y, valid) in sensors["parts"] if valid}
        bmap = {bid: (x, fill) for (bid, x, fill, valid) in sensors["boxes"] if valid}
        claimed = self._claimed()
        cmds = {}
        for name in sorted(sensors["robots"].keys()):     # Robot_A (upstream) first
            rx = ROBOT_X[name]
            s = self._state(name, rx)
            s["t"] += dt
            gc = sensors["robots"][name]["grip_confirm"]
            tcp, grip = self._robot(name, rx, s, pmap, bmap, gc, claimed, dt)
            cmds[name] = {"tcp": _clamp(tcp, rx), "grip": grip}
        return cmds

    def _robot(self, name, rx, s, pmap, bmap, gc, claimed, dt):
        home = (rx, 0.0, HOME_Z)
        ph = s["phase"]

        # only the pick chase gives up (a part slipped by before grabbing). Once a
        # part is GRABBED, the robot NEVER abandons it -- carrying phases wait as
        # long as needed for a tote.
        if ph == "track" and s["t"] > PHASE_TIMEOUT:
            s.update(phase="idle", part=None, box=None)
            ph = "idle"

        if ph == "idle":
            # A (upstream) SPLITS -- it claims only its share, leaving the rest for B.
            # B (downstream) is greedy -- it claims any reachable part (the catch-all).
            best, bd = None, 1e9
            for pid, (x, y) in pmap.items():
                if pid in claimed:
                    continue
                if name == "Robot_A" and _pref(pid) != "Robot_A":
                    continue
                if rx - CLAIM_LO <= x <= rx + WIN and abs(y) < REACH_XY:
                    if abs(x - rx) < bd:
                        bd, best = abs(x - rx), pid
            if best is not None:
                s.update(phase="track", part=best, box=None, t=0.0, lock=0.0)
                claimed.add(best)
            return home, False

        if ph == "track":
            if gc:                                        # grabbed
                s.update(phase="lift", t=0.0)
                return (s["gx"], s["gy"], PICK_HI), True
            if s["part"] not in pmap:                     # part gone before grab
                s.update(phase="idle", part=None, box=None)
                return home, False
            x, y = pmap[s["part"]]
            if x > rx + WIN:                              # passed my reach -> free it for the next robot
                s.update(phase="idle", part=None, box=None)
                return home, False
            s["gx"], s["gy"] = x, y
            if abs(x - rx) < WIN:                          # in window -> descend + RIDE with it
                s["lock"] = s.get("lock", 0.0) + dt        # conveyor-tracking dwell (velocity-matched)
                return (x, y, PICK_Z), s["lock"] >= self.lock_time   # grip only after the lock
            s["lock"] = 0.0
            hx = rx - WIN if x < rx else rx + WIN          # else hover at window edge, ready
            return (hx, y, PICK_HI), False

        if ph == "lift":
            if s["t"] > 0.25:
                s.update(phase="transfer", t=0.0)
            return (s["gx"], s["gy"], PICK_HI), True

        if ph == "transfer":
            if s.get("box") not in bmap:                  # COMMIT to one tote; re-pick only if it's gone
                s["box"] = self._nearest_box(rx, bmap)
            if s["box"] is None:
                return (rx, cs.BOX_Y, PLACE_HI), True     # hold the part, wait for a tote
            bx, _fill = bmap[s["box"]]
            if abs(bx - rx) < WIN and s["t"] > 0.15:
                s.update(phase="place", t=0.0, place_t=0.0)   # fresh descend timer each place
            return (min(max(bx, rx - WIN), rx + WIN), cs.BOX_Y, PLACE_HI), True

        if ph == "place":
            if s.get("box") not in bmap:                  # stay committed to the same tote
                s["box"] = self._nearest_box(rx, bmap)
            if s["box"] is None:
                s["place_t"] = 0.0
                return (rx, cs.BOX_Y, PLACE_HI), True      # no tote -> HOLD and wait
            bx, fill = bmap[s["box"]]
            if abs(bx - rx) < WIN:
                # descend timer runs ONLY while the tote is in the window, so we never
                # release before the TCP has actually gone down into the tote
                s["place_t"] = s.get("place_t", 0.0) + dt
                stack_z = STACK0 + fill * THICK + GRIP_OFFSET   # gripper drops it AT stack level
                grip = s["place_t"] < 0.35                 # descend fully, THEN release
                if not gc:                                 # placed -> done
                    s.update(phase="retract", t=0.0)
                return (bx, cs.BOX_Y, stack_z), grip
            s["place_t"] = 0.0                             # tote drifted out -> wait again
            return (min(max(bx, rx - WIN), rx + WIN), cs.BOX_Y, PLACE_HI), True

        # retract
        if s["t"] > 0.25:
            s.update(phase="idle", part=None, box=None)
        return home, False
