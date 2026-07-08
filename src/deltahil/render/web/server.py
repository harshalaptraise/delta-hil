"""Web render backend for the delta-hil cell.

An aiohttp WebSocket server that runs the SAME `cell_plant` + `cell_controller`
(or the live TwinCAT PLC over ADS) headless and streams JSON snapshots to a
Three.js viewer at ~30 Hz. This replaces NVIDIA Isaac Sim as the *renderer* -- no
GPU, no Windows, no Omniverse.

Principle 1 (controller invariance): the plant, `MockCellController`, `CellAdsLink`,
and the TwinCAT ST program are byte-identical to the Isaac path; only the render
seam changed. The browser is a passive snapshot consumer (principle 2).
"""
from __future__ import annotations

import asyncio
import json
import os
import time

from aiohttp import WSMsgType, web

from ...plant import cell_scene as cs
from ...plant.cell_plant import BOX_TOP, REACH_XY, VEL_TOL, CellPlant
from ...plc.cell_controller import MockCellController

STATIC = os.path.join(os.path.dirname(__file__), "static")
SNAP_HZ = 30
DT = 0.01                              # control tick (matches the mock/test cadence)


def cell_config() -> dict:
    """Geometry the viewer needs, derived from the ONE authoritative scene so the
    view can never drift from the plant."""
    return {
        "robots": [{"name": n, "base": [rx, ry, rz]} for n, (rx, ry, rz) in cs.ROBOTS.items()],
        "src_belt": {"y": cs.SRC_Y, "top": cs.SRC_TOP, "len": cs.BELT_LEN, "w": 0.26},
        "box_belt": {"y": cs.BOX_Y, "top": BOX_TOP, "len": cs.BELT_LEN, "w": 0.30},
        "tortilla": {"r": 0.075, "h": 0.010, "part_z": cs.PART_Z},
        "tote": {"w": 0.26, "d": 0.26, "h": 0.10},
        "reach": REACH_XY, "vel_tol": VEL_TOL, "mount_r": 0.20, "base_z": cs.BASE_Z,
    }


def snapshot(plant, dt: float, cmds: dict) -> dict:
    """JSON-safe snapshot of the plant (same fields the Isaac render records)."""
    rob, vx, vc, grip, matched = {}, {}, {}, {}, {}
    for n, rb in plant.robots.items():
        tcp = rb["tcp"]
        rob[n] = [float(tcp[0]), float(tcp[1]), float(tcp[2])]
        v = float((tcp[0] - rb["tcp_prev"][0]) / dt) if dt > 0 else 0.0
        vx[n] = v
        vc[n] = float(cmds.get(n, {}).get("vel", (0.0, 0.0, 0.0))[0])
        grip[n] = rb["carry"] is not None
        matched[n] = abs(vc[n]) > 1e-6 and abs(v - vc[n]) < VEL_TOL
    L = plant.ledger
    return {
        "t": round(plant.t, 3),
        "rob": rob, "vx": vx, "vcmd": vc, "grip": grip, "matched": matched,
        "vsrc": float(plant.vs), "vbox": float(plant.vb),
        "parts": [[p["id"], float(p["x"]), float(p["y"]), float(p["z"])] + list(p.get("quat", (1, 0, 0, 0)))
                  for p in plant.parts],
        "boxes": [[b["id"], float(b["x"]), int(b["fill"])] for b in plant.boxes],
        "ledger": {"picked": L["picked"], "placed": L["placed"], "passed": L["passed"],
                   "spawned": L["spawned"], "conserved": bool(plant.conserved()),
                   "reach": int(plant.reach_violations),
                   "rate": round(L["placed"] / max(plant.t, 1e-6) * 60.0, 1)},   # items/min
    }


async def _broadcast(app, snap: dict) -> None:
    data = json.dumps(snap)
    dead = []
    for ws in app["clients"]:
        try:
            await ws.send_str(data)
        except Exception:
            dead.append(ws)
    for ws in dead:
        app["clients"].discard(ws)


async def control_loop(app, plc_ams: str | None = None) -> None:
    """Advance the cell in real time and broadcast at ~30 Hz. Mock controller by
    default; live TwinCAT over ADS if `plc_ams` is given (blocking ADS runs in a
    thread so the event loop keeps serving)."""
    if app.get("plant_kind") == "mujoco":
        from ...plant.mujoco_cell_plant import MuJoCoCellPlant
        plant = MuJoCoCellPlant()
    else:
        plant = CellPlant()
    native = None
    if app.get("native") and app.get("plant_kind") == "mujoco":
        try:
            import mujoco.viewer
            native = mujoco.viewer.launch_passive(plant.m, plant.d)
        except Exception as exc:
            print(f"[web] --native MuJoCo window unavailable ({exc})")
    link = ctrl = None
    if plc_ams:
        from ...plc.cell_link import CellAdsLink
        link = CellAdsLink(plc_ams)
    else:
        ctrl = MockCellController()
    app["source"] = "live TwinCAT" if link else "mock controller"

    last_cmds: dict = {}
    prev = time.monotonic()
    acc, prev_plc = 0.0, None
    try:
        while True:
            now = time.monotonic()
            acc = min(acc + (now - prev), 0.1)          # cap so a stall doesn't spiral
            prev = now
            while acc >= DT:
                sensors = plant.read_sensors()
                if link is None:
                    last_cmds = ctrl.decide(sensors, DT)
                    plant.apply_commands(last_cmds)
                    plant.step(DT)
                else:
                    def _io():                          # blocking ADS round-trip
                        link.write_sensors(sensors)
                        return link.read_commands()
                    last_cmds, enable, plc_ns = await asyncio.to_thread(_io)
                    rdt = ((plc_ns - prev_plc) / 1e9 if plc_ns and prev_plc else DT)
                    rdt = min(max(rdt, 0.0), 0.05)
                    prev_plc = plc_ns
                    plant.apply_commands(last_cmds)
                    if enable and rdt > 0.0:
                        plant.step(rdt)
                acc -= DT
            snap = snapshot(plant, DT, last_cmds)
            snap["source"] = app.get("source", "mock controller")
            await _broadcast(app, snap)
            if native is not None and native.is_running():
                native.sync()
            await asyncio.sleep(1.0 / SNAP_HZ)
    except asyncio.CancelledError:
        if link is not None:
            link.close()
        if native is not None:
            native.close()
        raise


async def _index(request):
    return web.FileResponse(os.path.join(STATIC, "viewer.html"))


async def _config(request):
    return web.json_response({**cell_config(), "robot_model": request.app.get("robot", "stylized")})


async def _ws(request):
    ws = web.WebSocketResponse(heartbeat=20)
    await ws.prepare(request)
    request.app["clients"].add(ws)
    try:
        async for msg in ws:                            # viewer is read-only; just drain
            if msg.type == WSMsgType.ERROR:
                break
    finally:
        request.app["clients"].discard(ws)
    return ws


def make_app(plc_ams: str | None = None, robot: str = "stylized",
             plant_kind: str = "kinematic", native: bool = False) -> web.Application:
    app = web.Application()
    app["clients"] = set()
    app["robot"] = robot
    app["plant_kind"] = plant_kind
    app["native"] = native
    app.router.add_get("/", _index)
    app.router.add_get("/config.json", _config)
    app.router.add_get("/ws", _ws)
    app.router.add_static("/static/", STATIC)

    async def _start(a):
        a["loop_task"] = asyncio.create_task(control_loop(a, plc_ams))

    async def _stop(a):
        a["loop_task"].cancel()
        try:
            await a["loop_task"]
        except asyncio.CancelledError:
            pass

    app.on_startup.append(_start)
    app.on_cleanup.append(_stop)
    return app


def serve(host: str = "127.0.0.1", port: int = 8080, plc_ams: str | None = None,
          robot: str = "stylized", plant_kind: str = "kinematic", native: bool = False) -> None:
    print(f"[web] delta-hil cell viewer -> http://{host}:{port}  "
          f"({'live TwinCAT ' + plc_ams if plc_ams else 'mock controller'}, "
          f"robot={robot}, plant={plant_kind})")
    web.run_app(make_app(plc_ams, robot, plant_kind, native), host=host, port=port, print=None)
