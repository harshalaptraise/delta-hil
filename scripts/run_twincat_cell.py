"""PLC-driven tortilla cell -> GIF: TwinCAT runs the continuous two-robot line.

The convergence of both workstreams. A CellPlant (pure plant) is driven by the
real TwinCAT PLC over ADS (CellAdsLink) -- the PLC tracks the streamed parts,
assigns robots, and commands each TCP + grip on the fly; the sim executes, senses,
and conserves. The control loop runs fast (recording snapshots + ADS latency),
then the snapshots are rendered to assets/render/twincat_cell.gif using the same
USD cell (cell_scene) and IRB 360 (irb360.usd) as the animation.

Run on the rig, inside isaacenv, with TwinCAT running the cell program
(docs/twincat_cell_program.md):

    python scripts/run_twincat_cell.py 5.1.204.123.1.1        # AMS NetId (live PLC)
    python scripts/run_twincat_cell.py 5.1.204.123.1.1 20     # + sim seconds
    python scripts/run_twincat_cell.py mock                   # Python controller (no PLC)
"""
from __future__ import annotations

import os
import sys
import time

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from pxr import UsdLux  # noqa: E402

from deltahil.plant import cell_scene as cs  # noqa: E402
from deltahil.plant.cell_plant import CellPlant, STACK0, THICK  # noqa: E402
from deltahil.plant.irb360_pose import pose, world_to_local  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IRB360 = os.path.join(REPO, "assets", "irb360.usd").replace("\\", "/")
RENDER_DIR = os.path.join(REPO, "assets", "render").replace("\\", "/")
OUT_GIF = os.path.join(RENDER_DIR, "twincat_cell.gif").replace("\\", "/")
os.makedirs(RENDER_DIR, exist_ok=True)

N_TORT, N_BOX = 44, 16
HIDE = (6.0, 4.0, -3.0)


def snapshot(plant):
    return {
        "rob": {n: (rb["tcp"][0], rb["tcp"][1], rb["tcp"][2])
                for n, rb in plant.robots.items()},
        "parts": [(p["id"], p["x"], p["y"], p["z"]) for p in plant.parts],
        "boxes": [(b["id"], b["x"], b["fill"]) for b in plant.boxes],
    }


def main(ams, sim_seconds=50.0, dt=0.01, sample_every=7):
    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    bases = cs.build_cell(stage, IRB360)
    for i in range(N_TORT):
        cs.spawn_tortilla(stage, f"/World/CT_{i}", HIDE)
    for i in range(N_BOX):
        cs.spawn_box(stage, f"/World/CB_{i}", HIDE)
    for _ in range(60):
        app.update()
    UsdLux.DomeLight.Define(stage, "/World/Light_Dome").CreateIntensityAttr(700.0)
    UsdLux.DistantLight.Define(stage, "/World/Light_Key").CreateIntensityAttr(2500.0)

    plant = CellPlant()
    if ams == "mock":
        from deltahil.plc.cell_controller import MockCellController
        ctrl, link = MockCellController(), None
        print("[cell/plc] MOCK controller (no TwinCAT)")
    else:
        from deltahil.plc.cell_link import CellAdsLink
        ctrl, link = None, CellAdsLink(ams)
        print(f"[cell/plc] live TwinCAT AMS={ams}")

    # -- phase 1: closed loop, record snapshots + ADS latency ----------------
    # With the LIVE PLC, advance the sim in REAL time so the PLC's TON timers (the
    # 300 ms tracking lock, phase dwells) line up with the sim's belt/part motion.
    # A fixed dt with a fast ADS loop over-advances the sim per PLC tick, so a part
    # leaves the pick window before the lock completes -> the robot never grips.
    # The mock keeps a fixed dt (deterministic, matches the golden reference).
    SNAP_DT = 0.06
    snaps, lat = [], []
    next_snap = 0.0
    if link is None:
        # mock: deterministic sim-time (matches the golden reference), always enabled
        sim_t = 0.0
        while sim_t < sim_seconds:
            sensors = plant.read_sensors()
            plant.apply_commands(ctrl.decide(sensors, dt))
            plant.step(dt)
            sim_t += dt
            if sim_t >= next_snap:
                snaps.append(snapshot(plant)); next_snap += SNAP_DT
    else:
        # live PLC: run in REAL wall-clock time so the PLC's TON timers line up with
        # the sim motion; GVL_Cell.enable (forced in the Watch) gates all motion --
        # FALSE freezes the sim, TRUE runs it. Snapshots on wall-time so a freeze
        # shows in the render.
        start = time.perf_counter(); prev = start
        while (time.perf_counter() - start) < sim_seconds:
            sensors = plant.read_sensors()
            t0 = time.perf_counter()
            link.write_sensors(sensors)
            cmds, enable = link.read_commands()
            lat.append((time.perf_counter() - t0) * 1000.0)
            now = time.perf_counter()
            rdt = min(max(now - prev, 0.001), 0.05)
            prev = now
            plant.apply_commands(cmds)
            if enable:
                plant.step(rdt)                          # frozen when the operator disables
            if (now - start) >= next_snap:
                snaps.append(snapshot(plant)); next_snap += SNAP_DT
    L = plant.ledger
    print(f"[cell] loop done: picked={L['picked']} placed={L['placed']} "
          f"passed={L['passed']} reach_violations={plant.reach_violations} "
          f"conserved={plant.conserved()}")
    if link is not None:
        a = np.asarray(lat)
        print(f"[cell] ADS round-trip  mean {a.mean():.3f} ms  jitter {a.std():.3f} ms  n={len(a)}")

    # -- phase 2: render the recorded snapshots ------------------------------
    import omni.replicator.core as rep
    from PIL import Image

    cam = rep.create.camera(position=(2.0, 3.8, 2.6), look_at=(0.0, 0.0, 0.5))
    rp = rep.create.render_product(cam, (900, 560))
    rgb = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb.attach([rp])
    for _ in range(12):
        rep.orchestrator.step(rt_subframes=10)

    def capture():
        for _ in range(6):
            rep.orchestrator.step(rt_subframes=10)
            im = np.asarray(rgb.get_data())
            if im.ndim == 3 and im.size and im.shape[2] >= 3:
                return im[:, :, :3].astype("uint8")
        return None

    tort_map, box_map = {}, {}
    free_t, free_b = list(range(N_TORT)), list(range(N_BOX))
    print(f"[cell] rendering {len(snaps)} frames ...")
    imgs = []
    for fi, snap in enumerate(snaps):
        for name, tcp in snap["rob"].items():
            pose(stage, f"/World/Cell/{name}", world_to_local(bases[name], tcp))
        live_t = set()
        for (pid, x, y, z) in snap["parts"]:
            live_t.add(pid)
            if pid not in tort_map and free_t:
                tort_map[pid] = free_t.pop(0)
            if pid in tort_map:
                cs.move_prim(stage, f"/World/CT_{tort_map[pid]}", (x, y, z))
        for pid in [k for k in tort_map if k not in live_t]:
            cs.move_prim(stage, f"/World/CT_{tort_map[pid]}", HIDE)
            free_t.append(tort_map.pop(pid))
        live_b = set()
        for (bid, bx, fill) in snap["boxes"]:
            live_b.add(bid)
            if bid not in box_map and free_b:
                box_map[bid] = free_b.pop(0)
            if bid in box_map:
                cs.move_prim(stage, f"/World/CB_{box_map[bid]}", (bx, cs.BOX_Y, cs.BOX_TOP))
        for bid in [k for k in box_map if k not in live_b]:
            cs.move_prim(stage, f"/World/CB_{box_map[bid]}", HIDE)
            free_b.append(box_map.pop(bid))
        arr = capture()
        if arr is not None:
            imgs.append(Image.fromarray(arr))
        if fi % 15 == 0:
            print(f"  frame {fi+1}/{len(snaps)}")

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

    if link is not None:
        link.close()
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("usage: python scripts/run_twincat_cell.py <AMS_NET_ID|mock> [sim_seconds]")
        raise SystemExit(2)
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 50.0
    main(sys.argv[1], secs)
