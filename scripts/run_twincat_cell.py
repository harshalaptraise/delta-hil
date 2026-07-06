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
from pxr import Gf, UsdGeom, UsdLux  # noqa: E402

from deltahil.plant import cell_scene as cs  # noqa: E402
from deltahil.plant.cell_plant import (BOX_TOP, BOX_Y, CellPlant, STACK0,  # noqa: E402
                                        THICK, VEL_TOL)
from deltahil.plant.irb360_pose import pose, world_to_local  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IRB360 = os.path.join(REPO, "assets", "irb360.usd").replace("\\", "/")
RENDER_DIR = os.path.join(REPO, "assets", "render").replace("\\", "/")
OUT_GIF = os.path.join(RENDER_DIR, "twincat_cell.gif").replace("\\", "/")
os.makedirs(RENDER_DIR, exist_ok=True)

N_TORT, N_BOX = 44, 16
HIDE = (6.0, 4.0, -3.0)
RES = (1600, 1000)      # HD render
RT_SUB = 32             # RTX subframes/frame (higher -> less per-frame noise -> less flicker)
VPLOT = True            # overlay the TCP-vx-vs-belt strip chart (top-right)


def snapshot(plant, dt, cmds):
    rob, vx, vc, grip = {}, {}, {}, {}
    for n, rb in plant.robots.items():
        rob[n] = (rb["tcp"][0], rb["tcp"][1], rb["tcp"][2])
        vx[n] = (rb["tcp"][0] - rb["tcp_prev"][0]) / dt if dt > 0 else 0.0   # realized TCP X-vel
        vc[n] = float(cmds.get(n, {}).get("vel", (0.0, 0.0, 0.0))[0])        # commanded vff (X)
        grip[n] = rb["carry"] is not None
    return {
        "rob": rob, "vx": vx, "vcmd": vc, "grip": grip,
        "vsrc": plant.vs, "vbox": plant.vb,
        "parts": [(p["id"], p["x"], p["y"], p["z"]) for p in plant.parts],
        "boxes": [(b["id"], b["x"], b["fill"]) for b in plant.boxes],
    }


def _draw_vplot(im, snaps, fi, win=120):
    """Top-right strip chart: each robot's realized TCP X-velocity vs the belt
    speeds, so the conveyor-tracking velocity match is visible frame by frame --
    during a pick the vx trace rides the source-belt line; o marks a grab."""
    from PIL import ImageDraw
    d = ImageDraw.Draw(im, "RGBA")
    PW, PH, M = 360, 190, 18
    x0, y0 = im.width - PW - M, M
    d.rectangle([x0, y0, x0 + PW, y0 + PH], fill=(14, 16, 20, 195), outline=(90, 95, 105, 255))
    padl, padt, padb = 8, 22, 16
    gx0, gy0 = x0 + padl, y0 + padt
    gw, gh = PW - padl - 10, PH - padt - padb
    VMAX = 0.35
    def vy(v): return gy0 + gh - (min(max(v, 0.0), VMAX) / VMAX) * gh
    seg = snaps[max(0, fi - win + 1):fi + 1]
    n = len(seg)
    def xat(i): return gx0 + gw - (n - 1 - i) / max(win - 1, 1) * gw
    vsrc, vbox = seg[-1]["vsrc"], seg[-1]["vbox"]
    d.line([gx0, vy(vsrc), gx0 + gw, vy(vsrc)], fill=(90, 220, 130, 255), width=1)   # belt (src)
    d.line([gx0, vy(vbox), gx0 + gw, vy(vbox)], fill=(90, 150, 240, 170), width=1)   # box belt
    for nm, col in (("Robot_A", (90, 220, 245, 255)), ("Robot_B", (250, 180, 80, 255))):
        pts = [(xat(i), vy(seg[i]["vx"][nm])) for i in range(n)]
        if len(pts) > 1:
            d.line(pts, fill=col, width=2)
        for i in range(1, n):                              # mark grabs (carry begins)
            if seg[i]["grip"][nm] and not seg[i - 1]["grip"][nm]:
                px, py = xat(i), vy(seg[i]["vx"][nm])
                d.ellipse([px - 3, py - 3, px + 3, py + 3], fill=(255, 255, 255, 255))
    d.text((x0 + 8, y0 + 4), "TCP vx (X) vs belt", fill=(235, 235, 240, 255))
    d.text((gx0 + gw - 62, vy(vsrc) - 12), f"belt {vsrc:.2f}", fill=(90, 220, 130, 255))
    d.text((x0 + 8, y0 + PH - 14), "A", fill=(90, 220, 245, 255))
    d.text((x0 + 24, y0 + PH - 14), "B   o=pick", fill=(250, 180, 80, 255))
    return im


def _polish(stage):
    """Additive industrial dressing (cell_scene stays frozen): brushed steel on the
    frame, a base slab + back/end panels, and the tortilla belt recoloured light
    brown + grain. Nothing on the camera-facing (+Y) side or over the belts."""
    from pxr import Sdf, UsdShade

    def pbr(path, diffuse, metallic, roughness):
        m = UsdShade.Material.Define(stage, path)
        s = UsdShade.Shader.Define(stage, path + "/PBR")
        s.CreateIdAttr("UsdPreviewSurface")
        s.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*diffuse))
        s.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
        s.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
        m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(), "surface")
        return m, s

    def bind(prim, m):
        try:
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(m)
        except Exception:
            UsdShade.MaterialBindingAPI(prim).Bind(m)

    steel, _ = pbr("/World/Look/Steel", (0.34, 0.36, 0.40), 0.65, 0.55)   # brushed, less mirror
    for prim in stage.Traverse():
        nm = prim.GetName()
        if nm.startswith("Frame") or nm.startswith("Mount"):
            bind(prim, steel)

    # --- source (tortilla) belt -> brown, matte, with a little real grain ---
    brown, bsh = pbr("/World/Look/Belt", (0.42, 0.30, 0.19), 0.0, 0.92)
    src = stage.GetPrimAtPath("/World/SrcConveyor")
    if src.IsValid():
        try:                                               # narrow the belt (top height unchanged)
            xf = UsdGeom.Xformable(src)
            xf.ClearXformOpOrder()
            xf.AddTransformOp().Set(
                Gf.Matrix4d().SetScale(Gf.Vec3d(cs.BELT_LEN, 0.26, 0.14))
                * Gf.Matrix4d().SetTranslate(Gf.Vec3d(0.0, cs.SRC_Y, cs.SRC_TOP - 0.07)))
        except Exception:
            pass
        bind(src, brown)
        try:
            import numpy as _np
            from PIL import Image as _Img
            rng = _np.random.default_rng(3)
            g = _np.clip(_np.array([107, 77, 48], _np.float32)
                         + rng.normal(0, 14, (256, 256, 1)), 0, 255).astype(_np.uint8)
            tp = os.path.join(RENDER_DIR, "belt_grain.png").replace("\\", "/")
            _Img.fromarray(g).save(tp)                       # g is already (H,W,3)
            rd = UsdShade.Shader.Define(stage, "/World/Look/Belt/ST")
            rd.CreateIdAttr("UsdPrimvarReader_float2")
            rd.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
            tx = UsdShade.Shader.Define(stage, "/World/Look/Belt/Tex")
            tx.CreateIdAttr("UsdUVTexture")
            tx.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(tp)
            tx.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
            tx.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
            tx.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
                rd.CreateOutput("result", Sdf.ValueTypeNames.Float2))
            bsh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
                tx.CreateOutput("rgb", Sdf.ValueTypeNames.Float3))
            skin = UsdGeom.Mesh.Define(stage, "/World/Polish/SrcSkin")   # UV'd top so grain maps
            hx, y0, hy, z = cs.BELT_LEN / 2, cs.SRC_Y, 0.12, cs.SRC_TOP + 0.002
            skin.CreatePointsAttr([(-hx, y0 - hy, z), (hx, y0 - hy, z),
                                   (hx, y0 + hy, z), (-hx, y0 + hy, z)])
            skin.CreateFaceVertexCountsAttr([4])
            skin.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
            UsdGeom.PrimvarsAPI(skin).CreatePrimvar(
                "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.varying).Set(
                [(0, 0), (12, 0), (12, 2), (0, 2)])
            bind(skin.GetPrim(), brown)
        except Exception as exc:
            print(f"[cell] belt grain skipped ({exc})")

    def slab(path, size, pos, color, m=steel):
        c = UsdGeom.Cube.Define(stage, path)
        c.CreateSizeAttr(1.0)
        c.AddTransformOp().Set(Gf.Matrix4d().SetScale(Gf.Vec3d(*size))
                               * Gf.Matrix4d().SetTranslate(Gf.Vec3d(*pos)))
        c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        if m is not None:
            bind(c.GetPrim(), m)

    # raise + narrow the box belt to match the logic (cell_scene frozen), + support posts
    box = stage.GetPrimAtPath("/World/BoxConveyor")
    if box.IsValid():
        try:
            xb = UsdGeom.Xformable(box)
            xb.ClearXformOpOrder()
            xb.AddTransformOp().Set(
                Gf.Matrix4d().SetScale(Gf.Vec3d(cs.BELT_LEN, 0.30, 0.14))
                * Gf.Matrix4d().SetTranslate(Gf.Vec3d(0.0, BOX_Y, BOX_TOP - 0.07)))
        except Exception:
            pass
    for i, sx in enumerate((-1.1, 0.0, 1.1)):
        slab(f"/World/Polish/BoxStand_{i}", (0.08, 0.08, BOX_TOP - 0.14),
             (sx, BOX_Y, (BOX_TOP - 0.14) / 2.0), (0.10, 0.11, 0.13))

    L, W = cs.FR_L, cs.FR_W
    slab("/World/Polish/Base",      (L + 0.5, W + 0.5, 0.10), (0.0, 0.0, -0.05), (0.13, 0.14, 0.16), m=None)
    slab("/World/Polish/BackPanel", (L, 0.04, 1.45), (0.0, -W / 2.0 - 0.02, 0.83), (0.22, 0.24, 0.27))
    slab("/World/Polish/KickL",     (0.05, W, 0.30), (-L / 2.0 - 0.02, 0.0, 0.15), (0.17, 0.18, 0.21))
    slab("/World/Polish/KickR",     (0.05, W, 0.30), (L / 2.0 + 0.02, 0.0, 0.15), (0.17, 0.18, 0.21))


def main(ams, sim_seconds=50.0, dt=0.01, sample_every=7):
    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    bases = cs.build_cell(stage, IRB360)
    for i in range(N_TORT):
        cs.spawn_tortilla(stage, f"/World/CT_{i}", HIDE)
        pr = stage.GetPrimAtPath(f"/World/CT_{i}")
        if pr.IsValid():
            UsdGeom.Gprim(pr).CreateDisplayColorAttr([Gf.Vec3f(0.93, 0.91, 0.86)])  # off-white
    for i in range(N_BOX):
        cs.spawn_box(stage, f"/World/CB_{i}", HIDE)
    for _ in range(60):
        app.update()
    try:
        _polish(stage)                                     # industrial dressing (never fatal)
    except Exception as exc:
        print(f"[cell] polish skipped ({exc})")

    try:
        # diffused: soft ambient dome dominant, gentle wide-angle key (soft shadows)
        UsdLux.DomeLight.Define(stage, "/World/Light_Dome").CreateIntensityAttr(900.0)
        key = UsdLux.DistantLight.Define(stage, "/World/Light_Key")
        key.CreateIntensityAttr(500.0)
        key.CreateColorAttr(Gf.Vec3f(1.0, 0.96, 0.90))     # gently warm
        key.CreateAngleAttr(6.0)                            # wide -> soft shadow edges
        UsdGeom.Xformable(key.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-55.0, 0.0, 30.0))
        fill = UsdLux.DistantLight.Define(stage, "/World/Light_Fill")
        fill.CreateIntensityAttr(300.0)
        fill.CreateColorAttr(Gf.Vec3f(0.88, 0.92, 1.0))    # soft cool fill
        fill.CreateAngleAttr(6.0)
        UsdGeom.Xformable(fill.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-30.0, 0.0, -150.0))
    except Exception as exc:
        print(f"[cell] 3-point lights failed ({exc}); simple lights")
        UsdLux.DomeLight.Define(stage, "/World/Light_Dome").CreateIntensityAttr(800.0)
        UsdLux.DistantLight.Define(stage, "/World/Light_Key").CreateIntensityAttr(600.0)

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
            cmds = ctrl.decide(sensors, dt)
            plant.apply_commands(cmds)
            plant.step(dt)
            sim_t += dt
            if sim_t >= next_snap:
                snaps.append(snapshot(plant, dt, cmds)); next_snap += SNAP_DT
    else:
        # live PLC: run in REAL wall-clock time so the PLC's TON timers line up with
        # the sim motion; GVL_Cell.enable (forced in the Watch) gates all motion --
        # FALSE freezes the sim, TRUE runs it. Snapshots on wall-time so a freeze
        # shows in the render.
        start = time.perf_counter(); prev = start; prev_plc = None; clock_src = "wall clock"
        seen_ids, ab = set(), {"Robot_A": 0, "Robot_B": 0}
        vm_pick, vm_place = [], []
        while (time.perf_counter() - start) < sim_seconds:
            sensors = plant.read_sensors()
            t0 = time.perf_counter()
            link.write_sensors(sensors)
            cmds, enable, plc_ns = link.read_commands()
            lat.append((time.perf_counter() - t0) * 1000.0)
            now = time.perf_counter()
            # SAMPLED-DATA: advance the continuous plant by the PLC's OWN elapsed time
            # between samples. Fall back to the wall clock if the PLC doesn't publish it.
            if plc_ns is not None and prev_plc is not None:
                rdt = min((plc_ns - prev_plc) / 1.0e9, 0.05)   # PLC clock; clamp MAX only
                clock_src = "PLC clock (GVL_Cell.plc_time_ns)"  # rdt may be 0 (no new tick)
            else:
                rdt = min(max(now - prev, 0.001), 0.05)        # wall-clock fallback
            prev, prev_plc = now, plc_ns
            plant.apply_commands(cmds)
            vdt = 0.01
            if enable and rdt > 0.0:
                # sub-step the CONTINUOUS plant between (coarse) PLC ticks so fast
                # tracking stays smooth -- the control sample rate is unchanged (one
                # command per sample), only the plant integration is finer.
                nsub = max(1, min(8, int(rdt / 0.003 + 0.5)))
                vdt = rdt / nsub
                for _ in range(nsub):
                    plant.step(vdt)
                for pt in plant.parts:                          # tally which robot picked
                    if pt["state"] in ("carried", "placed") and pt["id"] not in seen_ids:
                        seen_ids.add(pt["id"]); ab[pt["robot"]] += 1
                for nm, rb in plant.robots.items():             # velocity-lock diagnostic
                    vcmd = cmds[nm].get("vel", (0.0, 0.0, 0.0))[0]
                    if abs(vcmd) > 1e-6:                         # a feed-forward is commanded
                        vx = (rb["tcp"][0] - rb["tcp_prev"][0]) / vdt if vdt > 0 else 0.0
                        (vm_pick if abs(vcmd - plant.vs) < 1e-6 else vm_place).append(abs(vx - vcmd))
            if (now - start) >= next_snap:
                snaps.append(snapshot(plant, vdt, cmds)); next_snap += SNAP_DT
        print(f"[cell] sim clock source: {clock_src}")
        print(f"[cell] per-robot picks  A/B = {ab['Robot_A']}/{ab['Robot_B']}")

        def _vrep(label, arr):
            if not arr:
                return
            locked = sum(1 for e in arr if e < VEL_TOL)
            print(f"[cell]   {label}: {locked}/{len(arr)} steps locked < {VEL_TOL} m/s "
                  f"(mean {sum(arr) / len(arr):.4f} m/s)")
        print("[cell] velocity feed-forward vx-match (X):")
        _vrep("pick track  (vs source belt)", vm_pick)   # the pick match you care about
        _vrep("place track (vs box belt, incl. slew)", vm_place)
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
    rp = rep.create.render_product(cam, RES)
    rgb = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb.attach([rp])
    for _ in range(24):                                    # longer warm-up -> denoiser settles
        rep.orchestrator.step(rt_subframes=RT_SUB)

    def capture():
        for _ in range(6):
            rep.orchestrator.step(rt_subframes=RT_SUB)
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
                cs.move_prim(stage, f"/World/CB_{box_map[bid]}", (bx, BOX_Y, BOX_TOP))
        for bid in [k for k in box_map if k not in live_b]:
            cs.move_prim(stage, f"/World/CB_{box_map[bid]}", HIDE)
            free_b.append(box_map.pop(bid))
        arr = capture()
        if arr is not None:
            im = Image.fromarray(arr)
            if VPLOT:
                try:
                    _draw_vplot(im, snaps, fi)
                except Exception as exc:
                    if fi == 0:
                        print(f"[cell] vx overlay skipped ({exc})")
            imgs.append(im)
        if fi % 15 == 0:
            print(f"  frame {fi+1}/{len(snaps)}")

    if imgs:
        out = None
        out_mp4 = OUT_GIF[:-4] + ".mp4"
        try:                                                 # HD H.264 -- right format for HD
            import imageio
            imageio.mimwrite(out_mp4, [np.asarray(im) for im in imgs],
                             fps=15, codec="libx264", quality=8, macro_block_size=8)
            out = out_mp4
        except Exception as exc:
            print(f"[cell] mp4 encode unavailable ({exc}); writing a downscaled gif")
        if out is None:                                      # gif fallback (downscaled, shared palette)
            sm = [im.resize((1000, 1000 * im.height // im.width)) for im in imgs]
            try:
                pal = sm[len(sm) // 2].convert("P", palette=Image.ADAPTIVE, colors=128)
                fp = [im.quantize(palette=pal, dither=Image.Dither.NONE) for im in sm]
                fp[0].save(OUT_GIF, save_all=True, append_images=fp[1:], duration=70, loop=0, disposal=2)
            except Exception:
                sm[0].save(OUT_GIF, save_all=True, append_images=sm[1:], duration=70, loop=0)
            out = OUT_GIF
        print(f"\n[cell] wrote {out}  exists={os.path.exists(out)}  frames={len(imgs)}\n")
    else:
        print("\n[cell] no frames captured\n")

    try:                                                       # velocity trace for offline plotting
        import csv
        cp = OUT_GIF[:-4] + "_velocity.csv"
        with open(cp, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["frame", "t_s", "vx_A", "vx_B", "vcmd_A", "vcmd_B",
                        "belt_src", "belt_box", "carry_A", "carry_B"])
            for i, s in enumerate(snaps):
                w.writerow([i, round(i * SNAP_DT, 3),
                            round(s["vx"]["Robot_A"], 4), round(s["vx"]["Robot_B"], 4),
                            round(s["vcmd"]["Robot_A"], 4), round(s["vcmd"]["Robot_B"], 4),
                            s["vsrc"], s["vbox"],
                            int(s["grip"]["Robot_A"]), int(s["grip"]["Robot_B"])])
        print(f"[cell] wrote velocity trace {cp}")
    except Exception as exc:
        print(f"[cell] velocity csv skipped ({exc})")

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
