"""Render the IRB 360 being driven by the LIVE TwinCAT PLC -> GIF.

The full HIL money shot: the real robot moves because the real controller
commanded it. Closes TwinCAT <-> Bridge <-> MockPlant over ADS (the plant gives
the PLC its sensors and eases the TCP toward the PLC's target), and poses the
articulated IRB 360 to that live TCP each frame, capturing a GIF.

Run on the rig, inside isaacenv, with TwinCAT running (GVLs + the dwell MAIN from
docs/twincat_program.md, config activated):

    python scripts/run_twincat_render.py 5.1.204.123.1.1        # AMS NetId
    python scripts/run_twincat_render.py 5.1.204.123.1.1 90     # + frame count

Output: assets/render/twincat_pick.gif
"""
from __future__ import annotations

import os
import sys

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from pxr import UsdLux  # noqa: E402

from deltahil.plant.irb360_pose import pose  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
USD = os.path.join(REPO, "assets", "irb360.usd").replace("\\", "/")
RENDER_DIR = os.path.join(REPO, "assets", "render").replace("\\", "/")
OUT_GIF = os.path.join(RENDER_DIR, "twincat_pick.gif").replace("\\", "/")
os.makedirs(RENDER_DIR, exist_ok=True)

PICK = (120.0, -80.0, -1180.0)   # must match `pick` in the TwinCAT MAIN


def main(ams_net_id: str, frames: int = 90) -> int:
    from deltahil.bridge import Bridge
    from deltahil.plant.mock_plant import MockPlant
    from deltahil.plc.twincat_plc import TwinCATAdsLink

    omni.usd.get_context().open_stage(USD)
    for _ in range(60):
        app.update()
    stage = omni.usd.get_context().get_stage()
    UsdLux.DomeLight.Define(stage, "/World/Light_Dome").CreateIntensityAttr(700.0)
    UsdLux.DistantLight.Define(stage, "/World/Light_Key").CreateIntensityAttr(2500.0)

    import omni.replicator.core as rep
    from PIL import Image

    cam = rep.create.camera(position=(2600, -3000, 400), look_at=(60, -40, -1050))
    rp = rep.create.render_product(cam, (860, 600))
    rgb = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb.attach([rp])
    for _ in range(12):
        rep.orchestrator.step(rt_subframes=8)

    def capture():
        for _ in range(6):
            rep.orchestrator.step(rt_subframes=8)
            a = np.asarray(rgb.get_data())
            if a.ndim == 3 and a.size and a.shape[2] >= 3:
                return a[:, :, :3].astype("uint8")
        return None

    plc = TwinCATAdsLink(ams_net_id)          # live TwinCAT over ADS
    plant = MockPlant()
    plant.set_part(PICK, present=True)        # a part where the PLC picks
    bridge = Bridge(plc, plant)

    print(f"[twincat/render] AMS={ams_net_id}  {frames} frames driven by the live PLC ...")
    imgs, last = [], -1
    for i in range(frames):
        bridge.scan()                          # PLC <-> plant; PLC commands the target
        tcp = plant.read_sensors()["sensor.tcp_xyz"]
        pose(stage, "/World/IRB360", np.asarray(tcp, float))   # robot follows the live TCP
        arr = capture()
        if arr is not None:
            imgs.append(Image.fromarray(arr))
        cyc = plc._plc.read_by_name("GVL_Sup.cycle_count", plc._plctype("int"))
        if cyc != last and cyc > 0:            # re-arm a fresh part each completed cycle
            plant.set_part(PICK, present=True); last = cyc
        if i % 10 == 0:
            print(f"  frame {i+1}/{frames}  tcp=({tcp[0]:.0f},{tcp[1]:.0f},{tcp[2]:.0f})  cycles={cyc}")

    if imgs:
        try:
            pal = imgs[len(imgs) // 2].convert("P", palette=Image.ADAPTIVE, colors=128)
            fp = [im.quantize(palette=pal, dither=Image.Dither.NONE) for im in imgs]
            fp[0].save(OUT_GIF, save_all=True, append_images=fp[1:], duration=90, loop=0, disposal=2)
        except Exception as exc:
            print(f"[twincat/render] palette quantize failed ({exc}); saving RGB gif")
            imgs[0].save(OUT_GIF, save_all=True, append_images=imgs[1:], duration=90, loop=0)
        print(f"\n[twincat/render] wrote {OUT_GIF}  exists={os.path.exists(OUT_GIF)}  frames={len(imgs)}\n")
    else:
        print("\n[twincat/render] no frames captured\n")

    plc.close()
    return 0


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("usage: python scripts/run_twincat_render.py <AMS_NET_ID> [frames]")
        raise SystemExit(2)
    nframes = int(sys.argv[2]) if len(sys.argv) > 2 else 90
    raise SystemExit(main(sys.argv[1], nframes))
