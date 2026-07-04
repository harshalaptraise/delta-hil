"""Close the HIL loop against a live TwinCAT PLC via the Loupe Beckhoff Bridge.

Run on the rig, inside isaacenv, with TwinCAT running and the Loupe extension on
the Isaac extension path:

    python scripts/run_twincat_loop.py 5.1.204.123.1.1        # AMS NetId
    python scripts/run_twincat_loop.py 5.1.204.123.1.1 30     # + seconds

Wires TwinCATLink + KinematicDeltaPlant + Bridge and interleaves app.update()
(drives the bridge's cyclic ADS reads -> data callback) with bridge.scan().
Reports the FAST-tier latency/jitter (bridge.fast_meter) -- the eval-5 home
(rig-verifiable only). See docs/twincat_gvl_spec.md for the TwinCAT side.
"""
from __future__ import annotations

import sys


def main(ams_net_id: str, seconds: float = 20.0) -> int:
    from deltahil.bridge import Bridge
    from deltahil.plant.kinematic_delta import KinematicDeltaPlant
    from deltahil.tags import Tier

    # 1) plant boots Isaac (SimulationApp) via _boot_isaac (one app / process)
    plant = KinematicDeltaPlant(headless=True)
    app = plant._sim_app

    # 2) enable the Loupe Beckhoff Bridge extension, then create the link
    import omni.kit.app
    mgr = omni.kit.app.get_app().get_extension_manager()
    try:
        mgr.set_extension_enabled_immediate("loupe.simulation.beckhoff_bridge", True)
    except Exception as exc:  # not on the path -> clear message
        print(f"[twincat] could not enable the Loupe extension: {exc}\n"
              f"          add its exts/ to the Isaac extension search path "
              f"(see docs/twincat_gvl_spec.md).")
        return 1

    from deltahil.plc.twincat_plc import TwinCATLink
    plc = TwinCATLink(ams_net_id)

    # a part to pick (ground truth); the TwinCAT program commands the target
    plant.set_part((0.0, 0.0, -900.0), present=True)
    bridge = Bridge(plc, plant)

    n = int(round(seconds / bridge.dt))
    print(f"[twincat] AMS={ams_net_id}  running {seconds:.0f}s ({n} scans) ...")
    for i in range(n):
        app.update()            # drive the bridge's cyclic ADS read -> data callback
        bridge.scan()           # cmd (from PLC) -> plant -> sensor (to PLC)
        if i % 250 == 0:
            cmd = plc.read_commands(Tier.FAST)
            s = plant.read_sensors()
            print(f"  scan {i:5d}  cmd.target={tuple(round(v) for v in cmd['cmd.target_xyz'])} "
                  f"grip={cmd['cmd.grip']}  grip_confirm={s['sensor.grip_confirm']}")

    lat = bridge.fast_meter.summary_ms()
    print("-" * 56)
    print(f"fast-tier scan  mean {lat['mean_ms']:.3f} ms  jitter {lat['jitter_ms']:.3f} ms  n={lat['n']}")
    print("  (eval 5 needs <10 ms round-trip, sigma<1 ms -- rig-verifiable; ADS "
          "polling may need true EtherCAT process-image I/O to meet it)")
    print("-" * 56)
    return 0


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("usage: python scripts/run_twincat_loop.py <AMS_NET_ID> [seconds]")
        raise SystemExit(2)
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
    raise SystemExit(main(sys.argv[1], secs))
