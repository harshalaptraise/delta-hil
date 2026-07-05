"""Stage-2 TwinCAT loop: TwinCAT <-> Bridge <-> KinematicDeltaPlant over ADS.

Same HIL loop as run_twincat_mock.py but with the Isaac kinematic Delta plant
(boots Isaac). Run on the rig, inside isaacenv, with TwinCAT running (GVLs + MAIN
from docs/twincat_program.md):

    python scripts/run_twincat_loop.py 5.1.204.123.1.1        # AMS NetId
    python scripts/run_twincat_loop.py 5.1.204.123.1.1 30     # + seconds

Reports the FAST-tier latency/jitter (bridge.fast_meter) -- the eval-5 home.
"""
from __future__ import annotations

import sys


def main(ams_net_id: str, seconds: float = 20.0) -> int:
    from deltahil.bridge import Bridge
    from deltahil.plant.kinematic_delta import KinematicDeltaPlant
    from deltahil.plc.twincat_plc import TwinCATAdsLink
    from deltahil.tags import Tier

    plant = KinematicDeltaPlant(headless=True)          # boots Isaac
    plc = TwinCATAdsLink(ams_net_id)                    # direct ADS (pyads)
    plant.set_part((0.0, 0.0, -900.0), present=True)    # part where the PLC picks
    bridge = Bridge(plc, plant)

    n = int(round(seconds / bridge.dt))
    print(f"[twincat] AMS={ams_net_id}  Isaac loop {seconds:.0f}s ({n} scans) ...")
    last = -1
    for i in range(n):
        bridge.scan()                                  # plant.step drives Isaac physics
        if i % 250 == 0:
            cmd = plc.read_commands(Tier.FAST)
            s = plant.read_sensors()
            cyc = plc._plc.read_by_name("GVL_Sup.cycle_count", plc._plctype("int"))
            print(f"  scan {i:5d}  target={tuple(round(v) for v in cmd['cmd.target_xyz'])} "
                  f"grip={cmd['cmd.grip']}  confirm={s['sensor.grip_confirm']}  cycles={cyc}")
            if cyc != last and cyc > 0:
                plant.set_part((0.0, 0.0, -900.0), present=True)
                last = cyc

    lat = bridge.fast_meter.summary_ms()
    print("-" * 56)
    print(f"fast-tier scan  mean {lat['mean_ms']:.3f} ms  jitter {lat['jitter_ms']:.3f} ms  n={lat['n']}")
    print("  (eval 5 needs <10 ms round-trip, sigma<1 ms -- rig-verifiable; ADS "
          "polling may need true EtherCAT process-image I/O to meet it)")
    print("-" * 56)
    plc.close()
    return 0


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("usage: python scripts/run_twincat_loop.py <AMS_NET_ID> [seconds]")
        raise SystemExit(2)
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
    raise SystemExit(main(sys.argv[1], secs))
