"""Stage-1 TwinCAT bring-up: TwinCAT <-> Bridge <-> MockPlant over ADS, NO Isaac.

The fastest way to prove the real controller closes the loop -- pure Python +
pyads, no Isaac boot. Run on the rig with TwinCAT running (GVLs + MAIN from
docs/twincat_program.md, config activated):

    python scripts/run_twincat_mock.py 5.1.204.123.1.1        # AMS NetId
    python scripts/run_twincat_mock.py 5.1.204.123.1.1 15     # + seconds

Watch cmd.target follow the PLC and grip_confirm/cycle_count advance. Reports the
FAST-tier round-trip latency (bridge.fast_meter) -- the eval-5 home.
"""
from __future__ import annotations

import sys


def main(ams_net_id: str, seconds: float = 15.0) -> int:
    from deltahil.bridge import Bridge
    from deltahil.plant.mock_plant import MockPlant
    from deltahil.plc.twincat_plc import TwinCATAdsLink
    from deltahil.tags import Tier

    plc = TwinCATAdsLink(ams_net_id)         # raises if pyads/route missing
    plant = MockPlant()
    plant.set_part((0.0, 0.0, -900.0), present=True)   # a part where the PLC picks
    bridge = Bridge(plc, plant)

    n = int(round(seconds / bridge.dt))
    print(f"[twincat/ads] AMS={ams_net_id}  mock loop {seconds:.0f}s ({n} scans) ...")
    last_cycles = -1
    for i in range(n):
        bridge.scan()
        if i % 200 == 0:
            cmd = plc.read_commands(Tier.FAST)
            sup = plc.read_commands(Tier.SLOW)
            s = plant.read_sensors()
            cyc = plc._plc.read_by_name("GVL_Sup.cycle_count", plc._plctype("int"))
            print(f"  scan {i:5d}  target={tuple(round(v) for v in cmd['cmd.target_xyz'])} "
                  f"grip={cmd['cmd.grip']}  present={s['sensor.part_present']} "
                  f"confirm={s['sensor.grip_confirm']}  cycles={cyc}")
            if cyc != last_cycles and cyc > 0:      # re-arm a fresh part per cycle
                plant.set_part((0.0, 0.0, -900.0), present=True)
                last_cycles = cyc

    lat = bridge.fast_meter.summary_ms()
    print("-" * 56)
    print(f"fast-tier scan  mean {lat['mean_ms']:.3f} ms  jitter {lat['jitter_ms']:.3f} ms  n={lat['n']}")
    print("  (eval 5 needs <10 ms round-trip, sigma<1 ms -- rig-verifiable)")
    print("-" * 56)
    plc.close()
    return 0


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("usage: python scripts/run_twincat_mock.py <AMS_NET_ID> [seconds]")
        raise SystemExit(2)
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0
    raise SystemExit(main(sys.argv[1], secs))
