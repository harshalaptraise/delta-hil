"""The HIL bridge -- the closed loop, made explicit.

One scan = one turn of the loop (principle P1/P2):

    PLC commands --(fast+slow)--> plant  ->  plant physics step
    plant sensors --(fast+slow)--> PLC   ->  PLC logic scan

The bridge is the *only* writer across the seam, so P2 holds by construction:
control can influence nothing that is not a declared tag.

It also meters per-scan round-trip time and jitter. On the mock these numbers
are illustrative only -- eval 5's real thresholds are rig-verifiable, and the
meter exists so the acceptance test has a home when the real PLC + bridge arrive.
Per (A) the meter is reported for the FAST tier alone; the SLOW (OPC UA) tier is
exempt.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field

from .interfaces import PLCLink, PlantModel
from .tags import Dir, Tier


@dataclass
class LatencyMeter:
    samples: list = field(default_factory=list)

    def add(self, seconds: float):
        self.samples.append(seconds)

    def summary_ms(self):
        if not self.samples:
            return {"mean_ms": 0.0, "jitter_ms": 0.0, "n": 0}
        import statistics
        ms = [s * 1e3 for s in self.samples]
        return {
            "mean_ms": statistics.fmean(ms),
            "jitter_ms": statistics.pstdev(ms) if len(ms) > 1 else 0.0,
            "n": len(ms),
        }


class Bridge:
    def __init__(self, plc: PLCLink, plant: PlantModel, dt: float = 0.004):
        self.plc = plc
        self.plant = plant
        self.dt = dt  # 4 ms nominal -> matches EGM's 250 Hz on the real path
        self.fast_meter = LatencyMeter()

    def scan(self) -> None:
        t0 = time.perf_counter()
        # 1. commands PLC -> plant, both tiers
        cmds = {}
        cmds.update(self.plc.read_commands(Tier.FAST))
        cmds.update(self.plc.read_commands(Tier.SLOW))
        self.plant.apply_commands(cmds)
        # 2. advance the plant one physics step
        self.plant.step(self.dt)
        # 3. sensors plant -> PLC, split by tier
        sensors = self.plant.read_sensors()
        fast = {k: v for k, v in sensors.items() if k.startswith("sensor.")}
        self.plc.write_sensors(Tier.FAST, fast)
        # 4. advance controller logic (no-op for a real free-running PLC)
        self.plc.scan(self.dt)
        self.fast_meter.add(time.perf_counter() - t0)

    def run(self, max_scans: int, until=None) -> int:
        for i in range(max_scans):
            self.scan()
            if until is not None and until():
                return i + 1
        return max_scans
