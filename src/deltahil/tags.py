"""The tag map -- the shared-state contract between PLC and plant.

Constitutional role
-------------------
Principle **P2**: the I/O contract is the *only* channel. Nothing crosses the
loop except what is declared here. Principle **(A)**: tags are split into two
tiers with different timing guarantees.

  FAST tier -- motion-critical axis + sensor I/O. Bound by eval 5
               (<10 ms round-trip, sigma < 1 ms jitter). On real hardware this
               is EtherNet/IP implicit or EtherCAT.
  SLOW tier -- supervisory: mode, recipe, scenario select, calibration params,
               metrics. Explicitly EXEMPT from the eval-5 jitter bound. On real
               hardware this is OPC UA on the L8x.

This module is deliberately transport-agnostic. ``bridge.py`` moves these tags;
``plc/`` and ``plant/`` read and write them. Swapping the mock for pycomm3 /
opcua changes the transport, never this contract.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class Tier(Enum):
    FAST = "fast"  # eval-5 jitter bound applies
    SLOW = "slow"  # supervisory, exempt


class Dir(Enum):
    PLC_TO_PLANT = "cmd"     # actuator commands
    PLANT_TO_PLC = "sensor"  # sensor states


@dataclass(frozen=True)
class Tag:
    name: str
    tier: Tier
    direction: Dir
    kind: str  # "bool" | "real" | "vec3" | "int"
    note: str = ""


# --- Fast tier: the real-time loop -----------------------------------------
FAST_TAGS = [
    Tag("cmd.target_xyz", Tier.FAST, Dir.PLC_TO_PLANT, "vec3", "commanded pick point, base frame (mm)"),
    Tag("cmd.grip", Tier.FAST, Dir.PLC_TO_PLANT, "bool", "gripper close request"),
    Tag("cmd.tracking", Tier.FAST, Dir.PLC_TO_PLANT, "bool", "conveyor-tracking enabled"),
    Tag("sensor.part_present", Tier.FAST, Dir.PLANT_TO_PLC, "bool", "part detected at pick window"),
    Tag("sensor.grip_confirm", Tier.FAST, Dir.PLANT_TO_PLC, "bool", "grasp achieved (P3 coincidence)"),
    Tag("sensor.tcp_xyz", Tier.FAST, Dir.PLANT_TO_PLC, "vec3", "tool-centre-point pose (mm)"),
]

# --- Slow tier: supervisory -------------------------------------------------
SLOW_TAGS = [
    Tag("sup.mode", Tier.SLOW, Dir.PLC_TO_PLANT, "int", "0=idle 1=run 2=calibrate"),
    Tag("sup.calib_offset_xyz", Tier.SLOW, Dir.PLC_TO_PLANT, "vec3", "applied vision->base correction (mm)"),
    Tag("sup.cycle_count", Tier.SLOW, Dir.PLANT_TO_PLC, "int", "completed pick cycles"),
    Tag("sup.last_residual_mm", Tier.SLOW, Dir.PLANT_TO_PLC, "real", "reported calibration residual"),
]

ALL_TAGS = FAST_TAGS + SLOW_TAGS


@dataclass
class TagMap:
    """Live values keyed by tag name. The bridge is the only writer per tier."""

    values: dict = field(default_factory=dict)

    def __post_init__(self):
        for tag in ALL_TAGS:
            self.values.setdefault(tag.name, _default(tag.kind))

    def get(self, name: str):
        return self.values[name]

    def set(self, name: str, value):
        self.values[name] = value

    def fast(self):
        return [t for t in ALL_TAGS if t.tier is Tier.FAST]

    def slow(self):
        return [t for t in ALL_TAGS if t.tier is Tier.SLOW]


def _default(kind: str):
    return {"bool": False, "real": 0.0, "int": 0, "vec3": (0.0, 0.0, 0.0)}[kind]
