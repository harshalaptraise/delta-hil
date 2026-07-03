"""Headless plant -- enough physics to make P3 real, no GPU required.

A pick succeeds iff the three P3 conditions hold *jointly*:
  1. pose coincidence  -- ||tcp - true_part|| < grip_tol_mm
  2. timing window     -- grip asserted while the TCP is settled on the part
  3. force/friction    -- sufficient (modelled as a fixed capability here)

This is the *coincidence* -- not a decoration. Telemetry's eval 10 doubles as a
check that this stays real: if any of the three were faked, calibrated success
would not track the noise ceiling.

The Isaac seam (``isaac_plant.py``) replaces this class wholesale with a PhysX
Delta; the interface is identical so nothing upstream changes.
"""
from __future__ import annotations

import numpy as np

from ..tags import Dir, TagMap


class MockPlant:
    def __init__(self, grip_tol_mm: float = 0.5, settle_mm: float = 0.05,
                 conveyor_mm_s: float = 0.0):
        self.grip_tol_mm = grip_tol_mm
        self.settle_mm = settle_mm
        self.conveyor_mm_s = conveyor_mm_s
        self._tcp = np.zeros(3)
        self._target = np.zeros(3)
        self._grip = False
        self._true_part = np.zeros(3)
        self._part_present = False
        self._grip_confirm = False

    # -- scenario/telemetry inject ground truth (never visible to the PLC) --
    def set_part(self, true_xyz, present: bool):
        self._true_part = np.zeros(3) if true_xyz is None else np.asarray(true_xyz, float)
        self._part_present = present
        self._grip_confirm = False

    def true_part_xyz(self):
        return tuple(self._true_part)

    # -- PlantModel interface ------------------------------------------------
    def apply_commands(self, values: dict) -> None:
        if "cmd.target_xyz" in values:
            self._target = np.asarray(values["cmd.target_xyz"], float)
        if "cmd.grip" in values:
            self._grip = bool(values["cmd.grip"])

    def step(self, dt: float) -> None:
        # simple critically-damped move toward the commanded target
        self._tcp = self._tcp + 0.6 * (self._target - self._tcp)
        if self.conveyor_mm_s:
            self._true_part = self._true_part + np.array([self.conveyor_mm_s * dt, 0, 0])
        settled = np.linalg.norm(self._tcp - self._target) < self.settle_mm
        coincident = np.linalg.norm(self._tcp - self._true_part) < self.grip_tol_mm
        # P3: pose coincidence AND grip in window AND (force capability assumed)
        if self._part_present and self._grip and settled and coincident:
            self._grip_confirm = True

    def read_sensors(self) -> dict:
        return {
            "sensor.part_present": self._part_present,
            "sensor.grip_confirm": self._grip_confirm,
            "sensor.tcp_xyz": tuple(self._tcp),
        }
