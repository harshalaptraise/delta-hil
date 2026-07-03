"""Headless stand-in for the ControlLogix DUT.

Runs a minimal pick-and-place state machine so the loop closes without real
hardware. It obeys the same discipline the real PLC does:

  * acts ONLY on sensor tags it has sampled (P2 -- never touches ground truth);
  * applies the calibration correction it was given on the SLOW tier, then
    commands a pick on the FAST tier;
  * free-runs its own scan (P1) -- here we advance it explicitly, but it never
    reads plant internals.

The real seam is ``logix_plc.py`` (pycomm3 fast tier + opcua slow tier). Note
that swapping it in means the *real deployed ladder/ST program* becomes the
state machine -- this mock exists only so the harness is runnable and testable.
"""
from __future__ import annotations

import numpy as np

from ..tags import Dir, Tier


class MockPLC:
    def __init__(self):
        self._sensors = {}          # last sampled sensor tags
        self._calib_offset = np.zeros(3)
        self._pending_target = None
        self._cycle = 0
        self._phase = "await_part"  # await_part -> approach -> grip -> done

    def set_target_from_vision(self, reported_xyz):
        """Supervisor hands the PLC a vision report (already the PLC's to use).
        The PLC applies its slow-tier calibration correction before commanding."""
        self._pending_target = None if reported_xyz is None else np.asarray(reported_xyz, float)
        self._phase = "await_part"

    def set_calibration(self, offset_xyz):
        self._calib_offset = np.asarray(offset_xyz, float)

    # -- PLCLink interface ---------------------------------------------------
    def read_commands(self, tier: Tier) -> dict:
        if tier is Tier.FAST:
            if self._pending_target is None or self._phase == "done":
                return {"cmd.target_xyz": (0.0, 0.0, 0.0), "cmd.grip": False, "cmd.tracking": False}
            corrected = self._pending_target + self._calib_offset
            grip = self._phase == "grip"
            return {"cmd.target_xyz": tuple(corrected), "cmd.grip": grip, "cmd.tracking": False}
        return {"sup.mode": 1, "sup.calib_offset_xyz": tuple(self._calib_offset)}

    def write_sensors(self, tier: Tier, values: dict) -> None:
        self._sensors.update(values)

    def scan(self, dt: float) -> None:
        present = self._sensors.get("sensor.part_present", False)
        confirm = self._sensors.get("sensor.grip_confirm", False)
        if self._phase == "await_part" and present and self._pending_target is not None:
            self._phase = "approach"
        elif self._phase == "approach":
            self._phase = "grip"
        elif self._phase == "grip" and confirm:
            self._phase = "done"
            self._cycle += 1

    @property
    def cycle_count(self):
        return self._cycle

    @property
    def done(self):
        return self._phase == "done"

    @property
    def corrected_target(self):
        if self._pending_target is None:
            return None
        return self._pending_target + self._calib_offset
