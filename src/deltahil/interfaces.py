"""The swap points. Everything real drops in behind these two interfaces.

``PLCLink``   -- the controller side. MockPLC (headless) or LogixLink (pycomm3
                fast tier + opcua slow tier) both implement it.
``PlantModel`` -- the plant side. MockPlant (headless kinematic coincidence) or
                IsaacPlant (PhysX Delta) both implement it.

The bridge only ever talks to these abstractions, so the constitution is
enforced at the seam, not in any one implementation.
"""
from __future__ import annotations

from typing import Protocol

from .tags import Dir, Tier


class PLCLink(Protocol):
    """Controller under test. Reads sensor tags, writes command tags."""

    def read_commands(self, tier: Tier) -> dict:
        """Return {tag_name: value} for command tags on this tier."""
        ...

    def write_sensors(self, tier: Tier, values: dict) -> None:
        """Push sensor tag values (this tier) into the controller's inputs."""
        ...

    def scan(self, dt: float) -> None:
        """Advance the controller one logic scan. On real hardware this is a
        no-op (the PLC free-runs on its own oscillator -- principle P1)."""
        ...


class PlantModel(Protocol):
    """Virtual cell. Consumes command tags, produces sensor tags."""

    def apply_commands(self, values: dict) -> None:
        ...

    def step(self, dt: float) -> None:
        """Advance physics by dt seconds."""
        ...

    def read_sensors(self) -> dict:
        ...

    def true_part_xyz(self):
        """Ground-truth part pose -- visible to the sim, never to the PLC
        (principle P2). Used only by telemetry/scenario, never by control."""
        ...
