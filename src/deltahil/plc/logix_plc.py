"""ControlLogix 1756-L8x link -- the real controller seam. STUB.

Implements the two-tier I/O contract (A) against real hardware:

  FAST tier -- EtherNet/IP implicit (or EtherCAT). This is the motion-critical
               path under the eval-5 jitter bound. pycomm3's tag access is
               request/response and is fine for setup, but the <10 ms /
               sigma<1 ms loop must ride implicit I/O, not polled reads. Measure
               it on the rig; do not trust desk numbers (eval 5 is
               rig-verifiable only).
  SLOW tier -- OPC UA, native on the L8x from firmware V36. Carries mode,
               recipe, calibration params, metrics. Explicitly EXEMPT from the
               jitter bound -- which is *why* calibration output goes here and
               never on a servo signal.

The controller free-runs (P1): ``scan()`` is a no-op -- we never step the real
PLC, we only exchange tags with it.
"""
from __future__ import annotations

from ..tags import Tier


class LogixLink:
    def __init__(self, host: str, opcua_url: str | None = None):
        raise NotImplementedError(
            "LogixLink needs pycomm3 (fast tier) and an OPC UA client (slow tier). "
            "Install the [logix] extra and implement read_commands/write_sensors "
            "against the tag map in tags.py. The mock PLC runs the full loop "
            "headless without real hardware."
        )

    def read_commands(self, tier: Tier) -> dict: ...
    def write_sensors(self, tier: Tier, values: dict) -> None: ...
    def scan(self, dt: float) -> None:
        # Real PLC free-runs on its own oscillator (P1). Nothing to advance.
        return None
