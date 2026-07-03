"""Real-time metering for the plant seam -- the home for evals 3/9.

Mirrors the ``LatencyMeter`` pattern in ``bridge.py`` (which owns eval 5, the
FAST-tier round-trip). This one lives on the plant side because RTF and FPS are
properties of the *simulation* keeping up, not of the I/O seam -- and keeping it
here means ``bridge.py`` stays untouched (additive discipline).

Why these two numbers gate the sim (P1, P7):
  - RTF (real-time factor) = advanced sim-time / elapsed wall-time. The PLC
    free-runs on its own oscillator (P1); if the sim advances slower than real
    time (RTF < 1.0) it is silently lying to the controller about how fast the
    world moved. Eval 3/9 requires RTF >= 1.0.
  - FPS = physics frames per wall-second. The >= 30 FPS bound is the interactive
    visualisation target; headless soaks run far above it.

Pure-Python, no Isaac import -- unit-testable on the laptop.
"""
from __future__ import annotations

import time
from dataclasses import dataclass


@dataclass
class RTFMeter:
    """Accumulates sim-time vs wall-time. Call ``tick(dt)`` once per physics
    step; read ``summary()`` at the end of a soak.

    The wall clock starts on the *first* tick (not construction) so setup /
    stage-loading time is excluded from the ratio -- eval 3/9 measures the
    steady-state loop, not asset import.
    """
    sim_time: float = 0.0
    frames: int = 0
    _wall_start: float | None = None
    _wall_end: float | None = None

    def tick(self, dt: float) -> None:
        now = time.perf_counter()
        if self._wall_start is None:
            self._wall_start = now
        self.sim_time += dt
        self.frames += 1
        self._wall_end = now

    def summary(self) -> dict:
        if self._wall_start is None or self.frames == 0:
            return {"rtf": 0.0, "fps": 0.0, "sim_s": 0.0, "wall_s": 0.0, "n": 0}
        # measure to *now* so the final step's own duration is counted
        wall = time.perf_counter() - self._wall_start
        if wall <= 0.0:
            return {"rtf": float("inf"), "fps": float("inf"),
                    "sim_s": self.sim_time, "wall_s": 0.0, "n": self.frames}
        return {
            "rtf": self.sim_time / wall,
            "fps": self.frames / wall,
            "sim_s": self.sim_time,
            "wall_s": wall,
            "n": self.frames,
        }
