"""Delta HIL -- hardware-in-the-loop pick-and-place simulator (ABB Delta on Isaac
Sim, Allen-Bradley ControlLogix L8x controller), built to a fixed constitution.

Runs fully headless on the mock PLC + mock plant; the real controller and Isaac
Sim drop in behind the interfaces in ``interfaces.py``.
"""
from .constitution import PRINCIPLES, REFINEMENTS  # noqa: F401

__version__ = "0.1.0"
