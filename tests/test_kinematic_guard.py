"""Laptop-verifiable contract for KinematicDeltaPlant (no Isaac here)."""
import pytest

import deltahil.plant.kinematic_delta as kd


def test_module_imports_without_isaac():
    assert hasattr(kd, "KinematicDeltaPlant")


def test_instantiation_without_runtime_raises_runtimeerror():
    with pytest.raises(RuntimeError, match="Isaac Sim runtime"):
        kd.KinematicDeltaPlant()
