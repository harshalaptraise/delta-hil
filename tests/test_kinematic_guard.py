"""Laptop-verifiable contract for KinematicDeltaPlant (no Isaac here)."""
import importlib.util

import pytest

import deltahil.plant.kinematic_delta as kd

# The "no runtime" guard only fires when isaacsim can't be imported. On a rig with the
# Isaac Sim runtime installed it never raises, so that one contract is unobservable there.
_ISAAC_PRESENT = importlib.util.find_spec("isaacsim") is not None


def test_module_imports_without_isaac():
    assert hasattr(kd, "KinematicDeltaPlant")


@pytest.mark.skipif(_ISAAC_PRESENT, reason="'no Isaac runtime' guard is unobservable when isaacsim is installed (rig)")
def test_instantiation_without_runtime_raises_runtimeerror():
    with pytest.raises(RuntimeError, match="Isaac Sim runtime"):
        kd.KinematicDeltaPlant()
