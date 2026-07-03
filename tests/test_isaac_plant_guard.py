"""Laptop-verifiable contract for IsaacPlant (no Isaac installed here).

Two guarantees that keep CI green and the seam honest:
  1. the module imports without the Isaac runtime (only instantiation needs it);
  2. the P3 grasp-confirm *logic* is correct independent of PhysX.
"""
import pytest

import deltahil.plant.isaac_plant as ip  # must import clean, no omni/isaacsim


def test_module_imports_without_isaac():
    assert hasattr(ip, "IsaacPlant")


def test_instantiation_without_runtime_raises_runtimeerror():
    with pytest.raises(RuntimeError, match="Isaac Sim runtime"):
        ip.IsaacPlant("/no/such/stage.usd")


def test_bad_grasp_mode_rejected_before_touching_isaac():
    # arg validation happens before _require_isaac(); ValueError, not RuntimeError
    with pytest.raises(ValueError, match="grasp_mode"):
        ip.IsaacPlant("/x.usd", grasp_mode="nope")


def test_confirm_grasp_requires_all_three_conjuncts():
    c = ip._confirm_grasp
    # force present + tracking + grip, sustained -> confirms on the 3rd step
    held = 0
    for step in range(3):
        confirmed, held = c(True, 9.0, True, 5.0, held, need_steps=3)
    assert confirmed and held == 3
    # missing any one conjunct never confirms and resets the counter
    assert c(False, 9.0, True, 5.0, 2, 3) == (False, 0)   # no grip
    assert c(True, 1.0, True, 5.0, 2, 3) == (False, 0)    # force below threshold
    assert c(True, 9.0, False, 5.0, 2, 3) == (False, 0)   # part not tracking TCP


def test_confirm_grasp_needs_sustained_window():
    c = ip._confirm_grasp
    confirmed, held = c(True, 9.0, True, 5.0, held_steps=0, need_steps=3)
    assert not confirmed and held == 1   # one good frame is not enough
