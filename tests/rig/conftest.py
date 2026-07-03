"""Gate the whole rig test package on the Isaac runtime.

These tests need Isaac Sim + an RTX GPU, so they are skipped on the dev laptop
(keeping ``pytest`` green -- the README's regression-net promise). Enable them on
the rig by exporting ``DELTAHIL_ISAAC=1`` (and ``DELTAHIL_DELTA_USD`` pointing at
a rigged Delta asset, or letting the fixture build one via build_delta).
"""
import os

import pytest

if os.environ.get("DELTAHIL_ISAAC") != "1":
    collect_ignore_glob = ["*"]  # do not even import Isaac-touching test modules


@pytest.fixture(scope="session")
def delta_usd(tmp_path_factory):
    """Path to a rigged Delta USD: use $DELTAHIL_DELTA_USD if set, else build one."""
    env = os.environ.get("DELTAHIL_DELTA_USD")
    if env:
        return env
    from pxr import Usd, UsdGeom
    from deltahil.plant.rig.build_delta import build_delta
    out = str(tmp_path_factory.mktemp("rig") / "delta.usd")
    stage = Usd.Stage.CreateNew(out)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    build_delta(stage)
    stage.GetRootLayer().Save()
    return out
