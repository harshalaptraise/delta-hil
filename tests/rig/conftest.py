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
def delta_usd():
    """Delta source for IsaacPlant: a rigged USD asset path if $DELTAHIL_DELTA_USD
    is set, else None -> IsaacPlant builds the Delta procedurally onto its own
    live stage (no file, no reference composition)."""
    return os.environ.get("DELTAHIL_DELTA_USD")
