"""Laptop-verifiable contract for TwinCATLink (no Loupe bridge / pyads here)."""
import pytest

import deltahil.plc.twincat_plc as tw


def test_module_imports_without_bridge():
    assert hasattr(tw, "TwinCATLink")


def test_instantiation_without_runtime_raises_runtimeerror():
    with pytest.raises(RuntimeError, match="Loupe"):
        tw.TwinCATLink("1.2.3.4.5.6")


def test_ads_link_without_pyads_raises_runtimeerror():
    try:
        import pyads  # noqa: F401
        pytest.skip("pyads is installed; a connection test needs a live TwinCAT")
    except ImportError:
        with pytest.raises(RuntimeError, match="pyads"):
            tw.TwinCATAdsLink("1.2.3.4.5.6")
