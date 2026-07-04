"""Laptop-verifiable contract for TwinCATLink (no Loupe bridge / pyads here)."""
import pytest

import deltahil.plc.twincat_plc as tw


def test_module_imports_without_bridge():
    assert hasattr(tw, "TwinCATLink")


def test_instantiation_without_runtime_raises_runtimeerror():
    with pytest.raises(RuntimeError, match="Loupe"):
        tw.TwinCATLink("1.2.3.4.5.6")
