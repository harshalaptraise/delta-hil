"""Probe which STEP->USD conversion path is available in this Isaac install.

Run on the rig (inside the isaacenv):  python scripts/check_cad.py

Reports whether the Omniverse CAD Converter extension can be enabled (the clean
native STEP->USD path). If it can't, we fall back to FreeCAD. Boots Kit headless,
so expect the usual ~30-60 s startup + log spam; the RESULT lines are at the end.
"""
from __future__ import annotations

import importlib

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import omni.kit.app  # noqa: E402  (must import after SimulationApp)

mgr = omni.kit.app.get_app().get_extension_manager()

# 1) list converter/importer-related extensions the registry knows about
try:
    summaries = mgr.get_extensions()
    names = sorted({s.get("name", "") for s in summaries})
except Exception as exc:  # API shape varies across versions
    names = []
    print(f"(could not list extensions: {type(exc).__name__}: {exc})")

related = [n for n in names if any(k in n.lower()
           for k in ("cad", "converter", "importer"))]

print("\n================ CONVERTER PROBE ================")
print("converter/importer-related extensions found:")
for n in related:
    print("   ", n)

# 2) try to enable + import the CAD converter specifically
CANDIDATES = ["omni.kit.converter.cad", "omni.kit.converter.cad_core",
              "omni.kit.converter.dgn"]
for ext in CANDIDATES:
    status = "not-in-registry"
    if any(ext == n for n in names):
        try:
            mgr.set_extension_enabled_immediate(ext, True)
            importlib.import_module(ext)
            status = "ENABLED + IMPORTABLE"
        except Exception as exc:
            status = f"present but failed: {type(exc).__name__}: {exc}"
    print(f"  {ext}: {status}")

print("================================================\n")
app.close()
