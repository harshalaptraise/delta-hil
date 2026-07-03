"""Batch-convert the IRB 360 STEP parts to USD via the HOOPS CAD converter.

Run on the rig (inside isaacenv):  python scripts/step_to_usd.py

STEP is not handled by omni.kit.asset_converter (mesh formats only). It goes
through the CAD converter (HOOPS backend), whose API is:
    module.get_instance().create_converter_task(src, out, args)  # coroutine
This script discovers which converter module actually exposes that API on this
install, then converts every *.STEP under assets/IRB360.../ to assets/usd/.
Success is judged by the output .usd actually appearing on disk.
"""
from __future__ import annotations

import asyncio
import glob
import importlib
import os

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import omni.kit.app  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(REPO, "assets", "IRB360_6-1600-STD-4D_rev05_STEP_J")
OUT = os.path.join(REPO, "assets", "usd")
os.makedirs(OUT, exist_ok=True)

# candidate CAD converter extensions/modules (HOOPS handles STEP/IGES/JT/etc.)
CANDIDATES = [
    "omni.kit.converter.hoops",
    "omni.kit.converter.hoops_core",
    "omni.kit.converter.cad",
    "omni.kit.converter.jt",
    "omni.kit.converter.dgn",
]

mgr = omni.kit.app.get_app().get_extension_manager()
for ext in CANDIDATES:
    try:
        mgr.set_extension_enabled_immediate(ext, True)
    except Exception:
        pass


def find_converter():
    for name in CANDIDATES:
        try:
            m = importlib.import_module(name)
        except Exception:
            continue
        if not hasattr(m, "get_instance"):
            continue
        try:
            inst = m.get_instance()
        except Exception:
            continue
        if hasattr(inst, "create_converter_task"):
            return name, inst
    return None, None


mod_name, converter = find_converter()
print(f"\n[step_to_usd] CAD converter module: {mod_name}")
if converter is None:
    print("  NO converter with create_converter_task found. Importable modules:")
    for name in CANDIDATES:
        try:
            importlib.import_module(name)
            print(f"    {name}: importable")
        except Exception as exc:
            print(f"    {name}: {type(exc).__name__}")
    app.close()
    raise SystemExit(1)

results = {}


async def _convert(inp: str, outp: str):
    if os.path.exists(outp):
        os.remove(outp)
    try:
        res = await converter.create_converter_task(inp, outp, [])
    except TypeError:
        # some builds take no options arg
        res = await converter.create_converter_task(inp, outp)
    except Exception as exc:
        results[inp] = f"EXC: {type(exc).__name__}: {exc}"
        return
    results[inp] = "OK" if os.path.exists(outp) else f"NO-OUTPUT (res={res})"


jobs = []
for step in sorted(glob.glob(os.path.join(SRC, "*.STEP"))):
    base = os.path.splitext(os.path.basename(step))[0]
    jobs.append((step, os.path.join(OUT, base + ".usd")))

print(f"[step_to_usd] converting {len(jobs)} parts -> {OUT}")
for i, (inp, outp) in enumerate(jobs, 1):
    print(f"  ({i}/{len(jobs)}) {os.path.basename(inp)} ...", flush=True)
    fut = asyncio.ensure_future(_convert(inp, outp))
    while not fut.done():
        app.update()

print("\n================ CONVERSION RESULTS ================")
ok = sum(1 for v in results.values() if v == "OK")
for inp, status in results.items():
    tag = "ok " if status == "OK" else "ERR"
    print(f"  [{tag}] {os.path.basename(inp)}: {status}")
print(f"  {ok}/{len(jobs)} converted -> {OUT}")
print("====================================================\n")

app.close()
