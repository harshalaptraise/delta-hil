"""Batch-convert the IRB 360 STEP parts to USD via Omniverse's asset converter.

Run on the rig (inside isaacenv):  python scripts/step_to_usd.py

Uses omni.kit.asset_converter, which routes .step/.stp through the bundled HOOPS
CAD backend. Reads every *.STEP under assets/IRB360.../ and writes a matching
.usd under assets/usd/. Large B-rep parts (BASE ~9 MB, MovingPlate ~5 MB) tessellate
slowly -- expect a few minutes total. Result lines print before Kit shuts down.
"""
from __future__ import annotations

import asyncio
import glob
import os

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import omni.kit.asset_converter as ac  # noqa: E402 (after SimulationApp)

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(REPO, "assets", "IRB360_6-1600-STD-4D_rev05_STEP_J")
OUT = os.path.join(REPO, "assets", "usd")
os.makedirs(OUT, exist_ok=True)

results = {}


async def _convert(inp: str, outp: str):
    cfg = ac.AssetConverterContext()
    cfg.ignore_materials = False
    task = ac.get_instance().create_converter_task(inp, outp, None, cfg)
    ok = await task.wait_until_finished()
    if ok:
        results[inp] = "OK"
    else:
        results[inp] = f"FAIL: {task.get_status()} {task.get_error_message()}"


jobs = []
for step in sorted(glob.glob(os.path.join(SRC, "*.STEP"))):
    base = os.path.splitext(os.path.basename(step))[0]
    jobs.append((step, os.path.join(OUT, base + ".usd")))

print(f"\n[step_to_usd] converting {len(jobs)} parts -> {OUT}")
for i, (inp, outp) in enumerate(jobs, 1):
    name = os.path.basename(inp)
    print(f"  ({i}/{len(jobs)}) {name} ...", flush=True)
    fut = asyncio.ensure_future(_convert(inp, outp))
    while not fut.done():          # pump Kit so the async converter progresses
        app.update()

print("\n================ CONVERSION RESULTS ================")
ok = 0
for inp, status in results.items():
    tag = "ok " if status == "OK" else "ERR"
    if status == "OK":
        ok += 1
    print(f"  [{tag}] {os.path.basename(inp)}: {status}")
print(f"  {ok}/{len(jobs)} converted -> {OUT}")
print("====================================================\n")

app.close()
