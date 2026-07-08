"""Convert the IRB 360 STEP CAD -> decimated glTF for the hilweb --realbot viewer.

The .glb files are committed (so `--realbot` works offline out of the box), but this
regenerates them from the tracked STEP source -- e.g. to re-decimate:

    pip install cascadio
    python scripts/build_robot_glb.py

Output: src/deltahil/render/web/static/robot/<part>.glb, in METRES in the CAD assembly
frame (z-down) -- exactly what src/deltahil/plant/irb360_pose.py (and its JS port in
viewer.html) expects. Part names match irb360_pose (BASE, UA1..3, LA{i}_{CL,CU,1,2},
MovingPlate, RevoluteLink*).
"""
import glob
import os
import re
import sys

SRC = "assets/IRB360_6-1600-STD-4D_rev05_STEP_J"
OUT = "src/deltahil/render/web/static/robot"
TOL_LINEAR, TOL_ANGULAR = 0.4, 0.6      # mm / rad -- coarser = lighter glTF


def part_name(f: str) -> str:
    b = os.path.basename(f)
    b = re.sub(r"IRB360_6-1600-STD-4D_rev0[0-9]_", "", b)
    return re.sub(r"_CAD.*", "", b)


def main() -> int:
    try:
        import cascadio
    except ImportError:
        print("needs: pip install cascadio", file=sys.stderr)
        return 1
    os.makedirs(OUT, exist_ok=True)
    files = sorted(glob.glob(os.path.join(SRC, "*.STEP")))
    if not files:
        print(f"no STEP files under {SRC}", file=sys.stderr)
        return 1
    total = 0
    for f in files:
        out = os.path.join(OUT, part_name(f) + ".glb")
        cascadio.step_to_glb(f, out, TOL_LINEAR, TOL_ANGULAR)
        total += os.path.getsize(out)
    print(f"wrote {len(files)} parts -> {OUT}  ({total / 1e6:.2f} MB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
