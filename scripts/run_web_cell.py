"""Run the delta-hil cell with the WEB (Three.js) renderer -- GPU-free, no Isaac.

The same cell_plant + cell_controller (or live TwinCAT over ADS) as the Isaac path;
only the renderer is a browser viewer streamed over WebSocket.

    python scripts/run_web_cell.py                          # mock controller, http://127.0.0.1:8080
    python scripts/run_web_cell.py --port 9000
    python scripts/run_web_cell.py --plc 5.1.204.123.1.1    # live TwinCAT over ADS
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from deltahil.render.web.server import serve  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="delta-hil cell web viewer")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--plc", default=None, metavar="AMS_NET_ID",
                    help="drive the cell with a live TwinCAT PLC over ADS (else mock)")
    ap.add_argument("--realbot", action="store_true",
                    help="render the real ABB IRB 360 CAD (loads ~3 MB glTF); else a light delta")
    ap.add_argument("--plant", choices=["kinematic", "mujoco"], default="kinematic",
                    help="physics backend: kinematic (default) or mujoco (real contact dynamics)")
    ap.add_argument("--native", action="store_true",
                    help="also open MuJoCo's own viewer window (--plant mujoco only)")
    args = ap.parse_args()
    serve(args.host, args.port, args.plc, "real" if args.realbot else "stylized",
          args.plant, args.native)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
