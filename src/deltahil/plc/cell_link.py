"""CellAdsLink -- direct-ADS exchange of the cell GVLs with TwinCAT (pyads).

Parallel to the single-robot TwinCATAdsLink but for the richer 2-robot cell: it
writes the plant's sensor arrays (nearest part/box slots, per-robot TCP +
grip_confirm, belt velocities) and reads the PLC's per-robot commands (TCP + grip)
-- all in ONE sum-write / ONE sum-read (batched). Metres in the plant <-> mm in
the PLC (LREAL). This does NOT touch the frozen single-robot tag map; the cell has
its own GVL set (docs/twincat_cell_program.md).
"""
from __future__ import annotations

from ..plant.cell_plant import K_BOXES, K_PARTS
from .twincat_plc import _require_pyads

G = "GVL_Cell"


class CellAdsLink:
    def __init__(self, ams_net_id: str, *, ams_port: int = 851):
        self._pyads = _require_pyads()
        self._plc = self._pyads.Connection(ams_net_id, ams_port)
        self._plc.open()
        self._cmd_names = [f"{G}.cA_tcp", f"{G}.cA_grip", f"{G}.cB_tcp", f"{G}.cB_grip",
                           f"{G}.enable"]
        self._has_time = True          # read the PLC clock until proven absent

    # -- plant -> PLC : one sum-write of all sensor arrays (m -> mm) ----------
    def write_sensors(self, sensors: dict) -> None:
        parts, boxes, rob = sensors["parts"], sensors["boxes"], sensors["robots"]
        d = {
            f"{G}.part_id":    [int(p[0]) for p in parts],
            f"{G}.part_x":     [p[1] * 1000.0 for p in parts],
            f"{G}.part_y":     [p[2] * 1000.0 for p in parts],
            f"{G}.part_valid": [bool(p[3]) for p in parts],
            f"{G}.box_id":     [int(b[0]) for b in boxes],
            f"{G}.box_x":      [b[1] * 1000.0 for b in boxes],
            f"{G}.box_fill":   [int(b[2]) for b in boxes],
            f"{G}.box_valid":  [bool(b[3]) for b in boxes],
            f"{G}.rA_tcp":     [c * 1000.0 for c in rob["Robot_A"]["tcp"]],
            f"{G}.rA_confirm": bool(rob["Robot_A"]["grip_confirm"]),
            f"{G}.rB_tcp":     [c * 1000.0 for c in rob["Robot_B"]["tcp"]],
            f"{G}.rB_confirm": bool(rob["Robot_B"]["grip_confirm"]),
            f"{G}.belt_v_src": sensors["belt_v_src"] * 1000.0,
            f"{G}.belt_v_box": sensors["belt_v_box"] * 1000.0,
        }
        self._plc.write_list_by_name(d)

    # -- PLC -> plant : one sum-read of commands (+ enable, + PLC clock) ------
    def read_commands(self):
        """Returns (cmds, enable, plc_time_ns). plc_time_ns is the PLC's own clock
        in ns (the sim derives dt from it); None if the PLC doesn't publish it ->
        the caller falls back to the wall clock. enable=False -> freeze."""
        names = self._cmd_names + ([f"{G}.plc_time_ns"] if self._has_time else [])
        try:
            v = self._plc.read_list_by_name(names)
        except Exception:
            if not self._has_time:
                raise
            self._has_time = False                       # clock symbol absent -> fall back
            v = self._plc.read_list_by_name(self._cmd_names)
        cmds = {
            "Robot_A": {"tcp": tuple(c / 1000.0 for c in v[f"{G}.cA_tcp"]),
                        "grip": bool(v[f"{G}.cA_grip"])},
            "Robot_B": {"tcp": tuple(c / 1000.0 for c in v[f"{G}.cB_tcp"]),
                        "grip": bool(v[f"{G}.cB_grip"])},
        }
        plc_ns = int(v[f"{G}.plc_time_ns"]) if self._has_time else None
        return cmds, bool(v[f"{G}.enable"]), plc_ns

    def close(self) -> None:
        try:
            self._plc.close()
        except Exception:
            pass


# sizes the TwinCAT GVL arrays must match (see docs/twincat_cell_program.md)
ARRAY_SIZES = {"parts": K_PARTS, "boxes": K_BOXES}
