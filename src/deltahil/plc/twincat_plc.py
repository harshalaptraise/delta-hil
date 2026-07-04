"""TwinCAT (Beckhoff) soft-PLC link -- the real controller seam.

Implements the two-tier I/O contract (A) against a TwinCAT runtime via the
**Loupe Omniverse Beckhoff Bridge** (https://github.com/loupeteam/
Omniverse_Beckhoff_Bridge_Extension), which rides **ADS** (pyads) under the hood.
Refinement A already names EtherCAT/ADS as a FAST-tier transport, so this fits
the constitution with no change.

  FAST tier -- the motion-critical loop (cmd.*/sensor.*), under the eval-5 jitter
               bound. ADS request/response via the bridge is fine for bring-up;
               the <10 ms / sigma<1 ms bound is rig-verifiable only (LatencyMeter
               in bridge.py) and may need true EtherCAT process-image I/O.
  SLOW tier -- supervisory (sup.*): mode, calibration params, metrics. Exempt.

The controller free-runs (P1): ``scan()`` is a no-op -- TwinCAT advances on its
own task cycle; we only exchange tags.

Bridge API used (all on ``BeckhoffBridge.Manager``):
  add_cyclic_read_variables([symbols])  -- subscribe PLC->plant symbols
  register_data_callback(cb)            -- cb(event); event.payload['data'][GVL][member]
  write_variable(symbol, value)         -- push a plant->PLC value

TwinCAT side declares GVLs matching the tag map (see docs/twincat_gvl_spec.md):
  cmd.*  -> GVL_Cmd.*   sensor.* -> GVL_Sensor.*   sup.* -> GVL_Sup.*
  vec3   <-> ARRAY[0..2] OF LREAL (mm)

The module imports cleanly on the laptop (only ``_require_bridge`` touches the
Loupe/pyads runtime); the tag<->symbol mapping helpers below are pure and unit-
tested without any runtime.
"""
from __future__ import annotations

from ..tags import ALL_TAGS, Dir, Tier, _default

# tag namespace ("cmd"/"sensor"/"sup") -> TwinCAT GVL that carries it
GVL_FOR = {"cmd": "GVL_Cmd", "sensor": "GVL_Sensor", "sup": "GVL_Sup"}

_TAG_BY_NAME = {t.name: t for t in ALL_TAGS}


def symbol_for(tag_name: str) -> str:
    """`cmd.target_xyz` -> `GVL_Cmd.target_xyz` (the TwinCAT symbol path)."""
    ns, leaf = tag_name.split(".", 1)
    return f"{GVL_FOR[ns]}.{leaf}"


def gvl_member(tag_name: str) -> tuple[str, str]:
    """`cmd.target_xyz` -> ('GVL_Cmd', 'target_xyz'). For payload navigation."""
    return tuple(symbol_for(tag_name).split(".", 1))  # type: ignore[return-value]


def _decode(kind: str, value):
    """ADS/bridge value -> the Python encoding tags.py uses."""
    if kind == "vec3":
        v = list(value)
        return (float(v[0]), float(v[1]), float(v[2]))
    if kind == "bool":
        return bool(value)
    if kind == "int":
        return int(value)
    return float(value)  # real


def _encode(kind: str, value):
    """Python tag value -> the ADS/bridge encoding (vec3 tuple -> list of float)."""
    if kind == "vec3":
        return [float(value[0]), float(value[1]), float(value[2])]
    if kind == "bool":
        return bool(value)
    if kind == "int":
        return int(value)
    return float(value)


def read_symbols() -> list[str]:
    """Symbols the link subscribes to (all PLC->plant tags, both tiers)."""
    return [symbol_for(t.name) for t in ALL_TAGS if t.direction is Dir.PLC_TO_PLANT]


def flatten_payload(data: dict) -> dict:
    """Nested bridge payload {GVL: {member: value}} -> {tag_name: decoded value}
    for every PLC->plant tag present. Missing symbols are skipped (caller keeps
    its cached/default)."""
    out = {}
    for t in ALL_TAGS:
        if t.direction is not Dir.PLC_TO_PLANT:
            continue
        gvl, member = gvl_member(t.name)
        if gvl in data and member in data[gvl]:
            out[t.name] = _decode(t.kind, data[gvl][member])
    return out


def _require_bridge():
    try:
        from loupe.simulation.beckhoff_bridge import BeckhoffBridge
    except ImportError as exc:
        raise RuntimeError(
            "TwinCATLink needs the Loupe Omniverse Beckhoff Bridge extension "
            "(enable it in Isaac via Window > Extensions) plus pyads. The mock "
            "PLC runs the full loop headless without it -- see this module's "
            "docstring and docs/twincat_gvl_spec.md."
        ) from exc
    return BeckhoffBridge


class TwinCATLink:
    """PLCLink over the Loupe Beckhoff Bridge. Caches cyclically-read command
    tags; writes sensor tags on demand; free-runs (scan is a no-op)."""

    def __init__(self, ams_net_id: str, *, ams_port: int = 851):
        self.ams_net_id = ams_net_id
        self.ams_port = ams_port
        # seed the command cache with type defaults so read_commands works before
        # the first cyclic payload arrives.
        self._cache = {t.name: _default(t.kind)
                       for t in ALL_TAGS if t.direction is Dir.PLC_TO_PLANT}

        BeckhoffBridge = _require_bridge()  # raises on the laptop
        self._mgr = BeckhoffBridge.Manager()
        self._mgr.register_data_callback(self._on_data)
        self._mgr.add_cyclic_read_variables(read_symbols())

    # -- Loupe bridge callback ----------------------------------------------
    def _on_data(self, event) -> None:
        data = event.payload.get("data", {})
        self._cache.update(flatten_payload(data))

    # -- PLCLink interface ---------------------------------------------------
    def read_commands(self, tier: Tier) -> dict:
        return {t.name: self._cache[t.name] for t in ALL_TAGS
                if t.direction is Dir.PLC_TO_PLANT and t.tier is tier}

    def write_sensors(self, tier: Tier, values: dict) -> None:
        for name, value in values.items():
            tag = _TAG_BY_NAME.get(name)
            if tag is None or tag.direction is not Dir.PLANT_TO_PLC:
                continue
            self._mgr.write_variable(symbol_for(name), _encode(tag.kind, value))

    def scan(self, dt: float) -> None:
        # TwinCAT free-runs on its own task cycle (P1). Nothing to advance.
        return None
