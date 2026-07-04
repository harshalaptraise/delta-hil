"""The tag<->TwinCAT-symbol mapping is pure and testable off the rig.

Verifies the symbol naming, the nested-payload flatten (with vec3->tuple), and
the write-encode -- and that a fake Loupe Manager drives read_commands/
write_sensors correctly without any Isaac/TwinCAT runtime.
"""
from deltahil.plc import twincat_plc as tw
from deltahil.tags import Tier


def test_symbol_naming():
    assert tw.symbol_for("cmd.target_xyz") == "GVL_Cmd.target_xyz"
    assert tw.symbol_for("sensor.grip_confirm") == "GVL_Sensor.grip_confirm"
    assert tw.symbol_for("sup.calib_offset_xyz") == "GVL_Sup.calib_offset_xyz"


def test_read_symbols_are_plc_to_plant_only():
    syms = set(tw.read_symbols())
    assert "GVL_Cmd.target_xyz" in syms and "GVL_Sup.mode" in syms
    # sensor.* / sup.*-from-plant are written, never subscribed
    assert "GVL_Sensor.grip_confirm" not in syms
    assert "GVL_Sup.cycle_count" not in syms


def test_flatten_payload_decodes_vec3_and_scalars():
    data = {
        "GVL_Cmd": {"target_xyz": [25.0, -15.0, -900.0], "grip": True, "tracking": False},
        "GVL_Sup": {"mode": 1, "calib_offset_xyz": [1.0, 2.0, 3.0]},
    }
    flat = tw.flatten_payload(data)
    assert flat["cmd.target_xyz"] == (25.0, -15.0, -900.0)   # list -> tuple of float
    assert flat["cmd.grip"] is True
    assert flat["sup.mode"] == 1
    assert flat["sup.calib_offset_xyz"] == (1.0, 2.0, 3.0)


def test_encode_roundtrips_kinds():
    assert tw._encode("vec3", (1, 2, 3)) == [1.0, 2.0, 3.0]
    assert tw._encode("bool", 1) is True
    assert tw._encode("int", 2.0) == 2
    assert isinstance(tw._encode("real", 3), float)


class _FakeManager:
    """Stand-in for BeckhoffBridge.Manager -- records writes, replays a payload."""
    def __init__(self):
        self.subscribed = []
        self.writes = {}
        self._cb = None

    def register_data_callback(self, cb):
        self._cb = cb

    def add_cyclic_read_variables(self, variables):
        self.subscribed = list(variables)

    def write_variable(self, symbol, value):
        self.writes[symbol] = value

    def push(self, data):
        class _Evt:
            payload = {"data": data}
        self._cb(_Evt())


def test_link_with_fake_manager(monkeypatch):
    fake = _FakeManager()

    class _FakeBeckhoff:
        Manager = staticmethod(lambda: fake)

    monkeypatch.setattr(tw, "_require_bridge", lambda: _FakeBeckhoff)
    link = tw.TwinCATLink("1.1.1.1.1.1")

    # subscribed to all PLC->plant symbols
    assert "GVL_Cmd.target_xyz" in fake.subscribed and "GVL_Sup.mode" in fake.subscribed

    # before any data: defaults
    assert link.read_commands(Tier.FAST)["cmd.grip"] is False

    # a cyclic payload updates the cache
    fake.push({"GVL_Cmd": {"target_xyz": [10.0, 0.0, -900.0], "grip": True, "tracking": False}})
    fast = link.read_commands(Tier.FAST)
    assert fast["cmd.target_xyz"] == (10.0, 0.0, -900.0) and fast["cmd.grip"] is True
    assert link.read_commands(Tier.SLOW)["sup.mode"] == 0   # untouched -> default

    # writing sensors goes to the PLC as encoded symbols
    link.write_sensors(Tier.FAST, {"sensor.grip_confirm": True,
                                   "sensor.tcp_xyz": (1.0, 2.0, 3.0)})
    assert fake.writes["GVL_Sensor.grip_confirm"] is True
    assert fake.writes["GVL_Sensor.tcp_xyz"] == [1.0, 2.0, 3.0]

    # scan is a no-op
    assert link.scan(0.004) is None
