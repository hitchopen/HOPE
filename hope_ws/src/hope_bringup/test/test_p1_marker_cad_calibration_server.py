"""Exercise the service's real command construction without a ROS runtime."""

import importlib.machinery
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
from p1_marker_layout import MARKER_NAMES


def test_service_selects_all_v3_stickers_and_retains_live_gates(monkeypatch, tmp_path):
    class FakeNode:
        def __init__(self, _name):
            self.parameters = {}

        def declare_parameter(self, name, value):
            self.parameters[name] = value

        def get_parameter(self, name):
            return SimpleNamespace(value=self.parameters[name])

        def create_service(self, *_args):
            return object()

        def get_logger(self):
            return SimpleNamespace(info=lambda *_args: None)

    for name in ("rclpy", "rclpy.node", "std_srvs", "std_srvs.srv"):
        monkeypatch.setitem(sys.modules, name, ModuleType(name))
    sys.modules["rclpy.node"].Node = FakeNode
    sys.modules["std_srvs.srv"].Trigger = object
    loader = importlib.machinery.SourceFileLoader(
        "calibration_server_test", str(SCRIPTS / "p1_marker_cad_calibration_server")
    )
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    server = module.P1MarkerCadCalibrationServer()
    command = server._command(tmp_path / "receipt.json")
    assert command[command.index("--marker-names") + 1] == ",".join(MARKER_NAMES)
    assert command[command.index("--minimum-frames") + 1] == "200"
    assert "--stationary-prepare" in command
    assert "--attest-installed-layout" in command
    assert "--allow-nominal-only-markers" not in command
    assert server.parameters["service_name"] == "/a3/calibration/recompute_p1"
