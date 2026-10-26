"""Static production transport/config contract, independent of ROS installation."""
from pathlib import Path
import ast
import json
import sys

import yaml


ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "a3_deploy/a3_deploy_example/src/a3/a3_deploy_onnx_ref/config/a3_aimrt_config.pingpong_ros2body.yaml"


def test_aimrt_keeps_runner_routes_without_elink_admission_transport():
    channel = yaml.safe_load(CONFIG.read_text())["aimrt"]["channel"]
    backend = next(item for item in channel["backends"] if item["type"] == "ros2")
    for direction, topic in (("pub", "/hope/runner/state_flat"),
                             ("sub", "/hope/runner/control_request_flat")):
        key = direction + "_topics_options"
        route = [item for item in channel[key] if item["topic_name"] == topic]
        assert len(route) == 1 and route[0]["enable_backends"] == ["ros2"]
        transport = [item for item in backend["options"][key] if item["topic_name"] == topic]
        assert len(transport) == 1
        assert transport[0]["qos"]["reliability"] == "reliable"
        assert transport[0]["qos"]["durability"] == "volatile"
        if direction == "sub":
            assert transport[0]["use_serialized"] is False
    assert "elink_permit" not in CONFIG.read_text()
    assert "elink_challenge" not in CONFIG.read_text()


def test_python_and_native_relays_do_not_add_an_elink_dependency():
    path = ROOT / "foxglove/a3/hope_runner_transport_relay.py"
    tree = ast.parse(path.read_text())
    constants = {
        item.targets[0].id: item.value.value
        for item in tree.body
        if isinstance(item, ast.Assign) and len(item.targets) == 1
        and isinstance(item.targets[0], ast.Name) and isinstance(item.value, ast.Constant)
    }
    expected = {
        "REMOTE_STATE_TOPIC": "/hope/runner/state_flat",
        "LOCAL_STATE_TOPIC": "/hope/runner/state_hdu_flat",
        "LOCAL_CONTROL_TOPIC": "/hope/runner/control_request_hdu_flat",
        "REMOTE_CONTROL_TOPIC": "/hope/runner/control_request_flat",
    }
    native = (ROOT / "foxglove/a3/native/runner_transport.cpp").read_text()
    for name, value in expected.items():
        assert constants[name] == value
        assert native.count('"' + value + '"') == 1
    assert len(set(expected.values())) == 4
    for source in (path.read_text(), native):
        assert "elink_permit" not in source and "elink_challenge" not in source
    native_root = CONFIG.parent.parent
    for relative in ("include/robot_io/a3_aimrt_backend.hpp", "src/robot_io/a3_aimrt_backend.cpp"):
        source = (native_root / relative).read_text()
        assert "SetElinkPermitCallback" not in source and "PublishElinkChallenge" not in source


def test_telemetry_example_is_schema_valid_but_not_audited_evidence():
    sys.path.insert(0, str(ROOT / "foxglove/a3"))
    from hope_elink_telemetry import TelemetryContract
    value = json.loads((ROOT / "foxglove/a3/elink_telemetry.example.json").read_text())
    contract = TelemetryContract.from_dict(value)
    assert not contract.audited
    assert contract.hardware_profile == contract.hal_library_sha256 == ""
    assert all(channel.required for channel in contract.channels.values())
    assert all(channel.acquisition_evidence == channel.fault_semantics_evidence == ""
               for channel in contract.channels.values())
