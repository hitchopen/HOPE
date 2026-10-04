"""Validate both public launch paths using ROS launch, without running nodes."""

import importlib.util
from pathlib import Path
from unittest.mock import patch

import pytest

pytest.importorskip("launch")
pytest.importorskip("launch_ros")
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction


ROOT = Path(__file__).resolve().parents[4]
LAUNCHES = [
    ROOT / "hope_ws/src/hope_bringup/launch/optitrack_hope_bridge.launch.py",
    ROOT / "NatNet2ROS2/src/motion_capture_tracking/launch/natnet2ros2.launch.py",
]


@pytest.fixture(params=LAUNCHES, ids=["hope_bridge", "standalone"])
def module(request):
    spec = importlib.util.spec_from_file_location("natnet_launch_test", request.param)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def validate(module, value, backend="optitrack", start="true"):
    context = LaunchContext()
    context.launch_configurations.update(
        interface_ip=value, mocap_type=backend, start_mocap_node=start)
    return module.validate_mocap_interface(context)


@pytest.mark.parametrize("value", ["", "0.0.0.0", "192.168.50.2", "127.0.0.1"])
def test_automatic_and_explicit_interfaces(module, value):
    assert validate(module, value) == []


@pytest.mark.parametrize("value", ["typo", "999.1.1.1", "::1",
                                   "239.255.42.99", "255.255.255.255"])
def test_invalid_interface_stops_launch(module, value):
    with pytest.raises(RuntimeError, match="Invalid interface_ip"):
        validate(module, value)


def test_mock_does_not_need_network_configuration(module):
    assert validate(module, "unused", backend="mock") == []


def test_replay_does_not_need_network_configuration(module):
    if Path(module.__file__).name == "natnet2ros2.launch.py":
        pytest.skip("Standalone adapter has no bag-replay launch mode")
    assert validate(module, "unused", start="false") == []


def test_validation_runs_before_any_nodes(module):
    # Resolve only the installed-package path; use real ROS launch actions.
    if hasattr(module, "get_package_share_directory"):
        with patch.object(module, "get_package_share_directory", return_value="/tmp"):
            description = module.generate_launch_description()
    else:
        description = module.generate_launch_description()
    first_action = next(action for action in description.entities
                        if not isinstance(action, DeclareLaunchArgument))
    assert isinstance(first_action, OpaqueFunction)
    context = LaunchContext()
    context.launch_configurations.update(
        interface_ip="typo", mocap_type="optitrack", start_mocap_node="true")
    with pytest.raises(RuntimeError, match="Invalid interface_ip"):
        first_action.execute(context)
