"""Check rally argument forwarding without starting ROS or touching hardware."""

import os
from pathlib import Path
import shlex
import subprocess

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/run_rally_v10_hdu.sh"


def rally(*args, interface_env=None):
    env = os.environ.copy()
    env.pop("HOPE_MOTIVE_INTERFACE_IP", None)
    if interface_env is not None:
        env["HOPE_MOTIVE_INTERFACE_IP"] = interface_env
    return subprocess.run(
        ["bash", str(SCRIPT), "--mocap-backend", "optitrack",
         "--mocap-ip", "127.0.0.1", "--print-cmd", *args],
        env=env, text=True, capture_output=True, timeout=5,
    )


@pytest.mark.parametrize("args,env,expected", [
    ([], None, ""),
    ([], "192.168.50.2", "192.168.50.2"),
    (["--mocap-interface-ip", "192.168.50.3"], "192.168.50.2", "192.168.50.3"),
    (["--mocap-interface-ip", "0.0.0.0"], "192.168.50.2", "0.0.0.0"),
    (["--mocap-interface-ip", ""], "192.168.50.2", ""),
])
def test_forward_interface(args, env, expected):
    result = rally(*args, interface_env=env)
    assert result.returncode == 0, result.stderr
    bridge = shlex.split(result.stdout.splitlines()[1])
    assert f"interface_ip:={expected}" in bridge
    assert "hostname:=127.0.0.1" in bridge


@pytest.mark.parametrize("value", ["not-an-ip", "999.1.1.1", "::1",
                                   "239.255.42.99", "255.255.255.255"])
@pytest.mark.parametrize("from_env", [False, True])
def test_invalid_interface_rejected_before_launch(value, from_env):
    result = (rally(interface_env=value) if from_env else
              rally("--mocap-interface-ip", value))
    assert result.returncode != 0
    assert "interface_ip" in result.stderr
    assert "ros2 launch" not in result.stdout


def test_missing_interface_argument():
    result = rally("--mocap-interface-ip")
    assert result.returncode != 0
    assert "requires an IPv4 address" in result.stderr
