"""Execute the unit's startup selection with inert ROS/Python/DDS stand-ins."""
import configparser
import os
from pathlib import Path
import shlex
import subprocess

import pytest


def test_canonical_unit_keeps_jazzy_failure_restart_contract():
    unit = configparser.ConfigParser(interpolation=None, strict=False)
    unit.read(Path(__file__).resolve().parents[1] / "a3/hope-lifecycle-supervisor.service")
    service = unit["Service"]
    assert "source /opt/ros/jazzy/setup.bash" in service["ExecStart"]
    assert "python3 /usr/local/bin/hope_operations.py" in service["ExecStart"]
    assert service["Restart"] == "on-failure"
    # Exit 1 must retain systemd's default failure/restart classification.
    assert not service.get("SuccessExitStatus", "").strip()
    assert not service.get("RestartPreventExitStatus", "").strip()


def test_attended_bridge_exposes_draining_status_without_broadening_actions():
    import yaml
    path = Path(__file__).resolve().parents[1] / "a3/bridge_params_control.yaml"
    params = yaml.safe_load(path.read_text())["/**"]["ros__parameters"]
    assert "^/hope/operations/status$" in params["topic_whitelist"]
    assert "^/hope/operations/telemetry_fault(_count)?$" in params["topic_whitelist"]
    assert "^/hope/calibrate_v2$" in params["service_whitelist"]
    assert "^/hope/operations/status$" not in params["service_whitelist"]
    assert params["client_topic_whitelist"] == ["(?!)"]


@pytest.mark.parametrize("peers", ["", "10.42.20.1;10.42.10.12"])
def test_operations_starts_without_peers_and_preserves_explicit_peers(tmp_path, peers):
    unit = (Path(__file__).resolve().parents[1] / "a3/hope-lifecycle-supervisor.service").read_text()
    start = unit.split("ExecStart=:", 1)[1].split("\nRestart=", 1)[0]
    command = shlex.split(start.replace("\\\n", ""))[2]
    setup = tmp_path / "setup.bash"
    setup.write_text(f'export PATH={shlex.quote(str(tmp_path))}:"$PATH"\n')
    python = tmp_path / "python3"
    python.write_text('#!/bin/bash\nprintf "PYTHON %s\\n" "$@"\n')
    wrapper = tmp_path / "dds-wrapper"
    wrapper.write_text('#!/bin/bash\nprintf "DDS %s\\n" "$@"\n'
                       'while [[ "$1" != -- ]]; do shift; done\nshift\nexec "$@"\n')
    for program in (python, wrapper):
        program.chmod(0o755)
    command = command.replace("/opt/ros/jazzy/setup.bash", shlex.quote(str(setup)))
    command = command.replace("/home/agi/hope_ws/install/hope_bringup/lib/hope_bringup/with_fastdds_unicast.sh",
                              shlex.quote(str(wrapper)))
    result = subprocess.run(["bash", "-c", command], capture_output=True, text=True,
                            env={**os.environ, "ROS_STATIC_PEERS": peers, "ROS_DOMAIN_ID": "219"})
    assert result.returncode == 0, result.stderr
    assert "PYTHON /usr/local/bin/hope_operations.py" in result.stdout
    assert "PYTHON calibration_service_v2:=/a3/calibration/recompute_ucb_robot_v2\n" in result.stdout
    assert "PYTHON calibration_service_v3:=/a3/calibration/recompute_ucb_robot_v3\n" in result.stdout
    if peers:
        assert result.stdout.count("DDS --peer\n") == 2
        assert "DDS 10.42.20.1\n" in result.stdout and "DDS 10.42.10.12\n" in result.stdout
        assert "DDS 219\n" in result.stdout
    else:
        assert "DDS " not in result.stdout
        assert "ROS_STATIC_PEERS is empty" in result.stderr
        assert "Configure network.env before START" in result.stderr
