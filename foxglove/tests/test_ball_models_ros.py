"""Physics admission checks on isolated localhost DDS domain 218 only."""
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

pytestmark = pytest.mark.skipif(os.environ.get("ROS_DOMAIN_ID") != "218" or
                                os.environ.get("ROS_LOCALHOST_ONLY") != "1",
                                reason="requires isolated localhost DDS domain 218")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "a3"))
from hope_ball_models import PROFILES, handle, profile, select

CONFIG = Path(__file__).resolve().parents[2] / "hope_ws/src/hope_planner_cpp/config"


@pytest.mark.parametrize("model", PROFILES)
def test_actual_ros_parameter_service_matches_all_fourteen_and_rejects_other_ball(tmp_path, model):
    params = profile(CONFIG, model)["hope_planner"]["ros__parameters"]
    code = '''import json,sys,rclpy
rclpy.init()
n=rclpy.create_node("hope_planner")
n.declare_parameters("",list(json.loads(sys.argv[1]).items()))
print("READY",flush=True)
rclpy.spin(n)
'''
    path = tmp_path / "selection.json"
    select(path, CONFIG, model)
    defaults = profile(CONFIG, "ppq002" if model == "berkeley" else "berkeley")["hope_planner"]["ros__parameters"]
    child = subprocess.Popen([sys.executable, "-c", code, json.dumps(defaults),
                              "--ros-args", "--params-file", str(path)],
                             stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    try:
        assert child.stdout.readline().strip() == "READY"
        result = handle(path, CONFIG, "hdu", {"op": "verify_ball_model", "require_running": True})
        assert result["runtime"] == "VERIFIED" and result["parameters"] == params
        select(path, CONFIG, "ppq002" if model == "berkeley" else "berkeley")
        with pytest.raises(ValueError, match="different physics"):
            handle(path, CONFIG, "hdu", {"op": "verify_ball_model", "require_running": True})
    finally:
        child.terminate()
        child.wait(timeout=5)


def test_bad_planner_physics_stops_lifecycle_before_hal_or_runner():
    from hope_lifecycle_supervisor import LifecycleBackend
    from hope_lifecycle_core import LifecycleConfig
    from unittest.mock import Mock
    backend = LifecycleBackend()
    calls = []
    backend._invoke = lambda argv, **kwargs: calls.append(argv)
    backend.exchange_assets = Mock(side_effect=ValueError("physics mismatch"))
    config = LifecycleConfig(revision=1)
    with pytest.raises(ValueError, match="physics mismatch"):
        backend.start(config, "model21800_20261016T110000Z", lambda _: None, ball_model="ppq002")
    assert any("start-planner" in argv for argv in calls)
    assert not any("start-hal" in argv or "start-runner" in argv for argv in calls)
