import importlib.util
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location(
    "live_joint_stream", Path(__file__).resolve().parents[1] / "a3/hope_live_joint_stream.py")
stream = importlib.util.module_from_spec(spec)
spec.loader.exec_module(stream)


def write_runtime(tmp_path, hdu="192.168.0.126", mdu="10.42.10.12"):
    (tmp_path / "laptop-runtime.env").write_text(
        "# Generated from HDU lifecycle config; edit through Foxglove only.\n"
        f"HOPE_LIFECYCLE_HDU_IP={hdu}\nHOPE_LIFECYCLE_MDU_IP={mdu}\n"
        "HOPE_LIFECYCLE_VRPN_IP=''\n")


def test_route_jumps_through_current_hdu_to_mdu(tmp_path):
    write_runtime(tmp_path)
    assert stream.mdu_route(tmp_path, environ={}) == ["-J", "agi@192.168.0.126", "agi@10.42.10.12"]


def test_route_follows_regenerated_site_addresses(tmp_path):
    write_runtime(tmp_path, hdu="10.0.0.3")
    assert stream.mdu_route(tmp_path, environ={})[1] == "agi@10.0.0.3"


def test_robot_user_from_environment_then_lifecycle_env(tmp_path):
    write_runtime(tmp_path)
    (tmp_path / "lifecycle.env").write_text("HOPE_ROBOT_USER=field\n")
    assert stream.mdu_route(tmp_path, environ={})[2] == "field@10.42.10.12"
    assert stream.mdu_route(tmp_path, environ={"HOPE_ROBOT_USER": "ops"})[2] == "ops@10.42.10.12"


@pytest.mark.parametrize("content", [
    "",
    "HOPE_LIFECYCLE_HDU_IP=192.168.0.126\n",
    "HOPE_LIFECYCLE_HDU_IP=host;rm -rf /\nHOPE_LIFECYCLE_MDU_IP=10.42.10.12\n",
    "HOPE_LIFECYCLE_HDU_IP=-oProxyCommand=x\nHOPE_LIFECYCLE_MDU_IP=10.42.10.12\n",
])
def test_missing_or_malformed_route_fails_closed(tmp_path, content):
    (tmp_path / "laptop-runtime.env").write_text(content)
    with pytest.raises(stream.Unavailable, match="lifecycle_route_unconfigured"):
        stream.mdu_route(tmp_path, environ={})


def test_missing_config_dir_fails_closed(tmp_path):
    with pytest.raises(stream.Unavailable):
        stream.mdu_route(tmp_path / "absent", environ={})


def test_unsafe_robot_user_rejected(tmp_path):
    write_runtime(tmp_path)
    with pytest.raises(stream.Unavailable, match="invalid_robot_user"):
        stream.mdu_route(tmp_path, environ={"HOPE_ROBOT_USER": "-oProxyCommand=x"})
