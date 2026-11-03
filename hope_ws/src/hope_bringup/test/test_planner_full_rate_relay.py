"""The local Planner gets every fresh ball exposure; legacy outputs stay capped."""
import importlib.machinery
import importlib.util
from pathlib import Path
import sys

import pytest

rclpy = pytest.importorskip("rclpy")
messages = pytest.importorskip("motion_capture_tracking_interfaces.msg")
SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
loader = importlib.machinery.SourceFileLoader(
    "full_rate_relay_test", str(SCRIPTS / "optitrack_mct_relay"))
spec = importlib.util.spec_from_loader(loader.name, loader)
module = importlib.util.module_from_spec(spec)
loader.exec_module(module)


@pytest.fixture(params=["P1", "P2"])
def relay(request):
    rclpy.init(args=[
        '--ros-args', '-p', 'publish_tf:=false', '-p',
        'planner_poses_topic:=/poses/planner_full', '-p',
        'table_side:=' + request.param, '-p',
        'ball_orientation_authoritative:=true',
    ])
    node = module.OptitrackMctRelay(messages.NamedPoseArrayV2)
    node.full, node.limited, node.observations = [], [], []
    node._ball_observation_v2_pub.publish = node.observations.append
    node._planner_pose_array_pub.publish = node.full.append
    node._pose_array_pub.publish = node.limited.append
    node._ball_pub.publish = lambda message: None
    node._ball_pose_pub.publish = lambda message: None
    yield node
    node.destroy_node()
    rclpy.shutdown()


def frame(index, ball=True, rate_hz=360):
    msg = messages.NamedPoseArrayV2()
    ns = 100_000_000_000 + round(index * 1e9 / rate_hz)
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 1_000_000_000)
    msg.header.frame_id = "world"
    if ball:
        pose = messages.NamedPose()
        pose.name = "Ball"
        pose.pose.position.x = 1.0
        pose.pose.position.y = -0.5
        pose.pose.position.z = 0.3
        pose.pose.orientation.w = 1.0
        msg.poses.append(pose)
    return msg


@pytest.mark.parametrize('rate_hz', [150, 300, 360])
def test_full_rate_preserves_exposures_and_side_transform(relay, rate_hz):
    for i in range(rate_hz):
        relay._on_frame(frame(i, rate_hz=rate_hz))
    assert len(relay.full) == rate_hz
    assert len(relay.observations) == rate_hz
    assert all(o.orientation_authoritative and o.orientation_valid
               for o in relay.observations)
    assert 149 <= len(relay.limited) <= 151
    for i, msg in enumerate(relay.full):
        expected_stamp = frame(i, rate_hz=rate_hz).header.stamp
        assert msg.header.stamp == expected_stamp
        observation = relay.observations[i]
        assert observation.source_time_ns == (
            expected_stamp.sec * 1_000_000_000 + expected_stamp.nanosec)
        assert observation.position == msg.poses[0].position
        assert len(msg.poses) == 1
        expected = [1.0, -0.5, 0.3] if relay._table_side == "P1" else [1.74, -1.025, 0.3]
        p = msg.poses[0].position
        assert [p.x, p.y, p.z] == pytest.approx(expected)
    for i in range(rate_hz, rate_hz + 20):
        relay._on_frame(frame(i, ball=False, rate_hz=rate_hz))
    assert len(relay.full) == rate_hz  # never repeat a cached ball at a new stamp


def test_position_only_ball_remains_available_to_planner(relay):
    msg = frame(0)
    msg.poses[0].pose.orientation.w = 0.0
    relay._on_frame(msg)
    assert len(relay.full) == 1
    assert len(relay.observations) == 1
    observation = relay.observations[0]
    assert observation.source_time_ns == 100_000_000_000
    assert observation.position_valid
    assert observation.orientation_authoritative
    assert not observation.orientation_valid
    assert not observation.omega_valid
    assert relay.full[0].poses[0].orientation.w == pytest.approx(
        1.0 if relay._table_side == "P1" else 0.0, abs=1e-9)
    bad = frame(1)
    bad.poses[0].pose.position.x = float("nan")
    relay._on_frame(bad)
    assert len(relay.full) == 1


def test_numeric_identity_without_source_authority_is_position_only(relay):
    """Keep numeric identity unqualified without a measured-orientation contract."""
    relay._ball_orientation_authoritative = False
    relay._on_frame(frame(0))
    assert len(relay.observations) == 1
    observation = relay.observations[0]
    assert observation.position_valid
    assert not observation.orientation_authoritative
    assert not observation.orientation_valid


@pytest.mark.parametrize('depth', [1, 8])
def test_qualified_output_history_matches_bounded_ingress(depth):
    """DDS writers must retain the same qualified prefix as their input reader."""
    rclpy.init(args=[
        '--ros-args', '-p', 'publish_tf:=false', '-p',
        'planner_poses_topic:=/poses/planner_full', '-p',
        f'input_history_depth:={depth}',
    ])
    node = module.OptitrackMctRelay(messages.NamedPoseArrayV2)
    try:
        assert node._sub.qos_profile.depth == depth
        assert node._ball_observation_v2_pub.qos_profile.depth == depth
        assert node._planner_pose_array_pub.qos_profile.depth == depth
        assert node._pose_array_pub.qos_profile.depth == 1
        assert node._ball_pub.qos_profile.depth == 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
