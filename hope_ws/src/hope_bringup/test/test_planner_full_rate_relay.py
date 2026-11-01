"""The local Planner gets every fresh ball exposure; legacy outputs stay capped."""
import importlib.machinery
import importlib.util
from pathlib import Path
import sys

import pytest

rclpy = pytest.importorskip("rclpy")
messages = pytest.importorskip("motion_capture_tracking_interfaces.msg")
from geometry_msgs.msg import PoseStamped
SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
loader = importlib.machinery.SourceFileLoader(
    "full_rate_relay_test", str(SCRIPTS / "optitrack_mct_relay"))
spec = importlib.util.spec_from_loader(loader.name, loader)
module = importlib.util.module_from_spec(spec)
loader.exec_module(module)
vrpn_loader = importlib.machinery.SourceFileLoader(
    "orientation_contract_vrpn", str(SCRIPTS / "avatar_pro_vrpn_relay"))
vrpn_spec = importlib.util.spec_from_loader(vrpn_loader.name, vrpn_loader)
vrpn_module = importlib.util.module_from_spec(vrpn_spec)
sys.modules[vrpn_loader.name] = vrpn_module
vrpn_loader.exec_module(vrpn_module)
from vrpn_pants_contract import Selection


@pytest.fixture(params=["P1", "P2"])
def relay(request):
    rclpy.init(args=["--ros-args", "-p", "publish_tf:=false", "-p",
                    "planner_poses_topic:=/poses/planner_full", "-p",
                    "table_side:=" + request.param])
    node = module.OptitrackMctRelay(messages.NamedPoseArrayV2)
    node.full, node.limited = [], []
    node.points, node.ball_poses, node.transforms = [], [], []
    node.raw_players = []
    for pub in node._player_position_publishers.values():
        pub.publish = node.raw_players.append
    node._planner_pose_array_pub.publish = node.full.append
    node._pose_array_pub.publish = node.limited.append
    node._ball_pub.publish = node.points.append
    node._ball_pose_pub.publish = node.ball_poses.append
    node._broadcast_tf = lambda key, message: node.transforms.append((key, message))
    node._publish_tf = True
    yield node
    node.destroy_node()
    rclpy.shutdown()


@pytest.fixture
def vrpn(relay):
    node = vrpn_module.AvatarProVrpnRelay()
    node._position_scale = 1.0
    node._selection = Selection(side=relay._table_side)
    yield node
    node.destroy_node()


def frame(index, ball=True, hz=360):
    msg = messages.NamedPoseArrayV2()
    ns = 100_000_000_000 + round(index * 1e9 / hz)
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


@pytest.mark.parametrize("hz", [300, 360])
def test_full_rate_preserves_exposures_and_side_transform(relay, hz):
    for i in range(hz):
        relay._on_frame(frame(i, hz=hz))
    assert len(relay.full) == hz
    assert 149 <= len(relay.limited) <= 151
    for i, msg in enumerate(relay.full):
        assert msg.header.stamp == frame(i, hz=hz).header.stamp
        assert len(msg.poses) == 1
        expected = [1.0, -0.5, 0.3] if relay._table_side == "P1" else [1.74, -1.025, 0.3]
        p = msg.poses[0].position
        assert [p.x, p.y, p.z] == pytest.approx(expected)
    for i in range(hz, hz + 20):
        relay._on_frame(frame(i, ball=False, hz=hz))
    assert len(relay.full) == hz  # never repeat a cached ball at a new stamp


def quaternion(pose):
    q = pose.orientation
    return [q.x, q.y, q.z, q.w]


@pytest.mark.parametrize("raw_q", [
    (0., 0., 0., 0.),
    (float("nan"), 0., 0., 1.),
    (0., float("inf"), 0., 1.),
    (0., 0., 0., 0.49),
    (0., 0., 0., 1.51),
])
def test_position_only_ball_remains_available_with_invalid_orientation(relay, vrpn, raw_q):
    msg = frame(0)
    q = msg.poses[0].pose.orientation
    q.x, q.y, q.z, q.w = raw_q
    relay._on_frame(msg)
    source = PoseStamped(header=msg.header, pose=msg.poses[0].pose)
    assert relay.full[0].poses[0] == vrpn._normalize(source, position_only=True).pose
    assert len(relay.full) == 1
    assert len(relay.limited) == len(relay.points) == 1
    expected = [1., -.5, .3] if relay._table_side == "P1" else [1.74, -1.025, .3]
    for output in [relay.full[0], relay.limited[0]]:
        assert quaternion(output.poses[0]) == [0., 0., 0., 0.]
        p = output.poses[0].position
        assert [p.x, p.y, p.z] == pytest.approx(expected)
        assert output.header.stamp == msg.header.stamp
    assert relay.points[0].header.stamp == msg.header.stamp
    assert not relay.ball_poses
    assert not relay.transforms  # invalid quaternion must never enter TF
    bad = frame(1)
    bad.poses[0].pose.position.x = float("nan")
    relay._on_frame(bad)
    assert len(relay.full) == 1


@pytest.mark.parametrize("scale", [0.5, 1.0, 1.5])
def test_valid_ball_orientation_is_normalized_and_rotated_for_p2(relay, vrpn, scale):
    msg = frame(0)
    q = msg.poses[0].pose.orientation
    q.x = scale * 0.6
    q.w = scale * 0.8
    relay._on_frame(msg)
    source = PoseStamped(header=msg.header, pose=msg.poses[0].pose)
    assert quaternion(relay.full[0].poses[0]) == pytest.approx(
        quaternion(vrpn._normalize(source, position_only=True).pose))
    expected = [.6, 0., 0., .8] if relay._table_side == "P1" else [0., .6, .8, 0.]
    for output in [relay.full[0].poses[0], relay.limited[0].poses[0],
                   relay.ball_poses[0].pose, relay.transforms[0][1].pose]:
        assert quaternion(output) == pytest.approx(expected)


@pytest.mark.parametrize("hz", [150, 300])
def test_orientation_loss_and_reacquisition_preserve_position_exposures(relay, hz):
    # A fixed, nonidentity ball orientation must not become identity during
    # the gap, which would invent angular velocity when tracking resumes.
    count = hz // 5 + 1
    for i in range(count):
        msg = frame(i, hz=hz)
        q = msg.poses[0].pose.orientation
        q.x, q.w = (0.6, 0.8) if i in (0, count - 1) else (0., 0.)
        relay._on_frame(msg)
    assert len(relay.full) == count
    expected = [.6, 0., 0., .8] if relay._table_side == "P1" else [0., .6, .8, 0.]
    assert quaternion(relay.full[0].poses[0]) == pytest.approx(expected)
    assert quaternion(relay.full[-1].poses[0]) == pytest.approx(expected)
    for i, output in enumerate(relay.full[1:-1], 1):
        assert quaternion(output.poses[0]) == [0., 0., 0., 0.]
        assert output.header.stamp == frame(i, hz=hz).header.stamp
    assert len(relay.ball_poses) == len(relay.transforms) == 2


def test_player_assignment_observation_stays_global_before_p2_rotation(relay):
    msg = frame(0, ball=False)
    player = messages.NamedPose()
    player.name = 'UCB_P2'
    player.pose.position.x = 3.0
    player.pose.position.z = 1.0
    player.pose.orientation.w = 1.0
    msg.poses.append(player)
    relay._on_frame(msg)
    assert relay.raw_players[-1].header.frame_id == 'hope_table_global'
    assert relay.raw_players[-1].pose.position.x == pytest.approx(3.0)
    assert relay._latest['UCB_P2'].pose.position.x == pytest.approx(
        3.0 if relay._table_side == 'P1' else -.26)


@pytest.mark.parametrize('hz', [150, 300])
def test_player_admission_is_capped_at_30hz_without_throttling_base(relay, hz):
    base = []
    relay._pose_publishers['UCB_P1'].publish = base.append
    for i in range(hz):
        msg = frame(i, ball=False, hz=hz)
        player = messages.NamedPose(name='UCB_P1')
        player.pose.position.x = -.5
        player.pose.orientation.w = 1.
        msg.poses.append(player)
        relay._on_frame(msg)
    assert len(relay.raw_players) == 30
    assert len(base) == hz
