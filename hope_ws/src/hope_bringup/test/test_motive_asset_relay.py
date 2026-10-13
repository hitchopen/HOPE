"""Exercise real ROS message callbacks without hardware or network publishers."""
import importlib.machinery
import importlib.util
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

import pytest

rclpy = pytest.importorskip("rclpy")
from geometry_msgs.msg import PoseStamped
messages = pytest.importorskip("motion_capture_tracking_interfaces.msg")
RigidBodyMarker = messages.RigidBodyMarker
RigidBodyMarkerArray = messages.RigidBodyMarkerArray

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
from motive_asset_contract import load_asset_points

loader = importlib.machinery.SourceFileLoader("motive_test_relay", str(SCRIPTS / "hope_base_pose_flat_relay"))
spec = importlib.util.spec_from_loader(loader.name, loader)
module = importlib.util.module_from_spec(spec)
loader.exec_module(module)


@pytest.fixture
def relay(tmp_path, request, monkeypatch):
    old = tmp_path / "old.json"
    old.write_text('{"approved": true, "old": "must not be used or replaced"}')
    source = getattr(request, "param", "motive_asset")
    if source == "motive_live":
        import motive_asset_contract
        def forbid_file_load(*args):
            pytest.fail("live relay must not load the pinned Motive asset")
        monkeypatch.setattr(motive_asset_contract, "load_asset_points", forbid_file_load)
    rclpy.init(args=["--ros-args", "-p", "localization_source:=" + source,
                    "-p", "world_frame_calibrated:=true", "-p", "world_frame_sha256:=" + "a" * 64,
                    "-p", "calibration_file:=" + str(old)])
    node = module.BasePoseFlatRelay()
    node.output = []
    node._pub.publish = node.output.append
    yield node
    assert old.read_text() == '{"approved": true, "old": "must not be used or replaced"}'
    node.destroy_node()
    rclpy.shutdown()


def observations(node, points=None):
    markers = RigidBodyMarkerArray()
    markers.header.frame_id = "world"
    markers.header.stamp = node.get_clock().now().to_msg()
    markers.rigid_body_name = "UCB_P1"
    markers.rigid_body_id = 17
    for i, xyz in enumerate(load_asset_points() if points is None else points):
        marker = RigidBodyMarker(member_id=i)
        marker.model_position.x, marker.model_position.y, marker.model_position.z = xyz
        markers.markers.append(marker)
    pose = PoseStamped()
    pose.header = markers.header
    pose.pose.position.x = 1.3
    pose.pose.position.y = -0.4
    pose.pose.position.z = 0.1
    pose.pose.orientation.w = 0.9238795325112867
    pose.pose.orientation.z = 0.3826834323650898
    return markers, pose


def test_asset_frame_is_used_once_and_stale_definition_invalidates(relay):
    markers, pose = observations(relay)
    relay._pose_cb(pose)
    assert relay.output[-1].data[1] == 0
    relay._marker_identity_cb(markers)
    relay._pose_cb(pose)
    flat = relay.output[-1].data
    assert flat[1] == 1
    assert flat[5:8] == pytest.approx([1.3, -0.4, 0.86])
    assert flat[8:12] == pytest.approx([pose.pose.orientation.w, 0, 0, pose.pose.orientation.z])
    assert flat[14] > 0
    relay._asset_seen_at = time.monotonic() - 1
    relay._pose_cb(pose)
    assert relay.output[-1].data[1] == 0


def test_changed_pivot_revokes_previous_match(relay):
    markers, pose = observations(relay)
    relay._marker_identity_cb(markers)
    assert relay._extrinsic_calibrated
    markers.markers[0].model_position.x += 0.02
    relay._marker_identity_cb(markers)
    relay._pose_cb(pose)
    assert not relay._extrinsic_calibrated
    assert relay.output[-1].data[1] == 0
    assert relay._calibration_id == 0


@pytest.mark.parametrize("relay", ["motive_live"], indirect=True)
def test_new_17_marker_file_is_accepted_without_pin_and_stale_data_is_rejected(relay):
    path = SCRIPTS.parents[3] / "agibot/mocap_sticker_shell_v3/motive_asset/A3_v3_17_stickers.motive"
    body = ET.parse(path).find("./node_assets/rigid_body")
    assert body.find('./properties/property[@name="NodeName"]').get("value") == "UCB_P1"
    points = []
    for marker in body.find("constraints"):
        x, y, z = map(float, marker.findtext("offset").split(","))
        points.append((x, -z, y))
    assert len(points) == 17
    markers, pose = observations(relay, points)
    relay._pose_cb(pose)
    assert relay.output[-1].data[1] == 0
    relay._marker_identity_cb(markers)
    relay._pose_cb(pose)
    assert relay.output[-1].data[1] == 1
    assert relay.output[-1].data[5:8] == pytest.approx([1.3, -0.4, 0.86])
    assert relay._asset_marker_count == 17
    status = []
    relay._asset_text_pub.publish = status.append
    relay._publish_asset_status()
    assert "17 markers" in status[-1].data and "no file/layout pin" in status[-1].data
    # An intentional redefinition is accepted, without installing another file.
    markers.markers.pop()
    markers.markers[0].model_position.x += 0.02
    relay._marker_identity_cb(markers)
    assert relay._extrinsic_calibrated
    assert relay._asset_marker_count == 16
    relay._asset_seen_at = time.monotonic() - 1
    relay._pose_cb(pose)
    assert relay.output[-1].data[1] == 0
    relay._marker_identity_cb(markers)
    markers.markers[0].model_position.x = float("nan")
    relay._marker_identity_cb(markers)
    assert not relay._extrinsic_calibrated and relay._calibration_id == 0
