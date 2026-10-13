"""Exercise dynamic marker totals with the ROS messages used at runtime."""
from pathlib import Path
import sys
import time

import pytest

rclpy = pytest.importorskip("rclpy")
messages = pytest.importorskip("motion_capture_tracking_interfaces.msg")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "laptop"))
from hope_marker_monitor import HopeMarkerMonitor


def test_live_total_changes_and_visibility_remains_physical():
    rclpy.init()
    node = HopeMarkerMonitor()
    counts, totals, complete, texts = [], [], [], []
    node._pub_counts[0].publish = counts.append
    node._pub_expected_count.publish = totals.append
    node._pub_completeness[0].publish = complete.append
    node._pub_texts[0].publish = texts.append
    try:
        node._publish()
        assert totals[-1].data == 0 and not complete[-1].data
        msg = messages.RigidBodyMarkerArray(rigid_body_name="UCB_P1", rigid_body_id=9)
        for index in range(17):
            msg.markers.append(messages.RigidBodyMarker(member_id=index))
        node._on_markers(msg)
        node._publish()
        assert totals[-1].data == 17 and counts[-1].data == 0
        assert not complete[-1].data
        for marker in msg.markers:
            marker.has_live_sample = True
            marker.params = 0x02
        node._on_markers(msg)
        node._publish()
        assert totals[-1].data == counts[-1].data == 17
        assert complete[-1].data
        assert "17/17" in texts[-1].data
        msg.markers = msg.markers[:12]
        node._on_markers(msg)
        node._publish()
        assert totals[-1].data == counts[-1].data == 12
        assert complete[-1].data
        node._receipt_monotonic = time.monotonic() - 1
        node._publish()
        assert counts[-1].data == 0 and not complete[-1].data
    finally:
        node.destroy_node()
        rclpy.shutdown()
