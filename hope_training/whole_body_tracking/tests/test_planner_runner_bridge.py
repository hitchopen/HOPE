"""Planner -> runner control-path tests (finding: the documented path must be implemented).

Two layers:

  * ``racket_command_from_msg`` field-mapping tests run everywhere (the conversion is
    duck-typed, so a stand-in message object exercises it without ROS);
  * a live ROS 2 round-trip publishes a ``hope_msgs/RacketCommand`` and asserts the
    runner-side ``RosRacketCommandSource`` receives it. This layer is skipped
    automatically when ``rclpy`` / ``hope_msgs`` are not available (e.g. outside a
    sourced ROS environment).

Run:  python tests/test_planner_runner_bridge.py   (or pytest)
"""

from __future__ import annotations

import os
import sys
import time
from types import SimpleNamespace

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_REPO = os.path.dirname(os.path.dirname(_ROOT))
_REFERENCE_DIR = os.path.join(_REPO, "a3_deploy", "a3_deploy_example", "reference")
sys.path.insert(0, _REFERENCE_DIR)

from a3_deploy_onnx_ref_pingpong.racket_command import BACKHAND, FOREHAND  # noqa: E402
from a3_deploy_onnx_ref_pingpong.ros_command_source import (  # noqa: E402
    racket_command_from_msg,
)


def _stub_msg(**overrides):
    fields = dict(
        task_id=7,
        task_revision=3,
        swing_side=BACKHAND,
        position=SimpleNamespace(x=0.5, y=-0.3, z=0.9),
        velocity=SimpleNamespace(x=1.5, y=0.8, z=0.4),
        time_to_strike=0.42,
    )
    fields.update(overrides)
    return SimpleNamespace(**fields)


def test_msg_conversion_field_mapping():
    cmd = racket_command_from_msg(_stub_msg())
    assert cmd.task_id == 7
    assert cmd.task_revision == 3
    assert cmd.swing_side == BACKHAND
    np.testing.assert_allclose(cmd.position, [0.5, -0.3, 0.9])
    np.testing.assert_allclose(cmd.velocity, [1.5, 0.8, 0.4])
    assert cmd.time_to_strike == 0.42


def test_msg_conversion_normalizes_side():
    # The dataclass normalizes any non-negative side to FOREHAND, negative to BACKHAND.
    assert racket_command_from_msg(_stub_msg(swing_side=1)).swing_side == FOREHAND
    assert racket_command_from_msg(_stub_msg(swing_side=-1)).swing_side == BACKHAND


def _ros_available() -> bool:
    try:
        import rclpy  # noqa: F401
        from hope_msgs.msg import RacketCommand  # noqa: F401
    except ImportError:
        return False
    return True


def test_ros_round_trip_planner_command_reaches_runner_source():
    """Publish on /racket/command; the runner's source must poll the same command."""
    if not _ros_available():
        import pytest

        pytest.skip("rclpy / hope_msgs not available (needs a sourced ROS 2 + hope_ws overlay)")

    import rclpy
    from hope_msgs.msg import RacketCommand as RacketCommandMsg
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    from a3_deploy_onnx_ref_pingpong.ros_command_source import RosRacketCommandSource

    topic = "/racket/command"
    source = RosRacketCommandSource(topic=topic, node_name="bridge_test_sink")

    ctx = rclpy.Context()
    rclpy.init(context=ctx, args=None)
    pub_node = rclpy.create_node("bridge_test_pub", context=ctx)
    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )
    pub = pub_node.create_publisher(RacketCommandMsg, topic, qos)

    msg = RacketCommandMsg()
    msg.task_id = 11
    msg.task_revision = 2
    msg.swing_side = RacketCommandMsg.FOREHAND
    msg.position.x, msg.position.y, msg.position.z = 0.55, -0.35, 0.85
    msg.velocity.x, msg.velocity.y, msg.velocity.z = 1.5, 1.3, 0.6
    msg.time_to_strike = 0.8

    try:
        received = None
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            pub.publish(msg)
            time.sleep(0.05)
            received = source.poll()
            if received is not None:
                break
        assert received is not None, "planner command never reached the runner source"
        assert received.task_id == 11
        assert received.task_revision == 2
        assert received.swing_side == FOREHAND
        np.testing.assert_allclose(received.position, [0.55, -0.35, 0.85])
        np.testing.assert_allclose(received.velocity, [1.5, 1.3, 0.6])
        assert abs(received.time_to_strike - 0.8) < 1e-12
    finally:
        pub_node.destroy_node()
        rclpy.shutdown(context=ctx)
        source.close()


def main() -> int:
    tests = [
        ("test_msg_conversion_field_mapping", test_msg_conversion_field_mapping),
        ("test_msg_conversion_normalizes_side", test_msg_conversion_normalizes_side),
    ]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print(f"[ok] {name}")
        except AssertionError as e:
            failed += 1
            print(f"[FAIL] {name}: {e}")
    if _ros_available():
        try:
            test_ros_round_trip_planner_command_reaches_runner_source()
            print("[ok] test_ros_round_trip_planner_command_reaches_runner_source")
        except AssertionError as e:
            failed += 1
            print(f"[FAIL] test_ros_round_trip: {e}")
    else:
        print("[skip] ROS round-trip (rclpy / hope_msgs not available)")
    print(f"\nplanner-runner bridge tests: {'FAILED' if failed else 'passed'}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
