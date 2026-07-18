"""Smoke test: the mocap adapter turns tracker PoseStamped topics into the planner's PoseArray.

Covers the default real-mocap launch graph contract (finding: the VRPN client publishes
per-tracker ``PoseStamped`` while the planner needs a ``PoseArray`` on ``/poses``):

  * the adapter republishes the ball tracker pose as ``poses[0]``;
  * the output header stamp is the tracker message's capture stamp (passed through);
  * with two input topics the PoseArray slot order equals the input topic order and
    only the trigger topic (the ball) causes a publish.

Skipped automatically when ``rclpy`` is unavailable (needs a sourced ROS 2 environment).
"""

from __future__ import annotations

import importlib.machinery
import importlib.util
import pathlib
import time
import types

import pytest

rclpy = pytest.importorskip("rclpy")

from geometry_msgs.msg import PoseArray, PoseStamped  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import (  # noqa: E402
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

_SCRIPT = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "pose_to_posearray"

_IN_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
_OUT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def _load_adapter_module() -> types.ModuleType:
    loader = importlib.machinery.SourceFileLoader("pose_to_posearray_script", str(_SCRIPT))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


def _stamped(x: float, y: float, z: float, sec: int, nanosec: int, frame: str) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = frame
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = nanosec
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    msg.pose.orientation.w = 1.0
    return msg


@pytest.fixture()
def ros_context():
    ctx = rclpy.Context()
    rclpy.init(context=ctx, args=None)
    yield ctx
    rclpy.shutdown(context=ctx)


def _make_adapter(module, ctx, input_topics):
    return module.PoseToPoseArray(
        context=ctx,
        parameter_overrides=[
            Parameter("input_topics", value=list(input_topics)),
            Parameter("trigger_index", value=0),
            Parameter("output_topic", value="/poses"),
            Parameter("frame_id", value=""),
        ],
    )


def _pump_until(executor, predicate, publish, timeout_s: float = 5.0) -> None:
    deadline = time.monotonic() + timeout_s
    while not predicate() and time.monotonic() < deadline:
        publish()
        executor.spin_once(timeout_sec=0.05)


def test_ball_pose_reaches_planner_topic_with_capture_stamp(ros_context):
    module = _load_adapter_module()
    adapter = _make_adapter(module, ros_context, ["/vrpn_mocap/ball/pose"])
    helper = rclpy.create_node("pose_adapter_test_helper", context=ros_context)
    received: list[PoseArray] = []
    helper.create_subscription(PoseArray, "/poses", received.append, _OUT_QOS)
    pub = helper.create_publisher(PoseStamped, "/vrpn_mocap/ball/pose", _IN_QOS)

    executor = SingleThreadedExecutor(context=ros_context)
    executor.add_node(adapter)
    executor.add_node(helper)
    msg = _stamped(1.25, -0.5, 0.31, sec=42, nanosec=7, frame="world")
    try:
        _pump_until(executor, lambda: received, lambda: pub.publish(msg))
        assert received, "adapter never published a PoseArray on /poses"
        out = received[-1]
        assert len(out.poses) == 1
        assert out.poses[0].position.x == pytest.approx(1.25)
        assert out.poses[0].position.y == pytest.approx(-0.5)
        assert out.poses[0].position.z == pytest.approx(0.31)
        # Capture stamp + frame passed through, never re-stamped.
        assert out.header.stamp.sec == 42 and out.header.stamp.nanosec == 7
        assert out.header.frame_id == "world"
    finally:
        adapter.destroy_node()
        helper.destroy_node()


def test_slot_order_matches_input_topic_order(ros_context):
    module = _load_adapter_module()
    adapter = _make_adapter(
        module, ros_context, ["/vrpn_mocap/ball/pose", "/vrpn_mocap/robot/pose"]
    )
    helper = rclpy.create_node("pose_adapter_test_helper2", context=ros_context)
    received: list[PoseArray] = []
    helper.create_subscription(PoseArray, "/poses", received.append, _OUT_QOS)
    ball_pub = helper.create_publisher(PoseStamped, "/vrpn_mocap/ball/pose", _IN_QOS)
    robot_pub = helper.create_publisher(PoseStamped, "/vrpn_mocap/robot/pose", _IN_QOS)

    executor = SingleThreadedExecutor(context=ros_context)
    executor.add_node(adapter)
    executor.add_node(helper)
    ball = _stamped(2.0, -0.7, 0.4, sec=1, nanosec=0, frame="world")
    robot = _stamped(0.1, -0.9, 1.0, sec=1, nanosec=0, frame="world")
    try:
        # Robot pose arrives first (must NOT trigger a publish on its own), then the ball.
        def _publish_both():
            robot_pub.publish(robot)
            time.sleep(0.01)
            ball_pub.publish(ball)

        _pump_until(
            executor,
            lambda: any(len(m.poses) == 2 and m.poses[1].position.z > 0.5 for m in received),
            _publish_both,
        )
        good = [m for m in received if len(m.poses) == 2 and m.poses[1].position.z > 0.5]
        assert good, "adapter never published a 2-slot PoseArray with both trackers filled"
        out = good[-1]
        assert out.poses[0].position.x == pytest.approx(2.0)   # slot 0 = ball (topic 0)
        assert out.poses[1].position.x == pytest.approx(0.1)   # slot 1 = robot (topic 1)
    finally:
        adapter.destroy_node()
        helper.destroy_node()
