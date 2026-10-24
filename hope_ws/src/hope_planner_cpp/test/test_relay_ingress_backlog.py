"""A blocked actual relay callback must retain a bounded physical-event burst."""
import importlib.machinery
import importlib.util
from pathlib import Path
import sys
import threading
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from hope_msgs.msg import BallObservationV2
from motion_capture_tracking_interfaces.msg import NamedPose, NamedPoseArrayV2

SCRIPTS = Path(__file__).resolve().parents[2] / 'hope_bringup/scripts'
sys.path.insert(0, str(SCRIPTS))
loader = importlib.machinery.SourceFileLoader('backlog_relay', str(SCRIPTS / 'optitrack_mct_relay'))
spec = importlib.util.spec_from_loader(loader.name, loader)
module = importlib.util.module_from_spec(spec)
loader.exec_module(module)


@pytest.mark.parametrize('rate', [150, 300])
@pytest.mark.parametrize('depth', [1, 8])
def test_short_callback_pause_retains_intervening_physical_rejection(rate, depth):
    entered, release = threading.Event(), threading.Event()
    consumed = []

    class PausingRelay(module.OptitrackMctRelay):
        def _on_frame(self, message):
            super()._on_frame(message)
            consumed.append(message.header.stamp.sec * 10**9 + message.header.stamp.nanosec)
            if len(consumed) == 1:
                entered.set()
                assert release.wait(5), 'test did not release the callback'

    rclpy.init(args=['--ros-args', '-p', 'publish_tf:=false', '-p', f'input_history_depth:={depth}'])
    relay = PausingRelay(NamedPoseArrayV2)
    node = rclpy.create_node('bounded_ingress_test')
    qos = QoSProfile(depth=16, reliability=ReliabilityPolicy.BEST_EFFORT)
    publisher = node.create_publisher(NamedPoseArrayV2, '/optitrack/poses', qos)
    output = []
    node.create_subscription(BallObservationV2, '/hope/ball/observation_v2', output.append, qos)
    executor = SingleThreadedExecutor()
    executor.add_node(relay)
    thread = threading.Thread(target=executor.spin)
    thread.start()

    def spin_until(predicate, timeout=5):
        until = time.monotonic() + timeout
        while not predicate() and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.002)
        assert predicate(), 'DDS discovery or receipt timeout'

    offered = []

    def publish(index):
        message = NamedPoseArrayV2()
        stamp = time.time_ns()
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 10**9)
        message.header.frame_id = 'world'
        ball = NamedPose()
        ball.name = 'Ball'
        ball.pose.position.x, ball.pose.position.y = 1., -.7
        ball.pose.position.z = -.2 if index in (1, 2) else .4
        ball.pose.orientation.w = 1.
        message.poses.append(ball)
        offered.append(stamp)
        publisher.publish(message)

    try:
        spin_until(lambda: publisher.get_subscription_count() > 0 and
                   node.count_publishers('/hope/ball/observation_v2') > 0)
        assert relay._sub.qos_profile.depth == depth
        publish(0)
        spin_until(lambda: entered.is_set() and len(output) == 1)
        for index in (1, 2, 3):
            time.sleep(1 / rate)
            publish(index)
        time.sleep(.002)  # DDS can receive while the application callback is blocked.
        release.set()
        expected = offered if depth == 8 else [offered[0], offered[-1]]
        spin_until(lambda: len(output) >= len(expected))
        assert consumed == expected
        assert [observation.source_time_ns for observation in output] == expected
        assert any(observation.position.z < 0 for observation in output) == (depth == 8)
    finally:
        release.set()
        executor.shutdown(timeout_sec=5)
        thread.join(timeout=5)
        node.destroy_node()
        relay.destroy_node()
        rclpy.shutdown()
        assert not thread.is_alive()


@pytest.mark.parametrize('depth', [0, 9])
def test_unbounded_or_invalid_ingress_history_is_rejected(depth):
    rclpy.init(args=['--ros-args', '-p', f'input_history_depth:={depth}'])
    try:
        with pytest.raises(ValueError, match='between 1 and 8'):
            module.OptitrackMctRelay(NamedPoseArrayV2)
    finally:
        rclpy.shutdown()
