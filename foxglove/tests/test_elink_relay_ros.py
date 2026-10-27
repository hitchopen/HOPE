"""Isolated DDS route checks for both production relay implementations.

Run only in a network-isolated container with ROS_LOCALHOST_ONLY=1,
ROS_DOMAIN_ID=219. Set HOPE_TEST_NATIVE_RUNNER_RELAY to the freshly built native
executable to include its runtime check; no Runner/hardware endpoint is used.
"""
import importlib.util
import os
from pathlib import Path
import subprocess
import time

import pytest


pytestmark = pytest.mark.skipif(
    os.environ.get("ROS_LOCALHOST_ONLY") != "1" or os.environ.get("ROS_DOMAIN_ID") != "219",
    reason="requires isolated localhost ROS domain 219",
)


@pytest.mark.parametrize("implementation", ["python", "native"])
def test_baseline_runner_routes_preserve_payload_without_admission_dependency(implementation):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout

    native_path = os.environ.get("HOPE_TEST_NATIVE_RUNNER_RELAY", "")
    if implementation == "native" and not native_path:
        pytest.skip("set HOPE_TEST_NATIVE_RUNNER_RELAY to test native executable")
    child = None
    if implementation == "native":
        assert Path(native_path).is_absolute() and Path(native_path).is_file()
        child = subprocess.Popen([native_path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    executor = client = relay = None
    initialized = False
    try:
        rclpy.init()
        initialized = True
        executor = SingleThreadedExecutor()
        client = rclpy.create_node("isolated_elink_relay_test")
        executor.add_node(client)
        if implementation == "python":
            path = Path(__file__).parents[1] / "a3/hope_runner_transport_relay.py"
            spec = importlib.util.spec_from_file_location("isolated_elink_relay", path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            relay = module.HopeRunnerTransportRelay()
            executor.add_node(relay)
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        received = [[], []]
        publishers = [client.create_publisher(Float64MultiArray, topic, qos) for topic in (
            "/hope/runner/state_flat", "/hope/runner/control_request_hdu_flat")]
        subscriptions = [client.create_subscription(Float64MultiArray, topic,
            lambda msg, output=output: output.append(msg), qos) for topic, output in zip((
                "/hope/runner/state_hdu_flat", "/hope/runner/control_request_flat"), received)]

        def spin_until(predicate, timeout=5.0):
            deadline = time.monotonic() + timeout
            while not predicate() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.01)
                assert child is None or child.poll() is None
            assert predicate()

        spin_until(lambda: all(pub.get_subscription_count() > 0 for pub in publishers)
                   and all(client.count_publishers(sub.topic_name) > 0 for sub in subscriptions))
        payloads = ([1., 32., 23., 7.], [1., 32., 23., 7., 1., .125])
        for pub, payload in zip(publishers, payloads):
            pub.publish(Float64MultiArray(
                layout=MultiArrayLayout(data_offset=3, dim=[MultiArrayDimension(label="opaque", size=9, stride=9)]),
                data=list(payload)))
        spin_until(lambda: all(len(items) == 1 for items in received))
        for messages, payload in zip(received, payloads):
            assert list(messages[0].data) == list(payload)
            assert messages[0].layout.data_offset == 3
            assert messages[0].layout.dim[0].label == "opaque"
        end = time.monotonic() + .65
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.01)
        assert [len(items) for items in received] == [1, 1]
    finally:
        if child is not None:
            child.terminate()
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait(timeout=5)
        if executor is not None:
            executor.shutdown()
        if relay is not None:
            relay.destroy_node()
        if client is not None:
            client.destroy_node()
        if initialized:
            rclpy.shutdown()
