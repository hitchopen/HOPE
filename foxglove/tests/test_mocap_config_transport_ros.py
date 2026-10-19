"""Read-only config transport checks, explicitly isolated from the robot domain."""
import importlib.util
import os
from pathlib import Path
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

pytestmark = pytest.mark.skipif(
    os.environ.get('ROS_DOMAIN_ID') != '229' or os.environ.get('ROS_LOCALHOST_ONLY') != '1',
    reason='requires isolated localhost domain 229')


def test_original_snapshot_only_and_network_change_recreates_participant(monkeypatch):
    path = Path(__file__).resolve().parents[1] / 'a3/hope_mocap_config_transport.py'
    spec = importlib.util.spec_from_file_location('config_transport', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'network_addresses', lambda: [('eth_test', '10.1.1.1')])
    rclpy.init()
    transport = module.MocapConfigTransport()
    observer = rclpy.create_node('config_transport_contract_test')
    executor = SingleThreadedExecutor()
    executor.add_node(transport)
    executor.add_node(observer)
    values = []
    publisher = observer.create_publisher(String, '/hope/lifecycle/config/mocap', 10)
    subscription = observer.create_subscription(String, '/hope/lifecycle/config/mocap_transport',
                                                lambda msg: values.append(msg.data), 10)
    def spin(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=.01)
    try:
        for _ in range(100):
            if publisher.get_subscription_count() and transport.publisher.get_subscription_count():
                break
            spin(.02)
        assert publisher.get_subscription_count() == 1
        assert transport.publisher.get_subscription_count() == 1
        assert values == []
        original = '{"revision":39,"state":"RUNNING","busy":false,"pants_profile":"v2"}'
        publisher.publish(String(data=original))
        spin(.3)
        assert values == [original]
        spin(1.1)
        assert values == [original]  # Input loss cannot be disguised as a fresh config.
        monkeypatch.setattr(module, 'network_addresses', lambda: [('eth_test', '10.1.1.2')])
        with pytest.raises(RuntimeError, match='Network addresses changed'):
            transport.check_network()
    finally:
        executor.shutdown()
        observer.destroy_node()
        transport.destroy_node()
        rclpy.shutdown()
