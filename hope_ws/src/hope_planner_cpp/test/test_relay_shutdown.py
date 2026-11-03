"""ROS signal shutdown must not turn a completed relay into a failed child."""
import importlib.machinery
import importlib.util
from pathlib import Path
import sys

import pytest
import rclpy
from rclpy.node import Node

SCRIPTS = Path(__file__).resolve().parents[2] / 'hope_bringup/scripts'
sys.path.insert(0, str(SCRIPTS))
@pytest.mark.parametrize(('script', 'factory'), [
    ('hope_base_pose_flat_relay', 'BasePoseFlatRelay'),
    ('optitrack_mct_relay', 'OptitrackMctRelay'),
])
@pytest.mark.parametrize('context_closed', [True, False])
@pytest.mark.parametrize('message', [
    "Unable to convert call argument '0' to Python object",
    'failed to create wait set: the given context is not valid',
])
def test_shutdown_race_is_clean_but_live_runtime_errors_propagate(
        monkeypatch, context_closed, message, script, factory):
    loader = importlib.machinery.SourceFileLoader('shutdown_' + script, str(SCRIPTS / script))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    relay = importlib.util.module_from_spec(spec)
    loader.exec_module(relay)
    destroyed = []

    class TestNode(Node):
        def __init__(self, *args):
            super().__init__('base_relay_shutdown_regression')

        def destroy_node(self):
            destroyed.append(True)
            return super().destroy_node()

    def spin_at_shutdown(node):
        # The real signal handler can invalidate this context between a DDS
        # wait and take. Reproduce that order without a probabilistic signal.
        if context_closed:
            rclpy.shutdown()
            assert not node.context.ok()
        raise RuntimeError(message)

    monkeypatch.setattr(relay, factory, TestNode)
    monkeypatch.setattr(relay.rclpy, 'spin', spin_at_shutdown)
    if context_closed:
        relay.main(args=[])
    else:
        with pytest.raises(RuntimeError, match=message):
            relay.main(args=[])
    assert destroyed == [True]
    assert not rclpy.ok()
