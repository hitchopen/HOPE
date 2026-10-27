"""Computer role echoes are independent of UI presence and never timer-replayed."""
from pathlib import Path
import sys
from types import SimpleNamespace
import pytest
pytest.importorskip('rclpy')
SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
from operator_heartbeat import OperatorHeartbeat, session_fingerprint


class Node:
    def __init__(self):
        self.messages = []
    def create_publisher(self, *args):
        return SimpleNamespace(publish=self.messages.append)
    def create_subscription(self, *args):
        return args


def test_only_matching_session_challenges_produce_computer_role_echo():
    node = Node()
    heartbeat = OperatorHeartbeat(node, '')
    challenge = [1.,42.,float(session_fingerprint('test')),1.]
    heartbeat._challenge_cb(SimpleNamespace(data=challenge))
    assert not node.messages
    heartbeat = OperatorHeartbeat(node, 'test')
    heartbeat._challenge_cb(SimpleNamespace(data=challenge))
    assert list(node.messages[-1].data) == challenge + [2.]
    for values in [challenge, challenge[:3] + [0.], [1.,43.,0.,2.], [1.,42.,float('nan'),2.]]:
        heartbeat._challenge_cb(SimpleNamespace(data=values))
    assert len(node.messages) == 1
    challenge[-1] = 2.
    heartbeat._challenge_cb(SimpleNamespace(data=challenge))
    assert list(node.messages[-1].data) == challenge + [2.]
    heartbeat = OperatorHeartbeat(node, 'next')
    heartbeat._challenge_cb(SimpleNamespace(data=challenge))
    assert len(node.messages) == 2


def test_session_fingerprint_matches_native_control_contract():
    sys.path.insert(0, str(SCRIPTS.parents[3] / 'foxglove/a3'))
    from hope_runner_control_core import runner_session_fingerprint
    for session in ('test', 'model21800_20261027T123456Z', '机器人'):
        assert session_fingerprint(session) == runner_session_fingerprint(session)
