"""Physical end checks use raw player poses, never ball or side-rotated poses."""
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'a3'))
from hope_player_positions import PlayerPositions, PLAYER_POSITION_FRAME


def sample(tracker, name, x, stamp=1, now=0.0, frame=PLAYER_POSITION_FRAME):
    tracker.update(name, (x, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0), stamp, frame, now)


def test_strict_boundaries_and_either_player_can_establish_presence():
    tracker = PlayerPositions()
    assert not tracker.status(0.0)['P1'] and not tracker.status(0.0)['P2']
    for x in (0.0, 1.37, 2.74):
        sample(tracker, 'UCB_P1', x, stamp=int(x*100)+1)
        assert not tracker.status(0.0)['P1'] and not tracker.status(0.0)['P2']
    sample(tracker, 'UCB_P2', -0.01)
    assert tracker.status(0.0)['P1']
    sample(tracker, 'UCB_P1', 2.75, stamp=1000)
    assert tracker.status(0.0)['P2']


@pytest.mark.parametrize('hz', [150, 200, 300])
def test_stationary_live_pose_is_valid_but_duplicates_cannot_renew_it(hz):
    tracker = PlayerPositions()
    for i in range(hz):
        sample(tracker, 'UCB_P1', -0.5, stamp=i+1, now=i/hz)
        assert tracker.status(i/hz)['P1']
    sample(tracker, 'UCB_P1', -0.5, stamp=hz, now=2.0)
    assert not tracker.status(2.0)['P1']
    with pytest.raises(ValueError, match='PLAYER_POSITION_UNVERIFIED'):
        tracker.require('P1', 2.0)


def test_wrong_frame_ball_nan_and_invalid_orientation_cannot_enable_side():
    tracker = PlayerPositions()
    sample(tracker, 'Ball', -1.0)
    sample(tracker, 'UCB_P1', -1.0, frame='world')
    assert not tracker.status(0.0)['P1']
    sample(tracker, 'UCB_P1', -1.0)
    sample(tracker, 'UCB_P1', float('nan'), stamp=2)
    assert not tracker.status(0.0)['P1']
    tracker.update('UCB_P1', (-1., 0., 1.), (0., 0., 0., 0.), 3, PLAYER_POSITION_FRAME, 0.)
    assert not tracker.status(0.0)['P1']


@pytest.mark.parametrize('side,x,mode,success', [
    ('P1', None, 'OPTITRACK', False), ('P2', -0.5, 'OPTITRACK', False),
    ('P1', -0.5, 'OPTITRACK', True), ('P2', 3.0, 'OPTITRACK', True),
    ('P1', None, 'KERNEL', True),
])
def test_actual_start_checks_once_before_backend_and_offline_kernel_is_preserved(side, x, mode, success, monkeypatch):
    pytest.importorskip('rclpy')
    import time
    import hope_lifecycle_supervisor as module
    monkeypatch.setenv('HOPE_AUTO_RECORD_ON_START', '0')
    monkeypatch.setattr(module, 'release_hardware_operation_lock', Mock())
    tracker = PlayerPositions()
    if x is not None:
        sample(tracker, 'UCB_P2', x, now=time.monotonic())
    node = SimpleNamespace(
        _lock=threading.Lock(), _player_positions=tracker,
        _field=SimpleNamespace(status=lambda: {'mode': mode}, recorder=Mock(),
                               runner_selection=Mock(return_value=(mode, 'DEFAULT'))),
        _backend=Mock(), _on_event=Mock(), _wait_for_authoritative_runner=Mock(),
        _hardware_operation_lock=None, get_logger=Mock(return_value=Mock()))
    # Losing/moving the player after admission never reruns the position gate.
    node._backend.start.side_effect = lambda *a, **kw: tracker._samples.clear()
    module.HopeLifecycleSupervisor._run_start(node, SimpleNamespace(table_side=side), 'test')
    assert node._state == ('RUNNING' if success else 'STOPPED')
    assert node._backend.start.called is success
