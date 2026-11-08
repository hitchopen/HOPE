"""Exercise the production callback with publishers captured off the robot domain."""
import importlib.machinery
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import PoseStamped

SRC = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SRC / 'hope_bringup/scripts'))


@pytest.fixture
def relay():
    rclpy.init(args=['--ros-args', '-p', 'localization_source:=vrpn_ground_truth',
                    '-p', 'vrpn_pelvis_asset_confirmed:=true',
                    '-p', 'extrinsic_calibrated:=true',
                    '-p', 'world_frame_calibrated:=true',
                    '-p', 'calibration_sha256:=' + 'a' * 64,
                    '-p', 'world_frame_sha256:=' + 'b' * 64])
    loader = importlib.machinery.SourceFileLoader(
        'stale_drop_bringup', str(SRC / 'hope_bringup/scripts/hope_base_pose_flat_relay'))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    node = module.BasePoseFlatRelay()
    node._extrinsic_calibrated = node._world_calibrated = True
    node._calibration_id = node._world_frame_id = 123
    node._flags = module.V17_REQUIRED_FLAGS if hasattr(module, 'V17_REQUIRED_FLAGS') else 61
    node._asset_mode = False
    emitted, pelvis = [], []
    node._pub.publish = lambda msg: emitted.append(list(msg.data))
    if hasattr(node, '_pelvis_pub'):
        node._pelvis_pub.publish = pelvis.append
    clock = SimpleNamespace(ns=10_000_000_000)
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=clock.ns))

    def send(age_ns=0, x=0., qw=1., stamp_ns=None):
        msg = PoseStamped()
        msg.header.frame_id = node._expected_input_frame
        ns = clock.ns - age_ns if stamp_ns is None else stamp_ns
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 10**9)
        msg.pose.position.x = x
        msg.pose.orientation.w = qw
        node._pose_cb(msg)
        return msg

    yield node, clock, emitted, pelvis, send
    node.destroy_node()
    rclpy.shutdown()


def test_short_backlog_does_not_invalidate_or_refresh_mailbox(relay):
    node, clock, output, pelvis, send = relay
    send()
    last_stamp = node._last_source_ns
    last_guard = node._jump_guard._last
    clock.ns += 600_000_000
    send(age_ns=500_000_000)
    assert len(output) == 1 and output[0][1] == 1
    assert node._last_source_ns == last_stamp
    assert node._jump_guard._last == last_guard
    assert node._stale_dropped == 1
    send()
    assert len(output) == 2 and output[-1][1] == 1
    assert node._last_source_ns == clock.ns


def test_endless_old_frames_cannot_renew_runner_rescue_deadline(relay):
    node, clock, output, pelvis, send = relay
    send()
    accepted_at = clock.ns
    for i in range(1, 31):
        clock.ns = accepted_at + 200_000_000 + i * 100_000_000
        send(stamp_ns=accepted_at + i * 1_000_000)
    assert clock.ns - accepted_at > 1_000_000_000
    # No synthetic heartbeat or renewed receipt enters the existing MDU mailbox.
    assert len(output) == 1
    assert node._last_source_ns == accepted_at
    assert node._stale_dropped == 30


@pytest.mark.parametrize('fault', ['calibration', 'frame', 'nan', 'quaternion', 'future'])
def test_hard_invalid_samples_still_publish_immediate_invalidation(relay, fault):
    node, clock, output, pelvis, send = relay
    send()
    clock.ns += 500_000_000
    if fault == 'calibration':
        node._extrinsic_calibrated = False
        send(age_ns=200_000_000)
    elif fault == 'frame':
        msg = PoseStamped()
        msg.header.frame_id = 'wrong_frame'
        node._pose_cb(msg)
    elif fault == 'nan':
        send(age_ns=200_000_000, x=float('nan'))
    elif fault == 'quaternion':
        send(age_ns=200_000_000, qw=0.)
    else:
        send(age_ns=-20_000_000)
    assert len(output) == 2 and output[-1][1] == 0
    assert node._stale_dropped == 0


def test_pose_jump_after_backlog_is_dropped_without_renewing_last_good(relay):
    node, clock, output, pelvis, send = relay
    send()
    clock.ns += 300_000_000
    send(age_ns=200_000_000)
    accepted_stamp = node._last_source_ns
    accepted_guard = node._jump_guard._last
    send(x=10.)
    assert len(output) == 1
    assert node._jump_dropped == 1
    assert node._last_source_ns == accepted_stamp
    assert node._jump_guard._last == accepted_guard
    clock.ns += 3_333_333
    send()
    assert len(output) == 2 and output[-1][1] == 1


def test_age_boundary_and_pelvis_publication(relay):
    node, clock, output, pelvis, send = relay
    send(age_ns=100_000_000)
    assert len(output) == 1 and output[-1][1] == 1
    before_pelvis = len(pelvis)
    clock.ns += 10_000_000
    send(age_ns=100_000_001)
    assert len(output) == 1 and len(pelvis) == before_pelvis
    assert node._stale_dropped == 1
    node._log_health()  # verify format/counter compatibility


@pytest.mark.parametrize('delta_ns', [0, -1, -10_000_000])
def test_old_or_duplicate_frame_does_not_invalidate_and_new_frame_recovers(relay, delta_ns):
    node, clock, output, pelvis, send = relay
    send()
    accepted_stamp = node._last_source_ns
    guard = node._jump_guard._last
    clock.ns += 3_333_333
    send(stamp_ns=accepted_stamp + delta_ns)
    assert len(output) == 1
    assert node._reordered_dropped == 1
    assert node._last_source_ns == accepted_stamp
    assert node._jump_guard._last == guard
    send()
    assert len(output) == 2 and output[-1][1] == 1
    assert node._last_source_ns == clock.ns


def test_continuous_rejected_jumps_cannot_renew_no_progress_deadline(relay):
    node, clock, output, pelvis, send = relay
    send()
    accepted_stamp = node._last_source_ns
    for i in range(120):
        clock.ns += 10_000_000
        send(x=10.)
    assert clock.ns - accepted_stamp > 1_000_000_000
    assert len(output) == 1
    assert node._jump_dropped == 120
    assert node._last_source_ns == accepted_stamp
