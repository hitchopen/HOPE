"""Preserve pre-composition calibration guards without a Node or hardware."""
from pathlib import Path
import sys
from unittest.mock import patch

import pytest
pytest.importorskip("rclpy")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "a3"))
from hope_command_proxy import CommandProxyModule
from hope_observer_core import REQUIRED_BASE_FLAGS
from hope_operations_core import OperationsState
from hope_runner_control_core import runner_session_fingerprint


@pytest.fixture
def proxy():
    module = CommandProxyModule.__new__(CommandProxyModule)
    module._shared_state = OperationsState()
    module._runner_state_condition = module._shared_state.runner.condition
    module._base_packet_condition = module._shared_state.base.condition
    module._runner_state_stale_after_s = 1.5
    module._base_pose_timeout_s = 0.
    return module


def base_packet(valid=True):
    return [2., float(valid), 5., 1., 0., 0., 0., .2, 1., 0., 0., 0., 1.,
            float(REQUIRED_BASE_FLAGS), 77., 1.]


def test_valid_base_wrong_calibration_id_reports_receipt_mismatch(proxy):
    proxy._shared_state.base.update(base_packet())
    assert proxy._shared_state.base.snapshot().error == ""
    with pytest.raises(TimeoutError, match="receipt mismatch"):
        proxy._wait_for_calibrated_base(expected_calibration_id=78, not_before_monotonic=0.)


def test_malformed_runner_keeps_fresh_pd_stand_but_never_refreshes_its_age(proxy):
    session = "model21800_20261022T010203Z"
    packet = [2., 1234., 8., 1., 1., 1., 0., 2., 1., 1., 1., 1., 0., -1., -1.,
              0., 99., 2., 1., 1., float(runner_session_fingerprint(session))]
    with patch("hope_operations_core.time.monotonic_ns", return_value=100_000_000_000):
        proxy._shared_state.runner.update(packet)
    for received_ns in (100_500_000_000, 101_000_000_000, 102_000_000_000):
        with patch("hope_operations_core.time.monotonic_ns", return_value=received_ns):
            proxy._shared_state.runner.update([2.])
    with patch("hope_command_proxy.time.monotonic", return_value=100.6):
        assert proxy._runner_pd_stand_error(session) == ""
    with patch("hope_command_proxy.time.monotonic", return_value=102.):
        assert proxy._runner_pd_stand_error(session)
    assert proxy._shared_state.runner.snapshot().received_ns == 100_000_000_000


def test_malformed_base_cannot_turn_precalibration_receipt_into_a_new_one(proxy):
    with patch("hope_operations_core.time.monotonic_ns", return_value=100_000_000_000):
        proxy._shared_state.base.update(base_packet())
    with patch("hope_operations_core.time.monotonic_ns", return_value=100_600_000_000):
        proxy._shared_state.base.update([2.])
    assert proxy._shared_state.base.snapshot().value.valid
    with patch("hope_command_proxy.time.monotonic", return_value=100.7):
        with pytest.raises(TimeoutError, match="MALFORMED"):
            proxy._wait_for_calibrated_base(expected_calibration_id=77, not_before_monotonic=100.5)
        # The previous receipt still meets a request made before it arrived.
        proxy._wait_for_calibrated_base(expected_calibration_id=77, not_before_monotonic=99.9)
    with patch("hope_command_proxy.time.monotonic", return_value=101.1):
        with pytest.raises(TimeoutError, match="MALFORMED"):
            proxy._wait_for_calibrated_base(expected_calibration_id=77, not_before_monotonic=99.9)


def test_explicit_invalid_base_replaces_last_good_for_calibration(proxy):
    proxy._shared_state.base.update(base_packet())
    proxy._shared_state.base.update(base_packet(valid=False))
    assert not proxy._shared_state.base.snapshot().value.valid
    with pytest.raises(TimeoutError, match="producer marked base packet invalid"):
        proxy._wait_for_calibrated_base(expected_calibration_id=77, not_before_monotonic=0.)
