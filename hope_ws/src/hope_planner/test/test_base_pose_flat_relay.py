import math

import pytest

from hope_planner.base_pose_contract import (
    SOURCE_STAMP_INPUT_HEADER,
    SOURCE_STAMP_LOCAL_RECEIPT,
    V17_REQUIRED_FLAGS,
    pose_to_base_flat,
    receipt_id_u52,
    resolve_wire_source_stamp_ns,
)


CALIBRATION_ID = receipt_id_u52("a" * 64)
WORLD_ID = receipt_id_u52("b" * 64)


def test_wire_stamp_can_preserve_a_synchronized_input_header():
    assert resolve_wire_source_stamp_ns(
        1234, 567, 9_000_000_000_000, SOURCE_STAMP_INPUT_HEADER
    ) == 1_234_000_000_567


def test_wire_stamp_uses_local_receipt_across_clock_domains():
    assert resolve_wire_source_stamp_ns(
        1234, 567, 9_000_000_000_000, SOURCE_STAMP_LOCAL_RECEIPT
    ) == 9_000_000_000_000


@pytest.mark.parametrize(
    ("sec", "nsec", "receipt_ns", "mode"),
    [
        (0, 0, 1, SOURCE_STAMP_LOCAL_RECEIPT),
        (1, 1_000_000_000, 1, SOURCE_STAMP_LOCAL_RECEIPT),
        (1, 0, 0, SOURCE_STAMP_LOCAL_RECEIPT),
        (1, 0, 1, "unknown"),
    ],
)
def test_wire_stamp_rejects_invalid_contract(sec, nsec, receipt_ns, mode):
    with pytest.raises(ValueError):
        resolve_wire_source_stamp_ns(sec, nsec, receipt_ns, mode)


def make_flat(
    position,
    marker_quat,
    offset,
    offset_quat=(1.0, 0.0, 0.0, 0.0),
    z_offset=0.0,
    previous_quat=None,
):
    return pose_to_base_flat(
        position,
        marker_quat,
        offset,
        offset_quat,
        z_offset,
        sequence=7,
        source_sec=1234,
        source_nsec=567,
        tracking_quality=1.0,
        flags=V17_REQUIRED_FLAGS,
        calibration_id=CALIBRATION_ID,
        world_frame_id=WORLD_ID,
        previous_base_quaternion_wxyz=previous_quat,
    )


def test_identity_pose_and_policy_z_offset():
    flat = make_flat(
        (1.0, 2.0, 3.0),
        (1.0, 0.0, 0.0, 0.0),
        (0.1, -0.2, 0.3),
        z_offset=0.76,
    )
    assert len(flat) == 16
    assert flat[:5] == [2.0, 1.0, 7.0, 1234.0, 567.0]
    assert flat[5:8] == pytest.approx([1.1, 1.8, 4.06])
    assert flat[8:12] == pytest.approx([1.0, 0.0, 0.0, 0.0])
    assert flat[13] == float(V17_REQUIRED_FLAGS)
    assert int(flat[14]) == CALIBRATION_ID
    assert int(flat[15]) == WORLD_ID


def test_marker_offset_rotates_with_pose_quaternion():
    half = math.sqrt(0.5)
    flat = make_flat(
        (0.0, 0.0, 0.0),
        (half, 0.0, 0.0, half),
        (1.0, 0.0, 0.0),
    )
    assert flat[5:8] == pytest.approx([0.0, 1.0, 0.0], abs=1.0e-12)


def test_marker_to_base_rotation_is_composed():
    half = math.sqrt(0.5)
    flat = make_flat(
        (0.0, 0.0, 0.0),
        (half, 0.0, 0.0, half),
        (0.0, 0.0, 0.0),
        (half, half, 0.0, 0.0),
    )
    assert flat[8:12] == pytest.approx([0.5, 0.5, 0.5, 0.5])


def test_zero_robot_quaternion_fails_closed():
    with pytest.raises(ValueError, match="norm"):
        make_flat(
            (1.0, 2.0, 3.0),
            (0.0, 0.0, 0.0, 0.0),
            (0.1, 0.2, 0.3),
        )


def test_nonfinite_position_fails_closed():
    with pytest.raises(ValueError, match="non-finite"):
        make_flat(
            (float("nan"), 0.0, 0.0),
            (1.0, 0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
        )


def test_quaternion_sign_is_continuous():
    flat = make_flat(
        (0.0, 0.0, 0.0),
        (-1.0, 0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        previous_quat=(1.0, 0.0, 0.0, 0.0),
    )
    assert flat[8:12] == [1.0, 0.0, 0.0, 0.0]


def test_missing_calibration_flags_fail_closed():
    with pytest.raises(ValueError, match="flags"):
        pose_to_base_flat(
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0, 0.0),
            0.0,
            sequence=1,
            source_sec=1,
            source_nsec=0,
            tracking_quality=1.0,
            flags=0,
            calibration_id=CALIBRATION_ID,
            world_frame_id=WORLD_ID,
        )
