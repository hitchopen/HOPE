import math

import pytest

import sys
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from table_side_transform import (  # noqa: E402
    runtime_world_frame_sha256,
    transform_pose_xyzw,
    transform_position_xyz,
)


LENGTH = 2.74
WIDTH = 1.525


def test_p1_is_identity():
    position, quaternion = transform_pose_xyzw(
        (0.2, -0.4, 0.9), (0.0, 0.0, 0.0, 1.0), "P1", LENGTH, WIDTH
    )
    assert position == pytest.approx((0.2, -0.4, 0.9))
    assert quaternion == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_p2_rotates_about_table_center():
    assert transform_position_xyz((0.0, 0.0, 0.0), "P2", LENGTH, WIDTH) == pytest.approx(
        (LENGTH, -WIDTH, 0.0)
    )
    assert transform_position_xyz((LENGTH, -WIDTH, 0.0), "P2", LENGTH, WIDTH) == pytest.approx(
        (0.0, 0.0, 0.0)
    )
    center = (LENGTH / 2.0, -WIDTH / 2.0, 0.0)
    assert transform_position_xyz(center, "P2", LENGTH, WIDTH) == pytest.approx(center)


def test_p2_rotates_orientation_by_pi():
    _, quaternion = transform_pose_xyzw(
        (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), "P2", LENGTH, WIDTH
    )
    assert quaternion == pytest.approx((0.0, 0.0, 1.0, 0.0), abs=1.0e-12)

    half = math.sqrt(0.5)
    _, quaternion = transform_pose_xyzw(
        (0.0, 0.0, 0.0), (0.0, 0.0, half, half), "P2", LENGTH, WIDTH
    )
    assert quaternion == pytest.approx((0.0, 0.0, half, -half), abs=1.0e-12)


def test_runtime_world_receipt_distinguishes_table_side():
    motive = "a" * 64
    assert runtime_world_frame_sha256(motive, "P1", LENGTH, WIDTH) == motive
    p2 = runtime_world_frame_sha256(motive, "P2", LENGTH, WIDTH)
    assert len(p2) == 64
    assert p2 != motive
    assert p2 == runtime_world_frame_sha256(motive, "p2", LENGTH, WIDTH)


@pytest.mark.parametrize("side", ["", "near", "P3"])
def test_unknown_side_fails_closed(side):
    with pytest.raises(ValueError, match="P1 or P2"):
        transform_position_xyz((0.0, 0.0, 0.0), side, LENGTH, WIDTH)
