"""Pure helpers for the per-robot canonical ping-pong table frame.

Motive owns one physical ``world`` frame.  A robot at the P1 end consumes that
frame directly.  A robot at the P2 end must see the same policy convention as
the P1 robot, so every world pose is rotated 180 degrees about the table
centre.  With the HOPE surface-frame bounds ``x=[0,L]`` and ``y=[-W,0]`` this
is:

    x_local = L - x_motive
    y_local = -W - y_motive
    z_local = z_motive

The transform is proper (determinant +1), so it applies consistently to ball
positions, rigid-body positions, and rigid-body orientations.
"""

from __future__ import annotations

import hashlib
import json
import math
from typing import Sequence


TABLE_SIDE_P1 = "P1"
TABLE_SIDE_P2 = "P2"
TABLE_SIDES = (TABLE_SIDE_P1, TABLE_SIDE_P2)


def normalize_table_side(value: object) -> str:
    side = str(value).strip().upper()
    if side not in TABLE_SIDES:
        raise ValueError("table_side must be P1 or P2")
    return side


def _dimensions(length_m: object, width_m: object) -> tuple[float, float]:
    length = float(length_m)
    width = float(width_m)
    if not math.isfinite(length) or length <= 0.0:
        raise ValueError("table length must be positive and finite")
    if not math.isfinite(width) or width <= 0.0:
        raise ValueError("table width must be positive and finite")
    return length, width


def _vector(value: Sequence[float], length: int, name: str) -> tuple[float, ...]:
    result = tuple(float(component) for component in value)
    if len(result) != length or not all(math.isfinite(component) for component in result):
        raise ValueError(f"{name} must contain {length} finite values")
    return result


def transform_position_xyz(
    position_xyz: Sequence[float],
    table_side: object,
    table_length_m: object,
    table_width_m: object,
) -> tuple[float, float, float]:
    """Map a Motive-world point into this robot's canonical table frame."""

    position = _vector(position_xyz, 3, "position")
    side = normalize_table_side(table_side)
    length, width = _dimensions(table_length_m, table_width_m)
    if side == TABLE_SIDE_P1:
        return position  # type: ignore[return-value]
    return (length - position[0], -width - position[1], position[2])


def _normalize_quaternion_xyzw(
    quaternion_xyzw: Sequence[float],
) -> tuple[float, float, float, float]:
    quaternion = _vector(quaternion_xyzw, 4, "quaternion")
    norm = math.sqrt(sum(component * component for component in quaternion))
    if norm < 0.5 or norm > 1.5:
        raise ValueError("quaternion norm is outside [0.5,1.5]")
    return tuple(component / norm for component in quaternion)  # type: ignore[return-value]


def transform_quaternion_xyzw(
    quaternion_xyzw: Sequence[float], table_side: object
) -> tuple[float, float, float, float]:
    """Rotate a Motive-world orientation into the selected table frame."""

    x, y, z, w = _normalize_quaternion_xyzw(quaternion_xyzw)
    if normalize_table_side(table_side) == TABLE_SIDE_P1:
        return (x, y, z, w)
    # q_local = q_z(pi) * q_motive, with q_z(pi)=(0,0,1,0) in xyzw.
    return (-y, x, w, -z)


def transform_pose_xyzw(
    position_xyz: Sequence[float],
    quaternion_xyzw: Sequence[float],
    table_side: object,
    table_length_m: object,
    table_width_m: object,
) -> tuple[
    tuple[float, float, float], tuple[float, float, float, float]
]:
    return (
        transform_position_xyz(
            position_xyz, table_side, table_length_m, table_width_m
        ),
        transform_quaternion_xyzw(quaternion_xyzw, table_side),
    )


def runtime_world_frame_sha256(
    motive_world_sha256: object,
    table_side: object,
    table_length_m: object,
    table_width_m: object,
) -> str:
    """Content-address the actual world contract consumed by one robot.

    P1 preserves the existing Motive-world receipt ID.  P2 derives a new ID
    from that receipt and the exact, versioned table-centre transform.
    """

    source = str(motive_world_sha256).strip().lower()
    if len(source) != 64 or any(character not in "0123456789abcdef" for character in source):
        raise ValueError("Motive world receipt must be exactly 64 hexadecimal characters")
    side = normalize_table_side(table_side)
    length, width = _dimensions(table_length_m, table_width_m)
    if side == TABLE_SIDE_P1:
        return source
    document = {
        "schema": "hope.table_side_transform.v1",
        "motive_world_sha256": source,
        "table_side": side,
        "table_length_m": length,
        "table_width_m": width,
        "operation": "rotate_z_pi_about_table_center",
    }
    encoded = json.dumps(
        document, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("ascii")
    return hashlib.sha256(encoded).hexdigest()
