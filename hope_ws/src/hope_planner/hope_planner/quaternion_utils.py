"""Racket-normal-to-quaternion conversion.

The face normal defines paddle tilt (2 DOF) but not roll about the face axis.
The constrain_up option aligns the paddle handle (local Y) with world -Z.

See HOPE_7DOF_Racket_Model_based_Planner_Reference_Setup.md, Section 8.
"""

import numpy as np


def _quat_to_matrix(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def _quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def normal_to_quaternion(n_racket: np.ndarray, constrain_up: bool = False) -> np.ndarray:
    """Convert a racket face normal to an orientation quaternion [x, y, z, w]."""
    n = n_racket / np.linalg.norm(n_racket)
    ref = np.array([1.0, 0.0, 0.0])

    axis = np.cross(ref, n)
    sin_angle = np.linalg.norm(axis)
    cos_angle = np.dot(ref, n)

    if sin_angle < 1e-8:
        q = np.array([0.0, 0.0, 0.0, 1.0]) if cos_angle > 0 else np.array([0.0, 1.0, 0.0, 0.0])
    else:
        axis = axis / sin_angle
        half_angle = np.arctan2(sin_angle, cos_angle) / 2.0
        qw = np.cos(half_angle)
        qxyz = axis * np.sin(half_angle)
        q = np.array([qxyz[0], qxyz[1], qxyz[2], qw])

    if not constrain_up:
        return q

    # Constrain roll: align paddle handle (local Y) with world -Z
    R = _quat_to_matrix(q)
    current_y = R @ np.array([0.0, 1.0, 0.0])
    down = np.array([0.0, 0.0, -1.0])
    desired_y = down - np.dot(down, n) * n
    desired_y_norm = np.linalg.norm(desired_y)
    if desired_y_norm < 1e-6:
        return q
    desired_y = desired_y / desired_y_norm

    cos_roll = np.clip(np.dot(current_y, desired_y), -1.0, 1.0)
    sin_roll = np.dot(np.cross(current_y, desired_y), n)
    roll_half = np.arctan2(sin_roll, cos_roll) / 2.0

    q_roll = np.array([
        n[0] * np.sin(roll_half), n[1] * np.sin(roll_half),
        n[2] * np.sin(roll_half), np.cos(roll_half),
    ])
    return _quat_multiply(q_roll, q)
