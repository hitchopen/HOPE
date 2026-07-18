"""Quaternion utility tests."""

import numpy as np

from hope_planner.quaternion_utils import _quat_to_matrix, normal_to_quaternion


def test_quaternion_is_normalized():
    normals = [
        np.array([1.0, 0.0, 0.0]),
        np.array([-1.0, 0.0, 0.0]),
        np.array([0.0, 1.0, 0.0]),
        np.array([1.0, 1.0, 1.0]),
        np.array([0.2, -0.7, 0.4]),
    ]
    for n in normals:
        q = normal_to_quaternion(n)
        assert np.isclose(np.linalg.norm(q), 1.0, atol=1e-9)


def test_quaternion_normalized_with_constrain_up():
    for n in [np.array([1.0, 0.2, 0.3]), np.array([0.5, -0.5, 0.7])]:
        q = normal_to_quaternion(n, constrain_up=True)
        assert np.isclose(np.linalg.norm(q), 1.0, atol=1e-6)


def test_quaternion_rotates_local_x_to_target_normal():
    normals = [
        np.array([1.0, 0.0, 0.0]),
        np.array([-1.0, 0.0, 0.0]),
        np.array([0.0, 1.0, 0.0]),
        np.array([0.2, -0.7, 0.4]),
    ]
    for n in normals:
        target = n / np.linalg.norm(n)
        q = normal_to_quaternion(n)
        rotated_x = _quat_to_matrix(q) @ np.array([1.0, 0.0, 0.0])
        assert np.allclose(rotated_x, target, atol=1e-8)


def test_constrained_quaternion_still_rotates_local_x_to_target_normal():
    n = np.array([0.5, -0.5, 0.7])
    target = n / np.linalg.norm(n)
    q = normal_to_quaternion(n, constrain_up=True)
    rotated_x = _quat_to_matrix(q) @ np.array([1.0, 0.0, 0.0])
    assert np.allclose(rotated_x, target, atol=1e-8)
