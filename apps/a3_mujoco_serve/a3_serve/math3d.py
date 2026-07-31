"""Small dependency-free SO(3) helpers used by DLS IK."""

from __future__ import annotations

import math

import numpy as np


def normalize(vector: np.ndarray, *, eps: float = 1.0e-12) -> np.ndarray:
    vector = np.asarray(vector, dtype=np.float64)
    norm = float(np.linalg.norm(vector))
    if norm < eps:
        raise ValueError("cannot normalize a near-zero vector")
    return vector / norm


def skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = np.asarray(vector, dtype=np.float64)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def rotation_from_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = normalize(axis)
    k = skew(axis)
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)


def align_local_axis(
    current_rotation: np.ndarray,
    local_axis: int,
    target_world: np.ndarray,
) -> np.ndarray:
    """Rotate the current pose by the shortest arc that aligns one local axis."""

    current_rotation = np.asarray(current_rotation, dtype=np.float64).reshape(3, 3)
    source = normalize(current_rotation[:, local_axis])
    target = normalize(target_world)
    cross = np.cross(source, target)
    sine = float(np.linalg.norm(cross))
    cosine = float(np.clip(np.dot(source, target), -1.0, 1.0))
    if sine < 1.0e-12:
        if cosine > 0.0:
            return current_rotation.copy()
        basis = np.eye(3)[int(np.argmin(np.abs(source)))]
        axis = normalize(np.cross(source, basis))
        return rotation_from_axis_angle(axis, math.pi) @ current_rotation
    delta = rotation_from_axis_angle(cross / sine, math.atan2(sine, cosine))
    return delta @ current_rotation


def rotation_error(current: np.ndarray, target: np.ndarray) -> np.ndarray:
    """World-frame logarithmic rotation error taking current toward target."""

    relative = np.asarray(target) @ np.asarray(current).T
    trace = float(np.trace(relative))
    angle = math.acos(float(np.clip((trace - 1.0) * 0.5, -1.0, 1.0)))
    if angle < 1.0e-9:
        return np.zeros(3)
    vector = np.array(
        [
            relative[2, 1] - relative[1, 2],
            relative[0, 2] - relative[2, 0],
            relative[1, 0] - relative[0, 1],
        ],
        dtype=np.float64,
    )
    sine = math.sin(angle)
    if abs(sine) < 1.0e-7:
        values, vectors = np.linalg.eigh((relative + np.eye(3)) * 0.5)
        axis = vectors[:, int(np.argmax(values))]
        return normalize(axis) * angle
    return vector * (angle / (2.0 * sine))


def matrix_to_quaternion(rotation: np.ndarray) -> np.ndarray:
    """Return a normalized scalar-first quaternion [w, x, y, z]."""

    r = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    trace = float(np.trace(r))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q = np.array([0.25 * s, (r[2, 1] - r[1, 2]) / s,
                      (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s])
    else:
        i = int(np.argmax(np.diag(r)))
        if i == 0:
            s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
            q = np.array([(r[2, 1] - r[1, 2]) / s, 0.25 * s,
                          (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s])
        elif i == 1:
            s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
            q = np.array([(r[0, 2] - r[2, 0]) / s,
                          (r[0, 1] + r[1, 0]) / s, 0.25 * s,
                          (r[1, 2] + r[2, 1]) / s])
        else:
            s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
            q = np.array([(r[1, 0] - r[0, 1]) / s,
                          (r[0, 2] + r[2, 0]) / s,
                          (r[1, 2] + r[2, 1]) / s, 0.25 * s])
    return normalize(q)


def quaternion_to_matrix(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = normalize(quaternion)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def slerp(rotation_a: np.ndarray, rotation_b: np.ndarray, fraction: float) -> np.ndarray:
    qa = matrix_to_quaternion(rotation_a)
    qb = matrix_to_quaternion(rotation_b)
    dot = float(np.dot(qa, qb))
    if dot < 0.0:
        qb = -qb
        dot = -dot
    u = float(np.clip(fraction, 0.0, 1.0))
    if dot > 0.9995:
        return quaternion_to_matrix(normalize(qa + u * (qb - qa)))
    theta = math.acos(float(np.clip(dot, -1.0, 1.0)))
    q = (math.sin((1.0 - u) * theta) * qa + math.sin(u * theta) * qb) / math.sin(theta)
    return quaternion_to_matrix(q)


def minimum_jerk(fraction: float) -> float:
    u = float(np.clip(fraction, 0.0, 1.0))
    return u * u * u * (10.0 + u * (-15.0 + 6.0 * u))

