from __future__ import annotations

import numpy as np

from a3_serve.math3d import align_local_axis, minimum_jerk, rotation_error
from a3_serve.physics import ServeCandidate, invert_normal_impact
from a3_serve.trajectory import build_cartesian_schedule


def test_impact_inversion_reconstructs_requested_normal_velocity_change() -> None:
    incoming = np.array([0.0, 0.0, -1.0])
    outgoing = np.array([4.0, 0.2, 1.1])
    restitution = 0.654
    racket_velocity, normal = invert_normal_impact(
        incoming, outgoing, restitution
    )
    reconstructed = incoming - (1.0 + restitution) * np.dot(
        incoming - racket_velocity, normal
    ) * normal
    assert np.allclose(reconstructed, outgoing, atol=1.0e-12)


def test_axis_alignment_preserves_a_rotation_and_hits_target() -> None:
    current = np.eye(3)
    target = np.array([0.8, -0.2, 0.4])
    result = align_local_axis(current, 1, target)
    assert np.allclose(result.T @ result, np.eye(3), atol=1.0e-12)
    assert np.allclose(result[:, 1], target / np.linalg.norm(target), atol=1.0e-12)
    assert np.linalg.det(result) > 0.999999
    assert np.linalg.norm(rotation_error(result, result)) < 1.0e-12


def test_minimum_jerk_has_fixed_endpoints() -> None:
    assert minimum_jerk(-1.0) == 0.0
    assert minimum_jerk(0.0) == 0.0
    assert minimum_jerk(1.0) == 1.0
    assert minimum_jerk(2.0) == 1.0


def test_candidate_json_converts_numpy_scalars() -> None:
    candidate = ServeCandidate(
        outgoing_velocity_world=np.ones(3),
        racket_velocity_world=np.ones(3),
        racket_normal_world=np.ones(3),
        racket_contact_position_world=np.ones(3),
        ball_contact_position_world=np.ones(3),
        first_bounce_table=np.ones(3),
        second_bounce_table=np.ones(3),
        net_clearance_m=0.2,
        legal=np.bool_(True),
        score=np.float64(1.0),
    )
    payload = candidate.json()
    assert payload["legal"] is True
    assert payload["score"] == 1.0


def test_cartesian_schedule_uses_nearest_two_sided_face_normal() -> None:
    candidate = ServeCandidate(
        outgoing_velocity_world=np.array([3.0, 0.0, 1.0]),
        racket_velocity_world=np.array([1.0, 0.0, 0.0]),
        racket_normal_world=np.array([1.0, 0.0, 0.0]),
        racket_contact_position_world=np.array([0.03, 0.0, 1.0]),
        ball_contact_position_world=np.array([0.05, 0.0, 1.0]),
        first_bounce_table=None,
        second_bounce_table=None,
        net_clearance_m=None,
        legal=True,
        score=0.0,
    )
    ready_rotation = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    schedule = build_cartesian_schedule(
        frame_count=20,
        ready_frame=5,
        stroke_start_frame=8,
        strike_frame=10,
        follow_end_frame=12,
        return_end_frame=18,
        initial_position=np.array([0.0, 0.0, 1.0]),
        initial_rotation=ready_rotation,
        ready_position=np.array([0.0, 0.0, 1.0]),
        ready_rotation=ready_rotation,
        candidate=candidate,
        normal_axis=1,
    )
    assert np.allclose(
        schedule.planned_ready_rotation_world[:, 1],
        [-1.0, 0.0, 0.0],
        atol=1.0e-12,
    )
