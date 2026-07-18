"""Racket target planner unit tests (constant paddle restitution, no-spin)."""

import numpy as np

from hope_planner.ball_trajectory_predictor import StrikeTarget
from hope_planner.constants import BallPhysics, PlannerConfig, TableParams
from hope_planner.racket_target_planner import RacketTargetPlanner


def _planner():
    return RacketTargetPlanner(BallPhysics(), PlannerConfig(), TableParams())


def _incoming_strike():
    return StrikeTarget(
        p_ball=np.array([0.0, -0.7625, 0.3]),
        v_ball=np.array([-3.0, 0.0, -0.5]),
        t_strike=0.4, num_bounces=1, valid=True,
    )


def test_incoming_ball_produces_finite_command():
    cmd = _planner().plan(_incoming_strike())
    assert cmd.num_bounces == 1
    assert np.all(np.isfinite(cmd.v_racket))
    assert np.all(np.isfinite(cmd.p_intercept))


def test_normal_vector_is_unit_length():
    cmd = _planner().plan(_incoming_strike())
    assert np.isclose(np.linalg.norm(cmd.n_racket), 1.0, atol=1e-9)


def test_normal_vector_faces_opponent_side():
    cmd = _planner().plan(_incoming_strike())
    assert np.dot(cmd.n_racket, np.array([1.0, 0.0, 0.0])) > 0.0


def test_degenerate_sideways_and_reversed_normals_face_opponent():
    pl = _planner()
    cases = [
        (np.array([1.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0])),   # delta_v ~= 0
        (np.array([0.0, -1.0, 0.0]), np.array([0.0, 1.0, 0.0])),  # pure sideways delta_v
        (np.array([3.0, 0.0, 0.0]), np.array([-1.0, 0.0, 0.0])),  # raw delta_v points -x
    ]
    for v_in, v_out in cases:
        _, n = pl._compute_racket_velocity(v_in, v_out, pl.config.C_r)
        assert np.isclose(np.linalg.norm(n), 1.0, atol=1e-9)
        assert n[0] > 0.0


def test_outgoing_velocity_with_drag_lands_near_target():
    pl = _planner()
    p_strike = np.array([0.0, -0.7625, 0.3])
    p_land = np.array([2.055, -0.7625, 0.0])
    dt = 0.55
    v_out = pl._compute_outgoing_velocity(p_strike, p_land, dt)
    p_end, _ = pl._integrate_free_flight(p_strike, v_out, dt)
    assert np.allclose(p_end, p_land, atol=5e-4)


def test_constant_restitution_is_self_consistent():
    """The commanded racket normal speed must satisfy the restitution identity
    v_o_n - v_r_n = -C_r (v_i_n - v_r_n)."""
    pl = _planner()
    v_in = np.array([-6.0, 0.3, -1.0])
    v_out = np.array([4.0, -0.2, 2.5])
    v_r, n = pl._compute_racket_velocity(v_in, v_out, pl.config.C_r)
    v_r_n = float(np.dot(v_r, n))
    v_i_n = float(np.dot(v_in, n))
    v_o_n = float(np.dot(v_out, n))
    assert abs((v_o_n - v_r_n) + pl.config.C_r * (v_i_n - v_r_n)) < 1e-9
