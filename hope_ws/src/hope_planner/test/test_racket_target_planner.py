"""Stage 3 unit tests (HOPE_7DOF...Planner_Reference_Setup.md Section 5 gates)."""

import numpy as np
import pytest

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


def test_normal_incoming_ball_produces_valid_command():
    cmd = _planner().plan(_incoming_strike())
    assert cmd.valid
    assert cmd.num_bounces == 1


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


def _legacy_outgoing_velocity(pl, p_strike, p_land, delta_t):
    """Pre-optimization scalar-loop reference for numerical parity."""
    v = (p_land - p_strike) / delta_t - 0.5 * pl.physics.g * delta_t
    for _ in range(12):
        p_end, _ = pl._integrate_free_flight(p_strike, v, delta_t)
        residual = p_end - p_land
        if np.linalg.norm(residual) < 1e-5:
            break
        jac = np.zeros((3, 3))
        for axis in range(3):
            eps = 1e-4 * max(1.0, abs(v[axis]))
            v_eps = v.copy()
            v_eps[axis] += eps
            p_eps, _ = pl._integrate_free_flight(p_strike, v_eps, delta_t)
            jac[:, axis] = (p_eps - p_end) / eps
        try:
            step = np.linalg.solve(jac, residual)
        except np.linalg.LinAlgError:
            step = np.linalg.lstsq(jac, residual, rcond=None)[0]
        v = v - step
    return v


@pytest.mark.parametrize(
    "p_strike,p_land,delta_t",
    [
        (np.array([0.00, -0.76, 0.30]), np.array([2.055, -0.76, 0.00]), 0.50),
        (np.array([0.12, -0.42, 1.08]), np.array([2.20, -1.05, 0.00]), 0.35),
        (np.array([-0.08, -1.10, 0.82]), np.array([2.00, -0.35, 0.00]), 0.70),
    ],
)
def test_batched_newton_matches_legacy_scalar_reference(
    p_strike, p_land, delta_t
):
    pl = _planner()
    expected = _legacy_outgoing_velocity(pl, p_strike, p_land, delta_t)
    actual = pl._compute_outgoing_velocity(p_strike, p_land, delta_t)
    np.testing.assert_allclose(actual, expected, rtol=0.0, atol=1e-10)


def test_batched_integrator_matches_scalar_integrator():
    pl = _planner()
    p0 = np.array([0.05, -0.7, 0.9])
    velocities = np.array([
        [3.2, 0.0, 2.1],
        [4.1, -0.4, 3.0],
        [2.7, 0.6, 1.4],
        [5.0, -0.2, 4.0],
    ])
    p_batch, v_batch = pl._integrate_free_flight_batch(p0, velocities, 0.537)
    for index, velocity in enumerate(velocities):
        p_scalar, v_scalar = pl._integrate_free_flight(p0, velocity, 0.537)
        np.testing.assert_allclose(p_batch[index], p_scalar, rtol=0.0, atol=1e-12)
        np.testing.assert_allclose(v_batch[index], v_scalar, rtol=0.0, atol=1e-12)


def test_net_clearance_reported_correctly():
    pl = _planner()
    p_strike = np.array([0.0, -0.7625, 0.3])
    clears_hi, bypass_hi = pl._check_net_clearance(p_strike, np.array([5.0, 0.0, 2.0]))
    clears_lo, _ = pl._check_net_clearance(p_strike, np.array([5.0, 0.0, -2.0]))
    assert clears_hi and not bypass_hi
    assert not clears_lo


def test_invalid_strike_produces_valid_false():
    invalid = StrikeTarget(
        p_ball=np.array([0.0, -0.7625, 0.3]),
        v_ball=np.array([1.0, 0.0, 0.0]),
        t_strike=0.0, num_bounces=0, valid=False,
    )
    cmd = _planner().plan(invalid)
    assert not cmd.valid


def test_velocity_dependent_restitution_self_consistent():
    """The fixed-point e(u_n) solve must satisfy the restitution identity
    v_o_n - v_r_n = -e(u_n) * (v_i_n - v_r_n) with e = g1*exp(g2*|u_n|)."""
    pl = _planner()
    v_in = np.array([-6.0, 0.3, -1.0])
    v_out = np.array([4.0, -0.2, 2.5])
    v_r, n = pl._compute_racket_velocity(v_in, v_out, pl.config.C_r)
    v_r_n = float(np.dot(v_r, n))
    v_i_n = float(np.dot(v_in, n))
    v_o_n = float(np.dot(v_out, n))
    u_n = abs(v_i_n - v_r_n)
    e = pl.config.e_exp_g1 * np.exp(pl.config.e_exp_g2 * u_n)
    assert abs((v_o_n - v_r_n) + e * (v_i_n - v_r_n)) < 1e-3


def test_velocity_dependent_restitution_needs_faster_racket():
    """F4 physics: a faster contact has LOWER e, so the planner must command
    more racket speed than the constant-C_r model would."""
    pl = _planner()
    v_in = np.array([-7.0, 0.0, -1.0])
    v_out = np.array([5.0, 0.0, 2.0])
    v_r, n = pl._compute_racket_velocity(v_in, v_out, pl.config.C_r)
    # constant-e reference
    v_o_n = float(np.dot(v_out, n))
    v_i_n = float(np.dot(v_in, n))
    v_r_n_const = (v_o_n + pl.config.C_r * v_i_n) / (1.0 + pl.config.C_r)
    assert float(np.dot(v_r, n)) > v_r_n_const
