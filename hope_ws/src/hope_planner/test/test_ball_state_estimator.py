"""Ball state estimator unit tests."""

import numpy as np

from hope_planner.ball_state_estimator import BallStateEstimator
from hope_planner.constants import PlannerConfig


def _dt():
    return 1.0 / 300.0


def test_constant_velocity_returns_correct_velocity():
    cfg = PlannerConfig()
    est = BallStateEstimator(cfg)
    v_true = np.array([1.0, -0.5, 0.0])
    p0 = np.array([0.2, -0.3, 0.5])  # z constant > bounce tol -> no false bounce
    dt = _dt()
    for i in range(20):
        est.push(i * dt, p0 + v_true * (i * dt))
    assert est.ready
    _, v_est, _ = est.estimate()
    assert np.allclose(v_est, v_true, atol=1e-3)


def test_parabolic_z_returns_correct_vertical_velocity():
    cfg = PlannerConfig()
    est = BallStateEstimator(cfg)
    dt = _dt()
    z0, vz0, g = 1.0, 0.0, -9.81
    for i in range(20):
        t = i * dt
        p = np.array([0.0, 0.0, z0 + vz0 * t + 0.5 * g * t * t])
        est.push(t, p)
    _, v_est, t_ref = est.estimate()
    expected_vz = vz0 + g * t_ref  # derivative at the latest (reference) sample
    assert abs(v_est[2] - expected_vz) < 1e-3


def test_bounce_pattern_clears_buffer():
    cfg = PlannerConfig()
    est = BallStateEstimator(cfg)
    dt = _dt()
    zs = [0.1, 0.1, 0.1, 0.1, 0.1, 0.002, 0.1]  # descend -> contact -> rise
    for i, z in enumerate(zs):
        est.push(i * dt, np.array([0.0, 0.0, z]))
    assert est.bounce_detected
    # reset() runs before the rising sample is appended -> only that sample remains
    assert len(est.t_buffer) == 1
    assert not est.ready


def test_center_geometry_bounce_clears_buffer():
    """A centre-tracked ball's minimum height at contact is the ball radius, which
    the point-ball dip condition never sees. A physically consistent 300 Hz
    centre-height bounce (descent to z = radius, restitution rebound) must clear
    the buffer via the local-minimum detector."""
    cfg = PlannerConfig()
    est = BallStateEstimator(cfg)
    dt = 1.0 / 300.0
    radius, vz_in, e_n = 0.02, 3.0, 0.92
    zs_down = [radius + vz_in * dt * k for k in (4, 3, 2, 1)]      # 0.06 -> 0.03
    zs_up = [radius + e_n * vz_in * dt * k for k in (1, 2, 3, 4)]  # rebound
    zs = zs_down + [radius] + zs_up                                # local min at z = 0.02
    detected = False
    for i, z in enumerate(zs):
        est.push(i * dt, np.array([0.0, 0.0, z]))
        detected = detected or est.bounce_detected
    assert detected
    assert len(est.t_buffer) <= len(zs_up) + 1


def test_no_false_bounce_on_low_flat_or_monotonic_flight():
    """Low but flat / monotonically descending tracks (no local minimum) must NOT clear."""
    cfg = PlannerConfig()
    est = BallStateEstimator(cfg)
    dt = 1.0 / 300.0
    for i in range(12):  # flat inside the centre band
        est.push(i * dt, np.array([0.0, 0.0, 0.04]))
        assert not est.bounce_detected
    est2 = BallStateEstimator(cfg)
    for i, z in enumerate([0.08, 0.06, 0.045, 0.035, 0.028, 0.024]):  # still descending
        est2.push(i * dt, np.array([0.0, 0.0, z]))
        assert not est2.bounce_detected
    assert len(est.t_buffer) == 12 and len(est2.t_buffer) == 6


def test_fewer_than_six_samples_not_ready():
    cfg = PlannerConfig()
    est = BallStateEstimator(cfg)
    dt = _dt()
    for i in range(5):
        est.push(i * dt, np.array([0.0, 0.0, 0.5]))
    assert not est.ready
