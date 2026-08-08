"""Stage 1 unit tests (HOPE_7DOF...Planner_Reference_Setup.md Section 3 gates)."""

import numpy as np

from hope_planner.ball_state_estimator import BallStateEstimator
from hope_planner.constants import PlannerConfig


def _dt():
    return 1.0 / 360.0


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


def test_serve_linear_horizontal_fit_reduces_endpoint_noise_amplification():
    """The serve selector opts into a linear X/Y fit without changing Z.

    Use a deterministic, slowly varying position bias representative of a
    correlated marker error.  The extra quadratic horizontal degree can bend
    toward the final samples and amplify its endpoint derivative.
    """
    cfg = PlannerConfig(fit_window=31)
    legacy = BallStateEstimator(cfg)
    serve = BallStateEstimator(cfg, horizontal_poly_order=1)
    dt = 1.0 / 300.0
    vx_true = 0.20
    for i in range(31):
        t = i * dt
        correlated_bias = 0.002 * np.sin(2.0 * np.pi * i / 45.0)
        p = np.array(
            [
                0.5 + vx_true * t + correlated_bias,
                0.04,
                1.2 + 2.0 * t - 4.905 * t * t,
            ]
        )
        legacy.push(t, p)
        serve.push(t, p)

    _, legacy_v, _ = legacy.estimate()
    _, serve_v, _ = serve.estimate()
    assert abs(serve_v[0] - vx_true) < abs(legacy_v[0] - vx_true)
    # Z remains the configured quadratic ballistic fit.
    assert abs(serve_v[2] - (2.0 - 9.81 * 30 * dt)) < 1.0e-6


def test_parabolic_z_returns_correct_vertical_velocity():
    cfg = PlannerConfig()
    est = BallStateEstimator(cfg)
    dt = _dt()
    z0, vz0, g = 1.0, 0.0, -9.81
    n = 20
    for i in range(n):
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
    """Real mocap tracks the ball CENTER: its minimum height at contact is the ball RADIUS
    (0.02 m), which the legacy 5 mm dip condition can never see (audit 2026-07-07). A physically
    consistent 300 Hz center-height bounce (ballistic descent, dip to z = radius, restitution
    rebound) must clear the buffer via the center-geometry local-minimum detector."""
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
    # buffer was cleared at the local-minimum sample -> only post-bounce samples remain
    assert len(est.t_buffer) <= len(zs_up) + 1


def test_subsampled_bounce_clears_buffer_above_old_contact_band():
    """A depth-1 callback can skip the exact contact and only observe a 12 cm minimum."""
    cfg = PlannerConfig(bounce_center_z_max=0.20)
    est = BallStateEstimator(cfg)
    zs = [0.55, 0.31, 0.12, 0.48, 0.77]
    for i, z in enumerate(zs):
        est.push(i * 0.04, np.array([0.0, 0.0, z]))
    assert est.bounce_detected is False  # one-shot flag belongs to the latest sample
    assert len(est.t_buffer) == 2  # reset occurred when the first rebound sample arrived


def test_sparse_samples_cannot_stretch_fit_window():
    cfg = PlannerConfig(fit_window=31, fit_window_max_span_s=0.15)
    est = BallStateEstimator(cfg)
    for i in range(20):
        est.push(i * 0.10, np.array([1.0 - 0.1 * i, 0.0, 0.6]))
    assert len(est.t_buffer) <= 2
    assert est.t_buffer[-1] - est.t_buffer[0] <= 0.15
    assert not est.ready


def test_estimator_waits_for_minimum_time_span():
    cfg = PlannerConfig(fit_window=31, fit_window_min_span_s=0.10)
    est = BallStateEstimator(cfg)
    for i in range(20):
        est.push(i / 300.0, np.array([1.0 - i / 300.0, 0.0, 0.6]))
    assert len(est.t_buffer) >= 6
    assert est.t_buffer[-1] - est.t_buffer[0] < 0.10
    assert not est.ready
    for i in range(20, 36):
        est.push(i / 300.0, np.array([1.0 - i / 300.0, 0.0, 0.6]))
    assert est.ready


def test_no_false_bounce_on_low_flat_or_monotonic_flight():
    """Low but flat / monotonically descending tracks (no local minimum) must NOT clear."""
    cfg = PlannerConfig()
    est = BallStateEstimator(cfg)
    dt = 1.0 / 300.0
    for i in range(12):  # flat inside the center band
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
