"""BallKalmanEstimator physics: venue-noise tracking, bounce continuity, gaps.

The noise model matches configs/ball_physics_venue.yaml capture.position_noise
(1.9 mm white + 5.2 mm-marginal AR(1), tau 60 ms, 300 Hz) — the colored term
is exactly what the polyfit estimator cannot reject, so the RMSE comparison
below is the KF's reason to exist.
"""

import numpy as np
import pytest

from hope_planner.ball_kalman_estimator import BallKalmanEstimator
from hope_planner.ball_state_estimator import BallStateEstimator
from hope_planner.constants import BallPhysics, PlannerConfig

HZ = 300.0
DT = 1.0 / HZ


def _integrate_truth(p0, v0, n_frames, physics, bounce=False, substeps=20):
    """Ground-truth drag+gravity flight (optional diagonal table bounce),
    sub-stepped Euler so integration error is far below the noise floor."""
    h = DT / substeps
    p = p0.astype(float).copy()
    v = v0.astype(float).copy()
    ps, vs = [p.copy()], [v.copy()]
    for _ in range(n_frames - 1):
        for _ in range(substeps):
            a = physics.g - physics.k * np.linalg.norm(v) * v
            p_new = p + v * h + 0.5 * a * h ** 2
            v_new = v + a * h
            if bounce and p_new[2] < 0.0 and v_new[2] < 0.0:
                frac = p[2] / (p[2] - p_new[2])
                v_at = v + a * (frac * h)
                v_new = np.array([physics.C_h * v_at[0],
                                  physics.C_h * v_at[1],
                                  -physics.C_v * v_at[2]])
                p_new = p + frac * (p_new - p)
                p_new[2] = 0.0
            p, v = p_new, v_new
        ps.append(p.copy())
        vs.append(v.copy())
    return np.array(ps), np.array(vs)


def _venue_noise(n_frames, rng, white=0.0019, ar1_marg=0.0052, tau=0.060):
    """White + AR(1) per-axis noise, rho = exp(-dt/tau), stationary marginal."""
    rho = np.exp(-DT / tau)
    b = np.zeros((n_frames, 3))
    b[0] = rng.normal(0.0, ar1_marg, 3)
    step_std = ar1_marg * np.sqrt(1.0 - rho ** 2)
    for k in range(1, n_frames):
        b[k] = rho * b[k - 1] + rng.normal(0.0, step_std, 3)
    return b + rng.normal(0.0, white, (n_frames, 3))


def _no_bounce_arc(n_frames=150, seed=7):
    """0.5 s descending arc that stays above the bounce tolerance."""
    physics = BallPhysics()
    p0 = np.array([2.5, -0.76, 0.35])
    v0 = np.array([-3.0, 0.3, 2.0])
    ps, vs = _integrate_truth(p0, v0, n_frames, physics)
    assert ps[:, 2].min() > 0.05  # no false bounce triggers
    rng = np.random.default_rng(seed)
    zs = ps + _venue_noise(n_frames, rng)
    return ps, vs, zs


def test_kf_beats_polyfit_position_and_velocity_after_150ms():
    cfg = PlannerConfig()
    ps, vs, zs = _no_bounce_arc()
    kf = BallKalmanEstimator(cfg)
    poly = BallStateEstimator(cfg)

    kf_p_err, kf_v_err, poly_p_err = [], [], []
    for k in range(len(zs)):
        t = k * DT
        kf.push(t, zs[k])
        poly.push(t, zs[k])
        if t < 0.150:
            continue
        p_kf, v_kf, _ = kf.estimate()
        kf_p_err.append(np.linalg.norm(p_kf - ps[k]))
        kf_v_err.append(np.linalg.norm(v_kf - vs[k]))
        p_pf, _, _ = poly.estimate()
        poly_p_err.append(np.linalg.norm(p_pf - ps[k]))

    kf_p_rmse = np.sqrt(np.mean(np.square(kf_p_err)))
    poly_p_rmse = np.sqrt(np.mean(np.square(poly_p_err)))
    kf_v_rmse = np.sqrt(np.mean(np.square(kf_v_err)))
    assert kf_p_rmse < poly_p_rmse, (kf_p_rmse, poly_p_rmse)
    assert kf_v_rmse < 0.15, kf_v_rmse


def test_kf_bounce_continuity_no_nan_psd_available_next_frame():
    """The KF must NOT clear at a bounce: estimate stays available within one
    frame, covariance stays PSD, nothing goes NaN."""
    cfg = PlannerConfig()
    physics = BallPhysics()
    p0 = np.array([1.5, -0.76, 0.3])
    v0 = np.array([-2.0, 0.0, -1.0])
    n = 150  # 0.5 s, bounce at ~0.166 s
    ps, vs = _integrate_truth(p0, v0, n, physics, bounce=True)
    rng = np.random.default_rng(4)
    zs = ps + _venue_noise(n, rng)

    kf = BallKalmanEstimator(cfg)
    bounce_frame = None
    for k in range(n):
        kf.push(k * DT, zs[k])
        assert np.all(np.isfinite(kf._x))
        assert np.linalg.eigvalsh(kf._P).min() > -1e-10
        if kf.bounce_detected and bounce_frame is None:
            bounce_frame = k
        if bounce_frame is not None and k >= bounce_frame:
            assert kf.ready  # available within 1 frame of the bounce
            p_est, v_est, _ = kf.estimate()
            assert np.all(np.isfinite(p_est)) and np.all(np.isfinite(v_est))

    assert bounce_frame is not None
    # A few frames after the bounce the filter must have flipped vz upward
    # and re-converged (the polyfit path is still refilling its window here).
    _, v_end, _ = kf.estimate()
    assert v_end[2] * vs[-1][2] > 0.0
    assert np.linalg.norm(v_end - vs[-1]) < 0.25


def test_kf_tolerates_30ms_measurement_gap():
    """Occlusion gaps (capture.timing.racket_occlusion_ms) must not break the
    filter: one predict with a bigger dt and bigger Q, then reconverge."""
    cfg = PlannerConfig()
    ps, vs, zs = _no_bounce_arc(seed=23)
    kf = BallKalmanEstimator(cfg)
    for k in range(len(zs)):
        t = k * DT
        if 0.20 <= t < 0.23:
            continue  # 30 ms occlusion mid-arc
        kf.push(t, zs[k])
        assert np.all(np.isfinite(kf._x))
        assert np.linalg.eigvalsh(kf._P).min() > -1e-10
    assert kf.ready
    p_end, v_end, _ = kf.estimate()
    assert np.linalg.norm(p_end - ps[-1]) < 0.02
    assert np.linalg.norm(v_end - vs[-1]) < 0.15


def test_kf_robustly_clips_and_counts_outlier():
    cfg = PlannerConfig()
    ps, _, zs = _no_bounce_arc(seed=42)
    kf = BallKalmanEstimator(cfg)
    spike_frame = 90
    for k in range(len(zs)):
        z = zs[k].copy()
        if k == spike_frame:
            z += np.array([0.10, -0.08, 0.12])  # mislabeled marker / ghost
        kf.push(k * DT, z)
    assert kf.rejected_count >= 1
    p_end, _, _ = kf.estimate()
    assert np.linalg.norm(p_end - ps[-1]) < 0.02  # spike did not poison the state


def test_kf_restarts_disconnected_track_after_source_gap():
    cfg = PlannerConfig(estimator_track_gap_s=0.05)
    kf = BallKalmanEstimator(cfg)
    for index in range(6):
        kf.push(index * DT, np.array([1.5 - index * 0.01, -0.7, 0.4]))
    assert kf.ready

    kf.push(0.20, np.array([2.4, -0.6, 0.8]))
    assert not kf.ready
    assert kf.sample_count == 1
    assert kf.track_restart_count == 1
    assert kf.last_restart_reason == "source_gap"


def test_kf_does_not_bounce_on_low_noisy_local_minimum():
    """A raw local minimum at 0.17 m is not a physical table crossing."""
    kf = BallKalmanEstimator(PlannerConfig())
    for index, z in enumerate((0.18, 0.17, 0.18)):
        kf.push(index * DT, np.array([1.4 - index * 0.01, -0.7, z]))
        assert not kf.bounce_detected


def test_kf_rejects_nonfinite_measurement_without_poisoning_state():
    kf = BallKalmanEstimator(PlannerConfig())
    kf.push(0.0, np.array([1.4, -0.7, 0.4]))
    state_before = kf._x.copy()
    with pytest.raises(ValueError, match="finite"):
        kf.push(DT, np.array([1.39, np.nan, 0.4]))
    np.testing.assert_array_equal(kf._x, state_before)
    assert kf.sample_count == 1


def test_kf_estimate_full_and_predict_to():
    cfg = PlannerConfig()
    physics = BallPhysics()
    ps, vs, zs = _no_bounce_arc(seed=3)
    kf = BallKalmanEstimator(cfg)
    for k in range(len(zs)):
        kf.push(k * DT, zs[k])
    x, P = kf.estimate_full()
    assert x.shape == (9,) and P.shape == (9, 9)
    assert np.linalg.eigvalsh(P).min() > -1e-10

    # predict_to must match the same flight model integrated from the truth.
    t_end = (len(zs) - 1) * DT
    horizon = 0.1
    p_pred, v_pred = kf.predict_to(t_end + horizon)
    truth_p, truth_v = _integrate_truth(
        ps[-1], vs[-1], int(round(horizon / DT)) + 1, physics)
    assert np.linalg.norm(p_pred - truth_p[-1]) < 0.02
    assert np.linalg.norm(v_pred - truth_v[-1]) < 0.2
