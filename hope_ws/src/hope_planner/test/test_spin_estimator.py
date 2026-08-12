"""SpinFromQuats gates: recovery above the gate, None below the noise floor,
and no aliasing at venue-max spin (15 rev/s over a 100 ms window = 1.5 rev,
which an endpoint finite difference would fold)."""

import numpy as np

from hope_planner.spin_estimator import SpinFromQuats

HZ = 300.0
DT = 1.0 / HZ


def _quat_from_rotvec_wxyz(rv):
    angle = np.linalg.norm(rv)
    if angle < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = rv / angle
    return np.concatenate([[np.cos(angle / 2.0)], np.sin(angle / 2.0) * axis])


def _quat_mul_wxyz(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def _spin_stream(omega, n, q0=None):
    """Orientation stream of a ball spinning at constant world-frame omega."""
    if q0 is None:
        q0 = np.array([1.0, 0.0, 0.0, 0.0])
    return [
        (k * DT, _quat_mul_wxyz(_quat_from_rotvec_wxyz(np.asarray(omega) * k * DT), q0))
        for k in range(n)
    ]


def test_recovers_constant_spin_above_gate():
    est = SpinFromQuats()
    omega = np.array([0.0, 40.0, 0.0])  # 6.4 rev/s > 3 rev/s gate
    out = None
    for t, q in _spin_stream(omega, 45):  # 150 ms
        out = est.push(t, q)
    assert out is not None
    assert np.allclose(out, omega, atol=1e-6)


def test_below_noise_floor_returns_none():
    est = SpinFromQuats()
    omega = np.array([0.0, 10.0, 0.0])  # 1.6 rev/s < gate (venue floor 2 rev/s)
    out = est.push(0.0, np.array([1.0, 0.0, 0.0, 0.0]))
    for t, q in _spin_stream(omega, 45):
        out = est.push(t, q)
    assert out is None


def test_no_aliasing_at_15_rev_s():
    """1.5 revolutions cross the window; per-step chaining must still read
    the true rate (each 300 Hz step is only ~0.05 rev)."""
    est = SpinFromQuats()
    omega = np.array([15.0 * 2.0 * np.pi, 0.0, 0.0])
    q0 = _quat_from_rotvec_wxyz(np.array([0.3, -0.2, 0.5]))  # arbitrary start
    out = None
    for t, q in _spin_stream(omega, 45, q0=q0):
        out = est.push(t, q)
    assert out is not None
    assert np.allclose(out, omega, atol=1e-6)


def test_too_short_baseline_returns_none():
    est = SpinFromQuats(window_s=0.10)
    omega = np.array([0.0, 0.0, 50.0])
    outs = [est.push(t, q) for t, q in _spin_stream(omega, 10)]  # 30 ms < window/2
    assert all(o is None for o in outs)


def test_gap_resets_window_instead_of_bridging_occlusion():
    est = SpinFromQuats(max_gap_s=0.05)
    omega = np.array([0.0, 40.0, 0.0])
    for t, q in _spin_stream(omega, 35):
        est.push(t, q)
    assert est.omega() is not None
    assert est.push(1.0, _quat_from_rotvec_wxyz([0.0, 2.0, 0.0])) is None
    assert est.omega() is None


def test_impossible_one_frame_relock_impulse_is_rejected():
    est = SpinFromQuats(max_rev_s=20.0)
    assert est.push(0.0, np.array([1.0, 0.0, 0.0, 0.0])) is None
    # pi rad in one 300-Hz frame = 150 rev/s, a rigid-body relock, not a
    # physically admissible venue ball sample.
    assert est.push(DT, np.array([0.0, 1.0, 0.0, 0.0])) is None
    assert est.omega() is None
