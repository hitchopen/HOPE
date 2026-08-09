"""Ball-physics calibration tests (Planner_Reference_Setup.md Section 2.1)."""

import numpy as np

from hope_planner.calibration import calibrate_ball_physics


def _free_flight(dt=1.0 / 360.0, n=160):
    """Pure ballistic flight (gravity only, no drag) -> fitted k should be ~0."""
    g = -9.81
    v = np.array([3.0, 0.5, 4.0])
    p = np.array([0.0, 0.0, 0.5])
    ts, ps = [], []
    t = 0.0
    for _ in range(n):
        ts.append(t)
        ps.append(p.copy())
        v = v + np.array([0.0, 0.0, g]) * dt
        p = p + v * dt
        t += dt
    return np.array(ts), np.array(ps)


def _clean_bounce(c_v, c_h, vz1=-10.0, vx1=4.0, dt=1.0 / 360.0, n_pre=12, n_post=12):
    """Deterministic descend->contact->rise trajectory with known C_v, C_h.

    Piecewise-linear so finite-difference velocities are exact, and vz1 is large
    enough that both neighbours of the z=0 contact sample exceed the detector's
    0.01 m threshold regardless of phase.
    """
    v_post_z = -c_v * vz1   # positive (upward)
    v_post_x = c_h * vx1
    ts, ps = [], []
    # descending samples (z > 0), contact at index n_pre (z = 0)
    for k in range(n_pre, 0, -1):
        ts.append(len(ts) * dt)
        ps.append([vx1 * (-k) * dt + 0.0, 0.0, vz1 * (-k) * dt])  # vz1<0, -k<0 -> z>0
    ts.append(len(ts) * dt)
    ps.append([0.0, 0.0, 0.0])  # contact
    for k in range(1, n_post + 1):
        ts.append(len(ts) * dt)
        ps.append([v_post_x * k * dt, 0.0, v_post_z * k * dt])  # z>0 rising
    return np.array(ts), np.array(ps)


def test_drag_free_flight_fits_near_zero_k():
    t, p = _free_flight()
    phys = calibrate_ball_physics([p], [t])
    assert phys.k < 0.05, f"k={phys.k}"


def test_restitution_recovered_from_synthetic_bounce():
    # Far from the defaults (0.64 / 0.9215): only passes if the bounce is detected.
    c_v_true, c_h_true = 0.60, 0.55
    t, p = _clean_bounce(c_v_true, c_h_true)
    phys = calibrate_ball_physics([p], [t])
    assert abs(phys.C_v - c_v_true) < 0.02, f"C_v={phys.C_v}"
    assert abs(phys.C_h - c_h_true) < 0.02, f"C_h={phys.C_h}"


def test_empty_input_returns_defaults():
    phys = calibrate_ball_physics([], [])
    assert phys.k == 0.1261 and phys.C_h == 0.64 and phys.C_v == 0.9215
