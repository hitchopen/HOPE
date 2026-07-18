"""Ball trajectory predictor unit tests (no-spin flight + diagonal table bounce)."""

import numpy as np

from hope_planner.ball_trajectory_predictor import BallTrajectoryPredictor
from hope_planner.constants import BallPhysics, PlannerConfig, TableParams


def _predictor():
    return BallTrajectoryPredictor(BallPhysics(), PlannerConfig(), TableParams())


def test_incoming_trajectory_crosses_hit_plane():
    pred = _predictor()
    # Ball above the table, moving toward the robot (-x), high enough to avoid a bounce.
    p0 = np.array([0.5, -0.7625, 0.5])
    v0 = np.array([-4.0, 0.0, 2.0])
    strike = pred.predict(p0, v0, 0.0)
    assert strike.valid
    assert abs(strike.p_ball[0] - 0.0) < 1e-6   # x_hit default = 0.0
    assert strike.num_bounces == 0
    assert 0.05 < strike.t_strike < 0.4


def test_ball_moving_away_produces_no_valid_command():
    pred = _predictor()
    p0 = np.array([0.5, -0.7625, 0.5])
    v0 = np.array([3.0, 0.0, 1.0])  # moving toward the opponent (+x): never crosses x_hit
    strike = pred.predict(p0, v0, 0.0)
    assert not strike.valid


def test_table_bounce_reverses_z_velocity():
    pred = _predictor()
    v_minus = np.array([2.0, 0.0, -3.0])
    v_plus = pred._apply_bounce(v_minus)
    assert v_plus[2] > 0.0                       # downward -> upward
    assert np.isclose(v_plus[2], BallPhysics().C_v * 3.0)
    assert np.isclose(v_plus[0], BallPhysics().C_h * 2.0)


def test_bounce_then_cross_hit_plane_path():
    pred = _predictor()
    p0 = np.array([0.2, -0.7625, 0.03])
    v0 = np.array([-1.5, 0.0, -2.0])
    strike = pred.predict(p0, v0, 0.0)
    assert strike.valid
    assert strike.num_bounces == 1
    assert abs(strike.p_ball[0]) < 1e-6
    assert strike.p_ball[2] > 0.0


def test_bounce_outside_table_bounds_not_valid():
    pred = _predictor()
    on_table = np.array([1.0, -0.7625, -0.01])   # within bounds (expanded by radius)
    off_table = np.array([3.0, -0.7625, -0.01])  # x beyond table length + radius
    assert pred._is_on_table(on_table)
    assert not pred._is_on_table(off_table)


def test_bounce_contact_plane_is_ball_radius():
    """The bounce must trigger when the CENTROID reaches z = ball radius, not z = 0.

    With drag disabled the whole trajectory is analytic, so the strike time pins the
    contact convention: a z = 0 bounce plane would put the bounce ~14 ms later and
    shift every post-bounce prediction.
    """
    physics = BallPhysics(k=0.0)          # pure gravity -> closed-form expectations
    pred = BallTrajectoryPredictor(physics, PlannerConfig(), TableParams())
    g = 9.81
    r = physics.radius
    z0, x0, vx = 0.12, 0.10, -0.5
    strike = pred.predict(np.array([x0, -0.7625, z0]), np.array([vx, 0.0, 0.0]), 0.0)

    # Analytic: free fall to the contact plane z = r, diagonal bounce, rise to x_hit = 0.
    t_bounce = np.sqrt(2.0 * (z0 - r) / g)
    x_bounce = x0 + vx * t_bounce
    vx_post = physics.C_h * vx
    vz_post = physics.C_v * (g * t_bounce)
    tau = (0.0 - x_bounce) / vx_post
    t_strike_expected = t_bounce + tau
    z_expected = r + vz_post * tau - 0.5 * g * tau**2

    assert strike.valid
    assert strike.num_bounces == 1
    assert abs(strike.t_strike - t_strike_expected) < 2e-3, (
        f"strike time {strike.t_strike:.4f} != analytic {t_strike_expected:.4f} "
        "(bounce plane must be z = ball radius)"
    )
    assert abs(strike.p_ball[2] - z_expected) < 5e-3
    # Sanity: the z = 0 convention lands ~8 ms away and must NOT fit.
    t_bounce_wrong = np.sqrt(2.0 * z0 / g)
    t_strike_wrong = t_bounce_wrong + (0.0 - (x0 + vx * t_bounce_wrong)) / vx_post
    assert abs(strike.t_strike - t_strike_wrong) > 4e-3
