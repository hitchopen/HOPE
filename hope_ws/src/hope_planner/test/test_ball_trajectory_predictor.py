"""Stage 2 unit tests (HOPE_7DOF...Planner_Reference_Setup.md Section 4 gates)."""

import numpy as np
import pytest

from hope_planner.ball_trajectory_predictor import BallTrajectoryPredictor
from hope_planner.constants import BallPhysics, PlannerConfig, TableParams


def _predictor():
    return BallTrajectoryPredictor(BallPhysics(), PlannerConfig(), TableParams())


def test_incoming_trajectory_crosses_hit_plane():
    pred = _predictor()
    # Ball above the table, moving toward P1 (-x), high enough to avoid a bounce.
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
    v0 = np.array([3.0, 0.0, 1.0])  # moving toward P2 (+x): never crosses x_hit
    strike = pred.predict(p0, v0, 0.0)
    assert not strike.valid


def test_adaptive_horizon_extends_slow_incoming_search_only():
    base = BallTrajectoryPredictor(
        BallPhysics(), PlannerConfig(max_predict_time=2.0), TableParams()
    )
    adaptive = BallTrajectoryPredictor(
        BallPhysics(),
        PlannerConfig(
            max_predict_time=2.0,
            adaptive_predict_horizon=True,
            max_predict_time_cap=3.0,
        ),
        TableParams(),
    )
    p0 = np.array([1.0, -0.7625, 0.5])
    slow_incoming = np.array([-0.5, 0.0, 1.0])
    assert base._prediction_horizon_s(p0, slow_incoming) == 2.0
    assert adaptive._prediction_horizon_s(p0, slow_incoming) == 3.0
    assert adaptive._prediction_horizon_s(
        p0, np.array([0.5, 0.0, 1.0])
    ) == 2.0


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


# --- Magnus + nakashima bounce upgrade (flag-gated; defaults = legacy) ---


def test_magnus_off_equals_legacy_prediction():
    """omega=None and omega=0 must be bit-identical (the Magnus term adds an
    exact 0.0), and killing k_m must neutralize even a large spin."""
    pred = _predictor()
    p0 = np.array([0.5, -0.7625, 0.5])
    v0 = np.array([-4.0, 0.0, 2.0])
    s_default = pred.predict(p0, v0, 0.0)
    s_zeros = pred.predict(p0, v0, 0.0, omega0=np.zeros(3))
    assert np.array_equal(s_default.p_ball, s_zeros.p_ball)
    assert np.array_equal(s_default.v_ball, s_zeros.v_ball)
    assert s_default.t_strike == s_zeros.t_strike

    pred_no_km = BallTrajectoryPredictor(
        BallPhysics(), PlannerConfig(k_m=0.0), TableParams())
    s_spun = pred_no_km.predict(p0, v0, 0.0, omega0=np.array([0.0, -90.0, 0.0]))
    assert np.allclose(s_spun.p_ball, s_default.p_ball)
    assert np.isclose(s_spun.t_strike, s_default.t_strike)


@pytest.mark.parametrize(
    "p0,v0",
    [
        (np.array([0.5, -0.7625, 0.5]), np.array([-4.0, 0.0, 2.0])),
        (np.array([0.2, -0.7625, 0.03]), np.array([-1.5, 0.0, -2.0])),
        (np.array([0.2, 1.0, 0.03]), np.array([-1.5, 0.0, -2.0])),
        (np.array([3.0, -0.7625, 0.5]), np.array([-0.5, 0.0, 1.0])),
        (np.array([0.5, -0.7625, 0.5]), np.array([3.0, 0.0, 1.0])),
    ],
)
def test_scalar_diagonal_predictor_matches_vector_reference(p0, v0):
    """Production scalar path must preserve the retained vector physics."""
    pred = _predictor()
    scalar = pred.predict(p0, v0, 123.0)
    scalar_reason = pred.last_reason
    vector = pred.predict(p0, v0, 123.0, omega0=np.zeros(3))
    vector_reason = pred.last_reason
    np.testing.assert_allclose(scalar.p_ball, vector.p_ball, rtol=0.0, atol=1e-12)
    np.testing.assert_allclose(scalar.v_ball, vector.v_ball, rtol=0.0, atol=1e-12)
    assert scalar.t_strike == pytest.approx(vector.t_strike, abs=1e-12)
    assert scalar.num_bounces == vector.num_bounces
    assert scalar.valid == vector.valid
    assert scalar_reason == vector_reason


def test_magnus_topspin_drops_the_ball():
    """Topspin on a ball flying toward P1 (-x) means omega about -y; the
    Magnus force then points downward, so the strike arrives lower/earlier."""
    pred = _predictor()
    p0 = np.array([1.5, -0.7625, 0.5])
    v0 = np.array([-4.0, 0.0, 1.5])
    s_flat = pred.predict(p0, v0, 0.0)
    s_top = pred.predict(p0, v0, 0.0, omega0=np.array([0.0, -94.2, 0.0]))
    assert s_flat.valid and s_top.valid
    assert s_top.p_ball[2] < s_flat.p_ball[2]


def _nakashima_predictor():
    return BallTrajectoryPredictor(
        BallPhysics(), PlannerConfig(bounce_model="nakashima"), TableParams())


def _contact_tangential(v, w, r):
    """Contact-point tangential velocity of the ball on the table."""
    return np.array([v[0] - r * w[1], v[1] + r * w[0], 0.0])


def test_nakashima_rolling_bounce_zeroes_contact_velocity():
    """Physical invariant: once the contact sticks (alpha capped at 2/5) the
    OUTGOING contact-point tangential velocity is exactly zero."""
    pred = _nakashima_predictor()
    phys = BallPhysics()
    v_in = np.array([2.0, 0.5, -3.0])
    w_in = np.array([30.0, 94.2, 10.0])
    # Confirm this case is in the rolling regime.
    v_T = _contact_tangential(v_in, w_in, phys.radius)
    alpha_slide = pred.config.mu_table * (1 + phys.C_v) * abs(v_in[2]) / np.linalg.norm(v_T)
    assert alpha_slide > 0.4

    v_out, w_out = pred._apply_bounce_nakashima(v_in, w_in)
    v_T_out = _contact_tangential(v_out, w_out, phys.radius)
    assert np.linalg.norm(v_T_out) < 1e-9
    assert np.isclose(v_out[2], phys.C_v * 3.0)  # e_n = C_v on the normal axis
    assert w_out[2] == w_in[2]                   # spin about the normal untouched


def test_nakashima_sliding_bounce_matches_impulse_law():
    """While sliding (nu_s > 0): v+_t = v_t - alpha v_T with
    alpha = mu (1+e_n) |vz| / |v_T|."""
    pred = _nakashima_predictor()
    phys = BallPhysics()
    v_in = np.array([5.0, 0.0, -2.0])
    w_in = np.zeros(3)  # no spin: v_T = v_t = [5, 0, 0]
    alpha = pred.config.mu_table * (1 + phys.C_v) * 2.0 / 5.0
    assert 1.0 - 2.5 * alpha > 0.0  # sliding persists
    v_out, w_out = pred._apply_bounce_nakashima(v_in, w_in)
    assert np.isclose(v_out[0], 5.0 - alpha * 5.0)
    assert np.isclose(v_out[1], 0.0)
    assert np.isclose(v_out[2], phys.C_v * 2.0)
    # Friction torque spins the ball forward (topspin, +y for +x travel).
    assert w_out[1] > 0.0


def test_nakashima_topspin_kick_magnitude_at_venue_speeds():
    """Report §10.1 anchor: ~0.7 m/s extra outgoing tangential speed for a
    15 rev/s topspin ball at typical venue speeds. When both the spun and
    no-spin bounces roll, the kick is exactly (2/5) r |w| = 0.75 m/s."""
    pred = _nakashima_predictor()
    v_in = np.array([2.0, 0.0, -3.0])
    w_top = np.array([0.0, 15.0 * 2.0 * np.pi, 0.0])  # topspin for +x travel

    v_flat, _ = pred._apply_bounce_nakashima(v_in, np.zeros(3))
    v_spun, _ = pred._apply_bounce_nakashima(v_in, w_top)
    kick = v_spun[0] - v_flat[0]
    assert 0.6 < kick < 0.9
    assert np.isclose(kick, 0.4 * pred.physics.radius * w_top[1], atol=1e-9)


def test_nakashima_predict_path_uses_spin_coupled_bounce():
    """End-to-end: same incoming state, topspin + nakashima must cross the
    hit plane faster (hotter tangential exit) than the diagonal model."""
    p0 = np.array([0.6, -0.7625, 0.15])
    v0 = np.array([-3.0, 0.0, -1.0])
    # Topspin for motion toward P1 (-x): top surface moves -x -> omega about -y.
    w_top = np.array([0.0, -94.2, 0.0])

    s_diag = _predictor().predict(p0, v0, 0.0)
    s_nak = _nakashima_predictor().predict(p0, v0, 0.0, omega0=w_top)
    assert s_diag.valid and s_nak.valid
    assert s_diag.num_bounces == 1 and s_nak.num_bounces == 1
    # Spin-coupled bounce keeps more tangential speed than C_h = 0.64 under
    # topspin -> earlier plane crossing.
    assert s_nak.t_strike < s_diag.t_strike
