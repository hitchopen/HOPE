"""Ball-paddle contact model (numpy port of the training-side virtual ball).

EXACT single-sample port of ``predict_paddle_contact`` /``orient_normal`` from
hope_training/.../tasks/tracking/mdp/virtual_ball.py (the vectorized torch
implementation used by the Tier-1 landing reward), so planner and training
share ONE venue-fitted contact law:

* spin-impulse tangential block  s = (a_t + b_t*cos_theta) * |u_t|,
  capped by the friction cone   mu * (1 + e) * |u_n|
* F4 velocity-dependent normal restitution  e(u_n) = g1 * exp(g2 * |u_n|),
  clamped to [0.05, 0.95]
* spin update from the impulse torque  delta_omega = -(1/(cR)) n x delta_v_t
  with hollow-sphere inertia coefficient c = 2/3.

Params come from constants.py (PlannerConfig paddle_* / e_exp_* /
ball_inertia_coeff, BallPhysics.radius), which mirror
configs/ball_physics_venue.yaml. Validity envelope: paddle u_n 1.4-7.2 m/s
(the e(u_n) clamp keeps far-out contacts physical, not accurate).
"""

from typing import Tuple

import numpy as np

from .constants import BallPhysics, PlannerConfig

# Same epsilon as the torch reference (virtual_ball._EPS) so the two ports
# agree bit-for-bit on degenerate inputs (zero tangential slip, zero normal).
_EPS = 1e-9


def orient_normal(n: np.ndarray, v_minus: np.ndarray, v_r: np.ndarray) -> np.ndarray:
    """Orient ``n`` so the incoming relative velocity approaches the face.

    Port of virtual_ball.orient_normal: normalize, then flip when
    ``(v_minus - v_r) . n > 0``. The returned normal points from the face
    toward the incoming-ball side, which is the sign convention the impulse
    equations below assume.
    """
    n = np.asarray(n, dtype=float)
    n = n / (np.linalg.norm(n) + _EPS)
    if float(np.dot(np.asarray(v_minus, dtype=float) - np.asarray(v_r, dtype=float), n)) > 0.0:
        return -n
    return n


def predict_paddle_contact(
    v_minus: np.ndarray,
    v_r: np.ndarray,
    n: np.ndarray,
    omega_minus: np.ndarray,
    physics: BallPhysics,
    config: PlannerConfig,
) -> Tuple[np.ndarray, np.ndarray]:
    """Outgoing ball ``(v_plus, omega_plus)`` from the fitted spin equation.

    Parameters
    ----------
    v_minus : incoming ball velocity (world frame, m/s).
    v_r : racket CONTACT-POINT velocity (world frame, m/s) — the venue fit
        included the omega_racket x r term, so callers must too.
    n : racket face normal (any sign/length; re-oriented internally).
    omega_minus : incoming ball spin (rad/s).
    physics, config : venue constants (see module docstring for provenance).
    """
    R = physics.radius
    c = config.ball_inertia_coeff

    v_minus = np.asarray(v_minus, dtype=float)
    v_r = np.asarray(v_r, dtype=float)
    omega_minus = np.asarray(omega_minus, dtype=float)

    n = orient_normal(n, v_minus, v_r)
    r = -R * n  # face -> ball-center contact offset

    # Relative velocity of the ball CONTACT POINT w.r.t. the racket surface.
    u = v_minus + np.cross(omega_minus, r) - v_r
    u_n_signed = float(np.dot(u, n))
    u_t_vec = u - u_n_signed * n
    u_t_mag = float(np.linalg.norm(u_t_vec))
    u_n_abs = abs(u_n_signed)

    # F4: velocity-dependent normal restitution (venue fit, valid u_n 1.4-7.2 m/s).
    e_eff = float(np.clip(config.e_exp_g1 * np.exp(config.e_exp_g2 * u_n_abs), 0.05, 0.95))

    cos_theta = u_n_abs / (np.hypot(u_t_mag, u_n_signed) + _EPS)
    raw = (config.paddle_a_t + config.paddle_b_t * cos_theta) * u_t_mag
    cap = config.paddle_mu * (1.0 + e_eff) * u_n_abs
    s = min(max(raw, 0.0), cap)

    if u_t_mag > _EPS:
        delta_v_t = -s * (u_t_vec / (u_t_mag + _EPS))
    else:
        delta_v_t = np.zeros(3)

    delta_v_n = -(1.0 + e_eff) * u_n_signed * n
    delta_omega = -(1.0 / (c * R)) * np.cross(n, delta_v_t)

    v_plus = v_minus + delta_v_n + delta_v_t
    omega_plus = omega_minus + delta_omega
    return v_plus, omega_plus
