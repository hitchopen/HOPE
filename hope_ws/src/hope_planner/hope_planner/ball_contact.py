"""No-spin ball-paddle contact model.

Given the incoming ball velocity, the racket contact-point velocity, and the
racket face normal, predict the outgoing ball velocity with the same
simplified impulse model as ``configs/ball_physics.yaml``:

* normal restitution      v_n_out = -e_n v_n_in;
* tangential damping       s = clip((a_t + b_t cos_theta) |u_t|, 0,
                               mu (1 + e_n) |u_n|),
  v_t_out = v_t_in - s * unit(u_t),  cos_theta = |u_n| / |u|.

Parameters come from :class:`PlannerConfig` (``C_r`` normal restitution,
``paddle_a_t``, ``paddle_b_t``, ``paddle_mu``). No spin is modelled.
"""

import numpy as np

from .constants import BallPhysics, PlannerConfig

_EPS = 1e-9


def orient_normal(n: np.ndarray, v_minus: np.ndarray, v_r: np.ndarray) -> np.ndarray:
    """Orient ``n`` so the incoming relative velocity approaches the face.

    Normalize, then flip when ``(v_minus - v_r) . n > 0`` so the returned normal
    points from the face toward the incoming-ball side (the sign convention the
    impulse equations below assume).
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
    physics: BallPhysics,
    config: PlannerConfig,
) -> np.ndarray:
    """Outgoing ball velocity ``v_plus`` from the no-spin impulse model.

    Parameters
    ----------
    v_minus : incoming ball velocity (world frame, m/s).
    v_r : racket contact-point velocity (world frame, m/s).
    n : racket face normal (any sign/length; re-oriented internally).
    physics, config : contact constants.
    """
    v_minus = np.asarray(v_minus, dtype=float)
    v_r = np.asarray(v_r, dtype=float)

    n = orient_normal(n, v_minus, v_r)

    # Relative velocity of the ball w.r.t. the racket surface.
    u = v_minus - v_r
    u_n_signed = float(np.dot(u, n))
    u_t_vec = u - u_n_signed * n
    u_t_mag = float(np.linalg.norm(u_t_vec))
    u_n_abs = abs(u_n_signed)
    u_mag = float(np.linalg.norm(u))

    e_n = config.C_r
    cos_theta = u_n_abs / (u_mag + _EPS)
    raw = (config.paddle_a_t + config.paddle_b_t * cos_theta) * u_t_mag
    cap = config.paddle_mu * (1.0 + e_n) * u_n_abs
    s = min(max(raw, 0.0), cap)

    if u_t_mag > _EPS:
        delta_v_t = -s * (u_t_vec / (u_t_mag + _EPS))
    else:
        delta_v_t = np.zeros(3)

    delta_v_n = -(1.0 + e_n) * u_n_signed * n
    return v_minus + delta_v_n + delta_v_t
