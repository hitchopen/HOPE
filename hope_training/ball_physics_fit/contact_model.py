"""No-spin contact model.

How the ball's linear velocity changes when it hits a surface (the table top or
the racket face). No angular velocity / spin is modelled anywhere.

Take the incoming ball velocity relative to the surface, ``u = v_minus - v_r``
(``v_r`` = surface velocity: 0 for the static table, the paddle contact-point
velocity for a racket hit), and split it about the unit surface normal ``n``::

    u_n = u . n            (signed; negative while approaching the surface)
    u_t = u - u_n * n      (tangential part)

The outgoing velocity is then::

    normal      :  dv_n = -(1 + e_n) * u_n * n                 # restitution e_n
    tangential  :  s    = clip((a_t + b_t*cos_theta) * |u_t|,  # grip / damping
                               0, mu * (1 + e_n) * |u_n|)      # Coulomb cap
                   dv_t = -s * unit(u_t)
    v_plus = v_minus + dv_n + dv_t

with ``cos_theta = |u_n| / |u|`` the impact-angle factor. With ``b_t = 0`` the
tangential response is simply ``|u_t| -> (1 - a_t)|u_t|`` until the friction cap
``mu`` binds (only on very oblique contacts). All inputs are SI, world frame.
"""
import numpy as np


def predict_contact(v_minus, v_r, n, e_n, a_t, b_t=0.0, mu=1.0):
    """Predict the post-contact ball velocity.

    Inputs are ``(N, 3)`` arrays (or broadcastable to one); ``e_n`` is a scalar
    or ``(N,)``. Returns a dict with:
      * ``v_plus``   ``(N, 3)`` outgoing velocity
      * ``u_n``      ``(N,)`` signed normal approach speed
      * ``u_t``      ``(N,)`` tangential approach speed magnitude
      * ``cap_binds`` ``(N,)`` bool, True where the friction cap limited the impulse
      * ``n``        ``(N, 3)`` the normal actually used (oriented against approach)
    """
    v_minus = np.atleast_2d(np.asarray(v_minus, float))
    v_r = np.broadcast_to(np.atleast_2d(np.asarray(v_r, float)), v_minus.shape)
    n = np.broadcast_to(np.atleast_2d(np.asarray(n, float)), v_minus.shape).astype(float).copy()
    # orient n so that the approach is into the surface (u_n < 0)
    approach = ((v_minus - v_r) * n).sum(1)
    n[approach > 0] *= -1

    u = v_minus - v_r
    u_n = (u * n).sum(1)
    u_t_vec = u - u_n[:, None] * n
    u_t = np.linalg.norm(u_t_vec, axis=1)
    cos_th = np.abs(u_n) / (np.hypot(u_t, u_n) + 1e-12)

    e = np.broadcast_to(np.asarray(e_n, float), u_n.shape)
    raw = (a_t + b_t * cos_th) * u_t
    cap = mu * (1.0 + e) * np.abs(u_n)
    s = np.clip(raw, 0.0, cap)
    unit_t = u_t_vec / (u_t[:, None] + 1e-12)

    dv_n = -((1.0 + e) * u_n)[:, None] * n
    dv_t = -s[:, None] * unit_t
    return dict(v_plus=v_minus + dv_n + dv_t, u_n=u_n, u_t=u_t,
                cap_binds=raw > cap, n=n)


def restitution_vs_speed(u_n, form="const", e_const=None, a=None, b=None,
                         g1=None, g2=None):
    """Optional speed-dependent normal restitution ``e_n(|u_n|)``.

    Some surfaces show a mild fall of ``e_n`` with impact speed. Supported forms:
      * ``const``  -> ``e_const``
      * ``linear`` -> ``a - b * |u_n|``
      * ``exp``    -> ``g1 * exp(g2 * |u_n|)``  (stays positive when extrapolated)
    The shipped configs use the constant form; the others are here so a re-fit
    can adopt a speed-dependent ``e_n`` if the data supports it (see falsify/).
    """
    un = np.abs(np.asarray(u_n, float))
    if form == "const":
        return np.full(un.shape, e_const) if un.ndim else e_const
    if form == "linear":
        return a - b * un
    if form == "exp":
        return g1 * np.exp(g2 * un)
    raise ValueError(f"unknown restitution form {form!r}")
