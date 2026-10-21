"""
Angle-dependent tangential-impulse contact model (port of the v0
Record/analysis/contact_model/spin_equation.py, vectorized, SI units).

    u    = v- + w- x (-R n) - v_r
    s    = clip((a_t + b_t cos(theta)) |u_t|, 0, mu (1+e) |u_n|)
    dv_t = -s unit(u_t);  dv_n = -(1+e) u_n n;  dw = -(1/(cR)) n x dv_t

Extended forms for the falsification fixes:
    e(u_n) linear     : e = e0 - e1 * |u_n|
    e(u_n) exponential: e = g1 * exp(g2 * |u_n|)   (Ace paddle form)
"""
import numpy as np

R_BALL = 0.020
C_INERTIA = 2.0 / 3.0


def predict_contact(v_minus, v_r, n, omega_minus, e_eff, a_t, b_t, mu):
    """All inputs (N,3) or broadcastable; e_eff scalar or (N,). Returns v_plus,
    omega_plus, u_n (signed, N), u_t_mag (N)."""
    v_minus = np.atleast_2d(np.asarray(v_minus, float))
    v_r = np.broadcast_to(np.atleast_2d(np.asarray(v_r, float)), v_minus.shape)
    omega_minus = np.broadcast_to(np.atleast_2d(np.asarray(omega_minus, float)), v_minus.shape)
    n = np.broadcast_to(np.atleast_2d(np.asarray(n, float)), v_minus.shape).copy()
    # orient n against approach
    approach = ((v_minus - v_r) * n).sum(1)
    n[approach > 0] *= -1
    r = -R_BALL * n
    u = v_minus + np.cross(omega_minus, r) - v_r
    u_n_signed = (u * n).sum(1)
    u_n_vec = u_n_signed[:, None] * n
    u_t_vec = u - u_n_vec
    u_t_mag = np.linalg.norm(u_t_vec, axis=1)
    cos_th = np.abs(u_n_signed) / (np.hypot(u_t_mag, u_n_signed) + 1e-12)
    e = np.broadcast_to(np.asarray(e_eff, float), u_n_signed.shape)
    raw = (a_t + b_t * cos_th) * u_t_mag
    cap = mu * (1.0 + e) * np.abs(u_n_signed)
    s = np.clip(raw, 0.0, cap)
    unit_t = u_t_vec / (u_t_mag[:, None] + 1e-12)
    dv_t = -s[:, None] * unit_t
    dv_n = -((1.0 + e) * u_n_signed)[:, None] * n
    dw = -(1.0 / (C_INERTIA * R_BALL)) * np.cross(n, dv_t)
    return dict(v_plus=v_minus + dv_n + dv_t, omega_plus=omega_minus + dw,
                u_n=u_n_signed, u_t=u_t_mag, cap_binds=raw > cap, n=n)
