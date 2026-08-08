"""Tier-1 virtual-ball physics for the at-strike landing reward (rewardDesign.md).

Vectorized torch port of the VENUE-FITTED ball model (``configs/ball_physics_venue.yaml``,
2026-07-03 Avatar Pro fit on the 3.4 g coated MATCH ball):

* flight:  a = g - k_d |v| v + k_m (omega x v)          (k_d = 0.1261, k_m = 0.00444)
* paddle:  fitted spin-impulse contact (spin_contact.py form) with the F4-adopted
  VELOCITY-DEPENDENT restitution e(u_n) = g1 * exp(g2 * |u_n|) instead of constant e
  (report §4 F4: constant-e KILLED; exponential preferred for extrapolation).

The landing predictor here is the COARSE, fixed-length, branch-free rollout mandated by
verify_tier1-reward-soundness.md (b): RK4 at h = 10 ms with linear-interpolation crossing
extraction — NOT the shipped ``predict_landing`` (h = 1 ms + 24-iter bisection), which is
~15x too slow for the training step loop (+300 %/iter). Interp error at h = 10 ms is
~0.5 mm, far below the sigma = 0.3 m landing kernel.

Everything is per-env ``(N, 3)`` tensors and runs on the full batch every strike step
(the cost is kernel-launch-bound, batch-size independent).

Validity envelope of the constants (do not extrapolate silently): ball speed 1-7 m/s,
spin 0-15 rev/s, paddle u_n 1.4-7.2 m/s. The samplers in hope_commands stay inside it.
"""

from __future__ import annotations

import os
from dataclasses import dataclass

import torch
import yaml

_EPS = 1e-9


@dataclass(frozen=True)
class VirtualBallParams:
    """Flight + paddle-contact constants (venue fit)."""

    k_d: float
    k_m: float
    g: float
    ball_radius: float
    inertia_coeff: float
    paddle_a_t: float
    paddle_b_t: float
    paddle_mu: float
    paddle_e_g1: float  # e(u_n) = g1 * exp(g2 * |u_n|)
    paddle_e_g2: float
    source_path: str


def default_venue_yaml_path() -> str:
    """Resolve ``configs/ball_physics_venue.yaml`` (env override, then upward search)."""
    env = os.environ.get("HOPE_BALL_PHYSICS_YAML")
    if env:
        return env
    d = os.path.dirname(os.path.abspath(__file__))
    while True:
        cand = os.path.join(d, "configs", "ball_physics_venue.yaml")
        if os.path.exists(cand):
            return cand
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    raise FileNotFoundError(
        "ball_physics_venue.yaml not found; set $HOPE_BALL_PHYSICS_YAML or place it at "
        "<repo>/configs/ball_physics_venue.yaml"
    )


def load_venue_params(path: str | None = None) -> VirtualBallParams:
    path = path or default_venue_yaml_path()
    with open(path, "r") as fh:
        raw = yaml.safe_load(fh)
    pad = raw["contact"]["paddle"]
    return VirtualBallParams(
        k_d=float(raw["flight"]["k_d"]),
        k_m=float(raw["flight"]["k_m"]),
        g=float(raw["flight"]["g"]),
        ball_radius=float(raw["ball"]["radius"]),
        inertia_coeff=float(raw["ball"]["inertia_coeff"]),
        paddle_a_t=float(pad["a_t"]),
        paddle_b_t=float(pad["b_t"]),
        paddle_mu=float(pad["mu_safety"]),
        paddle_e_g1=float(pad["e_exp_g1"]),
        paddle_e_g2=float(pad["e_exp_g2"]),
        source_path=os.path.abspath(path),
    )


def _normalize(v: torch.Tensor, eps: float = _EPS) -> torch.Tensor:
    return v / (torch.linalg.norm(v, dim=-1, keepdim=True) + eps)


def orient_normal(n: torch.Tensor, v_minus: torch.Tensor, v_r: torch.Tensor) -> torch.Tensor:
    """Orient ``n`` per env so the incoming relative velocity approaches the face.

    Same as the v0 branch ``spin_contact.orient_normal``: flip where ``(v_minus - v_r) . n > 0``.
    The returned normal points from the face toward the incoming-ball side.
    """
    n = _normalize(n)
    approaching = torch.sum((v_minus - v_r) * n, dim=-1, keepdim=True) > 0.0
    return torch.where(approaching, -n, n)


def predict_paddle_contact(
    v_minus: torch.Tensor,
    v_r: torch.Tensor,
    n: torch.Tensor,
    omega_minus: torch.Tensor,
    prm: VirtualBallParams,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Outgoing ``(v_plus, omega_plus)`` from the fitted spin equation with e(u_n).

    Port of the v0-branch ``spin_contact.predict_contact`` with the single F4 change:
    ``e_eff`` -> ``e(u_n) = g1 * exp(g2 * |u_n|)`` per env (clamped to [0.05, 0.95] so
    far-out-of-envelope contacts stay physical). All inputs ``(N, 3)``.
    """
    R = prm.ball_radius
    c = prm.inertia_coeff

    n = orient_normal(n, v_minus, v_r)
    r = -R * n

    u = v_minus + torch.cross(omega_minus, r, dim=-1) - v_r
    u_n_signed = torch.sum(u * n, dim=-1, keepdim=True)          # (N,1)
    u_t_vec = u - u_n_signed * n                                  # (N,3)
    u_t_mag = torch.linalg.norm(u_t_vec, dim=-1, keepdim=True)    # (N,1)
    u_n_abs = torch.abs(u_n_signed)                               # (N,1)

    # F4: velocity-dependent normal restitution (venue fit, valid u_n 1.4-7.2 m/s).
    e_eff = (prm.paddle_e_g1 * torch.exp(prm.paddle_e_g2 * u_n_abs)).clamp(0.05, 0.95)

    cos_theta = u_n_abs / (torch.hypot(u_t_mag, u_n_signed) + _EPS)
    raw = (prm.paddle_a_t + prm.paddle_b_t * cos_theta) * u_t_mag
    cap = prm.paddle_mu * (1.0 + e_eff) * u_n_abs
    s = torch.clamp(raw, min=0.0).minimum(cap)

    safe_dir = u_t_vec / (u_t_mag + _EPS)
    delta_v_t = torch.where(u_t_mag > _EPS, -s * safe_dir, torch.zeros_like(u_t_vec))

    delta_v_n = -(1.0 + e_eff) * u_n_signed * n
    delta_omega = -(1.0 / (c * R)) * torch.cross(n, delta_v_t, dim=-1)

    v_plus = v_minus + delta_v_n + delta_v_t
    omega_plus = omega_minus + delta_omega
    return v_plus, omega_plus


def flight_accel(v: torch.Tensor, omega: torch.Tensor, prm: VirtualBallParams) -> torch.Tensor:
    """a = g - k_d |v| v + k_m (omega x v). Inputs ``(N, 3)``."""
    speed = torch.linalg.norm(v, dim=-1, keepdim=True)
    a = -prm.k_d * speed * v + prm.k_m * torch.cross(omega, v, dim=-1)
    a[..., 2] -= prm.g
    return a


def rk4_step(
    p: torch.Tensor, v: torch.Tensor, omega: torch.Tensor, h: float, prm: VirtualBallParams
) -> tuple[torch.Tensor, torch.Tensor]:
    """One RK4 flight step (omega constant in flight, matching the fit)."""
    a1 = flight_accel(v, omega, prm)
    a2 = flight_accel(v + 0.5 * h * a1, omega, prm)
    a3 = flight_accel(v + 0.5 * h * a2, omega, prm)
    a4 = flight_accel(v + h * a3, omega, prm)
    v_new = v + (h / 6.0) * (a1 + 2 * a2 + 2 * a3 + a4)
    p_new = p + (h / 6.0) * (v + 2 * (v + 0.5 * h * a1) + 2 * (v + 0.5 * h * a2) + (v + h * a3))
    return p_new, v_new


def coarse_landing(
    p0: torch.Tensor,
    v0: torch.Tensor,
    omega0: torch.Tensor,
    prm: VirtualBallParams,
    surface_z: float,
    net_x: float,
    h: float = 0.01,
    n_steps: int = 100,
) -> dict[str, torch.Tensor]:
    """Fixed-length branch-free rollout: first descending table-plane crossing + net-plane crossing.

    Returns (all first-crossing values; envs that never cross keep valid=False):
      ``land_xy`` (N,2)  interpolated table-plane crossing point,
      ``land_valid`` (N) crossed the plane downward within the horizon,
      ``net_z`` (N)      ball height when first crossing the net plane in +x,
      ``net_valid`` (N)  crossed the net plane (in +x) BEFORE landing.

    The net collider itself is NOT simulated (v0 parity); the reward gates on the
    predicted clearance instead, so net-hitting shots score zero rather than bounce.
    """
    p, v = p0, v0
    landed = torch.zeros(p0.shape[0], dtype=torch.bool, device=p0.device)
    net_crossed = torch.zeros_like(landed)
    land_xy = torch.zeros(p0.shape[0], 2, device=p0.device)
    net_z = torch.zeros(p0.shape[0], device=p0.device)

    for _ in range(n_steps):
        p_new, v_new = rk4_step(p, v, omega0, h, prm)

        # net-plane crossing (+x through net_x), only counts before landing
        ncross = (~net_crossed) & (~landed) & (p[:, 0] < net_x) & (p_new[:, 0] >= net_x)
        fn = ((net_x - p[:, 0]) / (p_new[:, 0] - p[:, 0]).clamp(min=_EPS)).clamp(0.0, 1.0)
        z_at = p[:, 2] + (p_new[:, 2] - p[:, 2]) * fn
        net_z = torch.where(ncross, z_at, net_z)
        net_crossed = net_crossed | ncross

        # first downward table-plane crossing
        cross = (~landed) & (p[:, 2] > surface_z) & (p_new[:, 2] <= surface_z)
        fz = ((p[:, 2] - surface_z) / (p[:, 2] - p_new[:, 2]).clamp(min=_EPS)).clamp(0.0, 1.0)
        xy = p[:, :2] + (p_new[:, :2] - p[:, :2]) * fz.unsqueeze(-1)
        land_xy = torch.where(cross.unsqueeze(-1), xy, land_xy)
        landed = landed | cross

        p, v = p_new, v_new

    return {"land_xy": land_xy, "land_valid": landed, "net_z": net_z, "net_valid": net_crossed}
