"""Vectorized YAML-selected paddle contact and RK4 flight to first table bounce.

PPQ004 includes drag, gravity, Magnus and capped tangential impulse/spin transfer.
Landing is the first descending ball-center crossing of surface + radius.
"""

from __future__ import annotations

import os
from dataclasses import dataclass

import torch
import importlib.util
from pathlib import Path

_profile_spec = importlib.util.spec_from_file_location(
    "hope_virtual_ball_profile", Path(__file__).resolve().parents[6] / "scripts/ball_physics_profile.py")
_profile = importlib.util.module_from_spec(_profile_spec)
_profile_spec.loader.exec_module(_profile)

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
    return_model: str = "legacy_venue"
    restitution_min: float | None = None
    restitution_max: float | None = None
    integrate_dt_s: float = 0.01
    max_predict_time_s: float = 1.0
    net_clearance_margin: float = 0.0
    numeric_epsilon: float = _EPS

    def __post_init__(self):
        if self.restitution_min is None or self.restitution_max is None:
            model = _profile.load_profile(Path(__file__).resolve().parents[8] / _profile.DEFAULT_PROFILE)["return_model"]
            object.__setattr__(self, "restitution_min", float(model["restitution_min"]))
            object.__setattr__(self, "restitution_max", float(model["restitution_max"]))


def default_venue_yaml_path() -> str:
    """Resolve the default PPQ004 profile or an explicit environment override."""
    return str(_profile.default_profile_path())


def load_venue_params(path: str | None = None) -> VirtualBallParams:
    path = path or default_venue_yaml_path()
    raw = _profile.load_profile(path)
    model = raw.get("return_model", {})
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
        return_model=model.get("id", "legacy_venue"),
        restitution_min=model.get("restitution_min"),
        restitution_max=model.get("restitution_max"),
        integrate_dt_s=float(model.get("integrate_dt_s", 0.01)),
        max_predict_time_s=float(model.get("max_predict_time_s", 1.0)),
        net_clearance_margin=float(model.get("net_clearance_margin", 0.0)),
        numeric_epsilon=float(model.get("numeric_epsilon", _EPS)),
    )


def _normalize(v: torch.Tensor, eps: float = _EPS) -> torch.Tensor:
    return v / (torch.linalg.norm(v, dim=-1, keepdim=True) + eps)


def orient_normal(n: torch.Tensor, v_minus: torch.Tensor, v_r: torch.Tensor, eps: float = _EPS) -> torch.Tensor:
    """Orient ``n`` per env so the incoming relative velocity approaches the face.

    Same as the v0 branch ``spin_contact.orient_normal``: flip where ``(v_minus - v_r) . n > 0``.
    The returned normal points from the face toward the incoming-ball side.
    """
    n = _normalize(n, eps)
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

    eps = prm.numeric_epsilon
    n = orient_normal(n, v_minus, v_r, eps)
    r = -R * n

    u = v_minus + torch.cross(omega_minus, r, dim=-1) - v_r
    u_n_signed = torch.sum(u * n, dim=-1, keepdim=True)          # (N,1)
    u_t_vec = u - u_n_signed * n                                  # (N,3)
    u_t_mag = torch.linalg.norm(u_t_vec, dim=-1, keepdim=True)    # (N,1)
    u_n_abs = torch.abs(u_n_signed)                               # (N,1)

    # F4: velocity-dependent normal restitution (venue fit, valid u_n 1.4-7.2 m/s).
    e_eff = (prm.paddle_e_g1 * torch.exp(prm.paddle_e_g2 * u_n_abs)).clamp(prm.restitution_min, prm.restitution_max)

    cos_theta = u_n_abs / (torch.hypot(u_t_mag, u_n_signed) + eps)
    raw = (prm.paddle_a_t + prm.paddle_b_t * cos_theta) * u_t_mag
    cap = prm.paddle_mu * (1.0 + e_eff) * u_n_abs
    s = torch.clamp(raw, min=0.0).minimum(cap)

    safe_dir = u_t_vec / (u_t_mag + eps)
    delta_v_t = torch.where(u_t_mag > eps, -s * safe_dir, torch.zeros_like(u_t_vec))

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
    h: float | None = None,
    n_steps: int | None = None,
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
    import math
    h = prm.integrate_dt_s if h is None else h
    reverse_legacy = h < 0 and n_steps is not None and prm.return_model == "legacy_venue"
    if h == 0 or (h < 0 and not reverse_legacy):
        raise ValueError("Full return flight requires a positive step")
    n_steps = math.ceil(prm.max_predict_time_s / h) if n_steps is None else n_steps
    if n_steps < 1:
        raise ValueError("Flight horizon must be positive")
    if prm.return_model == "full_ppq004_first_bounce_v1":
        surface_z += prm.ball_radius
    p, v = p0, v0
    eligible = (torch.isfinite(p0).all(dim=-1) & torch.isfinite(v0).all(dim=-1)
                & torch.isfinite(omega0).all(dim=-1))
    if prm.return_model == "full_ppq004_first_bounce_v1":
        eligible &= p0[:, 2] > surface_z
    landed = torch.zeros(p0.shape[0], dtype=torch.bool, device=p0.device)
    net_crossed = torch.zeros_like(landed)
    land_xy = torch.zeros(p0.shape[0], 2, device=p0.device, dtype=p0.dtype)
    net_z = torch.zeros(p0.shape[0], device=p0.device, dtype=p0.dtype)

    net_y = torch.zeros_like(net_z)

    for _ in range(n_steps):
        p_new, v_new = rk4_step(p, v, omega0, h, prm)

        cross = eligible & (~landed) & (p[:, 2] > surface_z) & (p_new[:, 2] <= surface_z)
        fz = ((p[:, 2] - surface_z) / (p[:, 2] - p_new[:, 2]).clamp(min=_EPS)).clamp(0.0, 1.0)
        # Net events after the first table crossing, even within this step, do not count.
        ncross = eligible & (~net_crossed) & (~landed) & (p[:, 0] < net_x) & (p_new[:, 0] >= net_x)
        fn = ((net_x - p[:, 0]) / (p_new[:, 0] - p[:, 0]).clamp(min=_EPS)).clamp(0.0, 1.0)
        ncross &= (~cross) | (fn <= fz)
        z_at = p[:, 2] + (p_new[:, 2] - p[:, 2]) * fn
        net_z = torch.where(ncross, z_at, net_z)
        y_at = p[:, 1] + (p_new[:, 1] - p[:, 1]) * fn
        net_y = torch.where(ncross, y_at, net_y)
        net_crossed = net_crossed | ncross

        # first downward table-plane crossing
        cross = eligible & (~landed) & (p[:, 2] > surface_z) & (p_new[:, 2] <= surface_z)
        fz = ((p[:, 2] - surface_z) / (p[:, 2] - p_new[:, 2]).clamp(min=_EPS)).clamp(0.0, 1.0)
        xy = p[:, :2] + (p_new[:, :2] - p[:, :2]) * fz.unsqueeze(-1)
        land_xy = torch.where(cross.unsqueeze(-1), xy, land_xy)
        landed = landed | cross

        p, v = p_new, v_new

    return {"land_xy": land_xy, "land_valid": landed, "net_z": net_z, "net_y": net_y, "net_valid": net_crossed}


class CoarseLandingPredictor:
    """Replay the unchanged RK4 tensor operations in one CUDA graph submission.

    One instance belongs to one command term. Shape, device and physics parameter
    changes rebuild the graph. Outputs own their storage, so retained metrics and
    checkpoint state cannot be overwritten by the next replay. CPU/autograd uses
    the original eager function; no graph is serialized into a checkpoint.
    """

    def __init__(self):
        self._key = None
        self._graph = None

    def __call__(self, p0, v0, omega0, prm, surface_z, net_x, h=None, n_steps=None):
        inputs = (p0, v0, omega0)
        if (p0.device.type != "cuda" or os.environ.get("HOPE_DISABLE_BALL_CUDA_GRAPH") == "1"
                or (torch.is_grad_enabled() and any(t.requires_grad for t in inputs))):
            return coarse_landing(*inputs, prm, surface_z, net_x, h, n_steps)
        key = (tuple((tuple(t.shape), t.dtype, t.device) for t in inputs),
               prm, surface_z, net_x, h, n_steps)
        with torch.cuda.device(p0.device):
            if key != self._key:
                # A side stream is required for warmup and CUDA graph capture.
                static_inputs = tuple(t.clone() for t in inputs)
                current = torch.cuda.current_stream()
                stream = torch.cuda.Stream()
                stream.wait_stream(current)
                with torch.cuda.stream(stream):
                    for _ in range(2):
                        coarse_landing(*static_inputs, prm, surface_z, net_x, h, n_steps)
                current.wait_stream(stream)
                graph = torch.cuda.CUDAGraph()
                with torch.cuda.graph(graph, stream=stream):
                    outputs = coarse_landing(*static_inputs, prm, surface_z, net_x, h, n_steps)
                current.wait_stream(stream)
                self._inputs, self._outputs = static_inputs, outputs
                self._graph, self._key = graph, key
            for target, source in zip(self._inputs, inputs):
                target.copy_(source)
            self._graph.replay()
            return {name: value.clone() for name, value in self._outputs.items()}
