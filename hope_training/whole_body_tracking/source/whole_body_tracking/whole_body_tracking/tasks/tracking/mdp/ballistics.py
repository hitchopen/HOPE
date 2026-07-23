"""Small gravity-only ballistic helpers for racket target generation."""

from __future__ import annotations

import torch

GRAVITY = 9.81


def ballistic_velocity_from_landing(
    p0: torch.Tensor,
    land_xy: torch.Tensor,
    flight_time: torch.Tensor,
    surface_z: float,
) -> torch.Tensor:
    """Return gravity-only velocity that lands at ``land_xy, surface_z`` after ``flight_time``."""
    t = flight_time.clamp_min(1.0e-3)
    vx = (land_xy[:, 0] - p0[:, 0]) / t
    vy = (land_xy[:, 1] - p0[:, 1]) / t
    vz = (float(surface_z) - p0[:, 2] + 0.5 * GRAVITY * t**2) / t
    return torch.stack((vx, vy, vz), dim=-1)


def ballistic_z_at_x(p0: torch.Tensor, vel: torch.Tensor, x: float) -> torch.Tensor:
    """Return gravity-only height when the trajectory reaches env-local x."""
    t = (float(x) - p0[:, 0]) / vel[:, 0].clamp_min(1.0e-3)
    return p0[:, 2] + vel[:, 2] * t - 0.5 * GRAVITY * t**2
