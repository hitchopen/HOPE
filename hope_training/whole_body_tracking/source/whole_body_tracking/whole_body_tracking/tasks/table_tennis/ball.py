"""Ping-pong ball aerodynamics — no-spin quadratic drag.

PhysX already integrates gravity and resolves rigid-body contacts (table / net / floor / racket) with
restitution + friction. What PhysX does **not** model is aerodynamics: a 40 mm ball is light enough that
air drag noticeably bends its flight. This module supplies that missing force.

The model is **no-spin**: the only aerodynamic force is quadratic drag opposing the velocity,

    a_drag = -k * |v| * v          (k in 1/m, from configs/ball_physics.yaml)
    F_drag = m * a_drag = -m * k * |v| * v

There is no spin, angular velocity, or Magnus/lift term anywhere in this model. Everything is expressed
in the **world frame** and is pure ``torch`` (no Isaac imports) so it can be unit-tested without a
simulator. The environment reads the ball velocity, calls :func:`compute_drag_force`, rotates the force
into the body frame if its physics API expects body-frame forces, and writes it to the sim each physics
substep.
"""

from __future__ import annotations

from dataclasses import dataclass

import torch

from .physics_config import load_ball_physics


@dataclass
class BallAerodynamicsCfg:
    """Configuration for the per-substep no-spin drag force field."""

    enabled: bool = True
    """Master switch. If False, the ball flies on PhysX gravity + contacts alone (still a valid scene)."""

    drag_coefficient: float = 0.1261
    """Quadratic drag coefficient ``k`` (1/m). a_drag = -k|v|v. Overridden by ``configs/ball_physics.yaml``
    (``drag.k``) via :meth:`from_physics_config`."""

    linear_velocity_clip: float = 50.0
    """Safety clip on |v| (m/s) used when computing drag, so a numerical blowup can't inject huge forces."""

    @classmethod
    def from_physics_config(cls, enabled: bool = True) -> "BallAerodynamicsCfg":
        """Build a config with the drag coefficient / clip read from ``configs/ball_physics.yaml``."""
        drag = load_ball_physics()["drag"]
        return cls(
            enabled=enabled,
            drag_coefficient=float(drag["k"]),
            linear_velocity_clip=float(drag["velocity_clip"]),
        )


def compute_drag_force(
    lin_vel_w: torch.Tensor,
    mass: float,
    cfg: BallAerodynamicsCfg,
) -> torch.Tensor:
    """Aerodynamic drag force on the ball, in the **world frame**. Shape ``(N, 3)``.

    Args:
        lin_vel_w: ``(N, 3)`` ball linear velocity in the world frame (m/s).
        mass: ball mass (kg).
        cfg: aerodynamics configuration.

    Returns:
        ``(N, 3)`` world-frame drag force (N), opposing the velocity. No spin / Magnus term exists.
    """
    speed_raw = torch.linalg.norm(lin_vel_w, dim=-1, keepdim=True)
    speed = torch.clamp(speed_raw, max=cfg.linear_velocity_clip)
    # Velocity vector rescaled to the clipped magnitude, so the drag *force* is bounded by
    # m * k * clip^2 (clamping |v| alone would only bound one of the two |v| factors).
    vel_clipped = lin_vel_w * (speed / torch.clamp(speed_raw, min=1e-8))

    # Quadratic drag, opposing velocity.
    return -mass * cfg.drag_coefficient * speed * vel_clipped
