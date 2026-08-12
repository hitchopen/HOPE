"""Ping-pong ball aerodynamics model.

PhysX already integrates gravity and resolves rigid-body contacts (table / net / floor / racket) with
restitution + friction. What PhysX does **not** model is aerodynamics: a 40 mm ball is light enough that
air drag noticeably bends its flight. This module supplies that missing force so the simulated flight
matches the model the HOPE planner was calibrated against.

Drag model (matches ``hope_planner.constants.BallPhysics`` / the planner's flight integrator):

    a_drag = -k * |v| * v          (k = 0.5 s/m, fit from recorded trajectories)
    F_drag = m * a_drag = -m * k * |v| * v

The planner **neglects spin**, so the Magnus (lift) term is **off by default**. It is provided as an
opt-in extension for higher-fidelity flight:

    a_magnus = k_magnus * (omega x v)
    F_magnus = m * k_magnus * (omega x v)

Everything here is expressed in the **world frame** and is pure ``torch`` (no Isaac imports) so it can be
unit-tested without a simulator. The environment is responsible for reading the ball state, calling
:func:`compute_aero_wrench`, rotating the wrench into the body frame if its physics API expects
body-frame forces, and writing it to the sim each physics substep.
"""

from __future__ import annotations

from dataclasses import dataclass

import torch


@dataclass
class BallAerodynamicsCfg:
    """Configuration for the per-substep ball aerodynamic force field."""

    enabled: bool = True
    """Master switch. If False, the ball flies on PhysX gravity + contacts alone (still a valid scene)."""

    drag_coefficient: float = 0.5
    """Quadratic drag coefficient ``k`` (s/m). a_drag = -k|v|v. HOPE-calibrated default is 0.5."""

    magnus_coefficient: float = 0.0
    """Magnus/lift coefficient ``k_magnus``. a_magnus = k_magnus (omega x v). 0 disables spin lift
    (the planner neglects spin; enable only for higher-fidelity flight studies)."""

    linear_velocity_clip: float = 50.0
    """Safety clip on |v| (m/s) used when computing drag, so a numerical blowup can't inject huge forces."""


def compute_aero_wrench(
    lin_vel_w: torch.Tensor,
    ang_vel_w: torch.Tensor,
    mass: float,
    cfg: BallAerodynamicsCfg,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Aerodynamic force (and zero torque) on the ball, in the **world frame**.

    Args:
        lin_vel_w: ``(N, 3)`` ball linear velocity in the world frame (m/s).
        ang_vel_w: ``(N, 3)`` ball angular velocity in the world frame (rad/s).
        mass: ball mass (kg).
        cfg: aerodynamics configuration.

    Returns:
        ``(force_w, torque_w)``, each ``(N, 3)`` in the world frame. Torque is always zero: this model
        adds no aerodynamic spin-decay term, so any served spin is non-decaying during flight (the ball
        rigid body uses ``angular_damping = 0``). The torque slot exists only for a uniform wrench
        interface.
    """
    speed_raw = torch.linalg.norm(lin_vel_w, dim=-1, keepdim=True)
    speed = torch.clamp(speed_raw, max=cfg.linear_velocity_clip)
    # Velocity vector rescaled to the clipped magnitude, so the drag *force* is bounded by
    # m * k * clip^2 (clamping |v| alone would only bound one of the two |v| factors).
    vel_clipped = lin_vel_w * (speed / torch.clamp(speed_raw, min=1e-8))

    # Quadratic drag, opposing velocity.
    force = -mass * cfg.drag_coefficient * speed * vel_clipped

    # Optional Magnus lift from spin (uses the true, unclipped velocity).
    if cfg.magnus_coefficient != 0.0:
        force = force + mass * cfg.magnus_coefficient * torch.cross(ang_vel_w, lin_vel_w, dim=-1)

    torque = torch.zeros_like(force)
    return force, torque
