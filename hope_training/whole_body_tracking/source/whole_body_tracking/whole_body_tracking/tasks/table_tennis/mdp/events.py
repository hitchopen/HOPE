"""Event terms for the table-tennis environment (scripted no-spin ball serve on reset)."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg

from ..geometry import ServeConfig

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def _uniform(n: int, rng: tuple[float, float], device: torch.device) -> torch.Tensor:
    return torch.empty(n, device=device).uniform_(rng[0], rng[1])


def reset_ball_serve(
    env: "ManagerBasedRLEnv",
    env_ids: torch.Tensor,
    serve_cfg: ServeConfig = ServeConfig(),
    asset_cfg: SceneEntityCfg = SceneEntityCfg("ball"),
) -> None:
    """Place the ball over the P2 half of the table and launch it toward the P1-side robot.

    Positions/velocities are sampled in the world frame and offset by each environment's origin, so the
    serve is identical in every court's local frame. The ball carries no angular velocity (no-spin model).
    """
    ball: RigidObject = env.scene[asset_cfg.name]
    n = len(env_ids)
    device = env.device

    pos = torch.stack(
        [
            _uniform(n, serve_cfg.pos_x_range, device),
            _uniform(n, serve_cfg.pos_y_range, device),
            _uniform(n, serve_cfg.pos_z_range, device),
        ],
        dim=1,
    )
    pos = pos + env.scene.env_origins[env_ids]

    quat = torch.zeros(n, 4, device=device)
    quat[:, 0] = 1.0  # identity orientation (w, x, y, z)

    lin_vel = torch.stack(
        [
            _uniform(n, serve_cfg.vel_x_range, device),
            _uniform(n, serve_cfg.vel_y_range, device),
            _uniform(n, serve_cfg.vel_z_range, device),
        ],
        dim=1,
    )
    ang_vel = torch.zeros(n, 3, device=device)  # no spin

    pose = torch.cat([pos, quat], dim=1)
    velocity = torch.cat([lin_vel, ang_vel], dim=1)
    ball.write_root_pose_to_sim(pose, env_ids=env_ids)
    ball.write_root_velocity_to_sim(velocity, env_ids=env_ids)
