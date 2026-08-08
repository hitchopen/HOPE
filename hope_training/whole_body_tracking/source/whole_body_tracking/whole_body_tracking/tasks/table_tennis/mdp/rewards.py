"""Reward terms for the table-tennis environment.

Only a small example ball-aware term lives here; the generic robot rewards (alive, action-rate, ...) are
reused from ``isaaclab.envs.mdp``. Add real match objectives (return success, ball-over-net, landing in
the opponent half, racket-to-ball tracking) here as the policy is developed.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def ball_above_surface(
    env: "ManagerBasedRLEnv",
    asset_cfg: SceneEntityCfg = SceneEntityCfg("ball"),
) -> torch.Tensor:
    """1.0 while the ball is above the table surface (HOPE z > 0), else 0.0. Shape ``(N,)``.

    A placeholder "ball in play" signal demonstrating how to read ball state in the HOPE frame
    (subtract the per-environment origin) for reward shaping."""
    ball: RigidObject = env.scene[asset_cfg.name]
    z_hope = ball.data.root_pos_w[:, 2] - env.scene.env_origins[:, 2]
    return (z_hope > 0.0).float()
