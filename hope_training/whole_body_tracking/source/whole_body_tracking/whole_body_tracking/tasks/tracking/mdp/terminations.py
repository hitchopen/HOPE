"""Physical-fall terminations (deploy-honest: IMU tilt + root height only).

The continuous-rally episode resets on a fixed time-out (``mdp.time_out``, wired in the env config) or
a physical fall. A fall is defined from quantities the real robot has — base tilt (from the IMU) and
base height — never from the reference clip, so the same definition is meaningful on hardware.
"""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def base_tilted(env: ManagerBasedRLEnv, threshold: float, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """True when the base is tilted past ``threshold`` (horizontal component of projected gravity)."""
    asset: Articulation = env.scene[asset_cfg.name]
    return torch.norm(asset.data.projected_gravity_b[:, :2], dim=-1) > threshold


def base_too_low(env: ManagerBasedRLEnv, min_height: float, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """True when the base root height drops below ``min_height`` (the robot has fallen/collapsed)."""
    asset: Articulation = env.scene[asset_cfg.name]
    return asset.data.root_pos_w[:, 2] < min_height
