"""Domain-randomization events not provided by ``isaaclab.envs.mdp``."""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING, Literal

import isaaclab.utils.math as math_utils
from isaaclab.assets import Articulation
from isaaclab.envs.mdp.events import _randomize_prop_by_op
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv


def _asset_joint_names(asset: Articulation) -> list[str]:
    names = getattr(asset.data, "joint_names", None)
    if names is None:
        names = getattr(asset, "joint_names", None)
    if names is None:
        raise RuntimeError("joint default randomization requires the articulation's resolved joint names")
    return list(names)


def _action_columns_for_asset_joints(
    action_term,
    asset: Articulation,
    joint_ids: torch.Tensor | slice,
) -> torch.Tensor:
    """Map articulation joint ids to action columns by exact joint name."""
    action_joint_names = getattr(action_term, "_joint_names", None)
    if action_joint_names is None:
        raise RuntimeError("joint default randomization requires the action term's resolved joint names")
    action_index = {name: i for i, name in enumerate(action_joint_names)}
    asset_names = _asset_joint_names(asset)
    if isinstance(joint_ids, slice):
        selected_names = asset_names[joint_ids]
    else:
        selected_names = [asset_names[int(i)] for i in joint_ids.detach().cpu().tolist()]
    missing = [name for name in selected_names if name not in action_index]
    if missing:
        raise RuntimeError(
            f"joint default randomization selected joints that are not driven by the joint_pos action: {missing}"
        )
    return torch.tensor([action_index[name] for name in selected_names], dtype=torch.long, device=asset.device)


def randomize_joint_default_pos(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | None,
    asset_cfg: SceneEntityCfg,
    pos_distribution_params: tuple[float, float] | None = None,
    operation: Literal["add", "scale", "abs"] = "abs",
    distribution: Literal["uniform", "log_uniform", "gaussian"] = "uniform",
):
    """Randomize joint default positions (encoder-calibration offsets from the URDF)."""
    asset: Articulation = env.scene[asset_cfg.name]
    asset.data.default_joint_pos_nominal = torch.clone(asset.data.default_joint_pos[0])

    if env_ids is None:
        env_ids = torch.arange(env.scene.num_envs, device=asset.device)

    if asset_cfg.joint_ids == slice(None):
        joint_ids = slice(None)
    else:
        joint_ids = torch.tensor(asset_cfg.joint_ids, dtype=torch.long, device=asset.device)

    if pos_distribution_params is not None:
        pos = asset.data.default_joint_pos.to(asset.device).clone()
        pos = _randomize_prop_by_op(
            pos, pos_distribution_params, env_ids, joint_ids, operation=operation, distribution=distribution
        )[env_ids][:, joint_ids]
        asset_env_ids = env_ids[:, None] if not isinstance(joint_ids, slice) else env_ids
        asset.data.default_joint_pos[asset_env_ids, joint_ids] = pos

        # ``joint_ids`` are articulation columns. The HOPE action term may expose the same joints in
        # deploy-canonical order, so its ``_offset`` must be scattered by action column, not by
        # articulation id.
        action_term = env.action_manager.get_term("joint_pos")
        action_cols = _action_columns_for_asset_joints(action_term, asset, joint_ids)
        action_env_ids = env_ids[:, None]
        action_term._offset[action_env_ids, action_cols] = pos


def randomize_rigid_body_com(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | None,
    com_range: dict[str, tuple[float, float]],
    asset_cfg: SceneEntityCfg,
):
    """Randomize the center of mass of selected rigid bodies (adds a per-axis uniform offset)."""
    asset: Articulation = env.scene[asset_cfg.name]
    if env_ids is None:
        env_ids = torch.arange(env.scene.num_envs, device="cpu")
    else:
        env_ids = env_ids.cpu()

    if asset_cfg.body_ids == slice(None):
        body_ids = torch.arange(asset.num_bodies, dtype=torch.int, device="cpu")
    else:
        body_ids = torch.tensor(asset_cfg.body_ids, dtype=torch.int, device="cpu")

    range_list = [com_range.get(key, (0.0, 0.0)) for key in ("x", "y", "z")]
    ranges = torch.tensor(range_list, device="cpu")
    rand_samples = math_utils.sample_uniform(ranges[:, 0], ranges[:, 1], (len(env_ids), 3), device="cpu").unsqueeze(1)

    coms = asset.root_physx_view.get_coms().clone()
    coms[:, body_ids, :3] += rand_samples
    asset.root_physx_view.set_coms(coms, env_ids)
