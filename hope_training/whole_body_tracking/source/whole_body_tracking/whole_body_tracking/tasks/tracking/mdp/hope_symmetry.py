"""V15 lower-body left/right symmetry for RSL-RL.

HUGWBC uses a mirror loss to regularize locomotion.  HOPE's forehand/backhand racket motions are
not themselves mirror images, so this transform is deliberately active only during the finite
pre-swing STAND/STEP phase and only mirrors the waist/legs plus their locomotion commands.  During
the racket swing (``locomotion_mode == -1``) both the duplicated observation and action are exact
identities, making the mirror loss zero instead of corrupting HITTER's asymmetric strike policy.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch
from tensordict import TensorDict

from whole_body_tracking.tasks.tracking.actor_observation_contract import HITTER_PURE_V15

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

__all__ = ["compute_hitter_v15_lower_body_symmetric_states"]


def _term_slices() -> dict[str, slice]:
    start = 0
    result = {}
    for term in HITTER_PURE_V15.terms:
        result[term.name] = slice(start, start + term.dim)
        start += term.dim
    return result


_SLICES = _term_slices()


def _lower_body_mirror_map(joint_names: list[str]) -> tuple[list[int], list[int], list[float]]:
    """Return destination/source/sign lists for A3 waist and bilateral leg joints."""
    index = {name: i for i, name in enumerate(joint_names)}
    dst: list[int] = []
    src: list[int] = []
    signs: list[float] = []

    for side, other in (("left_", "right_"), ("right_", "left_")):
        for i, name in enumerate(joint_names):
            if not name.startswith(side):
                continue
            if not any(part in name for part in ("_hip_", "_knee_", "_ankle_")):
                continue
            partner = other + name[len(side) :]
            if partner not in index:
                raise RuntimeError(
                    f"V15 lower-body symmetry cannot find partner {partner!r} for {name!r}"
                )
            dst.append(i)
            src.append(index[partner])
            signs.append(-1.0 if ("_roll_" in name or "_yaw_" in name) else 1.0)

    for name in ("waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"):
        if name not in index:
            continue
        dst.append(index[name])
        src.append(index[name])
        signs.append(-1.0 if ("yaw" in name or "roll" in name) else 1.0)

    return dst, src, signs


def _mirror_joint_tensor(
    values: torch.Tensor, joint_names: list[str], active: torch.Tensor
) -> torch.Tensor:
    mirrored = values.clone()
    dst, src, signs = _lower_body_mirror_map(joint_names)
    if dst:
        dst_t = torch.tensor(dst, dtype=torch.long, device=values.device)
        src_t = torch.tensor(src, dtype=torch.long, device=values.device)
        sign_t = torch.tensor(signs, dtype=values.dtype, device=values.device)
        candidate = values.index_select(-1, src_t) * sign_t
        rows = torch.where(active)[0]
        if rows.numel() > 0:
            mirrored[rows[:, None], dst_t[None, :]] = candidate[rows]
    return mirrored


def _action_joint_names(env) -> list[str]:
    base_env = env.unwrapped
    term = base_env.action_manager.get_term("joint_pos")
    names = list(getattr(term, "_joint_names", ()))
    if len(names) != 31:
        raise RuntimeError(
            f"V15 symmetry requires the 31-D A3 joint action order, got {len(names)} names"
        )
    return names


@torch.no_grad()
def compute_hitter_v15_lower_body_symmetric_states(
    env: ManagerBasedRLEnv,
    obs: TensorDict | None = None,
    actions: torch.Tensor | None = None,
):
    """Return original+left/right-mirrored batches for V15's locomotion phase."""
    base_env = env.unwrapped
    joint_names = _action_joint_names(env)
    active = None

    if obs is not None:
        policy = obs["policy"]
        if policy.shape[-1] != HITTER_PURE_V15.total_dim:
            raise RuntimeError(
                "V15 symmetry observation width mismatch: "
                f"expected {HITTER_PURE_V15.total_dim}, got {policy.shape[-1]}"
            )
        mode = policy[:, _SLICES["locomotion_mode"]].squeeze(-1)
        active = mode >= -0.5
        base_env._hope_v15_symmetry_batch_mask = active

        mirrored = policy.clone()

        def signed(term: str, signs: tuple[float, ...]) -> None:
            section = mirrored[:, _SLICES[term]]
            factor = torch.tensor(signs, device=policy.device, dtype=policy.dtype)
            section[active] = section[active] * factor

        signed("base_ang_vel", (-1.0, 1.0, -1.0))
        signed("projected_gravity", (1.0, -1.0, 1.0))
        signed("base_forward_xy", (1.0, -1.0))
        signed("base_target_delta_xy", (1.0, -1.0))
        signed("base_velocity_xy", (1.0, -1.0))
        signed("desired_lateral_velocity", (-1.0,))

        for term in ("joint_pos", "joint_vel", "actions"):
            values = policy[:, _SLICES[term]]
            mirrored[:, _SLICES[term]] = _mirror_joint_tensor(
                values, joint_names, active
            )

        clocks = policy[:, _SLICES["gait_clock"]]
        mirrored[active, _SLICES["gait_clock"]] = clocks[active].flip(-1)

        batch_size = obs.batch_size[0]
        obs_aug = obs.repeat(2)
        obs_aug["policy"][:batch_size] = policy
        obs_aug["policy"][batch_size:] = mirrored
    else:
        obs_aug = None

    if actions is not None:
        if active is None:
            active = getattr(base_env, "_hope_v15_symmetry_batch_mask", None)
        if active is None or active.shape[0] != actions.shape[0]:
            raise RuntimeError(
                "V15 symmetry action transform requires the matching observation batch mask"
            )
        mirrored_actions = _mirror_joint_tensor(actions, joint_names, active)
        actions_aug = torch.cat((actions, mirrored_actions), dim=0)
    else:
        actions_aug = None

    return obs_aug, actions_aug
