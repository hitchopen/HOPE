"""Actual-q hard-stop clearance cost. Independent of PD offsets and support load."""
from __future__ import annotations

import math
import torch


def limit_clearance_cost(q, hard_limits, margin):
    """Zero inside the buffer; quadratic approach and linear, non-saturating overrun.

    At the hard stop the per-joint debt is 1. A small physical overshoot keeps
    increasing it; clipping q or bounding the debt at the stop would hide this.
    Parameters/asset limits are validated once by the environment wrapper.
    """
    distance = torch.minimum(q-hard_limits[..., 0], hard_limits[..., 1]-q)
    penetration = (1-distance/margin).clamp_min(0)
    debt = torch.where(penetration <= 1, penetration.square(), 2*penetration-1)
    return debt, distance


def evaluate_actual_joint_limit(env, margin_rad=0.0872664626, waist_margin_rad=0.1396263402,
                                hold_recover_multiplier=3.0):
    from .task_pose_reference import phase_gate

    if (not all(math.isfinite(x) for x in (margin_rad, waist_margin_rad, hold_recover_multiplier))
            or min(margin_rad, waist_margin_rad) <= 0 or hold_recover_multiplier < 1):
        raise ValueError("invalid actual hard-limit clearance parameters")
    action = env.action_manager.get_term("joint_pos")
    robot = action._asset
    cmd = env.command_manager.get_term("racket_target")
    motion = env.command_manager.get_term("motion")
    key = (margin_rad, waist_margin_rad)
    cache = getattr(env, "_actual_joint_limit_cache", None)
    if cache is None or cache[0] != key:
        names = list(action._joint_names)
        passive = set(action._passive_action_cols.detach().cpu().tolist())
        columns = [i for i in range(len(names)) if i not in passive]
        active_names = [names[i] for i in columns]
        waist = [active_names.index(f"waist_{a}_joint") for a in ("yaw", "roll", "pitch")]
        hard = action._select_action_joints(robot.data.joint_pos_limits)[:, columns].detach().clone()
        span = hard[..., 1]-hard[..., 0]
        if not bool(torch.isfinite(hard).all() and (span > 0).all()):
            raise ValueError("actual hard-limit reward requires finite official joint_pos_limits")
        requested = hard.new_full((len(columns),), margin_rad)
        requested[waist] = waist_margin_rad
        # Preserve an interior region for narrow-range joints. Never substitute
        # decoder q_des limits or soft limits for the plant's hard joint limits.
        margin = torch.minimum(requested[None, :], .2*span)
        cache = (key, columns, waist, hard, margin)
        env._actual_joint_limit_cache = cache
    _, columns, waist, hard, margin = cache
    actual = action._select_action_joints(robot.data.joint_pos)[:, columns]
    per_joint, distance = limit_clearance_cost(actual, hard, margin)
    ready_gate, phase = phase_gate(motion.in_hold, cmd.time_to_strike)
    multiplier = 1+(hold_recover_multiplier-1)*ready_gate
    # A single rail-pinned joint must not disappear in a mean over 29 joints.
    # Waist at a stop has full debt; another single joint at a stop has half.
    debt = (.5*per_joint.amax(-1)+.5*per_joint[:, waist].amax(-1))*multiplier
    metrics = {
        "valid": torch.ones_like(debt), "phase": phase.float(), "debt": debt,
        "phase_multiplier": multiplier, "min_clearance_rad": distance.amin(-1),
        "near_joint_count": (distance < margin).sum(-1).float(),
        "hard_exceedance_rad": (-distance.amin(-1)).clamp_min(0),
        "waist_min_clearance_rad": distance[:, waist].amin(-1),
        "waist_near": (distance[:, waist] < margin[:, waist]).any(-1).float(),
    }
    pitch = waist[2]
    metrics.update({"waist_pitch_lower_clearance_rad": actual[:, pitch]-hard[:, pitch, 0],
                    "waist_pitch_upper_clearance_rad": hard[:, pitch, 1]-actual[:, pitch],
                    "waist_pitch_buffer_rad": margin[:, pitch],
                    "waist_pitch_debt": per_joint[:, pitch]*multiplier})
    for key, value in metrics.items():
        name = "actual_limit_"+key
        if name not in cmd.metrics:
            cmd.metrics[name] = value.detach().clone()
        else:
            cmd.metrics[name].copy_(value.detach())
    return debt
