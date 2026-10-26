"""v12 actual-state whole-body imitation with a non-exponential error tail.

No actor Jacobian, action smoothing, or q_des-to-q auxiliary loss. The task
owns racket tracking near impact; all 29 active joints still have a soft prior.
Static HOLD is owned exclusively by the common-terminal HOLD reward.
"""
import torch

from .full_body_hold import GROUPS, rpy, wrap_angle
from .ready_recovery import _log


def tracking_score(error, scale):
    """Positive pseudo-Huber reciprocal; polynomial rather than exp(-e^2) tail."""
    x = error / scale
    cost = (torch.sqrt(1+x.square())-1).mean(-1)
    return 1/(1+cost)


def full_body_imitation(env):
    motion = env.command_manager.get_term("motion")
    cmd = env.command_manager.get_term("racket_target")
    robot = cmd.robot
    if motion.step_motion is None:
        raise ValueError("full_body_imitation requires the four-clip reference")
    cache = getattr(env, "_full_body_imitation_ids", None)
    if cache is None:
        cache = {group: robot.find_joints(names, preserve_order=True)[0] for group, names in GROUPS.items()}
        env._full_body_imitation_ids = cache
    reference = motion.joint_pos
    from .reference_recovery import base_reference, enabled, static_gate
    target, _ = base_reference(env, cmd)
    distance = (target-cmd.base_pos_w[:, :2]).norm(dim=-1)
    # Soften pose tracking during travel without paying a bonus for moving away.
    leg_factor = .35+.65*((.25-distance)/.20).clamp(0, 1)
    impact_free = ((cmd.time_to_strike.abs()-.12)/.25).clamp(0, 1)
    arm_factor = .15+.85*impact_free
    scores = []
    for group, ids in cache.items():
        error = robot.data.joint_pos[:, ids]-reference[:, ids]
        scale = .18 if group == "waist" else (.28 if group.endswith("leg") else .35)
        score = tracking_score(error, scale)
        factor = leg_factor if group.endswith("leg") else (arm_factor if group == "right_arm" else 1.)
        scores.append(score*factor)
        _log(cmd, "imitation_"+group+"_rms_rad", error.square().mean(-1).sqrt())
        _log(cmd, "imitation_"+group+"_score", score)
    # Joint angles alone cannot detect a whole robot leaning over. Track torso
    # roll/pitch independently in the table frame, while heading remains owned
    # by the existing heading objective. No reference translation/root locking.
    actual = rpy(motion.robot_anchor_quat_w)[:, :2]
    target = rpy(motion.anchor_quat_w)[:, :2]
    torso = tracking_score(wrap_angle(actual-target), .15)
    dynamic = 1-static_gate(motion, cmd.time_to_strike) if enabled(motion) else (~motion.in_hold).float()
    score = (.75*torch.stack(scores, -1).mean(-1)+.25*torso)*dynamic
    if getattr(getattr(motion, "cfg", None), "step_reference_mode", "") == "discrete_four_stroke":
        from .four_stroke import supervision_gate
        score = score*supervision_gate(motion, cmd)
    _log(cmd, "imitation_step_weight", motion.step_reference_weight())
    _log(cmd, "imitation_torso_rp_error_rad", wrap_angle(actual-target).norm(dim=-1))
    _log(cmd, "imitation_score", score)
    return score
