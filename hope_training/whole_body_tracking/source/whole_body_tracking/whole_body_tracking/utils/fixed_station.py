"""Fixed commanded station, with four demonstrated strokes and physical steps."""
import torch


def reach_style_weight(contact, clip, standing, stepping):
    """Observable continuous projection onto each side's two contact exemplars.

    No randomized style label and no dependence on the measured drifting base.
    The backhand stepping clip is lower and slightly right of its standing
    exemplar, so a generic 'far left means step' rule would label it incorrectly.
    """
    first = contact.new_tensor(standing)[clip]
    direction = contact.new_tensor(stepping)[clip] - first
    u = ((contact-first)*direction).sum(-1) / direction.square().sum(-1).clamp_min(1e-8)
    u = u.clamp(0, 1)
    return u.square()*(3-2*u)


def foot_reference_debt(error, margin=.015, std=.06):
    # Huber tail has a nonvanishing state-score slope for accumulated drift.
    x = ((error.norm(dim=-1)-margin)/std).clamp_min(0)
    cost = torch.where(x < 1, .5*x.square(), x-.5)
    return .5*cost.mean(-1)+.5*cost.amax(-1)


def fixed_station_feet(env, start_steps=0, ramp_steps=64000, clearance_std=0., yaw_std=0.,
                       persistent_home=False, position_fraction=.60):
    """Track the demonstrated step, then recover BOTH feet to original home.

    Reference is expressed in the immutable environment frame. Neither a wrap,
    next-ball interruption nor physical slip can recenter the foot goals.
    Reward acts on position only, never suppressing corrective joint velocities.
    """
    motion = env.command_manager.get_term("motion")
    cmd = env.command_manager.get_term("racket_target")
    cache = getattr(env, "_fixed_station_foot_indices", None)
    if cache is None:
        names = ["left_ankle_roll_Link", "right_ankle_roll_Link"]
        ids, found = cmd.robot.find_bodies(names, preserve_order=True)
        if list(found) != names:
            raise ValueError("fixed station requires both named feet")
        ref_ids = [motion.cfg.body_names.index(name) for name in names]
        cache = (ids, ref_ids)
        env._fixed_station_foot_indices = cache
    ids, ref_ids = cache
    reference = motion.body_pos_w[:, ref_ids].clone()
    # HOLD is a static terminal goal, independent of the frozen clip frame.
    home = motion.motion.body_pos_w[0, ref_ids] + env.scene.env_origins[:, None, :]
    reference = torch.where(motion.in_hold[:, None, None], home, reference)
    urgent = torch.zeros_like(motion.in_hold)
    if persistent_home:
        from .four_stroke import complete_reference
        urgent = ~complete_reference(motion, cmd)
        reference = torch.where(urgent[:, None, None], home, reference)
    error = cmd.robot.data.body_pos_w[:, ids] - reference
    scale = min(1., max(0., (env.common_step_counter-start_steps)/max(1, ramp_steps)))
    debt = foot_reference_debt(error)
    if clearance_std > 0. and yaw_std > 0.:
        # Opt-in v15: a 5--6 cm swing-foot lift must not disappear in a loose
        # 3-D average. Follow the physical reference; never reward high steps
        # outside its swing phase, or suppress corrective actuator velocities.
        from .full_body_hold import rpy, wrap_angle
        height = ((error[..., 2].abs()-.005)/clearance_std).clamp_min(0)
        height = torch.where(height < 1, .5*height.square(), height-.5).mean(-1)
        actual_yaw = rpy(cmd.robot.data.body_quat_w[:, ids])[..., 2]
        ref_yaw = rpy(motion.body_quat_w[:, ref_ids])[..., 2]
        ref_yaw = torch.where((motion.in_hold|urgent)[:, None], torch.zeros_like(ref_yaw), ref_yaw)
        yaw = ((wrap_angle(actual_yaw-ref_yaw).abs()-.035)/yaw_std).clamp_min(0)
        yaw = torch.where(yaw < 1, .5*yaw.square(), yaw-.5).mean(-1)
        debt = position_fraction*debt + .25*height + .15*yaw
        from .ready_recovery import _log
        _log(cmd, "four_stroke_id", motion.clip_id.float()+2*motion.step_reference_weight())
        _log(cmd, "four_stroke_active", (~motion.in_hold).float())
        _log(cmd, "four_stroke_foot_height_error_m", error[..., 2].abs().amax(-1))
        _log(cmd, "four_stroke_foot_yaw_error_rad", wrap_angle(actual_yaw-ref_yaw).abs().amax(-1))
    if getattr(motion.cfg, "step_reference_mode", "") == "discrete_four_stroke":
        from .four_stroke import complete_reference, supervision_gate
        if not persistent_home:
            debt = debt*supervision_gate(motion, cmd)
        from .ready_recovery import _log
        _log(cmd, "four_stroke_complete", complete_reference(motion, cmd).float())
    debt = debt*scale
    from .ready_recovery import _log
    _log(cmd, "fixed_station_foot_reference_error_m", error.norm(dim=-1).amax(-1))
    _log(cmd, "fixed_station_foot_home_error_m",
         (cmd.robot.data.body_pos_w[:, ids]-home).norm(dim=-1).amax(-1))
    _log(cmd, "fixed_station_feet_debt", debt)
    return debt
