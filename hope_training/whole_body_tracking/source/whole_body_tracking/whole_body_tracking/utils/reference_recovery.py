"""One visible ball clock owns swing, natural recovery and the terminal pose.

The arrival clock is exogenous: it can interrupt recovery but never stops a
demonstrated step. No next-ball reference is exposed before that ball arrives.
"""
import torch


def enabled(motion):
    return bool(getattr(getattr(motion, "cfg", None), "continue_recovery_reference", False))


def static_gate(motion, tts):
    # All four references have reached their common terminal pose at +1 s.
    # The 100 ms crossfade starts AFTER that, not halfway through collecting a foot.
    return torch.where(motion.in_hold, torch.ones_like(tts),
                       ((-tts - 1.0) / .1).clamp(0, 1))


def base_reference(env, cmd):
    """A transient teacher offset; never changes the commanded home/actor error."""
    motion = env.command_manager.get_term("motion")
    if not enabled(motion):
        return cmd.base_target_pos_w, torch.zeros_like(cmd.base_target_pos_w)
    stamp = getattr(env, "common_step_counter", None)
    cached = getattr(env, "_transient_base_reference_cache", None)
    if stamp is not None and cached is not None and cached[0] == stamp:
        return cached[1], cached[2]
    index = motion.cfg.body_names.index("pelvis_link")
    start = motion.motion.seg_start[motion.clip_id]
    offset = motion.reference_at("body_pos_w")[:, index, :2] - motion.motion.body_pos_w[start, index, :2]
    velocity = motion.reference_at("body_lin_vel_w")[:, index, :2]
    dynamic = (~motion.in_hold).unsqueeze(-1)
    if getattr(motion.cfg, "step_reference_mode", "") == "discrete_four_stroke":
        from .four_stroke import complete_reference
        # A question announced too late has no full motion teacher; keep the
        # fixed task station instead of asking for a skipped step's root pose.
        dynamic = dynamic & complete_reference(motion, cmd).unsqueeze(-1)
    position, velocity = cmd.base_target_pos_w + offset * dynamic, velocity * dynamic
    if stamp is not None:
        env._transient_base_reference_cache = (stamp, position, velocity)
    return position, velocity


def advance_tail(motion, arrived):
    """Called after the ordinary one-frame clock advance; returns next-ball ids."""
    from .interruptible_recovery import sample_arrivals
    clock = motion.inter_ball_recovery
    eligible = (motion.time_steps >= motion.rally_wrap_step) & ~clock.waiting & ~arrived
    ids = torch.where(eligible)[0]
    if len(ids):
        budget, arrival = sample_arrivals(len(ids), motion.device,
            motion.cfg.recovery_window_steps_range, motion.cfg.next_ball_delay_steps_range,
            motion.cfg.next_ball_immediate_prob)
        # An exogenous mixture adds post-strike recovery practice without
        # blocking short arrivals or making the ball wait for the policy.
        dwell_probability = float(getattr(motion.cfg, "recovery_dwell_probability", 0.))
        if dwell_probability:
            if not 0 < dwell_probability < 1:
                raise ValueError("recovery dwell must retain ordinary interrupting rallies")
            lo, hi = motion.cfg.recovery_dwell_delay_steps_range
            if not 0 <= lo <= hi:
                raise ValueError("invalid recovery dwell range")
            dwell = torch.randint(lo, hi+1, (len(ids),), device=motion.device)
            arrival = torch.where(torch.rand(len(ids), device=motion.device) < dwell_probability,
                                  dwell, arrival)
        clock.begin(ids, budget, arrival)
        arrived[ids] = arrival == 0
    end = motion.motion.seg_start[motion.clip_id] + motion.motion.seg_len[motion.clip_id] - 1
    motion.time_steps.copy_(torch.minimum(motion.time_steps, end))
    return torch.where(arrived)[0]
