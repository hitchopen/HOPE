"""Four discrete teachers; only the announced question may select a stroke.

This selector is a reference assignment, NOT a dynamic reachability certificate.
It preserves each bank row's position/velocity/deadline. Too-late questions are
task challenges without a partial-stroke teacher, not certified feasible balls.
"""
import torch


def observable_clip(contact, original_clip, position_boxes, standing, stepping):
    """Give the overlap ONE observable handedness without changing any tuple.

    The 110-D actor has no swing-sign channel. Outside overlap, the published
    boxes already determine side. Inside it, nearest full-body contact exemplar
    resolves side from the announced position; a source clip ID cannot do so.
    This is a training-view routing only, never a rewrite of the source bank.
    """
    boxes = contact.new_tensor(position_boxes)
    inside = ((contact[:, None] >= boxes[None, :, :, 0]-1e-6)
              & (contact[:, None] <= boxes[None, :, :, 1]+1e-6)).all(-1)
    examples = contact.new_tensor([*standing, *stepping])
    closest = (contact[:, None]-examples[None]).square().sum(-1).argmin(-1) % 2
    return torch.where(inside.all(-1), closest, original_clip)


def geometric_step(contact, side, standing, stepping):
    first = contact.new_tensor(standing)[side]
    second = contact.new_tensor(stepping)[side]
    return (contact-second).square().sum(-1) < (contact-first).square().sum(-1)


def select_step(contact, initial_tts, side, standing, stepping, minimum_lead):
    """Nearest contact exemplar, subject to enough time for the complete step.

    minimum_lead covers the first nonstationary frame of each asset under its
    bounded preparation clock. Never depends on body drift. This predicate is
    for sampling eligible complete steps; geometric_step owns the family label.
    """
    nearer = geometric_step(contact, side, standing, stepping)
    return nearer & (initial_tts + 1e-6 >= contact.new_tensor(minimum_lead)[side])


def waiting_for_first_ball(command):
    if not hasattr(command, "_motion"):
        return None
    motion = command._motion()
    if not getattr(motion.cfg, "causal_idle", False):
        return None
    # Terminal HOLD has counter == 0 and must retain the visible OLD ball.
    return motion.hold_counter > 0


def idle_target(command, value, *, velocity=False):
    waiting = waiting_for_first_ball(command)
    if waiting is None:
        return value
    # Match the existing 110-D runner's startup FH box-center packet. Its TTS
    # clamps to the common strike frame (1.2 s). No future side/row is exposed.
    key = "racket_vel_range_per_clip" if velocity else "racket_pos_range_per_clip"
    bounds = getattr(command.cfg, key)[0]
    center = value.new_tensor([(lo+hi)*.5 for lo, hi in bounds]).expand_as(value)
    if not velocity:
        center = center + command._env.scene.env_origins
    return torch.where(waiting[:, None], center, value)


def idle_clock(command, value):
    waiting = waiting_for_first_ball(command)
    return value if waiting is None else torch.where(waiting, 1.2, value)


def complete_reference(motion, command):
    """Which questions can play a whole preparation within the bounded clock?"""
    if getattr(motion.cfg, "step_reference_mode", "") != "discrete_four_stroke":
        return torch.ones_like(motion.clip_id, dtype=torch.bool)
    initial = command._stroke_initial_tts
    preparation = torch.where(command._stroke_step,
                       initial.new_tensor(motion.cfg.step_minimum_lead_s)[motion.clip_id],
                       initial.new_tensor(motion.cfg.stand_minimum_lead_s)[motion.clip_id])
    return initial+1e-6 >= preparation


def reference_clock(motion, command):
    """Compress only preparation, with unit source-clock rate at contact.

    f(t)=t+(L-T)*smoothstep(t/T), for remaining time 0<=t<=T. Each family has
    one FIXED preparation duration T. Given the same visible target and current
    TTS the teacher is identical, regardless of the initial announcement time.
    Max average compression 1.5 gives peak clock rate 1.75.
    Source frames are selected intact (FK exact); no blending between strokes.
    Recovery retains its original speed and may still be interrupted.
    """
    initial = command._stroke_initial_tts.clamp_min(.02)
    lead = torch.where(command._stroke_step,
                       initial.new_tensor(motion.cfg.step_complete_lead_s)[motion.clip_id],
                       initial.new_tensor(motion.cfg.stand_complete_lead_s)[motion.clip_id])
    preparation = torch.where(command._stroke_step,
                       initial.new_tensor(motion.cfg.step_minimum_lead_s)[motion.clip_id],
                       initial.new_tensor(motion.cfg.stand_minimum_lead_s)[motion.clip_id])
    complete = complete_reference(motion, command)
    start = motion.motion.seg_start[motion.clip_id]
    remaining = (60-(motion.time_steps-start)).float()*.02
    u = (remaining/preparation).clamp(0, 1)
    extra = (lead-preparation).clamp_min(0)
    source_remaining = remaining + extra*u.square()*(3-2*u)
    frame = start + (60-source_remaining/.02).round().long().clamp_min(0)
    rate = 1 + extra/preparation*6*u*(1-u)
    return frame, rate, complete


def supervision_gate(motion, command):
    # A too-late question is still a task challenge, but an infeasible partial
    # stroke is not a teacher. Static final Home/HOLD always remains supervised.
    return (complete_reference(motion, command) | motion.in_hold).float()


def sample_style_pool(command, bank, pool, size, key):
    """Stratify WITHIN source/cohort, leaving all original tuples intact.

    Empty style strata fall back to the existing pool; they cannot manufacture
    real demonstrations. A per-source count is printed when pools are built.
    """
    motion_cfg = command._motion().cfg
    if getattr(motion_cfg, "step_reference_mode", "") != "discrete_four_stroke":
        return pool[torch.randint(len(pool), (size,), device=command.device)]
    cached = getattr(command, "_four_stroke_pools", None)
    if cached is None:
        cached = command._four_stroke_pools = {}
    if key not in cached:
        side = torch.full((len(pool),), int(key[0]), device=command.device, dtype=torch.long)
        step = geometric_step(bank["target_position_offset"][pool], side,
                              motion_cfg.stand_contact_xyz, motion_cfg.step_contact_xyz)
        lead = torch.where(step, side.new_tensor(motion_cfg.step_minimum_lead_s, dtype=torch.float)[side],
                            side.new_tensor(motion_cfg.stand_minimum_lead_s, dtype=torch.float)[side])
        effective = torch.ceil(bank["time_to_strike"][pool]/.02)*.02
        complete = effective+1e-6 >= lead
        cached[key] = (pool[~step & complete], pool[step & complete], pool[~complete])
        print(f"[four-stroke] source {key}: complete_stand={len(cached[key][0])}, "
              f"complete_step={len(cached[key][1])}, urgent={len(cached[key][2])}", flush=True)
    stand, step, urgent = cached[key]
    if not len(stand) and not len(step):
        return urgent[torch.randint(len(urgent), (size,), device=command.device)]
    # No synchronization per selected row: sample both then choose on device.
    step_probability = motion_cfg.step_sampling_probability if len(stand) and len(step) else float(bool(len(step)))
    choose = torch.rand(size, device=command.device) < step_probability
    a_pool = stand if len(stand) else step
    b_pool = step if len(step) else stand
    if getattr(motion_cfg, "stroke_adapter_path", None):
        from .stroke_tracking import sample_pool
        a = sample_pool(command, bank, a_pool, size)
        b = sample_pool(command, bank, b_pool, size)
    else:
        a = a_pool[torch.randint(len(a_pool), (size,), device=command.device)]
        b = b_pool[torch.randint(len(b_pool), (size,), device=command.device)]
    selected = torch.where(choose, b, a)
    if len(urgent):
        challenge = torch.rand(size, device=command.device) < motion_cfg.urgent_sampling_probability
        u = urgent[torch.randint(len(urgent), (size,), device=command.device)]
        selected = torch.where(challenge, u, selected)
    return selected


def family_summaries(metrics):
    """Conditional live errors: a good standing stroke cannot hide a bad step.

    These are state snapshots, not completed-shot success rates. An absent
    family reports NaN instead of a misleading zero-error success.
    """
    if "four_stroke_id" not in metrics:
        return {}
    out = {}
    for i, name in enumerate(("stand_forehand", "stand_backhand", "step_forehand", "step_backhand")):
        mask = ((metrics["four_stroke_active"] > 0) & (metrics["four_stroke_id"] == i)
                & (metrics["four_stroke_complete"] > 0))
        count = mask.sum()
        out[f"stroke_{name}_fraction"] = mask.float().mean()
        for field in ("fixed_station_foot_reference_error_m", "four_stroke_foot_height_error_m",
                      "four_stroke_foot_yaw_error_rad", "imitation_left_leg_rms_rad",
                      "imitation_right_leg_rms_rad", "imitation_torso_rp_error_rad"):
            if field in metrics:
                value = metrics[field]
                out[f"stroke_{name}_{field}"] = torch.where(
                    count > 0, (value*mask).sum()/count.clamp_min(1), value.new_tensor(float("nan")))
    return out
