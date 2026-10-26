"""Evidence-gated mocap progression for four strokes and original-Home recovery.

All evidence is actual physical state, conditional on an applicable published
reference. No target, action, arrival clock or termination is changed here.
State lives as plain tensors/scalars on the command for checkpoint capture.
"""
import math
import torch

MODE = "four_stroke_quality_v1"


def enabled(command):
    return getattr(getattr(command, "cfg", None), "ability_curriculum_mode", "disabled") == MODE


def validate(cfg):
    for name in ("ability_min_family_quality", "ability_min_hold_quality", "ability_min_home_quality"):
        value = float(getattr(cfg, name))
        if not math.isfinite(value) or not 0 < value <= 1:
            raise ValueError(f"{name} must be in (0, 1]")
    for name in ("ability_quality_min_samples", "ability_quality_max_age_steps"):
        if int(getattr(cfg, name)) < 1:
            raise ValueError(f"{name} must be positive")


def initialize(command):
    # Four stroke families, terminal HOLD, and BOTH feet/base at original Home.
    command._motion_quality_ema = torch.zeros(6, device=command.device)
    command._motion_quality_samples = torch.zeros(6, device=command.device)
    command._motion_quality_last_step = torch.full((6,), -1, dtype=torch.long, device=command.device)
    command._motion_quality_progress_steps = 0
    command._motion_quality_update_step = -1
    command._motion_quality_gate_ready = False


def observe(command, slot, selected, success):
    """A missing phase supplies no evidence; repeated diagnostics are idempotent."""
    if not enabled(command):
        return
    step = int(command._env.common_step_counter)
    count = selected.sum()
    fresh = (count > 0) & (command._motion_quality_last_step[slot] != step)
    rate = (selected & success).float().sum() / count.clamp_min(1)
    old = command._motion_quality_ema[slot]
    command._motion_quality_ema[slot] = torch.where(fresh, .995*old + .005*rate, old)
    command._motion_quality_samples[slot] += torch.where(fresh, count, 0)
    command._motion_quality_last_step[slot] = torch.where(
        fresh, step, command._motion_quality_last_step[slot])


def recovery_evidence_mask(motion, gate, episode_step, dt):
    """Startup HOLD and an interrupted swing are not completed recovery trials."""
    terminal = motion.motion.seg_start[motion.clip_id]+motion.motion.seg_len[motion.clip_id]-1
    return ((motion.time_steps >= terminal) & (motion.hold_counter == 0)
            & (gate >= .95) & (episode_step*dt >= .5))


def advance(command, strike_ready):
    """Only qualified control steps advance the ramp; failure pauses immediately.

    Keep attained corruption instead of rapidly switching the observation
    distribution. Re-admission requires the full dwell again. The ramp never
    catches up to wall time after a pause.
    """
    step = int(command._env.common_step_counter)
    if command._motion_quality_update_step == step:
        return command._motion_quality_gate_ready
    if command._motion_quality_update_step != step-1:
        command._ability_gate_dwell = 0
    command._motion_quality_update_step = step
    cfg = command.cfg
    thresholds = command._motion_quality_ema.new_tensor([
        cfg.ability_min_family_quality]*4 + [cfg.ability_min_hold_quality, cfg.ability_min_home_quality])
    age = step-command._motion_quality_last_step
    evidence = ((command._motion_quality_samples >= cfg.ability_quality_min_samples)
                & (age >= 0) & (age <= cfg.ability_quality_max_age_steps))
    passes = evidence & (command._motion_quality_ema >= thresholds)
    quality_ready = bool(passes.all())
    ready = bool(strike_ready) and quality_ready
    command._motion_quality_gate_ready = ready
    command._ability_gate_dwell = command._ability_gate_dwell+1 if ready else 0
    advancing = ready and command._ability_gate_dwell >= cfg.ability_gate_dwell_steps
    if advancing:
        if not command._ability_unlocked:
            command._ability_unlocked = True
            command._ability_unlock_step = step
            command._base_mocap_residual_needs_sample.fill_(True)
        command._motion_quality_progress_steps += 1
    command._base_mocap_robustness_scale = min(
        command._motion_quality_progress_steps / cfg.base_mocap_robustness_ramp_steps, 1.)
    for name, value in {
        "ability_motion_quality_ready": quality_ready,
        "ability_gate_condition": ready,
        "ability_progress_active": advancing,
        "ability_progress_steps": command._motion_quality_progress_steps,
    }.items():
        if name not in command.metrics:
            command.metrics[name] = torch.zeros(command.num_envs, device=command.device)
        command.metrics[name][:] = float(value)
    for i, name in enumerate(("stand_fh", "stand_bh", "step_fh", "step_bh", "hold", "home")):
        for suffix, value in (("quality", command._motion_quality_ema[i]),
                              ("evidence", evidence[i]), ("pass", passes[i])):
            key = f"ability_{name}_{suffix}"
            if key not in command.metrics:
                command.metrics[key] = torch.zeros(command.num_envs, device=command.device)
            command.metrics[key][:] = value
    return ready
