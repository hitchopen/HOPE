"""Small task-conditioned velocity impulses for ping-pong sim-to-real training.

The implementation only depends on torch so its production event can be tested on a CPU host.
Mass/material DR remains startup-only; this event models occasional external disturbances.
"""

from __future__ import annotations

import math

import torch


def bounded_planar_push(
    env,
    env_ids: torch.Tensor | None,
    *,
    asset_name: str = "robot",
    command_name: str = "racket_target",
    max_delta_v_xy: float = 0.10,
    cohort_fraction: float = 0.0,
    ramp_steps: int = 96_000,
    strike_exclusion_s: float = 0.15,
) -> None:
    """Add a bounded XY velocity impulse to a fixed, interleaved environment cohort.

    ``max_delta_v_xy`` bounds each planar component in m/s, not absolute root velocity.
    The common control-step counter ramps the magnitude from zero. No impulse is applied near
    contact or on terminal/reset ticks, and Z velocity and all angular velocities are preserved.
    Zero cohort fraction is deliberately inert for direct historical Gym construction.
    """
    if not math.isfinite(float(cohort_fraction)) or not 0.0 <= cohort_fraction <= 1.0:
        raise ValueError("push cohort_fraction must lie in [0, 1]")
    for name, value in (("max_delta_v_xy", max_delta_v_xy), ("strike_exclusion_s", strike_exclusion_s)):
        if not math.isfinite(float(value)) or value < 0.0:
            raise ValueError(f"push {name} must be finite and nonnegative")
    if int(ramp_steps) != ramp_steps or ramp_steps < 0:
        raise ValueError("push ramp_steps must be a nonnegative integer")
    if cohort_fraction == 0.0 or max_delta_v_xy == 0.0:
        return
    scale = min(max(float(env.common_step_counter), 0.0) / ramp_steps, 1.0) if ramp_steps else 1.0
    if scale == 0.0:
        return

    asset = env.scene[asset_name]
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=asset.device, dtype=torch.long)
    else:
        env_ids = env_ids.to(device=asset.device, dtype=torch.long)
    if env_ids.numel() == 0:
        return

    # Evenly distribute a permanent cohort across FH/BH/core blocks instead of selecting the
    # first fraction of environments. For fraction=.5 this selects every second environment.
    ids_float = env_ids.to(torch.float64)
    selected = torch.floor((ids_float + 1.0) * cohort_fraction) > torch.floor(ids_float * cohort_fraction)
    command = env.command_manager.get_term(command_name)
    tts = command.time_to_strike[env_ids]
    selected &= torch.isfinite(tts) & (tts.abs() > float(strike_exclusion_s))
    selected &= env.episode_length_buf[env_ids] > 1
    # reset_buf can retain terminal evidence after ManagerBasedRLEnv has already reset those
    # environments; checking both flags avoids injecting an impulse into a new initial state.
    reset_buf = getattr(env, "reset_buf", None)
    if torch.is_tensor(reset_buf):
        selected &= ~reset_buf[env_ids].bool()
    dones = getattr(getattr(env, "termination_manager", None), "dones", None)
    if torch.is_tensor(dones):
        selected &= ~dones[env_ids].bool()
    pushed_ids = env_ids[selected]
    if pushed_ids.numel() == 0:
        return
    # clone is intentional: do not mutate the cached sensor state before the PhysX write.
    velocity = asset.data.root_vel_w[pushed_ids].clone()
    delta_xy = torch.empty((pushed_ids.numel(), 2), device=velocity.device, dtype=velocity.dtype)
    delta_xy.uniform_(-float(max_delta_v_xy) * scale, float(max_delta_v_xy) * scale)
    velocity[:, :2] += delta_xy
    asset.write_root_velocity_to_sim(velocity, env_ids=pushed_ids)
    metrics = getattr(command, "metrics", None)
    if isinstance(metrics, dict):
        if "sim2real_push_count" not in metrics:
            metrics["sim2real_push_count"] = torch.zeros(env.num_envs, device=velocity.device)
            metrics["sim2real_push_last_delta_mps"] = torch.zeros(env.num_envs, device=velocity.device)
        metrics["sim2real_push_count"][pushed_ids] += 1
        metrics["sim2real_push_last_delta_mps"][pushed_ids] = torch.linalg.vector_norm(delta_xy, dim=-1)
