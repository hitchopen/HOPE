"""v7 actual ready-state objectives; no q_des-q cost and no physical-model changes."""
import math
import torch

from .task_pose_reference import load_reference, reference_joints


def bounded_debt(error, margin, std):
    if not math.isfinite(margin) or margin < 0 or not math.isfinite(std) or std <= 0:
        raise ValueError("invalid ready-state margin/std")
    x = (error.abs() - margin).clamp_min(0) / std
    h = torch.where(x < 1, .5 * x.square(), x - .5)
    return h / (1 + h)


def recovery_gate(in_hold, tts):
    # Protect strike follow-through; wait-only recovery ends immediately on arrival.
    return torch.where(in_hold, torch.ones_like(tts), ((-tts - .5) / .3).clamp(0, 1))


def leg_pose_debt(knees, height, knee_ref, height_ref, knee_margin, knee_std,
                  height_margin, height_std):
    return (.6 * bounded_debt(knees - knee_ref, knee_margin, knee_std).mean(-1)
            + .4 * bounded_debt(height - height_ref, height_margin, height_std))


def station_debt(delta, velocity, position_margin=.03, position_std=.10,
                 velocity_margin=.05, velocity_std=.20):
    distance = torch.linalg.vector_norm(delta, dim=-1)
    # A moving approach is rewarded over freezing far from the station. Inside
    # the position tolerance the desired velocity is exactly zero.
    direction = delta / distance.clamp_min(1e-6).unsqueeze(-1)
    desired = direction * (2 * (distance - position_margin).clamp_min(0)).clamp_max(.4).unsqueeze(-1)
    velocity_error = torch.linalg.vector_norm(velocity - desired, dim=-1)
    return (.65 * bounded_debt(distance, position_margin, position_std)
            + .35 * bounded_debt(velocity_error, velocity_margin, velocity_std))


def _context(env, start_steps, ramp_steps):
    if start_steps < 0 or ramp_steps <= 0:
        raise ValueError("invalid ready-state curriculum")
    cmd = env.command_manager.get_term("racket_target")
    motion = env.command_manager.get_term("motion")
    gate = recovery_gate(motion.in_hold, cmd.time_to_strike)
    scale = min(1., max(0., (env.common_step_counter - start_steps) / ramp_steps))
    return cmd, gate, scale


def _log(cmd, key, value):
    # Never alias simulator buffers: command reset zeroes metrics in place.
    if key not in cmd.metrics:
        cmd.metrics[key] = value.detach().clone()
    else:
        cmd.metrics[key].copy_(value.detach())


def actual_ready_legs(env, knee_margin=.0872664626, knee_std=.1745329252,
                      height_margin=.02, height_std=.04, start_steps=448000, ramp_steps=128000):
    cmd, gate, scale = _context(env, start_steps, ramp_steps)
    robot = cmd.robot
    cache = getattr(env, "_ready_leg_reference", None)
    if cache is None:
        names = ["left_knee_joint", "right_knee_joint"]
        ids, resolved = robot.find_joints(names, preserve_order=True)
        if list(resolved) != names:
            raise ValueError("ready pose requires both named knee joints")
        cache = (ids, reference_joints(names, robot.data.joint_pos),
                 load_reference()["metrics"]["pelvis_height_cm"] / 100.)
        env._ready_leg_reference = cache
    ids, knee_ref, height_ref = cache
    knees = robot.data.joint_pos[:, ids]
    height = robot.data.root_pos_w[:, 2] - env.scene.env_origins[:, 2]
    debt = leg_pose_debt(knees, height, knee_ref, height_ref,
                         knee_margin, knee_std, height_margin, height_std) * gate * scale
    _log(cmd, "ready_state_gate", gate)
    _log(cmd, "ready_state_curriculum_scale", torch.full_like(gate, scale))
    _log(cmd, "ready_state_knee_error_rad", (knees - knee_ref).abs().mean(-1))
    _log(cmd, "ready_state_height_error_m", height - height_ref)
    _log(cmd, "ready_state_legs_debt", debt)
    return debt


def station_recovery(env, position_margin=.03, position_std=.10, velocity_margin=.05,
                     velocity_std=.20, start_steps=448000, ramp_steps=128000):
    cmd, gate, scale = _context(env, start_steps, ramp_steps)
    # Same target exposed by actor_base_target_delta_xy_w and base_position.
    delta = cmd.base_target_pos_w - cmd.base_pos_w[:, :2]
    velocity = cmd.robot.data.root_lin_vel_w[:, :2]
    debt = station_debt(delta, velocity, position_margin, position_std,
                        velocity_margin, velocity_std) * gate * scale
    _log(cmd, "ready_state_station_error_m", torch.linalg.vector_norm(delta, dim=-1))
    _log(cmd, "ready_state_station_debt", debt)
    return debt


def loaded_ankle_quiet(env, margin=.08, std=.30, start_steps=448000, ramp_steps=128000):
    cmd, gate, scale = _context(env, start_steps, ramp_steps)
    cache = getattr(env, "_ready_ankle_ids", None)
    if cache is None:
        ids, _ = cmd.robot.find_joints(["left_ankle_roll_joint", "right_ankle_roll_joint"], preserve_order=True)
        sensor = env.scene.sensors["contact_forces"]
        feet, _ = sensor.find_bodies(["left_ankle_roll_Link", "right_ankle_roll_Link"], preserve_order=True)
        if len(ids) != 2 or len(feet) != 2:
            raise ValueError("ankle quiet requires two ankle joints and feet")
        cache = (ids, sensor, feet)
        env._ready_ankle_ids = cache
    ids, sensor, feet = cache
    loaded = (sensor.data.net_forces_w[:, feet, 2] > 20).all(-1)
    distance = torch.linalg.vector_norm(cmd.base_target_pos_w - cmd.base_pos_w[:, :2], dim=-1)
    near = ((.15 - distance) / .10).clamp(0, 1)
    qd = cmd.robot.data.joint_vel[:, ids]
    eligible = gate * loaded * near
    debt = bounded_debt(qd, margin, std).mean(-1) * eligible * scale
    _log(cmd, "ready_state_ankle_velocity_rms_rad_s", qd.square().mean(-1).sqrt())
    _log(cmd, "ready_state_ankle_quiet_gate", eligible)
    _log(cmd, "ready_state_ankle_quiet_debt", debt)
    return debt
