"""Actual task posture and PD diagnostics. No differentiable q_des-q objective."""
from __future__ import annotations

import csv
import hashlib
import json
import math
from functools import lru_cache
from pathlib import Path

import torch

REFERENCE_NAME = "B_32_5_K20"
REFERENCE_SHA256 = "c17d7ef1371fc2bbf3e922946b5254d17c7ed0bd5ead053d472edfc1c2580e5f"
PHASE_NAMES = ("hold", "strike", "ready_recover")


@lru_cache(maxsize=1)
def load_reference():
    path = Path(__file__).resolve().parents[4] / "cfg/references/hold_pose_B_32_5_K20.json"
    raw = path.read_bytes()
    if hashlib.sha256(raw).hexdigest() != REFERENCE_SHA256:
        raise ValueError("selected HOLD reference changed; review and version its contract")
    pose = json.loads(raw)
    if pose["candidate_key"] != REFERENCE_NAME:
        raise ValueError("unexpected HOLD reference")
    return pose


def reference_joints(names, like):
    pose = load_reference()
    by_name = dict(zip(pose["joint_order"], pose["actual_q_reference_rad"]))
    return like.new_tensor([by_name[name] for name in names])


def torso_roll_pitch(quat):
    w, x, y, z = quat.unbind(-1)
    return torch.stack((torch.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
                        torch.asin((2*(w*y-z*x)).clamp(-1, 1))), -1)


def phase_gate(in_hold, tts, *, ready_free_s=.45, ready_full_s=.85,
               followthrough_free_s=.50, recovery_full_s=.80):
    if not (0 <= ready_free_s < ready_full_s and 0 <= followthrough_free_s < recovery_full_s):
        raise ValueError("invalid task posture phase windows")
    ready = ((tts-ready_free_s)/(ready_full_s-ready_free_s)).clamp(0, 1)
    recover = ((-tts-followthrough_free_s)/(recovery_full_s-followthrough_free_s)).clamp(0, 1)
    gate = torch.where(in_hold, torch.ones_like(tts), torch.maximum(ready, recover))
    # Includes preparation in ready_recover because the continuous rally clock
    # restarts after follow-through; a negative-TTS-only recovery misses it.
    phase = torch.where(in_hold, 0, torch.where(gate > 0, 2, 1))
    return gate, phase


def actual_pose_debt(waist_pitch, torso_rp, waist_ref, torso_ref, *, margin=.0523598776, std=.15,
                     torso_margin=None, torso_std=None):
    if not math.isfinite(margin) or not math.isfinite(std) or margin < 0 or std <= 0:
        raise ValueError("invalid actual task posture margin/std")
    torso_margin = margin if torso_margin is None else torso_margin
    torso_std = std if torso_std is None else torso_std
    if not math.isfinite(torso_margin) or torso_margin < 0 or not math.isfinite(torso_std) or torso_std <= 0:
        raise ValueError("invalid torso posture margin/std")
    def debt(error, deadband, scale):
        x = ((error.abs()-deadband)/scale).clamp_min(0)
        h = torch.where(x < 1, .5*x.square(), x-.5)
        return h/(1+h)
    waist = debt(waist_pitch-waist_ref, margin, std)
    torso = debt(torso_rp-torso_ref, torso_margin, torso_std).mean(-1)
    return .5*waist + .5*torso


class PoseChainTrace:
    """Synchronized 50-Hz bursts, four environments, before reset can erase state."""
    def __init__(self, directory, reference_scope="waist/torso HOLD and ready_recover; inapplicable during STRIKE"):
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        self.path = directory / "task_pose_chain.csv"
        self.last_step = None
        self.pending = []
        (directory / "task_pose_reference.json").write_text(json.dumps(load_reference(), indent=2)+"\n")
        (directory / "task_pose_chain_contract.json").write_text(json.dumps({
            "reference": REFERENCE_NAME, "source_sha256": REFERENCE_SHA256,
            "phases": dict(enumerate(PHASE_NAMES)),
            "sampling": "first 4 envs, first 512 steps of each 64000-control-step period, before reset",
            "reference_scope": reference_scope,
            "pd_offset": "executed q_des minus actual q; diagnostic only",
            "pd_torque_estimate": "ImplicitActuator clipped PD estimate, not measured motor torque",
            "joint_axis_torque": "PhysX projected incoming joint force; includes constraint effects",
        }, indent=2)+"\n")

    def record(self, step, episode_steps, metrics):
        # 10.24 s at startup and each 1000 updates (64 control steps/update).
        # Bounds a 30k-update trace to ~61k rows, without decimating away chatter.
        if step == self.last_step or step % 64000 >= 512:
            return
        self.last_step = step
        keys = sorted(metrics)
        count = min(4, len(episode_steps))
        # Copy only sampled rows and batch the device-to-host transfer. The
        # stack owns its data even when the environment reuses metric buffers.
        matrix = torch.stack([episode_steps[:count].float(), *[metrics[k][:count] for k in keys]], -1).detach()
        self.pending.append((step, keys, matrix))
        if len(self.pending) >= 64:
            self.flush()

    def flush(self):
        if not self.pending:
            return
        matrices = torch.stack([entry[2] for entry in self.pending]).cpu().tolist()
        fresh = not self.path.exists()
        with self.path.open("a", newline="") as stream:
            writer = csv.writer(stream)
            if fresh:
                writer.writerow(["control_step", "env_id", "episode_step", *self.pending[0][1]])
            for (step, keys, _), matrix in zip(self.pending, matrices):
                for env_id, row in enumerate(matrix):
                    writer.writerow([step, env_id, *row])
        self.pending.clear()


def evaluate_task_pose(env, *, start_steps=448000, ramp_steps=128000, margin=.0523598776,
                       std=.15, ready_free_s=.45, ready_full_s=.85,
                       followthrough_free_s=.50, recovery_full_s=.80,
                       torso_margin=None, torso_std=None):
    """Return a real state reward; q_des and torque participate only in logging."""
    if start_steps < 0 or ramp_steps <= 0:
        raise ValueError("invalid actual task posture curriculum")
    cmd = env.command_manager.get_term("racket_target")
    motion = env.command_manager.get_term("motion")
    robot = cmd.robot
    action = env.action_manager.get_term("joint_pos")
    cache = getattr(env, "_actual_task_pose_cache", None)
    if cache is None:
        names = [f"waist_{axis}_joint" for axis in ("yaw", "roll", "pitch")]
        ids, resolved = robot.find_joints(names, preserve_order=True)
        if list(resolved) != names:
            raise ValueError("actual task posture requires the three named waist joints")
        torso_id = cmd._resolve_body_index("torso_Link")
        if torso_id < 0:
            raise ValueError("actual task posture requires torso_Link")
        ref = reference_joints(names, robot.data.joint_pos)
        rp = robot.data.joint_pos.new_tensor(load_reference()["metrics"]["torso_rpy_deg"][:2])*math.pi/180
        columns = [action._joint_names.index(name) for name in names]
        cache = (names, ids, torso_id, ref, rp, columns)
        env._actual_task_pose_cache = cache
    names, ids, torso_id, ref, rp, columns = cache
    actual = robot.data.joint_pos[:, ids]
    torso = torso_roll_pitch(robot.data.body_quat_w[:, torso_id])
    gate, phase = phase_gate(motion.in_hold, cmd.time_to_strike,
                            ready_free_s=ready_free_s, ready_full_s=ready_full_s,
                            followthrough_free_s=followthrough_free_s, recovery_full_s=recovery_full_s)
    # No force/speed/error eligibility: unloading a foot or remaining crooked
    # cannot disable the torso/waist objective. Legs remain free to balance.
    scale = min(1., max(0., (env.common_step_counter-start_steps)/ramp_steps))
    debt = actual_pose_debt(actual[:, 2], torso, ref[2], rp, margin=margin, std=std,
                           torso_margin=torso_margin, torso_std=torso_std)*gate*scale
    qdes = action._qdes_executed[:, columns]
    torque = robot.data.applied_torque[:, ids]
    # ImplicitActuator.applied_torque is a clipped PD estimate in this Isaac
    # version. PhysX projects actual joint reaction onto each DOF below; that
    # includes constraints and is NOT an isolated actuator/motor measurement.
    joint_torque = robot.root_physx_view.get_dof_projected_joint_forces()[:, ids]
    values = {"valid": torch.ones_like(gate), "phase": phase.float(), "phase_gate": gate,
              "curriculum_scale": torch.full_like(gate, scale), "debt": debt,
              "reference_applicable": (gate > 0).float()}
    for i, axis in enumerate(("yaw", "roll", "pitch")):
        stem = f"waist_{axis}"
        values.update({f"{stem}_reference_rad": ref[i].expand_as(gate),
                       f"{stem}_actual_rad": actual[:, i],
                       f"{stem}_qdes_rad": qdes[:, i],
                       f"{stem}_pd_torque_estimate_Nm": torque[:, i],
                       f"{stem}_joint_axis_torque_Nm": joint_torque[:, i],
                       f"{stem}_velocity_rad_s": robot.data.joint_vel[:, ids[i]],
                       f"{stem}_task_error_rad": actual[:, i]-ref[i],
                       f"{stem}_pd_offset_rad": qdes[:, i]-actual[:, i]})
    for i, axis in enumerate(("roll", "pitch")):
        values[f"torso_{axis}_reference_rad"] = rp[i].expand_as(gate)
        values[f"torso_{axis}_actual_rad"] = torso[:, i]
        values[f"torso_{axis}_task_error_rad"] = torso[:, i]-rp[i]
    # The actual-limit reward precedes this reward in the registered manager.
    # Put clearance beside q_des/q/torque so hard-stop support is inspectable.
    for field in ("waist_pitch_lower_clearance_rad", "waist_pitch_upper_clearance_rad",
                  "waist_pitch_buffer_rad", "waist_pitch_debt"):
        key = "actual_limit_"+field
        if key in cmd.metrics:
            values[key] = cmd.metrics[key]
    # CommandTerm.reset zeroes metrics in-place. Never alias articulation data.
    for key, value in values.items():
        name = "task_pose_"+key
        if name not in cmd.metrics:
            cmd.metrics[name] = value.detach().clone()
        else:
            cmd.metrics[name].copy_(value.detach())
    trace = getattr(env, "_task_pose_chain_trace", None)
    if trace is not None:
        trace.record(int(env.common_step_counter), env.episode_length_buf, values)
    return debt
