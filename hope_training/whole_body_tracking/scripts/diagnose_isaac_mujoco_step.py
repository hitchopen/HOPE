#!/usr/bin/env python3
# Copyright (c) 2026 Intelligent Racing Inc. (dba Hitch Interactive)
# SPDX-License-Identifier: Apache-2.0
"""One-step Isaac vs MuJoCo actuator / observation / torque diagnostic.

This script intentionally does not change training. It splits the diagnostic into
two phases so Isaac Sim and MuJoCo do not have to live in the same Python process:

1. ``capture-isaac`` runs the Torch checkpoint in Isaac for one environment and
   stores the exact policy observation, raw action, processed q_des, joint torque
   and base pose for each 50 Hz policy tick.
2. ``compare-mujoco`` replays those samples through the exported ONNX and the
   reference MuJoCo bridge, then writes aligned per-step and per-joint tables.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import pathlib
import sys
from typing import Iterable

import numpy as np


PASSIVE_HEAD_INDICES = (3, 4)

FK_DIAGNOSTIC_BODY_NAMES = (
    "pelvis_link",
    "torso_Link",
    "left_hip_roll_Link",
    "left_knee_Link",
    "left_ankle_roll_Link",
    "right_hip_roll_Link",
    "right_knee_Link",
    "right_ankle_roll_Link",
    "left_shoulder_roll_Link",
    "left_elbow_Link",
    "left_wrist_yaw_Link",
    "right_shoulder_roll_Link",
    "right_elbow_Link",
    "right_wrist_yaw_Link",
    "pingpang_red_Link",
)


def _repo_root() -> pathlib.Path:
    here = pathlib.Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "a3_deploy").is_dir() and (parent / "hope_training").is_dir():
            return parent
    return here.parents[3]


def _reference_pkg_dir(repo_root: pathlib.Path) -> pathlib.Path:
    return repo_root / "a3_deploy" / "a3_deploy_example" / "reference"


def _default_runtime_config(repo_root: pathlib.Path) -> pathlib.Path:
    return repo_root / "a3_deploy" / "a3_deploy_example" / "config" / "hope_pingpong_runtime.yaml"


def _default_ball_physics_config(repo_root: pathlib.Path) -> pathlib.Path:
    return repo_root / "configs" / "ball_physics.yaml"


def _ensure_reference_importable(reference_dir: str | None = None) -> pathlib.Path:
    repo_root = _repo_root()
    ref_dir = pathlib.Path(reference_dir).resolve() if reference_dir else _reference_pkg_dir(repo_root)
    if str(ref_dir) not in sys.path:
        sys.path.insert(0, str(ref_dir))
    return ref_dir


def _as_numpy(value, *, row: int | None = None, dtype=np.float64) -> np.ndarray:
    if value is None:
        raise ValueError("cannot convert None to numpy")
    if hasattr(value, "detach"):
        arr = value.detach().cpu().numpy()
    else:
        arr = np.asarray(value)
    if row is not None and arr.ndim > 0:
        arr = arr[row]
    return np.asarray(arr, dtype=dtype).copy()


def _optional_numpy(value, *, row: int | None = None, shape: tuple[int, ...], dtype=np.float64) -> np.ndarray:
    if value is None:
        return np.full(shape, np.nan, dtype=dtype)
    return _as_numpy(value, row=row, dtype=dtype).reshape(shape)


def _first_attr(obj, names: Iterable[str]):
    for name in names:
        if hasattr(obj, name):
            return getattr(obj, name)
    return None


def _canonical_from_named(values: np.ndarray, names: list[str], canonical_names: list[str]) -> np.ndarray:
    idx = {name: i for i, name in enumerate(names)}
    missing = [name for name in canonical_names if name not in idx]
    if missing:
        raise RuntimeError(f"missing canonical names in source order: {missing}")
    return np.asarray(values, dtype=np.float64)[[idx[name] for name in canonical_names]].copy()


def _max_abs(arr: np.ndarray) -> float:
    arr = np.asarray(arr, dtype=np.float64)
    if arr.size == 0 or not np.any(np.isfinite(arr)):
        return float("nan")
    return float(np.nanmax(np.abs(arr)))


def _mean_abs(arr: np.ndarray) -> float:
    arr = np.asarray(arr, dtype=np.float64)
    finite = np.isfinite(arr)
    if arr.size == 0 or not np.any(finite):
        return float("nan")
    return float(np.mean(np.abs(arr[finite])))


def _nan_corr(a: np.ndarray, b: np.ndarray) -> float:
    a = np.asarray(a, dtype=np.float64).reshape(-1)
    b = np.asarray(b, dtype=np.float64).reshape(-1)
    mask = np.isfinite(a) & np.isfinite(b)
    if int(mask.sum()) < 2:
        return float("nan")
    aa = a[mask] - float(np.mean(a[mask]))
    bb = b[mask] - float(np.mean(b[mask]))
    den = float(np.linalg.norm(aa) * np.linalg.norm(bb))
    if den <= 1e-12:
        return float("nan")
    return float(np.dot(aa, bb) / den)


def _write_json(path: str | pathlib.Path, data: dict) -> None:
    path = pathlib.Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as fh:
        json.dump(data, fh, indent=2, sort_keys=True)
        fh.write("\n")


def _jsonable(value):
    if isinstance(value, pathlib.Path):
        return str(value)
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    return value


def _write_rows(path: str | pathlib.Path, rows: list[dict], fieldnames: list[str]) -> None:
    path = pathlib.Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _load_ball_physics_config(path: str | None) -> dict:
    import yaml

    cfg_path = pathlib.Path(path).resolve() if path else _default_ball_physics_config(_repo_root())
    with cfg_path.open("r", encoding="utf-8") as fh:
        return yaml.safe_load(fh) or {}


def capture_isaac(args: argparse.Namespace) -> int:
    checkpoint = pathlib.Path(args.checkpoint).resolve()
    if not checkpoint.is_file():
        raise FileNotFoundError(f"checkpoint not found: {checkpoint}")

    # Isaac/Kit must not see this script's argparse subcommand arguments.
    sys.argv = sys.argv[:1]
    from isaaclab.app import AppLauncher

    app_launcher = AppLauncher(headless=True, device=args.device)
    simulation_app = app_launcher.app
    status = 0
    try:
        import gymnasium as gym
        import torch

        from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlVecEnvWrapper
        from isaaclab_tasks.utils import parse_env_cfg

        import whole_body_tracking.tasks  # noqa: F401
        from train import _apply_motion_metadata, _resolve_motion_plan
        from whole_body_tracking.utils.action_adapter_config import (
            load_joint_order,
            resolve_joint_order_mapping,
        )
        from whole_body_tracking.utils.my_on_policy_runner import HOPEOnPolicyRunner
        from whole_body_tracking.utils.ppo_cfg import load_ppo_params, runner_kwargs

        _ensure_reference_importable(args.reference_dir)
        from a3_deploy_onnx_ref_pingpong.config import RuntimeConfig
        from a3_deploy_onnx_ref_pingpong.observation import ObsTarget, RobotState, build_observation

        repo_root = _repo_root()
        runtime_cfg = RuntimeConfig.load(args.runtime_config or _default_runtime_config(repo_root))
        canonical_names = list(load_joint_order())

        motion_cfg = argparse.Namespace(
            motion_manifest=args.motion_manifest,
            motion_files=None,
            motion_file=args.motion_file,
            motion_file_2=args.motion_file_2,
            task={
                "motion_file": args.motion_file,
                "motion_file_2": args.motion_file_2,
            },
        )

        env_cfg = parse_env_cfg(args.task, device=args.device, num_envs=1)
        env_cfg.seed = int(args.seed)
        env_cfg.sim.device = args.device
        if args.disable_randomization and hasattr(env_cfg, "events"):
            for event_name in (
                "physics_material",
                "base_com",
                "link_mass",
                "joint_default_pos",
                "pd_gains",
            ):
                if hasattr(env_cfg.events, event_name):
                    setattr(env_cfg.events, event_name, None)
        clips, motion_metadata = _resolve_motion_plan(motion_cfg)
        env_cfg.commands.motion.motion_file = clips if len(clips) > 1 else clips[0]
        applied: list[str] = []
        _apply_motion_metadata(env_cfg, clips, motion_metadata, applied)

        env_raw = gym.make(args.task, cfg=env_cfg, render_mode=None)
        base_env = env_raw.unwrapped
        env = RslRlVecEnvWrapper(env_raw)

        robot = base_env.scene["robot"]
        articulation_names = list(robot.data.joint_names)
        body_names = list(getattr(robot.data, "body_names", getattr(robot, "body_names", [])) or [])
        body_diag_names = [name for name in FK_DIAGNOSTIC_BODY_NAMES if name in body_names]
        body_diag_ids = [body_names.index(name) for name in body_diag_names]
        mapping = resolve_joint_order_mapping(articulation_names, canonical_joint_names=canonical_names)
        canonical_to_articulation = list(mapping.canonical_to_articulation)

        action_term = base_env.action_manager.get_term(args.action_term)
        action_joint_names = list(getattr(action_term, "_joint_names", canonical_names))
        action_to_canonical = [action_joint_names.index(name) for name in canonical_names]

        agent_cfg = RslRlOnPolicyRunnerCfg(**runner_kwargs(load_ppo_params(), args.experiment_name))
        agent_cfg.device = args.device
        runner = HOPEOnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=args.device)
        runner.load(str(checkpoint))
        policy = runner.get_inference_policy(device=base_env.device)

        cmd = base_env.command_manager.get_term("racket_target")
        env_origins = _as_numpy(base_env.scene.env_origins, row=0)

        def robot_snapshot(prefix: str) -> dict[str, np.ndarray]:
            data = robot.data
            base_pos_w = _as_numpy(_first_attr(data, ["root_pos_w", "root_link_pos_w"]), row=0)
            base_pos_local = base_pos_w - env_origins
            base_quat = _as_numpy(_first_attr(data, ["root_quat_w", "root_link_quat_w"]), row=0)
            base_lin_vel = _optional_numpy(
                _first_attr(data, ["root_lin_vel_w", "root_com_lin_vel_w"]),
                row=0,
                shape=(3,),
            )
            base_ang_vel_b = _optional_numpy(
                _first_attr(data, ["root_ang_vel_b", "root_link_ang_vel_b"]),
                row=0,
                shape=(3,),
            )
            base_ang_vel_w = _optional_numpy(
                _first_attr(data, ["root_ang_vel_w", "root_link_ang_vel_w"]),
                row=0,
                shape=(3,),
            )
            q_art = _as_numpy(data.joint_pos, row=0)
            qd_art = _as_numpy(data.joint_vel, row=0)
            tau_attr = _first_attr(data, ["applied_torque", "computed_torque"])
            tau_art = _optional_numpy(tau_attr, row=0, shape=(len(articulation_names),))
            q = q_art[canonical_to_articulation]
            qd = qd_art[canonical_to_articulation]
            tau = tau_art[canonical_to_articulation]
            return {
                f"base_pos_{prefix}": base_pos_local,
                f"base_quat_{prefix}": base_quat,
                f"base_lin_vel_{prefix}": base_lin_vel,
                f"base_ang_vel_b_{prefix}": base_ang_vel_b,
                f"base_ang_vel_w_{prefix}": base_ang_vel_w,
                f"q_{prefix}": q,
                f"qd_{prefix}": qd,
                f"tau_{prefix}": tau,
            }

        def body_pos_snapshot(prefix: str) -> dict[str, np.ndarray]:
            data = robot.data
            pos_attr = _first_attr(data, ["body_pos_w", "body_link_pos_w"])
            if pos_attr is None or not body_diag_ids:
                return {f"body_pos_{prefix}": np.zeros((0, 3), dtype=np.float64)}
            body_pos_w = _as_numpy(pos_attr, row=0)
            body_pos_local = body_pos_w[body_diag_ids] - env_origins.reshape(1, 3)
            return {f"body_pos_{prefix}": body_pos_local}

        def command_snapshot() -> dict[str, np.ndarray]:
            target_pos_w = _first_attr(cmd, ["racket_target_pos_w", "target_pos_w", "racket_target_w"])
            target_vel_w = _first_attr(cmd, ["racket_target_vel_w", "target_vel_w"])
            time_to_strike = _first_attr(cmd, ["time_to_strike", "tts"])
            swing_side = _first_attr(cmd, ["swing_side", "swing_sign"])
            fixed_station_w = _first_attr(cmd, ["fixed_station_w", "station_w"])
            missing = [
                name
                for name, value in (
                    ("racket_target_pos_w", target_pos_w),
                    ("racket_target_vel_w", target_vel_w),
                    ("time_to_strike", time_to_strike),
                    ("swing_side/swing_sign", swing_side),
                )
                if value is None
            ]
            if missing:
                raise AttributeError(f"racket_target command is missing required fields: {missing}")
            target_pos = _as_numpy(target_pos_w, row=0) - env_origins
            target_vel = _as_numpy(target_vel_w, row=0)
            tts = _as_numpy(time_to_strike, row=0).reshape(-1)[:1]
            swing = _as_numpy(swing_side, row=0).reshape(-1)[:1]
            if fixed_station_w is None:
                fixed_station = np.full(2, np.nan)
            else:
                station = _as_numpy(fixed_station_w, row=0).reshape(-1)
                if station.shape[0] >= 3:
                    fixed_station = (station[:3] - env_origins)[:2]
                elif station.shape[0] == 2:
                    fixed_station = station[:2] - env_origins[:2]
                else:
                    raise ValueError(f"fixed_station_w must have 2 or 3 values, got shape {station.shape}")
            return {
                "target_pos": target_pos,
                "target_vel": target_vel,
                "time_to_strike": tts,
                "swing_side": swing,
                "fixed_station_xy": fixed_station,
            }

        def action_param_snapshot(name: str) -> np.ndarray:
            value = getattr(action_term, name, None)
            if value is None:
                return np.full(31, np.nan, dtype=np.float64)
            if isinstance(value, (float, int)):
                return np.full(31, float(value), dtype=np.float64)
            arr = _as_numpy(value, dtype=np.float64)
            if arr.ndim == 0:
                return np.full(31, float(arr), dtype=np.float64)
            if arr.ndim >= 2:
                arr = arr[0]
            arr = np.asarray(arr, dtype=np.float64).reshape(-1)
            if arr.shape[0] != len(action_joint_names):
                return np.full(31, np.nan, dtype=np.float64)
            return arr[action_to_canonical]

        def actuator_gain_snapshot(name: str) -> np.ndarray:
            values = np.full(len(articulation_names), np.nan, dtype=np.float64)
            actuators = getattr(robot, "actuators", {}) or {}
            for actuator in actuators.values():
                if not hasattr(actuator, name):
                    continue
                raw = _as_numpy(getattr(actuator, name), row=0, dtype=np.float64)
                if raw.ndim == 0:
                    raw = np.full(1, float(raw), dtype=np.float64)
                raw = raw.reshape(-1)
                joint_indices = getattr(actuator, "joint_indices", slice(None))
                if isinstance(joint_indices, slice):
                    ids = list(range(len(articulation_names)))[joint_indices]
                else:
                    ids = [int(i) for i in list(joint_indices)]
                for local_i, joint_id in enumerate(ids):
                    if 0 <= joint_id < len(values) and local_i < len(raw):
                        values[joint_id] = raw[local_i]
            return values[canonical_to_articulation]

        obs, _ = env.get_observations()
        rows = []
        arrays: dict[str, list[np.ndarray]] = {
            "obs_policy": [],
            "obs_rebuilt": [],
            "isaac_raw_action": [],
            "isaac_applied_action": [],
            "isaac_q_des": [],
            "base_pos_pre": [],
            "base_quat_pre": [],
            "base_lin_vel_pre": [],
            "base_ang_vel_b_pre": [],
            "base_ang_vel_w_pre": [],
            "q_pre": [],
            "qd_pre": [],
            "tau_pre": [],
            "base_pos_post": [],
            "base_quat_post": [],
            "base_lin_vel_post": [],
            "base_ang_vel_b_post": [],
            "base_ang_vel_w_post": [],
            "q_post": [],
            "qd_post": [],
            "tau_post": [],
            "target_pos": [],
            "target_vel": [],
            "time_to_strike": [],
            "swing_side": [],
            "fixed_station_xy": [],
            "body_pos_pre": [],
            "body_pos_post": [],
            "isaac_kp": [],
            "isaac_kd": [],
            "isaac_action_offset": [],
            "isaac_action_scale": [],
            "reward": [],
            "done": [],
        }

        for tick in range(int(args.steps)):
            snap_pre = robot_snapshot("pre")
            body_pre = body_pos_snapshot("pre")
            cmd_pre = command_snapshot()
            obs_policy = _as_numpy(obs, row=0, dtype=np.float32).reshape(111)
            fixed_station_xy = cmd_pre["fixed_station_xy"]
            if not np.all(np.isfinite(fixed_station_xy)):
                fixed_station_xy = snap_pre["base_pos_pre"][:2].copy()
            rebuilt = build_observation(
                RobotState(
                    base_pos_w=snap_pre["base_pos_pre"],
                    base_quat_w=snap_pre["base_quat_pre"],
                    base_ang_vel_b=snap_pre["base_ang_vel_b_pre"],
                    q=snap_pre["q_pre"],
                    qd=snap_pre["qd_pre"],
                ),
                ObsTarget(
                    pos_w=cmd_pre["target_pos"],
                    vel_w=cmd_pre["target_vel"],
                    time_to_strike=float(cmd_pre["time_to_strike"][0]),
                    swing_side=float(cmd_pre["swing_side"][0]),
                ),
                obs_policy[65:96],
                runtime_cfg.action_adapter.default_q,
                fixed_station_xy,
            )

            with torch.inference_mode():
                actions = policy(obs)
                raw_action = _as_numpy(actions, row=0)
                obs_next, rewards, dones, _infos = env.step(actions)

            snap_post = robot_snapshot("post")
            body_post = body_pos_snapshot("post")
            raw_canonical = raw_action[action_to_canonical]
            applied_attr = getattr(action_term, "applied_raw_actions", None)
            applied = _as_numpy(applied_attr, row=0)[action_to_canonical] if applied_attr is not None else raw_canonical
            processed_attr = getattr(action_term, "_processed_actions", None)
            if processed_attr is None:
                processed_attr = getattr(action_term, "processed_actions", None)
            if processed_attr is None:
                q_des = runtime_cfg.action_adapter.decode(applied)
                q_des[list(PASSIVE_HEAD_INDICES)] = runtime_cfg.action_adapter.default_q[list(PASSIVE_HEAD_INDICES)]
            else:
                q_des = _as_numpy(processed_attr, row=0)[action_to_canonical]
            action_offset = action_param_snapshot("_offset")
            action_scale = action_param_snapshot("_scale")
            isaac_kp = actuator_gain_snapshot("stiffness")
            isaac_kd = actuator_gain_snapshot("damping")

            reward = _as_numpy(rewards, row=0).reshape(-1)[:1]
            done = _as_numpy(dones, row=0).reshape(-1)[:1].astype(np.float64)

            arrays["obs_policy"].append(obs_policy)
            arrays["obs_rebuilt"].append(rebuilt)
            arrays["isaac_raw_action"].append(raw_canonical)
            arrays["isaac_applied_action"].append(applied)
            arrays["isaac_q_des"].append(q_des)
            for key, value in snap_pre.items():
                arrays[key].append(value)
            for key, value in body_pre.items():
                arrays[key].append(value)
            for key, value in snap_post.items():
                arrays[key].append(value)
            for key, value in body_post.items():
                arrays[key].append(value)
            for key, value in cmd_pre.items():
                arrays[key].append(value)
            arrays["isaac_kp"].append(isaac_kp)
            arrays["isaac_kd"].append(isaac_kd)
            arrays["isaac_action_offset"].append(action_offset)
            arrays["isaac_action_scale"].append(action_scale)
            arrays["reward"].append(reward)
            arrays["done"].append(done)
            rows.append(
                {
                    "tick": tick,
                    "base_z_pre": float(snap_pre["base_pos_pre"][2]),
                    "base_z_post": float(snap_post["base_pos_post"][2]),
                    "raw_action_max_abs": _max_abs(raw_canonical),
                    "applied_action_max_abs": _max_abs(applied),
                    "q_des_max_abs": _max_abs(q_des),
                    "tau_post_max_abs": _max_abs(snap_post["tau_post"]),
                    "obs_policy_vs_rebuilt_max_abs": _max_abs(obs_policy - rebuilt),
                    "action_offset_vs_deploy_default_max_abs": _max_abs(
                        action_offset - runtime_cfg.action_adapter.default_q
                    ),
                    "action_scale_vs_deploy_scale_max_abs": _max_abs(
                        action_scale - runtime_cfg.action_adapter.action_scale
                    ),
                    "isaac_kp_vs_deploy_max_abs": _max_abs(isaac_kp - runtime_cfg.sim_kp),
                    "isaac_kd_vs_deploy_max_abs": _max_abs(isaac_kd - runtime_cfg.sim_kd),
                    "time_to_strike": float(cmd_pre["time_to_strike"][0]),
                    "swing_side": float(cmd_pre["swing_side"][0]),
                    "reward": float(reward[0]),
                    "done": int(done[0]),
                }
            )
            obs = obs_next

        save_arrays = {key: np.stack(value, axis=0) for key, value in arrays.items()}
        meta = {
            "checkpoint": str(checkpoint),
            "task": args.task,
            "steps": int(args.steps),
            "device": args.device,
            "experiment_name": args.experiment_name,
            "motion_manifest": args.motion_manifest,
            "motion_files": clips,
            "motion_metadata_applied": applied,
            "disable_randomization": bool(args.disable_randomization),
            "runtime_config": str(runtime_cfg.config_dir / "hope_pingpong_runtime.yaml"),
            "joint_names": canonical_names,
            "articulation_joint_names": articulation_names,
            "action_joint_names": action_joint_names,
            "body_names": body_names,
            "fk_diagnostic_body_names": body_diag_names,
            "canonical_to_articulation": canonical_to_articulation,
            "action_to_canonical": action_to_canonical,
        }
        save_arrays["joint_names"] = np.asarray(canonical_names)
        save_arrays["fk_body_names"] = np.asarray(body_diag_names)
        meta = _jsonable(meta)
        save_arrays["meta_json"] = np.asarray(json.dumps(meta, sort_keys=True))
        out_npz = pathlib.Path(args.out_npz)
        out_npz.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(out_npz, **save_arrays)
        if args.out_csv:
            _write_rows(args.out_csv, rows, list(rows[0].keys()) if rows else ["tick"])
        print(json.dumps({"out_npz": str(out_npz), "out_csv": args.out_csv, "meta": meta}))
        env.close()
    except Exception:
        import traceback

        print("\n[diagnose_isaac_mujoco_step] ERROR during Isaac capture:", flush=True)
        traceback.print_exc()
        status = 1
    finally:
        simulation_app.close()
    return status


def _set_mujoco_state(scene, base_pos, base_quat, base_lin_vel, base_ang_vel_b, q, qd) -> None:
    d = scene.data
    d.qpos[scene._base_qadr:scene._base_qadr + 3] = np.asarray(base_pos, dtype=np.float64)
    d.qpos[scene._base_qadr + 3:scene._base_qadr + 7] = np.asarray(base_quat, dtype=np.float64)
    d.qvel[scene._base_vadr:scene._base_vadr + 3] = np.asarray(base_lin_vel, dtype=np.float64)
    d.qvel[scene._base_vadr + 3:scene._base_vadr + 6] = np.asarray(base_ang_vel_b, dtype=np.float64)
    d.qpos[scene._q_adr] = np.asarray(q, dtype=np.float64)
    d.qvel[scene._v_adr] = np.asarray(qd, dtype=np.float64)
    scene._mj.mj_forward(scene.model, scene.data)


def _mujoco_obj_id(scene, obj_type, name: str) -> int:
    return int(scene._mj.mj_name2id(scene.model, obj_type, name))


def _mujoco_pd_torque(scene, q_des: np.ndarray, kp: np.ndarray, kd: np.ndarray) -> tuple[np.ndarray, np.ndarray, int]:
    q = scene.data.qpos[scene._q_adr]
    qd = scene.data.qvel[scene._v_adr]
    tau_unclipped = kp * (np.asarray(q_des, dtype=np.float64) - q) - kd * qd
    scene.write_targets(q_des, kp, kd)
    scene._apply_pd()
    tau = scene.data.ctrl[scene._act_idx].copy()
    clip_count = int(np.count_nonzero(np.abs(tau - tau_unclipped) > 1e-9))
    return tau, tau_unclipped.copy(), clip_count


def compare_mujoco(args: argparse.Namespace) -> int:
    repo_root = _repo_root()
    _ensure_reference_importable(args.reference_dir)
    from a3_deploy_onnx_ref_pingpong.config import RuntimeConfig
    from a3_deploy_onnx_ref_pingpong.joint_order import HEAD_INDICES, JOINT_NAMES
    from a3_deploy_onnx_ref_pingpong.observation import ObsTarget, build_observation
    from a3_deploy_onnx_ref_pingpong.onnx_policy import OnnxPolicy
    from mujoco_eval_onnx import _obs_for_policy_joint_order, _raw_action_to_canonical
    from mujoco_pingpong_scene import PingPongRealPhysicsScene

    runtime_cfg = RuntimeConfig.load(args.runtime_config or _default_runtime_config(repo_root))
    onnx_path = pathlib.Path(args.onnx or runtime_cfg.onnx_path).resolve()
    if not onnx_path.is_file():
        raise FileNotFoundError(f"ONNX policy not found: {onnx_path}")
    robot_xml = pathlib.Path(args.model_xml or runtime_cfg.model_xml_path).resolve()
    ball_cfg = _load_ball_physics_config(args.ball_physics_config)
    policy = OnnxPolicy(onnx_path)
    scene_replay = PingPongRealPhysicsScene(
        str(robot_xml),
        ball_cfg,
        JOINT_NAMES,
        control_dt=runtime_cfg.control_dt,
        near_edge_x=args.near_edge_x,
        launch_viewer=False,
    )
    scene_closed = PingPongRealPhysicsScene(
        str(robot_xml),
        ball_cfg,
        JOINT_NAMES,
        control_dt=runtime_cfg.control_dt,
        near_edge_x=args.near_edge_x,
        launch_viewer=False,
    )

    npz = np.load(args.isaac_npz, allow_pickle=False)
    joint_names = [str(x) for x in npz["joint_names"].tolist()]
    if joint_names != list(JOINT_NAMES):
        raise RuntimeError(f"Isaac NPZ joint order != deploy joint order: {joint_names} vs {list(JOINT_NAMES)}")

    obs_policy = np.asarray(npz["obs_policy"], dtype=np.float32)
    obs_rebuilt = np.asarray(npz["obs_rebuilt"], dtype=np.float32)
    isaac_raw = np.asarray(npz["isaac_raw_action"], dtype=np.float64)
    isaac_applied = np.asarray(npz["isaac_applied_action"], dtype=np.float64)
    isaac_q_des = np.asarray(npz["isaac_q_des"], dtype=np.float64)
    q_pre = np.asarray(npz["q_pre"], dtype=np.float64)
    qd_pre = np.asarray(npz["qd_pre"], dtype=np.float64)
    tau_post = np.asarray(npz["tau_post"], dtype=np.float64)
    base_pos_pre = np.asarray(npz["base_pos_pre"], dtype=np.float64)
    base_quat_pre = np.asarray(npz["base_quat_pre"], dtype=np.float64)
    base_lin_vel_pre = np.asarray(npz["base_lin_vel_pre"], dtype=np.float64)
    base_ang_vel_b_pre = np.asarray(npz["base_ang_vel_b_pre"], dtype=np.float64)
    target_pos = np.asarray(npz["target_pos"], dtype=np.float64)
    target_vel = np.asarray(npz["target_vel"], dtype=np.float64)
    time_to_strike = np.asarray(npz["time_to_strike"], dtype=np.float64).reshape(-1)
    swing_side = np.asarray(npz["swing_side"], dtype=np.float64).reshape(-1)
    fixed_station_xy = np.asarray(npz["fixed_station_xy"], dtype=np.float64)
    isaac_body_pos_pre = (
        np.asarray(npz["body_pos_pre"], dtype=np.float64)
        if "body_pos_pre" in npz and "fk_body_names" in npz
        else None
    )
    fk_body_names = (
        [str(x) for x in npz["fk_body_names"].tolist()]
        if "fk_body_names" in npz
        else []
    )
    isaac_action_offset = (
        np.asarray(npz["isaac_action_offset"], dtype=np.float64)
        if "isaac_action_offset" in npz
        else np.full_like(isaac_q_des, np.nan)
    )
    isaac_action_scale = (
        np.asarray(npz["isaac_action_scale"], dtype=np.float64)
        if "isaac_action_scale" in npz
        else np.full_like(isaac_q_des, np.nan)
    )
    isaac_kp = (
        np.asarray(npz["isaac_kp"], dtype=np.float64)
        if "isaac_kp" in npz
        else None
    )
    isaac_kd = (
        np.asarray(npz["isaac_kd"], dtype=np.float64)
        if "isaac_kd" in npz
        else None
    )
    if args.gain_source == "isaac" and (isaac_kp is None or isaac_kd is None):
        raise RuntimeError("--gain-source isaac requires an NPZ captured with isaac_kp/isaac_kd")
    done = np.asarray(npz["done"], dtype=np.float64).reshape(-1) if "done" in npz else np.zeros(len(obs_policy))

    steps = min(int(args.steps or len(obs_policy)), len(obs_policy))
    kp_runtime = runtime_cfg.sim_kp.copy()
    kd_runtime = runtime_cfg.sim_kd.copy()
    kp = kp_runtime * float(args.kp_scale)
    kd = kd_runtime * float(args.kd_scale)
    default_q = runtime_cfg.action_adapter.default_q.copy()
    head_idx = list(HEAD_INDICES)
    last_action_closed = np.zeros(31, dtype=np.float64)
    fixed_station = fixed_station_xy[0].copy()
    if not np.all(np.isfinite(fixed_station)):
        fixed_station = scene_closed.base_pos_w()[:2].copy()

    step_rows: list[dict] = []
    joint_rows: list[dict] = []
    onnx_raw_isaac = np.zeros((steps, 31), dtype=np.float64)
    onnx_applied_isaac = np.zeros((steps, 31), dtype=np.float64)
    onnx_qdes_isaac = np.zeros((steps, 31), dtype=np.float64)
    mujoco_replay_tau = np.zeros((steps, 31), dtype=np.float64)
    mujoco_replay_tau_unclipped = np.zeros((steps, 31), dtype=np.float64)
    mujoco_closed_raw = np.zeros((steps, 31), dtype=np.float64)
    mujoco_closed_applied = np.zeros((steps, 31), dtype=np.float64)
    mujoco_closed_qdes = np.zeros((steps, 31), dtype=np.float64)
    mujoco_closed_tau = np.zeros((steps, 31), dtype=np.float64)
    mujoco_closed_obs = np.zeros((steps, 111), dtype=np.float32)
    mujoco_base_pre = np.zeros((steps, 3), dtype=np.float64)
    mujoco_base_post = np.zeros((steps, 3), dtype=np.float64)
    fk_body_pos_max_abs = np.full(steps, np.nan, dtype=np.float64)
    fk_body_pos_mean_abs = np.full(steps, np.nan, dtype=np.float64)
    fk_body_count = np.zeros(steps, dtype=np.int64)
    fk_racket_dist = np.full(steps, np.nan, dtype=np.float64)
    fk_left_foot_dist = np.full(steps, np.nan, dtype=np.float64)
    fk_right_foot_dist = np.full(steps, np.nan, dtype=np.float64)
    fk_right_wrist_dist = np.full(steps, np.nan, dtype=np.float64)
    replay_clip_counts = np.zeros(steps, dtype=np.int64)
    closed_clip_counts = np.zeros(steps, dtype=np.int64)

    scene_replay.reset_stand()
    scene_closed.reset_stand()
    mujoco_body_ids = []
    if isaac_body_pos_pre is not None:
        for name in fk_body_names:
            mujoco_body_ids.append(_mujoco_obj_id(scene_replay, scene_replay._mj.mjtObj.mjOBJ_BODY, name))
    racket_site_id = _mujoco_obj_id(scene_replay, scene_replay._mj.mjtObj.mjOBJ_SITE, "right_racket")
    isaac_racket_body_index = fk_body_names.index("pingpang_red_Link") if "pingpang_red_Link" in fk_body_names else -1
    if steps:
        _set_mujoco_state(
            scene_closed,
            base_pos_pre[0],
            base_quat_pre[0],
            base_lin_vel_pre[0],
            base_ang_vel_b_pre[0],
            q_pre[0],
            qd_pre[0],
        )
    for tick in range(steps):
        policy_obs = _obs_for_policy_joint_order(
            obs_policy[tick],
            obs_policy[tick, 65:96],
            args.policy_joint_order,
        )
        raw_on_isaac = _raw_action_to_canonical(policy.infer(policy_obs), args.policy_joint_order)
        applied_on_isaac = raw_on_isaac.copy()
        if runtime_cfg.passive_neck:
            applied_on_isaac[head_idx] = 0.0
        qdes_on_isaac = runtime_cfg.action_adapter.decode(applied_on_isaac)
        if runtime_cfg.passive_neck:
            qdes_on_isaac[head_idx] = default_q[head_idx]
        isaac_qdes_formula = isaac_action_offset[tick] + isaac_applied[tick] * isaac_action_scale[tick]
        isaac_qdes_formula = np.clip(
            isaac_qdes_formula,
            runtime_cfg.action_adapter.clamp_lower,
            runtime_cfg.action_adapter.clamp_upper,
        )
        if runtime_cfg.passive_neck:
            isaac_qdes_formula[head_idx] = default_q[head_idx]
        kp_tick = kp
        kd_tick = kd
        if args.gain_source == "isaac":
            kp_tick = np.asarray(isaac_kp[tick], dtype=np.float64) * float(args.kp_scale)
            kd_tick = np.asarray(isaac_kd[tick], dtype=np.float64) * float(args.kd_scale)

        _set_mujoco_state(
            scene_replay,
            base_pos_pre[tick],
            base_quat_pre[tick],
            base_lin_vel_pre[tick],
            base_ang_vel_b_pre[tick],
            q_pre[tick],
            qd_pre[tick],
        )
        tau_replay, tau_replay_unclipped, replay_clip = _mujoco_pd_torque(
            scene_replay, isaac_q_des[tick], kp_tick, kd_tick
        )
        if isaac_body_pos_pre is not None and fk_body_names:
            common_pos = []
            common_ref = []
            for i, bid in enumerate(mujoco_body_ids):
                if bid >= 0:
                    common_pos.append(scene_replay.data.xpos[bid].copy())
                    common_ref.append(isaac_body_pos_pre[tick, i].copy())
            if common_pos:
                common_pos = np.asarray(common_pos, dtype=np.float64)
                common_ref = np.asarray(common_ref, dtype=np.float64)
                fk_diff = common_pos - common_ref
                fk_body_pos_max_abs[tick] = _max_abs(fk_diff)
                fk_body_pos_mean_abs[tick] = _mean_abs(fk_diff)
                fk_body_count[tick] = len(common_pos)
            if isaac_racket_body_index >= 0 and racket_site_id >= 0:
                fk_racket_dist[tick] = float(
                    np.linalg.norm(
                        scene_replay.data.site_xpos[racket_site_id]
                        - isaac_body_pos_pre[tick, isaac_racket_body_index]
                    )
                )
            for metric, body_name in (
                (fk_left_foot_dist, "left_ankle_roll_Link"),
                (fk_right_foot_dist, "right_ankle_roll_Link"),
                (fk_right_wrist_dist, "right_wrist_yaw_Link"),
            ):
                if body_name in fk_body_names:
                    i = fk_body_names.index(body_name)
                    bid = mujoco_body_ids[i]
                    if bid >= 0:
                        metric[tick] = float(
                            np.linalg.norm(scene_replay.data.xpos[bid] - isaac_body_pos_pre[tick, i])
                        )

        # Closed MuJoCo tick: same captured target command, MuJoCo state evolves
        # from its own previous state, and last_action is the deploy applied action.
        state_closed = scene_closed.read_robot_state()
        obs_closed = build_observation(
            state_closed,
            ObsTarget(
                pos_w=target_pos[tick],
                vel_w=target_vel[tick],
                time_to_strike=float(time_to_strike[tick]),
                swing_side=float(swing_side[tick]),
            ),
            last_action_closed,
            default_q,
            fixed_station,
        )
        policy_obs_closed = _obs_for_policy_joint_order(
            obs_closed,
            last_action_closed,
            args.policy_joint_order,
        )
        raw_closed = _raw_action_to_canonical(policy.infer(policy_obs_closed), args.policy_joint_order)
        applied_closed = raw_closed.copy()
        if runtime_cfg.passive_neck:
            applied_closed[head_idx] = 0.0
        qdes_closed = runtime_cfg.action_adapter.decode(applied_closed)
        if runtime_cfg.passive_neck:
            qdes_closed[head_idx] = default_q[head_idx]
        tau_closed, _tau_closed_unclipped, closed_clip = _mujoco_pd_torque(
            scene_closed, qdes_closed, kp_tick, kd_tick
        )
        mujoco_base_pre[tick] = scene_closed.base_pos_w()
        scene_closed.step()
        mujoco_base_post[tick] = scene_closed.base_pos_w()
        last_action_closed = applied_closed

        onnx_raw_isaac[tick] = raw_on_isaac
        onnx_applied_isaac[tick] = applied_on_isaac
        onnx_qdes_isaac[tick] = qdes_on_isaac
        mujoco_replay_tau[tick] = tau_replay
        mujoco_replay_tau_unclipped[tick] = tau_replay_unclipped
        mujoco_closed_raw[tick] = raw_closed
        mujoco_closed_applied[tick] = applied_closed
        mujoco_closed_qdes[tick] = qdes_closed
        mujoco_closed_tau[tick] = tau_closed
        mujoco_closed_obs[tick] = obs_closed
        replay_clip_counts[tick] = replay_clip
        closed_clip_counts[tick] = closed_clip

        step_row = {
            "tick": tick,
            "time_to_strike": float(time_to_strike[tick]),
            "swing_side": float(swing_side[tick]),
            "isaac_done": int(done[tick]) if tick < len(done) else 0,
            "isaac_base_x": float(base_pos_pre[tick, 0]),
            "isaac_base_y": float(base_pos_pre[tick, 1]),
            "isaac_base_z": float(base_pos_pre[tick, 2]),
            "mujoco_base_x": float(mujoco_base_pre[tick, 0]),
            "mujoco_base_y": float(mujoco_base_pre[tick, 1]),
            "mujoco_base_z": float(mujoco_base_pre[tick, 2]),
            "mujoco_base_z_post": float(mujoco_base_post[tick, 2]),
            "base_pos_diff_max_abs": _max_abs(mujoco_base_pre[tick] - base_pos_pre[tick]),
            "isaac_raw_max_abs": _max_abs(isaac_raw[tick]),
            "onnx_raw_on_isaac_obs_max_abs": _max_abs(raw_on_isaac),
            "mujoco_raw_max_abs": _max_abs(raw_closed),
            "onnx_vs_isaac_raw_max_abs": _max_abs(raw_on_isaac - isaac_raw[tick]),
            "mujoco_vs_isaac_raw_max_abs": _max_abs(raw_closed - isaac_raw[tick]),
            "isaac_qdes_max_abs": _max_abs(isaac_q_des[tick]),
            "onnx_qdes_on_isaac_obs_max_abs": _max_abs(qdes_on_isaac),
            "mujoco_qdes_max_abs": _max_abs(qdes_closed),
            "onnx_vs_isaac_qdes_max_abs": _max_abs(qdes_on_isaac - isaac_q_des[tick]),
            "mujoco_vs_isaac_qdes_max_abs": _max_abs(qdes_closed - isaac_q_des[tick]),
            "isaac_tau_max_abs": _max_abs(tau_post[tick]),
            "mujoco_replay_tau_max_abs": _max_abs(tau_replay),
            "mujoco_replay_tau_unclipped_max_abs": _max_abs(tau_replay_unclipped),
            "mujoco_closed_tau_max_abs": _max_abs(tau_closed),
            "replay_tau_vs_isaac_tau_max_abs": _max_abs(tau_replay - tau_post[tick]),
            "replay_tau_clip_count": int(replay_clip),
            "closed_tau_clip_count": int(closed_clip),
            "isaac_policy_obs_vs_rebuilt_max_abs": _max_abs(obs_policy[tick] - obs_rebuilt[tick]),
            "mujoco_obs_vs_isaac_rebuilt_max_abs": _max_abs(obs_closed - obs_rebuilt[tick]),
            "isaac_action_offset_vs_deploy_default_max_abs": _max_abs(
                isaac_action_offset[tick] - default_q
            ),
            "isaac_action_scale_vs_deploy_scale_max_abs": _max_abs(
                isaac_action_scale[tick] - runtime_cfg.action_adapter.action_scale
            ),
            "isaac_qdes_formula_vs_processed_max_abs": _max_abs(
                isaac_qdes_formula - isaac_q_des[tick]
            ),
            "isaac_kp_vs_runtime_max_abs": _max_abs(
                isaac_kp[tick] - kp_runtime
            ) if isaac_kp is not None else float("nan"),
            "isaac_kd_vs_runtime_max_abs": _max_abs(
                isaac_kd[tick] - kd_runtime
            ) if isaac_kd is not None else float("nan"),
            "mujoco_fallen": int(scene_closed.base_fallen()),
            "fk_body_pos_max_abs": float(fk_body_pos_max_abs[tick]),
            "fk_body_pos_mean_abs": float(fk_body_pos_mean_abs[tick]),
            "fk_common_body_count": int(fk_body_count[tick]),
            "fk_racket_site_vs_isaac_racket_body_dist": float(fk_racket_dist[tick]),
            "fk_left_foot_dist": float(fk_left_foot_dist[tick]),
            "fk_right_foot_dist": float(fk_right_foot_dist[tick]),
            "fk_right_wrist_dist": float(fk_right_wrist_dist[tick]),
        }
        step_rows.append(step_row)

        for j, name in enumerate(joint_names):
            joint_rows.append(
                {
                    "tick": tick,
                    "joint_index": j,
                    "joint": name,
                    "isaac_raw_action": float(isaac_raw[tick, j]),
                    "onnx_raw_on_isaac_obs": float(raw_on_isaac[j]),
                    "mujoco_raw_action": float(raw_closed[j]),
                    "isaac_applied_action": float(isaac_applied[tick, j]),
                    "onnx_applied_on_isaac_obs": float(applied_on_isaac[j]),
                    "mujoco_applied_action": float(applied_closed[j]),
                    "isaac_q": float(q_pre[tick, j]),
                    "isaac_qd": float(qd_pre[tick, j]),
                    "isaac_action_offset": float(isaac_action_offset[tick, j]),
                    "deploy_action_default_q": float(default_q[j]),
                    "isaac_action_scale": float(isaac_action_scale[tick, j]),
                    "deploy_action_scale": float(runtime_cfg.action_adapter.action_scale[j]),
                    "isaac_kp": float(isaac_kp[tick, j]) if isaac_kp is not None else float("nan"),
                    "isaac_kd": float(isaac_kd[tick, j]) if isaac_kd is not None else float("nan"),
                    "runtime_kp": float(kp_runtime[j]),
                    "runtime_kd": float(kd_runtime[j]),
                    "mujoco_kp": float(kp_tick[j]),
                    "mujoco_kd": float(kd_tick[j]),
                    "isaac_q_des": float(isaac_q_des[tick, j]),
                    "isaac_q_des_from_offset_scale": float(isaac_qdes_formula[j]),
                    "onnx_q_des_on_isaac_obs": float(qdes_on_isaac[j]),
                    "mujoco_q_des": float(qdes_closed[j]),
                    "isaac_tau": float(tau_post[tick, j]),
                    "mujoco_replay_tau": float(tau_replay[j]),
                    "mujoco_replay_tau_unclipped": float(tau_replay_unclipped[j]),
                    "mujoco_closed_tau": float(tau_closed[j]),
                    "target_pos_x": float(target_pos[tick, 0]),
                    "target_pos_y": float(target_pos[tick, 1]),
                    "target_pos_z": float(target_pos[tick, 2]),
                    "target_vel_x": float(target_vel[tick, 0]),
                    "target_vel_y": float(target_vel[tick, 1]),
                    "target_vel_z": float(target_vel[tick, 2]),
                    "time_to_strike": float(time_to_strike[tick]),
                    "swing_side": float(swing_side[tick]),
                    "isaac_base_x": float(base_pos_pre[tick, 0]),
                    "isaac_base_y": float(base_pos_pre[tick, 1]),
                    "isaac_base_z": float(base_pos_pre[tick, 2]),
                    "mujoco_base_x": float(mujoco_base_pre[tick, 0]),
                    "mujoco_base_y": float(mujoco_base_pre[tick, 1]),
                    "mujoco_base_z": float(mujoco_base_pre[tick, 2]),
                }
            )

    first_done = next((i for i, v in enumerate(done[:steps]) if v > 0.5), None)
    first_mujoco_fall = next(
        (i for i in range(steps) if float(mujoco_base_post[i, 2]) < float(args.fall_base_z)),
        None,
    )
    isaac_qdes_formula_all = np.clip(
        isaac_action_offset[:steps] + isaac_applied[:steps] * isaac_action_scale[:steps],
        runtime_cfg.action_adapter.clamp_lower.reshape(1, -1),
        runtime_cfg.action_adapter.clamp_upper.reshape(1, -1),
    )
    if runtime_cfg.passive_neck:
        isaac_qdes_formula_all[:, head_idx] = default_q[head_idx]
    summary = {
        "isaac_npz": str(pathlib.Path(args.isaac_npz).resolve()),
        "onnx": str(onnx_path),
        "robot_xml": str(robot_xml),
        "steps": steps,
        "policy_joint_order": args.policy_joint_order,
        "gain_source": args.gain_source,
        "kp_scale": float(args.kp_scale),
        "kd_scale": float(args.kd_scale),
        "first_isaac_done_tick": first_done,
        "first_mujoco_fall_tick": first_mujoco_fall,
        "max_abs_onnx_vs_isaac_raw": _max_abs(onnx_raw_isaac - isaac_raw[:steps]),
        "mean_abs_onnx_vs_isaac_raw": _mean_abs(onnx_raw_isaac - isaac_raw[:steps]),
        "max_abs_onnx_vs_isaac_qdes": _max_abs(onnx_qdes_isaac - isaac_q_des[:steps]),
        "mean_abs_onnx_vs_isaac_qdes": _mean_abs(onnx_qdes_isaac - isaac_q_des[:steps]),
        "max_abs_mujoco_vs_isaac_raw": _max_abs(mujoco_closed_raw - isaac_raw[:steps]),
        "max_abs_mujoco_vs_isaac_qdes": _max_abs(mujoco_closed_qdes - isaac_q_des[:steps]),
        "max_abs_isaac_policy_obs_vs_rebuilt": _max_abs(obs_policy[:steps] - obs_rebuilt[:steps]),
        "max_abs_mujoco_obs_vs_isaac_rebuilt": _max_abs(mujoco_closed_obs - obs_rebuilt[:steps]),
        "max_abs_isaac_action_offset_vs_deploy_default": _max_abs(
            isaac_action_offset[:steps] - default_q.reshape(1, -1)
        ),
        "max_abs_isaac_action_scale_vs_deploy_scale": _max_abs(
            isaac_action_scale[:steps] - runtime_cfg.action_adapter.action_scale.reshape(1, -1)
        ),
        "max_abs_isaac_qdes_formula_vs_processed": _max_abs(
            isaac_qdes_formula_all - isaac_q_des[:steps]
        ),
        "max_abs_isaac_kp_vs_runtime": _max_abs(
            isaac_kp[:steps] - kp_runtime.reshape(1, -1)
        ) if isaac_kp is not None else float("nan"),
        "max_abs_isaac_kd_vs_runtime": _max_abs(
            isaac_kd[:steps] - kd_runtime.reshape(1, -1)
        ) if isaac_kd is not None else float("nan"),
        "max_abs_replay_tau_vs_isaac_tau": _max_abs(mujoco_replay_tau - tau_post[:steps]),
        "mean_abs_replay_tau_vs_isaac_tau": _mean_abs(mujoco_replay_tau - tau_post[:steps]),
        "corr_replay_tau_vs_isaac_tau": _nan_corr(mujoco_replay_tau, tau_post[:steps]),
        "max_replay_tau_clip_count": int(np.max(replay_clip_counts)) if steps else 0,
        "max_closed_tau_clip_count": int(np.max(closed_clip_counts)) if steps else 0,
        "max_abs_fk_body_pos": _max_abs(fk_body_pos_max_abs),
        "mean_abs_fk_body_pos": _mean_abs(fk_body_pos_mean_abs),
        "max_fk_racket_site_vs_isaac_racket_body_dist": _max_abs(fk_racket_dist),
        "max_fk_left_foot_dist": _max_abs(fk_left_foot_dist),
        "max_fk_right_foot_dist": _max_abs(fk_right_foot_dist),
        "max_fk_right_wrist_dist": _max_abs(fk_right_wrist_dist),
        "isaac_base_z_first": float(base_pos_pre[0, 2]) if steps else None,
        "isaac_base_z_last": float(base_pos_pre[steps - 1, 2]) if steps else None,
        "mujoco_base_z_first": float(mujoco_base_pre[0, 2]) if steps else None,
        "mujoco_base_z_last": float(mujoco_base_post[steps - 1, 2]) if steps else None,
    }

    _write_rows(args.out_csv, step_rows, list(step_rows[0].keys()) if step_rows else ["tick"])
    if args.out_joints_csv:
        _write_rows(args.out_joints_csv, joint_rows, list(joint_rows[0].keys()) if joint_rows else ["tick"])
    if args.out_json:
        _write_json(args.out_json, summary)
    print(json.dumps(summary))
    scene_replay.close()
    scene_closed.close()
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    cap = sub.add_parser("capture-isaac", help="Capture one-env Isaac single-step diagnostics.")
    cap.add_argument("--checkpoint", required=True)
    cap.add_argument("--task", default="HOPE-PingPong-AgibotA3-v0")
    cap.add_argument("--steps", type=int, default=80)
    cap.add_argument("--device", default="cuda:0")
    cap.add_argument("--seed", type=int, default=0)
    cap.add_argument("--experiment-name", default="hope_pingpong")
    cap.add_argument("--runtime-config", default=None)
    cap.add_argument("--reference-dir", default=None)
    cap.add_argument("--action-term", default="joint_pos")
    cap.add_argument("--motion-manifest", default=None)
    cap.add_argument("--motion-file", default="hope_training/motions/preprocessed/hope_forehand.npz")
    cap.add_argument("--motion-file-2", default=None)
    cap.add_argument("--disable-randomization", action="store_true")
    cap.add_argument("--out-npz", required=True)
    cap.add_argument("--out-csv", default=None)
    cap.set_defaults(func=capture_isaac)

    cmp = sub.add_parser("compare-mujoco", help="Replay captured Isaac samples in MuJoCo/ONNX.")
    cmp.add_argument("--isaac-npz", required=True)
    cmp.add_argument("--onnx", default=None)
    cmp.add_argument("--model-xml", default=None)
    cmp.add_argument("--runtime-config", default=None)
    cmp.add_argument("--reference-dir", default=None)
    cmp.add_argument("--ball-physics-config", default=None)
    cmp.add_argument("--steps", type=int, default=None)
    cmp.add_argument(
        "--policy-joint-order",
        choices=["canonical", "isaac-articulation-action", "isaac-articulation-obs-action"],
        default="canonical",
    )
    cmp.add_argument("--gain-source", choices=["runtime", "isaac"], default="runtime")
    cmp.add_argument("--kp-scale", type=float, default=1.0)
    cmp.add_argument("--kd-scale", type=float, default=1.0)
    cmp.add_argument("--near-edge-x", type=float, default=0.30)
    cmp.add_argument("--fall-base-z", type=float, default=0.40)
    cmp.add_argument("--out-csv", required=True)
    cmp.add_argument("--out-joints-csv", default=None)
    cmp.add_argument("--out-json", default=None)
    cmp.set_defaults(func=compare_mujoco)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
