"""Align retargeted motion clips to the A3 body model.

This script keeps the canonical 31-DOF joint trajectory from each input clip, but regenerates the
tracked-body positions/orientations from the live Isaac A3 articulation.  The output body reference
is therefore consistent with the joint reference used for RSI resets and imitation rewards.

Alignment policy:
* compute a dataset-level source-to-A3 vertical scale from feet -> core/shoulder height;
* preserve each clip's root XY trajectory relative to its ready-frame root, scaled by that factor;
* place the ready-frame root XY at the A3 default root XY;
* optionally rotate the ready-frame root yaw to the A3 default yaw;
* shift each frame vertically so the lower A3 ankle sits at the default A3 ankle clearance.

Optionally, the script can stabilize unreliable lower-body retargeting from monocular videos while
leaving the upper-body swing untouched.  This is an explicit data-pipeline correction, not a policy
or reward change.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Any

import numpy as np
import torch
import yaml


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True, type=Path, help="Input TSV manifest.")
    parser.add_argument("--output-dir", required=True, type=Path, help="Directory for aligned clips.")
    parser.add_argument("--output-manifest", type=Path, default=None, help="Output TSV manifest path.")
    parser.add_argument("--device", default="cuda:0", help="Isaac device.")
    parser.add_argument("--task", default="HOPE-PingPong-AgibotA3-v0", help="Gym task id.")
    parser.add_argument("--target-ground-z", type=float, default=None, help="Override target ankle clearance.")
    parser.add_argument("--ready-frames", type=int, default=10, help="Frames used for ready root XY averaging.")
    parser.add_argument(
        "--root-yaw-mode",
        choices=("preserve", "ready_to_default"),
        default="preserve",
        help="preserve keeps source yaw; ready_to_default rotates ready-frame root yaw to the A3 default.",
    )
    parser.add_argument(
        "--lower-body-mode",
        choices=("preserve", "stabilize"),
        default="preserve",
        help="preserve keeps input lower-body joints; stabilize blends/clamps unreliable leg retargeting.",
    )
    parser.add_argument(
        "--lower-body-blend",
        type=float,
        default=0.2,
        help="When stabilizing, fraction of source lower-body deviation from the A3 default to keep.",
    )
    return parser.parse_args()


def _load_manifest(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open("r", encoding="utf-8", newline="") as fh:
        reader = csv.DictReader(fh, delimiter="\t")
        return list(reader.fieldnames or []), list(reader)


def _clip_path(row: dict[str, str]) -> Path:
    value = row.get("output") or row.get("motion_file") or row.get("path") or row.get("file")
    if not value:
        raise ValueError(f"Manifest row has no output/motion_file/path/file column: {row}")
    return Path(value).expanduser().resolve()


def _as_f32(array: np.ndarray) -> np.ndarray:
    return np.asarray(array, dtype=np.float32)


def _normalize_quat(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64)
    norm = np.linalg.norm(q, axis=-1, keepdims=True)
    return (q / np.maximum(norm, 1e-12)).astype(np.float32)


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = np.moveaxis(a, -1, 0)
    bw, bx, by, bz = np.moveaxis(b, -1, 0)
    return np.stack(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        axis=-1,
    )


def _quat_inv(q: np.ndarray) -> np.ndarray:
    out = q.copy()
    out[..., 1:] *= -1.0
    return out


def _quat_yaw(q: np.ndarray) -> np.ndarray:
    q = _normalize_quat(q).astype(np.float64)
    w, x, y, z = np.moveaxis(q, -1, 0)
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _circular_mean(angles: np.ndarray) -> float:
    return float(np.arctan2(np.sin(angles).mean(), np.cos(angles).mean()))


def _yaw_quat(yaw: float) -> np.ndarray:
    half = 0.5 * float(yaw)
    return np.array([np.cos(half), 0.0, 0.0, np.sin(half)], dtype=np.float32)


def _rotate_xy(xy: np.ndarray, yaw: float) -> np.ndarray:
    c = float(np.cos(yaw))
    s = float(np.sin(yaw))
    out = xy.copy()
    out[..., 0] = c * xy[..., 0] - s * xy[..., 1]
    out[..., 1] = s * xy[..., 0] + c * xy[..., 1]
    return out


def _angular_velocity_from_quat(q: np.ndarray, dt: float) -> np.ndarray:
    q = _normalize_quat(q)
    prev = np.concatenate([q[:1], q[:-1]], axis=0)
    nxt = np.concatenate([q[1:], q[-1:]], axis=0)
    span = np.full((q.shape[0],), 2.0 * dt, dtype=np.float64)
    span[0] = dt
    span[-1] = dt

    dq = _normalize_quat(_quat_mul(nxt, _quat_inv(prev)))
    sign = np.where(dq[..., :1] < 0.0, -1.0, 1.0)
    dq = dq * sign
    vec = dq[..., 1:]
    vec_norm = np.linalg.norm(vec, axis=-1, keepdims=True)
    angle = 2.0 * np.arctan2(vec_norm, np.clip(dq[..., :1], -1.0, 1.0))
    axis = vec / np.maximum(vec_norm, 1e-12)
    omega = axis * angle / span[:, None, None]
    omega[vec_norm[..., 0] < 1e-9] = 0.0
    return omega.astype(np.float32)


def _linear_velocity(pos: np.ndarray, dt: float) -> np.ndarray:
    return np.gradient(pos.astype(np.float64), dt, axis=0, edge_order=1).astype(np.float32)


def _dataset_vertical_scale(files: list[Path], default_body_pos: np.ndarray) -> float:
    feet = [3, 6]
    core = [0, 7, 8, 11]
    src_positions = []
    for path in files:
        data = np.load(path)
        src_positions.append(data["body_pos_w"])
    src = np.concatenate(src_positions, axis=0)
    src_span = float(src[:, core, 2].mean() - src[:, feet, 2].mean())
    dst_span = float(default_body_pos[core, 2].mean() - default_body_pos[feet, 2].mean())
    if src_span <= 1e-6:
        raise ValueError(f"Invalid source body height span: {src_span}")
    return dst_span / src_span


def _lower_body_clamp(name: str) -> tuple[float, float] | None:
    if "_hip_pitch_joint" in name:
        return (-0.55, 0.25)
    if "_hip_roll_joint" in name:
        return (-0.25, 0.25)
    if "_hip_yaw_joint" in name:
        return (-0.35, 0.35)
    if "_knee_joint" in name:
        return (0.15, 0.75)
    if "_ankle_pitch_joint" in name:
        return (-0.45, 0.15)
    if "_ankle_roll_joint" in name:
        return (-0.20, 0.20)
    return None


def _stabilize_lower_body(
    joint_pos: np.ndarray,
    canonical_joint_names: list[str],
    default_joint_pos: np.ndarray,
    blend: float,
) -> tuple[np.ndarray, list[str]]:
    """Blend unreliable retargeted leg joints toward the A3 default pose and clamp to trainable ranges."""
    blend = float(np.clip(blend, 0.0, 1.0))
    out = joint_pos.copy()
    changed: list[str] = []
    for i, name in enumerate(canonical_joint_names):
        clamp = _lower_body_clamp(name)
        if clamp is None:
            continue
        lo, hi = clamp
        out[:, i] = default_joint_pos[i] + blend * (joint_pos[:, i] - default_joint_pos[i])
        out[:, i] = np.clip(out[:, i], lo, hi)
        changed.append(name)
    return out.astype(np.float32), changed


def _write_yaml_sidecar(src_yaml: Path, dst_yaml: Path, updates: dict[str, Any]) -> None:
    doc: dict[str, Any] = {}
    if src_yaml.is_file():
        with src_yaml.open("r", encoding="utf-8") as fh:
            doc = yaml.safe_load(fh) or {}
    doc["name"] = dst_yaml.stem
    doc["alignment"] = updates
    with dst_yaml.open("w", encoding="utf-8") as fh:
        yaml.safe_dump(doc, fh, sort_keys=False, allow_unicode=False)


def _update_manifest_rows(
    fieldnames: list[str],
    rows: list[dict[str, str]],
    outputs: list[Path],
    scale: float,
    method: str,
    lower_body_mode: str,
    lower_body_blend: float,
    root_yaw_mode: str,
) -> tuple[list[str], list[dict[str, str]]]:
    out_fields = list(fieldnames)
    for field in (
        "alignment_method",
        "alignment_source",
        "alignment_scale",
        "lower_body_mode",
        "lower_body_blend",
        "root_yaw_mode",
    ):
        if field not in out_fields:
            out_fields.append(field)

    out_rows = []
    for row, output in zip(rows, outputs):
        src = _clip_path(row)
        new_row = dict(row)
        new_row["source"] = str(src)
        new_row["output"] = str(output)
        new_row["alignment_method"] = method
        new_row["alignment_source"] = str(src)
        new_row["alignment_scale"] = f"{scale:.9f}"
        new_row["lower_body_mode"] = lower_body_mode
        new_row["lower_body_blend"] = f"{lower_body_blend:.6f}"
        new_row["root_yaw_mode"] = root_yaw_mode
        out_rows.append(new_row)
    return out_fields, out_rows


def _write_manifest(path: Path, fieldnames: list[str], rows: list[dict[str, str]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames, delimiter="\t", extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    args = _parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    output_manifest = args.output_manifest or (args.output_dir / "manifest.tsv")

    fieldnames, rows = _load_manifest(args.manifest)
    if not rows:
        raise RuntimeError(f"No rows in manifest: {args.manifest}")
    files = [_clip_path(row) for row in rows]
    if any(not path.is_file() for path in files):
        missing = [str(path) for path in files if not path.is_file()]
        raise FileNotFoundError(f"Missing motion files: {missing}")

    # Isaac imports must happen after argument parsing so --help stays lightweight.
    from isaaclab.app import AppLauncher

    app_launcher = AppLauncher(headless=True)
    simulation_app = app_launcher.app
    try:
        import gymnasium as gym
        import whole_body_tracking.tasks.tracking  # noqa: F401
        from isaaclab_tasks.utils import parse_env_cfg

        from whole_body_tracking.robots.agibot_a3 import A3_TRACKED_BODIES
        from whole_body_tracking.utils.action_adapter_config import load_joint_order, resolve_joint_order_mapping

        max_frames = max(int(np.load(path)["joint_pos"].shape[0]) for path in files)
        env_cfg = parse_env_cfg(args.task, device=args.device, num_envs=max_frames)
        if hasattr(env_cfg, "events"):
            for name in ("link_mass", "pd_gains"):
                if hasattr(env_cfg.events, name):
                    setattr(env_cfg.events, name, None)

        env = gym.make(args.task, cfg=env_cfg)
        env.reset(seed=0)
        unwrapped = env.unwrapped
        robot = unwrapped.scene["robot"]
        tracked_ids = torch.tensor(
            robot.find_bodies(A3_TRACKED_BODIES, preserve_order=True)[0], dtype=torch.long, device=robot.device
        )
        mapping = resolve_joint_order_mapping(robot.data.joint_names, canonical_joint_names=load_joint_order())
        canonical_joint_names = list(mapping.canonical)
        canonical_to_art = torch.tensor(mapping.canonical_to_articulation, dtype=torch.long, device=robot.device)
        default_joint_pos_canonical = (
            robot.data.default_joint_pos[0, canonical_to_art].detach().cpu().numpy().astype(np.float32)
        )

        env_origins = unwrapped.scene.env_origins
        origin0 = env_origins[0].detach().cpu().numpy().astype(np.float32)
        default_body = robot.data.body_pos_w[0, tracked_ids].detach().cpu().numpy() - origin0[None, :]
        default_root = robot.data.root_state_w[0].detach().clone()
        default_root_local = default_root.clone()
        default_root_local[:3] -= env_origins[0]
        default_root_xy = default_body[0, :2].astype(np.float32)
        default_root_quat = default_root_local[3:7].detach().cpu().numpy().astype(np.float32)
        default_root_yaw = float(_quat_yaw(default_root_quat))
        default_ground_z = float(default_body[[3, 6], 2].min())
        target_ground_z = default_ground_z if args.target_ground_z is None else float(args.target_ground_z)
        scale = _dataset_vertical_scale(files, default_body)

        output_paths: list[Path] = []
        method_parts = ["a3_fk_ground_scale_root"]
        if args.root_yaw_mode == "ready_to_default":
            method_parts.append("root_yaw_canonical")
        if args.lower_body_mode == "stabilize":
            method_parts.append("lower_body_stabilized")
        method = "_".join(method_parts) + "_v1"
        for src in files:
            data = np.load(src)
            joint_pos = _as_f32(data["joint_pos"])
            changed_lower_body_joints: list[str] = []
            if args.lower_body_mode == "stabilize":
                joint_pos, changed_lower_body_joints = _stabilize_lower_body(
                    joint_pos,
                    canonical_joint_names,
                    default_joint_pos_canonical,
                    args.lower_body_blend,
                )
            frames = int(joint_pos.shape[0])
            dt = 1.0 / float(data["fps"])
            env_ids = torch.arange(frames, dtype=torch.long, device=robot.device)

            source_body_pos = _as_f32(data["body_pos_w"])
            source_body_quat = _normalize_quat(data["body_quat_w"])
            ready_n = min(max(int(args.ready_frames), 1), frames)
            ready_root_xy = source_body_pos[:ready_n, 0, :2].mean(axis=0)
            ready_root_yaw = _circular_mean(_quat_yaw(source_body_quat[:ready_n, 0]))
            yaw_correction = 0.0
            if args.root_yaw_mode == "ready_to_default":
                yaw_correction = default_root_yaw - ready_root_yaw
            root_xy_delta = source_body_pos[:, 0, :2] - ready_root_xy[None, :]
            root_xy_delta = _rotate_xy(root_xy_delta, yaw_correction)
            root_xy = default_root_xy[None, :] + scale * root_xy_delta
            ready_root_z = float(source_body_pos[:ready_n, 0, 2].mean())
            root_z = float(default_root_local[2].detach().cpu()) + scale * (source_body_pos[:, 0, 2] - ready_root_z)
            root_quat = source_body_quat[:, 0]
            if args.root_yaw_mode == "ready_to_default":
                root_quat = _normalize_quat(_quat_mul(_yaw_quat(yaw_correction), root_quat))

            root_state = default_root.repeat(frames, 1)
            root_pos_local = torch.zeros((frames, 3), dtype=torch.float32, device=robot.device)
            root_pos_local[:, 0:2] = torch.tensor(root_xy, dtype=torch.float32, device=robot.device)
            root_pos_local[:, 2] = torch.tensor(root_z, dtype=torch.float32, device=robot.device)
            root_state[:, :3] = root_pos_local + env_origins[:frames]
            root_state[:, 3:7] = torch.tensor(root_quat, dtype=torch.float32, device=robot.device)
            root_state[:, 7:] = 0.0

            art_joint_pos = robot.data.default_joint_pos[:frames].clone()
            art_joint_vel = torch.zeros_like(art_joint_pos)
            art_joint_pos[:, canonical_to_art] = torch.tensor(joint_pos, dtype=torch.float32, device=robot.device)
            if "joint_vel" in data:
                art_joint_vel[:, canonical_to_art] = torch.tensor(
                    _as_f32(data["joint_vel"]), dtype=torch.float32, device=robot.device
                )

            robot.write_root_state_to_sim(root_state, env_ids=env_ids)
            robot.write_joint_state_to_sim(art_joint_pos, art_joint_vel, env_ids=env_ids)
            unwrapped.scene.write_data_to_sim()
            unwrapped.sim.forward()
            unwrapped.scene.update(dt=0.0)

            fk_body_pos = (
                robot.data.body_pos_w[:frames, tracked_ids] - env_origins[:frames, None, :]
            ).detach().cpu().numpy().astype(np.float32)
            fk_body_quat = robot.data.body_quat_w[:frames, tracked_ids].detach().cpu().numpy().astype(np.float32)
            ankle_min = fk_body_pos[:, [3, 6], 2].min(axis=1)
            z_shift = (target_ground_z - ankle_min).astype(np.float32)
            fk_body_pos[:, :, 2] += z_shift[:, None]

            joint_vel = _linear_velocity(joint_pos, dt)
            body_lin_vel = _linear_velocity(fk_body_pos, dt)
            body_ang_vel = _angular_velocity_from_quat(fk_body_quat, dt)

            dst = args.output_dir / f"{src.stem}_a3fk_aligned.npz"
            np.savez(
                dst,
                fps=np.float32(data["fps"]),
                joint_pos=joint_pos.astype(np.float32),
                joint_vel=joint_vel.astype(np.float32),
                body_pos_w=fk_body_pos.astype(np.float32),
                body_quat_w=_normalize_quat(fk_body_quat),
                body_lin_vel_w=body_lin_vel.astype(np.float32),
                body_ang_vel_w=body_ang_vel.astype(np.float32),
            )
            output_paths.append(dst.resolve())

            _write_yaml_sidecar(
                src.with_suffix(".yaml"),
                dst.with_suffix(".yaml"),
                {
                    "method": method,
                    "source_motion": str(src),
                    "dataset_vertical_scale": float(scale),
                    "target_ground_z": target_ground_z,
                    "default_root_xy": [float(x) for x in default_root_xy],
                    "default_root_yaw_rad": default_root_yaw,
                    "ready_root_yaw_rad": ready_root_yaw,
                    "root_yaw_correction_rad": yaw_correction,
                    "root_yaw_mode": args.root_yaw_mode,
                    "ready_frames": ready_n,
                    "lower_body_mode": args.lower_body_mode,
                    "lower_body_blend": float(args.lower_body_blend),
                    "lower_body_joints": changed_lower_body_joints,
                    "regenerated_fields": [
                        "body_pos_w",
                        "body_quat_w",
                        "body_lin_vel_w",
                        "body_ang_vel_w",
                        "joint_vel",
                    ],
                },
            )

        out_fields, out_rows = _update_manifest_rows(
            fieldnames,
            rows,
            output_paths,
            scale,
            method,
            args.lower_body_mode,
            args.lower_body_blend,
            args.root_yaw_mode,
        )
        _write_manifest(output_manifest, out_fields, out_rows)

        print(f"scale={scale:.9f}")
        print(f"target_ground_z={target_ground_z:.9f}")
        print(f"clips={len(output_paths)}")
        print(f"manifest={output_manifest.resolve()}")
        env.close()
    finally:
        simulation_app.close()


if __name__ == "__main__":
    main()
