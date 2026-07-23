"""Convert old HOPE A3 motion npz files to the current motion schema.

Old local HOPE clips were recorded directly from Isaac articulation tensors:

* joint_pos / joint_vel are in Isaac articulation order;
* body_* arrays contain all 32 articulation bodies.

The current training task consumes canonical 31-DOF joint order and exactly the 14 tracked bodies
listed in docs/REPLACE_MOTIONS.md. This utility performs that deterministic reorder/crop and writes
matching YAML sidecars for provenance and manual phase review.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Any

import numpy as np
import yaml


CANONICAL_TO_ARTICULATION = (
    2,
    5,
    8,
    11,
    16,
    12,
    17,
    21,
    23,
    25,
    27,
    29,
    13,
    18,
    22,
    24,
    26,
    28,
    30,
    0,
    3,
    6,
    9,
    14,
    19,
    1,
    4,
    7,
    10,
    15,
    20,
)

# Latest A3 articulation body indices for A3_TRACKED_BODIES, resolved once in Isaac with
# robot.find_bodies(A3_TRACKED_BODIES, preserve_order=True).
OLD_ARTICULATION_TRACKED_BODY_IDS = (0, 4, 10, 20, 5, 11, 21, 9, 18, 24, 30, 19, 25, 31)

TRACKED_BODIES = (
    "pelvis_link",
    "left_hip_roll_Link",
    "left_knee_Link",
    "left_ankle_roll_Link",
    "right_hip_roll_Link",
    "right_knee_Link",
    "right_ankle_roll_Link",
    "torso_Link",
    "left_shoulder_roll_Link",
    "left_elbow_Link",
    "left_wrist_yaw_Link",
    "right_shoulder_roll_Link",
    "right_elbow_Link",
    "right_wrist_yaw_Link",
)

REQUIRED_KEYS = (
    "fps",
    "joint_pos",
    "joint_vel",
    "body_pos_w",
    "body_quat_w",
    "body_lin_vel_w",
    "body_ang_vel_w",
)


def repo_root() -> Path:
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "hope_training").is_dir():
            return parent
    raise RuntimeError(f"could not find repo root from {here}")


def load_joint_order(root: Path) -> list[str]:
    path = root / "hope_training" / "config" / "joint_order_agibot_a3.yaml"
    with path.open("r", encoding="utf-8") as fh:
        doc = yaml.safe_load(fh) or {}
    joints = list(doc.get("joint_order") or [])
    if len(joints) != 31:
        raise ValueError(f"{path} should contain 31 joints, got {len(joints)}")
    return joints


def infer_swing_side(path: Path) -> int:
    name = path.stem.lower()
    if "backhand" in name:
        return -1
    return 1


def scalar_fps(value: np.ndarray) -> float:
    flat = np.asarray(value).reshape(-1)
    if flat.size < 1:
        raise ValueError("fps is empty")
    return float(flat[0])


def as_float32(array: np.ndarray) -> np.ndarray:
    return np.asarray(array, dtype=np.float32)


def validate_input(path: Path, data: Any) -> None:
    missing = [key for key in REQUIRED_KEYS if key not in data]
    if missing:
        raise ValueError(f"{path}: missing keys: {missing}")
    joint_pos = data["joint_pos"]
    joint_vel = data["joint_vel"]
    if joint_pos.ndim != 2 or joint_pos.shape[1] != 31:
        raise ValueError(f"{path}: joint_pos must be [F,31], got {joint_pos.shape}")
    if joint_vel.shape != joint_pos.shape:
        raise ValueError(f"{path}: joint_vel shape {joint_vel.shape} != joint_pos {joint_pos.shape}")
    for key, width in (
        ("body_pos_w", 3),
        ("body_quat_w", 4),
        ("body_lin_vel_w", 3),
        ("body_ang_vel_w", 3),
    ):
        arr = data[key]
        if arr.ndim != 3 or arr.shape[0] != joint_pos.shape[0] or arr.shape[1] < 32 or arr.shape[2] != width:
            raise ValueError(f"{path}: {key} must be [F,>=32,{width}], got {arr.shape}")
    for key in REQUIRED_KEYS:
        arr = np.asarray(data[key])
        if not np.all(np.isfinite(arr)):
            raise ValueError(f"{path}: {key} contains non-finite values")


def convert_file(src: Path, dst: Path, root: Path, joint_order: list[str]) -> dict[str, Any]:
    with np.load(src) as data:
        validate_input(src, data)
        fps = scalar_fps(data["fps"])
        frame_count = int(data["joint_pos"].shape[0])

        joint_pos = as_float32(data["joint_pos"][:, CANONICAL_TO_ARTICULATION])
        joint_vel = as_float32(data["joint_vel"][:, CANONICAL_TO_ARTICULATION])
        body_pos_w = as_float32(data["body_pos_w"][:, OLD_ARTICULATION_TRACKED_BODY_IDS, :])
        body_quat_w = as_float32(data["body_quat_w"][:, OLD_ARTICULATION_TRACKED_BODY_IDS, :])
        body_lin_vel_w = as_float32(data["body_lin_vel_w"][:, OLD_ARTICULATION_TRACKED_BODY_IDS, :])
        body_ang_vel_w = as_float32(data["body_ang_vel_w"][:, OLD_ARTICULATION_TRACKED_BODY_IDS, :])

    dst.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        dst,
        fps=np.array(fps, dtype=np.float32),
        joint_pos=joint_pos,
        joint_vel=joint_vel,
        body_pos_w=body_pos_w,
        body_quat_w=body_quat_w,
        body_lin_vel_w=body_lin_vel_w,
        body_ang_vel_w=body_ang_vel_w,
    )

    strike_frame = int(round(0.5 * (frame_count - 1))) if frame_count else 0
    strike_phase = float(strike_frame / (frame_count - 1)) if frame_count > 1 else 0.0
    ready_end = max(0, min(frame_count - 1, int(round(0.15 * (frame_count - 1)))))
    follow_end = max(strike_frame, min(frame_count - 1, int(round(0.75 * (frame_count - 1)))))
    swing_side = infer_swing_side(src)
    duration = frame_count / fps if fps > 0.0 else 0.0

    sidecar = {
        "name": dst.stem,
        "swing_side": swing_side,
        "fps": fps,
        "frame_count": frame_count,
        "frame_time_s": 1.0 / fps if fps > 0.0 else None,
        "duration_s": duration,
        "strike_frame": strike_frame,
        "strike_phase": strike_phase,
        "ready_interval_frames": [0, ready_end],
        "follow_through_end_frame": follow_end,
        "recover_end_frame": max(0, frame_count - 1),
        "joint_order": joint_order,
        "tracked_bodies": list(TRACKED_BODIES),
        "anchor_body": "torso_Link",
        "root_body": "pelvis_link",
        "racket_link": "right_wrist_yaw_Link",
        "racket_body": "pingpang_red_Link",
        "mount_offset_xyz": [0.21, 0.032, 0.032],
        "blade_normal_axis": "y",
        "blade_normal_sign": "+1 (red/forehand face)" if swing_side > 0 else "-1 (black/backhand face)",
        "source": {
            "schema": "old_hope_articulation_31j_full_32b",
            "file": str(src),
            "joint_reorder": "old articulation -> canonical using CANONICAL_TO_ARTICULATION",
            "body_crop": "old 32 articulation bodies -> current 14 tracked bodies",
        },
        "manual_review": {
            "phase_annotation_needed": True,
            "note": "Generated strike/phase fields are midpoint heuristics; review before long training.",
        },
    }
    with dst.with_suffix(".yaml").open("w", encoding="utf-8") as fh:
        yaml.safe_dump(sidecar, fh, sort_keys=False, allow_unicode=False)

    return {
        "source": str(src),
        "output": str(dst),
        "frames": frame_count,
        "fps": fps,
        "duration_s": duration,
        "swing_side": swing_side,
        "strike_phase": strike_phase,
    }


def parse_args() -> argparse.Namespace:
    root = repo_root()
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--input-dir", required=True, type=Path, help="Old HOPE sample_motions directory.")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=root / "hope_training" / "motions" / "converted_old_hope",
        help="Directory for converted npz/yaml files.",
    )
    parser.add_argument(
        "--manifest",
        type=Path,
        default=None,
        help="TSV manifest path (default: <output-dir>/manifest.tsv).",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    root = repo_root()
    input_dir = args.input_dir.resolve()
    output_dir = args.output_dir.resolve()
    manifest = (args.manifest or output_dir / "manifest.tsv").resolve()
    joint_order = load_joint_order(root)

    files = sorted(input_dir.rglob("*.npz"))
    if not files:
        raise FileNotFoundError(f"no .npz files under {input_dir}")

    rows: list[dict[str, Any]] = []
    for src in files:
        rel = src.relative_to(input_dir)
        dst = output_dir / rel
        rows.append(convert_file(src, dst, root, joint_order))

    manifest.parent.mkdir(parents=True, exist_ok=True)
    with manifest.open("w", encoding="utf-8", newline="") as fh:
        fieldnames = ("source", "output", "frames", "fps", "duration_s", "swing_side", "strike_phase")
        writer = csv.DictWriter(fh, fieldnames=fieldnames, delimiter="\t")
        writer.writeheader()
        writer.writerows(rows)

    total_frames = sum(int(row["frames"]) for row in rows)
    durations = [float(row["duration_s"]) for row in rows]
    print(f"converted_files={len(rows)}")
    print(f"output_dir={output_dir}")
    print(f"manifest={manifest}")
    print(f"total_frames={total_frames}")
    print(f"duration_s_min={min(durations):.3f} median={float(np.median(durations)):.3f} max={max(durations):.3f}")
    print("phase_annotation_needed=true")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
