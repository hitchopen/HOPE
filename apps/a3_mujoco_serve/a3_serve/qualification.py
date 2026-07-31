"""Offline qualification for A3 high-level arm motion packages.

The motion manifest binds files together.  This module independently applies
the non-relaxable runtime safety profile and looks up hardware approval in the
source-controlled approval registry.  A safe but unapproved motion remains a
candidate: it can be packaged and inspected, but the wrapper will not publish
it on a robot.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable


PROFILE_NAME = "a3_high_level_arm_v1"
SOURCE_HZ = 200.0
COMMAND_HZ = 100.0
FRAME_COUNT = 3878
READY_FRAME = 1600
STROKE_START_FRAME = 1848
STRIKE_FRAME = 1860
SOURCE_STRIDE = 2
LAST_STROKE_FRAME = 3876
MAX_PREPARE_STEP_RAD = 0.03
MAX_PREPARE_SPEED_RAD_S = 3.0
MAX_SOURCE_STROKE_STEP_RAD = 0.026
MAX_SOURCE_STROKE_SPEED_RAD_S = 5.2
MAX_COMMAND_VELOCITY_LIMIT_RATIO = 0.5

ROOT_COLUMNS = (
    "root_translateX",
    "root_translateY",
    "root_translateZ",
    "root_rotateX",
    "root_rotateY",
    "root_rotateZ",
)
JOINT_NAMES = (
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "head_yaw_joint",
    "head_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
)
CSV_COLUMNS = ROOT_COLUMNS + JOINT_NAMES
ARM_JOINTS = JOINT_NAMES[5:19]
ARM_CSV_INDICES = tuple(CSV_COLUMNS.index(name) for name in ARM_JOINTS)
ACTIVE_ARM_INDICES = tuple(index for index in range(14) if index != 4)
POSITION_LO = (
    -2.967,
    -1.588,
    -2.793,
    -1.047,
    -0.576,
    -1.623,
    -2.793,
) * 2
POSITION_HI = (
    2.967,
    1.588,
    2.793,
    2.444,
    0.576,
    1.623,
    2.793,
) * 2
VELOCITY_LIMITS = (
    13.613568165555769,
    13.613568165555769,
    15.707963267948966,
    15.707963267948966,
    15.707963267948966,
    12.775810124598491,
    12.775810124598491,
) * 2


class QualificationError(ValueError):
    """Raised when a motion violates the fixed packaging contract."""


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_json(path: Path, description: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise QualificationError(f"cannot read {description} {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise QualificationError(f"{description} root must be an object")
    return value


def _manifest_timeline(manifest: dict[str, Any]) -> tuple[int, int, int, int, int]:
    timeline = manifest.get("timeline")
    if not isinstance(timeline, dict):
        raise QualificationError("manifest.timeline must be an object")
    if "ready_source_frame" in timeline:
        ready = timeline.get("ready_source_frame")
    else:
        left_ready = manifest.get("left_ready")
        ready = left_ready.get("source_frame") if isinstance(left_ready, dict) else None
    start = timeline.get("stroke_source_start_frame")
    strike = timeline.get("nominal_strike_source_frame")
    stride = timeline.get("source_stride")
    if "stroke_source_end_frame" in timeline:
        last = timeline.get("stroke_source_end_frame")
    elif "command_stroke_frames" in timeline and isinstance(stride, int):
        command_frames = timeline.get("command_stroke_frames")
        last = (
            start + (command_frames - 1) * stride
            if isinstance(start, int) and isinstance(command_frames, int)
            else None
        )
    else:
        last = LAST_STROKE_FRAME
    values = (ready, start, strike, stride, last)
    if not all(isinstance(value, int) for value in values):
        raise QualificationError("manifest timeline fields must be integers")
    return values  # type: ignore[return-value]


def _load_arm_rows(path: Path) -> list[tuple[float, ...]]:
    try:
        with path.open(newline="", encoding="utf-8") as stream:
            reader = csv.reader(stream)
            header = tuple(next(reader))
            if header != CSV_COLUMNS:
                raise QualificationError(
                    f"CSV header must contain the exact {len(CSV_COLUMNS)} columns"
                )
            rows: list[tuple[float, ...]] = []
            for frame, raw in enumerate(reader):
                if len(raw) != len(CSV_COLUMNS):
                    raise QualificationError(
                        f"CSV row {frame} has {len(raw)} columns, expected {len(CSV_COLUMNS)}"
                    )
                try:
                    values = tuple(float(value) for value in raw)
                except ValueError as exc:
                    raise QualificationError(
                        f"CSV row {frame} contains a non-numeric value"
                    ) from exc
                if not all(math.isfinite(value) for value in values):
                    raise QualificationError(f"CSV row {frame} contains a non-finite value")
                rows.append(
                    tuple(math.radians(values[index]) for index in ARM_CSV_INDICES)
                )
    except StopIteration as exc:
        raise QualificationError("motion CSV is empty") from exc
    except OSError as exc:
        raise QualificationError(f"cannot read motion CSV {path}: {exc}") from exc
    if len(rows) != FRAME_COUNT:
        raise QualificationError(
            f"CSV frame count must be {FRAME_COUNT}, got {len(rows)}"
        )
    return rows


def _maximum_step(
    rows: list[tuple[float, ...]], frames: Iterable[int]
) -> tuple[float, int, int]:
    selected = iter(frames)
    try:
        previous_frame = next(selected)
    except StopIteration:
        return 0.0, 0, 0
    maximum = 0.0
    maximum_frame = previous_frame
    maximum_joint = 0
    for frame in selected:
        for joint in ACTIVE_ARM_INDICES:
            step = abs(rows[frame][joint] - rows[previous_frame][joint])
            if step > maximum:
                maximum = step
                maximum_frame = frame
                maximum_joint = joint
        previous_frame = frame
    return maximum, maximum_frame, maximum_joint


def _approval_for(
    registry: dict[str, Any], motion_sha256: str
) -> dict[str, Any] | None:
    if registry.get("schema_version") != 1:
        raise QualificationError("approval registry schema_version must be 1")
    if registry.get("safety_profile") != PROFILE_NAME:
        raise QualificationError("approval registry safety profile mismatch")
    motions = registry.get("motions")
    if not isinstance(motions, list):
        raise QualificationError("approval registry motions must be a list")
    matches = [
        entry
        for entry in motions
        if isinstance(entry, dict) and entry.get("sha256") == motion_sha256
    ]
    if len(matches) > 1:
        raise QualificationError("approval registry contains a duplicate motion hash")
    if not matches or matches[0].get("status") != "approved":
        return None
    if not isinstance(matches[0].get("approval_id"), str):
        raise QualificationError("approved registry entry lacks approval_id")
    return matches[0]


def qualify(
    motion_path: str | Path,
    manifest_path: str | Path,
    registry_path: str | Path,
) -> dict[str, Any]:
    """Validate one motion and return its candidate/approved qualification."""

    motion = Path(motion_path).expanduser().resolve()
    manifest_file = Path(manifest_path).expanduser().resolve()
    registry_file = Path(registry_path).expanduser().resolve()
    manifest = _load_json(manifest_file, "motion manifest")
    registry = _load_json(registry_file, "approval registry")
    motion_sha256 = sha256_file(motion)

    source = manifest.get("source")
    if not isinstance(source, dict):
        raise QualificationError("manifest.source must be an object")
    if source.get("sha256") != motion_sha256:
        raise QualificationError("motion SHA-256 does not match manifest.source.sha256")
    if float(source.get("fps", 0.0)) != SOURCE_HZ:
        raise QualificationError(f"manifest source fps must be {SOURCE_HZ}")
    if source.get("frame_count") != FRAME_COUNT:
        raise QualificationError(f"manifest frame_count must be {FRAME_COUNT}")

    timeline = _manifest_timeline(manifest)
    expected_timeline = (
        READY_FRAME,
        STROKE_START_FRAME,
        STRIKE_FRAME,
        SOURCE_STRIDE,
        LAST_STROKE_FRAME,
    )
    if timeline != expected_timeline:
        raise QualificationError(
            f"manifest timeline {timeline!r} does not match runtime profile "
            f"{expected_timeline!r}"
        )
    manifest_joint_order = manifest.get("joint_order")
    if manifest_joint_order is not None and tuple(manifest_joint_order) != ARM_JOINTS:
        raise QualificationError("manifest joint_order does not match the A3 arm order")
    declared_profile = manifest.get("safety_profile")
    if isinstance(declared_profile, dict) and declared_profile.get("name") != PROFILE_NAME:
        raise QualificationError("manifest safety profile mismatch")

    offline = manifest.get("offline_validation")
    if isinstance(offline, dict):
        declared_limits = {
            "max_prepare_step_rad": MAX_PREPARE_STEP_RAD,
            "max_prepare_speed_rad_s": MAX_PREPARE_SPEED_RAD_S,
            "max_source_stroke_step_rad": MAX_SOURCE_STROKE_STEP_RAD,
            "max_source_stroke_speed_rad_s": MAX_SOURCE_STROKE_SPEED_RAD_S,
            "max_scaled_stroke_velocity_limit_ratio": MAX_COMMAND_VELOCITY_LIMIT_RATIO,
        }
        for key, fixed_limit in declared_limits.items():
            value = float(offline.get(key, fixed_limit))
            if value > fixed_limit:
                raise QualificationError(
                    f"manifest {key}={value} relaxes fixed profile limit {fixed_limit}"
                )

    rows = _load_arm_rows(motion)
    for frame, row in enumerate(rows):
        for joint in ACTIVE_ARM_INDICES:
            if row[joint] < POSITION_LO[joint] or row[joint] > POSITION_HI[joint]:
                raise QualificationError(
                    f"active joint {ARM_JOINTS[joint]} exceeds the high-level "
                    f"position limit at frame {frame}"
                )

    prepare_step, prepare_frame, prepare_joint = _maximum_step(
        rows, range(0, READY_FRAME + 1, SOURCE_STRIDE)
    )
    prepare_speed = prepare_step * COMMAND_HZ
    if prepare_step > MAX_PREPARE_STEP_RAD + 1.0e-12:
        raise QualificationError(
            f"prepare step {prepare_step:.12g} exceeds {MAX_PREPARE_STEP_RAD} at "
            f"frame {prepare_frame}, joint {ARM_JOINTS[prepare_joint]}"
        )
    if prepare_speed > MAX_PREPARE_SPEED_RAD_S + 1.0e-12:
        raise QualificationError(
            f"prepare speed {prepare_speed:.12g} exceeds {MAX_PREPARE_SPEED_RAD_S}"
        )

    source_step, source_frame, source_joint = _maximum_step(
        rows, range(STROKE_START_FRAME, LAST_STROKE_FRAME + 1)
    )
    source_speed = source_step * SOURCE_HZ
    if source_step > MAX_SOURCE_STROKE_STEP_RAD + 1.0e-12:
        raise QualificationError(
            f"source stroke step {source_step:.12g} exceeds "
            f"{MAX_SOURCE_STROKE_STEP_RAD} at frame {source_frame}, "
            f"joint {ARM_JOINTS[source_joint]}"
        )
    if source_speed > MAX_SOURCE_STROKE_SPEED_RAD_S + 1.0e-12:
        raise QualificationError(
            f"source stroke speed {source_speed:.12g} exceeds "
            f"{MAX_SOURCE_STROKE_SPEED_RAD_S}"
        )

    command_step, command_frame, command_joint = _maximum_step(
        rows,
        range(STROKE_START_FRAME, LAST_STROKE_FRAME + 1, SOURCE_STRIDE),
    )
    command_velocity_ratio = max(
        abs(rows[frame][joint] - rows[previous][joint])
        * COMMAND_HZ
        / VELOCITY_LIMITS[joint]
        for previous, frame in zip(
            range(
                STROKE_START_FRAME,
                LAST_STROKE_FRAME - SOURCE_STRIDE + 1,
                SOURCE_STRIDE,
            ),
            range(
                STROKE_START_FRAME + SOURCE_STRIDE,
                LAST_STROKE_FRAME + 1,
                SOURCE_STRIDE,
            ),
        )
        for joint in ACTIVE_ARM_INDICES
    )
    if command_velocity_ratio > MAX_COMMAND_VELOCITY_LIMIT_RATIO + 1.0e-12:
        raise QualificationError(
            f"command velocity ratio {command_velocity_ratio:.12g} exceeds "
            f"{MAX_COMMAND_VELOCITY_LIMIT_RATIO}"
        )

    approval = _approval_for(registry, motion_sha256)
    status = "approved" if approval is not None else "candidate"
    return {
        "schema_version": 1,
        "status": status,
        "approval_id": approval.get("approval_id") if approval else None,
        "approval_basis": approval.get("basis") if approval else None,
        "safety_profile": PROFILE_NAME,
        "safety_checks": "pass",
        "motion": {
            "sha256": motion_sha256,
            "frame_count": len(rows),
            "source_hz": SOURCE_HZ,
        },
        "manifest_sha256": sha256_file(manifest_file),
        "approval_registry_sha256": sha256_file(registry_file),
        "metrics": {
            "max_prepare_step_rad": prepare_step,
            "max_prepare_speed_rad_s": prepare_speed,
            "max_source_stroke_step_rad": source_step,
            "max_source_stroke_speed_rad_s": source_speed,
            "max_command_stroke_step_rad": command_step,
            "max_command_stroke_step_frame": command_frame,
            "max_command_stroke_step_joint": ARM_JOINTS[command_joint],
            "max_command_velocity_limit_ratio": command_velocity_ratio,
        },
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--motion", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--registry", required=True)
    parser.add_argument("--output")
    parser.add_argument("--require-approved", action="store_true")
    args = parser.parse_args(argv)
    try:
        result = qualify(args.motion, args.manifest, args.registry)
    except QualificationError as exc:
        parser.exit(1, f"qualification failed: {exc}\n")
    if args.output:
        output = Path(args.output).expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, sort_keys=True))
    if args.require_approved and result["status"] != "approved":
        parser.exit(2, "qualification passed, but motion is not hardware-approved\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
