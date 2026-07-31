"""A3 SDK CSV parsing, deterministic export, and provenance manifests."""

from __future__ import annotations

import csv
import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from .constants import (
    A3_SAFETY_PROFILE,
    ARM_JOINTS,
    CSV_COLUMNS,
    JOINT_NAMES,
    RIGHT_ARM_JOINTS,
    VALIDATED_FRAME_COUNT,
)


class CsvContractError(ValueError):
    """Raised when a CSV is not compatible with the validated A3 runtime."""


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


@dataclass(frozen=True)
class MotionCsv:
    path: Path
    values: np.ndarray

    @classmethod
    def load(cls, path: str | Path) -> "MotionCsv":
        path = Path(path).expanduser().resolve()
        try:
            with path.open(newline="", encoding="utf-8") as stream:
                reader = csv.reader(stream)
                header = tuple(next(reader))
                if header != CSV_COLUMNS:
                    raise CsvContractError(
                        f"CSV header mismatch: expected {len(CSV_COLUMNS)} exact columns"
                    )
                rows = [[float(value) for value in row] for row in reader]
        except StopIteration as exc:
            raise CsvContractError("CSV is empty") from exc
        except (OSError, ValueError) as exc:
            raise CsvContractError(f"cannot parse {path}: {exc}") from exc
        values = np.asarray(rows, dtype=np.float64)
        if values.shape != (VALIDATED_FRAME_COUNT, len(CSV_COLUMNS)):
            raise CsvContractError(
                f"CSV shape must be {(VALIDATED_FRAME_COUNT, len(CSV_COLUMNS))}, "
                f"got {values.shape}"
            )
        if not np.all(np.isfinite(values)):
            raise CsvContractError("CSV contains non-finite values")
        return cls(path=path, values=values)

    def joint_radians(self) -> np.ndarray:
        """Return all 31 joint columns in radians."""

        return np.deg2rad(self.values[:, len(CSV_COLUMNS) - len(JOINT_NAMES):])

    def arm_radians(self) -> np.ndarray:
        indices = [CSV_COLUMNS.index(name) for name in ARM_JOINTS]
        return np.deg2rad(self.values[:, indices])

    def with_right_arm(self, radians: np.ndarray) -> np.ndarray:
        radians = np.asarray(radians, dtype=np.float64)
        expected = (VALIDATED_FRAME_COUNT, len(RIGHT_ARM_JOINTS))
        if radians.shape != expected:
            raise CsvContractError(f"right-arm trajectory must have shape {expected}")
        if not np.all(np.isfinite(radians)):
            raise CsvContractError("right-arm trajectory contains non-finite values")
        output = self.values.copy()
        for joint, column in enumerate(RIGHT_ARM_JOINTS):
            output[:, CSV_COLUMNS.index(column)] = np.rad2deg(radians[:, joint])
        return output


def write_motion_csv(path: str | Path, values: np.ndarray) -> Path:
    path = Path(path).expanduser().resolve()
    values = np.asarray(values, dtype=np.float64)
    if values.shape != (VALIDATED_FRAME_COUNT, len(CSV_COLUMNS)):
        raise CsvContractError("cannot export a CSV with an incompatible shape")
    if not np.all(np.isfinite(values)):
        raise CsvContractError("cannot export non-finite values")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(CSV_COLUMNS)
        for row in values:
            writer.writerow([format(float(value), ".12g") for value in row])
    return path


def write_generation_manifest(
    path: str | Path,
    *,
    motion_path: str | Path,
    template_path: str | Path,
    model_path: str | Path,
    physics_reference_path: str | Path,
    config_path: str | Path,
    timing: dict[str, Any],
    result: dict[str, Any],
) -> Path:
    """Write the artifact identity consumed by the generalized MDU build."""

    path = Path(path).expanduser().resolve()
    motion_path = Path(motion_path).resolve()
    payload = {
        "schema_version": 4,
        "artifact_name": "a3_mujoco_generated_serve",
        "source": {
            "path": motion_path.name,
            "sha256": sha256_file(motion_path),
            "fps": 200.0,
            "frame_count": VALIDATED_FRAME_COUNT,
            "units": "degrees_in_csv_radians_internal",
        },
        "generation": {
            "method": "mujoco_ballistic_search_then_damped_least_squares_ik",
            "template_csv": str(Path(template_path).resolve()),
            "template_sha256": sha256_file(template_path),
            "model_xml": str(Path(model_path).resolve()),
            "model_xml_sha256": sha256_file(model_path),
            "physics_reference": str(Path(physics_reference_path).resolve()),
            "physics_reference_sha256": sha256_file(physics_reference_path),
            "config": str(Path(config_path).resolve()),
            "config_sha256": sha256_file(config_path),
            "racket_pose_is_not_hand_tuned": True,
        },
        "transport": {
            "kind": "ros2_vendor_motion_control",
            "state_topic": "/motion/control/arm_joint_state",
            "command_topic": "/motion/control/arm_joint_command",
            "message_type": "sensor_msgs/msg/JointState",
            "publish_hz": 100.0,
            "command_joint_count": 14,
            "vendor_keeps_ownership_of": ["waist", "legs", "neck"],
        },
        "timeline": {
            "ready_source_frame": int(timing["ready_frame"]),
            "stroke_source_start_frame": int(timing["stroke_start_frame"]),
            "nominal_strike_source_frame": int(timing["strike_frame"]),
            "follow_end_source_frame": int(timing["follow_end_frame"]),
            "return_end_source_frame": int(timing["return_end_frame"]),
            "source_stride": 2,
        },
        "safety_profile": {
            "name": A3_SAFETY_PROFILE,
            "limits_are_non_relaxable": True,
        },
        "planning_result": result,
        "evidence": {
            "validated_reference_runtime": (
                "exact PR #18 application fully tested, executable, and safe on A3"
            ),
            "generated_motion_requires_its_own_hardware_qualification": True,
        },
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return path
