#!/usr/bin/env python3
"""Implementation for calibrating mocap ``P1`` to A3 ``pelvis_link``.

This is a setup-session tool. It compares the P1 pose reported on
``/P1/pose`` with an independent, real-time ``PoseStamped`` measurement of
``world -> pelvis_link`` at matching timestamps. The robot may move during
collection; from each dynamic pose pair it estimates the same constant
transform::

    ^P1 T_pelvis_link

The normal runtime use is:

    world --dynamic mocap--> P1 --static calibration--> pelvis_link

As an alternative, the same correction can be absorbed into Motive's P1 pivot
definition so that P1 itself reports the A3 pelvis frame. Never apply both
methods.

NatNet is read-only with respect to Motive asset definitions; this tool
therefore never attempts to mutate Motive. It writes an auditable result and
prints the settings for an operator to apply in Motive's Builder/Modify pane.
It does not consume a Table topic or TF.

The pelvis input must be a genuinely independent full 6-DOF measurement. The
A3 hardware interface currently present in this repository publishes an IMU,
not this absolute pose; an external tracker or robot state estimator must
publish the required topic before real-hardware calibration. Never derive that
topic from P1 or from this tool's saved P1-to-pelvis transform.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]  # ROS order: x, y, z, w

# Source: agibot/pku/README.md, marker-coordinate table v2. These are ball
# centres in pelvis_link, in metres. The 0702 Parasolid asset and its layout
# define all ten points, and physical mocap testing confirmed that f1-f5 and
# b1-b5 are all visible. The complete ten-marker set is therefore the default.
MARKER_NAMES = ("f1", "f2", "f3", "f4", "f5", "b1", "b2", "b3", "b4", "b5")
MODEL_NOMINAL: dict[str, Vector3] = {
    "f1": (0.090, 0.000, -0.130),
    "f2": (0.080, 0.050, -0.140),
    "f3": (0.080, -0.050, -0.140),
    "f4": (0.078, -0.030, -0.180),
    "f5": (0.078, 0.030, -0.180),
    "b1": (-0.090, 0.000, -0.100),
    "b2": (-0.085, 0.055, -0.130),
    "b3": (-0.085, -0.055, -0.130),
    "b4": (-0.085, -0.030, -0.180),
    "b5": (-0.085, 0.030, -0.180),
}
CURRENT_SHELL_MARKER_NAMES = MARKER_NAMES


@dataclass(frozen=True)
class Transform:
    """Rigid transform ``^parent T_child`` with metres and xyzw quaternion."""

    translation: Vector3
    quaternion: Quaternion


def _dot(a: Sequence[float], b: Sequence[float]) -> float:
    return sum(x * y for x, y in zip(a, b))


def _norm(vector: Sequence[float]) -> float:
    return math.sqrt(_dot(vector, vector))


def _normalize_quaternion(quaternion: Sequence[float]) -> Quaternion:
    magnitude = _norm(quaternion)
    if magnitude < 1e-12 or not math.isfinite(magnitude):
        raise ValueError("zero or non-finite quaternion")
    return tuple(float(value / magnitude) for value in quaternion)  # type: ignore[return-value]


def _quat_conjugate(quaternion: Quaternion) -> Quaternion:
    return (-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3])


def _quat_multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return _normalize_quaternion((
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ))


def _rotate(quaternion: Quaternion, vector: Vector3) -> Vector3:
    """Rotate a vector with a unit xyzw quaternion without creating a matrix."""
    x, y, z, w = quaternion
    vx, vy, vz = vector
    # q * (v, 0) * q^-1, written in the numerically compact cross-product form.
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def compose(left: Transform, right: Transform) -> Transform:
    """Return ``left * right``; e.g. ``^A T_B * ^B T_C = ^A T_C``."""
    rotated = _rotate(left.quaternion, right.translation)
    return Transform(
        tuple(left.translation[index] + rotated[index] for index in range(3)),  # type: ignore[arg-type]
        _quat_multiply(left.quaternion, right.quaternion),
    )


def inverse(transform: Transform) -> Transform:
    """Return the inverse of a rigid transform."""
    inverse_quaternion = _quat_conjugate(transform.quaternion)
    inverse_translation = _rotate(
        inverse_quaternion,
        tuple(-value for value in transform.translation),  # type: ignore[arg-type]
    )
    return Transform(inverse_translation, inverse_quaternion)


def quaternion_average(quaternions: Iterable[Quaternion]) -> Quaternion:
    """Average nearby unit quaternions while handling q / -q equivalence."""
    values = [_normalize_quaternion(quaternion) for quaternion in quaternions]
    if not values:
        raise ValueError("need at least one quaternion")
    estimate = values[0]
    for _ in range(20):
        accumulator = [0.0, 0.0, 0.0, 0.0]
        for value in values:
            sign = 1.0 if _dot(estimate, value) >= 0.0 else -1.0
            for index in range(4):
                accumulator[index] += sign * value[index]
        updated = _normalize_quaternion(accumulator)
        if abs(_dot(estimate, updated)) > 1.0 - 1e-14:
            return updated
        estimate = updated
    return estimate


def _median_scalar(values: Sequence[float]) -> float:
    if not values:
        raise ValueError("need at least one value")
    ordered = sorted(values)
    midpoint = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[midpoint]
    return (ordered[midpoint - 1] + ordered[midpoint]) / 2.0


def quaternion_medoid(quaternions: Iterable[Quaternion]) -> Quaternion:
    """Return a robust observed orientation for the initial outlier gate.

    A chordal mean can be pulled between a valid cluster and a burst of stale
    samples. The medoid remains on an observed quaternion and minimizes first
    the median, then the total, geodesic distance to all samples.
    """
    values = [_normalize_quaternion(quaternion) for quaternion in quaternions]
    if not values:
        raise ValueError("need at least one quaternion")
    scored = []
    for index, candidate in enumerate(values):
        distances = [rotation_angle_rad(candidate, other) for other in values]
        scored.append((_median_scalar(distances), sum(distances), index))
    return values[min(scored)[2]]


def rotation_angle_rad(left: Quaternion, right: Quaternion) -> float:
    """Smallest angle between two orientations."""
    cosine_half_angle = min(1.0, abs(_dot(left, right)))
    return 2.0 * math.acos(cosine_half_angle)


def _mean_vector(vectors: Sequence[Vector3]) -> Vector3:
    count = len(vectors)
    if count == 0:
        raise ValueError("need at least one vector")
    return tuple(sum(vector[index] for vector in vectors) / count for index in range(3))  # type: ignore[return-value]


def parse_marker_names(value: str) -> tuple[str, ...]:
    """Parse a CAD marker-position set; its stream/order is intentionally irrelevant."""
    names = tuple(name.strip().lower() for name in value.split(",") if name.strip())
    if len(names) < 3:
        raise ValueError("at least three marker names are required")
    if len(set(names)) != len(names):
        raise ValueError("marker names must be unique")
    unknown = sorted(set(names) - set(MARKER_NAMES))
    if unknown:
        raise ValueError(f"unknown marker names: {', '.join(unknown)}")
    return names


def marker_centroid(marker_names: Sequence[str]) -> Vector3:
    """Return the selected marker-centre mean, expressed in ``pelvis_link``.

    A centroid is a set operation: marker order has no effect. This is only a
    CAD cross-check; the live calibration below uses the solved P1 pose, never
    individual marker topics or their labels.
    """
    if not marker_names:
        raise ValueError("need at least one marker name")
    try:
        return _mean_vector([MODEL_NOMINAL[name] for name in marker_names])
    except KeyError as exc:
        raise ValueError(f"unknown marker name: {exc.args[0]}") from exc


def _median_vector(vectors: Sequence[Vector3]) -> Vector3:
    """Component-wise robust location used before outlier rejection."""
    count = len(vectors)
    if count == 0:
        raise ValueError("need at least one vector")
    midpoint = count // 2
    values = []
    for index in range(3):
        ordered = sorted(vector[index] for vector in vectors)
        values.append(
            ordered[midpoint] if count % 2 else (ordered[midpoint - 1] + ordered[midpoint]) / 2.0
        )
    return tuple(values)  # type: ignore[return-value]


def trajectory_statistics(
    poses: Sequence[Transform],
    stamps_ns: Sequence[int],
) -> dict[str, float | int | bool]:
    """Measure motion excitation and time coverage from accepted source poses."""
    if len(poses) != len(stamps_ns) or not poses:
        raise ValueError("poses and timestamps must be non-empty and have equal length")
    translation_span = 0.0
    rotation_span = 0.0
    for left_index, left in enumerate(poses):
        for right in poses[left_index + 1:]:
            translation_span = max(
                translation_span,
                _norm(tuple(
                    left.translation[axis] - right.translation[axis]
                    for axis in range(3)
                )),
            )
            rotation_span = max(
                rotation_span,
                rotation_angle_rad(left.quaternion, right.quaternion),
            )
    duration_s = (max(stamps_ns) - min(stamps_ns)) * 1e-9
    return {
        "translation_span_m": translation_span,
        "rotation_span_deg": math.degrees(rotation_span),
        "duration_s": duration_s,
        "average_rate_hz": (
            (len(stamps_ns) - 1) / duration_s
            if duration_s > 0.0 and len(stamps_ns) > 1
            else 0.0
        ),
        "unique_timestamps": len(set(stamps_ns)),
        "timestamps_strictly_increasing": all(
            later > earlier for earlier, later in zip(stamps_ns, stamps_ns[1:])
        ),
    }


def synchronization_statistics(pair_skews_ns: Sequence[int]) -> dict[str, float]:
    if not pair_skews_ns:
        raise ValueError("need at least one synchronized pair")
    skew_ms = [value * 1e-6 for value in pair_skews_ns]
    return {
        "signed_mean_ms": sum(skew_ms) / len(skew_ms),
        "absolute_rms_ms": math.sqrt(sum(value * value for value in skew_ms) / len(skew_ms)),
        "absolute_max_ms": max(abs(value) for value in skew_ms),
    }


def estimate_fixed_transform(
    samples: Sequence[Transform],
    translation_outlier_m: float,
    rotation_outlier_rad: float,
) -> tuple[Transform, list[Transform], dict[str, float], list[int]]:
    """Robustly average fixed transforms and reject gross timing/pose outliers."""
    if len(samples) < 3:
        raise ValueError("need at least three synchronized samples")
    accepted_indices = list(range(len(samples)))
    # Bursts of stale data can move ordinary means far enough that a good
    # cluster rejects itself. Seed translation with a component-wise median
    # and orientation with a geodesic medoid before any averaging.
    estimate = Transform(
        _median_vector([samples[index].translation for index in accepted_indices]),
        quaternion_medoid(samples[index].quaternion for index in accepted_indices),
    )
    for _ in range(4):
        filtered_indices = [
            sample_index for sample_index in accepted_indices
            if _norm(tuple(
                samples[sample_index].translation[axis] - estimate.translation[axis]
                for axis in range(3)
            ))
            <= translation_outlier_m
            and rotation_angle_rad(
                samples[sample_index].quaternion, estimate.quaternion
            ) <= rotation_outlier_rad
        ]
        if len(filtered_indices) < 3:
            raise ValueError("all samples rejected; check acquisition timestamps and frame names")
        if filtered_indices == accepted_indices:
            break
        accepted_indices = filtered_indices
        estimate = Transform(
            _mean_vector([samples[index].translation for index in accepted_indices]),
            quaternion_average(samples[index].quaternion for index in accepted_indices),
        )

    accepted = [samples[index] for index in accepted_indices]
    estimate = Transform(
        _mean_vector([sample.translation for sample in accepted]),
        quaternion_average(sample.quaternion for sample in accepted),
    )
    translation_errors = [
        _norm(tuple(sample.translation[index] - estimate.translation[index] for index in range(3)))
        for sample in accepted
    ]
    rotation_errors = [rotation_angle_rad(sample.quaternion, estimate.quaternion) for sample in accepted]
    statistics = {
        "translation_rms_m": math.sqrt(sum(error * error for error in translation_errors) / len(translation_errors)),
        "translation_max_m": max(translation_errors),
        "rotation_rms_deg": math.degrees(
            math.sqrt(sum(error * error for error in rotation_errors) / len(rotation_errors))
        ),
        "rotation_max_deg": math.degrees(max(rotation_errors)),
    }
    return estimate, accepted, statistics, accepted_indices


def quaternion_to_matrix(quaternion: Quaternion) -> tuple[tuple[float, float, float], ...]:
    x, y, z, w = _normalize_quaternion(quaternion)
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


def axis_angle(quaternion: Quaternion) -> tuple[Vector3, float]:
    x, y, z, w = _normalize_quaternion(quaternion)
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w
    half_sine = math.sqrt(x * x + y * y + z * z)
    if half_sine < 1e-12:
        return (1.0, 0.0, 0.0), 0.0
    return (x / half_sine, y / half_sine, z / half_sine), 2.0 * math.atan2(half_sine, w)


def pose_to_transform(pose) -> Transform:
    return Transform(
        (pose.position.x, pose.position.y, pose.position.z),
        _normalize_quaternion((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)),
    )


def relative_pose_transform(p1_pose, pelvis_pose) -> Transform:
    """Return P1 -> pelvis from two poses expressed in one common frame."""
    return compose(inverse(pose_to_transform(p1_pose)), pose_to_transform(pelvis_pose))


def _json_transform(transform: Transform) -> dict[str, list[float]]:
    return {
        "translation_m": [float(value) for value in transform.translation],
        "quaternion_xyzw": [float(value) for value in transform.quaternion],
    }


def result_document(
    correction: Transform,
    stats: dict[str, float],
    observability: dict,
    synchronization: dict[str, float],
    received_samples: int,
    accepted_samples: int,
    reference_frame: str,
    p1_topic: str,
    pelvis_topic: str,
    p1_frame: str,
    pelvis_frame: str,
    marker_names: Sequence[str] = CURRENT_SHELL_MARKER_NAMES,
) -> dict:
    """Build a self-contained record usable by a setup checklist or launch file."""
    correction_axis, correction_angle = axis_angle(correction.quaternion)
    # Motive's Translation Offset is applied along its *current* local axes.
    # Once those axes have been rotated to pelvis_link, express the old-P1 to
    # new-pelvis displacement in the new pelvis axes.
    pivot_delta_in_pelvis_axes = _rotate(_quat_conjugate(correction.quaternion), correction.translation)
    nominal_centroid = marker_centroid(marker_names)
    nominal_pivot_to_pelvis = tuple(-value for value in nominal_centroid)
    return {
        "cad_cross_check": {
            "source": "agibot/pku/README.md marker coordinates v2",
            "parasolid_asset": (
                "agibot/pku/hip_marker_shell/"
                "a3_hip_marker_shell_p1_mocap_balls_0702.x_t"
            ),
            "current_shell_marker_names": list(CURRENT_SHELL_MARKER_NAMES),
            "selected_marker_names": list(marker_names),
            "marker_centroid_in_pelvis_link_m": [float(value) for value in nominal_centroid],
            "expected_pivot_delta_mm_if_axes_aligned": [
                float(value * 1000.0) for value in nominal_pivot_to_pelvis
            ],
            "warning": (
                "CAD is a cross-check only. The independent live pelvis pose is authoritative. "
                "Marker stream order is irrelevant: live calibration uses only the solved P1 pose. "
                "The 0702 shell uses all ten verified-visible points f1-f5 and b1-b5 by default; "
                "override --marker-names only for a deliberately different physical marker set."
            ),
        },
        "calibration": {
            "method": (
                "For each synchronized PoseStamped pair in one reference frame: "
                "inverse(reference_to_P1) * reference_to_pelvis_link; robustly average "
                "the resulting constant transform."
            ),
            "p1_topic": p1_topic,
            "pelvis_reference_topic": pelvis_topic,
            "pelvis_reference_independence": {
                "status": "operator_precondition_not_observable_from_pose_messages",
                "requirement": (
                    "The pelvis topic must come from an independent full-6DOF source and must "
                    "not descend from P1 or the saved P1-to-pelvis transform."
                ),
            },
            "timestamp_provenance": {
                "status": "operator_precondition_not_inferable_from_header_stamp",
                "requirement": (
                    "Both headers must represent acquisition time mapped into one clock epoch; "
                    "receipt-time equality cannot expose a common systematic latency."
                ),
            },
            "transform_model": "constant_rigid_attachment_P1_to_pelvis_link",
            "reference_frame": reference_frame,
            "p1_frame": p1_frame,
            "pelvis_frame": pelvis_frame,
            "samples_received": received_samples,
            "samples_accepted": accepted_samples,
            "synchronization_measured": synchronization,
            "observability_measured": observability,
            "quality": stats,
        },
        "p1_to_pelvis": {
            "parent_frame": p1_frame,
            "child_frame": pelvis_frame,
            **_json_transform(correction),
            "usage": (
                "Publish this constant P1-to-pelvis_link transform as a runtime static TF. "
                "Alternatively, absorb it into the Motive P1 pivot definition and disable the "
                "static TF. Never apply both."
            ),
        },
        "motive_pivot_registration": {
            "meaning": (
                "Optional alternative to the runtime static TF: change the P1 rigid-body pivot/axes "
                "so its emitted pose is the pelvis_link pose. Rotation maps new pelvis coordinates "
                "into the original P1 frame."
            ),
            "new_axes_in_p1_rotation_matrix": [list(row) for row in quaternion_to_matrix(correction.quaternion)],
            "rotation_quaternion_xyzw": [float(value) for value in correction.quaternion],
            "rotation_axis_in_p1": [float(value) for value in correction_axis],
            "rotation_angle_deg": math.degrees(correction_angle),
            "pivot_delta_m_in_p1_axes": [float(value) for value in correction.translation],
            "pivot_delta_mm_in_p1_axes": [float(value * 1000.0) for value in correction.translation],
            "pivot_delta_mm_in_pelvis_axes_after_rotation": [
                float(value * 1000.0) for value in pivot_delta_in_pelvis_axes
            ],
        },
    }


def _print_result(document: dict) -> None:
    cad = document["cad_cross_check"]
    calibration = document["calibration"]
    fallback = document["p1_to_pelvis"]
    motive = document["motive_pivot_registration"]
    quality = calibration["quality"]
    observability = calibration["observability_measured"]
    synchronization = calibration["synchronization_measured"]
    print("\nP1 / pelvis_link calibration result")
    print("=" * 42)
    print(
        f"Reference: {calibration['reference_frame']}  P1: {calibration['p1_frame']}  "
        f"target: {calibration['pelvis_frame']}"
    )
    print(
        f"Samples: {calibration['samples_accepted']} accepted / {calibration['samples_received']} received\n"
        f"Residual: translation RMS {quality['translation_rms_m'] * 1e3:.2f} mm "
        f"(max {quality['translation_max_m'] * 1e3:.2f} mm); rotation RMS "
        f"{quality['rotation_rms_deg']:.3f} deg (max {quality['rotation_max_deg']:.3f} deg)"
    )
    print(
        "Timing: pair skew RMS "
        f"{synchronization['absolute_rms_ms']:.3f} ms "
        f"(max {synchronization['absolute_max_ms']:.3f} ms)"
    )
    print(
        "Excitation: translation span "
        f"{observability['minimum_translation_span_m'] * 1000.0:.1f} mm; "
        f"rotation span {observability['minimum_rotation_span_deg']:.2f} deg; "
        f"duration {observability['minimum_duration_s']:.2f} s; "
        f"accepted rate {observability['minimum_accepted_rate_hz']:.1f} Hz"
    )
    print("\nSaved constant P1 -> pelvis_link transform (normal runtime use):")
    print(
        "  ros2 run tf2_ros static_transform_publisher "
        f"--x {fallback['translation_m'][0]:.9f} --y {fallback['translation_m'][1]:.9f} "
        f"--z {fallback['translation_m'][2]:.9f} "
        f"--qx {fallback['quaternion_xyzw'][0]:.9f} --qy {fallback['quaternion_xyzw'][1]:.9f} "
        f"--qz {fallback['quaternion_xyzw'][2]:.9f} --qw {fallback['quaternion_xyzw'][3]:.9f} "
        f"--frame-id {fallback['parent_frame']} --child-frame-id {fallback['child_frame']}"
    )
    print("\nOptional Motive direct-registration values (do not also publish the static TF):")
    print(
        "  1. Rotate P1's pivot axes to the reported pelvis axes: "
        f"axis ({motive['rotation_axis_in_p1'][0]:.6f}, "
        f"{motive['rotation_axis_in_p1'][1]:.6f}, {motive['rotation_axis_in_p1'][2]:.6f}), "
        f"angle {motive['rotation_angle_deg']:.6f} deg."
    )
    print(
        "  2. After that rotation, enter Motive Translation Offset (current/local axes, mm): "
        f"({motive['pivot_delta_mm_in_pelvis_axes_after_rotation'][0]:.3f}, "
        f"{motive['pivot_delta_mm_in_pelvis_axes_after_rotation'][1]:.3f}, "
        f"{motive['pivot_delta_mm_in_pelvis_axes_after_rotation'][2]:.3f})."
    )
    print(
        "     Equivalent displacement expressed in the original P1 axes (mm): "
        f"({motive['pivot_delta_mm_in_p1_axes'][0]:.3f}, "
        f"{motive['pivot_delta_mm_in_p1_axes'][1]:.3f}, "
        f"{motive['pivot_delta_mm_in_p1_axes'][2]:.3f})."
    )
    print("\nCAD cross-check (not a substitute for the live result):")
    print(f"  Motive asset markers: {', '.join(cad['selected_marker_names'])}")
    print(
        "  Expected centroid-to-pelvis pivot delta if axes already align (mm): "
        f"({cad['expected_pivot_delta_mm_if_axes_aligned'][0]:.3f}, "
        f"{cad['expected_pivot_delta_mm_if_axes_aligned'][1]:.3f}, "
        f"{cad['expected_pivot_delta_mm_if_axes_aligned'][2]:.3f})"
    )
    print(
        "  3. If using this optional Motive method, save the asset, restart streaming, and rerun "
        "this tool: the correction should be identity."
    )


def _parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Estimate the fixed mocap-P1 -> A3-pelvis_link transform from synchronized ROS 2 data."
    )
    parser.add_argument("--p1-topic", default="/P1/pose", help="mocap P1 PoseStamped topic")
    parser.add_argument(
        "--pelvis-topic",
        required=True,
        help=(
            "independent full-6DOF pelvis_link PoseStamped topic; it must not be derived from P1 "
            "or the saved P1-to-pelvis transform"
        ),
    )
    parser.add_argument(
        "--reference-frame",
        default="world",
        help="required header.frame_id on both input topics (default: world)",
    )
    parser.add_argument("--pelvis-frame", default="pelvis_link", help="A3 URDF pelvis TF frame")
    parser.add_argument(
        "--p1-frame", default="P1",
        help="mocap rigid-body child frame (default: P1)",
    )
    parser.add_argument(
        "--marker-names", default=",".join(CURRENT_SHELL_MARKER_NAMES),
        help=(
            "comma-separated CAD marker-position set; order is irrelevant and defaults to all ten "
            "verified-visible 0702-shell markers (f1,f2,f3,f4,f5,b1,b2,b3,b4,b5)"
        ),
    )
    parser.add_argument("--samples", type=int, default=200, help="accepted synchronized samples to collect")
    parser.add_argument("--timeout", type=float, default=30.0, help="wall-clock timeout in seconds")
    parser.add_argument(
        "--sync-wait", type=float, default=0.5,
        help="maximum seconds to queue either pose while waiting for its matching peer",
    )
    parser.add_argument(
        "--max-pair-skew-ms", type=float, default=2.0,
        help="maximum acquisition-timestamp difference for a P1/pelvis pair (default: 2 ms)",
    )
    parser.add_argument(
        "--min-translation-excitation-m", type=float, default=0.10,
        help="minimum accepted trajectory translation span for both sources (default: 0.10 m)",
    )
    parser.add_argument(
        "--min-rotation-excitation-deg", type=float, default=10.0,
        help="minimum accepted trajectory rotation span for both sources (default: 10 deg)",
    )
    parser.add_argument(
        "--min-sample-duration-s", type=float, default=1.0,
        help="minimum accepted acquisition-time coverage for both sources (default: 1 s)",
    )
    parser.add_argument(
        "--min-accepted-rate-hz", type=float, default=50.0,
        help="minimum accepted acquisition-time rate for both sources (default: 50 Hz)",
    )
    parser.add_argument(
        "--discovery-timeout", type=float, default=3.0,
        help="seconds to wait for publishers before failing with a missing-source error",
    )
    parser.add_argument(
        "--translation-outlier-mm", type=float, default=15.0,
        help="reject an estimate farther than this translation residual (default: 15 mm)",
    )
    parser.add_argument(
        "--rotation-outlier-deg", type=float, default=5.0,
        help="reject an estimate farther than this orientation residual (default: 5 deg)",
    )
    parser.add_argument(
        "--max-translation-rms-mm", type=float, default=3.0,
        help="do not approve Motive settings above this translation RMS (default: 3 mm)",
    )
    parser.add_argument(
        "--max-rotation-rms-deg", type=float, default=1.0,
        help="do not approve Motive settings above this rotation RMS (default: 1 deg)",
    )
    parser.add_argument(
        "--output", type=Path, required=True,
        help="persistent p1_to_pelvis JSON record (required)",
    )
    arguments = parser.parse_args()
    if arguments.samples < 3:
        parser.error("--samples must be at least 3")
    if arguments.timeout <= 0.0:
        parser.error("--timeout must be positive")
    if arguments.sync_wait <= 0.0:
        parser.error("--sync-wait must be positive")
    if arguments.max_pair_skew_ms <= 0.0:
        parser.error("--max-pair-skew-ms must be positive")
    if min(
        arguments.min_translation_excitation_m,
        arguments.min_rotation_excitation_deg,
        arguments.min_sample_duration_s,
        arguments.min_accepted_rate_hz,
    ) <= 0.0:
        parser.error("excitation, duration, and rate thresholds must be positive")
    if arguments.discovery_timeout <= 0.0:
        parser.error("--discovery-timeout must be positive")
    if min(arguments.translation_outlier_mm, arguments.rotation_outlier_deg) <= 0.0:
        parser.error("outlier thresholds must be positive")
    try:
        arguments.marker_names = parse_marker_names(arguments.marker_names)
    except ValueError as exc:
        parser.error(str(exc))
    return arguments


def main() -> int:
    args = _parse_arguments()
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    except ImportError as exc:
        print(f"ROS 2 dependencies are unavailable: {exc}", file=sys.stderr)
        return 2

    class CalibrationSampler(Node):
        def __init__(self) -> None:
            super().__init__("p1_pelvis_calibrator")
            sensor_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )
            self.samples: list[Transform] = []
            self.p1_source_poses: list[Transform] = []
            self.pelvis_source_poses: list[Transform] = []
            self.p1_stamps_ns: list[int] = []
            self.pelvis_stamps_ns: list[int] = []
            self.pair_skews_ns: list[int] = []
            self.p1_received = 0
            self.pelvis_received = 0
            self.sync_misses = 0
            self.frame_misses = 0
            self.pending_p1 = deque()
            self.pending_pelvis = deque()
            self.p1_subscription = self.create_subscription(
                PoseStamped, args.p1_topic, self._on_p1, sensor_qos
            )
            self.pelvis_subscription = self.create_subscription(
                PoseStamped, args.pelvis_topic, self._on_pelvis, sensor_qos
            )
            self.pending_timer = self.create_timer(0.005, self._process_pending)

        def _on_p1(self, message: PoseStamped) -> None:
            self.p1_received += 1
            self.pending_p1.append((time.monotonic(), message))

        def _on_pelvis(self, message: PoseStamped) -> None:
            self.pelvis_received += 1
            self.pending_pelvis.append((time.monotonic(), message))

        @staticmethod
        def _stamp_ns(message: PoseStamped) -> int:
            return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)

        def _process_pending(self) -> None:
            max_skew_ns = int(args.max_pair_skew_ms * 1_000_000.0)
            while self.pending_p1 and self.pending_pelvis and len(self.samples) < args.samples:
                _, p1_message = self.pending_p1[0]
                p1_stamp = self._stamp_ns(p1_message)
                best_index = min(
                    range(len(self.pending_pelvis)),
                    key=lambda index: abs(self._stamp_ns(self.pending_pelvis[index][1]) - p1_stamp),
                )
                pelvis_stamp = self._stamp_ns(self.pending_pelvis[best_index][1])
                skew_ns = pelvis_stamp - p1_stamp
                if abs(skew_ns) <= max_skew_ns:
                    _, p1_message = self.pending_p1.popleft()
                    _, pelvis_message = self.pending_pelvis[best_index]
                    del self.pending_pelvis[best_index]
                    self._try_add_sample(p1_message, pelvis_message)
                    continue
                # Drop only a message that is already older than every possible
                # peer in the other queue; otherwise wait for an out-of-order
                # delivery until --sync-wait expires it.
                if p1_stamp < self._stamp_ns(self.pending_pelvis[0][1]) - max_skew_ns:
                    self.pending_p1.popleft()
                    self.sync_misses += 1
                    continue
                if pelvis_stamp < p1_stamp - max_skew_ns and best_index == 0:
                    self.pending_pelvis.popleft()
                    self.sync_misses += 1
                    continue
                break

            now = time.monotonic()
            for pending in (self.pending_p1, self.pending_pelvis):
                while pending and now - pending[0][0] >= args.sync_wait:
                    pending.popleft()
                    self.sync_misses += 1

        def _try_add_sample(self, p1_message: PoseStamped, pelvis_message: PoseStamped) -> bool:
            p1_parent = p1_message.header.frame_id
            pelvis_parent = pelvis_message.header.frame_id
            if p1_parent != args.reference_frame or pelvis_parent != args.reference_frame:
                self.frame_misses += 1
                if self.frame_misses == 1:
                    self.get_logger().error(
                        "input frame mismatch: expected both PoseStamped headers to be "
                        f"'{args.reference_frame}', got P1='{p1_parent or '<empty>'}' and "
                        f"pelvis='{pelvis_parent or '<empty>'}'"
                    )
                return False
            try:
                reference_to_p1 = pose_to_transform(p1_message.pose)
                reference_to_pelvis = pose_to_transform(pelvis_message.pose)
                p1_to_pelvis = compose(inverse(reference_to_p1), reference_to_pelvis)
            except ValueError:
                return False
            self.samples.append(p1_to_pelvis)
            self.p1_source_poses.append(reference_to_p1)
            self.pelvis_source_poses.append(reference_to_pelvis)
            p1_stamp_ns = self._stamp_ns(p1_message)
            pelvis_stamp_ns = self._stamp_ns(pelvis_message)
            self.p1_stamps_ns.append(p1_stamp_ns)
            self.pelvis_stamps_ns.append(pelvis_stamp_ns)
            self.pair_skews_ns.append(pelvis_stamp_ns - p1_stamp_ns)
            if len(self.samples) % 25 == 0:
                self.get_logger().info(
                    f"collected {len(self.samples)}/{args.samples} synchronized samples "
                    f"({self.sync_misses} synchronization misses, {self.frame_misses} frame mismatches)"
                )
            return True

    rclpy.init()
    node = CalibrationSampler()
    discovery_deadline = time.monotonic() + args.discovery_timeout
    while rclpy.ok() and time.monotonic() < discovery_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.count_publishers(args.p1_topic) > 0 and node.count_publishers(args.pelvis_topic) > 0:
            break
    missing_topics = [
        topic
        for topic in (args.p1_topic, args.pelvis_topic)
        if node.count_publishers(topic) == 0
    ]
    if missing_topics:
        print(
            "CALIBRATION FAIL: no publisher discovered for "
            f"{', '.join(missing_topics)} after {args.discovery_timeout:.1f} s. "
            "The pelvis input must be an independent full-6DOF PoseStamped source in "
            f"'{args.reference_frame}', not the P1-derived runtime TF. The checked-in A3 "
            "hardware bridge publishes pelvis IMU data only and does not satisfy this contract.",
            file=sys.stderr,
        )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 1
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and len(node.samples) < args.samples and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if len(node.samples) < args.samples:
            print(
                f"CALIBRATION FAIL: only {len(node.samples)}/{args.samples} synchronized samples within "
                f"{args.timeout:.1f} s ({node.p1_received} P1 messages, "
                f"{node.pelvis_received} pelvis messages, {node.sync_misses} synchronization misses, "
                f"{node.frame_misses} frame mismatches). Ensure both PoseStamped topics carry "
                f"independent, time-synchronized poses with header.frame_id='{args.reference_frame}'.",
                file=sys.stderr,
            )
            return 1
        correction, accepted, stats, inlier_indices = estimate_fixed_transform(
            node.samples,
            args.translation_outlier_mm / 1000.0,
            math.radians(args.rotation_outlier_deg),
        )
        p1_trajectory = trajectory_statistics(
            [node.p1_source_poses[index] for index in inlier_indices],
            [node.p1_stamps_ns[index] for index in inlier_indices],
        )
        pelvis_trajectory = trajectory_statistics(
            [node.pelvis_source_poses[index] for index in inlier_indices],
            [node.pelvis_stamps_ns[index] for index in inlier_indices],
        )
        minimum_translation_span = min(
            float(p1_trajectory["translation_span_m"]),
            float(pelvis_trajectory["translation_span_m"]),
        )
        minimum_rotation_span = min(
            float(p1_trajectory["rotation_span_deg"]),
            float(pelvis_trajectory["rotation_span_deg"]),
        )
        minimum_duration = min(
            float(p1_trajectory["duration_s"]),
            float(pelvis_trajectory["duration_s"]),
        )
        minimum_unique_timestamps = min(
            int(p1_trajectory["unique_timestamps"]),
            int(pelvis_trajectory["unique_timestamps"]),
        )
        minimum_accepted_rate = min(
            float(p1_trajectory["average_rate_hz"]),
            float(pelvis_trajectory["average_rate_hz"]),
        )
        observability_pass = (
            minimum_translation_span >= args.min_translation_excitation_m
            and minimum_rotation_span >= args.min_rotation_excitation_deg
            and minimum_duration >= args.min_sample_duration_s
            and minimum_unique_timestamps >= max(3, math.ceil(len(accepted) * 0.9))
            and minimum_accepted_rate >= args.min_accepted_rate_hz
            and bool(p1_trajectory["timestamps_strictly_increasing"])
            and bool(pelvis_trajectory["timestamps_strictly_increasing"])
        )
        observability = {
            "gate_passed": observability_pass,
            "requirements": {
                "minimum_translation_span_m": args.min_translation_excitation_m,
                "minimum_rotation_span_deg": args.min_rotation_excitation_deg,
                "minimum_duration_s": args.min_sample_duration_s,
                "minimum_unique_timestamp_fraction": 0.9,
                "minimum_accepted_rate_hz": args.min_accepted_rate_hz,
                "timestamps_strictly_increasing": True,
            },
            "minimum_translation_span_m": minimum_translation_span,
            "minimum_rotation_span_deg": minimum_rotation_span,
            "minimum_duration_s": minimum_duration,
            "minimum_unique_timestamps": minimum_unique_timestamps,
            "minimum_accepted_rate_hz": minimum_accepted_rate,
            "p1_trajectory": p1_trajectory,
            "pelvis_trajectory": pelvis_trajectory,
            "scope": (
                "This necessary excitation gate detects stationary/degenerate collections; "
                "it does not prove source independence or eliminate an unmodelled common "
                "systematic timestamp offset."
            ),
        }
        synchronization = synchronization_statistics(
            [node.pair_skews_ns[index] for index in inlier_indices]
        )
        document = result_document(
            correction, stats, observability, synchronization,
            len(node.samples), len(accepted), args.reference_frame,
            args.p1_topic, args.pelvis_topic, args.p1_frame, args.pelvis_frame, args.marker_names,
        )
        _print_result(document)
        pass_quality = (
            stats["translation_rms_m"] <= args.max_translation_rms_mm / 1000.0
            and stats["rotation_rms_deg"] <= args.max_rotation_rms_deg
        )
        if not pass_quality:
            print(
                "\nCALIBRATION FAIL: residual exceeds the configured quality gate; do not apply the Motive "
                "settings. Check pose time synchronization, reference-frame registration, and marker rigidity.",
                file=sys.stderr,
            )
            return 1
        if not observability_pass:
            print(
                "\nCALIBRATION FAIL: residual consistency passed, but motion excitation did not. "
                "Move and rotate the pelvis smoothly throughout the collection; a stationary or "
                "near-degenerate capture is not accepted.",
                file=sys.stderr,
            )
            return 1
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")
        print(f"\nWrote required p1_to_pelvis calibration record: {args.output}")
        print("\nCALIBRATION PASS")
        return 0
    except ValueError as exc:
        print(f"CALIBRATION FAIL: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
