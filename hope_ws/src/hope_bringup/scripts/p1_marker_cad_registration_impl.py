#!/usr/bin/env python3
"""Register an OptiTrack P1 marker definition to the A3 pelvis CAD markers.

NatNet reports each rigid-body marker centre in the rigid body's local frame.
For a verified correspondence between those centres and the CAD centres in
``pelvis_link``, the fixed transform is directly observable:

    p_P1 = R_P1_pelvis * p_pelvis + t_P1_pelvis

This setup-session tool solves that rigid registration, validates same-frame
labeled-marker samples over multiple P1 headings, and writes an auditable JSON
receipt.  The receipt also records the stationary ``world -> pelvis_link``
snapshot obtained by composing the captured ``world -> P1`` pose with the
fixed registration. It does not edit Motive or ``hope_world_frame.yaml``.

The calculation is deliberately dependency-free outside ROS 2. Math-level
tests and JSON snapshots therefore work on machines without NumPy.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable, Sequence


Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]  # ROS xyzw

MARKER_NAMES = ("f1", "f2", "f3", "f4", "f5", "b1", "b2", "b3", "b4", "b5")
CAD_MARKERS_PELVIS_M: dict[str, Vector3] = {
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
NOMINAL_ONLY_MARKERS = frozenset(("f1", "b1"))
CURRENT_SHELL_MARKERS = tuple(
    name for name in MARKER_NAMES if name not in NOMINAL_ONLY_MARKERS
)


@dataclass(frozen=True)
class Transform:
    translation: Vector3
    quaternion: Quaternion


@dataclass(frozen=True)
class ModelMarker:
    member_id: int
    name: str
    position: Vector3
    required_active_label: int = -1


@dataclass(frozen=True)
class Registration:
    transform: Transform
    residuals_m: tuple[float, ...]
    rms_m: float
    max_m: float
    pairwise_rms_m: float
    scatter_eigenvalues: tuple[float, float, float]


@dataclass(frozen=True)
class Correspondence:
    mapping: dict[int, str]
    mode: str
    registration: Registration
    second_best_rms_m: float | None
    evaluated_assignments: int

    @property
    def margin_m(self) -> float | None:
        if self.second_best_rms_m is None:
            return None
        return self.second_best_rms_m - self.registration.rms_m


@dataclass
class Capture:
    rigid_body_name: str
    rigid_body_id: int
    frame_id: str
    markers: list[ModelMarker]
    frames_received: int = 0
    poses: list[Transform] = field(default_factory=list)
    vendor_timestamps: list[int] = field(default_factory=list)
    mean_marker_errors_m: list[float] = field(default_factory=list)
    live_errors_m: dict[int, list[float]] = field(default_factory=dict)
    live_residuals_m: dict[int, list[float]] = field(default_factory=dict)
    frames_with_physical_samples: int = 0
    definition_drift_max_m: float = 0.0


def _dot(left: Sequence[float], right: Sequence[float]) -> float:
    return sum(a * b for a, b in zip(left, right))


def _norm(vector: Sequence[float]) -> float:
    return math.sqrt(_dot(vector, vector))


def _subtract(left: Vector3, right: Vector3) -> Vector3:
    return (
        left[0] - right[0],
        left[1] - right[1],
        left[2] - right[2],
    )


def _mean(points: Sequence[Vector3]) -> Vector3:
    if not points:
        raise ValueError("at least one point is required")
    count = float(len(points))
    return (
        sum(point[0] for point in points) / count,
        sum(point[1] for point in points) / count,
        sum(point[2] for point in points) / count,
    )


def normalize_quaternion(value: Sequence[float]) -> Quaternion:
    magnitude = _norm(value)
    if magnitude < 1.0e-12 or not math.isfinite(magnitude):
        raise ValueError("zero or non-finite quaternion")
    result = tuple(float(component / magnitude) for component in value)
    if result[3] < 0.0:
        result = tuple(-component for component in result)
    return result  # type: ignore[return-value]


def quaternion_conjugate(value: Quaternion) -> Quaternion:
    return (-value[0], -value[1], -value[2], value[3])


def rotate(value: Quaternion, vector: Vector3) -> Vector3:
    x, y, z, w = normalize_quaternion(value)
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def transform_point(transform: Transform, point: Vector3) -> Vector3:
    rotated = rotate(transform.quaternion, point)
    return (
        rotated[0] + transform.translation[0],
        rotated[1] + transform.translation[1],
        rotated[2] + transform.translation[2],
    )


def quaternion_multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    """Compose two xyzw rotations, applying ``right`` before ``left``."""

    lx, ly, lz, lw = normalize_quaternion(left)
    rx, ry, rz, rw = normalize_quaternion(right)
    return normalize_quaternion(
        (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        )
    )


def compose(left: Transform, right: Transform) -> Transform:
    """Compose parent->middle and middle->child transforms."""

    rotated = rotate(left.quaternion, right.translation)
    return Transform(
        (
            left.translation[0] + rotated[0],
            left.translation[1] + rotated[1],
            left.translation[2] + rotated[2],
        ),
        quaternion_multiply(left.quaternion, right.quaternion),
    )


def representative_transform(samples: Sequence[Transform]) -> Transform | None:
    """Average a stationary pose capture with quaternion hemisphere alignment."""

    if not samples:
        return None
    translation = _mean([sample.translation for sample in samples])
    reference = normalize_quaternion(samples[0].quaternion)
    aligned: list[Quaternion] = []
    for sample in samples:
        quaternion = normalize_quaternion(sample.quaternion)
        if _dot(reference, quaternion) < 0.0:
            quaternion = tuple(-value for value in quaternion)  # type: ignore[assignment]
        aligned.append(quaternion)
    quaternion = normalize_quaternion(
        tuple(
            sum(sample[index] for sample in aligned)
            for index in range(4)
        )
    )
    return Transform(translation, quaternion)


def physical_marker_samples_ready(
    capture: Capture, minimum_live_samples_per_marker: int
) -> bool:
    """Return true only after every ModelDef member has enough live samples."""

    minimum = int(minimum_live_samples_per_marker)
    if minimum < 1:
        raise ValueError("minimum_live_samples_per_marker must be positive")
    return bool(capture.markers) and all(
        len(capture.live_errors_m.get(marker.member_id, ())) >= minimum
        for marker in capture.markers
    )


def world_point_to_body(pose: Transform, point: Vector3) -> Vector3:
    return rotate(
        quaternion_conjugate(normalize_quaternion(pose.quaternion)),
        _subtract(point, pose.translation),
    )


def quaternion_angle_deg(left: Quaternion, right: Quaternion) -> float:
    cosine_half = min(
        1.0,
        abs(_dot(normalize_quaternion(left), normalize_quaternion(right))),
    )
    return math.degrees(2.0 * math.acos(cosine_half))


def _symmetric_eigendecomposition(
    matrix: Sequence[Sequence[float]],
) -> tuple[list[float], list[list[float]]]:
    """Jacobi eigensolver; eigenvectors are returned as matrix columns."""
    size = len(matrix)
    if size == 0 or any(len(row) != size for row in matrix):
        raise ValueError("eigendecomposition requires a non-empty square matrix")
    values = [[float(component) for component in row] for row in matrix]
    vectors = [
        [1.0 if row == column else 0.0 for column in range(size)]
        for row in range(size)
    ]
    for _ in range(100 * size * size):
        p, q = max(
            (
                (row, column)
                for row in range(size)
                for column in range(row + 1, size)
            ),
            key=lambda pair: abs(values[pair[0]][pair[1]]),
        )
        if abs(values[p][q]) <= 1.0e-15:
            break
        angle = 0.5 * math.atan2(
            2.0 * values[p][q], values[p][p] - values[q][q]
        )
        cosine = math.cos(angle)
        sine = math.sin(angle)
        app = values[p][p]
        apq = values[p][q]
        aqq = values[q][q]
        for index in range(size):
            if index == p or index == q:
                continue
            aip = values[index][p]
            aiq = values[index][q]
            values[index][p] = values[p][index] = cosine * aip + sine * aiq
            values[index][q] = values[q][index] = -sine * aip + cosine * aiq
        values[p][p] = (
            cosine * cosine * app
            + 2.0 * sine * cosine * apq
            + sine * sine * aqq
        )
        values[q][q] = (
            sine * sine * app
            - 2.0 * sine * cosine * apq
            + cosine * cosine * aqq
        )
        values[p][q] = values[q][p] = 0.0
        for index in range(size):
            vip = vectors[index][p]
            viq = vectors[index][q]
            vectors[index][p] = cosine * vip + sine * viq
            vectors[index][q] = -sine * vip + cosine * viq
    return [values[index][index] for index in range(size)], vectors


def rigid_registration(
    source_points: Sequence[Vector3],
    target_points: Sequence[Vector3],
) -> Registration:
    """Least-squares proper rigid transform mapping source into target."""
    if len(source_points) != len(target_points) or len(source_points) < 3:
        raise ValueError("rigid registration needs at least three point pairs")
    source_centroid = _mean(source_points)
    target_centroid = _mean(target_points)
    source_centered = [_subtract(point, source_centroid) for point in source_points]
    target_centered = [_subtract(point, target_centroid) for point in target_points]

    scatter = [[0.0] * 3 for _ in range(3)]
    for point in source_centered:
        for row in range(3):
            for column in range(3):
                scatter[row][column] += point[row] * point[column]
    scatter_values, _ = _symmetric_eigendecomposition(scatter)
    scatter_values = sorted((max(0.0, value) for value in scatter_values), reverse=True)
    if scatter_values[1] <= 1.0e-10:
        raise ValueError("marker centres are collinear; orientation is not observable")

    # Davenport/Horn quaternion solution. B = sum(source * target^T).
    covariance = [[0.0] * 3 for _ in range(3)]
    for source, target in zip(source_centered, target_centered):
        for row in range(3):
            for column in range(3):
                covariance[row][column] += source[row] * target[column]
    sigma = sum(covariance[index][index] for index in range(3))
    symmetric = [
        [
            covariance[row][column] + covariance[column][row]
            for column in range(3)
        ]
        for row in range(3)
    ]
    z_vector = (
        covariance[1][2] - covariance[2][1],
        covariance[2][0] - covariance[0][2],
        covariance[0][1] - covariance[1][0],
    )
    horn = [[0.0] * 4 for _ in range(4)]
    for row in range(3):
        for column in range(3):
            horn[row][column] = symmetric[row][column]
            if row == column:
                horn[row][column] -= sigma
        horn[row][3] = horn[3][row] = z_vector[row]
    horn[3][3] = sigma
    horn_values, horn_vectors = _symmetric_eigendecomposition(horn)
    best_index = max(range(4), key=lambda index: horn_values[index])
    quaternion = normalize_quaternion(
        tuple(horn_vectors[row][best_index] for row in range(4))
    )
    rotated_centroid = rotate(quaternion, source_centroid)
    translation = (
        target_centroid[0] - rotated_centroid[0],
        target_centroid[1] - rotated_centroid[1],
        target_centroid[2] - rotated_centroid[2],
    )
    transform = Transform(translation, quaternion)
    residuals = tuple(
        _norm(_subtract(transform_point(transform, source), target))
        for source, target in zip(source_points, target_points)
    )
    pairwise_errors = []
    for left in range(len(source_points)):
        for right in range(left + 1, len(source_points)):
            pairwise_errors.append(
                _norm(_subtract(source_points[left], source_points[right]))
                - _norm(_subtract(target_points[left], target_points[right]))
            )
    return Registration(
        transform=transform,
        residuals_m=residuals,
        rms_m=math.sqrt(sum(value * value for value in residuals) / len(residuals)),
        max_m=max(residuals),
        pairwise_rms_m=math.sqrt(
            sum(value * value for value in pairwise_errors) / len(pairwise_errors)
        ),
        scatter_eigenvalues=tuple(scatter_values),  # type: ignore[arg-type]
    )


def canonical_marker_name(value: str) -> str | None:
    tokens = re.findall(r"(?<![a-z0-9])([fb][1-5])(?![a-z0-9])", value.lower())
    if len(set(tokens)) == 1:
        return tokens[0]
    compact = value.lower().replace("ball_", "").replace("_joint", "")
    if compact in MARKER_NAMES:
        return compact
    return None


def parse_marker_names(value: str) -> tuple[str, ...]:
    names = tuple(item.strip().lower() for item in value.split(",") if item.strip())
    if len(names) < 3:
        raise ValueError("at least three CAD marker names are required")
    if len(set(names)) != len(names):
        raise ValueError("CAD marker names must be unique")
    unknown = sorted(set(names) - set(MARKER_NAMES))
    if unknown:
        raise ValueError(f"unknown CAD marker names: {', '.join(unknown)}")
    return names


def parse_explicit_mapping(value: str) -> dict[int, str]:
    if not value.strip():
        return {}
    result: dict[int, str] = {}
    for item in value.split(","):
        try:
            member_text, name_text = item.split("=", 1)
            member_id = int(member_text.strip())
        except (ValueError, TypeError) as exc:
            raise ValueError(
                "mapping entries must use member_id=CAD_name, e.g. 0=f2"
            ) from exc
        name = name_text.strip().lower()
        if member_id < 0 or name not in CAD_MARKERS_PELVIS_M:
            raise ValueError(f"invalid mapping entry: {item}")
        if member_id in result:
            raise ValueError(f"duplicate member ID in mapping: {member_id}")
        result[member_id] = name
    if len(set(result.values())) != len(result):
        raise ValueError("each CAD marker may appear only once in the mapping")
    return result


def cad_names_for_markers(
    markers: Sequence[ModelMarker],
    requested: Sequence[str] | None = None,
) -> tuple[str, ...]:
    if requested is not None:
        if len(requested) != len(markers):
            raise ValueError(
                f"selected {len(requested)} CAD markers but Motive defines "
                f"{len(markers)} P1 markers"
            )
        return tuple(requested)
    if len(markers) == len(CURRENT_SHELL_MARKERS):
        return CURRENT_SHELL_MARKERS
    if len(markers) == len(MARKER_NAMES):
        return MARKER_NAMES
    raise ValueError(
        f"P1 ModelDef contains {len(markers)} markers; automatic selection "
        "supports only the realized 8-marker shell or the complete 10-marker table"
    )


def _distance_matrix(points: Sequence[Vector3]) -> list[list[float]]:
    return [
        [_norm(_subtract(left, right)) for right in points]
        for left in points
    ]


def _signature_cost(
    first_distances: Sequence[float], second_distances: Sequence[float]
) -> float:
    first = sorted(first_distances)
    second = sorted(second_distances)
    return math.sqrt(
        sum((left - right) ** 2 for left, right in zip(first, second))
        / len(first)
    )


def resolve_correspondence(
    markers: Sequence[ModelMarker],
    cad_names: Sequence[str],
    explicit_mapping: dict[int, str] | None = None,
    beam_width: int = 20000,
) -> Correspondence:
    """Resolve marker-to-CAD labels, using names first and geometry otherwise."""
    if len(markers) != len(cad_names):
        raise ValueError("Motive and CAD marker sets have different sizes")
    if len({marker.member_id for marker in markers}) != len(markers):
        raise ValueError("Motive ModelDef member IDs are not unique")
    if len(set(cad_names)) != len(cad_names):
        raise ValueError("CAD marker set is not unique")

    cad_names = tuple(cad_names)
    cad_points = [CAD_MARKERS_PELVIS_M[name] for name in cad_names]
    model_points = [marker.position for marker in markers]
    explicit_mapping = explicit_mapping or {}
    if explicit_mapping:
        marker_ids = {marker.member_id for marker in markers}
        if set(explicit_mapping) != marker_ids:
            missing = sorted(marker_ids - set(explicit_mapping))
            extra = sorted(set(explicit_mapping) - marker_ids)
            raise ValueError(
                f"explicit mapping must cover every ModelDef member ID; "
                f"missing={missing}, extra={extra}"
            )
        if set(explicit_mapping.values()) != set(cad_names):
            raise ValueError("explicit mapping does not exactly cover the selected CAD set")
        source = [
            CAD_MARKERS_PELVIS_M[explicit_mapping[marker.member_id]]
            for marker in markers
        ]
        registration = rigid_registration(source, model_points)
        return Correspondence(
            mapping=dict(explicit_mapping),
            mode="operator_explicit_member_id_mapping",
            registration=registration,
            second_best_rms_m=None,
            evaluated_assignments=1,
        )

    parsed_names = [canonical_marker_name(marker.name) for marker in markers]
    if (
        all(name is not None for name in parsed_names)
        and len(set(parsed_names)) == len(parsed_names)
        and set(parsed_names) == set(cad_names)
    ):
        mapping = {
            marker.member_id: parsed_names[index]  # type: ignore[index]
            for index, marker in enumerate(markers)
        }
        registration = rigid_registration(
            [CAD_MARKERS_PELVIS_M[mapping[marker.member_id]] for marker in markers],
            model_points,
        )
        return Correspondence(
            mapping=mapping,
            mode="natnet_marker_names",
            registration=registration,
            second_best_rms_m=None,
            evaluated_assignments=1,
        )

    model_distances = _distance_matrix(model_points)
    cad_distances = _distance_matrix(cad_points)
    signature_costs = [
        [
            _signature_cost(model_distances[model_index], cad_distances[cad_index])
            for cad_index in range(len(cad_names))
        ]
        for model_index in range(len(markers))
    ]
    candidates: list[list[int]] = []
    for model_index, marker in enumerate(markers):
        parsed = parsed_names[model_index]
        if parsed in cad_names:
            candidates.append([cad_names.index(parsed)])
            continue
        ranked = sorted(
            range(len(cad_names)),
            key=lambda cad_index: signature_costs[model_index][cad_index],
        )
        best = signature_costs[model_index][ranked[0]]
        selected = [
            cad_index
            for cad_index in ranked
            if signature_costs[model_index][cad_index] <= best + 0.006
        ][:6]
        # Retain enough alternatives to resolve mirror-symmetric signatures.
        candidates.append(selected if len(selected) >= 2 else ranked[:2])

    order = sorted(
        range(len(markers)),
        key=lambda index: (
            len(candidates[index]),
            signature_costs[index][candidates[index][0]],
            index,
        ),
    )
    # state = (incremental invariant cost, assignments by model index, used mask)
    states: list[tuple[float, tuple[int, ...], int]] = [
        (0.0, tuple([-1] * len(markers)), 0)
    ]
    for model_index in order:
        expanded: list[tuple[float, tuple[int, ...], int]] = []
        for cost, assignment, used_mask in states:
            for cad_index in candidates[model_index]:
                bit = 1 << cad_index
                if used_mask & bit:
                    continue
                incremental = 0.01 * signature_costs[model_index][cad_index] ** 2
                for other_model, other_cad in enumerate(assignment):
                    if other_cad < 0:
                        continue
                    difference = (
                        model_distances[model_index][other_model]
                        - cad_distances[cad_index][other_cad]
                    )
                    incremental += difference * difference
                updated = list(assignment)
                updated[model_index] = cad_index
                expanded.append((cost + incremental, tuple(updated), used_mask | bit))
        if not expanded:
            raise ValueError("no bijective marker correspondence satisfies the constraints")
        expanded.sort(key=lambda state: (state[0], state[1]))
        states = expanded[:beam_width]

    # Pairwise distances cannot distinguish an improper mirror. Evaluate the
    # best invariant assignments with a proper 3-D rigid fit to resolve it.
    evaluated = []
    for invariant_cost, assignment, _ in states[: min(4096, len(states))]:
        source = [cad_points[cad_index] for cad_index in assignment]
        try:
            registration = rigid_registration(source, model_points)
        except ValueError:
            continue
        evaluated.append((registration.rms_m, invariant_cost, assignment, registration))
    if not evaluated:
        raise ValueError("no geometrically valid marker correspondence was found")
    evaluated.sort(key=lambda item: (item[0], item[1], item[2]))
    best = evaluated[0]
    second_rms = evaluated[1][0] if len(evaluated) > 1 else None
    mapping = {
        marker.member_id: cad_names[best[2][index]]
        for index, marker in enumerate(markers)
    }
    return Correspondence(
        mapping=mapping,
        mode="geometry_inferred_from_pairwise_distances",
        registration=best[3],
        second_best_rms_m=second_rms,
        evaluated_assignments=len(evaluated),
    )


def _trajectory_span(poses: Sequence[Transform]) -> dict[str, float]:
    if not poses:
        return {"translation_span_m": 0.0, "rotation_span_deg": 0.0}
    if len(poses) > 200:
        indices = [
            round(index * (len(poses) - 1) / 199.0)
            for index in range(200)
        ]
        sampled = [poses[index] for index in indices]
    else:
        sampled = list(poses)
    translation_span = 0.0
    rotation_span = 0.0
    for left_index, left in enumerate(sampled):
        for right in sampled[left_index + 1 :]:
            translation_span = max(
                translation_span,
                _norm(_subtract(left.translation, right.translation)),
            )
            rotation_span = max(
                rotation_span,
                quaternion_angle_deg(left.quaternion, right.quaternion),
            )
    return {
        "translation_span_m": translation_span,
        "rotation_span_deg": rotation_span,
    }


def _rms(values: Iterable[float]) -> float | None:
    sequence = [value for value in values if math.isfinite(value)]
    if not sequence:
        return None
    return math.sqrt(sum(value * value for value in sequence) / len(sequence))


def analyze_capture(
    capture: Capture,
    cad_names: Sequence[str],
    explicit_mapping: dict[int, str],
    *,
    max_registration_rms_m: float,
    max_registration_max_m: float,
    max_pairwise_rms_m: float,
    minimum_mapping_margin_m: float,
    minimum_live_samples_per_marker: int,
    max_live_rms_m: float,
    max_live_max_m: float,
    minimum_rotation_span_deg: float,
    operator_attested_installed_layout: bool,
    allow_nominal_only_markers: bool,
) -> tuple[dict, list[str]]:
    correspondence = resolve_correspondence(
        capture.markers, cad_names, explicit_mapping
    )
    registration = correspondence.registration
    blockers: list[str] = []
    if registration.rms_m > max_registration_rms_m:
        blockers.append(
            f"CAD registration RMS {registration.rms_m * 1000.0:.2f} mm "
            f"exceeds {max_registration_rms_m * 1000.0:.2f} mm"
        )
    if registration.max_m > max_registration_max_m:
        blockers.append(
            f"CAD registration max {registration.max_m * 1000.0:.2f} mm "
            f"exceeds {max_registration_max_m * 1000.0:.2f} mm"
        )
    if registration.pairwise_rms_m > max_pairwise_rms_m:
        blockers.append(
            f"pairwise-distance RMS {registration.pairwise_rms_m * 1000.0:.2f} mm "
            f"exceeds {max_pairwise_rms_m * 1000.0:.2f} mm"
        )
    if (
        correspondence.mode == "geometry_inferred_from_pairwise_distances"
        and (
            correspondence.margin_m is None
            or correspondence.margin_m < minimum_mapping_margin_m
        )
    ):
        measured = (
            "unavailable"
            if correspondence.margin_m is None
            else f"{correspondence.margin_m * 1000.0:.2f} mm"
        )
        blockers.append(
            f"marker correspondence is ambiguous: best-to-second margin {measured}"
        )

    selected_nominal_only = sorted(set(cad_names) & NOMINAL_ONLY_MARKERS)
    if selected_nominal_only and not allow_nominal_only_markers:
        blockers.append(
            "selected f1/b1, but the reference 0702 shell documents them as "
            "nominal-only; confirm installed/measured mounts and rerun with "
            "--allow-nominal-only-markers"
        )
    if not operator_attested_installed_layout:
        blockers.append(
            "missing --attest-installed-layout operator confirmation"
        )

    per_marker_live = {}
    all_live_errors = []
    for marker in capture.markers:
        errors = capture.live_errors_m.get(marker.member_id, [])
        residuals = capture.live_residuals_m.get(marker.member_id, [])
        all_live_errors.extend(errors)
        marker_rms = _rms(errors)
        per_marker_live[str(marker.member_id)] = {
            "cad_name": correspondence.mapping[marker.member_id],
            "physical_samples": len(errors),
            "local_position_rms_m": marker_rms,
            "local_position_max_m": max(errors) if errors else None,
            "natnet_reconstruction_residual_rms_m": _rms(residuals),
        }
        if len(errors) < minimum_live_samples_per_marker:
            blockers.append(
                f"member {marker.member_id} "
                f"({correspondence.mapping[marker.member_id]}) has only "
                f"{len(errors)} physical samples; need "
                f"{minimum_live_samples_per_marker}"
            )
    live_rms = _rms(all_live_errors)
    if live_rms is None:
        blockers.append(
            "no point-cloud-solved, non-occluded labeled-marker samples; "
            "enable Motive Streaming / Labeled Markers"
        )
    elif live_rms > max_live_rms_m:
        blockers.append(
            f"live marker-to-ModelDef RMS {live_rms * 1000.0:.2f} mm "
            f"exceeds {max_live_rms_m * 1000.0:.2f} mm"
        )
    live_max = max(all_live_errors) if all_live_errors else None
    if live_max is not None and live_max > max_live_max_m:
        blockers.append(
            f"live marker-to-ModelDef max {live_max * 1000.0:.2f} mm "
            f"exceeds {max_live_max_m * 1000.0:.2f} mm"
        )

    trajectory = _trajectory_span(capture.poses)
    if trajectory["rotation_span_deg"] < minimum_rotation_span_deg:
        blockers.append(
            f"P1 heading span {trajectory['rotation_span_deg']:.2f} deg is below "
            f"{minimum_rotation_span_deg:.2f} deg"
        )

    qx, qy, qz, qw = registration.transform.quaternion
    tx, ty, tz = registration.transform.translation
    reference_to_p1 = representative_transform(capture.poses)
    reference_to_pelvis = (
        compose(reference_to_p1, registration.transform)
        if reference_to_p1 is not None
        else None
    )
    if reference_to_pelvis is None:
        blockers.append("no P1 rigid-body pose samples for world-to-pelvis snapshot")
    marker_records = []
    for marker in capture.markers:
        cad_name = correspondence.mapping[marker.member_id]
        marker_records.append(
            {
                "member_id": marker.member_id,
                "natnet_name": marker.name,
                "required_active_label": marker.required_active_label,
                "cad_name": cad_name,
                "model_position_in_P1_m": list(marker.position),
                "cad_position_in_pelvis_link_m": list(
                    CAD_MARKERS_PELVIS_M[cad_name]
                ),
                "fit_residual_m": registration.residuals_m[
                    capture.markers.index(marker)
                ],
            }
        )

    document = {
        "schema": "hope.p1_marker_cad_registration_receipt.v1",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "approved": not blockers,
        "blockers": blockers,
        "method": {
            "name": "NatNet_ModelDef_to_A3_CAD_rigid_registration",
            "equation": (
                "p_P1 = R_P1_pelvis_link * p_pelvis_link + "
                "t_P1_pelvis_link"
            ),
            "correspondence_mode": correspondence.mode,
            "evaluated_assignments": correspondence.evaluated_assignments,
            "trajectory_validation": (
                "stationary_named_marker_geometry"
                if minimum_rotation_span_deg == 0.0
                else "multi_heading_live_validation"
            ),
            "limitation": (
                "This observes P1 relative to the CAD marker centres. It does "
                "not independently measure pelvis_link; validity depends on "
                "the installed shell and marker centres matching the cited CAD."
            ),
        },
        "source": {
            "topic_type": (
                "motion_capture_tracking_interfaces/msg/RigidBodyMarkerArray"
            ),
            "rigid_body_name": capture.rigid_body_name,
            "rigid_body_id": capture.rigid_body_id,
            "reference_frame": capture.frame_id,
            "frames_received": capture.frames_received,
            "frames_with_physical_samples": capture.frames_with_physical_samples,
            "definition_drift_max_m": capture.definition_drift_max_m,
        },
        "cad": {
            "coordinate_source": "agibot/pku/README.md marker table v2",
            "selected_marker_names": list(cad_names),
            "nominal_only_marker_names": selected_nominal_only,
            "operator_attested_installed_layout": (
                operator_attested_installed_layout
            ),
        },
        "correspondence": {
            "best_to_second_rms_margin_m": correspondence.margin_m,
            "markers": marker_records,
        },
        "quality": {
            "registration_rms_m": registration.rms_m,
            "registration_max_m": registration.max_m,
            "pairwise_distance_rms_m": registration.pairwise_rms_m,
            "second_best_registration_rms_m": correspondence.second_best_rms_m,
            "source_scatter_eigenvalues_m2": list(
                registration.scatter_eigenvalues
            ),
            "live_local_position_rms_m": live_rms,
            "live_local_position_max_m": live_max,
            "vendor_mean_marker_error_rms_m": _rms(
                capture.mean_marker_errors_m
            ),
            "trajectory": trajectory,
            "per_marker_live": per_marker_live,
        },
        "p1_to_pelvis_link": {
            "parent_frame": "P1",
            "child_frame": "pelvis_link",
            "xyz_m": [tx, ty, tz],
            "quaternion_wxyz": [qw, qx, qy, qz],
            "quaternion_xyzw": [qx, qy, qz, qw],
        },
        # Canonical runtime spelling shared with p1_pelvis_calibrator and the
        # policy localization relay.  Keep p1_to_pelvis_link above for receipt
        # compatibility with the existing audited marker/CAD records.
        "p1_to_pelvis": {
            "parent_frame": "P1",
            "child_frame": "pelvis_link",
            "translation_m": [tx, ty, tz],
            "quaternion_xyzw": [qx, qy, qz, qw],
        },
        # An audited snapshot of the derived pose at the stationary calibration
        # instant. This satisfies operator provenance without turning the
        # moving robot's world pose into a static runtime transform. Runtime
        # localization still composes live world->P1 with P1->pelvis_link.
        "world_to_pelvis_snapshot": (
            None
            if reference_to_pelvis is None
            else {
                "parent_frame": capture.frame_id,
                "child_frame": "pelvis_link",
                "translation_m": list(reference_to_pelvis.translation),
                "quaternion_xyzw": list(reference_to_pelvis.quaternion),
                "sample_count": len(capture.poses),
                "semantics": (
                    "stationary calibration snapshot for audit; not a static "
                    "runtime world-to-pelvis transform"
                ),
            }
        ),
        "hope_world_frame_yaml_candidate": {
            "path": "hope_world.mocap_to_base_link.p1",
            "calibrated": not blockers,
            "calibration_sha256": (
                "fill with SHA-256 of the finalized receipt file"
            ),
            "xyz_m": [tx, ty, tz],
            "quaternion_wxyz": [qw, qx, qy, qz],
        },
    }
    return document, blockers


def capture_from_json(path: Path) -> Capture:
    data = json.loads(path.read_text(encoding="utf-8"))
    markers = [
        ModelMarker(
            member_id=int(marker["member_id"]),
            name=str(marker.get("name", "")),
            position=tuple(
                float(value) for value in marker["model_position_m"]
            ),  # type: ignore[arg-type]
            required_active_label=int(
                marker.get("required_active_label", -1)
            ),
        )
        for marker in data["markers"]
    ]
    return Capture(
        rigid_body_name=str(data.get("rigid_body_name", "P1")),
        rigid_body_id=int(data.get("rigid_body_id", -1)),
        frame_id=str(data.get("frame_id", "world")),
        markers=markers,
        frames_received=int(data.get("frames_received", 1)),
    )


def collect_ros_capture(args: argparse.Namespace) -> Capture:
    try:
        import rclpy
        from motion_capture_tracking_interfaces.msg import RigidBodyMarkerArray
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
    except ImportError as exc:
        raise RuntimeError(f"ROS 2 dependencies are unavailable: {exc}") from exc

    class Collector(Node):
        def __init__(self) -> None:
            super().__init__("p1_marker_cad_calibrator")
            self.capture: Capture | None = None
            self.first_frame_monotonic: float | None = None
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )
            self.subscription = self.create_subscription(
                RigidBodyMarkerArray, args.topic, self.on_message, qos
            )

        @staticmethod
        def point(value) -> Vector3:
            return (float(value.x), float(value.y), float(value.z))

        @staticmethod
        def pose(value) -> Transform:
            return Transform(
                (
                    float(value.position.x),
                    float(value.position.y),
                    float(value.position.z),
                ),
                normalize_quaternion(
                    (
                        float(value.orientation.x),
                        float(value.orientation.y),
                        float(value.orientation.z),
                        float(value.orientation.w),
                    )
                ),
            )

        def on_message(self, message) -> None:
            if message.rigid_body_name != args.asset_name:
                return
            incoming_markers = [
                ModelMarker(
                    member_id=int(marker.member_id),
                    name=str(marker.name),
                    position=self.point(marker.model_position),
                    required_active_label=int(marker.required_active_label),
                )
                for marker in message.markers
            ]
            if self.capture is None:
                if not incoming_markers:
                    return
                self.capture = Capture(
                    rigid_body_name=str(message.rigid_body_name),
                    rigid_body_id=int(message.rigid_body_id),
                    frame_id=str(message.header.frame_id),
                    markers=incoming_markers,
                )
                self.first_frame_monotonic = time.monotonic()
                for marker in incoming_markers:
                    self.capture.live_errors_m[marker.member_id] = []
                    self.capture.live_residuals_m[marker.member_id] = []
            capture = self.capture
            assert capture is not None
            if int(message.rigid_body_id) != capture.rigid_body_id:
                return
            if str(message.header.frame_id) != capture.frame_id:
                return
            baseline = {marker.member_id: marker for marker in capture.markers}
            if set(baseline) != {marker.member_id for marker in incoming_markers}:
                return
            for marker in incoming_markers:
                drift = _norm(
                    _subtract(marker.position, baseline[marker.member_id].position)
                )
                capture.definition_drift_max_m = max(
                    capture.definition_drift_max_m, drift
                )

            pose = self.pose(message.rigid_body_pose)
            capture.frames_received += 1
            capture.poses.append(pose)
            capture.vendor_timestamps.append(int(message.timestamp))
            mean_error = float(message.mean_marker_error_m)
            if math.isfinite(mean_error):
                capture.mean_marker_errors_m.append(mean_error)
            physical_in_frame = False
            for marker_message in message.markers:
                member_id = int(marker_message.member_id)
                params = int(marker_message.params)
                # NatNet: bit 0 occluded, bit 1 point-cloud solved. Reject
                # model-filled positions because they are predictions.
                physical = (
                    bool(marker_message.has_live_sample)
                    and (params & 0x01) == 0
                    and (params & 0x02) != 0
                )
                if not physical:
                    continue
                live_world = self.point(marker_message.position)
                if not all(math.isfinite(value) for value in live_world):
                    continue
                local = world_point_to_body(pose, live_world)
                error = _norm(
                    _subtract(local, baseline[member_id].position)
                )
                capture.live_errors_m[member_id].append(error)
                residual = float(marker_message.residual_m)
                if math.isfinite(residual):
                    capture.live_residuals_m[member_id].append(residual)
                physical_in_frame = True
            if physical_in_frame:
                capture.frames_with_physical_samples += 1

    rclpy.init()
    collector = Collector()
    started = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(collector, timeout_sec=0.05)
            elapsed = time.monotonic() - started
            if collector.capture is not None:
                capture_elapsed = (
                    time.monotonic()
                    - (collector.first_frame_monotonic or time.monotonic())
                )
                if (
                    collector.capture.frames_received >= args.minimum_frames
                    and capture_elapsed >= args.capture_duration
                    and physical_marker_samples_ready(
                        collector.capture, args.minimum_live_samples_per_marker
                    )
                ):
                    return collector.capture
            if elapsed >= args.timeout:
                if collector.capture is None:
                    raise RuntimeError(
                        f"no {args.asset_name} marker messages arrived on "
                        f"{args.topic} within {args.timeout:.1f} s"
                    )
                if collector.capture.frames_received >= args.minimum_frames:
                    # Preserve the detailed per-marker blocker report. A
                    # temporarily occluded marker gets the full timeout to
                    # recover, but a persistent occlusion still fails closed.
                    return collector.capture
                raise RuntimeError(
                    f"capture timed out after {args.timeout:.1f} s with "
                    f"{collector.capture.frames_received} frames"
                )
    finally:
        collector.destroy_node()
        rclpy.shutdown()
    raise RuntimeError("ROS 2 shut down before capture completed")


def _parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Calculate P1 -> pelvis_link from NatNet P1 marker centres and "
            "the A3 CAD marker table"
        )
    )
    parser.add_argument(
        "--topic", default="/optitrack/rigid_body_markers"
    )
    parser.add_argument("--asset-name", default="P1")
    parser.add_argument(
        "--input-json",
        type=Path,
        help="offline ModelDef snapshot instead of a live ROS subscription",
    )
    parser.add_argument(
        "--output", type=Path, required=True, help="JSON receipt path"
    )
    parser.add_argument(
        "--marker-names",
        default="auto",
        help=(
            "'auto', or comma-separated CAD names. Auto selects f2-f5,b2-b5 "
            "for 8 points and f1-f5,b1-b5 for 10 points."
        ),
    )
    parser.add_argument(
        "--mapping",
        default="",
        help=(
            "optional verified member_id=CAD_name list, e.g. "
            "'0=f2,1=f3,...'; otherwise NatNet names or geometry are used"
        ),
    )
    parser.add_argument("--minimum-frames", type=int, default=300)
    parser.add_argument("--capture-duration", type=float, default=12.0)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--max-registration-rms-mm", type=float, default=3.0)
    parser.add_argument("--max-registration-max-mm", type=float, default=6.0)
    # Pairwise errors compare two noisy marker centres, so their natural scale
    # is about sqrt(2) times the per-marker registration residual. Keep this
    # consistent with the 3 mm registration-RMS gate.
    parser.add_argument("--max-pairwise-rms-mm", type=float, default=5.0)
    parser.add_argument("--minimum-mapping-margin-mm", type=float, default=1.5)
    parser.add_argument("--minimum-live-samples-per-marker", type=int, default=30)
    parser.add_argument("--max-live-rms-mm", type=float, default=4.0)
    parser.add_argument("--max-live-max-mm", type=float, default=5.0)
    parser.add_argument("--minimum-rotation-span-deg", type=float, default=10.0)
    parser.add_argument(
        "--stationary-prepare",
        action="store_true",
        help=(
            "allow a stationary PD_STAND capture; the fixed transform is "
            "observable from the named 3-D marker geometry, while live "
            "per-marker residual gates remain mandatory"
        ),
    )
    parser.add_argument(
        "--attest-installed-layout",
        action="store_true",
        help=(
            "confirm the selected physical marker centres and rigid shell "
            "installation match the cited A3 CAD"
        ),
    )
    parser.add_argument(
        "--allow-nominal-only-markers",
        action="store_true",
        help=(
            "allow f1/b1 only after their physical mounts have been installed "
            "and independently confirmed"
        ),
    )
    args = parser.parse_args()
    if args.minimum_frames < 3:
        parser.error("--minimum-frames must be at least 3")
    if min(args.capture_duration, args.timeout) <= 0.0:
        parser.error("capture duration and timeout must be positive")
    if args.timeout <= args.capture_duration:
        parser.error("--timeout must be greater than --capture-duration")
    if args.minimum_live_samples_per_marker < 1:
        parser.error("--minimum-live-samples-per-marker must be positive")
    positive_thresholds = (
        args.max_registration_rms_mm,
        args.max_registration_max_mm,
        args.max_pairwise_rms_mm,
        args.minimum_mapping_margin_mm,
        args.max_live_rms_mm,
        args.max_live_max_mm,
    )
    if min(positive_thresholds) <= 0.0:
        parser.error("registration/live quality thresholds must be positive")
    if args.minimum_rotation_span_deg < 0.0:
        parser.error("--minimum-rotation-span-deg must be non-negative")
    if args.stationary_prepare:
        args.minimum_rotation_span_deg = 0.0
    try:
        args.explicit_mapping = parse_explicit_mapping(args.mapping)
        args.selected_marker_names = (
            None
            if args.marker_names.strip().lower() == "auto"
            else parse_marker_names(args.marker_names)
        )
    except ValueError as exc:
        parser.error(str(exc))
    return args


def _write_bytes_atomic(path: Path, encoded: bytes) -> None:
    """Durably replace one receipt without exposing a partial JSON file."""

    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(
        f".{path.name}.{os.getpid()}.{time.time_ns()}.tmp"
    )
    try:
        with temporary.open("xb") as handle:
            handle.write(encoded)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        try:
            directory_fd = os.open(path.parent, os.O_RDONLY)
        except OSError:
            return
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def _rejected_receipt_path(output: Path) -> Path:
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return output.with_name(f"{output.stem}.rejected.{timestamp}{output.suffix}")


def main() -> int:
    args = _parse_arguments()
    try:
        capture = (
            capture_from_json(args.input_json)
            if args.input_json is not None
            else collect_ros_capture(args)
        )
        cad_names = cad_names_for_markers(
            capture.markers, args.selected_marker_names
        )
        document, blockers = analyze_capture(
            capture,
            cad_names,
            args.explicit_mapping,
            max_registration_rms_m=args.max_registration_rms_mm * 1.0e-3,
            max_registration_max_m=args.max_registration_max_mm * 1.0e-3,
            max_pairwise_rms_m=args.max_pairwise_rms_mm * 1.0e-3,
            minimum_mapping_margin_m=args.minimum_mapping_margin_mm * 1.0e-3,
            minimum_live_samples_per_marker=args.minimum_live_samples_per_marker,
            max_live_rms_m=args.max_live_rms_mm * 1.0e-3,
            max_live_max_m=args.max_live_max_mm * 1.0e-3,
            minimum_rotation_span_deg=args.minimum_rotation_span_deg,
            operator_attested_installed_layout=args.attest_installed_layout,
            allow_nominal_only_markers=args.allow_nominal_only_markers,
        )
    except (OSError, KeyError, TypeError, ValueError, RuntimeError) as exc:
        print(f"calibration failed: {exc}", file=sys.stderr)
        return 2

    encoded = (
        json.dumps(document, indent=2, sort_keys=True, allow_nan=False) + "\n"
    ).encode("utf-8")
    receipt_sha256 = hashlib.sha256(encoded).hexdigest()
    transform = document["p1_to_pelvis_link"]
    print(f"receipt_sha256: {receipt_sha256}")
    print(f"approved: {document['approved']}")
    print(f"P1 -> pelvis_link xyz_m: {transform['xyz_m']}")
    print(
        "P1 -> pelvis_link quaternion_wxyz: "
        f"{transform['quaternion_wxyz']}"
    )
    if blockers:
        rejected_path = _rejected_receipt_path(args.output)
        _write_bytes_atomic(rejected_path, encoded)
        print(f"rejected_receipt: {rejected_path}")
        print(
            f"preserved_last_approved_receipt: {args.output}",
            file=sys.stderr,
        )
        print("blockers:", file=sys.stderr)
        for blocker in blockers:
            print(f"  - {blocker}", file=sys.stderr)
        return 1
    _write_bytes_atomic(args.output, encoded)
    print(f"receipt: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
