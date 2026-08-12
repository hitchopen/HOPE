#!/usr/bin/env python3
"""Audit OptiTrack Ball quaternions without affecting a runtime command.

The report deliberately describes observability, not command admissibility.
Quaternion sign changes are corrected before differencing.  A single angular
increment above ``--max-rev-s`` is classified as a possible marker relock and
discarded, while the new quaternion remains the reference for the next frame,
matching the C++ SpinEstimator semantics.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter
from pathlib import Path


TWO_PI = 2.0 * math.pi


def quantile(values: list[float], fraction: float) -> float | None:
    finite = sorted(value for value in values if math.isfinite(value))
    if not finite:
        return None
    position = (len(finite) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return finite[lower]
    return finite[lower] * (upper - position) + finite[upper] * (position - lower)


def finite_float(row: dict[str, str], key: str) -> float | None:
    try:
        value = float(row.get(key, ""))
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def normalize_xyzw(values: tuple[float, float, float, float]):
    norm = math.sqrt(sum(value * value for value in values))
    if not math.isfinite(norm) or not 0.5 <= norm <= 1.5:
        return None, norm
    return tuple(value / norm for value in values), norm


def dot(a, b) -> float:
    return sum(x * y for x, y in zip(a, b))


def relative_rotation_vector(current, previous):
    # q_current * inverse(q_previous), both in x/y/z/w order.
    x1, y1, z1, w1 = current
    x2, y2, z2, w2 = (-previous[0], -previous[1], -previous[2], previous[3])
    relative = (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )
    if relative[3] < 0.0:
        relative = tuple(-value for value in relative)
    vector_norm = math.sqrt(sum(value * value for value in relative[:3]))
    if vector_norm < 1.0e-12:
        return (0.0, 0.0, 0.0)
    angle = 2.0 * math.atan2(vector_norm, relative[3])
    return tuple(angle * value / vector_norm for value in relative[:3])


def analyze(
    path: Path,
    max_gap_s: float,
    max_rev_s: float,
    min_span_s: float,
    shot_predictions: Path | None,
    prefix_skip_s: float,
):
    raw_ball_rows = 0
    accepted_ball_rows = 0
    valid_orientation_rows = 0
    quaternion_norms: list[float] = []
    orientation_status = Counter()
    raw_sign_transitions = 0
    continuity_sign_negations = 0
    nonmonotonic_timestamps = 0
    source_gaps = 0
    maximum_source_gap_s = 0.0
    impossible_increments = 0
    retained_rates_rev_s: list[float] = []
    incoming_rates_rev_s: list[float] = []
    incoming_run_durations: list[float] = []
    usable_incoming_runs = 0

    previous = None
    previous_raw_quaternion = None
    run_start = None
    run_end = None
    run_retained = 0

    def finish_run() -> None:
        nonlocal run_start, run_end, run_retained, usable_incoming_runs
        if run_start is None or run_end is None:
            run_start = run_end = None
            run_retained = 0
            return
        duration = max(0.0, run_end - run_start)
        incoming_run_durations.append(duration)
        if duration >= min_span_s and run_retained >= 3:
            usable_incoming_runs += 1
        run_start = run_end = None
        run_retained = 0

    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            if (
                row.get("object_key", "").lower() != "ball"
                and row.get("object_name", "").lower() != "ball"
            ):
                continue
            raw_ball_rows += 1
            orientation_status[row.get("orientation_status", "")] += 1
            if row.get("pose_accepted", "1") != "1":
                finish_run()
                previous = None
                continue
            accepted_ball_rows += 1
            timestamp_ns = finite_float(row, "ros_stamp_ns")
            position = tuple(
                finite_float(row, key) for key in ("normalized_x", "normalized_y", "normalized_z")
            )
            quaternion_values = tuple(
                finite_float(row, key) for key in ("qx", "qy", "qz", "qw")
            )
            if timestamp_ns is None or any(value is None for value in position):
                finish_run()
                previous = None
                continue
            if any(value is None for value in quaternion_values):
                finish_run()
                previous = None
                continue
            quaternion, quaternion_norm = normalize_xyzw(quaternion_values)
            if quaternion_norm is not None and math.isfinite(quaternion_norm):
                quaternion_norms.append(quaternion_norm)
            if quaternion is None:
                finish_run()
                previous = None
                continue
            valid_orientation_rows += 1
            if (
                previous_raw_quaternion is not None
                and dot(quaternion, previous_raw_quaternion) < 0.0
            ):
                raw_sign_transitions += 1
            previous_raw_quaternion = quaternion
            sample = (timestamp_ns * 1.0e-9, position, quaternion)
            if previous is None:
                previous = sample
                continue
            dt = sample[0] - previous[0]
            if dt <= 0.0:
                nonmonotonic_timestamps += 1
                continue
            maximum_source_gap_s = max(maximum_source_gap_s, dt)
            if dt > max_gap_s:
                source_gaps += 1
                finish_run()
                previous = sample
                continue
            current_quaternion = sample[2]
            if dot(current_quaternion, previous[2]) < 0.0:
                continuity_sign_negations += 1
                current_quaternion = tuple(-value for value in current_quaternion)
                sample = (sample[0], sample[1], current_quaternion)
            rotation_vector = relative_rotation_vector(current_quaternion, previous[2])
            rate_rev_s = math.sqrt(sum(value * value for value in rotation_vector)) / (dt * TWO_PI)
            retained = math.isfinite(rate_rev_s) and rate_rev_s <= max_rev_s
            if retained:
                retained_rates_rev_s.append(rate_rev_s)
            else:
                impossible_increments += 1

            vx = (sample[1][0] - previous[1][0]) / dt
            incoming_air = vx < -0.2 and min(sample[1][2], previous[1][2]) > 0.04
            if incoming_air:
                if run_start is None:
                    run_start = previous[0]
                run_end = sample[0]
                if retained:
                    incoming_rates_rev_s.append(rate_rev_s)
                    run_retained += 1
            else:
                finish_run()
            # Adopt even a relock sample so a constant marker-frame offset
            # cancels on the next increment, exactly as in the C++ estimator.
            previous = sample
    finish_run()

    usable_denominator = len(incoming_run_durations)
    matched_shot_audit = None
    if shot_predictions is not None:
        rows = []
        with shot_predictions.open(newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                value = finite_float(row, "prefix_skip_s")
                if value is not None and abs(value - prefix_skip_s) <= 1.0e-9:
                    rows.append(row)
        spin_valid = sum(int(float(row.get("spin_valid", "0"))) == 1 for row in rows)
        prediction_available = sum(
            int(float(row.get("prediction_available", "0"))) == 1 for row in rows
        )
        matched_shot_audit = {
            "input": str(shot_predictions.resolve()),
            "prefix_skip_s": prefix_skip_s,
            "matched_shots": len(rows),
            "prediction_available_shots": prediction_available,
            "spin_valid_shots": spin_valid,
            "spin_valid_fraction": spin_valid / len(rows) if rows else None,
        }

    return {
        "audit_only": True,
        "input": str(path.resolve()),
        "definitions": {
            "possible_marker_relock": f"one corrected quaternion increment > {max_rev_s:g} rev/s",
            "incoming_air_run": "contiguous vx<-0.2 m/s and both centers z>0.04 m",
            "usable_incoming_run": f"incoming-air duration >= {min_span_s:g} s and >=3 retained increments",
            "runtime_gate": False,
        },
        "raw_ball_rows": raw_ball_rows,
        "accepted_ball_rows": accepted_ball_rows,
        "valid_orientation_rows": valid_orientation_rows,
        "valid_orientation_fraction_of_accepted": (
            valid_orientation_rows / accepted_ball_rows if accepted_ball_rows else None
        ),
        "orientation_status": dict(orientation_status),
        "quaternion_norm": {
            "minimum": min(quaternion_norms) if quaternion_norms else None,
            "median": quantile(quaternion_norms, 0.5),
            "p95": quantile(quaternion_norms, 0.95),
            "maximum": max(quaternion_norms) if quaternion_norms else None,
        },
        "raw_quaternion_sign_transitions": raw_sign_transitions,
        "samples_sign_negated_for_continuity": continuity_sign_negations,
        "nonmonotonic_timestamps": nonmonotonic_timestamps,
        "source_gaps_over_limit": source_gaps,
        "maximum_source_gap_s": maximum_source_gap_s,
        "possible_marker_relock_increments": impossible_increments,
        "retained_rate_rev_s": {
            "median": quantile(retained_rates_rev_s, 0.5),
            "p95": quantile(retained_rates_rev_s, 0.95),
        },
        "incoming_air_rate_rev_s": {
            "median": quantile(incoming_rates_rev_s, 0.5),
            "p95": quantile(incoming_rates_rev_s, 0.95),
        },
        "incoming_air_runs": usable_denominator,
        "usable_incoming_air_runs": usable_incoming_runs,
        "usable_incoming_air_fraction": (
            usable_incoming_runs / usable_denominator if usable_denominator else None
        ),
        "incoming_air_run_duration_s": {
            "median": quantile(incoming_run_durations, 0.5),
            "p95": quantile(incoming_run_durations, 0.95),
        },
        "matched_causal_shots": matched_shot_audit,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--max-gap-s", type=float, default=0.05)
    parser.add_argument("--max-rev-s", type=float, default=20.0)
    parser.add_argument("--min-span-s", type=float, default=0.05)
    parser.add_argument("--shot-predictions", type=Path)
    parser.add_argument("--prefix-skip-s", type=float, default=0.15)
    args = parser.parse_args()
    report = analyze(
        args.input,
        args.max_gap_s,
        args.max_rev_s,
        args.min_span_s,
        args.shot_predictions,
        args.prefix_skip_s,
    )
    rendered = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
