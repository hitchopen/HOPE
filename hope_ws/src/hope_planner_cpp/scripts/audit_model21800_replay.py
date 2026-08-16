#!/usr/bin/env python3
"""Score the C++ no-EKF planner against exported model_21800 field logs.

This tool is deliberately offline. Measured crossings are used only to group
and score causal planner revisions; no threshold or result is connected to the
runtime command path.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import subprocess
import tempfile
from collections import Counter
from pathlib import Path
from typing import Iterable


def number(row: dict[str, str], key: str, default: float = math.nan) -> float:
    try:
        value = float(row.get(key, ""))
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def integer(row: dict[str, str], key: str, default: int = 0) -> int:
    try:
        return int(float(row.get(key, "")))
    except (TypeError, ValueError):
        return default


def read_csv(path: Path) -> list[dict[str, str]]:
    # A historical field session can contain a sparse NUL-filled extent when a
    # second process truncates a CSV while the first writer keeps its old file
    # offset.  Such a line is not evidence and can exceed csv's field limit;
    # discard only lines that are unambiguously binary-corrupt so the remaining
    # shots stay available for offline audit.  The runtime logger now uses
    # exclusive file creation so a later attempt cannot truncate this file.
    def intact_lines(stream):
        for line in stream:
            if "\0" not in line:
                yield line

    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(intact_lines(stream)))


def quantile(values: Iterable[float], fraction: float) -> float:
    finite = sorted(value for value in values if math.isfinite(value))
    if not finite:
        return math.nan
    position = (len(finite) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return finite[lower]
    return (
        finite[lower] * (upper - position)
        + finite[upper] * (position - lower)
    )


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


def runner_trace_paths(session: Path) -> list[tuple[str, Path]]:
    attempts = sorted((session / "mdu" / "real").glob("attempt_*/runner_trace.csv"))
    if attempts:
        return [(path.parent.name, path) for path in attempts]
    path = session / "mdu" / "real" / "runner_trace.csv"
    return [("real", path)] if path.is_file() else []


def locked_shots(attempt: str, rows: list[dict[str, str]]) -> list[dict]:
    output: list[dict] = []
    seen: set[int] = set()
    for row in rows:
        shot_sequence = integer(row, "shot_seq")
        if shot_sequence <= 0 or shot_sequence in seen:
            continue
        seen.add(shot_sequence)
        output.append(
            {
                "attempt": attempt,
                "shot_seq": shot_sequence,
                "shot_key": f"{attempt}:{shot_sequence}",
                "engage_wall_ns": integer(row, "wall_time_ns"),
                "target_x": number(row, "target_x"),
                "target_y": number(row, "target_y"),
                "target_z": number(row, "target_z"),
                "frozen_command_seq": integer(row, "frozen_command_seq"),
                "frozen_flight_id": integer(row, "frozen_flight_id"),
                "frozen_revision_id": integer(row, "frozen_revision_id"),
            }
        )
    return output


def match_planner_rows(
    shots: list[dict], planner_rows: list[dict[str, str]], tolerance_m: float
) -> None:
    valid = [
        row
        for row in planner_rows
        if integer(row, "wire_valid", integer(row, "command_valid")) == 1
    ]
    by_command = {
        integer(row, "command_seq"): row
        for row in valid
        if integer(row, "command_seq") > 0
    }
    for shot in shots:
        frozen_command = shot.get("frozen_command_seq", 0)
        if frozen_command in by_command:
            row = by_command[frozen_command]
            shot.update(
                {
                    "planner_matched": True,
                    "target_match_error_m": 0.0,
                    "locked_strike_time_s": number(
                        row,
                        "strike_deadline_wall_s",
                        number(row, "strike_time_s"),
                    ),
                    "locked_swing_sign": number(row, "swing_sign", -1.0),
                    "x_hit": number(
                        row, "strike_x", number(row, "x_hit_active")
                    ),
                }
            )
            continue
        matches = []
        for row in valid:
            differences = (
                number(row, "published_policy_x", number(row, "strike_x"))
                - shot["target_x"],
                number(row, "published_policy_y", number(row, "strike_y"))
                - shot["target_y"],
                number(row, "published_policy_z", number(row, "strike_z"))
                - shot["target_z"],
            )
            if not all(math.isfinite(value) for value in differences):
                continue
            distance = math.sqrt(sum(value * value for value in differences))
            if distance <= tolerance_m:
                wall_delta = abs(
                    integer(
                        row,
                        "receipt_wall_ns",
                        int(number(row, "producer_wall_s", 0.0) * 1.0e9),
                    )
                    - shot["engage_wall_ns"]
                )
                matches.append((wall_delta, distance, row))
        if not matches:
            continue
        _, distance, row = min(matches)
        shot.update(
            {
                "planner_matched": True,
                "target_match_error_m": distance,
                "locked_strike_time_s": number(
                    row,
                    "strike_deadline_wall_s",
                    number(row, "strike_time_s"),
                ),
                "locked_swing_sign": number(row, "swing_sign", -1.0),
                "x_hit": number(
                    row, "strike_x", number(row, "x_hit_active")
                ),
            }
        )


def raw_crossings_by_plane(
    mocap_path: Path, planes: list[float], maximum_gap_s: float
) -> dict[float, list[dict]]:
    output = {plane: [] for plane in planes}
    previous = None
    with mocap_path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            if (
                row.get("object_key", "").lower() != "ball"
                and row.get("object_name", "").lower() != "ball"
            ):
                continue
            if row.get("pose_accepted", "1") != "1":
                continue
            sample = (
                integer(row, "ros_stamp_ns") * 1.0e-9,
                number(row, "normalized_x", number(row, "raw_x")),
                number(row, "normalized_y", number(row, "raw_y")),
                number(row, "normalized_z", number(row, "raw_z")),
            )
            if not all(math.isfinite(value) for value in sample):
                continue
            if previous is not None:
                gap_s = sample[0] - previous[0]
                if 0.0 < gap_s <= maximum_gap_s:
                    for plane in planes:
                        if previous[1] > plane >= sample[1]:
                            fraction = (previous[1] - plane) / (
                                previous[1] - sample[1]
                            )
                            output[plane].append(
                                {
                                    "time_s": previous[0] + fraction * gap_s,
                                    "y": previous[2]
                                    + fraction * (sample[2] - previous[2]),
                                    "z": previous[3]
                                    + fraction * (sample[3] - previous[3]),
                                    "source_gap_s": gap_s,
                                }
                            )
            previous = sample
    return output


def associate_crossings(
    shots: list[dict], crossings: dict[float, list[dict]], match_window_s: float
) -> None:
    for plane in sorted(crossings):
        plane_shots = [
            shot
            for shot in shots
            if shot.get("planner_matched")
            and round(shot["x_hit"], 6) == plane
        ]
        used: set[int] = set()
        for shot in plane_shots:
            candidates = [
                (
                    abs(item["time_s"] - shot["locked_strike_time_s"]),
                    index,
                    item,
                )
                for index, item in enumerate(crossings[plane])
                if index not in used
                and abs(item["time_s"] - shot["locked_strike_time_s"])
                <= match_window_s
            ]
            if not candidates:
                continue
            _, index, crossing = min(candidates)
            used.add(index)
            shot["actual_crossing"] = crossing


def run_replay(
    executable: Path,
    mocap_path: Path,
    plane: float,
    args,
) -> tuple[list[dict], dict, list[dict]]:
    with tempfile.NamedTemporaryFile(suffix=".csv") as temporary:
        command = [
            str(executable),
            "--input",
            str(mocap_path),
            "--output",
            temporary.name,
            "--x-hit",
            str(plane),
            "--solve-period",
            str(args.solve_period_s),
            "--window",
            str(args.estimator_window_s),
            "--min-span",
            str(args.estimator_min_span_s),
            "--min-samples",
            str(args.estimator_min_samples),
            "--huber-delta",
            str(args.huber_delta_m),
            "--recency-half-life",
            str(args.recency_half_life_s),
            "--iterations",
            str(args.estimator_iterations),
            "--drag-k",
            str(args.drag_k),
            "--restitution-h",
            str(args.restitution_h),
            "--restitution-v",
            str(args.restitution_v),
            "--spin-mode",
            args.spin_mode,
            "--spin-window",
            str(args.spin_window_s),
            "--spin-min-span",
            str(args.spin_min_span_s),
            "--spin-max-gap",
            str(args.spin_max_gap_s),
            "--spin-max-rev",
            str(args.spin_max_rev_s),
            "--spin-huber-delta-rev",
            str(args.spin_huber_delta_rev_s),
            "--magnus-k",
            str(args.magnus_k),
            "--nakashima-friction-mu",
            str(args.nakashima_friction_mu),
            "--table-tangential-gain",
            str(args.table_tangential_gain),
            "--table-friction-cap-mu",
            str(args.table_friction_cap_mu),
        ]
        if args.control_zero_spin:
            command.append("--control-zero-spin")
        if args.post_net_one_shot:
            command.extend(
                (
                    "--post-net-one-shot",
                    "--post-net-delay",
                    str(args.post_net_delay_s),
                    "--post-net-future-bounce-tangential-gain",
                    str(args.post_net_future_bounce_tangential_gain),
                    "--net-x",
                    str(args.net_x),
                    "--incoming-opponent-side-margin",
                    str(args.incoming_opponent_side_margin_m),
                    "--incoming-speed-threshold",
                    str(args.incoming_speed_threshold_mps),
                    "--outgoing-speed-threshold",
                    str(args.outgoing_speed_threshold_mps),
                    "--incoming-direction-fit-samples",
                    str(args.incoming_direction_fit_samples),
                    "--incoming-direction-confirmations",
                    str(args.incoming_direction_confirmations),
                    "--incoming-pre-roll-samples",
                    str(args.incoming_pre_roll_samples),
                    "--incoming-source-gap-reset",
                    str(args.incoming_source_gap_reset_s),
                )
            )
        if not args.replay_without_bounce_options:
            command.extend(
                (
                    "--bounce-min-reversal",
                    str(args.bounce_min_reversal_m),
                    "--bounce-min-excursion",
                    str(args.bounce_min_excursion_m),
                    "--bounce-confirmation-samples",
                    str(args.bounce_confirmation_samples),
                    "--bounce-confirmation-max-span",
                    str(args.bounce_confirmation_max_span_s),
                    "--bounce-sparse-confirmation-min-span",
                    str(args.bounce_sparse_confirmation_min_span_s),
                    "--bounce-sparse-confirmation-excursion",
                    str(args.bounce_sparse_confirmation_excursion_m),
                    "--bounce-refractory",
                    str(args.bounce_refractory_s),
                )
            )
        if args.adaptive_horizon:
            command.append("--adaptive-horizon")
        completed = subprocess.run(
            command, check=True, capture_output=True, text=True
        )
        metadata = json.loads(completed.stdout)
        valid_revisions = []
        solve_events = []
        with Path(temporary.name).open(newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                if row.get("kind") != "solve":
                    continue
                solve_events.append(
                    {
                        "sample_time_s": number(row, "source_time_s"),
                        "valid": integer(row, "valid"),
                        "one_shot_flight_seq": integer(
                            row, "one_shot_flight_seq"
                        ),
                        "net_cross_source_time_s": number(
                            row, "net_cross_source_time_s"
                        ),
                        "commit_source_time_s": number(
                            row, "commit_source_time_s"
                        ),
                    }
                )
                if integer(row, "valid") != 1:
                    continue
                valid_revisions.append(
                    {
                        "sample_time_s": number(row, "source_time_s"),
                        "strike_time_s": number(row, "strike_time_s"),
                        "y": number(row, "strike_y"),
                        "z": number(row, "strike_z"),
                        "swing_sign": number(row, "swing_sign"),
                        "estimator_ms": number(row, "estimator_ms"),
                        "stage2_ms": number(row, "stage2_ms"),
                        "stage3_ms": number(row, "stage3_ms"),
                        "total_ms": number(row, "total_ms"),
                        "spin_valid": integer(row, "spin_valid"),
                        "spin_wx_rad_s": number(row, "spin_wx_rad_s"),
                        "spin_wy_rad_s": number(row, "spin_wy_rad_s"),
                        "spin_wz_rad_s": number(row, "spin_wz_rad_s"),
                        "spin_magnitude_rev_s": number(
                            row, "spin_magnitude_rev_s"
                        ),
                        "spin_coherence": number(row, "spin_coherence"),
                        "spin_retained_time_fraction": number(
                            row, "spin_retained_time_fraction"
                        ),
                        "spin_rejected_increments": integer(
                            row, "spin_rejected_increments"
                        ),
                        "one_shot_flight_seq": integer(
                            row, "one_shot_flight_seq"
                        ),
                        "net_cross_source_time_s": number(
                            row, "net_cross_source_time_s"
                        ),
                        "commit_source_time_s": number(
                            row, "commit_source_time_s"
                        ),
                    }
                )
        return valid_revisions, metadata, solve_events


def commit_revision(
    revisions: list[dict], prefix_skip_s: float, locked_swing_sign: float
) -> dict | None:
    ordered = sorted(revisions, key=lambda item: item["sample_time_s"])
    for index, revision in enumerate(ordered):
        # Replay has no synchronized base/P1 stream, so its side estimate is
        # not authoritative. The historical runner lock is matched to the
        # exact Planner target and supplies the side actually used by the
        # model_21800 clip clock for this measured shot.
        windup_s = 0.82 if locked_swing_sign > 0.0 else 0.96
        skip_s = min(max(prefix_skip_s, 0.0), 0.45 * windup_s)
        commit_tts_s = windup_s - skip_s
        hard_late_tts_s = 0.55 * windup_s
        raw_tts_s = revision["strike_time_s"] - revision["sample_time_s"]
        if 0.0 < raw_tts_s <= commit_tts_s:
            result = dict(revision)
            result["commit_event_time_s"] = revision["sample_time_s"]
            result["raw_tts_s"] = raw_tts_s
            result["late_phase_clamped"] = raw_tts_s < hard_late_tts_s
            return result
        if raw_tts_s > commit_tts_s:
            event_time_s = revision["strike_time_s"] - commit_tts_s
            next_arrival_s = (
                ordered[index + 1]["sample_time_s"]
                if index + 1 < len(ordered)
                else math.inf
            )
            if event_time_s < next_arrival_s:
                result = dict(revision)
                result["commit_event_time_s"] = event_time_s
                result["raw_tts_s"] = commit_tts_s
                result["late_phase_clamped"] = False
                return result
    return None


def error_metrics(prediction: dict | None, crossing: dict) -> tuple[float, float]:
    if prediction is None:
        return math.nan, math.nan
    return (
        math.hypot(prediction["y"] - crossing["y"], prediction["z"] - crossing["z"]),
        abs(prediction["strike_time_s"] - crossing["time_s"]),
    )


def summarize_predictions(predictions: list[tuple[dict | None, dict]]) -> dict:
    position_errors = []
    timing_errors = []
    coverage = 0
    late = 0
    for prediction, crossing in predictions:
        position_error, timing_error = error_metrics(prediction, crossing)
        if prediction is not None:
            coverage += 1
            late += int(prediction.get("late_phase_clamped", False))
        position_errors.append(position_error)
        timing_errors.append(timing_error)
    return {
        "coverage": coverage,
        "late_phase_clamped": late,
        "position_yz_median_m": quantile(position_errors, 0.50),
        "position_yz_p95_m": quantile(position_errors, 0.95),
        "timing_median_s": quantile(timing_errors, 0.50),
        "timing_p95_s": quantile(timing_errors, 0.95),
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--replay-executable", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--estimator-window-s", type=float, default=0.18)
    parser.add_argument("--estimator-min-span-s", type=float, default=0.08)
    parser.add_argument("--estimator-min-samples", type=int, default=12)
    parser.add_argument("--huber-delta-m", type=float, default=0.003)
    parser.add_argument("--recency-half-life-s", type=float, default=0.0)
    parser.add_argument("--estimator-iterations", type=int, default=3)
    parser.add_argument("--solve-period-s", type=float, default=0.033)
    parser.add_argument("--post-net-one-shot", action="store_true")
    parser.add_argument("--post-net-delay-s", type=float, default=0.05)
    parser.add_argument(
        "--post-net-future-bounce-tangential-gain", type=float, default=0.369
    )
    parser.add_argument("--net-x", type=float, default=1.37)
    parser.add_argument(
        "--incoming-opponent-side-margin-m", type=float, default=0.05
    )
    parser.add_argument(
        "--incoming-speed-threshold-mps", type=float, default=0.25
    )
    parser.add_argument(
        "--outgoing-speed-threshold-mps", type=float, default=0.25
    )
    parser.add_argument("--incoming-direction-fit-samples", type=int, default=4)
    parser.add_argument("--incoming-direction-confirmations", type=int, default=2)
    parser.add_argument("--incoming-pre-roll-samples", type=int, default=24)
    parser.add_argument(
        "--incoming-source-gap-reset-s", type=float, default=0.25
    )
    parser.add_argument("--adaptive-horizon", action="store_true")
    parser.add_argument("--drag-k", type=float, default=0.1261)
    parser.add_argument("--restitution-h", type=float, default=0.64)
    parser.add_argument("--restitution-v", type=float, default=0.9215)
    parser.add_argument("--bounce-min-reversal-m", type=float, default=0.00005)
    parser.add_argument("--bounce-min-excursion-m", type=float, default=0.001)
    parser.add_argument("--bounce-confirmation-samples", type=int, default=5)
    parser.add_argument(
        "--bounce-confirmation-max-span-s", type=float, default=0.05
    )
    parser.add_argument(
        "--bounce-sparse-confirmation-min-span-s", type=float, default=0.012
    )
    parser.add_argument(
        "--bounce-sparse-confirmation-excursion-m", type=float, default=0.005
    )
    parser.add_argument("--bounce-refractory-s", type=float, default=0.12)
    parser.add_argument(
        "--replay-without-bounce-options",
        action="store_true",
        help="compatibility mode for the historical reset baseline binary",
    )
    parser.add_argument(
        "--spin-mode",
        choices=(
            "legacy",
            "nakashima",
            "nakashima-magnus",
            "venue-grip",
            "venue-grip-magnus",
        ),
        default="legacy",
    )
    parser.add_argument(
        "--control-zero-spin",
        action="store_true",
        help=(
            "force omega=0 in the control replay while retaining the selected "
            "table-contact law; this matches the model_21800 hardware node"
        ),
    )
    parser.add_argument("--spin-window-s", type=float, default=0.10)
    parser.add_argument("--spin-min-span-s", type=float, default=0.05)
    parser.add_argument("--spin-max-gap-s", type=float, default=0.05)
    parser.add_argument("--spin-max-rev-s", type=float, default=20.0)
    parser.add_argument("--spin-huber-delta-rev-s", type=float, default=2.0)
    parser.add_argument("--magnus-k", type=float, default=0.00444)
    parser.add_argument(
        "--table-friction-mu",
        "--nakashima-friction-mu",
        dest="nakashima_friction_mu",
        type=float,
        default=0.25,
    )
    parser.add_argument("--table-tangential-gain", type=float, default=0.369)
    parser.add_argument("--table-friction-cap-mu", type=float, default=2.0)
    parser.add_argument(
        "--prefix-skip-s", type=float, nargs="+", default=[0.0, 0.05, 0.10, 0.15]
    )
    parser.add_argument(
        "--selected-prefix-skip-s",
        type=float,
        default=0.15,
        help=(
            "offline profile scored by the acceptance summary; pass 0.45 for "
            "the model_21800 fixed dynamic-boundary schedule"
        ),
    )
    parser.add_argument("--crossing-match-window-s", type=float, default=1.5)
    parser.add_argument("--revision-lead-window-s", type=float, default=3.0)
    parser.add_argument("--mocap-crossing-max-gap-s", type=float, default=0.03)
    parser.add_argument("--target-match-tolerance-m", type=float, default=0.001)
    args = parser.parse_args(argv)

    session = args.session.resolve()
    planner_path = session / "hdu" / "planner.csv"
    if not planner_path.is_file():
        attempts = sorted((session / "hdu").glob("planner_attempt_*/planner.csv"))
        if attempts:
            planner_path = attempts[-1]
    mocap_path = session / "laptop" / "mocap_raw.csv"
    traces = runner_trace_paths(session)
    if not planner_path.is_file() or not mocap_path.is_file() or not traces:
        raise SystemExit("session is missing planner.csv, mocap_raw.csv, or runner trace")

    planner_rows = read_csv(planner_path)
    shots = []
    for attempt, path in traces:
        shots.extend(locked_shots(attempt, read_csv(path)))
    match_planner_rows(shots, planner_rows, args.target_match_tolerance_m)
    planes = sorted(
        {
            round(shot["x_hit"], 6)
            for shot in shots
            if shot.get("planner_matched") and math.isfinite(shot["x_hit"])
        }
    )
    crossings = raw_crossings_by_plane(
        mocap_path, planes, args.mocap_crossing_max_gap_s
    )
    associate_crossings(shots, crossings, args.crossing_match_window_s)
    scored_shots = [shot for shot in shots if "actual_crossing" in shot]

    replay_by_plane = {}
    replay_events_by_plane = {}
    replay_metadata = {}
    for plane in planes:
        revisions, metadata, solve_events = run_replay(
            args.replay_executable.resolve(), mocap_path, plane, args
        )
        replay_by_plane[plane] = revisions
        replay_events_by_plane[plane] = solve_events
        replay_metadata[str(plane)] = metadata

    revisions_by_shot = {}
    for shot in scored_shots:
        crossing = shot["actual_crossing"]
        candidates = [
            revision
            for revision in replay_by_plane[round(shot["x_hit"], 6)]
            if crossing["time_s"] - args.revision_lead_window_s
            <= revision["sample_time_s"]
            <= crossing["time_s"]
        ]
        if args.post_net_one_shot:
            # A one-shot replay has one immutable revision per incoming net
            # crossing. Associate the measured strike with the most recent
            # causal net crossing, not with an older flight that happens to
            # lie inside the legacy continuous-revision lead window.
            plane = round(shot["x_hit"], 6)
            causal_events = [
                event
                for event in replay_events_by_plane[plane]
                if crossing["time_s"] - args.revision_lead_window_s
                <= event["sample_time_s"]
                <= crossing["time_s"]
                and math.isfinite(event["net_cross_source_time_s"])
                and event["net_cross_source_time_s"] <= crossing["time_s"]
            ]
            if causal_events:
                latest_net_time = max(
                    event["net_cross_source_time_s"] for event in causal_events
                )
                candidates = [
                    revision
                    for revision in candidates
                    if revision["net_cross_source_time_s"] == latest_net_time
                ]
            else:
                candidates = []
        revisions_by_shot[shot["shot_key"]] = candidates

    prefix_results = {}
    shot_rows = []
    for prefix_skip_s in args.prefix_skip_s:
        predictions = []
        for shot in scored_shots:
            crossing = shot["actual_crossing"]
            prediction = commit_revision(
                revisions_by_shot[shot["shot_key"]],
                prefix_skip_s,
                shot["locked_swing_sign"],
            )
            predictions.append((prediction, crossing))
            position_error, timing_error = error_metrics(prediction, crossing)
            shot_rows.append(
                {
                    "attempt": shot["attempt"],
                    "shot_seq": shot["shot_seq"],
                    "x_hit": shot["x_hit"],
                    "locked_swing_sign": shot["locked_swing_sign"],
                    "prefix_skip_s": prefix_skip_s,
                    "actual_crossing_time_s": crossing["time_s"],
                    "actual_y": crossing["y"],
                    "actual_z": crossing["z"],
                    "prediction_available": int(prediction is not None),
                    "predicted_strike_time_s": (
                        prediction["strike_time_s"] if prediction else math.nan
                    ),
                    "prediction_sample_time_s": (
                        prediction["sample_time_s"] if prediction else math.nan
                    ),
                    "commit_event_time_s": (
                        prediction["commit_event_time_s"] if prediction else math.nan
                    ),
                    "predicted_y": prediction["y"] if prediction else math.nan,
                    "predicted_z": prediction["z"] if prediction else math.nan,
                    "position_yz_error_m": position_error,
                    "timing_error_s": timing_error,
                    "late_phase_clamped": int(
                        prediction.get("late_phase_clamped", False)
                        if prediction
                        else False
                    ),
                    "raw_tts_s": prediction["raw_tts_s"] if prediction else math.nan,
                    "spin_valid": prediction["spin_valid"] if prediction else 0,
                    "spin_wx_rad_s": (
                        prediction["spin_wx_rad_s"] if prediction else math.nan
                    ),
                    "spin_wy_rad_s": (
                        prediction["spin_wy_rad_s"] if prediction else math.nan
                    ),
                    "spin_wz_rad_s": (
                        prediction["spin_wz_rad_s"] if prediction else math.nan
                    ),
                    "spin_magnitude_rev_s": (
                        prediction["spin_magnitude_rev_s"]
                        if prediction
                        else math.nan
                    ),
                    "spin_coherence": (
                        prediction["spin_coherence"] if prediction else math.nan
                    ),
                    "spin_retained_time_fraction": (
                        prediction["spin_retained_time_fraction"]
                        if prediction
                        else math.nan
                    ),
                    "spin_rejected_increments": (
                        prediction["spin_rejected_increments"]
                        if prediction
                        else 0
                    ),
                }
            )
        prefix_results[str(prefix_skip_s)] = summarize_predictions(predictions)

    latest_predictions = []
    consecutive_position_deltas = []
    consecutive_deadline_deltas = []
    for shot in scored_shots:
        revisions = revisions_by_shot[shot["shot_key"]]
        latest_predictions.append(
            (revisions[-1] if revisions else None, shot["actual_crossing"])
        )
        for previous, current in zip(revisions, revisions[1:]):
            consecutive_position_deltas.append(
                math.hypot(current["y"] - previous["y"], current["z"] - previous["z"])
            )
            consecutive_deadline_deltas.append(
                abs(current["strike_time_s"] - previous["strike_time_s"])
            )

    selected_key = str(args.selected_prefix_skip_s)
    selected = prefix_results.get(selected_key, {})
    if not selected:
        raise SystemExit(
            "--selected-prefix-skip-s must also appear in --prefix-skip-s"
        )
    latest_summary = summarize_predictions(latest_predictions)
    revision_position_p95 = quantile(consecutive_position_deltas, 0.95)
    revision_deadline_p95 = quantile(consecutive_deadline_deltas, 0.95)
    position_latest_ratio = (
        selected.get("position_yz_median_m", math.inf)
        / max(latest_summary.get("position_yz_median_m", 0.0), 1.0e-12)
    )
    timing_latest_ratio = (
        selected.get("timing_median_s", math.inf)
        / max(latest_summary.get("timing_median_s", 0.0), 1.0e-12)
    )
    # These are offline model-selection checks only. They are deliberately not
    # exported to the ROS node or runner and cannot suppress a command.
    acceptance = {
        "coverage_28_of_29": (
            len(scored_shots) != 29 or selected.get("coverage", 0) >= 28
        ),
        "position_median_le_0_10_m": selected.get("position_yz_median_m", math.inf)
        <= 0.10,
        "position_p95_le_0_25_m": selected.get("position_yz_p95_m", math.inf)
        <= 0.25,
        "timing_median_le_0_040_s": selected.get("timing_median_s", math.inf)
        <= 0.040,
        "timing_p95_le_0_100_s": selected.get("timing_p95_s", math.inf)
        <= 0.100,
        "no_late_phase_clamp": selected.get("late_phase_clamped", math.inf) == 0,
        "revision_position_p95_lt_0_15_m": revision_position_p95 < 0.15,
        "revision_deadline_p95_lt_0_100_s": revision_deadline_p95 < 0.100,
        "latest_position_not_over_2x_better": position_latest_ratio < 2.0,
        "latest_timing_not_over_2x_better": timing_latest_ratio < 2.0,
    }
    acceptance["all"] = all(acceptance.values())

    reasons = Counter(row.get("reason", "") for row in planner_rows)
    summary = {
        "session": session.name,
        "audit_only": True,
        "planner_kind": (
            f"batch_physics_cpp_no_ekf_{args.spin_mode}"
            + ("_control_zero_spin" if args.control_zero_spin else "")
        ),
        "estimator": {
            "window_s": args.estimator_window_s,
            "min_span_s": args.estimator_min_span_s,
            "min_samples": args.estimator_min_samples,
            "huber_delta_m": args.huber_delta_m,
            "recency_half_life_s": args.recency_half_life_s,
            "iterations": args.estimator_iterations,
            "bounce_min_reversal_m": args.bounce_min_reversal_m,
            "bounce_min_excursion_m": args.bounce_min_excursion_m,
            "bounce_confirmation_samples": args.bounce_confirmation_samples,
            "bounce_confirmation_max_span_s": args.bounce_confirmation_max_span_s,
            "bounce_sparse_confirmation_min_span_s": (
                args.bounce_sparse_confirmation_min_span_s
            ),
            "bounce_sparse_confirmation_excursion_m": (
                args.bounce_sparse_confirmation_excursion_m
            ),
            "bounce_refractory_s": args.bounce_refractory_s,
        },
        "adaptive_horizon": args.adaptive_horizon,
        "post_net_one_shot": args.post_net_one_shot,
        "post_net_delay_s": args.post_net_delay_s,
        "post_net_future_bounce_tangential_gain": (
            args.post_net_future_bounce_tangential_gain
        ),
        "net_x": args.net_x,
        "incoming_trajectory": {
            "opponent_side_margin_m": args.incoming_opponent_side_margin_m,
            "incoming_speed_threshold_mps": args.incoming_speed_threshold_mps,
            "outgoing_speed_threshold_mps": args.outgoing_speed_threshold_mps,
            "direction_fit_samples": args.incoming_direction_fit_samples,
            "direction_confirmations": args.incoming_direction_confirmations,
            "pre_roll_samples": args.incoming_pre_roll_samples,
            "source_gap_reset_s": args.incoming_source_gap_reset_s,
        },
        "replay_without_bounce_options": args.replay_without_bounce_options,
        "physics": {
            "drag_k": args.drag_k,
            "restitution_h": args.restitution_h,
            "restitution_v": args.restitution_v,
            "spin_mode": args.spin_mode,
            "control_zero_spin": args.control_zero_spin,
            "spin_window_s": args.spin_window_s,
            "spin_min_span_s": args.spin_min_span_s,
            "spin_max_gap_s": args.spin_max_gap_s,
            "spin_max_rev_s": args.spin_max_rev_s,
            "spin_huber_delta_rev_s": args.spin_huber_delta_rev_s,
            "magnus_k": args.magnus_k,
            "nakashima_friction_mu": args.nakashima_friction_mu,
            "table_tangential_gain": args.table_tangential_gain,
            "post_net_future_bounce_tangential_gain": (
                args.post_net_future_bounce_tangential_gain
            ),
            "table_friction_cap_mu": args.table_friction_cap_mu,
        },
        "runner_shots": len(shots),
        "planner_matched_shots": sum(shot.get("planner_matched", False) for shot in shots),
        "measured_crossing_shots": len(scored_shots),
        "planes": planes,
        "prefix_results": prefix_results,
        "selected_prefix_skip_s": args.selected_prefix_skip_s,
        "latest_revision": latest_summary,
        "frozen_to_latest_median_ratio": {
            "position_yz": position_latest_ratio,
            "timing": timing_latest_ratio,
        },
        "revision_deltas": {
            "position_yz_p95_m": revision_position_p95,
            "deadline_p95_s": revision_deadline_p95,
        },
        "selected_prefix_acceptance": acceptance,
        "replay_metadata": replay_metadata,
        "historical_planner_reason_rows": dict(reasons),
        "crossing_grouping": {
            "match_window_s": args.crossing_match_window_s,
            "revision_lead_window_s": args.revision_lead_window_s,
            "measured_truth_used_only_offline": True,
        },
    }

    args.output_dir.mkdir(parents=True, exist_ok=True)
    summary_path = args.output_dir / "summary.json"
    summary_path.write_text(
        json.dumps(json_safe(summary), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    shot_path = args.output_dir / "shot_predictions.csv"
    with shot_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(shot_rows[0]) if shot_rows else [])
        if shot_rows:
            writer.writeheader()
            writer.writerows(shot_rows)
    print(json.dumps(json_safe(summary), indent=2, sort_keys=True))
    return 0 if acceptance["all"] else 3


if __name__ == "__main__":
    raise SystemExit(main())
