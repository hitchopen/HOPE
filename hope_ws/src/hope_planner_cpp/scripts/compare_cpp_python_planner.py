#!/usr/bin/env python3
"""Compare C++ Stage 2/3 replay rows with the retained Python oracle.

This is an offline migration test. It never publishes ROS commands and does
not turn numerical tolerances into a runtime control gate.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np


def finite_vector(row: dict[str, str], names: tuple[str, ...]) -> np.ndarray | None:
    try:
        value = np.asarray([float(row[name]) for name in names], dtype=float)
    except (KeyError, TypeError, ValueError):
        return None
    return value if np.all(np.isfinite(value)) else None


def scalar(row: dict[str, str], name: str) -> float:
    try:
        value = float(row[name])
    except (KeyError, TypeError, ValueError):
        return math.nan
    return value if math.isfinite(value) else math.nan


def update_maximum(summary: dict, name: str, actual, expected) -> bool:
    difference = float(np.max(np.abs(np.asarray(actual) - np.asarray(expected))))
    summary[name] = max(summary.get(name, 0.0), difference)
    return math.isfinite(difference)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--replay-csv", type=Path, required=True)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--x-hit", type=float, required=True)
    parser.add_argument("--drag-k", type=float, default=0.1261)
    parser.add_argument("--restitution-h", type=float, default=0.64)
    parser.add_argument("--restitution-v", type=float, default=0.9215)
    parser.add_argument("--adaptive-horizon", action="store_true")
    parser.add_argument("--max-predict-time", type=float, default=2.0)
    parser.add_argument("--max-predict-time-cap", type=float, default=3.0)
    parser.add_argument("--position-tolerance", type=float, default=1.0e-8)
    parser.add_argument("--velocity-tolerance", type=float, default=1.0e-8)
    parser.add_argument("--time-tolerance", type=float, default=1.0e-9)
    parser.add_argument("--sample-stride", type=int, default=1)
    parser.add_argument("--max-rows", type=int, default=0)
    parser.add_argument(
        "--python-package-root",
        type=Path,
        default=Path(__file__).resolve().parents[2] / "hope_planner",
    )
    args = parser.parse_args(argv)

    sys.path.insert(0, str(args.python_package_root.resolve()))
    from hope_planner.ball_trajectory_predictor import BallTrajectoryPredictor
    from hope_planner.constants import BallPhysics, PlannerConfig, TableParams
    from hope_planner.racket_target_planner import RacketTargetPlanner

    physics = BallPhysics(
        k=args.drag_k,
        C_h=args.restitution_h,
        C_v=args.restitution_v,
    )
    config = PlannerConfig()
    config.x_hit = args.x_hit
    config.max_predict_time = args.max_predict_time
    config.adaptive_predict_horizon = args.adaptive_horizon
    config.max_predict_time_cap = args.max_predict_time_cap
    config.target_land = np.asarray([2.055, -0.7625, 0.0], dtype=float)
    config.delta_t_flight = 0.50
    table = TableParams()
    predictor = BallTrajectoryPredictor(physics, config, table)
    target_planner = RacketTargetPlanner(physics, config, table)

    summary = {
        "audit_only": True,
        "replay_csv": str(args.replay_csv.resolve()),
        "compared_rows": 0,
        "valid_rows": 0,
        "reason_mismatches": 0,
        "validity_mismatches": 0,
        "boolean_mismatches": 0,
        "nonfinite_differences": 0,
        "max_abs_strike_position": 0.0,
        "max_abs_strike_velocity": 0.0,
        "max_abs_strike_time_s": 0.0,
        "max_abs_racket_velocity": 0.0,
        "max_abs_racket_normal": 0.0,
        "max_abs_outgoing_velocity": 0.0,
    }

    with args.replay_csv.open(newline="", encoding="utf-8") as stream:
        for row_index, row in enumerate(csv.DictReader(stream)):
            if row.get("kind") != "solve" or row_index % max(1, args.sample_stride):
                continue
            if row.get("estimate_valid") != "1":
                continue
            position = finite_vector(row, ("est_x", "est_y", "est_z"))
            velocity = finite_vector(row, ("est_vx", "est_vy", "est_vz"))
            source_time = scalar(row, "source_time_s")
            if position is None or velocity is None or not math.isfinite(source_time):
                continue
            strike = predictor.predict(position, velocity, source_time)
            command = target_planner.plan(strike)
            cpp_valid = row.get("valid") == "1"
            python_valid = bool(command.valid)
            summary["compared_rows"] += 1
            summary["valid_rows"] += int(cpp_valid and python_valid)
            if cpp_valid != python_valid:
                summary["validity_mismatches"] += 1
                continue
            expected_reason = (
                "command_valid"
                if python_valid
                else ("not_incoming" if velocity[0] >= 0.0 else predictor.last_reason)
            )
            if row.get("reason") != expected_reason:
                summary["reason_mismatches"] += 1
            if not python_valid:
                continue

            comparisons = (
                ("max_abs_strike_position", ("strike_x", "strike_y", "strike_z"), strike.p_ball),
                ("max_abs_strike_velocity", ("strike_vx", "strike_vy", "strike_vz"), strike.v_ball),
                ("max_abs_racket_velocity", ("racket_vx", "racket_vy", "racket_vz"), command.v_racket),
                ("max_abs_racket_normal", ("racket_nx", "racket_ny", "racket_nz"), command.n_racket),
                ("max_abs_outgoing_velocity", ("outgoing_vx", "outgoing_vy", "outgoing_vz"), command.v_ball_outgoing),
            )
            for metric, columns, expected in comparisons:
                actual = finite_vector(row, columns)
                if actual is None or not update_maximum(summary, metric, actual, expected):
                    summary["nonfinite_differences"] += 1
            strike_time = scalar(row, "strike_time_s")
            if math.isfinite(strike_time):
                update_maximum(
                    summary, "max_abs_strike_time_s", strike_time, strike.t_strike
                )
            else:
                summary["nonfinite_differences"] += 1
            if (
                int(float(row["clears_net"])) != int(command.clears_net)
                or int(float(row["bypasses_net_posts"]))
                != int(command.bypasses_net_posts)
            ):
                summary["boolean_mismatches"] += 1
            if args.max_rows > 0 and summary["compared_rows"] >= args.max_rows:
                break

    tolerances = {
        "max_abs_strike_position": args.position_tolerance,
        "max_abs_strike_velocity": args.velocity_tolerance,
        "max_abs_strike_time_s": args.time_tolerance,
        "max_abs_racket_velocity": args.velocity_tolerance,
        "max_abs_racket_normal": args.velocity_tolerance,
        "max_abs_outgoing_velocity": args.velocity_tolerance,
    }
    summary["tolerances"] = tolerances
    summary["pass"] = (
        summary["compared_rows"] > 0
        and summary["reason_mismatches"] == 0
        and summary["validity_mismatches"] == 0
        and summary["boolean_mismatches"] == 0
        and summary["nonfinite_differences"] == 0
        and all(summary[name] <= tolerance for name, tolerance in tolerances.items())
    )
    rendered = json.dumps(summary, indent=2, sort_keys=True) + "\n"
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    return 0 if summary["pass"] else 3


if __name__ == "__main__":
    raise SystemExit(main())
