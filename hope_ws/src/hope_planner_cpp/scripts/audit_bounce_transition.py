#!/usr/bin/env python3
"""Audit the C++ bounce transition on existing canonical C3D exports.

This is an offline comparison tool. It exports accepted ball-center samples to
the replay CSV contract, compares the reset baseline with a candidate binary,
and scores recovery/velocity against the already reconstructed bounce labels.
No metric or threshold is connected to ROS publication or runner admission.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import subprocess
import tempfile
from pathlib import Path

import numpy as np


def quantile(values: list[float], fraction: float) -> float:
    finite = sorted(value for value in values if math.isfinite(value))
    if not finite:
        return math.nan
    position = (len(finite) - 1) * fraction
    low = math.floor(position)
    high = math.ceil(position)
    if low == high:
        return finite[low]
    return finite[low] * (high - position) + finite[high] * (position - low)


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [json_safe(item) for item in value]
    return value


def number(row: dict[str, str], key: str) -> float:
    try:
        value = float(row.get(key, ""))
    except (TypeError, ValueError):
        return math.nan
    return value if math.isfinite(value) else math.nan


def integer(row: dict[str, str], key: str) -> int:
    try:
        return int(float(row.get(key, "")))
    except (TypeError, ValueError):
        return 0


def parse_take(value: str) -> tuple[str, Path]:
    if "=" not in value:
        raise argparse.ArgumentTypeError("--take must be NAME=/path/to/take.npz")
    name, path = value.split("=", 1)
    if not name or not path:
        raise argparse.ArgumentTypeError("--take must be NAME=/path/to/take.npz")
    return name.lower(), Path(path).resolve()


def export_replay_csv(npz_path: Path, output_path: Path) -> tuple[np.ndarray, np.ndarray]:
    data = np.load(npz_path, allow_pickle=False)
    time_s = np.asarray(data["t"], dtype=np.float64)
    position = np.asarray(data["ball_pos_t_m"], dtype=np.float64)
    present = np.asarray(data["ball_present"], dtype=bool)
    if "ball_below_table" in data:
        present &= ~np.asarray(data["ball_below_table"], dtype=bool)
    present &= np.isfinite(time_s) & np.isfinite(position).all(axis=1)
    with output_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(
            (
                "object_key",
                "pose_accepted",
                "ros_stamp_ns",
                "normalized_x",
                "normalized_y",
                "normalized_z",
            )
        )
        for t_value, xyz in zip(time_s[present], position[present]):
            writer.writerow(
                (
                    "ball",
                    1,
                    int(round(float(t_value) * 1.0e9)),
                    float(xyz[0]),
                    float(xyz[1]),
                    float(xyz[2]),
                )
            )
    return time_s[present], position[present]


def run_replay(
    executable: Path,
    input_path: Path,
    output_path: Path,
    args: argparse.Namespace,
    bounce_options_supported: bool,
) -> dict:
    command = [
        str(executable),
        "--input",
        str(input_path),
        "--output",
        str(output_path),
        "--x-hit",
        "-2.0",
        "--solve-period",
        str(args.solve_period_s),
        "--window",
        str(args.window_s),
        "--min-span",
        str(args.min_span_s),
        "--min-samples",
        str(args.min_samples),
        "--huber-delta",
        str(args.huber_delta_m),
        "--recency-half-life",
        str(args.recency_half_life_s),
        "--iterations",
        str(args.iterations),
        "--drag-k",
        str(args.drag_k),
        "--restitution-h",
        str(args.restitution_h),
        "--restitution-v",
        str(args.restitution_v),
        "--spin-mode",
        "legacy",
        "--table-tangential-gain",
        str(args.table_tangential_gain),
        "--table-friction-cap-mu",
        str(args.table_friction_cap_mu),
    ]
    if bounce_options_supported:
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
    completed = subprocess.run(command, capture_output=True, text=True)
    if completed.returncode != 0:
        raise RuntimeError(
            f"replay failed rc={completed.returncode}: {completed.stderr.strip()}"
        )
    return json.loads(completed.stdout)


def read_solves(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return [row for row in csv.DictReader(stream) if row.get("kind") == "solve"]


def propagate_velocity(
    velocity: np.ndarray, duration_s: float, drag_k: float
) -> np.ndarray:
    value = velocity.astype(np.float64).copy()
    remaining = max(0.0, duration_s)
    while remaining > 1.0e-12:
        step = min(0.0005, remaining)
        acceleration = -drag_k * np.linalg.norm(value) * value
        acceleration[2] -= 9.81
        value += acceleration * step
        remaining -= step
    return value


def measured_minimum_time(
    times: np.ndarray, positions: np.ndarray, contact_time_s: float
) -> float:
    nearby = np.flatnonzero(np.abs(times - contact_time_s) <= 0.030)
    if nearby.size == 0:
        return math.nan
    return float(times[nearby[np.argmin(positions[nearby, 2])]])


def score_bounce(
    bounce: dict,
    rows: list[dict[str, str]],
    measured_minimum_s: float,
    drag_k: float,
) -> dict:
    contact_time_s = float(bounce["t_c"])
    outgoing = np.asarray(bounce["v_out"], dtype=np.float64)
    start_s = measured_minimum_s + 0.005
    end_s = measured_minimum_s + 0.080
    window = [
        row
        for row in rows
        if start_s <= number(row, "source_time_s") <= end_s
    ]
    recovery_window = [
        row
        for row in rows
        if start_s <= number(row, "source_time_s") <= measured_minimum_s + 0.140
    ]
    supports_transition_audit = any(
        "bounce_transition_used" in row for row in recovery_window
    )

    def usable_post_bounce_state(row: dict[str, str]) -> bool:
        return integer(row, "estimate_valid") == 1 and (
            not supports_transition_audit
            or integer(row, "bounce_transition_used") == 1
        )

    valid_rows = [row for row in window if usable_post_bounce_state(row)]
    recovery_rows = [
        row for row in recovery_window if usable_post_bounce_state(row)
    ]
    transition_rows = [
        row for row in window if integer(row, "bounce_transition_used") == 1
    ]
    first_valid_s = (
        number(recovery_rows[0], "source_time_s") if recovery_rows else math.nan
    )
    velocity_errors = {}
    vz_errors = {}
    for offset_s in (0.020, 0.040, 0.060):
        target_time_s = contact_time_s + offset_s
        candidates = [
            row
            for row in rows
            if usable_post_bounce_state(row)
            and abs(number(row, "source_time_s") - target_time_s) <= 0.008
        ]
        if not candidates:
            velocity_errors[str(offset_s)] = math.nan
            vz_errors[str(offset_s)] = math.nan
            continue
        row = min(
            candidates,
            key=lambda item: abs(number(item, "source_time_s") - target_time_s),
        )
        row_time_s = number(row, "source_time_s")
        expected = propagate_velocity(outgoing, row_time_s - contact_time_s, drag_k)
        estimated = np.array(
            [number(row, "est_vx"), number(row, "est_vy"), number(row, "est_vz")]
        )
        velocity_errors[str(offset_s)] = float(np.linalg.norm(estimated - expected))
        vz_errors[str(offset_s)] = float(abs(estimated[2] - expected[2]))
    return {
        "contact_time_s": contact_time_s,
        "measured_minimum_s": measured_minimum_s,
        "recovery_latency_from_minimum_s": (
            first_valid_s - measured_minimum_s
            if math.isfinite(first_valid_s)
            else math.nan
        ),
        "valid_fraction_5_to_80_ms": (
            len(valid_rows) / len(window) if window else math.nan
        ),
        "transition_fraction_5_to_80_ms": (
            len(transition_rows) / len(window) if window else math.nan
        ),
        "velocity_error_m_s": velocity_errors,
        "vz_error_m_s": vz_errors,
    }


def summarize(items: list[dict]) -> dict:
    summary = {
        "bounce_count": len(items),
        "recovered_count": sum(
            math.isfinite(item["recovery_latency_from_minimum_s"])
            for item in items
        ),
        "recovery_latency_median_s": quantile(
            [item["recovery_latency_from_minimum_s"] for item in items], 0.50
        ),
        "recovery_latency_p95_s": quantile(
            [item["recovery_latency_from_minimum_s"] for item in items], 0.95
        ),
        "valid_fraction_5_to_80_ms_median": quantile(
            [item["valid_fraction_5_to_80_ms"] for item in items], 0.50
        ),
        "transition_fraction_5_to_80_ms_median": quantile(
            [item["transition_fraction_5_to_80_ms"] for item in items], 0.50
        ),
    }
    for offset_s in ("0.02", "0.04", "0.06"):
        errors = [item["velocity_error_m_s"][offset_s] for item in items]
        vz_errors = [item["vz_error_m_s"][offset_s] for item in items]
        summary[f"velocity_error_{float(offset_s) * 1000:.0f}ms_count"] = sum(
            math.isfinite(value) for value in errors
        )
        summary[f"velocity_error_{float(offset_s) * 1000:.0f}ms_median_m_s"] = quantile(
            errors, 0.50
        )
        summary[f"velocity_error_{float(offset_s) * 1000:.0f}ms_p95_m_s"] = quantile(
            errors, 0.95
        )
        summary[f"vz_error_{float(offset_s) * 1000:.0f}ms_median_m_s"] = quantile(
            vz_errors, 0.50
        )
    return summary


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--replay-executable", type=Path, required=True)
    parser.add_argument("--baseline-replay-executable", type=Path)
    parser.add_argument("--bounces-json", type=Path, required=True)
    parser.add_argument("--take", action="append", type=parse_take, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--window-s", type=float, default=0.12)
    parser.add_argument("--min-span-s", type=float, default=0.08)
    parser.add_argument("--min-samples", type=int, default=12)
    parser.add_argument("--huber-delta-m", type=float, default=0.003)
    parser.add_argument("--recency-half-life-s", type=float, default=0.03)
    parser.add_argument("--iterations", type=int, default=3)
    parser.add_argument("--solve-period-s", type=float, default=0.0025)
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
    parser.add_argument("--table-tangential-gain", type=float, default=0.369)
    parser.add_argument("--table-friction-cap-mu", type=float, default=2.0)
    args = parser.parse_args(argv)

    takes = dict(args.take)
    labels = json.loads(args.bounces_json.read_text(encoding="utf-8"))
    by_take = {
        name: [item for item in labels if item.get("take", "").lower() == name]
        for name in takes
    }
    executables = {"candidate": args.replay_executable.resolve()}
    if args.baseline_replay_executable:
        executables["reset_baseline"] = args.baseline_replay_executable.resolve()

    report = {
        "audit_only": True,
        "data_split": {
            "development": ["tui", "zhengchang2", "xuan"],
            "rally_holdout": ["zhengchang"],
            "pure_bounce_holdout": ["chuntan"],
        },
        "estimator": {
            "window_s": args.window_s,
            "min_span_s": args.min_span_s,
            "min_samples": args.min_samples,
            "huber_delta_m": args.huber_delta_m,
            "recency_half_life_s": args.recency_half_life_s,
            "iterations": args.iterations,
            "drag_k": args.drag_k,
            "restitution_v": args.restitution_v,
            "table_tangential_gain": args.table_tangential_gain,
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
        "configurations": {},
    }
    with tempfile.TemporaryDirectory(prefix="hope_bounce_audit_") as directory:
        temp_root = Path(directory)
        exported = {}
        sample_arrays = {}
        for name, npz_path in takes.items():
            csv_path = temp_root / f"{name}.csv"
            sample_arrays[name] = export_replay_csv(npz_path, csv_path)
            exported[name] = csv_path

        for configuration, executable in executables.items():
            configuration_report = {"takes": {}}
            all_items = []
            for name in takes:
                replay_path = temp_root / f"{configuration}_{name}.csv"
                metadata = run_replay(
                    executable,
                    exported[name],
                    replay_path,
                    args,
                    configuration == "candidate",
                )
                rows = read_solves(replay_path)
                times, positions = sample_arrays[name]
                items = []
                for bounce in by_take[name]:
                    minimum_s = measured_minimum_time(
                        times, positions, float(bounce["t_c"])
                    )
                    if not math.isfinite(minimum_s):
                        continue
                    items.append(
                        score_bounce(bounce, rows, minimum_s, args.drag_k)
                    )
                all_items.extend(items)
                configuration_report["takes"][name] = {
                    "summary": summarize(items),
                    "replay_metadata": metadata,
                }
            configuration_report["all"] = summarize(all_items)
            report["configurations"][configuration] = configuration_report

    args.output.parent.mkdir(parents=True, exist_ok=True)
    safe_report = json_safe(report)
    args.output.write_text(
        json.dumps(safe_report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(safe_report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
