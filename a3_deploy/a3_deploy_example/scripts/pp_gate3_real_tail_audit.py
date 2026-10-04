#!/usr/bin/env python3
"""Audit Gate3 Planner output against recorded real-command tail fixtures.

This is a distribution-coverage verdict, not a replacement for Gate3's
functional/physical verdict.  It consumes the production Planner's live CSV;
it never publishes or substitutes a command.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence


REQUIRED_CSV_FIELDS = {
    "flight_id",
    "wire_valid",
    "reason",
    "base_x",
    "base_y",
    "strike_x",
    "strike_y",
    "strike_z",
    "racket_vx",
    "racket_vy",
    "racket_vz",
    "swing_sign",
    "tts_s",
}


def load_fixture(path: str | Path) -> dict[str, Any]:
    fixture = json.loads(Path(path).read_text(encoding="utf-8"))
    if fixture.get("schema_version") != 1:
        raise ValueError("real-tail fixture schema_version must be 1")
    rows = fixture.get("commands")
    if not isinstance(rows, list) or not rows:
        raise ValueError("real-tail fixture commands must be a non-empty list")
    tolerances = fixture.get("matching_tolerance")
    if not isinstance(tolerances, Mapping):
        raise ValueError("real-tail fixture matching_tolerance must be an object")
    for key, size in (("target_position_m", 3), ("target_velocity_mps", 3)):
        values = tolerances.get(key)
        if not isinstance(values, list) or len(values) != size:
            raise ValueError(f"matching_tolerance.{key} must contain {size} values")
        if any(not math.isfinite(float(value)) or float(value) < 0.0 for value in values):
            raise ValueError(f"matching_tolerance.{key} must be finite and non-negative")
    tts_tolerance = float(tolerances.get("tts_s", math.nan))
    if not math.isfinite(tts_tolerance) or tts_tolerance < 0.0:
        raise ValueError("matching_tolerance.tts_s must be finite and non-negative")
    seen_ids: set[str] = set()
    for index, row in enumerate(rows):
        if not isinstance(row, Mapping):
            raise ValueError(f"commands[{index}] must be an object")
        fixture_id = str(row.get("fixture_id", ""))
        if not fixture_id or fixture_id in seen_ids:
            raise ValueError("fixture_id values must be non-empty and unique")
        seen_ids.add(fixture_id)
        if int(row.get("swing_sign", 0)) not in (-1, 1):
            raise ValueError(f"{fixture_id}: swing_sign must be +/-1")
        for key in ("target_position_m", "target_velocity_mps"):
            values = row.get(key)
            if not isinstance(values, list) or len(values) != 3:
                raise ValueError(f"{fixture_id}: {key} must contain 3 values")
            if any(not math.isfinite(float(value)) for value in values):
                raise ValueError(f"{fixture_id}: {key} must be finite")
        if not math.isfinite(float(row.get("tts_s", math.nan))):
            raise ValueError(f"{fixture_id}: tts_s must be finite")
    return fixture


def read_live_planner_commands(
    path: str | Path, *, policy_z_offset_m: float
) -> list[dict[str, Any]]:
    if not math.isfinite(float(policy_z_offset_m)):
        raise ValueError("policy_z_offset_m must be finite")
    with Path(path).open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        missing = sorted(REQUIRED_CSV_FIELDS - set(reader.fieldnames or ()))
        if missing:
            raise ValueError("Planner CSV missing columns: " + ", ".join(missing))
        commands: list[dict[str, Any]] = []
        for csv_row, row in enumerate(reader, start=2):
            if row["wire_valid"] != "1" or row["reason"] != "command_valid":
                continue
            try:
                position = [
                    float(row["strike_x"]) - float(row["base_x"]),
                    float(row["strike_y"]) - float(row["base_y"]),
                    float(row["strike_z"]) + float(policy_z_offset_m),
                ]
                velocity = [
                    float(row["racket_vx"]),
                    float(row["racket_vy"]),
                    float(row["racket_vz"]),
                ]
                command = {
                    "csv_row": csv_row,
                    "flight_id": int(row["flight_id"]),
                    "swing_sign": int(row["swing_sign"]),
                    "target_position_m": position,
                    "target_velocity_mps": velocity,
                    "tts_s": float(row["tts_s"]),
                }
            except (TypeError, ValueError) as exc:
                raise ValueError(f"Planner CSV row {csv_row} is unparsable: {exc}") from exc
            numeric = [*position, *velocity, command["tts_s"]]
            if command["swing_sign"] not in (-1, 1) or any(
                not math.isfinite(value) for value in numeric
            ):
                raise ValueError(f"Planner CSV row {csv_row} has invalid command values")
            commands.append(command)
    return commands


def _matches(
    expected: Mapping[str, Any], actual: Mapping[str, Any], tolerances: Mapping[str, Any]
) -> bool:
    if int(expected["swing_sign"]) != int(actual["swing_sign"]):
        return False
    for key in ("target_position_m", "target_velocity_mps"):
        if any(
            abs(float(got) - float(want)) > float(tolerance)
            for got, want, tolerance in zip(
                actual[key], expected[key], tolerances[key], strict=True
            )
        ):
            return False
    return abs(float(actual["tts_s"]) - float(expected["tts_s"])) <= float(
        tolerances["tts_s"]
    )


def audit_real_tail_coverage(
    commands: Sequence[Mapping[str, Any]], fixture: Mapping[str, Any]
) -> dict[str, Any]:
    """Require distinct live commands to cover every recorded fixture row."""
    expected = fixture["commands"]
    tolerances = fixture["matching_tolerance"]
    adjacency = [
        [
            actual_index
            for actual_index, actual in enumerate(commands)
            if _matches(row, actual, tolerances)
        ]
        for row in expected
    ]

    # Maximum bipartite matching prevents one broad-tolerance Gate3 command
    # from pretending to cover several distinct real commands.
    actual_owner: dict[int, int] = {}

    def assign(expected_index: int, visited: set[int]) -> bool:
        for actual_index in adjacency[expected_index]:
            if actual_index in visited:
                continue
            visited.add(actual_index)
            previous = actual_owner.get(actual_index)
            if previous is None or assign(previous, visited):
                actual_owner[actual_index] = expected_index
                return True
        return False

    for expected_index in range(len(expected)):
        assign(expected_index, set())
    expected_to_actual = {
        expected_index: actual_index
        for actual_index, expected_index in actual_owner.items()
    }
    rows = []
    for expected_index, row in enumerate(expected):
        actual_index = expected_to_actual.get(expected_index)
        rows.append(
            {
                "fixture_id": row["fixture_id"],
                "covered": actual_index is not None,
                "matched_flight_id": (
                    int(commands[actual_index]["flight_id"])
                    if actual_index is not None
                    else None
                ),
                "candidate_flight_ids": [
                    int(commands[index]["flight_id"]) for index in adjacency[expected_index]
                ],
            }
        )

    def axis_envelope(key: str, axis: int) -> list[float] | None:
        values = [float(command[key][axis]) for command in commands]
        return [min(values), max(values)] if values else None

    tts_values = [float(command["tts_s"]) for command in commands]
    report = {
        "schema_version": 1,
        "verdict_kind": "gate3_distribution_coverage_only_v1",
        "fixture_contract": fixture["fixture_contract"],
        "coordinate_contract": fixture.get("coordinate_contract", "unspecified"),
        "fixture_rows": len(expected),
        "live_commands": len(commands),
        "covered_rows": sum(row["covered"] for row in rows),
        "distribution_coverage_pass": all(row["covered"] for row in rows),
        "rows": rows,
        "live_envelope": {
            "target_x_base_relative_m": axis_envelope("target_position_m", 0),
            "target_y_base_relative_m": axis_envelope("target_position_m", 1),
            "target_z_m": axis_envelope("target_position_m", 2),
            "racket_vz_mps": axis_envelope("target_velocity_mps", 2),
            "tts_s": [min(tts_values), max(tts_values)] if tts_values else None,
            "negative_backhand_vz_count": sum(
                int(command["swing_sign"]) == -1
                and float(command["target_velocity_mps"][2]) < 0.0
                for command in commands
            ),
            "cold_tts_below_0p4_count": sum(value <= 0.4 for value in tts_values),
        },
    }
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--planner-csv", required=True)
    parser.add_argument("--fixture", required=True)
    parser.add_argument("--json", required=True)
    parser.add_argument("--policy-z-offset-m", type=float, default=0.760)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        fixture = load_fixture(args.fixture)
        commands = read_live_planner_commands(
            args.planner_csv, policy_z_offset_m=args.policy_z_offset_m
        )
        report = audit_real_tail_coverage(commands, fixture)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        report = {
            "schema_version": 1,
            "verdict_kind": "gate3_distribution_coverage_only_v1",
            "distribution_coverage_pass": False,
            "configuration_error": str(exc),
        }
        rc = 2
    else:
        rc = 0 if report["distribution_coverage_pass"] else 1

    output = Path(args.json)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(
        "[g3-tail] %s covered=%s/%s live=%s report=%s"
        % (
            "COVERED" if report.get("distribution_coverage_pass") else "NOT_COVERED",
            report.get("covered_rows", 0),
            report.get("fixture_rows", 0),
            report.get("live_commands", 0),
            output,
        )
    )
    if report.get("configuration_error"):
        print(f"[g3-tail] CONFIG ERROR: {report['configuration_error']}")
    else:
        missing = [row["fixture_id"] for row in report["rows"] if not row["covered"]]
        if missing:
            print("[g3-tail] missing real-command fixtures: " + ", ".join(missing))
        print("[g3-tail] live envelope: " + json.dumps(report["live_envelope"], sort_keys=True))
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
