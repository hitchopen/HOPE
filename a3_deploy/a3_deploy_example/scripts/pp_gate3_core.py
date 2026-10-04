#!/usr/bin/env python3
"""Pure helpers for the autonomous, physical-ball Gate3.

This module deliberately has no ROS dependency so the scenario, evidence, and
per-side verdict contracts can be unit-tested on the host.  A scenario contains
only initial ball state plus ``shot_id``; swing side is always supplied later by
the production planner/runner evidence.
"""

from __future__ import annotations

import ast
import csv
import math
import random
import re
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Iterable, Mapping, MutableMapping, Sequence


TABLE_LENGTH_M = 2.74
TABLE_WIDTH_M = 1.525
TABLE_Y_MAX_M = 0.0
TABLE_HEIGHT_M = 0.760
NET_X_M = 1.370
NET_HEIGHT_M = 0.1525
BALL_RADIUS_M = 0.020
# Contact counters are loss-detectable, but their accompanying ball pose is the
# first 250 Hz sample after the 1 kHz contact edge.  Accept only a tightly
# bounded observation delay and account for the distance the rebounding ball
# can travel during that measured delay.
CONTACT_EDGE_MAX_OBSERVATION_LAG_S = 0.010
# A normal Hitter return reaches the opponent table roughly 0.48--0.55 s after
# the measured racket edge.  A shot parked before this conservative horizon has
# not answered the landing question: the outcome is right-censored by the
# external launcher.  This is deliberately longer than the observed flight and
# much shorter than the 2.5 s qualification window.
DEFAULT_MIN_POST_CONTACT_LANDING_OBSERVATION_S = 0.75


def audit_cpp_planner_debug_csv(
    path: str | Path, expected_flights: int
) -> dict[str, Any]:
    """Fail-closed audit of commands emitted by the live C++ Planner.

    Gate3 previously generated its planner-contract receipt with a separate
    Python reimplementation before launching the production process.  That can
    only prove properties of the reimplementation.  This helper instead reads
    the C++ node's loss-detectable audit stream and requires exactly one valid,
    post-net command for every physical flight.
    """
    expected_flights = int(expected_flights)
    report: dict[str, Any] = {
        "schema_version": 1,
        "evidence_source": "live_cpp_planner_debug_csv_v1",
        "planner_runtime": "hope_planner_cpp_node",
        "planner_debug_csv": str(path),
        "flight_packet_audit_csv": str(path) + ".flight_packets.csv",
        "input_contract": "unknown",
        "expected_flights": expected_flights,
        "rows_observed": 0,
        "valid_commands": 0,
        "runner_harness_pass": False,
        "planner_contract_pass": False,
        "physical_contact_measured": False,
        "landing_measured": False,
        "errors": [],
        "rows": [],
    }
    errors: list[str] = report["errors"]
    if expected_flights <= 0:
        errors.append("expected_flights must be positive")
        return report

    try:
        with open(path, newline="") as stream:
            reader = csv.DictReader(stream)
            fieldnames = set(reader.fieldnames or ())
            raw_rows = list(reader)
    except (FileNotFoundError, OSError, csv.Error) as exc:
        errors.append(f"cannot read live C++ Planner audit CSV: {exc}")
        return report

    required_fields = {
        "session_id",
        "command_seq",
        "flight_id",
        "wire_valid",
        "reason",
        "strike_x",
        "strike_y",
        "strike_z",
        "racket_vx",
        "racket_vy",
        "racket_vz",
        "swing_sign",
        "tts_s",
        "source_age_at_publish_ms",
        "total_ms",
        "input_ring_drops",
        "logger_drops",
        "base_valid",
        "base_age_ms",
        "estimator_kind",
        "control_stage2_mode",
        "out_of_order_samples_total",
        "one_shot_enabled",
        "one_shot_flight_seq",
        "net_cross_source_time_s",
        "commit_source_time_s",
        "post_net_commit_delay_s",
        "post_net_future_bounce_tangential_gain",
        "trajectory_epoch",
        "snapshot_sequence",
        "snapshot_published_total",
        "snapshot_consumed_total",
        "snapshot_superseded_total",
    }
    packet_fields = {
        "flight_packet_input",
        "packet_session_id",
        "packet_producer_instance_id",
        "packet_payload_hash",
        "packet_transmit_index",
        "packet_transmit_count",
        "packet_received_total",
        "packet_accepted_total",
        "packet_duplicate_total",
        "packet_conflict_total",
        "packet_invalid_total",
        "packet_queue_depth",
    }
    packet_columns_present = bool(packet_fields & fieldnames)
    if packet_columns_present:
        required_fields |= packet_fields
    missing_fields = sorted(required_fields - fieldnames)
    if missing_fields:
        errors.append("missing C++ audit columns: " + ", ".join(missing_fields))
        return report

    report["rows_observed"] = len(raw_rows)
    if len(raw_rows) != expected_flights:
        errors.append(
            f"expected exactly {expected_flights} C++ solve rows, got {len(raw_rows)}"
        )

    integer_fields = (
        "command_seq",
        "flight_id",
        "wire_valid",
        "input_ring_drops",
        "logger_drops",
        "base_valid",
        "out_of_order_samples_total",
        "one_shot_enabled",
        "one_shot_flight_seq",
        "trajectory_epoch",
        "snapshot_sequence",
        "snapshot_published_total",
        "snapshot_consumed_total",
        "snapshot_superseded_total",
        "swing_sign",
    )
    if packet_columns_present:
        integer_fields += (
            "flight_packet_input",
            "packet_transmit_index",
            "packet_transmit_count",
            "packet_received_total",
            "packet_accepted_total",
            "packet_duplicate_total",
            "packet_conflict_total",
            "packet_invalid_total",
            "packet_queue_depth",
        )
    finite_fields = (
        "strike_x",
        "strike_y",
        "strike_z",
        "racket_vx",
        "racket_vy",
        "racket_vz",
        "tts_s",
        "source_age_at_publish_ms",
        "total_ms",
        "base_age_ms",
        "net_cross_source_time_s",
        "commit_source_time_s",
        "post_net_commit_delay_s",
        "post_net_future_bounce_tangential_gain",
    )
    normalized_rows: list[dict[str, Any]] = []
    for row_index, row in enumerate(raw_rows, start=1):
        try:
            integers = {name: int(row[name]) for name in integer_fields}
            finite = {name: float(row[name]) for name in finite_fields}
        except (TypeError, ValueError) as exc:
            errors.append(f"row {row_index} has an unparsable field: {exc}")
            continue
        nonfinite = [name for name, value in finite.items() if not math.isfinite(value)]
        if nonfinite:
            errors.append(
                f"row {row_index} has non-finite fields: {', '.join(nonfinite)}"
            )
            continue

        row_errors: list[str] = []
        if integers["wire_valid"] != 1 or row["reason"] != "command_valid":
            row_errors.append("wire command is not valid")
        if integers["swing_sign"] not in (-1, 1):
            row_errors.append("planner swing sign is not +/-1")
        if integers["one_shot_enabled"] != 1:
            row_errors.append("post-net one-shot is disabled")
        if integers["one_shot_flight_seq"] != integers["flight_id"]:
            row_errors.append("one-shot flight identity does not match wire flight_id")
        if integers["trajectory_epoch"] != integers["flight_id"]:
            row_errors.append("trajectory epoch does not match wire flight_id")
        if integers["snapshot_sequence"] != integers["flight_id"]:
            row_errors.append("snapshot identity does not match wire flight_id")
        if row["estimator_kind"] != "batch_physics_cpp_no_ekf_persistent_bounce":
            row_errors.append("unexpected estimator implementation")
        if row["control_stage2_mode"] != "post_net_one_shot_effective_bounce_zero_spin":
            row_errors.append("unexpected C++ Stage-2 mode")
        if integers["input_ring_drops"] != 0:
            row_errors.append("Planner input-ring drops are nonzero")
        if integers["logger_drops"] != 0:
            row_errors.append("Planner audit logger drops are nonzero")
        if integers["out_of_order_samples_total"] != 0:
            row_errors.append("out-of-order source samples are nonzero")
        if integers["base_valid"] != 1:
            row_errors.append("base snapshot is invalid")
        if finite["tts_s"] <= 0.0:
            row_errors.append("time-to-strike is not positive")
        if finite["source_age_at_publish_ms"] < 0.0:
            row_errors.append("Planner source age is negative")
        # The base snapshot is sampled just after solve_finished_steady_ns is
        # captured. A concurrently arriving base callback can therefore make
        # this diagnostic age a few tens of microseconds negative.
        if finite["base_age_ms"] < -0.1:
            row_errors.append("base age exceeds the concurrency tolerance")
        if finite["total_ms"] < 0.0:
            row_errors.append("solve duration is negative")
        if not math.isclose(
            finite["post_net_commit_delay_s"], 0.05, rel_tol=0.0, abs_tol=1.0e-6
        ):
            row_errors.append("post-net commit delay is not 50 ms")
        if not math.isclose(
            finite["commit_source_time_s"] - finite["net_cross_source_time_s"],
            finite["post_net_commit_delay_s"],
            rel_tol=0.0,
            abs_tol=1.0e-6,
        ):
            row_errors.append("commit timestamp is inconsistent with net-cross + delay")
        if not math.isclose(
            finite["post_net_future_bounce_tangential_gain"],
            0.369,
            rel_tol=0.0,
            abs_tol=1.0e-9,
        ):
            row_errors.append("future-bounce tangential gain is not the C++ profile value")

        packet_text: dict[str, str] = {}
        if packet_columns_present:
            packet_text = {
                name: str(row[name])
                for name in (
                    "packet_session_id",
                    "packet_producer_instance_id",
                    "packet_payload_hash",
                )
            }
            if integers["flight_packet_input"] not in (0, 1):
                row_errors.append("flight-packet input flag is not boolean")
            if integers["flight_packet_input"] == 1:
                if packet_text["packet_session_id"] != str(row["session_id"]):
                    row_errors.append("packet session does not match Planner session")
                if not packet_text["packet_producer_instance_id"]:
                    row_errors.append("packet producer instance is empty")
                if re.fullmatch(r"[0-9a-f]{16}", packet_text["packet_payload_hash"]) is None:
                    row_errors.append("packet payload hash is not a 64-bit lowercase hex digest")
                transmit_count = integers["packet_transmit_count"]
                transmit_index = integers["packet_transmit_index"]
                if transmit_count <= 0 or not 0 <= transmit_index < transmit_count:
                    row_errors.append("accepted packet transmit identity is invalid")
                if integers["packet_conflict_total"] != 0:
                    row_errors.append("packet identity conflicts are nonzero")
                if integers["packet_invalid_total"] != 0:
                    row_errors.append("invalid packets are nonzero")
                if integers["packet_queue_depth"] != 0:
                    row_errors.append("packet queue was not drained by the solve")

        if row_errors:
            errors.extend(f"row {row_index}: {message}" for message in row_errors)
        normalized_rows.append(
            {
                "session_id": row["session_id"],
                **integers,
                **finite,
                **packet_text,
                "reason": row["reason"],
                "estimator_kind": row["estimator_kind"],
                "control_stage2_mode": row["control_stage2_mode"],
                "planner_side": (
                    "forehand" if integers["swing_sign"] == 1 else "backhand"
                ),
                "row_pass": not row_errors,
            }
        )

    report["rows"] = normalized_rows
    expected_sequence = list(range(1, expected_flights + 1))
    for field in (
        "flight_id",
        "command_seq",
        "one_shot_flight_seq",
        "trajectory_epoch",
        "snapshot_sequence",
    ):
        observed = [row[field] for row in normalized_rows]
        if observed != expected_sequence:
            errors.append(f"{field} sequence is {observed}, expected {expected_sequence}")
    packet_modes = {
        row.get("flight_packet_input", 0) for row in normalized_rows
    }
    if len(packet_modes) > 1:
        errors.append(f"Planner input contract changed during Gate3: {sorted(packet_modes)}")
    packet_mode = packet_columns_present and packet_modes == {1}
    if packet_mode:
        report["input_contract"] = "immutable_flight_packet_v1"
        accepted = [row["packet_accepted_total"] for row in normalized_rows]
        if accepted != expected_sequence:
            errors.append(
                f"packet_accepted_total sequence is {accepted}, expected {expected_sequence}"
            )
        for field in ("snapshot_published_total", "snapshot_consumed_total"):
            observed = [row[field] for row in normalized_rows]
            if any(observed):
                errors.append(
                    f"legacy {field} is active during immutable packet input: {observed}"
                )
        producers = {
            row["packet_producer_instance_id"] for row in normalized_rows
        }
        if len(producers) != 1:
            errors.append(f"packet producer changed during Gate3: {sorted(producers)}")
        payload_hashes = [row["packet_payload_hash"] for row in normalized_rows]
        if len(set(payload_hashes)) != len(payload_hashes):
            errors.append("packet payload hashes are not unique per physical flight")
        received = [row["packet_received_total"] for row in normalized_rows]
        if any(
            current <= previous
            for previous, current in zip(received, received[1:])
        ):
            errors.append(f"packet receive counter is not strictly increasing: {received}")
        duplicates = [row["packet_duplicate_total"] for row in normalized_rows]
        if any(
            current < previous
            for previous, current in zip(duplicates, duplicates[1:])
        ):
            errors.append(f"packet duplicate counter regressed: {duplicates}")

        receive_path = Path(report["flight_packet_audit_csv"])
        try:
            with receive_path.open(newline="") as stream:
                receive_reader = csv.DictReader(stream)
                receive_fieldnames = set(receive_reader.fieldnames or ())
                receive_rows = list(receive_reader)
        except (FileNotFoundError, OSError, csv.Error) as exc:
            errors.append(f"cannot read C++ flight-packet receive audit CSV: {exc}")
            receive_fieldnames = set()
            receive_rows = []
        receive_required = {
            "session_id",
            "producer_instance_id",
            "trajectory_epoch",
            "flight_sequence",
            "payload_hash",
            "transmit_index",
            "transmit_count",
            "sample_count",
            "source_age_ms",
            "dedupe_result",
            "accepted_for_solve",
            "packet_queue_depth",
        }
        receive_missing = sorted(receive_required - receive_fieldnames)
        if receive_missing:
            errors.append(
                "missing C++ flight-packet audit columns: "
                + ", ".join(receive_missing)
            )
        grouped: dict[int, list[dict[str, Any]]] = {}
        if not receive_missing:
            for row_index, row in enumerate(receive_rows, start=1):
                try:
                    normalized = {
                        "session_id": str(row["session_id"]),
                        "producer_instance_id": str(row["producer_instance_id"]),
                        "trajectory_epoch": int(row["trajectory_epoch"]),
                        "flight_sequence": int(row["flight_sequence"]),
                        "payload_hash": str(row["payload_hash"]),
                        "transmit_index": int(row["transmit_index"]),
                        "transmit_count": int(row["transmit_count"]),
                        "sample_count": int(row["sample_count"]),
                        "source_age_ms": float(row["source_age_ms"]),
                        "dedupe_result": str(row["dedupe_result"]),
                        "accepted_for_solve": int(row["accepted_for_solve"]),
                        "packet_queue_depth": int(row["packet_queue_depth"]),
                    }
                except (TypeError, ValueError) as exc:
                    errors.append(
                        f"flight-packet audit row {row_index} is unparsable: {exc}"
                    )
                    continue
                if (
                    not math.isfinite(normalized["source_age_ms"])
                    or normalized["source_age_ms"] < 0.0
                ):
                    errors.append(
                        f"flight-packet audit row {row_index} has invalid source age"
                    )
                grouped.setdefault(normalized["flight_sequence"], []).append(
                    normalized
                )
        if sorted(grouped) != expected_sequence:
            errors.append(
                f"flight-packet receive identities are {sorted(grouped)}, "
                f"expected {expected_sequence}"
            )
        solve_by_flight = {
            row["flight_id"]: row for row in normalized_rows
        }
        for flight_sequence in expected_sequence:
            group = grouped.get(flight_sequence, [])
            if not group:
                continue
            transmit_counts = {row["transmit_count"] for row in group}
            if len(transmit_counts) != 1 or next(iter(transmit_counts)) <= 0:
                errors.append(
                    f"flight {flight_sequence} has inconsistent transmit counts: "
                    f"{sorted(transmit_counts)}"
                )
                continue
            transmit_count = next(iter(transmit_counts))
            indices = sorted(row["transmit_index"] for row in group)
            if indices != list(range(transmit_count)):
                errors.append(
                    f"flight {flight_sequence} transmit indices are {indices}, "
                    f"expected {list(range(transmit_count))}"
                )
            accepted_rows = [
                row for row in group if row["accepted_for_solve"] == 1
            ]
            duplicate_rows = [
                row for row in group if row["dedupe_result"] == "duplicate"
            ]
            if (
                len(accepted_rows) != 1
                or accepted_rows[0]["dedupe_result"] != "accepted"
                or len(duplicate_rows) != transmit_count - 1
            ):
                errors.append(
                    f"flight {flight_sequence} acceptance/dedupe pattern is invalid"
                )
            solve_row = solve_by_flight.get(flight_sequence)
            if solve_row is None:
                continue
            for receive_row in group:
                if (
                    receive_row["session_id"] != solve_row["packet_session_id"]
                    or receive_row["producer_instance_id"]
                    != solve_row["packet_producer_instance_id"]
                    or receive_row["payload_hash"]
                    != solve_row["packet_payload_hash"]
                    or receive_row["trajectory_epoch"] != flight_sequence
                    or receive_row["sample_count"] <= 0
                ):
                    errors.append(
                        f"flight {flight_sequence} receive identity does not match solve audit"
                    )
                    break
        report["flight_packet_receive_rows"] = len(receive_rows)
    else:
        report["input_contract"] = "poses_latest_snapshot_v1"
        for field in ("snapshot_published_total", "snapshot_consumed_total"):
            observed = [row[field] for row in normalized_rows]
            if observed != expected_sequence:
                errors.append(
                    f"{field} sequence is {observed}, expected {expected_sequence}"
                )

    superseded = [row["snapshot_superseded_total"] for row in normalized_rows]
    if any(value != 0 for value in superseded):
        errors.append(f"latest-wins snapshots were superseded: {superseded}")

    report["valid_commands"] = sum(
        row["wire_valid"] == 1 and row["reason"] == "command_valid"
        for row in normalized_rows
    )
    if normalized_rows:
        report["tts_s_range"] = [
            min(row["tts_s"] for row in normalized_rows),
            max(row["tts_s"] for row in normalized_rows),
        ]
        report["source_age_at_publish_ms_range"] = [
            min(row["source_age_at_publish_ms"] for row in normalized_rows),
            max(row["source_age_at_publish_ms"] for row in normalized_rows),
        ]
        report["racket_velocity_range"] = {
            axis: [
                min(row[field] for row in normalized_rows),
                max(row[field] for row in normalized_rows),
            ]
            for axis, field in (("x", "racket_vx"), ("y", "racket_vy"), ("z", "racket_vz"))
        }
    report["planner_contract_pass"] = not errors
    report["runner_harness_pass"] = report["planner_contract_pass"]
    report["pass"] = report["planner_contract_pass"]
    return report


def join_planner_evidence_by_flight_id(
    serve_rows: Sequence[MutableMapping[str, Any]],
    planner_report: Mapping[str, Any],
    expected_flights: int,
) -> dict[str, Any]:
    """Attach Planner side to every launched flight using immutable identity.

    A missed Runner admission has no engage log, but it still has a valid Planner
    command and therefore a Planner-selected side.  Joining by the launch window
    would silently drop precisely those missed rapid flights from the per-side
    denominator.  ``flight_id`` is the shared immutable identity across the ball
    launcher, packetizer, Planner, Runner trace and physical evidence.
    """

    errors: list[str] = []
    expected_sequence = list(range(1, int(expected_flights) + 1))
    serve_ids = [int(row.get("flight_id") or 0) for row in serve_rows]
    if serve_ids != expected_sequence:
        errors.append(
            f"serve flight_id sequence is {serve_ids}, expected {expected_sequence}"
        )
    shot_ids = [int(row.get("shot_id") or 0) for row in serve_rows]
    if shot_ids != serve_ids:
        errors.append(
            f"launcher shot_id sequence {shot_ids} does not match flight_id {serve_ids}"
        )

    planner_rows = list(planner_report.get("rows", []))
    planner_ids = [int(row.get("flight_id") or 0) for row in planner_rows]
    duplicate_planner_ids = sorted(
        flight_id
        for flight_id in set(planner_ids)
        if flight_id > 0 and planner_ids.count(flight_id) > 1
    )
    if duplicate_planner_ids:
        errors.append(f"duplicate Planner flight_ids: {duplicate_planner_ids}")
    planner_by_id = {
        int(row.get("flight_id") or 0): row
        for row in planner_rows
        if int(row.get("flight_id") or 0) > 0
    }
    missing = sorted(set(expected_sequence) - set(planner_by_id))
    extra = sorted(set(planner_by_id) - set(expected_sequence))
    if missing:
        errors.append(f"Planner evidence is missing flight_ids: {missing}")
    if extra:
        errors.append(f"Planner evidence has extra flight_ids: {extra}")

    joined: list[int] = []
    backfilled: list[int] = []
    side_mismatches: list[int] = []
    for serve in serve_rows:
        flight_id = int(serve.get("flight_id") or 0)
        planner = planner_by_id.get(flight_id)
        if planner is None:
            continue
        side = planner.get("planner_side")
        if side not in ("forehand", "backhand"):
            errors.append(f"flight {flight_id} has invalid Planner side {side!r}")
            continue
        if not bool(planner.get("row_pass", False)):
            errors.append(f"flight {flight_id} Planner row did not pass its contract")
        existing = serve.get("command_side")
        if existing is None:
            serve["command_side"] = side
            backfilled.append(flight_id)
        elif existing != side:
            side_mismatches.append(flight_id)
            errors.append(
                f"flight {flight_id} Runner side {existing!r} contradicts Planner side {side!r}"
            )
        serve["planner_side"] = side
        serve["planner_command"] = {
            "flight_id": flight_id,
            "command_seq": int(planner.get("command_seq") or 0),
            "planner_side": side,
            "strike_time_s": float(planner.get("tts_s") or 0.0),
        }
        joined.append(flight_id)

    return {
        "join_contract": "immutable_flight_id_v1",
        "expected_flight_ids": expected_sequence,
        "joined_flight_ids": joined,
        "side_backfilled_flight_ids": backfilled,
        "side_mismatch_flight_ids": side_mismatches,
        "missing_planner_flight_ids": missing,
        "extra_planner_flight_ids": extra,
        "errors": errors,
        "pass": not errors and joined == expected_sequence,
    }


def audit_runner_localization_trace(
    path: str | Path,
) -> dict[str, Any]:
    """Audit localization freshness on every authoritative MOTION tick.

    Serve stdout is deliberately not an evidence source here: a no-ball run has
    no serve rows, while the Runner still consumes localization on every 50 Hz
    MOTION tick.  The CSV must therefore contain a contiguous MOTION interval,
    and each row in that interval must carry an explicit boolean freshness bit.
    """

    errors: list[str] = []
    report: dict[str, Any] = {
        "schema_version": 1,
        "evidence_source": "runner_trace_all_motion_ticks_v1",
        "runner_trace_csv": str(path),
        "continuous_fresh_required": True,
        "trace_rows": 0,
        "motion_ticks": 0,
        "fresh_ticks": 0,
        "stale_ticks": 0,
        "fresh_fraction": 0.0,
        "first_motion_tick": None,
        "last_motion_tick": None,
        "first_motion_wall_time_ns": None,
        "last_motion_wall_time_ns": None,
        "motion_tick_span_contiguous": False,
        "continuous_fresh": False,
        "stale_tick_ranges": [],
        "errors": errors,
        "pass": False,
    }
    try:
        with open(path, newline="") as stream:
            reader = csv.DictReader(stream)
            fields = set(reader.fieldnames or ())
            rows = list(reader)
    except (FileNotFoundError, OSError, csv.Error) as exc:
        errors.append(f"cannot read Runner trace CSV: {exc}")
        return report

    required = {"tick", "wall_time_ns", "mode", "localization_fresh"}
    missing = sorted(required - fields)
    if missing:
        errors.append("missing Runner localization columns: " + ", ".join(missing))
        return report
    report["trace_rows"] = len(rows)
    if not rows:
        errors.append("Runner trace is empty")
        return report

    motion: list[tuple[int, int, bool]] = []
    for row_index, row in enumerate(rows, start=2):
        raw_mode = str(row.get("mode", "")).strip().upper()
        try:
            mode = 3 if raw_mode == "MOTION" else int(raw_mode)
        except ValueError as exc:
            errors.append(f"Runner localization row {row_index} has invalid mode: {exc}")
            continue
        if mode != 3:
            continue
        try:
            tick = int(row["tick"])
            wall_time_ns = int(row["wall_time_ns"])
        except (TypeError, ValueError) as exc:
            errors.append(
                f"Runner localization row {row_index} has invalid tick/time: {exc}"
            )
            continue
        raw_fresh = str(row.get("localization_fresh", "")).strip().lower()
        if raw_fresh in {"1", "true"}:
            fresh = True
        elif raw_fresh in {"0", "false"}:
            fresh = False
        else:
            errors.append(
                f"Runner localization row {row_index} has invalid freshness bit"
            )
            continue
        if tick < 0 or wall_time_ns <= 0:
            errors.append(
                f"Runner localization row {row_index} has non-positive tick/time"
            )
            continue
        motion.append((tick, wall_time_ns, fresh))

    if not motion:
        errors.append("Runner trace has no MOTION (mode=3) rows")
        return report

    report["motion_ticks"] = len(motion)
    report["fresh_ticks"] = sum(int(fresh) for _, _, fresh in motion)
    report["stale_ticks"] = len(motion) - report["fresh_ticks"]
    report["fresh_fraction"] = report["fresh_ticks"] / len(motion)
    report["first_motion_tick"] = motion[0][0]
    report["last_motion_tick"] = motion[-1][0]
    report["first_motion_wall_time_ns"] = motion[0][1]
    report["last_motion_wall_time_ns"] = motion[-1][1]

    contiguous = all(
        right[0] == left[0] + 1 and right[1] > left[1]
        for left, right in zip(motion, motion[1:])
    )
    report["motion_tick_span_contiguous"] = contiguous
    if not contiguous:
        errors.append("Runner MOTION trace is sparse, reordered, or interrupted")

    stale_ranges: list[dict[str, int]] = []
    range_start: int | None = None
    range_end: int | None = None
    for tick, _, fresh in motion:
        if not fresh:
            if range_start is None:
                range_start = tick
            range_end = tick
        elif range_start is not None and range_end is not None:
            stale_ranges.append(
                {
                    "first_tick": range_start,
                    "last_tick": range_end,
                    "ticks": range_end - range_start + 1,
                }
            )
            range_start = None
            range_end = None
    if range_start is not None and range_end is not None:
        stale_ranges.append(
            {
                "first_tick": range_start,
                "last_tick": range_end,
                "ticks": range_end - range_start + 1,
            }
        )
    report["stale_tick_ranges"] = stale_ranges
    report["continuous_fresh"] = bool(
        contiguous and not errors and report["stale_ticks"] == 0
    )
    if report["stale_ticks"]:
        errors.append(
            f"{report['stale_ticks']} of {len(motion)} MOTION ticks have stale localization"
        )
    report["pass"] = bool(report["continuous_fresh"] and not errors)
    return report


def audit_runner_engagement_trace(
    path: str | Path,
    expected_flight_ids: Iterable[int],
    *,
    require_completion_contract: bool = False,
    physical_contact_wall_time_ns_by_flight: Mapping[int, int] | None = None,
    contact_pose_max_gap_s: float = 0.03,
) -> dict[str, Any]:
    """Read authoritative Runner engagement and lifecycle-completion edges.

    The conductor's 250 ms polling timestamp is unsuitable for a 1.15 s cadence
    qualification.  The Runner writes the exact control-tick ``wall_time_ns`` and
    frozen immutable flight identity on its ``planner_status=engage`` row for
    an idle admission, or ``planner_status=preempt_engage`` for a schema22
    mid-swing fresh-flight admission.  Both statuses are one engagement edge;
    treating the latter as a miss would make a successful rapid policy
    impossible to qualify.
    Missing expected flights are reported as outcomes, not trace corruption.

    Schema22 also exposes a monotonically increasing completion sequence and
    the immutable identity of the *old* flight closed by an atomic preemption.
    This is deliberately separate from ``preempt_engage``: engaging flight N
    closes flight N-1, and neither edge is evidence of racket contact.  A
    native clip end closes the final flight.  Persistent per-tick snapshots are
    de-duplicated by sequence number so polling frequency cannot change counts.
    """

    expected = [int(value) for value in expected_flight_ids]
    errors: list[str] = []
    report: dict[str, Any] = {
        "schema_version": 2,
        "evidence_source": "runner_trace_engagement_edges_v2",
        "gap_time_source": "runner_trace.wall_time_ns",
        "runner_trace_csv": str(path),
        "expected_flight_ids": expected,
        "engaged_flight_ids": [],
        "missed_flight_ids": expected,
        "engage_events": [],
        "engage_gaps_s": [],
        "completion_contract_required": bool(require_completion_contract),
        "completion_contract_available": False,
        "completion_events": [],
        "completed_flight_ids": [],
        "incomplete_flight_ids": expected,
        "completion_contract_pass": not bool(require_completion_contract),
        "physical_contact_pose_requested": bool(
            physical_contact_wall_time_ns_by_flight
        ),
        "physical_contact_pose_events": [],
        "physical_contact_pose_missing_flight_ids": sorted(
            int(value)
            for value in (physical_contact_wall_time_ns_by_flight or {})
        ),
        "physical_contact_pose_contract_pass": not bool(
            physical_contact_wall_time_ns_by_flight
        ),
        "errors": errors,
        "trace_contract_pass": False,
    }
    if not expected or expected != sorted(set(expected)) or any(value <= 0 for value in expected):
        errors.append("expected flight IDs must be unique, positive and increasing")
        return report
    try:
        with open(path, newline="") as stream:
            reader = csv.DictReader(stream)
            fields = set(reader.fieldnames or ())
            rows = list(reader)
    except (FileNotFoundError, OSError, csv.Error) as exc:
        errors.append(f"cannot read Runner trace CSV: {exc}")
        return report
    required = {
        "tick", "wall_time_ns", "level", "planner_status", "frozen_flight_id"
    }
    completion_fields = {
        "frozen_producer_epoch",
        "planner_completion_seq",
        "planner_completed_producer_epoch",
        "planner_completed_flight_id",
        "planner_completion_kind",
    }
    missing_fields = sorted(required - fields)
    if missing_fields:
        errors.append("missing Runner trace columns: " + ", ".join(missing_fields))
        return report
    present_completion_fields = completion_fields & fields
    if (
        require_completion_contract
        and present_completion_fields
        and present_completion_fields != completion_fields
    ):
        errors.append(
            "partial Runner lifecycle completion columns: "
            + ", ".join(sorted(present_completion_fields))
        )
        return report
    completion_available = completion_fields <= fields
    report["completion_contract_available"] = completion_available
    if not rows:
        errors.append("Runner trace is empty")
        return report

    contact_pose_events: list[dict[str, Any]] = []
    requested_contact_stamps = {
        int(flight_id): int(stamp_ns)
        for flight_id, stamp_ns in (
            physical_contact_wall_time_ns_by_flight or {}
        ).items()
    }
    if requested_contact_stamps:
        if not {"base_x", "base_y"} <= fields:
            errors.append(
                "Runner trace lacks base_x/base_y required for physical-contact pose join"
            )
        elif (
            not math.isfinite(float(contact_pose_max_gap_s))
            or contact_pose_max_gap_s <= 0.0
        ):
            errors.append("contact_pose_max_gap_s must be finite and positive")
        else:
            trace_pose_rows: list[tuple[int, int, int, float, float]] = []
            for row_index, row in enumerate(rows, start=2):
                try:
                    wall_time_ns = int(row["wall_time_ns"])
                    frozen_flight_id = int(row["frozen_flight_id"])
                    base_x = float(row["base_x"])
                    base_y = float(row["base_y"])
                except (TypeError, ValueError) as exc:
                    errors.append(
                        f"Runner base-pose row {row_index} is unparsable: {exc}"
                    )
                    continue
                if (
                    wall_time_ns <= 0
                    or not math.isfinite(base_x)
                    or not math.isfinite(base_y)
                ):
                    errors.append(
                        f"Runner base-pose row {row_index} is non-finite/invalid"
                    )
                    continue
                trace_pose_rows.append(
                    (
                        row_index,
                        wall_time_ns,
                        frozen_flight_id,
                        base_x,
                        base_y,
                    )
                )
            max_gap_ns = int(round(contact_pose_max_gap_s * 1.0e9))
            for flight_id, contact_stamp_ns in sorted(
                requested_contact_stamps.items()
            ):
                flight_pose_rows = [
                    item for item in trace_pose_rows if item[2] == flight_id
                ]
                if contact_stamp_ns <= 0 or not flight_pose_rows:
                    continue
                nearest = min(
                    flight_pose_rows,
                    key=lambda item: abs(item[1] - contact_stamp_ns),
                )
                gap_ns = abs(nearest[1] - contact_stamp_ns)
                if gap_ns > max_gap_ns:
                    continue
                contact_pose_events.append(
                    {
                        "flight_id": flight_id,
                        "physical_contact_wall_time_ns": contact_stamp_ns,
                        "runner_tick_wall_time_ns": nearest[1],
                        "runner_csv_row": nearest[0],
                        "join_gap_s": round(gap_ns * 1.0e-9, 9),
                        "base_x": nearest[3],
                        "base_y": nearest[4],
                    }
                )

    events: list[dict[str, Any]] = []
    engagement_statuses = {"engage", "preempt_engage"}
    for row_index, row in enumerate(rows, start=2):
        edge_kind = str(row.get("planner_status", ""))
        if edge_kind not in engagement_statuses:
            continue
        try:
            event = {
                "flight_id": int(row["frozen_flight_id"]),
                "producer_epoch": (
                    int(row["frozen_producer_epoch"])
                    if "frozen_producer_epoch" in fields else 0
                ),
                "tick": int(row["tick"]),
                "wall_time_ns": int(row["wall_time_ns"]),
                "level": int(row["level"]),
                "edge_kind": edge_kind,
            }
        except (TypeError, ValueError) as exc:
            errors.append(f"Runner engage row {row_index} is unparsable: {exc}")
            continue
        if event["flight_id"] not in expected:
            errors.append(
                f"Runner engage row {row_index} has unexpected flight_id={event['flight_id']}"
            )
        if event["level"] != 1:
            errors.append(f"Runner engage row {row_index} does not enter level=1")
        if event["wall_time_ns"] <= 0:
            errors.append(f"Runner engage row {row_index} has non-positive wall_time_ns")
        events.append(event)

    engaged = [event["flight_id"] for event in events]
    duplicates = sorted(
        flight_id for flight_id in set(engaged) if engaged.count(flight_id) > 1
    )
    if duplicates:
        errors.append(f"Runner has duplicate engage edges for flight_ids: {duplicates}")
    if engaged != sorted(engaged):
        errors.append(f"Runner engage flight IDs are not increasing: {engaged}")
    walls = [event["wall_time_ns"] for event in events]
    if any(current <= previous for previous, current in zip(walls, walls[1:])):
        errors.append("Runner engage wall_time_ns is not strictly increasing")

    gaps = [
        round((current - previous) * 1.0e-9, 6)
        for previous, current in zip(walls, walls[1:])
    ]

    completion_events: list[dict[str, Any]] = []
    if completion_available and require_completion_contract:
        last_seq = 0
        event_by_seq: dict[int, tuple[int, int, str]] = {}
        active_identity: tuple[int, int] | None = None
        closed_identities: set[tuple[int, int]] = set()
        for row_index, row in enumerate(rows, start=2):
            edge_kind = str(row.get("planner_status", ""))
            engage_identity: tuple[int, int] | None = None
            if edge_kind in engagement_statuses:
                try:
                    engage_identity = (
                        int(row["frozen_producer_epoch"]),
                        int(row["frozen_flight_id"]),
                    )
                except (TypeError, ValueError) as exc:
                    errors.append(
                        f"Runner engage row {row_index} has invalid identity: {exc}"
                    )
            try:
                seq = int(row["planner_completion_seq"])
            except (TypeError, ValueError) as exc:
                errors.append(
                    f"Runner completion row {row_index} has invalid sequence: {exc}"
                )
                continue
            if seq < 0:
                errors.append(
                    f"Runner completion row {row_index} has negative sequence={seq}"
                )
                continue
            if seq == 0:
                if edge_kind == "engage":
                    if active_identity is not None:
                        errors.append(
                            f"Runner engage row {row_index} started {engage_identity} "
                            f"while {active_identity} remained active"
                        )
                    active_identity = engage_identity
                elif edge_kind == "preempt_engage":
                    errors.append(
                        f"Runner preempt row {row_index} has no completion edge"
                    )
                    active_identity = engage_identity
                continue
            try:
                identity = (
                    int(row["planner_completed_producer_epoch"]),
                    int(row["planner_completed_flight_id"]),
                    str(row["planner_completion_kind"]),
                )
            except (TypeError, ValueError) as exc:
                errors.append(
                    f"Runner completion row {row_index} is unparsable: {exc}"
                )
                continue
            if identity[0] <= 0 or identity[1] <= 0:
                errors.append(
                    f"Runner completion row {row_index} has invalid identity={identity[:2]}"
                )
            if identity[2] not in {"preempt", "native"}:
                errors.append(
                    f"Runner completion row {row_index} has invalid kind={identity[2]!r}"
                )
            if seq < last_seq:
                errors.append(
                    f"Runner completion sequence regressed: {last_seq} -> {seq}"
                )
                continue
            previous_identity = event_by_seq.get(seq)
            if previous_identity is not None:
                if previous_identity != identity:
                    errors.append(
                        f"Runner completion sequence {seq} changed identity from "
                        f"{previous_identity} to {identity}"
                    )
                if edge_kind == "engage":
                    if active_identity is not None:
                        errors.append(
                            f"Runner engage row {row_index} started {engage_identity} "
                            f"while {active_identity} remained active"
                        )
                    active_identity = engage_identity
                elif edge_kind == "preempt_engage":
                    errors.append(
                        f"Runner preempt row {row_index} reused completion sequence {seq}"
                    )
                    active_identity = engage_identity
                continue
            if seq != last_seq + 1:
                errors.append(
                    f"Runner completion sequence is not contiguous: {last_seq} -> {seq}"
                )
            last_seq = max(last_seq, seq)
            event_by_seq[seq] = identity
            try:
                event = {
                    "completion_seq": seq,
                    "producer_epoch": identity[0],
                    "flight_id": identity[1],
                    "completion_kind": identity[2],
                    "tick": int(row["tick"]),
                    "wall_time_ns": int(row["wall_time_ns"]),
                    "base_x": (
                        None if "base_x" not in fields else float(row["base_x"])
                    ),
                    "base_y": (
                        None if "base_y" not in fields else float(row["base_y"])
                    ),
                }
            except (TypeError, ValueError) as exc:
                errors.append(
                    f"Runner completion row {row_index} has invalid edge data: {exc}"
                )
                continue
            if event["wall_time_ns"] <= 0:
                errors.append(
                    f"Runner completion row {row_index} has non-positive wall_time_ns"
                )
            completed_identity = (identity[0], identity[1])
            if completed_identity in closed_identities:
                errors.append(
                    f"Runner completion row {row_index} closes {completed_identity} twice"
                )
            if active_identity is None:
                errors.append(
                    f"Runner completion row {row_index} closes {completed_identity} "
                    "before any active engage"
                )
            elif completed_identity != active_identity:
                errors.append(
                    f"Runner completion row {row_index} closes {completed_identity}, "
                    f"but active flight is {active_identity}"
                )
            if identity[2] == "preempt":
                if edge_kind != "preempt_engage" or engage_identity is None:
                    errors.append(
                        f"Runner preempt completion row {row_index} has no same-tick "
                        "preempt_engage identity"
                    )
                if int(row["level"]) != 1:
                    errors.append(
                        f"Runner preempt completion row {row_index} does not remain level=1"
                    )
                if engage_identity == completed_identity:
                    errors.append(
                        f"Runner preempt row {row_index} re-engages completed identity "
                        f"{completed_identity}"
                    )
                active_identity = engage_identity
            else:
                if edge_kind == "preempt_engage":
                    errors.append(
                        f"Runner native completion row {row_index} also claims preempt_engage"
                    )
                if int(row["level"]) != 0:
                    errors.append(
                        f"Runner native completion row {row_index} does not enter level=0"
                    )
                active_identity = None
            closed_identities.add(completed_identity)
            completion_events.append(event)

        completed = [event["flight_id"] for event in completion_events]
        duplicate_completed = sorted(
            flight_id
            for flight_id in set(completed)
            if completed.count(flight_id) > 1
        )
        if duplicate_completed:
            errors.append(
                "Runner has duplicate lifecycle completion edges for flight_ids: "
                f"{duplicate_completed}"
            )
        if completed != sorted(completed):
            errors.append(
                f"Runner completed flight IDs are not increasing: {completed}"
            )
        epochs = sorted({event["producer_epoch"] for event in completion_events})
        if len(epochs) > 1:
            errors.append(
                "Gate3 lifecycle evidence spans multiple producer epochs: "
                f"{epochs}"
            )
    else:
        completed = []

    completion_pass = bool(
        not require_completion_contract
        or (
            completion_available
            and not errors
            and completed == expected
            and len(completion_events) == len(expected)
        )
    )
    contact_pose_joined = [
        event["flight_id"] for event in contact_pose_events
    ]
    contact_pose_missing = sorted(
        set(requested_contact_stamps) - set(contact_pose_joined)
    )
    contact_pose_pass = bool(
        not requested_contact_stamps or not contact_pose_missing
    )
    report.update(
        {
            "engaged_flight_ids": engaged,
            "missed_flight_ids": sorted(set(expected) - set(engaged)),
            "engage_events": events,
            "engage_gaps_s": gaps,
            "completion_events": completion_events,
            "completed_flight_ids": completed,
            "incomplete_flight_ids": sorted(set(expected) - set(completed)),
            "completion_contract_pass": completion_pass,
            "physical_contact_pose_events": contact_pose_events,
            "physical_contact_pose_missing_flight_ids": contact_pose_missing,
            "physical_contact_pose_contract_pass": contact_pose_pass,
            "trace_contract_pass": not errors,
        }
    )
    return report


_ACTUAL_Q_AUDIT_ONSET_RE = re.compile(
    r"\[pp actual-q audit\] joint '([^']+)' measured q=([-+0-9.eE]+) "
    r"exceeds exported interval=\[([-+0-9.eE]+),([-+0-9.eE]+)\] "
    r"tolerance=([-+0-9.eE]+); audit-only"
)
_ACTUAL_Q_AUDIT_RECOVERY_RE = re.compile(
    r"\[pp actual-q audit\] joint '([^']+)' recovered inside the exported "
    r"hard-limit tolerance"
)
_ACTUAL_Q_FAULT_RE = re.compile(
    r"PHYSICAL SAFETY FAULT: measured q exceeds hard limit for joint '([^']+)' "
    r"\(q=([-+0-9.eE]+), interval=\[([-+0-9.eE]+),([-+0-9.eE]+)\], "
    r"tolerance=([-+0-9.eE]+)\)"
)


def _actual_q_excess(q: float, lo: float, hi: float) -> float:
    return max(lo - q, q - hi, 0.0)


def _actual_q_episode_step(
    active: MutableMapping[str, dict[str, Any]],
    events: list[dict[str, Any]],
    *,
    source: str,
    wall_time_ns: int,
    joint: str,
    q: float,
    excess: float,
    tolerance: float,
) -> None:
    violating = excess > tolerance
    event = active.get(joint)
    if violating:
        if event is None:
            event = {
                "source": source,
                "joint": joint,
                "first_wall_time_ns": wall_time_ns,
                "last_wall_time_ns": wall_time_ns,
                "violation_samples": 0,
                "q_at_onset_rad": q,
                "q_at_peak_rad": q,
                "max_excess_rad": excess,
                "tolerance_rad": tolerance,
            }
            active[joint] = event
        event["last_wall_time_ns"] = wall_time_ns
        event["violation_samples"] += 1
        if excess > event["max_excess_rad"]:
            event["max_excess_rad"] = excess
            event["q_at_peak_rad"] = q
    elif event is not None:
        event["duration_s"] = round(
            (event["last_wall_time_ns"] - event["first_wall_time_ns"])
            * 1.0e-9,
            9,
        )
        event["source_event_index"] = len(events)
        events.append(event)
        del active[joint]


def _actual_q_episode_finish(
    active: MutableMapping[str, dict[str, Any]],
    events: list[dict[str, Any]],
) -> None:
    for joint in sorted(active):
        event = active[joint]
        event["duration_s"] = round(
            (event["last_wall_time_ns"] - event["first_wall_time_ns"])
            * 1.0e-9,
            9,
        )
        event["source_event_index"] = len(events)
        events.append(event)
    active.clear()
    events.sort(key=lambda event: (event["first_wall_time_ns"], event["joint"]))
    for index, event in enumerate(events):
        event["source_event_index"] = index


def audit_actual_q_full_tail_ledger(
    runner_trace_path: str | Path,
    plant_path: str | Path,
    runner_log_path: str | Path,
    expected_flight_ids: Iterable[int],
    *,
    physical_contact_wall_time_ns_by_flight: Mapping[int, int] | None = None,
    required_post_completion_tail_s: float = 0.0,
    runner_process_exited: bool = False,
    source_join_tolerance_s: float = 0.04,
) -> dict[str, Any]:
    """Build a fail-closed measured-q ledger from Runner, trace and plant.

    The trace owns the exported hard interval and the 50 Hz policy samples.  The
    MuJoCo plant independently re-evaluates that same interval at every recorded
    simulation step. Runner stderr supplies the loss-visible onset/recovery
    protocol. The timestamped audit starts at the first expected flight engage,
    covers every flight and the final post-completion tail, and parses the full
    Runner log so an unscoped event fails closed. A Gate3 with any excursion,
    missing final-completion tail, missing source, or irreconcilable event fails.
    Physical-contact timestamps refine phase attribution when present; their
    coverage is reported independently and never substitutes for the lifecycle
    edges that define safety-ledger completeness.
    """

    expected = [int(value) for value in expected_flight_ids]
    contacts = {
        int(flight_id): int(stamp)
        for flight_id, stamp in (
            physical_contact_wall_time_ns_by_flight or {}
        ).items()
    }
    errors: list[str] = []
    report: dict[str, Any] = {
        "schema_version": 2,
        "evidence_source": "runner_trace_50hz_plant_simstep_runner_log_v2",
        "runner_trace_csv": str(runner_trace_path),
        "plant_csv": str(plant_path),
        "runner_log": str(runner_log_path),
        "expected_flight_ids": expected,
        "required_post_completion_tail_s": float(
            required_post_completion_tail_s
        ),
        "audit_window": {},
        "source_event_counts": {"runner_log": 0, "runner_trace": 0, "plant": 0},
        "source_violation_samples": {"runner_trace": 0, "plant": 0},
        "source_events": {"runner_log": [], "runner_trace": [], "plant": []},
        "canonical_events": [],
        "by_flight": {},
        "unattributed_events": [],
        "actual_q_violation_episode_count": 0,
        "actual_q_violation_sample_count": 0,
        "actual_q_max_excess_rad": 0.0,
        "physical_contact_timestamp_coverage": {},
        "source_event_count_parity_pass": False,
        "cross_source_consistency_pass": False,
        "full_tail_coverage_pass": False,
        "ledger_complete": False,
        "pass": False,
        "errors": errors,
    }
    if (
        not expected
        or expected != sorted(set(expected))
        or any(value <= 0 for value in expected)
    ):
        errors.append("expected flight IDs must be unique, positive and increasing")
        return report
    if (
        not math.isfinite(float(required_post_completion_tail_s))
        or required_post_completion_tail_s < 0.0
    ):
        errors.append("required post-completion tail must be finite and non-negative")
        return report
    if (
        not math.isfinite(float(source_join_tolerance_s))
        or source_join_tolerance_s <= 0.0
    ):
        errors.append("source join tolerance must be finite and positive")
        return report
    contact_coverage_pass = bool(
        physical_contact_wall_time_ns_by_flight is None
        or sorted(contacts) == expected
    )
    report["physical_contact_timestamp_coverage"] = {
        "requested": physical_contact_wall_time_ns_by_flight is not None,
        "required_for_ledger_complete": False,
        "observed_flight_ids": sorted(contacts),
        "missing_expected_flight_ids": sorted(set(expected) - set(contacts)),
        "unexpected_flight_ids": sorted(set(contacts) - set(expected)),
        "coverage_pass": contact_coverage_pass,
    }

    # Runner trace: parse immutable lifecycle edges and independently validate
    # every emitted measured-q excess/count against q and its exported interval.
    try:
        with open(runner_trace_path, newline="") as stream:
            reader = csv.DictReader(stream)
            trace_fields = list(reader.fieldnames or ())
            raw_trace_rows = list(reader)
    except (FileNotFoundError, OSError, csv.Error) as exc:
        errors.append(f"cannot read Runner trace CSV: {exc}")
        return report

    lo_prefix = "actual_q_hard_lo_"
    joints = [
        name[len(lo_prefix):]
        for name in trace_fields
        if name.startswith(lo_prefix)
    ]
    required_trace_fields = {
        "wall_time_ns",
        "mode",
        "level",
        "planner_status",
        "frozen_flight_id",
        "planner_completion_seq",
        "planner_completed_flight_id",
        "actual_q_hard_tolerance_rad",
        "actual_q_hard_audit_only",
        "actual_q_hard_violation_count",
        "actual_q_hard_max_excess_rad",
    }
    for joint in joints:
        required_trace_fields.update(
            {
                f"q_{joint}",
                f"actual_q_hard_lo_{joint}",
                f"actual_q_hard_hi_{joint}",
                f"actual_q_hard_excess_{joint}",
            }
        )
    missing_trace = sorted(required_trace_fields - set(trace_fields))
    if len(joints) != 31 or len(set(joints)) != 31:
        errors.append(
            "Runner trace must expose exactly 31 unique self-contained "
            "actual_q_hard_lo_* joints"
        )
    if missing_trace:
        errors.append("missing actual-q Runner trace columns: " + ", ".join(missing_trace))
    if not raw_trace_rows:
        errors.append("Runner trace is empty")
    if errors:
        return report

    trace_rows: list[dict[str, Any]] = []
    engagements: list[dict[str, int]] = []
    completions_by_flight: dict[int, int] = {}
    last_completion_seq = 0
    previous_trace_wall = 0
    for row_index, raw in enumerate(raw_trace_rows, start=2):
        try:
            wall = int(raw["wall_time_ns"])
            if wall <= previous_trace_wall:
                raise ValueError("wall_time_ns is not strictly increasing")
            previous_trace_wall = wall
            parsed = {
                "row": row_index,
                "wall_time_ns": wall,
                "mode": int(raw["mode"]),
                "level": int(raw["level"]),
                "planner_status": str(raw["planner_status"]),
                "frozen_flight_id": int(raw["frozen_flight_id"] or 0),
                "tolerance": float(raw["actual_q_hard_tolerance_rad"]),
                "audit_only": int(raw["actual_q_hard_audit_only"]),
                "declared_count": int(raw["actual_q_hard_violation_count"]),
                "declared_max": float(raw["actual_q_hard_max_excess_rad"]),
                "q": {},
                "lo": {},
                "hi": {},
                "excess": {},
            }
            for joint in joints:
                parsed["q"][joint] = float(raw[f"q_{joint}"])
                parsed["lo"][joint] = float(raw[f"actual_q_hard_lo_{joint}"])
                parsed["hi"][joint] = float(raw[f"actual_q_hard_hi_{joint}"])
                parsed["excess"][joint] = float(
                    raw[f"actual_q_hard_excess_{joint}"]
                )
            completion_seq = int(raw["planner_completion_seq"] or 0)
            completed_flight = int(raw["planner_completed_flight_id"] or 0)
        except (KeyError, TypeError, ValueError) as exc:
            errors.append(f"Runner actual-q trace row {row_index} is invalid: {exc}")
            continue
        finite_values = [
            parsed["tolerance"],
            parsed["declared_max"],
            *parsed["q"].values(),
            *parsed["lo"].values(),
            *parsed["hi"].values(),
            *parsed["excess"].values(),
        ]
        if not all(math.isfinite(value) for value in finite_values):
            errors.append(f"Runner actual-q trace row {row_index} is non-finite")
            continue
        if parsed["audit_only"] not in (0, 1):
            errors.append(
                f"Runner actual-q trace row {row_index} has invalid audit-only flag"
            )
            continue
        trace_rows.append(parsed)
        if (
            parsed["planner_status"] in {"engage", "preempt_engage"}
            and parsed["frozen_flight_id"] > 0
        ):
            engagements.append(
                {
                    "flight_id": parsed["frozen_flight_id"],
                    "wall_time_ns": wall,
                }
            )
        if completion_seq > last_completion_seq:
            if completed_flight > 0:
                completions_by_flight[completed_flight] = wall
            last_completion_seq = completion_seq
    if errors:
        return report

    # De-duplicate persistent trace snapshots defensively; the edge statuses are
    # intended to be one tick, while the immutable identity remains persistent.
    unique_engagements: list[dict[str, int]] = []
    seen_engagements: set[int] = set()
    for event in engagements:
        if event["flight_id"] not in seen_engagements:
            unique_engagements.append(event)
            seen_engagements.add(event["flight_id"])
    engagements = unique_engagements
    engaged_ids = [event["flight_id"] for event in engagements]
    if engaged_ids != expected:
        errors.append(
            f"actual-q audit engage identities {engaged_ids} do not equal expected {expected}"
        )
        return report

    missing_completions = [
        flight_id for flight_id in expected if flight_id not in completions_by_flight
    ]
    if missing_completions:
        errors.append(
            "Runner trace is missing completion edges for flights: "
            + ", ".join(str(value) for value in missing_completions)
        )
    engagement_wall_by_flight = {
        event["flight_id"]: event["wall_time_ns"] for event in engagements
    }
    for flight_id, contact_wall in contacts.items():
        engage_wall = engagement_wall_by_flight.get(flight_id)
        completion_wall = completions_by_flight.get(flight_id)
        if (
            engage_wall is None
            or completion_wall is None
            or not engage_wall <= contact_wall <= completion_wall
        ):
            errors.append(
                f"physical-contact timestamp for flight {flight_id} is outside "
                "its engage-to-completion interval"
            )
    # The qualification contract begins at the first committed shot, not while
    # Runner is still in its startup/HOLD posture.  Runner log entries have no
    # trustworthy timestamp, so they are still parsed in full below: any event
    # outside this timestamped trace/plant window remains unmatched and fails
    # the ledger closed instead of being silently discarded.
    window_start = engagements[0]["wall_time_ns"]
    window_end = trace_rows[-1]["wall_time_ns"]
    audited_trace_rows = [
        row for row in trace_rows if row["wall_time_ns"] >= window_start
    ]
    trace_gaps_ns = [
        right["wall_time_ns"] - left["wall_time_ns"]
        for left, right in zip(audited_trace_rows, audited_trace_rows[1:])
    ]
    trace_max_gap_ns = max(trace_gaps_ns, default=0)
    trace_cadence_pass = bool(
        len(audited_trace_rows) >= 2 and trace_max_gap_ns <= 100_000_000
    )
    if not trace_cadence_pass:
        errors.append(
            "Runner trace is too short or has a gap above 100 ms in the audit window"
        )
    first = audited_trace_rows[0]
    tolerance = first["tolerance"]
    audit_only = first["audit_only"]
    bounds = {
        joint: (first["lo"][joint], first["hi"][joint]) for joint in joints
    }
    if tolerance < 0.0 or any(lo >= hi for lo, hi in bounds.values()):
        errors.append("Runner trace exports an invalid actual-q interval/tolerance")
        return report

    trace_events: list[dict[str, Any]] = []
    trace_active: dict[str, dict[str, Any]] = {}
    trace_violation_samples = 0
    trace_contract_mismatch_rows: list[int] = []
    for row in audited_trace_rows:
        computed_count = 0
        computed_max = 0.0
        row_contract_ok = bool(
            abs(row["tolerance"] - tolerance) <= 1.0e-12
            and row["audit_only"] == audit_only
        )
        for joint in joints:
            lo, hi = bounds[joint]
            q = row["q"][joint]
            computed_excess = _actual_q_excess(q, lo, hi)
            emitted_excess = row["excess"][joint]
            row_contract_ok = bool(
                row_contract_ok
                and abs(row["lo"][joint] - lo) <= 1.0e-9
                and abs(row["hi"][joint] - hi) <= 1.0e-9
                and abs(emitted_excess - computed_excess) <= 2.0e-6
            )
            if computed_excess > tolerance:
                computed_count += 1
                trace_violation_samples += 1
            computed_max = max(computed_max, computed_excess)
            _actual_q_episode_step(
                trace_active,
                trace_events,
                source="runner_trace",
                wall_time_ns=row["wall_time_ns"],
                joint=joint,
                q=q,
                excess=computed_excess,
                tolerance=tolerance,
            )
        row_contract_ok = bool(
            row_contract_ok
            and row["declared_count"] == computed_count
            and abs(row["declared_max"] - computed_max) <= 2.0e-6
        )
        if not row_contract_ok and len(trace_contract_mismatch_rows) < 20:
            trace_contract_mismatch_rows.append(row["row"])
    _actual_q_episode_finish(trace_active, trace_events)
    if trace_contract_mismatch_rows:
        errors.append(
            "Runner actual-q emitted ledger disagrees with q/limits at CSV rows: "
            + ", ".join(str(value) for value in trace_contract_mismatch_rows)
        )

    # Plant: use the trace-owned immutable bounds, but recompute from independent
    # simulation-step q samples. Read streaming so a full Gate3 does not inflate
    # memory; the observed cadence is measured and reported below.
    plant_events: list[dict[str, Any]] = []
    plant_active: dict[str, dict[str, Any]] = {}
    plant_violation_samples = 0
    plant_first_wall: int | None = None
    plant_last_wall: int | None = None
    plant_first_in_window: int | None = None
    plant_last_in_window: int | None = None
    previous_plant_in_window: int | None = None
    plant_max_gap_ns = 0
    plant_rows_in_window = 0
    previous_plant_wall = 0
    try:
        with open(plant_path, newline="") as stream:
            reader = csv.DictReader(stream)
            plant_fields = set(reader.fieldnames or ())
            missing_plant = sorted(
                {"wall_time_ns", *(f"q_{joint}" for joint in joints)}
                - plant_fields
            )
            if missing_plant:
                errors.append(
                    "missing actual-q plant columns: " + ", ".join(missing_plant)
                )
            else:
                for row_index, row in enumerate(reader, start=2):
                    try:
                        wall = int(row["wall_time_ns"])
                    except (TypeError, ValueError) as exc:
                        errors.append(f"plant row {row_index} has invalid wall time: {exc}")
                        continue
                    if wall <= previous_plant_wall:
                        errors.append(
                            f"plant row {row_index} wall_time_ns is not strictly increasing"
                        )
                        continue
                    previous_plant_wall = wall
                    plant_first_wall = wall if plant_first_wall is None else plant_first_wall
                    plant_last_wall = wall
                    if wall < window_start or wall > window_end:
                        continue
                    plant_rows_in_window += 1
                    if plant_first_in_window is None:
                        plant_first_in_window = wall
                    if previous_plant_in_window is not None:
                        plant_max_gap_ns = max(
                            plant_max_gap_ns, wall - previous_plant_in_window
                        )
                    previous_plant_in_window = wall
                    plant_last_in_window = wall
                    for joint in joints:
                        try:
                            q = float(row[f"q_{joint}"])
                        except (TypeError, ValueError) as exc:
                            errors.append(
                                f"plant row {row_index} joint {joint} is invalid: {exc}"
                            )
                            continue
                        if not math.isfinite(q):
                            errors.append(
                                f"plant row {row_index} joint {joint} is non-finite"
                            )
                            continue
                        lo, hi = bounds[joint]
                        excess = _actual_q_excess(q, lo, hi)
                        if excess > tolerance:
                            plant_violation_samples += 1
                        _actual_q_episode_step(
                            plant_active,
                            plant_events,
                            source="plant",
                            wall_time_ns=wall,
                            joint=joint,
                            q=q,
                            excess=excess,
                            tolerance=tolerance,
                        )
    except (FileNotFoundError, OSError, csv.Error) as exc:
        errors.append(f"cannot read plant CSV: {exc}")
    _actual_q_episode_finish(plant_active, plant_events)

    # Runner stderr is intentionally independent of the CSV writer. It has no
    # trustworthy timestamp, so reconcile its ordered onset values to trace
    # events rather than inventing wall-clock attribution.
    runner_events: list[dict[str, Any]] = []
    runner_recovery_events: list[dict[str, Any]] = []
    runner_protocol_active: set[str] = set()
    runner_protocol_errors: list[str] = []
    try:
        with open(runner_log_path, errors="replace") as stream:
            for line_number, line in enumerate(stream, start=1):
                onset = _ACTUAL_Q_AUDIT_ONSET_RE.search(line)
                fault = _ACTUAL_Q_FAULT_RE.search(line)
                recovery = _ACTUAL_Q_AUDIT_RECOVERY_RE.search(line)
                match = onset or fault
                if match:
                    joint = match.group(1)
                    event = {
                        "source": "runner_log",
                        "source_event_index": len(runner_events),
                        "line": line_number,
                        "kind": "telemetry" if onset else "termination",
                        "joint": joint,
                        "q_at_onset_rad": float(match.group(2)),
                        "lo_rad": float(match.group(3)),
                        "hi_rad": float(match.group(4)),
                        "tolerance_rad": float(match.group(5)),
                    }
                    event["max_excess_rad"] = _actual_q_excess(
                        event["q_at_onset_rad"], event["lo_rad"], event["hi_rad"]
                    )
                    runner_events.append(event)
                    if joint in runner_protocol_active:
                        runner_protocol_errors.append(
                            "Runner log repeats onset for active joint "
                            f"{joint} at line {line_number}"
                        )
                    runner_protocol_active.add(joint)
                elif recovery:
                    joint = recovery.group(1)
                    runner_recovery_events.append(
                        {"joint": joint, "line": line_number}
                    )
                    if joint not in runner_protocol_active:
                        runner_protocol_errors.append(
                            f"Runner log recovers inactive joint {joint} at line {line_number}"
                        )
                    else:
                        runner_protocol_active.remove(joint)
    except (FileNotFoundError, OSError) as exc:
        errors.append(f"cannot read Runner log: {exc}")

    for event in runner_events:
        if event["joint"] not in bounds:
            runner_protocol_errors.append(
                f"Runner log actual-q event names unknown joint {event['joint']}"
            )
            continue
        lo, hi = bounds[event["joint"]]
        if (
            abs(event["lo_rad"] - lo) > 2.0e-6
            or abs(event["hi_rad"] - hi) > 2.0e-6
            or abs(event["tolerance_rad"] - tolerance) > 2.0e-6
            or event["max_excess_rad"] <= tolerance
        ):
            runner_protocol_errors.append(
                f"Runner log event {event['source_event_index']} disagrees with exported bounds"
            )
    if runner_protocol_errors:
        errors.extend(runner_protocol_errors)

    # Require the plant-step recorder to bracket the exact Runner audit window
    # without a silent interior hole. A 50 ms boundary/gap tolerance accommodates
    # orderly startup/teardown and scheduling jitter, never a missing recovery
    # or completion-tail interval.
    plant_coverage_slack_ns = 50_000_000
    plant_cadence_pass = bool(
        plant_rows_in_window >= 2 and plant_max_gap_ns <= plant_coverage_slack_ns
    )
    plant_coverage_pass = bool(
        plant_cadence_pass
        and plant_first_in_window is not None
        and plant_last_in_window is not None
        and plant_first_in_window <= window_start + plant_coverage_slack_ns
        and plant_last_in_window >= window_end - plant_coverage_slack_ns
    )
    final_flight = expected[-1]
    final_completion_wall = completions_by_flight.get(final_flight)
    observed_tail_s = (
        None
        if final_completion_wall is None
        else max(0.0, (window_end - final_completion_wall) * 1.0e-9)
    )
    tail_duration_pass = bool(
        observed_tail_s is not None
        and observed_tail_s + 1.0e-9 >= required_post_completion_tail_s
    )
    full_tail_coverage_pass = bool(
        runner_process_exited
        and trace_cadence_pass
        and plant_coverage_pass
        and tail_duration_pass
    )
    if final_completion_wall is None:
        errors.append(
            f"Runner trace has no final completion edge for flight {final_flight}"
        )
    if not runner_process_exited:
        errors.append("Runner process exit was not confirmed before actual-q audit")
    if not plant_coverage_pass:
        errors.append("plant CSV does not bracket the Runner actual-q audit window")
    if final_completion_wall is not None and not tail_duration_pass:
        errors.append(
            f"post-completion actual-q tail is {observed_tail_s:.3f}s, below "
            f"required {required_post_completion_tail_s:.3f}s"
        )

    # One-to-one time reconciliation. Exact zero-event parity is the normal pass
    # case; if an excursion exists, the union remains fully reported even when a
    # 50 Hz sampler missed a sub-tick plant event. Such a mismatch is retained
    # in the canonical union and fails cross-source consistency closed.
    join_ns = int(round(source_join_tolerance_s * 1.0e9))
    trace_to_plant: dict[int, int] = {}
    used_plant: set[int] = set()
    for trace_event in trace_events:
        candidates = []
        for plant_event in plant_events:
            plant_index = int(plant_event["source_event_index"])
            if plant_index in used_plant or plant_event["joint"] != trace_event["joint"]:
                continue
            if (
                plant_event["first_wall_time_ns"]
                    <= trace_event["last_wall_time_ns"] + join_ns
                and trace_event["first_wall_time_ns"]
                    <= plant_event["last_wall_time_ns"] + join_ns
            ):
                candidates.append(plant_event)
        if candidates:
            matched = min(
                candidates,
                key=lambda event: abs(
                    event["first_wall_time_ns"]
                    - trace_event["first_wall_time_ns"]
                ),
            )
            trace_index = int(trace_event["source_event_index"])
            plant_index = int(matched["source_event_index"])
            trace_to_plant[trace_index] = plant_index
            used_plant.add(plant_index)

    runner_to_trace: dict[int, int] = {}
    used_trace: set[int] = set()
    for runner_event in runner_events:
        candidate_index = None
        for trace_index in range(len(trace_events)):
            if trace_index in used_trace:
                continue
            trace_event = trace_events[trace_index]
            if trace_event["joint"] != runner_event["joint"]:
                continue
            if abs(
                trace_event["q_at_onset_rad"] - runner_event["q_at_onset_rad"]
            ) <= 2.0e-5:
                candidate_index = trace_index
                break
        if candidate_index is not None:
            runner_index = int(runner_event["source_event_index"])
            runner_to_trace[runner_index] = candidate_index
            used_trace.add(candidate_index)

    counts = {
        "runner_log": len(runner_events),
        "runner_trace": len(trace_events),
        "plant": len(plant_events),
    }
    parity_pass = len(set(counts.values())) == 1
    consistency_pass = bool(
        parity_pass
        and len(trace_to_plant) == len(trace_events) == len(plant_events)
        and len(runner_to_trace) == len(runner_events) == len(trace_events)
    )

    # Canonical union starts at the highest-rate source and retains source IDs so
    # the JSON never turns an unmatched excursion into a zero.
    canonical: list[dict[str, Any]] = []
    plant_to_canonical: dict[int, int] = {}
    for event in plant_events:
        canonical_event = dict(event)
        canonical_event.pop("source", None)
        canonical_event["sources"] = {"plant": event["source_event_index"]}
        canonical_event["canonical_event_id"] = len(canonical) + 1
        plant_to_canonical[int(event["source_event_index"])] = len(canonical)
        canonical.append(canonical_event)
    trace_to_canonical: dict[int, int] = {}
    for event in trace_events:
        trace_index = int(event["source_event_index"])
        if trace_index in trace_to_plant:
            canonical_index = plant_to_canonical[trace_to_plant[trace_index]]
            canonical[canonical_index]["sources"]["runner_trace"] = trace_index
            canonical[canonical_index]["max_excess_rad"] = max(
                canonical[canonical_index]["max_excess_rad"],
                event["max_excess_rad"],
            )
        else:
            canonical_event = dict(event)
            canonical_event.pop("source", None)
            canonical_event["sources"] = {"runner_trace": trace_index}
            canonical_event["canonical_event_id"] = len(canonical) + 1
            canonical_index = len(canonical)
            canonical.append(canonical_event)
        trace_to_canonical[trace_index] = canonical_index
    for event in runner_events:
        runner_index = int(event["source_event_index"])
        trace_index = runner_to_trace.get(runner_index)
        if trace_index is not None:
            canonical_index = trace_to_canonical[trace_index]
            canonical[canonical_index]["sources"]["runner_log"] = runner_index
        else:
            canonical_event = {
                "canonical_event_id": len(canonical) + 1,
                "joint": event["joint"],
                "first_wall_time_ns": None,
                "last_wall_time_ns": None,
                "violation_samples": 0,
                "q_at_onset_rad": event["q_at_onset_rad"],
                "q_at_peak_rad": event["q_at_onset_rad"],
                "max_excess_rad": event["max_excess_rad"],
                "tolerance_rad": event["tolerance_rad"],
                "duration_s": None,
                "sources": {"runner_log": runner_index},
            }
            canonical.append(canonical_event)

    canonical.sort(
        key=lambda event: (
            event.get("first_wall_time_ns") is None,
            event.get("first_wall_time_ns") or 0,
            event["joint"],
        )
    )
    for index, event in enumerate(canonical, start=1):
        event["canonical_event_id"] = index

    engagement_by_time = sorted(
        (event["wall_time_ns"], event["flight_id"]) for event in engagements
    )
    by_flight: dict[str, Any] = {
        str(flight_id): {
            "flight_id": flight_id,
            "actual_q_violation_episode_count": 0,
            "by_phase": {},
            "by_joint": {},
            "canonical_event_ids": [],
        }
        for flight_id in expected
    }
    unattributed: list[int] = []
    for event in canonical:
        stamp = event.get("first_wall_time_ns")
        flight_id = 0
        if stamp is not None:
            for engage_wall, candidate_flight in engagement_by_time:
                if engage_wall > stamp:
                    break
                flight_id = candidate_flight
        event["flight_id"] = flight_id or None
        if flight_id == 0:
            event["phase"] = "unattributed"
            unattributed.append(int(event["canonical_event_id"]))
            continue
        contact_wall = contacts.get(flight_id)
        completion_wall = completions_by_flight.get(flight_id)
        if contact_wall is not None and stamp < contact_wall:
            phase = "engage_to_contact"
        elif completion_wall is not None and stamp < completion_wall:
            phase = (
                "contact_to_completion"
                if contact_wall is not None
                else "engage_to_completion_no_physical_contact"
            )
        else:
            phase = "post_completion_tail"
        event["phase"] = phase
        flight = by_flight[str(flight_id)]
        flight["actual_q_violation_episode_count"] += 1
        flight["by_phase"][phase] = flight["by_phase"].get(phase, 0) + 1
        joint = str(event["joint"])
        flight["by_joint"][joint] = flight["by_joint"].get(joint, 0) + 1
        flight["canonical_event_ids"].append(event["canonical_event_id"])

    actual_q_count = len(canonical)
    actual_q_samples = max(trace_violation_samples, plant_violation_samples)
    actual_q_max = max(
        (float(event["max_excess_rad"]) for event in canonical), default=0.0
    )
    ledger_complete = bool(
        not errors
        and full_tail_coverage_pass
        and consistency_pass
        and not unattributed
    )
    report.update(
        {
            "joint_count": len(joints),
            "joint_names": joints,
            "actual_q_hard_tolerance_rad": tolerance,
            "actual_q_hard_audit_only": bool(audit_only),
            "audit_window": {
                "start_wall_time_ns": window_start,
                "start_reason": "first_expected_flight_engage",
                "end_wall_time_ns": window_end,
                "end_reason": "runner_trace_eof_after_process_exit",
                "runner_process_exited": bool(runner_process_exited),
                "final_flight_id": final_flight,
                "final_completion_wall_time_ns": final_completion_wall,
                "observed_post_completion_tail_s": observed_tail_s,
                "trace_rows": len(audited_trace_rows),
                "trace_max_gap_s": trace_max_gap_ns * 1.0e-9,
                "trace_effective_hz": (
                    (len(audited_trace_rows) - 1) * 1.0e9
                    / (window_end - window_start)
                    if len(audited_trace_rows) >= 2 and window_end > window_start
                    else 0.0
                ),
                "trace_cadence_pass": trace_cadence_pass,
                "plant_rows": plant_rows_in_window,
                "plant_first_wall_time_ns": plant_first_wall,
                "plant_last_wall_time_ns": plant_last_wall,
                "plant_first_in_window_wall_time_ns": plant_first_in_window,
                "plant_last_in_window_wall_time_ns": plant_last_in_window,
                "plant_max_gap_s": plant_max_gap_ns * 1.0e-9,
                "plant_effective_hz": (
                    (plant_rows_in_window - 1) * 1.0e9
                    / (plant_last_in_window - plant_first_in_window)
                    if plant_rows_in_window >= 2
                    and plant_first_in_window is not None
                    and plant_last_in_window is not None
                    and plant_last_in_window > plant_first_in_window
                    else 0.0
                ),
                "plant_cadence_pass": plant_cadence_pass,
                "plant_coverage_pass": plant_coverage_pass,
            },
            "runner_log_recovery_events": runner_recovery_events,
            "runner_log_unclosed_joints": sorted(runner_protocol_active),
            "source_event_counts": counts,
            "source_violation_samples": {
                "runner_trace": trace_violation_samples,
                "plant": plant_violation_samples,
            },
            "source_events": {
                "runner_log": runner_events,
                "runner_trace": trace_events,
                "plant": plant_events,
            },
            "source_joins": {
                "runner_log_to_runner_trace": runner_to_trace,
                "runner_trace_to_plant": trace_to_plant,
                "join_tolerance_s": source_join_tolerance_s,
            },
            "canonical_events": canonical,
            "by_flight": by_flight,
            "unattributed_events": unattributed,
            "actual_q_violation_episode_count": actual_q_count,
            "actual_q_violation_sample_count": actual_q_samples,
            "actual_q_max_excess_rad": actual_q_max,
            "source_event_count_parity_pass": parity_pass,
            "cross_source_consistency_pass": consistency_pass,
            "full_tail_coverage_pass": full_tail_coverage_pass,
            "ledger_complete": ledger_complete,
            "pass": bool(ledger_complete and actual_q_count == 0),
        }
    )
    return report


@dataclass(frozen=True)
class ServeSpec:
    """One side-neutral initial state expressed in the table-surface frame."""

    position: tuple[float, float, float]
    velocity: tuple[float, float, float]

    def world_position(self, table_height_m: float = TABLE_HEIGHT_M) -> tuple[float, float, float]:
        return (
            self.position[0],
            self.position[1],
            self.position[2] + float(table_height_m),
        )


def _finite_float(value: Any, label: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite, got {value!r}")
    return result


def parse_serves_list(value: str | Sequence[float]) -> list[ServeSpec]:
    """Parse a flat ``6*N`` list without accepting side labels or dictionaries."""
    parsed: Any = ast.literal_eval(value) if isinstance(value, str) else value
    if not isinstance(parsed, (list, tuple)):
        raise ValueError("Gate3 serves must be a flat numeric list")
    if any(isinstance(item, (dict, list, tuple)) for item in parsed):
        raise ValueError(
            "Gate3 serves may contain only initial p/v numbers; side labels are forbidden"
        )
    flat = [_finite_float(item, f"serves[{index}]") for index, item in enumerate(parsed)]
    if not flat or len(flat) % 6:
        raise ValueError(
            f"Gate3 serves must contain exactly 6*N numbers, got {len(flat)}"
        )
    return [
        ServeSpec(tuple(flat[index : index + 3]), tuple(flat[index + 3 : index + 6]))
        for index in range(0, len(flat), 6)
    ]


def generate_v17_r10_random_serves(count: int, seed: int) -> list[ServeSpec]:
    """Build a reproducible, balanced, side-neutral fixed-station Gate3 sweep.

    The two lateral lanes sit inside the exported R10 FH/BH strike-position
    support when the immutable session anchor is ``y=-0.7625``.  We perturb
    launch position and velocity, but never encode a side in the wire format;
    the production planner still owns side selection from the measured path.
    Consecutive pairs contain one sample from each lane so a long test cannot
    accidentally become a one-sided draw.
    """
    if int(count) != count or count < 8:
        raise ValueError("V17-r10 random Gate3 requires at least 8 serves")
    rng = random.Random(int(seed))
    result: list[ServeSpec] = []
    # With station_y=-0.7625 these centers map to R10 reach-y -0.44/-0.09.
    lane_centers = (-1.2025, -0.8525)
    for pair_index in range((int(count) + 1) // 2):
        lane_order = [0, 1]
        if rng.random() < 0.5:
            lane_order.reverse()
        for lane in lane_order:
            if len(result) >= int(count):
                break
            # Keep every draw within the training position support while
            # varying the incoming trajectory enough to expose brittle timing.
            x = rng.uniform(2.36, 2.44)
            y = lane_centers[lane] + rng.uniform(-0.030, 0.030)
            z = rng.uniform(0.485, 0.510)
            vx = rng.uniform(-3.12, -2.96)
            vy = rng.uniform(-0.035, 0.035)
            vz = rng.uniform(2.08, 2.28)
            result.append(ServeSpec((x, y, z), (vx, vy, vz)))
    return result


def serves_to_flat_list(serves: Sequence[ServeSpec]) -> list[float]:
    """Serialize side-neutral serve specs to the existing flat ``6*N`` wire."""
    return [
        value
        for spec in serves
        for value in (*spec.position, *spec.velocity)
    ]


def table_to_world_position(
    position: Sequence[float], table_height_m: float = TABLE_HEIGHT_M
) -> tuple[float, float, float]:
    if len(position) != 3:
        raise ValueError("position must have three values")
    values = tuple(_finite_float(value, "position") for value in position)
    return values[0], values[1], values[2] + float(table_height_m)


def world_to_table_position(
    position: Sequence[float], table_height_m: float = TABLE_HEIGHT_M
) -> tuple[float, float, float]:
    if len(position) != 3:
        raise ValueError("position must have three values")
    values = tuple(_finite_float(value, "position") for value in position)
    return values[0], values[1], values[2] - float(table_height_m)


def _normalize_quaternion_wxyz(
    quaternion_wxyz: Sequence[float], label: str
) -> tuple[float, float, float, float]:
    values = tuple(_finite_float(value, label) for value in quaternion_wxyz)
    if len(values) != 4:
        raise ValueError(f"{label} quaternion must contain four values")
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 0.5 or norm > 1.5:
        raise ValueError(f"{label} quaternion norm is outside [0.5,1.5]")
    return tuple(value / norm for value in values)


def _quat_mul_wxyz(
    lhs: Sequence[float], rhs: Sequence[float]
) -> tuple[float, float, float, float]:
    aw, ax, ay, az = lhs
    bw, bx, by, bz = rhs
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _quat_rotate_wxyz(
    quaternion_wxyz: Sequence[float], vector_xyz: Sequence[float]
) -> tuple[float, float, float]:
    qw, qx, qy, qz = quaternion_wxyz
    vx, vy, vz = vector_xyz
    return (
        (1.0 - 2.0 * (qy * qy + qz * qz)) * vx
        + 2.0 * (qx * qy - qw * qz) * vy
        + 2.0 * (qx * qz + qw * qy) * vz,
        2.0 * (qx * qy + qw * qz) * vx
        + (1.0 - 2.0 * (qx * qx + qz * qz)) * vy
        + 2.0 * (qy * qz - qw * qx) * vz,
        2.0 * (qx * qz - qw * qy) * vx
        + 2.0 * (qy * qz + qw * qx) * vy
        + (1.0 - 2.0 * (qx * qx + qy * qy)) * vz,
    )


def base_pose_to_marker_pose(
    base_position_xyz: Sequence[float],
    base_quaternion_wxyz: Sequence[float],
    marker_to_base_xyz: Sequence[float],
    marker_to_base_quaternion_wxyz: Sequence[float],
) -> tuple[
    tuple[float, float, float],
    tuple[float, float, float, float],
]:
    """Invert ``T_world_base = T_world_marker * T_marker_base``.

    Gate3's MuJoCo source reports the pelvis pose, while the production
    OptiTrack boundary reports the UCB_P1 rigid-body marker. Publishing the
    inverted marker pose makes the exact calibrated production relay reconstruct
    the original MuJoCo pelvis pose instead of relying on an identity shortcut.
    """
    base_position = tuple(
        _finite_float(value, "base_position") for value in base_position_xyz
    )
    marker_to_base = tuple(
        _finite_float(value, "marker_to_base_xyz")
        for value in marker_to_base_xyz
    )
    if len(base_position) != 3 or len(marker_to_base) != 3:
        raise ValueError("base position and marker-to-base translation must be 3-D")
    q_world_base = _normalize_quaternion_wxyz(
        base_quaternion_wxyz, "base"
    )
    q_marker_base = _normalize_quaternion_wxyz(
        marker_to_base_quaternion_wxyz, "marker-to-base"
    )
    q_base_marker = (
        q_marker_base[0],
        -q_marker_base[1],
        -q_marker_base[2],
        -q_marker_base[3],
    )
    q_world_marker = _normalize_quaternion_wxyz(
        _quat_mul_wxyz(q_world_base, q_base_marker), "world-to-marker"
    )
    world_offset = _quat_rotate_wxyz(q_world_marker, marker_to_base)
    marker_position = tuple(
        base_position[index] - world_offset[index] for index in range(3)
    )
    return marker_position, q_world_marker


def calibrated_p1_marker_contract(
    config: Mapping[str, Any],
) -> tuple[
    tuple[float, float, float],
    tuple[float, float, float, float],
]:
    """Resolve a calibrated UCB_P1 marker contract or fail closed."""
    world = config.get("hope_world", config)
    if not isinstance(world, Mapping):
        raise ValueError("hope_world configuration must be a mapping")
    contract = world.get("contract")
    offsets = world.get("mocap_to_base_link")
    if not isinstance(contract, Mapping) or not isinstance(offsets, Mapping):
        raise ValueError("hope_world contract/mocap_to_base_link is missing")
    p1 = offsets.get("p1")
    if not isinstance(p1, Mapping):
        raise ValueError("hope_world UCB_P1 marker contract is missing")
    if contract.get("venue_calibrated") is not True:
        raise ValueError("Motive world frame is not calibrated")
    if p1.get("calibrated") is not True:
        raise ValueError("UCB_P1 marker-to-base transform is not calibrated")
    for value, label in (
        (contract.get("calibration_sha256", ""), "world-frame receipt"),
        (p1.get("calibration_sha256", ""), "UCB_P1 marker receipt"),
    ):
        receipt = str(value)
        if len(receipt) != 64 or any(
            character not in "0123456789abcdef" for character in receipt
        ):
            raise ValueError(f"{label} must be a lowercase SHA256")
    translation = tuple(
        _finite_float(value, "UCB_P1 marker translation")
        for value in p1.get("xyz_m", ())
    )
    if len(translation) != 3:
        raise ValueError("UCB_P1 marker translation must contain three values")
    quaternion = _normalize_quaternion_wxyz(
        p1.get("quaternion_wxyz", ()), "UCB_P1 marker-to-base"
    )
    return translation, quaternion


def select_swing_side(
    intercept_y: float,
    base_y: float,
    previous_side: str | None,
    split_y: float = -0.25,
    hysteresis_y: float = 0.04,
) -> str:
    """Mirror the production planner's latched FH/BH split."""
    rel_y = float(intercept_y) - float(base_y)
    lo = float(split_y) - float(hysteresis_y)
    hi = float(split_y) + float(hysteresis_y)
    if previous_side == "forehand":
        return "backhand" if rel_y > hi else "forehand"
    if previous_side == "backhand":
        return "forehand" if rel_y < lo else "backhand"
    return "forehand" if rel_y < float(split_y) else "backhand"


@dataclass
class _Shot:
    shot_id: int
    samples: int = 0
    active_seen: bool = False
    inactive_seen: bool = False
    first_stamp_ns: int | None = None
    last_stamp_ns: int | None = None
    first_inactive_stamp_ns: int | None = None
    max_gap_s: float = 0.0
    counter_monotonic: bool = True
    counter_jump: bool = False
    last_racket_count: int = 0
    last_table_count: int = 0
    last_net_count: int = 0
    racket_events: list[dict[str, Any]] = field(default_factory=list)
    incoming_table_events: list[dict[str, Any]] = field(default_factory=list)
    post_racket_table_events: list[dict[str, Any]] = field(default_factory=list)
    incoming_net_events: list[dict[str, Any]] = field(default_factory=list)
    post_racket_net_events: list[dict[str, Any]] = field(default_factory=list)
    peak_racket_force_n: float = 0.0
    terminal_event: dict[str, Any] | None = None
    last_active_position: tuple[float, ...] | None = None
    last_active_speed: float = 0.0
    table_rest_since_ns: int | None = None


class PhysicalEvidenceAccumulator:
    """Accumulate loss-detectable plant contact and landing evidence by shot."""

    def __init__(
        self,
        expected_shot_ids: Iterable[int],
        *,
        min_samples: int = 20,
        max_sample_gap_s: float = 0.050,
        min_post_contact_landing_observation_s: float = (
            DEFAULT_MIN_POST_CONTACT_LANDING_OBSERVATION_S
        ),
    ) -> None:
        expected = [int(value) for value in expected_shot_ids]
        if not expected or any(value <= 0 for value in expected):
            raise ValueError("expected shot IDs must be positive")
        if len(set(expected)) != len(expected):
            raise ValueError("expected shot IDs must be unique")
        self.expected_shot_ids = expected
        self.min_samples = int(min_samples)
        self.max_sample_gap_s = float(max_sample_gap_s)
        self.min_post_contact_landing_observation_s = float(
            min_post_contact_landing_observation_s
        )
        if not math.isfinite(self.min_post_contact_landing_observation_s) or (
            self.min_post_contact_landing_observation_s <= 0.0
        ):
            raise ValueError(
                "min post-contact landing observation must be finite and positive"
            )
        self._shots: dict[int, _Shot] = {}
        self.unexpected_shot_ids: set[int] = set()

    @staticmethod
    def _event(
        stamp_ns: int,
        position: Sequence[float],
        velocity: Sequence[float],
        count: int,
        observation_lag_s: float,
    ) -> dict[str, Any]:
        return {
            "stamp_ns": int(stamp_ns),
            "position_world": [float(value) for value in position],
            "position_table": list(world_to_table_position(position)),
            "velocity_world": [float(value) for value in velocity],
            "count": int(count),
            "edge_observation_lag_s": float(observation_lag_s),
        }

    def ingest(
        self,
        *,
        stamp_ns: int,
        shot_id: int,
        active: bool,
        position: Sequence[float],
        velocity: Sequence[float],
        racket_contact_count: int,
        table_contact_count: int,
        net_contact_count: int,
        racket_normal_force_n: float = 0.0,
    ) -> None:
        shot_id = int(shot_id)
        if shot_id <= 0:
            return
        if shot_id not in self.expected_shot_ids:
            self.unexpected_shot_ids.add(shot_id)
        shot = self._shots.setdefault(shot_id, _Shot(shot_id=shot_id))
        stamp_ns = int(stamp_ns)
        observation_lag_s = 0.0
        if shot.last_stamp_ns is not None:
            if stamp_ns <= shot.last_stamp_ns:
                shot.counter_monotonic = False
            else:
                observation_lag_s = (
                    stamp_ns - shot.last_stamp_ns
                ) * 1.0e-9
                shot.max_gap_s = max(
                    shot.max_gap_s, observation_lag_s
                )
        shot.first_stamp_ns = stamp_ns if shot.first_stamp_ns is None else shot.first_stamp_ns
        shot.last_stamp_ns = stamp_ns
        shot.samples += 1
        shot.active_seen |= bool(active)
        shot.inactive_seen |= not bool(active)
        if not active and shot.first_inactive_stamp_ns is None:
            shot.first_inactive_stamp_ns = stamp_ns
        shot.peak_racket_force_n = max(
            shot.peak_racket_force_n, max(0.0, float(racket_normal_force_n))
        )
        x, y, z = (float(v) for v in position)
        vx, vy, vz = (float(v) for v in velocity)
        table_rest = (table_contact_count > 0 and
                      0 <= x <= TABLE_LENGTH_M and
                      TABLE_Y_MAX_M - TABLE_WIDTH_M <= y <= TABLE_Y_MAX_M and
                      abs(z - TABLE_HEIGHT_M - BALL_RADIUS_M) <= .001 and
                      math.hypot(vx, vy) <= .002 and abs(vz) <= .12)
        if not table_rest or observation_lag_s > self.max_sample_gap_s:
            shot.table_rest_since_ns = None
        elif shot.table_rest_since_ns is None:
            shot.table_rest_since_ns = stamp_ns
        # Independently verify a natural endpoint, including a ball that has
        # come to rest on the table. Never accept a launcher park/teleport.
        if not active:
            if shot.terminal_event is None and shot.last_active_position is not None:
                displacement = math.dist(position, shot.last_active_position)
                if (-0.10 <= float(position[2]) <= BALL_RADIUS_M
                        and float(velocity[2]) < 0.0
                        and displacement <= max(0.05, 2.0 * shot.last_active_speed * observation_lag_s)):
                    shot.terminal_event = self._event(stamp_ns, position, velocity, 0, observation_lag_s)
                    shot.terminal_event["kind"] = "ground_crossing"
                elif (shot.table_rest_since_ns is not None and
                      stamp_ns - shot.table_rest_since_ns >= 1_950_000_000 and
                      displacement <= .004):
                    # 50 ms accounts only for 250 Hz observation phase and
                    # transport jitter around the simulator's 2 s dwell.
                    shot.terminal_event = self._event(stamp_ns, position, velocity, 0, observation_lag_s)
                    shot.terminal_event["kind"] = "settled_on_table"
            return
        shot.last_active_position = tuple(float(v) for v in position)
        shot.last_active_speed = math.sqrt(sum(float(v) ** 2 for v in velocity))

        counts = (
            int(racket_contact_count),
            int(table_contact_count),
            int(net_contact_count),
        )
        previous = (
            shot.last_racket_count,
            shot.last_table_count,
            shot.last_net_count,
        )
        if any(current < old for current, old in zip(counts, previous)):
            shot.counter_monotonic = False
        if any(current - old > 1 for current, old in zip(counts, previous)):
            shot.counter_jump = True

        racket_delta = max(0, counts[0] - previous[0])
        table_delta = max(0, counts[1] - previous[1])
        net_delta = max(0, counts[2] - previous[2])
        had_racket = previous[0] > 0
        event = self._event(
            stamp_ns, position, velocity, 0, observation_lag_s
        )
        for offset in range(racket_delta):
            item = dict(event)
            item["count"] = previous[0] + offset + 1
            shot.racket_events.append(item)
        for offset in range(table_delta):
            item = dict(event)
            item["count"] = previous[1] + offset + 1
            (
                shot.post_racket_table_events
                if had_racket or racket_delta
                else shot.incoming_table_events
            ).append(item)
        for offset in range(net_delta):
            item = dict(event)
            item["count"] = previous[2] + offset + 1
            (
                shot.post_racket_net_events
                if had_racket or racket_delta
                else shot.incoming_net_events
            ).append(item)

        shot.last_racket_count, shot.last_table_count, shot.last_net_count = counts

    @staticmethod
    def _legal_table_top_event(
        event: Mapping[str, Any] | None,
        *,
        x_min: float,
        x_max: float,
        expected_vx_sign: int,
    ) -> bool:
        if event is None:
            return False
        x, y, z = (float(value) for value in event["position_world"])
        vx, _, vz = (float(value) for value in event["velocity_world"])
        observation_lag_s = float(
            event.get("edge_observation_lag_s", 0.0)
        )
        if not (
            0.0
            <= observation_lag_s
            <= CONTACT_EDGE_MAX_OBSERVATION_LAG_S
        ):
            return False
        top_center_z = TABLE_HEIGHT_M + BALL_RADIUS_M + 1.0e-4
        # The contact occurred somewhere since the prior sample.  The edge
        # sample is post-impulse, so its center can already be above the table
        # by roughly |vz|*lag.  The fixed 5 mm term covers contact softness;
        # the dynamic term accounts only for the measured sample interval.
        z_tolerance = (
            0.005
            + abs(vz) * observation_lag_s
            + 0.5 * 9.81 * observation_lag_s * observation_lag_s
        )
        return bool(
            x_min <= x <= x_max
            and TABLE_Y_MAX_M - TABLE_WIDTH_M + BALL_RADIUS_M
            <= y
            <= TABLE_Y_MAX_M - BALL_RADIUS_M
            and abs(z - top_center_z) <= z_tolerance
            and vz > 0.0
            and expected_vx_sign * vx > 0.0
        )

    @classmethod
    def _legal_incoming_bounce(cls, event: Mapping[str, Any] | None) -> bool:
        return cls._legal_table_top_event(
            event,
            x_min=BALL_RADIUS_M,
            x_max=NET_X_M - BALL_RADIUS_M,
            expected_vx_sign=-1,
        )

    @classmethod
    def _legal_landing(cls, event: Mapping[str, Any] | None) -> bool:
        return cls._legal_table_top_event(
            event,
            x_min=NET_X_M + BALL_RADIUS_M,
            x_max=TABLE_LENGTH_M - BALL_RADIUS_M,
            expected_vx_sign=1,
        )

    def report(self) -> dict[str, Any]:
        rows: list[dict[str, Any]] = []
        for shot_id in self.expected_shot_ids:
            shot = self._shots.get(shot_id, _Shot(shot_id=shot_id))
            landing = (
                shot.post_racket_table_events[0]
                if shot.post_racket_table_events
                else None
            )
            telemetry_complete = bool(
                shot.samples >= self.min_samples
                and shot.active_seen
                and shot.inactive_seen
                and shot.terminal_event is not None
                and shot.counter_monotonic
                and not shot.counter_jump
                and shot.max_gap_s <= self.max_sample_gap_s
            )
            incoming_bounce = (
                shot.incoming_table_events[0]
                if len(shot.incoming_table_events) == 1
                else None
            )
            incoming_bounce_pass = self._legal_incoming_bounce(incoming_bounce)
            contact_pass = bool(
                telemetry_complete
                and len(shot.incoming_table_events) == 1
                and incoming_bounce_pass
                and len(shot.incoming_net_events) == 0
                and len(shot.racket_events) == 1
                and float(shot.racket_events[0]["velocity_world"][0]) > 0.0
            )
            contact_stamp_ns = (
                int(shot.racket_events[0]["stamp_ns"])
                if len(shot.racket_events) == 1
                else None
            )
            post_contact_observation_s = (
                0.0
                if contact_stamp_ns is None or shot.last_stamp_ns is None
                else max(0.0, (shot.last_stamp_ns - contact_stamp_ns) * 1.0e-9)
            )
            landing_event_observed = bool(
                landing is not None or shot.post_racket_net_events
            )
            landing_observed = bool(contact_pass and shot.terminal_event is not None)
            landing_censored = bool(shot.racket_events and shot.terminal_event is None)
            landing_pass = bool(
                contact_pass
                and landing_observed
                and landing is not None
                and not shot.post_racket_net_events
                and self._legal_landing(landing)
            )
            if not contact_pass:
                landing_observation_status = "not_eligible_contact_failure"
            elif landing_censored:
                landing_observation_status = "censored_external_park"
            elif landing_event_observed:
                landing_observation_status = "observed_event"
            else:
                landing_observation_status = "observed_ground_no_landing"
            rows.append(
                {
                    "shot_id": shot_id,
                    "samples": shot.samples,
                    "first_stamp_ns": shot.first_stamp_ns,
                    "last_stamp_ns": shot.last_stamp_ns,
                    "first_inactive_stamp_ns": shot.first_inactive_stamp_ns,
                    "max_sample_gap_s": shot.max_gap_s,
                    "telemetry_complete": telemetry_complete,
                    "trajectory_complete": telemetry_complete,
                    "terminal_event": shot.terminal_event,
                    "counter_monotonic": shot.counter_monotonic,
                    "counter_jump": shot.counter_jump,
                    "incoming_table_events": shot.incoming_table_events,
                    "incoming_bounce_pass": incoming_bounce_pass,
                    "incoming_net_events": shot.incoming_net_events,
                    "racket_events": shot.racket_events,
                    "post_racket_table_events": shot.post_racket_table_events,
                    "post_racket_net_events": shot.post_racket_net_events,
                    "racket_contact_count": len(shot.racket_events),
                    "peak_racket_force_n": shot.peak_racket_force_n,
                    "landing_event": landing,
                    "contact_pass": contact_pass,
                    "post_contact_observation_s": post_contact_observation_s,
                    "minimum_post_contact_landing_observation_s": (
                        self.min_post_contact_landing_observation_s
                    ),
                    "landing_event_observed": landing_event_observed,
                    "landing_observed": landing_observed,
                    "landing_censored": landing_censored,
                    "landing_observation_status": landing_observation_status,
                    "landing_pass": landing_pass,
                }
            )
        measured = bool(
            rows
            and all(row["telemetry_complete"] for row in rows)
            and not self.unexpected_shot_ids
        )
        return {
            "schema_version": 3,
            "ball_lifecycle_contract": "continuous_flight_to_natural_rest_v2",
            "trajectory_complete_count": sum(row["trajectory_complete"] for row in rows),
            "trajectory_incomplete_shot_ids": [row["shot_id"] for row in rows if not row["trajectory_complete"]],
            "source": "mujoco_1khz_contact_edges",
            "expected_shot_ids": self.expected_shot_ids,
            "unexpected_shot_ids": sorted(self.unexpected_shot_ids),
            "min_samples_per_shot": self.min_samples,
            "max_sample_gap_limit_s": self.max_sample_gap_s,
            "min_post_contact_landing_observation_s": (
                self.min_post_contact_landing_observation_s
            ),
            "physical_contact_measured": measured,
            "landing_measured": bool(
                measured
                and any(row["landing_observed"] for row in rows)
                and all(
                    not row["contact_pass"]
                    or row["landing_observed"]
                    for row in rows
                )
            ),
            "landing_observation_count": sum(
                bool(row["landing_observed"]) for row in rows
            ),
            "landing_censored_count": sum(
                bool(row["landing_censored"]) for row in rows
            ),
            "physical_contact_pass": bool(
                measured and all(row["contact_pass"] for row in rows)
            ),
            "landing_pass": bool(
                measured
                and any(row["landing_observed"] for row in rows)
                and all(
                    row["landing_pass"]
                    for row in rows
                    if row["landing_observed"]
                )
            ),
            "rows": rows,
        }


def join_physical_evidence_by_side(
    serve_rows: Sequence[Mapping[str, Any]],
    physical_report: Mapping[str, Any],
    *,
    min_samples_per_side: int,
    min_contact_rate: float,
    min_landing_rate: float,
    exact_samples_per_side: int | None = None,
    min_contacts_per_side: int = 0,
    min_landings_per_side: int = 0,
    min_landing_observations_per_side: int | None = None,
    min_global_contacts: int = 0,
    min_global_landings: int = 0,
    min_global_landing_observations: int = 0,
    allowed_landing_censor_lanes: Iterable[str] = (),
) -> dict[str, Any]:
    """Join measured outcomes to the planner-selected side; never average sides.

    Contact uses every launched shot as its denominator.  Landing uses only
    shots whose post-contact outcome was actually observed.  A right-censored
    landing is accepted only in an explicitly named lane; it never becomes a
    miss and never enters the landing-rate denominator.
    """
    physical_rows = list(physical_report.get("rows", []))
    physical_row_ids = [int(row.get("shot_id") or 0) for row in physical_rows]
    physical_by_id = {
        int(row["shot_id"]): row for row in physical_rows
        if int(row.get("shot_id") or 0) > 0
    }
    serve_ids = [int(row.get("shot_id") or 0) for row in serve_rows]
    duplicate_serve_ids = sorted({
        shot_id for shot_id in serve_ids
        if shot_id > 0 and serve_ids.count(shot_id) > 1
    })
    duplicate_physical_ids = sorted({
        shot_id for shot_id in physical_row_ids
        if shot_id > 0 and physical_row_ids.count(shot_id) > 1
    })
    extra_physical_ids = sorted(set(physical_by_id) - set(serve_ids))
    min_landing_observations_per_side = int(
        min_samples_per_side
        if min_landing_observations_per_side is None
        else min_landing_observations_per_side
    )
    # Kept as an API argument for old callers; no Gate3 lane permits truncation.
    allowed_censor_lanes: set[str] = set()
    observation_horizon_s = float(
        physical_report.get(
            "min_post_contact_landing_observation_s",
            DEFAULT_MIN_POST_CONTACT_LANDING_OBSERVATION_S,
        )
    )

    def landing_observation(physical: Mapping[str, Any]) -> tuple[bool, bool]:
        """Return (observed, censored), including schema-1 report migration."""
        if "landing_observed" in physical or "landing_censored" in physical:
            observed = bool(physical.get("landing_observed", False))
            censored = bool(physical.get("landing_censored", False))
            return observed, censored
        if not bool(physical.get("contact_pass", False)):
            return False, False
        event_observed = bool(
            physical.get("landing_event") is not None
            or physical.get("post_racket_net_events")
        )
        racket_events = list(physical.get("racket_events", []))
        last_stamp_ns = physical.get("last_stamp_ns")
        post_contact_s = 0.0
        if len(racket_events) == 1 and last_stamp_ns is not None:
            contact_stamp_ns = racket_events[0].get("stamp_ns")
            if contact_stamp_ns is not None:
                post_contact_s = max(
                    0.0,
                    (int(last_stamp_ns) - int(contact_stamp_ns)) * 1.0e-9,
                )
        observed = bool(event_observed or post_contact_s >= observation_horizon_s)
        return observed, not observed

    per_side: dict[str, dict[str, Any]] = {}
    missing_shot_ids: list[int] = []
    unassigned_shot_ids: list[int] = []
    unexpected_censored_shot_ids: list[int] = []
    all_telemetry_complete = True
    for side in ("forehand", "backhand"):
        side_rows = [
            row for row in serve_rows if row.get("command_side") == side
        ]
        joined: list[tuple[Mapping[str, Any], Mapping[str, Any]]] = []
        for row in side_rows:
            shot_id = int(row.get("shot_id") or 0)
            physical = physical_by_id.get(shot_id)
            if physical is None:
                missing_shot_ids.append(shot_id)
                all_telemetry_complete = False
                continue
            all_telemetry_complete &= bool(physical.get("telemetry_complete", False))
            joined.append((row, physical))
        total = len(side_rows)
        contact_count = sum(
            bool(physical.get("contact_pass", False)) for _, physical in joined
        )
        landing_observed_ids: list[int] = []
        landing_censored_ids: list[int] = []
        landing_count = 0
        for serve, physical in joined:
            observed, censored = landing_observation(physical)
            shot_id = int(serve.get("shot_id") or 0)
            if observed:
                landing_observed_ids.append(shot_id)
                landing_count += int(bool(physical.get("landing_pass", False)))
            elif censored:
                landing_censored_ids.append(shot_id)
                if str(serve.get("lane", "unstratified")) not in allowed_censor_lanes:
                    unexpected_censored_shot_ids.append(shot_id)
        landing_observation_count = len(landing_observed_ids)
        contact_rate = contact_count / total if total else 0.0
        landing_rate = (
            landing_count / landing_observation_count
            if landing_observation_count else 0.0
        )
        passed = bool(
            total >= int(min_samples_per_side)
            and (
                exact_samples_per_side is None
                or total == int(exact_samples_per_side)
            )
            and len(joined) == total
            and contact_count >= int(min_contacts_per_side)
            and landing_observation_count
            >= int(min_landing_observations_per_side)
            and landing_count >= int(min_landings_per_side)
            and contact_rate >= float(min_contact_rate)
            and landing_rate >= float(min_landing_rate)
            and not any(
                shot_id in unexpected_censored_shot_ids
                for shot_id in landing_censored_ids
            )
        )
        per_side[side] = {
            "shots": total,
            "shot_ids": [int(row.get("shot_id") or 0) for row in side_rows],
            "contacts": contact_count,
            "landing_observations": landing_observation_count,
            "landing_observed_shot_ids": landing_observed_ids,
            "landing_censored": len(landing_censored_ids),
            "landing_censored_shot_ids": landing_censored_ids,
            "legal_landings": landing_count,
            "contact_rate": contact_rate,
            "landing_rate": landing_rate,
            "pass": passed,
        }

    for row in serve_rows:
        if row.get("command_side") not in ("forehand", "backhand"):
            unassigned_shot_ids.append(int(row.get("shot_id") or 0))
    expected_ids = serve_ids
    all_shots_joined = bool(
        expected_ids
        and all(shot_id > 0 and shot_id in physical_by_id for shot_id in expected_ids)
        and not missing_shot_ids
        and not duplicate_serve_ids
        and not duplicate_physical_ids
        and not extra_physical_ids
        and set(physical_by_id) == set(expected_ids)
    )
    measured = bool(
        physical_report.get("physical_contact_measured", False)
        and physical_report.get("landing_measured", False)
        and all_shots_joined
        and all_telemetry_complete
    )
    global_contacts = sum(item["contacts"] for item in per_side.values())
    global_landing_observations = sum(
        item["landing_observations"] for item in per_side.values()
    )
    global_landing_censored = sum(
        item["landing_censored"] for item in per_side.values()
    )
    global_landings = sum(item["legal_landings"] for item in per_side.values())
    global_counts_pass = bool(
        global_contacts >= int(min_global_contacts)
        and global_landing_observations
        >= int(min_global_landing_observations)
        and global_landings >= int(min_global_landings)
    )
    passed = bool(
        measured
        and not unassigned_shot_ids
        and not unexpected_censored_shot_ids
        and global_counts_pass
        and all(item["pass"] for item in per_side.values())
    )
    return {
        "physical_contact_measured": measured,
        "landing_measured": measured,
        "all_shots_joined": all_shots_joined,
        "all_sides_assigned": not unassigned_shot_ids,
        "missing_shot_ids": sorted(set(missing_shot_ids)),
        "extra_physical_shot_ids": extra_physical_ids,
        "duplicate_serve_shot_ids": duplicate_serve_ids,
        "duplicate_physical_shot_ids": duplicate_physical_ids,
        "unassigned_shot_ids": unassigned_shot_ids,
        "allowed_landing_censor_lanes": sorted(allowed_censor_lanes),
        "unexpected_censored_shot_ids": sorted(
            set(unexpected_censored_shot_ids)
        ),
        "landing_censoring_accounted": not unexpected_censored_shot_ids,
        "minimum_shots_per_side": int(min_samples_per_side),
        "exact_shots_per_side": (
            None if exact_samples_per_side is None
            else int(exact_samples_per_side)
        ),
        "minimum_contacts_per_side": int(min_contacts_per_side),
        "minimum_landing_observations_per_side": int(
            min_landing_observations_per_side
        ),
        "minimum_landings_per_side": int(min_landings_per_side),
        "minimum_contact_rate_per_side": float(min_contact_rate),
        "minimum_landing_rate_per_side": float(min_landing_rate),
        "global_contacts": global_contacts,
        "global_landing_observations": global_landing_observations,
        "global_landing_censored": global_landing_censored,
        "global_legal_landings": global_landings,
        "minimum_global_contacts": int(min_global_contacts),
        "minimum_global_landing_observations": int(
            min_global_landing_observations
        ),
        "minimum_global_landings": int(min_global_landings),
        "global_counts_pass": global_counts_pass,
        "by_side": per_side,
        "pass": passed,
    }


def physical_report_complete(
    report: Mapping[str, Any], expected_shot_ids: Iterable[int]
) -> bool:
    """Check that every expected shot reached a complete, loss-detectable window."""
    expected = [int(shot_id) for shot_id in expected_shot_ids]
    rows = list(report.get("rows", []))
    row_ids = [int(row.get("shot_id") or 0) for row in rows]
    return bool(
        expected
        and len(set(expected)) == len(expected)
        and row_ids == expected
        and report.get("physical_contact_measured", False)
        and report.get("landing_measured", False)
        and all(bool(row.get("telemetry_complete", False)) for row in rows)
    )


def serve_to_dict(spec: ServeSpec) -> dict[str, Any]:
    return asdict(spec)
