"""Host-only tests for the autonomous physical Gate3 contract."""

from __future__ import annotations

import csv
import tempfile
import pathlib
import subprocess
import sys
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
REPO_ROOT = ROOT.parents[1]
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from pp_gate3_core import (  # noqa: E402
    PhysicalEvidenceAccumulator,
    base_pose_to_marker_pose,
    calibrated_p1_marker_contract,
    generate_v17_r10_random_serves,
    join_physical_evidence_by_side,
    parse_serves_list,
    physical_report_complete,
    select_swing_side,
    serves_to_flat_list,
    table_to_world_position,
    world_to_table_position,
    audit_actual_q_full_tail_ledger,
    audit_cpp_planner_debug_csv,
    audit_runner_engagement_trace,
    audit_runner_localization_trace,
    join_planner_evidence_by_flight_id,
)


def cpp_planner_audit_row(sequence: int) -> dict[str, object]:
    net_cross = 1000.0 + sequence
    return {
        "session_id": "gate3_test",
        "command_seq": sequence,
        "flight_id": sequence,
        "wire_valid": 1,
        "reason": "command_valid",
        "strike_x": 0.08,
        "strike_y": -1.0,
        "strike_z": 0.50,
        "racket_vx": 2.0,
        "racket_vy": 0.2,
        "racket_vz": 0.6,
        "swing_sign": 1 if sequence % 2 else -1,
        "tts_s": 0.53,
        "source_age_at_publish_ms": 1.0,
        "total_ms": 0.2,
        "input_ring_drops": 0,
        "logger_drops": 0,
        "base_valid": 1,
        "base_age_ms": 0.3,
        "estimator_kind": "batch_physics_cpp_no_ekf_persistent_bounce",
        "control_stage2_mode": "post_net_one_shot_effective_bounce_zero_spin",
        "out_of_order_samples_total": 0,
        "one_shot_enabled": 1,
        "one_shot_flight_seq": sequence,
        "net_cross_source_time_s": net_cross,
        "commit_source_time_s": net_cross + 0.05,
        "post_net_commit_delay_s": 0.05,
        "post_net_future_bounce_tangential_gain": 0.348260,
        "trajectory_epoch": sequence,
        "snapshot_sequence": sequence,
        "snapshot_published_total": sequence,
        "snapshot_consumed_total": sequence,
        "snapshot_superseded_total": 0,
    }


def cpp_planner_packet_audit_row(sequence: int) -> dict[str, object]:
    row = cpp_planner_audit_row(sequence)
    row.update(
        {
            "snapshot_published_total": 0,
            "snapshot_consumed_total": 0,
            "flight_packet_input": 1,
            "packet_session_id": "gate3_test",
            "packet_producer_instance_id": "packetizer-test",
            "packet_payload_hash": f"{sequence:016x}",
            "packet_transmit_index": 0,
            "packet_transmit_count": 3,
            "packet_received_total": 3 * sequence - 2,
            "packet_accepted_total": sequence,
            "packet_duplicate_total": 2 * (sequence - 1),
            "packet_conflict_total": 0,
            "packet_invalid_total": 0,
            "packet_queue_depth": 0,
        }
    )
    return row


def write_cpp_planner_audit(path: pathlib.Path, rows: list[dict[str, object]]) -> None:
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def write_cpp_flight_packet_audit(path: pathlib.Path, flights: int) -> None:
    rows = []
    for sequence in range(1, flights + 1):
        for transmit_index in range(3):
            rows.append(
                {
                    "session_id": "gate3_test",
                    "producer_instance_id": "packetizer-test",
                    "trajectory_epoch": sequence,
                    "flight_sequence": sequence,
                    "payload_hash": f"{sequence:016x}",
                    "transmit_index": transmit_index,
                    "transmit_count": 3,
                    "sample_count": 45,
                    "source_age_ms": 1.0 + 5.0 * transmit_index,
                    "dedupe_result": (
                        "accepted" if transmit_index == 0 else "duplicate"
                    ),
                    "accepted_for_solve": 1 if transmit_index == 0 else 0,
                    "packet_queue_depth": 1 if transmit_index == 0 else 0,
                }
            )
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def complete_shot(accumulator: PhysicalEvidenceAccumulator, shot_id: int) -> None:
    """Feed a single-bounce, racket-contact, legal-return event sequence."""
    base = shot_id * 1_000_000_000
    samples = (
        # active, position, velocity, racket count, table count, net count
        (True, (2.40, -0.70, 1.25), (-3.0, 0.0, 2.0), 0, 0, 0),
        (True, (1.00, -0.70, 0.78), (-2.0, 0.0, 2.0), 0, 1, 0),
        (True, (0.10, -0.70, 1.08), (2.0, 0.0, 1.0), 1, 1, 0),
        (True, (1.90, -0.70, 0.78), (1.4, 0.0, 1.0), 1, 2, 0),
        (True, (2.00, -0.70, 0.82), (1.3, 0.0, 0.8), 1, 2, 0),
        (True, (2.20, -0.70, 0.030), (1.0, 0.0, -1.0), 1, 2, 0),
        (False, (2.21, -0.70, 0.019), (1.0, 0.0, -1.1), 1, 2, 0),
    )
    for index, (active, position, velocity, racket, table, net) in enumerate(samples):
        accumulator.ingest(
            stamp_ns=base + index * 10_000_000,
            shot_id=shot_id,
            active=active,
            position=position,
            velocity=velocity,
            racket_contact_count=racket,
            table_contact_count=table,
            net_contact_count=net,
        )


class Gate3ScenarioContractTest(unittest.TestCase):
    def test_serve_parser_is_flat_numeric_and_side_neutral(self) -> None:
        serves = parse_serves_list("[2.4,-0.7,0.49,-3,0,2]")
        self.assertEqual(len(serves), 1)
        self.assertEqual(serves[0].position, (2.4, -0.7, 0.49))
        for invalid in (
            "[]",
            "[1,2,3]",
            "[[2.4,-0.7,0.49,-3,0,2]]",
            "[{'side':'forehand'}]",
            "{'side':'forehand'}",
            "[2.4,-0.7,0.49,-3,0,float('nan')]",
        ):
            with self.subTest(invalid=invalid):
                with self.assertRaises((ValueError, SyntaxError)):
                    parse_serves_list(invalid)

    def test_table_world_frame_round_trip(self) -> None:
        table = (2.4, -0.7, 0.49)
        world = table_to_world_position(table)
        self.assertEqual(world, (2.4, -0.7, 1.25))
        for actual, expected in zip(world_to_table_position(world), table):
            self.assertAlmostEqual(actual, expected)

    def test_sim_pelvis_is_inverted_to_the_calibrated_marker_boundary(self) -> None:
        half = 2.0**-0.5
        marker_position, marker_quaternion = base_pose_to_marker_pose(
            (1.0, 2.0, 3.0),
            (half, 0.0, 0.0, half),
            (1.0, 0.0, 0.0),
            (1.0, 0.0, 0.0, 0.0),
        )
        for actual, expected in zip(marker_position, (1.0, 1.0, 3.0)):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(
            marker_quaternion, (half, 0.0, 0.0, half)
        ):
            self.assertAlmostEqual(actual, expected)

    def test_p1_contract_requires_both_calibration_receipts(self) -> None:
        config = {
            "hope_world": {
                "contract": {
                    "venue_calibrated": True,
                    "calibration_sha256": "a" * 64,
                },
                "mocap_to_base_link": {
                    "p1": {
                        "calibrated": True,
                        "calibration_sha256": "b" * 64,
                        "xyz_m": [0.1, -0.2, 0.3],
                        "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                    }
                },
            }
        }
        translation, quaternion = calibrated_p1_marker_contract(config)
        self.assertEqual(translation, (0.1, -0.2, 0.3))
        self.assertEqual(quaternion, (1.0, 0.0, 0.0, 0.0))
        config["hope_world"]["contract"]["venue_calibrated"] = False
        with self.assertRaisesRegex(ValueError, "world frame"):
            calibrated_p1_marker_contract(config)

    def test_side_hysteresis_matches_latched_planner_semantics(self) -> None:
        self.assertEqual(select_swing_side(-0.30, 0.0, None), "forehand")
        self.assertEqual(select_swing_side(-0.22, 0.0, "forehand"), "forehand")
        self.assertEqual(select_swing_side(-0.20, 0.0, "forehand"), "backhand")
        self.assertEqual(select_swing_side(-0.28, 0.0, "backhand"), "backhand")
        self.assertEqual(select_swing_side(-0.30, 0.0, "backhand"), "forehand")

    def test_v17_r10_random_sweep_is_reproducible_balanced_and_supported(self) -> None:
        first = generate_v17_r10_random_serves(48, 17010)
        second = generate_v17_r10_random_serves(48, 17010)
        changed = generate_v17_r10_random_serves(48, 17011)
        self.assertEqual(first, second)
        self.assertNotEqual(first, changed)
        self.assertEqual(len(parse_serves_list(serves_to_flat_list(first))), 48)

        station_y = -0.7625
        selected = [
            select_swing_side(
                spec.position[1], station_y, None, split_y=-0.265
            )
            for spec in first
        ]
        self.assertEqual(selected.count("forehand"), 24)
        self.assertEqual(selected.count("backhand"), 24)
        for spec, side in zip(first, selected):
            reach_y = spec.position[1] - station_y
            if side == "forehand":
                self.assertGreaterEqual(reach_y, -0.48)
                self.assertLessEqual(reach_y, -0.40)
            else:
                self.assertGreaterEqual(reach_y, -0.13)
                self.assertLessEqual(reach_y, -0.05)

        with self.assertRaisesRegex(ValueError, "at least 8"):
            generate_v17_r10_random_serves(7, 0)


class Gate3CppPlannerEvidenceTest(unittest.TestCase):
    def test_runner_localization_requires_every_motion_tick_fresh_without_serves(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as directory:
            trace = pathlib.Path(directory) / "runner_trace.csv"
            with trace.open("w", newline="") as stream:
                writer = csv.DictWriter(
                    stream,
                    fieldnames=(
                        "tick", "wall_time_ns", "mode", "localization_fresh",
                    ),
                )
                writer.writeheader()
                writer.writerows(
                    [
                        {"tick": 10, "wall_time_ns": 1_000_000_000,
                         "mode": 3, "localization_fresh": 1},
                        {"tick": 11, "wall_time_ns": 1_020_000_000,
                         "mode": 3, "localization_fresh": 0},
                        {"tick": 12, "wall_time_ns": 1_040_000_000,
                         "mode": 3, "localization_fresh": 1},
                    ]
                )
            report = audit_runner_localization_trace(trace)
        self.assertFalse(report["pass"])
        self.assertFalse(report["continuous_fresh"])
        self.assertEqual(report["motion_ticks"], 3)
        self.assertEqual(report["stale_ticks"], 1)
        self.assertEqual(
            report["stale_tick_ranges"],
            [{"first_tick": 11, "last_tick": 11, "ticks": 1}],
        )

    def test_runner_localization_accepts_contiguous_fresh_motion_trace(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            trace = pathlib.Path(directory) / "runner_trace.csv"
            with trace.open("w", newline="") as stream:
                writer = csv.DictWriter(
                    stream,
                    fieldnames=(
                        "tick", "wall_time_ns", "mode", "localization_fresh",
                    ),
                )
                writer.writeheader()
                writer.writerows(
                    [
                        {"tick": 9, "wall_time_ns": 980_000_000,
                         "mode": 1, "localization_fresh": 0},
                        {"tick": 10, "wall_time_ns": 1_000_000_000,
                         "mode": 3, "localization_fresh": 1},
                        {"tick": 11, "wall_time_ns": 1_020_000_000,
                         "mode": 3, "localization_fresh": 1},
                    ]
                )
            report = audit_runner_localization_trace(trace)
        self.assertTrue(report["pass"], report["errors"])
        self.assertTrue(report["continuous_fresh"])
        self.assertEqual(report["fresh_fraction"], 1.0)

    def test_runner_localization_fails_closed_on_sparse_motion_trace(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            trace = pathlib.Path(directory) / "runner_trace.csv"
            with trace.open("w", newline="") as stream:
                writer = csv.DictWriter(
                    stream,
                    fieldnames=(
                        "tick", "wall_time_ns", "mode", "localization_fresh",
                    ),
                )
                writer.writeheader()
                writer.writerows(
                    [
                        {"tick": 10, "wall_time_ns": 1_000_000_000,
                         "mode": 3, "localization_fresh": 1},
                        {"tick": 12, "wall_time_ns": 1_040_000_000,
                         "mode": 3, "localization_fresh": 1},
                    ]
                )
            report = audit_runner_localization_trace(trace)
        self.assertFalse(report["pass"])
        self.assertFalse(report["motion_tick_span_contiguous"])
        self.assertTrue(any("sparse" in error for error in report["errors"]))

    def test_live_cpp_one_shot_receipt_passes(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "planner.csv"
            write_cpp_planner_audit(
                path, [cpp_planner_audit_row(1), cpp_planner_audit_row(2)]
            )
            report = audit_cpp_planner_debug_csv(path, 2)
        self.assertTrue(report["planner_contract_pass"], report["errors"])
        self.assertEqual(report["planner_runtime"], "hope_planner_cpp_node")
        self.assertEqual(report["valid_commands"], 2)
        self.assertEqual(report["tts_s_range"], [0.53, 0.53])
        self.assertEqual(
            [row["planner_side"] for row in report["rows"]],
            ["forehand", "backhand"],
        )

    def test_physical_spin_mode_requires_matching_physics_evidence(self) -> None:
        physical = {
            "control_stage2_mode": "post_net_one_shot_physical_spin_magnus",
            "future_contact_magnus_enabled": 1,
            "physics_drag_k": 0.12861635416914574,
            "physics_magnus_k": 0.00379260504480258,
            "physics_table_gain": 0.38744443855065164,
            "physics_table_mu": 0.16706204822993975,
        }
        variants = [({}, True), ({"future_contact_magnus_enabled": 0}, False),
                    ({"physics_magnus_k": "nan"}, False),
                    ({"physics_table_mu": 0.0}, False),
                    ({"physics_drag_k": 0.119006}, False),
                    ({"physics_table_gain": 0.369}, False),
                    ({"control_stage2_mode": "unknown"}, False)]
        for change, expected_pass in variants:
            with self.subTest(change=change), tempfile.TemporaryDirectory() as directory:
                path = pathlib.Path(directory) / "planner.csv"
                row = cpp_planner_audit_row(1)
                row.update(physical)
                row.update(change)
                write_cpp_planner_audit(path, [row])
                report = audit_cpp_planner_debug_csv(path, 1)
                self.assertEqual(report["planner_contract_pass"], expected_pass, report["errors"])
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "planner.csv"
            row = cpp_planner_audit_row(1)
            row.update(physical)
            del row["physics_magnus_k"]
            write_cpp_planner_audit(path, [row])
            self.assertFalse(audit_cpp_planner_debug_csv(path, 1)["planner_contract_pass"])

    def test_planner_side_backfills_missed_flights_by_immutable_flight_id(self) -> None:
        serves = [
            {"serve": 1, "shot_id": 1, "flight_id": 1, "command_side": "forehand"},
            {"serve": 2, "shot_id": 2, "flight_id": 2, "command_side": None},
        ]
        planner = {
            "rows": [
                {"flight_id": 1, "planner_side": "forehand", "row_pass": True},
                {"flight_id": 2, "planner_side": "backhand", "row_pass": True},
            ]
        }
        joined = join_planner_evidence_by_flight_id(serves, planner, 2)
        self.assertTrue(joined["pass"], joined["errors"])
        self.assertEqual(joined["joined_flight_ids"], [1, 2])
        self.assertEqual(joined["side_backfilled_flight_ids"], [2])
        self.assertEqual(serves[1]["command_side"], "backhand")

    def test_runner_engage_gap_uses_trace_wall_time_and_frozen_flight_id(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            trace = pathlib.Path(directory) / "runner_trace.csv"
            with trace.open("w", newline="") as stream:
                writer = csv.DictWriter(
                    stream,
                    fieldnames=(
                        "tick", "wall_time_ns", "level", "planner_status",
                        "frozen_flight_id",
                    ),
                )
                writer.writeheader()
                writer.writerows(
                    [
                        {"tick": 10, "wall_time_ns": 1_000_000_000, "level": 1,
                         "planner_status": "engage", "frozen_flight_id": 15},
                        {"tick": 11, "wall_time_ns": 1_020_000_000, "level": 1,
                         "planner_status": "swinging", "frozen_flight_id": 15},
                        {"tick": 20, "wall_time_ns": 3_310_000_000, "level": 1,
                         "planner_status": "engage", "frozen_flight_id": 17},
                    ]
                )
            report = audit_runner_engagement_trace(trace, range(15, 19))
        self.assertTrue(report["trace_contract_pass"], report["errors"])
        self.assertEqual(report["engaged_flight_ids"], [15, 17])
        self.assertEqual(report["missed_flight_ids"], [16, 18])
        self.assertEqual(report["engage_gaps_s"], [2.31])
        self.assertEqual(report["gap_time_source"], "runner_trace.wall_time_ns")

    def test_runner_trace_counts_initial_and_rapid_preempt_engagement_edges(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            trace = pathlib.Path(directory) / "runner_trace.csv"
            with trace.open("w", newline="") as stream:
                writer = csv.DictWriter(
                    stream,
                    fieldnames=(
                        "tick", "wall_time_ns", "level", "planner_status",
                        "frozen_flight_id",
                    ),
                )
                writer.writeheader()
                writer.writerows(
                    [
                        {"tick": 10, "wall_time_ns": 1_000_000_000, "level": 1,
                         "planner_status": "engage", "frozen_flight_id": 15},
                        {"tick": 60, "wall_time_ns": 2_160_000_000, "level": 1,
                         "planner_status": "preempt_engage", "frozen_flight_id": 16},
                        {"tick": 110, "wall_time_ns": 3_320_000_000, "level": 1,
                         "planner_status": "preempt_engage", "frozen_flight_id": 17},
                    ]
                )
            report = audit_runner_engagement_trace(trace, range(15, 18))
        self.assertTrue(report["trace_contract_pass"], report["errors"])
        self.assertEqual(report["engaged_flight_ids"], [15, 16, 17])
        self.assertEqual(report["missed_flight_ids"], [])
        self.assertEqual(report["engage_gaps_s"], [1.16, 1.16])
        self.assertEqual(
            [event["edge_kind"] for event in report["engage_events"]],
            ["engage", "preempt_engage", "preempt_engage"],
        )

    def test_runner_trace_closes_old_flight_on_preempt_and_final_on_native_end(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            trace = pathlib.Path(directory) / "runner_trace.csv"
            fieldnames = (
                "tick", "wall_time_ns", "level", "planner_status",
                "frozen_flight_id", "frozen_producer_epoch",
                "planner_completion_seq",
                "planner_completed_producer_epoch",
                "planner_completed_flight_id", "planner_completion_kind",
                "base_x", "base_y",
            )
            rows = []
            wall_ns = 1_000_000_000
            for index, flight_id in enumerate(range(15, 35)):
                if index == 0:
                    status = "engage"
                    completion_seq = 0
                    completed_id = 0
                    completion_kind = ""
                else:
                    status = "preempt_engage"
                    completion_seq = index
                    completed_id = flight_id - 1
                    completion_kind = "preempt"
                rows.append(
                    {
                        "tick": 10 + 58 * index,
                        "wall_time_ns": wall_ns,
                        "level": 1,
                        "planner_status": status,
                        "frozen_flight_id": flight_id,
                        "frozen_producer_epoch": 1,
                        "planner_completion_seq": completion_seq,
                        "planner_completed_producer_epoch": (
                            0 if completion_seq == 0 else 1
                        ),
                        "planner_completed_flight_id": completed_id,
                        "planner_completion_kind": completion_kind,
                        "base_x": 0.0,
                        "base_y": 0.01,
                    }
                )
                wall_ns += 1_160_000_000
            # The final flight is not preempted. Its native clip end owns the
            # twentieth, immutable completion edge.
            rows.append(
                {
                    "tick": 10 + 58 * 20,
                    "wall_time_ns": wall_ns,
                    "level": 0,
                    "planner_status": "rest",
                    "frozen_flight_id": 34,
                    "frozen_producer_epoch": 1,
                    "planner_completion_seq": 20,
                    "planner_completed_producer_epoch": 1,
                    "planner_completed_flight_id": 34,
                    "planner_completion_kind": "native",
                    "base_x": 0.0,
                    "base_y": 0.0,
                }
            )
            # The next tick persists the snapshot and must not double count.
            rows.append(dict(rows[-1], tick=rows[-1]["tick"] + 1,
                             wall_time_ns=wall_ns + 20_000_000))
            with trace.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(rows)

            report = audit_runner_engagement_trace(
                trace, range(15, 35), require_completion_contract=True
            )

        self.assertTrue(report["trace_contract_pass"], report["errors"])
        self.assertTrue(report["completion_contract_available"])
        self.assertTrue(report["completion_contract_pass"], report["errors"])
        self.assertEqual(report["completed_flight_ids"], list(range(15, 35)))
        self.assertEqual(
            [event["completion_kind"] for event in report["completion_events"]],
            ["preempt"] * 19 + ["native"],
        )
        self.assertEqual(report["completion_events"][0]["flight_id"], 15)
        self.assertEqual(report["engage_events"][1]["flight_id"], 16)

    def test_runner_trace_rejects_completion_credited_to_new_preempt_flight(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            trace = pathlib.Path(directory) / "runner_trace.csv"
            with trace.open("w", newline="") as stream:
                writer = csv.DictWriter(
                    stream,
                    fieldnames=(
                        "tick", "wall_time_ns", "level", "planner_status",
                        "frozen_flight_id", "frozen_producer_epoch",
                        "planner_completion_seq",
                        "planner_completed_producer_epoch",
                        "planner_completed_flight_id",
                        "planner_completion_kind",
                    ),
                )
                writer.writeheader()
                writer.writerows(
                    [
                        {
                            "tick": 1, "wall_time_ns": 1_000_000_000,
                            "level": 1, "planner_status": "engage",
                            "frozen_flight_id": 15,
                            "frozen_producer_epoch": 1,
                            "planner_completion_seq": 0,
                            "planner_completed_producer_epoch": 0,
                            "planner_completed_flight_id": 0,
                            "planner_completion_kind": "",
                        },
                        {
                            "tick": 2, "wall_time_ns": 2_160_000_000,
                            "level": 1, "planner_status": "preempt_engage",
                            "frozen_flight_id": 16,
                            "frozen_producer_epoch": 1,
                            "planner_completion_seq": 1,
                            "planner_completed_producer_epoch": 1,
                            # Wrong: the edge must close old flight 15.
                            "planner_completed_flight_id": 16,
                            "planner_completion_kind": "preempt",
                        },
                    ]
                )
            report = audit_runner_engagement_trace(
                trace, (15, 16), require_completion_contract=True
            )

        self.assertFalse(report["trace_contract_pass"])
        self.assertFalse(report["completion_contract_pass"])
        self.assertEqual(report["completed_flight_ids"], [16])
        self.assertEqual(report["incomplete_flight_ids"], [15])
        self.assertTrue(
            any("active flight is (1, 15)" in error for error in report["errors"]),
            report["errors"],
        )

    def test_runner_trace_rejects_latched_completion_sequence_regression(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            trace = pathlib.Path(directory) / "runner_trace.csv"
            fieldnames = (
                "tick", "wall_time_ns", "level", "planner_status",
                "frozen_producer_epoch", "frozen_flight_id",
                "planner_completion_seq", "planner_completed_producer_epoch",
                "planner_completed_flight_id", "planner_completion_kind",
            )
            rows = [
                dict(tick=1, wall_time_ns=1_000_000_000, level=1,
                     planner_status="engage", frozen_producer_epoch=1,
                     frozen_flight_id=1, planner_completion_seq=0,
                     planner_completed_producer_epoch=0,
                     planner_completed_flight_id=0, planner_completion_kind=""),
                dict(tick=2, wall_time_ns=2_000_000_000, level=1,
                     planner_status="preempt_engage", frozen_producer_epoch=1,
                     frozen_flight_id=2, planner_completion_seq=1,
                     planner_completed_producer_epoch=1,
                     planner_completed_flight_id=1,
                     planner_completion_kind="preempt"),
                dict(tick=3, wall_time_ns=3_000_000_000, level=1,
                     planner_status="preempt_engage", frozen_producer_epoch=1,
                     frozen_flight_id=3, planner_completion_seq=2,
                     planner_completed_producer_epoch=1,
                     planner_completed_flight_id=2,
                     planner_completion_kind="preempt"),
                # A stale snapshot cannot roll the authoritative edge sequence back.
                dict(tick=4, wall_time_ns=4_000_000_000, level=1,
                     planner_status="swinging", frozen_producer_epoch=1,
                     frozen_flight_id=3, planner_completion_seq=1,
                     planner_completed_producer_epoch=1,
                     planner_completed_flight_id=1,
                     planner_completion_kind="preempt"),
            ]
            with trace.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(rows)
            report = audit_runner_engagement_trace(
                trace, (1, 2, 3), require_completion_contract=True
            )

        self.assertFalse(report["trace_contract_pass"])
        self.assertTrue(
            any("sequence regressed: 2 -> 1" in error for error in report["errors"]),
            report["errors"],
        )

    def test_schema21_ignores_inactive_completion_columns_between_native_swings(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            trace = pathlib.Path(directory) / "runner_trace.csv"
            fieldnames = (
                "tick", "wall_time_ns", "level", "planner_status",
                "frozen_producer_epoch", "frozen_flight_id",
                "planner_completion_seq", "planner_completed_producer_epoch",
                "planner_completed_flight_id", "planner_completion_kind",
            )
            rows = [
                dict(tick=1, wall_time_ns=1_000_000_000, level=1,
                     planner_status="engage", frozen_producer_epoch=0,
                     frozen_flight_id=1, planner_completion_seq=0,
                     planner_completed_producer_epoch=0,
                     planner_completed_flight_id=0,
                     planner_completion_kind="none"),
                dict(tick=100, wall_time_ns=3_000_000_000, level=0,
                     planner_status="rest", frozen_producer_epoch=0,
                     frozen_flight_id=1, planner_completion_seq=0,
                     planner_completed_producer_epoch=0,
                     planner_completed_flight_id=0,
                     planner_completion_kind="none"),
                dict(tick=120, wall_time_ns=4_000_000_000, level=1,
                     planner_status="engage", frozen_producer_epoch=0,
                     frozen_flight_id=2, planner_completion_seq=0,
                     planner_completed_producer_epoch=0,
                     planner_completed_flight_id=0,
                     planner_completion_kind="none"),
            ]
            with trace.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(rows)
            report = audit_runner_engagement_trace(trace, (1, 2))

        self.assertTrue(report["trace_contract_pass"], report["errors"])
        self.assertFalse(report["completion_contract_required"])
        self.assertTrue(report["completion_contract_available"])
        self.assertTrue(report["completion_contract_pass"])
        self.assertEqual(report["engaged_flight_ids"], [1, 2])

    def test_physical_contact_stamp_joins_nearest_runner_base_pose_not_completion_pose(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            trace = pathlib.Path(directory) / "runner_trace.csv"
            with trace.open("w", newline="") as stream:
                writer = csv.DictWriter(
                    stream,
                    fieldnames=(
                        "tick", "wall_time_ns", "level", "planner_status",
                        "frozen_flight_id", "base_x", "base_y",
                    ),
                )
                writer.writeheader()
                writer.writerows(
                    [
                        dict(tick=1, wall_time_ns=1_000_000_000, level=1,
                             planner_status="engage", frozen_flight_id=7,
                             base_x=0.0, base_y=0.04),
                        # Nearest 50 Hz sample to the measured racket-contact edge.
                        dict(tick=2, wall_time_ns=1_020_000_000, level=1,
                             planner_status="swinging", frozen_flight_id=7,
                             base_x=0.0, base_y=0.055),
                        # Lifecycle completion is much later and back at HOME;
                        # it must never be used as an at-contact movement sample.
                        dict(tick=60, wall_time_ns=2_200_000_000, level=0,
                             planner_status="rest", frozen_flight_id=7,
                             base_x=0.0, base_y=0.0),
                    ]
                )
            report = audit_runner_engagement_trace(
                trace,
                (7,),
                physical_contact_wall_time_ns_by_flight={7: 1_019_000_000},
            )

        self.assertTrue(report["trace_contract_pass"], report["errors"])
        self.assertTrue(report["physical_contact_pose_contract_pass"])
        self.assertEqual(report["physical_contact_pose_missing_flight_ids"], [])
        self.assertEqual(len(report["physical_contact_pose_events"]), 1)
        pose = report["physical_contact_pose_events"][0]
        self.assertEqual(pose["flight_id"], 7)
        self.assertEqual(pose["base_y"], 0.055)
        self.assertEqual(pose["join_gap_s"], 0.001)

    def test_immutable_flight_packet_receipt_passes(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "planner.csv"
            write_cpp_planner_audit(
                path,
                [
                    cpp_planner_packet_audit_row(1),
                    cpp_planner_packet_audit_row(2),
                ],
            )
            write_cpp_flight_packet_audit(
                pathlib.Path(str(path) + ".flight_packets.csv"), 2
            )
            report = audit_cpp_planner_debug_csv(path, 2)
        self.assertTrue(report["planner_contract_pass"], report["errors"])
        self.assertEqual(report["input_contract"], "immutable_flight_packet_v1")
        self.assertEqual(report["flight_packet_receive_rows"], 6)

    def test_immutable_flight_packet_identity_conflict_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "planner.csv"
            rows = [
                cpp_planner_packet_audit_row(1),
                cpp_planner_packet_audit_row(2),
            ]
            rows[1]["packet_conflict_total"] = 1
            write_cpp_planner_audit(path, rows)
            write_cpp_flight_packet_audit(
                pathlib.Path(str(path) + ".flight_packets.csv"), 2
            )
            report = audit_cpp_planner_debug_csv(path, 2)
        self.assertFalse(report["planner_contract_pass"])
        self.assertTrue(
            any("identity conflicts" in item for item in report["errors"])
        )

    def test_duplicate_identity_or_non_cpp_contract_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "planner.csv"
            first = cpp_planner_audit_row(1)
            second = cpp_planner_audit_row(2)
            second["command_seq"] = 1
            second["post_net_commit_delay_s"] = 0.06
            second["logger_drops"] = 1
            write_cpp_planner_audit(path, [first, second])
            report = audit_cpp_planner_debug_csv(path, 2)
        self.assertFalse(report["planner_contract_pass"])
        self.assertTrue(any("command_seq sequence" in item for item in report["errors"]))
        self.assertTrue(any("50 ms" in item for item in report["errors"]))
        self.assertTrue(any("logger drops" in item for item in report["errors"]))


class Gate3ActualQFullTailLedgerTest(unittest.TestCase):
    JOINTS = [f"joint_{index}" for index in range(31)]

    @classmethod
    def _trace_fields(cls) -> list[str]:
        fields = [
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
        ]
        for prefix in (
            "actual_q_hard_lo_",
            "actual_q_hard_hi_",
            "actual_q_hard_excess_",
            "q_",
        ):
            fields.extend(f"{prefix}{joint}" for joint in cls.JOINTS)
        return fields

    @classmethod
    def _trace_row(
        cls,
        wall_time_ns: int,
        *,
        status: str = "swinging",
        completion_seq: int = 0,
        completed_flight: int = 0,
        violation: bool = False,
    ) -> dict[str, object]:
        row: dict[str, object] = {
            "wall_time_ns": wall_time_ns,
            "mode": 3,
            "level": 1 if completion_seq == 0 else 0,
            "planner_status": status,
            "frozen_flight_id": 1,
            "planner_completion_seq": completion_seq,
            "planner_completed_flight_id": completed_flight,
            "actual_q_hard_tolerance_rad": 0.002,
            "actual_q_hard_audit_only": 1,
            "actual_q_hard_violation_count": int(violation),
            "actual_q_hard_max_excess_rad": 0.01 if violation else 0.0,
        }
        for joint in cls.JOINTS:
            q = 1.01 if violation and joint == cls.JOINTS[0] else 0.0
            row[f"actual_q_hard_lo_{joint}"] = -1.0
            row[f"actual_q_hard_hi_{joint}"] = 1.0
            row[f"actual_q_hard_excess_{joint}"] = max(abs(q) - 1.0, 0.0)
            row[f"q_{joint}"] = q
        return row

    @classmethod
    def _write_fixture(
        cls,
        root: pathlib.Path,
        *,
        violation: bool,
        tail_end_ns: int = 5_100_000_000,
        engage_wall_ns: int = 1_000_000_000,
        violation_wall_ns: int = 1_020_000_000,
        completion_wall_ns: int | None = 2_000_000_000,
    ) -> tuple[pathlib.Path, pathlib.Path, pathlib.Path]:
        trace = root / "trace.csv"
        plant = root / "plant.csv"
        runner_log = root / "runner.log"
        trace_rows = []
        for wall in range(1_000_000_000, tail_end_ns + 1, 20_000_000):
            complete = (
                completion_wall_ns is not None and wall >= completion_wall_ns
            )
            trace_rows.append(
                cls._trace_row(
                    wall,
                    status=(
                        "engage"
                        if wall == engage_wall_ns
                        else (
                            "complete"
                            if completion_wall_ns is not None
                            and wall == completion_wall_ns
                            else "swinging"
                        )
                    ),
                    completion_seq=int(complete),
                    completed_flight=int(complete),
                    violation=violation and wall == violation_wall_ns,
                )
            )
        with trace.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=cls._trace_fields())
            writer.writeheader()
            writer.writerows(trace_rows)

        plant_fields = ["wall_time_ns", *(f"q_{joint}" for joint in cls.JOINTS)]
        plant_rows = []
        for wall in range(990_000_000, tail_end_ns + 10_000_001, 10_000_000):
            active = violation and wall == violation_wall_ns
            row: dict[str, object] = {"wall_time_ns": wall}
            for joint in cls.JOINTS:
                row[f"q_{joint}"] = (
                    1.01 if active and joint == cls.JOINTS[0] else 0.0
                )
            plant_rows.append(row)
        with plant.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=plant_fields)
            writer.writeheader()
            writer.writerows(plant_rows)

        if violation:
            runner_log.write_text(
                "[pp actual-q audit] joint 'joint_0' measured q=+1.010000 "
                "exceeds exported interval=[-1.000000,+1.000000] "
                "tolerance=0.002000; audit-only, runner remains active\n"
                "[pp actual-q audit] joint 'joint_0' recovered inside the exported "
                "hard-limit tolerance; runner remained active\n"
            )
        else:
            runner_log.write_text("runner clean exit\n")
        return trace, plant, runner_log

    def test_zero_excursion_full_tail_passes(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            trace, plant, runner_log = self._write_fixture(
                pathlib.Path(tmp), violation=False
            )
            report = audit_actual_q_full_tail_ledger(
                trace,
                plant,
                runner_log,
                [1],
                required_post_completion_tail_s=3.0,
                runner_process_exited=True,
            )
        self.assertTrue(report["ledger_complete"], report["errors"])
        self.assertTrue(report["pass"])
        self.assertEqual(report["actual_q_violation_episode_count"], 0)
        self.assertEqual(
            report["source_event_counts"],
            {"runner_log": 0, "runner_trace": 0, "plant": 0},
        )

    def test_one_excursion_is_attributed_and_fails_stability(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            trace, plant, runner_log = self._write_fixture(
                pathlib.Path(tmp), violation=True
            )
            report = audit_actual_q_full_tail_ledger(
                trace,
                plant,
                runner_log,
                [1],
                physical_contact_wall_time_ns_by_flight={1: 1_010_000_000},
                required_post_completion_tail_s=3.0,
                runner_process_exited=True,
            )
        self.assertTrue(report["ledger_complete"], report["errors"])
        self.assertFalse(report["pass"])
        self.assertEqual(report["actual_q_violation_episode_count"], 1)
        self.assertEqual(
            report["source_event_counts"],
            {"runner_log": 1, "runner_trace": 1, "plant": 1},
        )
        self.assertEqual(report["canonical_events"][0]["flight_id"], 1)
        self.assertEqual(
            report["canonical_events"][0]["phase"], "contact_to_completion"
        )

    def test_missing_contact_timestamp_does_not_invalidate_safety_ledger(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            trace, plant, runner_log = self._write_fixture(
                pathlib.Path(tmp), violation=True
            )
            report = audit_actual_q_full_tail_ledger(
                trace,
                plant,
                runner_log,
                [1],
                physical_contact_wall_time_ns_by_flight={},
                required_post_completion_tail_s=3.0,
                runner_process_exited=True,
            )
        self.assertTrue(report["ledger_complete"], report["errors"])
        self.assertFalse(report["pass"])
        self.assertEqual(report["actual_q_violation_episode_count"], 1)
        self.assertEqual(
            report["canonical_events"][0]["phase"],
            "engage_to_completion_no_physical_contact",
        )
        coverage = report["physical_contact_timestamp_coverage"]
        self.assertFalse(coverage["coverage_pass"])
        self.assertFalse(coverage["required_for_ledger_complete"])
        self.assertEqual(coverage["missing_expected_flight_ids"], [1])

    def test_short_post_completion_tail_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            trace, plant, runner_log = self._write_fixture(
                pathlib.Path(tmp), violation=False, tail_end_ns=4_000_000_000
            )
            report = audit_actual_q_full_tail_ledger(
                trace,
                plant,
                runner_log,
                [1],
                required_post_completion_tail_s=3.0,
                runner_process_exited=True,
            )
        self.assertFalse(report["full_tail_coverage_pass"])
        self.assertFalse(report["ledger_complete"])
        self.assertTrue(any("post-completion" in value for value in report["errors"]))

    def test_missing_expected_completion_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            trace, plant, runner_log = self._write_fixture(
                pathlib.Path(tmp), violation=False, completion_wall_ns=None
            )
            report = audit_actual_q_full_tail_ledger(
                trace,
                plant,
                runner_log,
                [1],
                required_post_completion_tail_s=3.0,
                runner_process_exited=True,
            )
        self.assertFalse(report["ledger_complete"])
        self.assertFalse(report["full_tail_coverage_pass"])
        self.assertTrue(
            any("missing completion edges" in value for value in report["errors"])
        )

    def test_unscoped_runner_excursion_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            trace, plant, runner_log = self._write_fixture(
                pathlib.Path(tmp),
                violation=True,
                engage_wall_ns=1_100_000_000,
                violation_wall_ns=1_020_000_000,
            )
            report = audit_actual_q_full_tail_ledger(
                trace,
                plant,
                runner_log,
                [1],
                required_post_completion_tail_s=3.0,
                runner_process_exited=True,
            )
        # The timestamped contract starts at engage.  Since Runner log events
        # have no timestamp, an event that cannot be reconciled to that window
        # must remain visible and make the ledger incomplete.
        self.assertFalse(report["ledger_complete"])
        self.assertFalse(report["pass"])
        self.assertEqual(report["actual_q_violation_episode_count"], 1)
        self.assertEqual(report["source_event_counts"]["runner_log"], 1)
        self.assertEqual(report["source_event_counts"]["runner_trace"], 0)
        self.assertEqual(report["source_event_counts"]["plant"], 0)
        self.assertEqual(report["unattributed_events"], [1])
        self.assertEqual(
            report["by_flight"]["1"]["actual_q_violation_episode_count"], 0
        )


class Gate3PhysicalEvidenceTest(unittest.TestCase):
    def test_contact_edge_pose_accounts_only_for_measured_250hz_lag(self) -> None:
        top_center_z = 0.760 + 0.020 + 0.0001
        delayed_edge = {
            "position_world": [0.8, -0.7, top_center_z + 0.010],
            "velocity_world": [-1.5, 0.0, 3.2],
            "edge_observation_lag_s": 0.004,
        }
        self.assertTrue(
            PhysicalEvidenceAccumulator._legal_incoming_bounce(delayed_edge)
        )

        stale_edge = dict(delayed_edge)
        stale_edge["edge_observation_lag_s"] = 0.011
        self.assertFalse(
            PhysicalEvidenceAccumulator._legal_incoming_bounce(stale_edge)
        )

        impossible_height = dict(delayed_edge)
        impossible_height["position_world"] = [
            0.8,
            -0.7,
            top_center_z + 0.030,
        ]
        self.assertFalse(
            PhysicalEvidenceAccumulator._legal_incoming_bounce(
                impossible_height
            )
        )

    def test_complete_physical_sequence_passes(self) -> None:
        accumulator = PhysicalEvidenceAccumulator([1], min_samples=6)
        complete_shot(accumulator, 1)
        report = accumulator.report()
        self.assertTrue(report["physical_contact_measured"])
        self.assertTrue(report["physical_contact_pass"])
        self.assertTrue(report["landing_pass"])
        row = report["rows"][0]
        self.assertEqual(len(row["incoming_table_events"]), 1)
        self.assertTrue(row["incoming_bounce_pass"])
        self.assertEqual(len(row["racket_events"]), 1)
        self.assertEqual(len(row["post_racket_table_events"]), 1)

    def test_missing_landing_fails_closed(self) -> None:
        accumulator = PhysicalEvidenceAccumulator([1], min_samples=4)
        base = 1_000_000_000
        for index, values in enumerate((
            (True, (2.4, -0.7, 1.2), (-3.0, 0.0, 1.0), 0, 0, 0),
            (True, (1.0, -0.7, 0.78), (-2.0, 0.0, 2.0), 0, 1, 0),
            (True, (0.1, -0.7, 1.0), (2.0, 0.0, 1.0), 1, 1, 0),
            (True, (0.50, -0.70, 0.030), (2.0, 0.0, -1.0), 1, 1, 0),
            (False, (0.52, -0.70, 0.019), (2.0, 0.0, -1.1), 1, 1, 0),
        )):
            active, position, velocity, racket, table, net = values
            accumulator.ingest(
                stamp_ns=base + index * 10_000_000,
                shot_id=1,
                active=active,
                position=position,
                velocity=velocity,
                racket_contact_count=racket,
                table_contact_count=table,
                net_contact_count=net,
            )
        report = accumulator.report()
        self.assertTrue(report["physical_contact_measured"])
        self.assertTrue(report["physical_contact_pass"])
        self.assertFalse(report["landing_pass"])

    def test_table_side_or_wrong_half_contact_is_not_a_legal_bounce(self) -> None:
        accumulator = PhysicalEvidenceAccumulator([1], min_samples=6)
        complete_shot(accumulator, 1)
        shot = accumulator._shots[1]
        shot.incoming_table_events[0]["position_world"][2] = 0.74
        report = accumulator.report()
        self.assertFalse(report["rows"][0]["incoming_bounce_pass"])
        self.assertFalse(report["physical_contact_pass"])

    def test_report_completion_requires_every_parked_shot(self) -> None:
        accumulator = PhysicalEvidenceAccumulator([1, 2], min_samples=6)
        complete_shot(accumulator, 1)
        self.assertFalse(physical_report_complete(accumulator.report(), [1, 2]))
        complete_shot(accumulator, 2)
        self.assertTrue(physical_report_complete(accumulator.report(), [1, 2]))
        self.assertFalse(physical_report_complete(accumulator.report(), [2, 1]))

    def test_each_planner_selected_side_must_pass_independently(self) -> None:
        accumulator = PhysicalEvidenceAccumulator(range(1, 9), min_samples=6)
        for shot_id in range(1, 9):
            complete_shot(accumulator, shot_id)
        report = accumulator.report()
        serve_rows = [
            {
                "shot_id": shot_id,
                "command_side": "forehand" if shot_id <= 4 else "backhand",
            }
            for shot_id in range(1, 9)
        ]
        joined = join_physical_evidence_by_side(
            serve_rows,
            report,
            min_samples_per_side=4,
            min_contact_rate=0.8,
            min_landing_rate=0.8,
        )
        self.assertTrue(joined["pass"])

        # Destroy only one side. A pooled average could conceal this collapse;
        # the Gate3 verdict must not.
        for row in report["rows"]:
            if row["shot_id"] > 4:
                row["contact_pass"] = False
                row["landing_pass"] = False
        joined = join_physical_evidence_by_side(
            serve_rows,
            report,
            min_samples_per_side=4,
            min_contact_rate=0.8,
            min_landing_rate=0.8,
        )
        self.assertTrue(joined["by_side"]["forehand"]["pass"])
        self.assertFalse(joined["by_side"]["backhand"]["pass"])
        self.assertFalse(joined["pass"])

    def test_v17_qualification_requires_11_contacts_10_landings_and_five_per_side(
        self,
    ) -> None:
        accumulator = PhysicalEvidenceAccumulator(range(1, 13), min_samples=6)
        for shot_id in range(1, 13):
            complete_shot(accumulator, shot_id)
        report = accumulator.report()
        serve_rows = [
            {
                "shot_id": shot_id,
                "command_side": "forehand" if shot_id <= 6 else "backhand",
            }
            for shot_id in range(1, 13)
        ]
        joined = join_physical_evidence_by_side(
            serve_rows,
            report,
            min_samples_per_side=6,
            exact_samples_per_side=6,
            min_contact_rate=5.0 / 6.0,
            min_landing_rate=5.0 / 6.0,
            min_contacts_per_side=5,
            min_landings_per_side=5,
            min_global_contacts=11,
            min_global_landings=10,
        )
        self.assertTrue(joined["pass"])
        self.assertEqual(joined["global_contacts"], 12)
        self.assertEqual(joined["global_legal_landings"], 12)

        # Five contacts on each side satisfy the side floor but only ten globally;
        # the 11/12 global contact requirement must still fail.
        for shot_id in (1, 7):
            report["rows"][shot_id - 1]["contact_pass"] = False
            report["rows"][shot_id - 1]["landing_pass"] = False
        joined = join_physical_evidence_by_side(
            serve_rows,
            report,
            min_samples_per_side=6,
            exact_samples_per_side=6,
            min_contact_rate=5.0 / 6.0,
            min_landing_rate=5.0 / 6.0,
            min_contacts_per_side=5,
            min_landings_per_side=5,
            min_global_contacts=11,
            min_global_landings=10,
        )
        self.assertFalse(joined["global_counts_pass"])
        self.assertFalse(joined["pass"])

    def test_missing_side_duplicate_or_missing_shot_fails_closed(self) -> None:
        accumulator = PhysicalEvidenceAccumulator(range(1, 9), min_samples=6)
        for shot_id in range(1, 9):
            complete_shot(accumulator, shot_id)
        report = accumulator.report()
        rows = [
            {
                "shot_id": shot_id,
                "command_side": "forehand" if shot_id <= 4 else "backhand",
            }
            for shot_id in range(1, 9)
        ]
        for broken in (
            rows[:-1],
            [*rows[:-1], {"shot_id": 7, "command_side": "backhand"}],
            [{**rows[0], "command_side": None}, *rows[1:]],
        ):
            with self.subTest(rows=broken):
                joined = join_physical_evidence_by_side(
                    broken,
                    report,
                    min_samples_per_side=4,
                    min_contact_rate=0.8,
                    min_landing_rate=0.8,
                )
                self.assertFalse(joined["pass"])


class Gate3RunnerContractTest(unittest.TestCase):
    def _validate(self, args: str) -> subprocess.CompletedProcess[str]:
        helper = SCRIPTS / "pp_gate3_runner_common.sh"
        return subprocess.run(
            [
                "bash",
                "-c",
                f"source {helper!s}; rally_assert_policy_native_runner_args {args}",
            ],
            text=True,
            capture_output=True,
            check=False,
        )

    def test_production_policy_native_argv_is_accepted(self) -> None:
        result = self._validate(
            "--planner --policy-native --start passive --official-stand"
        )
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_manual_side_demo_and_recovery_overrides_are_rejected(self) -> None:
        for override in ("--side fh", "--demo", "--hold-recover 6.0"):
            with self.subTest(override=override):
                result = self._validate(
                    "--planner --policy-native --start passive "
                    f"--official-stand {override}"
                )
                self.assertNotEqual(result.returncode, 0)


class Gate3StaticPlantContractTest(unittest.TestCase):
    def test_ball_command_has_no_side_and_state_has_edge_counters(self) -> None:
        messages = (
            REPO_ROOT
            / "a3_deploy/A3_MuJoCo_Sim/aimrt_mujoco_sim/src/protocols/"
            "mujoco_sim_msgs/msg"
        )
        command = (messages / "Gate3BallCommand.msg").read_text()
        state = (messages / "Gate3BallState.msg").read_text()
        command_fields = [
            line.strip().split()[-1]
            for line in command.splitlines()
            if line.strip() and not line.lstrip().startswith("#")
        ]
        self.assertNotIn("side", command_fields)
        self.assertIn("uint64 shot_id", command)
        self.assertIn("uint32 racket_contact_count", state)
        self.assertIn("uint32 table_contact_count", state)
        self.assertIn("uint32 net_contact_count", state)

    def test_plant_commands_are_idempotent_and_monotonic(self) -> None:
        subscriber = (
            REPO_ROOT
            / "a3_deploy/A3_MuJoCo_Sim/aimrt_mujoco_sim/src/module/"
            "mujoco_sim_module/subscriber/gate3_ball_command_ros2_subscriber.cc"
        ).read_text()
        self.assertIn("msg->shot_id == current_shot_id", subscriber)
        self.assertIn("msg->active == current_active", subscriber)
        self.assertIn("msg->shot_id != current_shot_id + 1", subscriber)
        self.assertIn("each shot must be parked before the next", subscriber)

    def test_arena_ball_mass_and_floor_collision_contract(self) -> None:
        model = (
            REPO_ROOT
            / "a3_deploy/A3_MuJoCo_Sim/aimrt_mujoco_sim/src/models/bin/cfg/model/"
            "a3_pingpong/a3_pingpong.xml"
        ).read_text()
        self.assertIn('name="gate3_ball_collision"', model)
        self.assertIn('mass="0.0034"', model)
        self.assertIn('name="floor" size="300 300 0.125"', model)
        self.assertIn('contype="4" conaffinity="1"', model)
        for target in (
            "gate3_table_collision",
            "gate3_net_collision",
            "right_racket_collision",
        ):
            self.assertIn(
                f'geom1="gate3_ball_collision" geom2="{target}"', model
            )

    def test_public_gate3_has_one_joined_physical_verdict(self) -> None:
        conductor = (SCRIPTS / "pp_rally_conductor.py").read_text()
        engine = (SCRIPTS / "pp_gate3_rally.sh").read_text()
        self.assertIn('"gate_name": "Gate3"', conductor)
        self.assertIn('"selected_gate_verdict": "certification"', conductor)
        self.assertNotIn("Gate3CherryPick", conductor)
        self.assertNotIn("cherry_pick", conductor)
        self.assertIn("Gate3 — autonomous end-to-end", engine)
        self.assertIn("MIN_PHYSICAL_SAMPLES_PER_SIDE < 4", conductor)
        self.assertIn("REQUIRED_GLOBAL_CONTACTS = 11", conductor)
        self.assertIn("REQUIRED_GLOBAL_LANDINGS = 10", conductor)
        self.assertIn("REQUIRED_GLOBAL_CONTACTS = 25", conductor)
        self.assertIn("REQUIRED_GLOBAL_LANDINGS = 24", conductor)
        self.assertIn("actual_q_faults == 0", conductor)
        self.assertNotIn('"gate_name": "Gate4"', conductor)

    def test_gate3_uses_full_calibrated_base_path_without_identity_bypass(
        self,
    ) -> None:
        engine = (SCRIPTS / "pp_gate3_rally.sh").read_text()
        self.assertIn(
            "ros2 launch hope_bringup hope_world.launch.py", engine
        )
        self.assertIn(
            "--world-config '$WS/src/hope_bringup/config/hope_world_frame.yaml'",
            engine,
        )
        self.assertIn(
            "-p base_pose_flat_input_topic:=/a3/base_pose_flat", engine
        )
        self.assertIn("hope_ball_flight_packetizer", engine)
        self.assertIn("hope_planner_cpp_node", engine)
        self.assertIn("flight_packet_input_enabled:=true", engine)
        self.assertNotIn("ros2 run hope_planner hope_planner_node", engine)
        self.assertIn(
            'cmp -s "$WORLD_CONFIG_SOURCE" "$WORLD_CONFIG_INSTALL"', engine
        )
        self.assertIn(
            'cmp -s "$WORLD_LAUNCH_SOURCE" "$WORLD_LAUNCH_INSTALL"', engine
        )
        self.assertNotIn(
            "python3 -m hope_planner.base_pose_flat_relay", engine
        )
        self.assertNotIn(
            "marker_to_base_xyz:='[0.0, 0.0, 0.0]'", engine
        )

    def test_v17_runner_executes_the_training_action_and_hard_limit_contract(self) -> None:
        include = (
            ROOT
            / "src/a3/a3_deploy_onnx_ref/include/a3_pingpong"
        )
        onnx = (include / "pp_onnx_policy.hpp").read_text()
        policy = (include / "pp_policy.hpp").read_text()
        self.assertIn('"qdes_safe_lower_rad"', onnx)
        self.assertIn('"qdes_safe_upper_rad"', onnx)
        self.assertIn('"qdes_hard_lower_rad"', onnx)
        self.assertIn('"qdes_hard_upper_rad"', onnx)
        self.assertIn("ComputeV11AffineSafeQdes(", onnx)
        self.assertIn("has_safe_qdes_interval_contract()", policy)
        self.assertIn("qdes_actual_q_hard_tolerance_rad()", policy)
        self.assertIn("classify_actual_q_hard_limit(", policy)
        self.assertIn("actual_q_hard_limit_audit_only(", policy)


if __name__ == "__main__":
    unittest.main()
