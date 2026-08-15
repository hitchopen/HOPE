"""Host-only tests for the autonomous physical Gate3 contract."""

from __future__ import annotations

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
)


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
        (False, (0.0, 0.0, -10.0), (0.0, 0.0, 0.0), 0, 0, 0),
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
            (False, (0.0, 0.0, -10.0), (0.0, 0.0, 0.0), 0, 0, 0),
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
