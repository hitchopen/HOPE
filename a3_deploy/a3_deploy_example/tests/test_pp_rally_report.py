#!/usr/bin/env python3
"""Host-only contract tests for the Gate3 rally phase report."""

from __future__ import annotations

import contextlib
import csv
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest


REPORT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "pp_rally_report.py"
SPEC = importlib.util.spec_from_file_location("pp_rally_report", REPORT_PATH)
REPORT = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(REPORT)


class TestPpRallyReport(unittest.TestCase):
    def test_final_v3_windows_match_training_contract_and_legacy_is_unchanged(self):
        self.assertEqual(
            REPORT.WINDOWS["legacy"],
            {
                "pre": (0.12, 0.45),
                "post": (0.20, 1.00),
                "ready_heading": (0.45, 1.40),
                "post_heading": (0.35, 1.80),
            },
        )
        self.assertEqual(
            REPORT.WINDOWS["rally_final_v3"],
            {
                "pre": (0.12, 1.10),
                "post": (0.20, 1.55),
                "ready_heading": (0.45, 1.10),
                "post_heading": (0.20, 1.55),
            },
        )

    def test_side_is_runner_engage_order_not_velocity_sign(self):
        report = {
            "engaged": 2,
            "rows": [
                {"serve": 1, "engaged": None},
                {"serve": 2, "engaged": {"side": "backhand"}},
                {"serve": 3, "engaged": {"side": "forehand"}},
            ],
        }
        self.assertEqual(REPORT.engaged_sides_from_report(report), (["bh", "fh"], []))
        source = REPORT_PATH.read_text()
        self.assertNotIn('"fh" if rv[1]', source)

    def test_schema_v2_preserves_every_engage_event(self):
        report = {
            "total_engage_events": 3,
            "rows": [
                {"serve": 1, "engages": [
                    {"side": "forehand"}, {"side": "backhand"}
                ]},
                {"serve": 2, "engages": [{"side": "forehand"}]},
            ],
        }
        self.assertEqual(
            REPORT.engaged_sides_from_report(report), (["fh", "bh", "fh"], [])
        )

    def test_runner_clamp_stats_uses_executed_q_des_telemetry(self):
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "runner.log"
            log.write_text(
                "[status] mode=MOTION maxact=99.0 clamp=0\n"
                "[status] mode=MOTION maxact=1.0 clamp=2\n"
                "[pp WARN] q_des clamped to joint limits on 2 joint(s)\n"
            )
            self.assertEqual(REPORT.runner_clamp_stats(log), {
                "samples": 2, "nonzero_samples": 1, "peak": 2, "warned": True,
                "safe_samples": 0, "safe_nonzero_samples": 0, "safe_peak": 0,
                "audit_only": False, "warning_count": 1, "joints": {},
            })

    def test_runner_clamp_stats_reports_latest_per_joint_audit(self):
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "runner.log"
            log.write_text(
                "[status] mode=MOTION clamp=1\n"
                "[clamp-audit] phase=periodic ticks=100 active=1 "
                "waist_pitch_joint=3/100/0.012000\n"
                "[clamp-audit] phase=final ticks=200 active=2 "
                "waist_pitch_joint=7/200/0.021000 "
                "right_elbow_joint=2/200/0.004500\n"
            )
            stats = REPORT.runner_clamp_stats(log)
            self.assertEqual(stats["joints"], {
                "waist_pitch_joint": {"hits": 7, "ticks": 200, "max_viol": 0.021},
                "right_elbow_joint": {"hits": 2, "ticks": 200, "max_viol": 0.0045},
            })

    def test_raw_action_magnitude_is_not_a_hard_gate(self):
        source = REPORT_PATH.read_text()
        self.assertNotIn("|last_action| < 12", source)
        self.assertIn("policy q_des stays inside safe and hard joint limits", source)

    def test_end_to_end_uses_backhand_engage_even_when_vy_is_positive(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            csv_path = directory / "obs.csv"
            with csv_path.open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(["tick", "ts", "mode", "sync_miss"] + [
                    f"obs_{index}" for index in range(110)
                ])
                for tick in range(71):
                    obs = [0.0] * 110
                    obs[98] = -1.0       # upright projected gravity
                    obs[99] = 1.0        # square base-forward x
                    obs[103] = 0.70      # FinalV3 V7 fixed station-relative racket plane
                    obs[107] = 0.50      # positive vy is intentionally not a side oracle
                    obs[109] = 0.80 - 0.02 * tick
                    writer.writerow([tick, tick * 0.02, "MOTION", 0] + obs)
            conductor_path = directory / "conductor.json"
            conductor_path.write_text(json.dumps({
                "serves": 1,
                "engaged": 1,
                "returned": 1,
                "falls": 0,
                "station_drift_m": 0.0,
                "pass": True,
                "return_rate": 1.0,
                "station_transition_coverage_ok": True,
                "rows": [{"serve": 1, "engaged": {"side": "backhand"}}],
            }))
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                rc = REPORT.main(
                    [str(csv_path), str(conductor_path), "--mode", "rally_final_v3"]
                )
            self.assertEqual(rc, 0, output.getvalue())
            swing_line = next(
                line for line in output.getvalue().splitlines() if line.startswith("   1")
            )
            self.assertIn(" bh ", swing_line)

    def test_inconsistent_conductor_mapping_is_rejected(self):
        sides, errors = REPORT.engaged_sides_from_report({
            "engaged": 3,
            "rows": [
                {"serve": 2, "engaged": {"side": "forehand"}},
                {"serve": 1, "engaged": {"side": "wrong"}},
            ],
        })
        self.assertEqual(sides, ["fh"])
        self.assertTrue(any("strictly increasing" in error for error in errors))
        self.assertTrue(any("invalid engaged side" in error for error in errors))
        self.assertTrue(any("summary engage events" in error for error in errors))

    def test_auto_mode_accepts_exact_validated_v3_runner_marker(self):
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "runner.log"
            log.write_text("[pp] hitter_pure training_recipe=rally_final_v3\n")
            self.assertEqual(REPORT.resolve_mode("auto", log), "rally_final_v3")
            with self.assertRaisesRegex(ValueError, "contradicts"):
                REPORT.resolve_mode("legacy", log)

    def test_auto_mode_accepts_rally_v9_runner_marker_and_windows(self):
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "runner.log"
            log.write_text("[pp] hitter_pure training_recipe=rally_v9\n")
            self.assertEqual(REPORT.resolve_mode("auto", log), "rally_v9")
            self.assertEqual(REPORT.WINDOWS["rally_v9"]["pre"], (0.12, 0.96))
            with self.assertRaisesRegex(ValueError, "contradicts"):
                REPORT.resolve_mode("rally_v8", log)

    def test_auto_mode_accepts_rally_v10_and_enables_wrist_gate(self):
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "runner.log"
            log.write_text(
                "[pp] hitter_pure runtime_contract=rally_final_v2\n"
                "[pp] hitter_pure training_recipe=rally_v10\n"
            )
            self.assertEqual(REPORT.resolve_mode("auto", log), "rally_v10")
            self.assertEqual(REPORT.WINDOWS["rally_v10"]["pre"], (0.12, 1.10))
            self.assertIn(
                'idle {joint_name} range <= {idle_wrist_budget:.2f} rad',
                REPORT_PATH.read_text(),
            )
            self.assertIn(
                "contact right-elbow q p90 <= 1.35 rad", REPORT_PATH.read_text()
            )

    def test_auto_mode_accepts_rally_v11_and_extends_post_heading_window(self):
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "runner.log"
            log.write_text(
                "[pp] hitter_pure runtime_contract=rally_final_v2\n"
                "[pp] hitter_pure training_recipe=rally_v11\n"
            )
            self.assertEqual(REPORT.resolve_mode("auto", log), "rally_v11")
            self.assertEqual(REPORT.WINDOWS["rally_v11"], {
                "pre": (0.12, 1.10),
                "post": (0.20, 1.20),
                "ready_heading": (0.45, 1.00),
                "post_heading": (0.20, 1.55),
            })

    def test_auto_mode_accepts_rally_v12_with_same_gate_windows(self):
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "runner.log"
            log.write_text(
                "[pp] hitter_pure runtime_contract=rally_final_v2\n"
                "[pp] hitter_pure training_recipe=rally_v12\n"
            )
            self.assertEqual(REPORT.resolve_mode("auto", log), "rally_v12")
            self.assertEqual(REPORT.WINDOWS["rally_v12"], REPORT.WINDOWS["rally_v11"])

    def test_auto_mode_accepts_rally_v14_with_v13_gate_windows(self):
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "runner.log"
            log.write_text(
                "[pp] hitter_pure runtime_contract=rally_final_v2\n"
                "[pp] hitter_pure training_recipe=rally_v14\n"
            )
            self.assertEqual(REPORT.resolve_mode("auto", log), "rally_v14")
            self.assertEqual(REPORT.WINDOWS["rally_v14"], REPORT.WINDOWS["rally_v13"])

    def test_auto_mode_accepts_only_paired_rally_v17_and_scores_full_recovery(self):
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "runner.log"
            log.write_text(
                "[pp] hitter_pure runtime_contract=rally_final_v2\n"
                "[pp] hitter_pure training_recipe=rally_v17\n"
            )
            self.assertEqual(REPORT.resolve_mode("auto", log), "rally_v17")
            self.assertEqual(REPORT.WINDOWS["rally_v17"], {
                "pre": (0.12, 1.10),
                "post": (0.10, 1.55),
                "ready_heading": (0.45, 1.00),
                "post_heading": (0.20, 1.55),
            })
            log.write_text("[pp] hitter_pure training_recipe=rally_v17\n")
            with self.assertRaisesRegex(ValueError, "rally_final_v2"):
                REPORT.resolve_mode("rally_v17", log)

    def test_auto_mode_distinguishes_recipe10_fixed_station_runtime(self):
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "runner.log"
            log.write_text(
                "[pp] hitter_pure runtime_contract="
                "rally_v17_fixed_station_ball_clock_v1\n"
                "[pp] hitter_pure training_recipe=rally_v17 "
                "(stay-if-reachable ON: fh y band [-0.48,-0.40] "
                "bh [-0.13,-0.05] about the held station)\n"
                "[v17-r10-gate3] PROFILE ACCEPTED: immutable session station, "
                "schema-2 three-revision planner, ball-clock release, frozen target; "
                "x86 simulation only, hardware_authorized=false\n"
            )
            self.assertEqual(
                REPORT.resolve_mode("auto", log), "rally_v17_r10"
            )
            self.assertEqual(
                REPORT.WINDOWS["rally_v17_r10"], REPORT.WINDOWS["rally_v17"]
            )
            log.write_text(
                "[pp] hitter_pure runtime_contract="
                "rally_v17_fixed_station_ball_clock_v1\n"
                "[pp] hitter_pure training_recipe=rally_v17\n"
            )
            with self.assertRaisesRegex(ValueError, "isolated runner-validated"):
                REPORT.resolve_mode("rally_v17_r10", log)

    def test_auto_mode_accepts_only_paired_rally_v15_marker(self):
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "runner.log"
            log.write_text(
                "[pp] hitter_pure runtime_contract=rally_v15\n"
                "[pp] hitter_pure training_recipe=rally_v15\n"
            )
            self.assertEqual(REPORT.resolve_mode("auto", log), "rally_v15")
            self.assertEqual(REPORT.WINDOWS["rally_v15"], REPORT.WINDOWS["rally_v14"])
            log.write_text("[pp] hitter_pure training_recipe=rally_v15\n")
            with self.assertRaisesRegex(ValueError, "paired rally_v15"):
                REPORT.resolve_mode("rally_v15", log)

    def test_v15_gait_envelope_comes_from_validated_runner_banner(self):
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "runner.log"
            log.write_text(
                "[pp] V15 finite gait from ONNX/YAML: freq=1.50 Hz duty=0.50 "
                "deadband=0.12 m step=0.30 m cycles<=1 |vy|<=0.45 m/s; "
                "intervention deploy value=0\n"
            )
            self.assertEqual(REPORT.discover_finite_gait_from_runner(log), {
                "frequency_hz": 1.5,
                "duty_factor": 0.5,
                "move_deadband": 0.12,
                "step_distance": 0.3,
                "max_cycles": 1,
                "velocity_max": 0.45,
                "deploy_intervention": 0.0,
            })
            log.write_text(log.read_text() + log.read_text().replace("0.45", "0.40"))
            self.assertIsNone(REPORT.discover_finite_gait_from_runner(log))

    def test_rally_v10_report_rejects_recipe_without_runtime_v2_capability(self):
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "runner.log"
            log.write_text("[pp] hitter_pure training_recipe=rally_v10\n")
            with self.assertRaisesRegex(ValueError, "cannot prove"):
                REPORT.resolve_mode("auto", log)
            with self.assertRaisesRegex(ValueError, "rally_final_v2"):
                REPORT.resolve_mode("rally_v10", log)

    def test_component_recipe_missing_joint_defaults_fails_once_not_once_per_swing(self):
        source = REPORT_PATH.read_text()
        self.assertIn(
            '"loaded policy bundle exposes joint defaults for V10--V17 elbow gate"', source
        )
        self.assertGreaterEqual(source.count("joint_defaults is not None"), 5)
        self.assertIn('"rally_v15", "rally_v17"', source)

    def test_v15_projector_trace_reports_intervention_and_infeasibility(self):
        with tempfile.TemporaryDirectory() as directory:
            trace = Path(directory) / "runner.csv"
            fields = [
                "mode", "qdes_projector_active", "qdes_projector_rate",
                "qdes_projector_tracking", "qdes_projector_torque",
                "qdes_projector_infeasible", "qdes_projector_max_norm_debt",
            ]
            with trace.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                writer.writerow(dict(zip(fields, ["1", 0, 0, 0, 0, 0, 0.0])))
                writer.writerow(dict(zip(fields, ["3", 4, 3, 1, 2, 0, 0.25])))
                writer.writerow(dict(zip(fields, ["3", 2, 1, 0, 1, 1, 0.50])))
            stats = REPORT.qdes_projector_trace_stats(trace)
            self.assertEqual(stats["rows"], 2)
            self.assertEqual(stats["infeasible_ticks"], 1)
            self.assertEqual(stats["infeasible_peak"], 1)
            self.assertAlmostEqual(stats["joint_fractions"]["active"], 6.0 / 62.0)
            self.assertEqual(stats["max_norm_debt"], 0.5)

    def test_elbow_contact_window_falls_back_to_nearest_strike_tick(self):
        tts = [0.20, 0.08, -0.06, -0.20]
        self.assertEqual(REPORT.contact_window_indices(tts, 0, 3, 2), [2])
        tts = [0.20, 0.02, -0.01, -0.20]
        self.assertEqual(REPORT.contact_window_indices(tts, 0, 3, 2), [1, 2])

    def test_joint_defaults_come_from_loaded_onnx_metadata(self):
        try:
            import onnx
            from onnx import helper
        except ImportError:
            self.skipTest("onnx package unavailable")
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            model = helper.make_model(helper.make_graph([], "empty", [], []))
            for key, value in (
                ("joint_names", ",".join(REPORT.JOINT_NAMES)),
                ("default_joint_pos", ",".join("0.8" for _ in REPORT.JOINT_NAMES)),
            ):
                entry = model.metadata_props.add()
                entry.key = key
                entry.value = value
            onnx.save(model, directory / "policy.onnx")
            log = directory / "runner.log"
            log.write_text(
                "[pingpong] A3AimrtBackend initialised; model=policy.onnx\n"
            )
            defaults = REPORT.discover_joint_defaults_from_runner(log, directory)
            self.assertIsNotNone(defaults)
            self.assertEqual(defaults["right_elbow_joint"], 0.8)

    def test_joint_default_runner_banner_works_without_python_onnx(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            log = directory / "runner.log"
            log.write_text(
                "[pingpong] A3AimrtBackend initialised; model=models/policy.onnx\n"
                "[pp] hitter_pure joint_default right_elbow_joint=0.812500\n"
            )
            defaults = REPORT.discover_joint_defaults_from_runner(log, directory)
            self.assertEqual(defaults, {"right_elbow_joint": 0.8125})

    def test_joint_default_unitree_policy_directory_banner(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            log = directory / "runner.log"
            log.write_text(
                "[pingpong] A3AimrtBackend initialised; "
                "model=policy/exported/policy.onnx "
                "deploy_cfg=policy/params/deploy.yaml policy_dir=policy\n"
                "[pp] hitter_pure joint_default right_elbow_joint=0.800000\n"
            )
            defaults = REPORT.discover_joint_defaults_from_runner(log, directory)
            self.assertEqual(defaults, {"right_elbow_joint": 0.8})

    def test_conflicting_joint_default_runner_banners_fail_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            log = directory / "runner.log"
            log.write_text(
                "[pingpong] A3AimrtBackend initialised; model=models/policy.onnx\n"
                "[pp] hitter_pure joint_default right_elbow_joint=0.812500\n"
                "[pp] hitter_pure joint_default right_elbow_joint=0.900000\n"
            )
            self.assertIsNone(REPORT.discover_joint_defaults_from_runner(log, directory))

    def test_auto_mode_uses_loaded_onnx_metadata_for_existing_v2(self):
        try:
            import onnx
            from onnx import helper
        except ImportError:
            self.skipTest("onnx package unavailable")
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            model = helper.make_model(helper.make_graph([], "empty", [], []))
            entry = model.metadata_props.add()
            entry.key = "hitter_pure_training_recipe"
            entry.value = "rally_final_v2"
            entry = model.metadata_props.add()
            entry.key = "hitter_pure_training_recipe_version"
            entry.value = "2"
            onnx.save(model, directory / "policy.onnx")
            log = directory / "runner.log"
            log.write_text(
                "[pingpong] A3AimrtBackend initialised; model=policy.onnx\n"
            )
            self.assertEqual(REPORT.resolve_mode("auto", log, directory), "legacy")

    def test_auto_mode_fails_closed_without_recipe_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "runner.log"
            log.write_text("no model evidence\n")
            with self.assertRaisesRegex(ValueError, "cannot prove"):
                REPORT.resolve_mode("auto", log, directory)


if __name__ == "__main__":
    unittest.main()
