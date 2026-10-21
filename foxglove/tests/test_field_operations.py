import ast
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "foxglove/a3"), str(ROOT / "hope_ws/src/hope_bringup/scripts")]

from hope_field_assets import calibration_request, csv_request
from hope_field_operations import FieldOperations, ManagedRecorder
from hope_lifecycle_core import HelperEvent, LifecycleConfig, parse_helper_event, validate_session_id


def receipt(side="P1", profile="v2"):
    return json.dumps({"approved": True, "cad": {"marker_layout": profile},
        "runtime_world": {"table_side": side}, "source": {"rigid_body_id": 15},
        "p1_to_pelvis": {"parent_frame": f"UCB_{side}", "child_frame": "pelvis_link",
            "translation_m": [0, 0, 0.1], "quaternion_xyzw": [0, 0, 0, 1]}}, indent=2) + "\n"


class FieldAssetsTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.calls = []
        self.config = LifecycleConfig(revision=1, table_side="P1")

    def exchange(self, host, config, request):
        self.calls.append((host, request["op"]))
        if host == "laptop":
            return calibration_request(self.root / "laptop", request)
        return csv_request(self.root / "mdu", Path("/fixed/runner"), request, run=self.validate)

    def validate(self, argv, **kwargs):
        self.assertEqual(argv[:2], ["/fixed/runner", "--validate-serve-timeline"])
        valid = Path(argv[2]).read_text() == "qualified CSV"
        return SimpleNamespace(returncode=0 if valid else 2,
            stdout="SERVE_TIMELINE_VALID" if valid else "", stderr="bad trajectory" if not valid else "")

    def manager(self):
        return FieldOperations(self.root / "hdu", self.exchange,
            recorder=ManagedRecorder(self.root / "recordings"))

    def test_annotated_csv_requires_segment_validator_and_preserves_metadata(self):
        content = (ROOT / "a3_deploy/a3_deploy_example/assets/a3_runtime/serve/archive/a3p_op3_serve025_v21_segments_model14600.csv").read_text()
        request = {"op": "upload_csv", "name": "v21.csv", "content": content}
        def old_validator(*args, **kwargs):
            return SimpleNamespace(returncode=0, stdout="SERVE_TIMELINE_VALID", stderr="")
        with self.assertRaisesRegex(ValueError, "annotated serve-segment update"):
            csv_request(self.root, Path("/runner"), request, run=old_validator)
        def validator(*args, **kwargs):
            return SimpleNamespace(returncode=0, stdout="SERVE_SEGMENTS_VALID\nSERVE_TIMELINE_VALID", stderr="")
        result = csv_request(self.root, Path("/runner"), request, run=validator)
        self.assertEqual(result["segments"]["branch_frame"], 146)
        self.assertEqual(result["segments"]["clips"]["SERVE_TO_POLICY"], {"frames": 81, "duration_s": .8})
        verified = csv_request(self.root, Path("/runner"), {"op": "verify_csv", "sha256": result["sha256"]}, run=validator)
        self.assertEqual(verified["segments"], result["segments"])
        with self.assertRaisesRegex(ValueError, "annotated serve-segment update"):
            csv_request(self.root, Path("/runner"), {"op": "verify_csv", "sha256": result["sha256"]}, run=old_validator)

    def test_calibration_round_trip_survives_restart_and_failed_load(self):
        field = self.manager()
        field.command({"op": "load_calibration", "name": "venue-A.json", "content": receipt()}, self.config, stopped=True)
        restarted = self.manager()
        self.assertEqual(restarted.status()["calibration"]["name"], "venue-A.json")
        self.assertEqual(restarted.status()["recording"]["state"], "OFF")
        for bad in (receipt("P2"), receipt(profile="v9"), '{"approved": false}'):
            with self.assertRaises(ValueError):
                restarted.command({"op": "load_calibration", "name": "bad.json", "content": bad}, self.config, stopped=True)
        exported = restarted.command({"op": "export_calibration"}, self.config, stopped=False)
        self.assertEqual(exported, {"name": "venue-A.json", "content": receipt()})
        self.assertTrue(all(op in {"read_calibration", "load_calibration"} for _, op in self.calls))

    def test_v23_frame_event_contract_metadata(self):
        from hope_field_assets import csv_segments_metadata
        content = (ROOT / "a3_deploy/a3_deploy_example/assets/a3_runtime/serve/motions/a3p_op3_serve025_v23_branch110_model14600.csv").read_text()
        metadata = csv_segments_metadata(content)
        self.assertIn(",FRAME,SERVE_TO_POLICY,110,110,", content)
        self.assertEqual(metadata["branch_frame"], 110)
        self.assertEqual(set(metadata["clips"]), {"STAND_TO_SERVE", "SERVE_TO_STAND", "SERVE_TO_POLICY"})
        self.assertEqual(metadata["clips"]["SERVE_TO_POLICY"], {"frames": 87, "duration_s": .86})

    def test_explicit_calibration_capture_is_adopted_without_new_session_fit(self):
        active = self.root / "laptop/ucb_robot_to_pelvis.json"
        active.parent.mkdir()
        active.write_text(receipt())
        field = self.manager()
        field.refresh(self.config)
        before = field.status()["calibration"]["sha256"]
        active.write_text(receipt(profile="v3"))
        field.refresh(self.config)
        self.assertNotEqual(field.status()["calibration"]["sha256"], before)
        self.assertTrue((self.root / "hdu/calibrations" / (before + ".json")).exists())
        self.assertEqual(field.status()["calibration"]["profile"], "v3")

    def test_legacy_pure_serve_selection_migrates_to_kernel(self):
        import json
        from hope_field_operations import FieldOperations
        root = self.root / "legacy_field"
        root.mkdir()
        (root / "selection.json").write_text(json.dumps({
            "mode": "PURE_SERVE", "csv": None, "calibration": None,
        }))
        field = FieldOperations(root, lambda *args: {})
        self.assertEqual(field.status()["mode"], "KERNEL")
        self.assertEqual(field.status()["error"], "")
        field.command({"op": "set_mode", "mode": "PURE_SERVE"}, self.config, stopped=True)
        self.assertEqual(json.loads((root / "selection.json").read_text())["mode"], "KERNEL")

    def test_csv_upload_validation_failure_preserves_selection_and_freezes_while_running(self):
        field = self.manager()
        field.command({"op": "set_mode", "mode": "KERNEL"}, self.config, stopped=True)
        field.command({"op": "upload_csv", "name": "serve.csv", "content": "qualified CSV"}, self.config, stopped=True)
        before = field.status()["csv"]
        with self.assertRaisesRegex(ValueError, "CSV rejected"):
            field.command({"op": "upload_csv", "name": "invalid.csv", "content": "bad"}, self.config, stopped=True)
        self.assertEqual(field.status()["csv"], before)
        self.assertEqual(self.manager().runner_selection(self.config), ("KERNEL", before["sha256"]))
        for request in ({"op": "set_mode", "mode": "OPTITRACK"}, {"op": "use_default_csv"},
                        {"op": "load_calibration", "name": "x.json", "content": receipt()}):
            with self.assertRaisesRegex(ValueError, "stop the Runner"):
                field.command(request, self.config, stopped=False)
        Path(before["path"]).write_text("tampered")
        with self.assertRaisesRegex(ValueError, "changed on the MDU"):
            field.runner_selection(self.config)
        self.assertEqual(field.status()["recording"]["state"], "OFF")

    def test_mode_switch_preserves_and_verifies_the_selected_csv(self):
        def exchange(host, config, request):
            if request["op"] == "verify_ball_model":
                return {}
            return self.exchange(host, config, request)
        field = FieldOperations(self.root / "hdu", exchange,
            recorder=ManagedRecorder(self.root / "recordings"))
        field.command({"op": "upload_csv", "name": "serve.csv", "content": "qualified CSV"},
                      self.config, stopped=True)
        selected = field.status()["csv"]
        for mode in ("KERNEL", "OPTITRACK", "KERNEL"):
            with self.subTest(mode=mode):
                field.command({"op": "set_mode", "mode": mode}, self.config, stopped=True)
                self.assertEqual(field.runner_selection(self.config), (mode, selected["sha256"]))
                self.assertEqual(field.status()["csv"], selected)
                path = Path(selected["path"])
                path.write_text("tampered")
                with self.assertRaisesRegex(ValueError, "changed on the MDU"):
                    field.runner_selection(self.config)
                path.write_text("qualified CSV")
                path.unlink()
                with self.assertRaises((ValueError, FileNotFoundError)):
                    field.runner_selection(self.config)
                path.write_text("qualified CSV")
        field.command({"op": "use_default_csv"}, self.config, stopped=True)
        for mode in ("KERNEL", "OPTITRACK"):
            field.command({"op": "set_mode", "mode": mode}, self.config, stopped=True)
            self.assertEqual(field.runner_selection(self.config), (mode, "DEFAULT"))

    def test_helper_passes_same_csv_and_checks_bytes_in_both_modes(self):
        helper = (ROOT / "foxglove/helpers/hope-lifecycle").read_text()
        block = helper[helper.index("  local -a field_runner_args="):
                       helper.index('  if [[ -d "$PWD/teleop_humanlike" ]]')]
        block = block.replace("/agibot/hope-field-assets", str(self.root))
        import hashlib
        digest = hashlib.sha256(b"qualified CSV").hexdigest()
        path = self.root / (digest + ".csv")
        script = ('set -euo pipefail\ndie() { echo "$2" >&2; exit 1; }\n'
                  'probe() {\n' + block + '\nprintf "%s\\n" "${field_runner_args[@]}";\n}\n'
                  'FIELD_MODE="$1"; SERVE_CSV_SHA="$2"; MDU_DEPLOY=/fixed/package; probe')
        for mode in ("KERNEL", "OPTITRACK"):
            for state in ("valid", "tampered", "missing", "default"):
                with self.subTest(mode=mode, state=state):
                    path.write_text("qualified CSV" if state == "valid" else "tampered")
                    if state == "missing":
                        path.unlink()
                    result = subprocess.run(["bash", "-c", script, "test", mode,
                                             "DEFAULT" if state == "default" else digest],
                                            capture_output=True, text=True, timeout=5)
                    self.assertEqual(result.returncode == 0, state in {"valid", "default"}, result.stderr)
                    if state == "valid":
                        self.assertEqual(result.stdout.splitlines()[-2:], ["--serve-timeline", str(path)])
                    elif state == "default":
                        self.assertEqual(result.stdout.splitlines()[-2:], [
                            "--serve-timeline",
                            "/fixed/package/motions/a3p_op3_serve025_v23_branch110_model14600.csv"])
                    else:
                        self.assertIn("SELECTED_CSV_", result.stderr)

    def test_untrusted_filename_cannot_escape_fixed_store(self):
        for name in ("../outside.csv", "/tmp/file.csv", "foo\\bar.csv", "x\n.csv"):
            with self.assertRaises(ValueError):
                self.exchange("mdu", self.config, {"op": "upload_csv", "name": name, "content": "qualified CSV"})


class RecorderTest(unittest.TestCase):
    def test_default_off_start_and_stop_reap_only_owned_writer(self):
        with tempfile.TemporaryDirectory() as temporary:
            processes = []
            def popen(argv, **kwargs):
                self.assertEqual(argv, ["/usr/local/bin/hope-pingpong-telemetry-record"])
                child = subprocess.Popen([sys.executable, "-u", "-c",
                    "import os,pathlib,time\n"
                    "p=pathlib.Path(os.environ['HOPE_RECORD_OUTPUT']); p.mkdir()\n"
                    "with (p/'test.mcap').open('ab',buffering=0) as f:\n"
                    " while True: f.write(b'frame'); time.sleep(.01)\n"], **kwargs)
                processes.append(child)
                return child
            recorder = ManagedRecorder(Path(temporary), popen=popen)
            self.addCleanup(recorder.stop)
            self.assertEqual(recorder.status()["state"], "OFF")
            self.assertEqual(processes, [])
            recorder.start()
            deadline = time.monotonic() + 3
            while recorder.status()["state"] != "ON" and time.monotonic() < deadline:
                time.sleep(.02)
            self.assertEqual(recorder.status()["state"], "ON")
            recorder.start()
            self.assertEqual(len(processes), 1)
            recorder.stop()
            self.assertEqual(recorder.status()["state"], "OFF")
            self.assertIsNotNone(processes[0].poll())
            output = Path(recorder.output) / "test.mcap"
            size = output.stat().st_size
            time.sleep(.05)
            self.assertEqual(output.stat().st_size, size)


class LifecycleModeTest(unittest.TestCase):
    def backend(self, calls):
        # Execute the real backend in isolation from ROS, with an argv recorder.
        source = ROOT / "foxglove/a3/hope_lifecycle_supervisor.py"
        tree = ast.parse(source.read_text())
        classes = [node for node in tree.body if isinstance(node, ast.ClassDef)
                   and node.name in {"LifecycleBackend", "LifecycleFailure"}]
        namespace = dict(subprocess=subprocess, LifecycleConfig=LifecycleConfig,
            HelperEvent=HelperEvent, validate_session_id=validate_session_id,
            parse_helper_event=parse_helper_event, LIFECYCLE_HELPER="/fixed/helper",
            LAPTOP_LIFECYCLE_HELPER="/fixed/laptop-helper",
            LAPTOP_USER="operator", ROBOT_USER="agi", SSH_OPTIONS=("-T",), json=json)
        exec(compile(ast.Module(body=classes, type_ignores=[]), str(source), "exec"), namespace)
        def run(argv, **kwargs):
            calls.append(argv)
            return SimpleNamespace(returncode=0, stdout="", stderr="")
        return namespace["LifecycleBackend"](run=run)

    def test_pure_start_and_stop_need_no_laptop_base_planner_or_mocap(self):
        config = LifecycleConfig(laptop_wifi_ip="10.0.0.2", hdu_wifi_ip="10.0.0.3",
            mdu_internal_ip="10.42.10.12", motive_ip="", table_side="P1", revision=1)
        calls = []
        backend = self.backend(calls)
        session = "model21800_20260912T010203Z"
        backend.start(config, session, lambda event: None, "KERNEL", "a" * 64)
        self.assertEqual(len(calls), 7)
        for argv in calls:
            self.assertNotIn("operator@10.0.0.2", argv)
            self.assertNotIn("start-base", argv)
            self.assertNotIn("start-planner", argv)
            self.assertEqual(argv[-3:], ["KERNEL", "a" * 64, "legacy"])
        calls.clear()
        backend.kill_all_and_collect(config, session, lambda event: None, "KERNEL")
        self.assertEqual(len(calls), 2)
        self.assertFalse(any("collect" in argv for argv in calls))
        calls.clear()
        backend.start(config, session, lambda event: None)
        self.assertTrue(any("start-base" in argv for argv in calls))
        self.assertTrue(any("start-planner" in argv for argv in calls))


if __name__ == "__main__":
    unittest.main()
