"""Hermetic Mac build-entry checks: no Docker, ROS, network, or robot needed."""
from contextlib import redirect_stderr
from io import StringIO
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("hope_macos_build_entry", ROOT / "scripts/macos/hope.py")
entry = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(entry)
INSIDE = (ROOT / "scripts/macos/inside.sh").read_text()
BUILDER = (ROOT / "a3_deploy/a3_deploy_example/scripts/build_a3_deploy_pkg.sh").read_text()


class MacBuildEntryTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="hope mac build ")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.deploy = self.root / "a3_deploy/a3_deploy_example"
        self.sim = self.root / "a3_deploy/A3_MuJoCo_Sim/aimrt_mujoco_sim"
        for source in (self.deploy, self.sim):
            source.mkdir(parents=True)
            (source / "CMakeLists.txt").touch()
        scripts = self.root / "scripts/macos"
        scripts.mkdir(parents=True)
        (scripts / "inside.sh").write_text("# test entry snapshot\n")

    def invoke(self, *arguments):
        with patch.object(entry, "ROOT", self.root), \
                patch.object(entry, "run") as run, \
                patch("sys.argv", ["hope.py", *arguments]):
            entry.main()
        return run

    def test_direct_cli_help_does_not_import_sibling_operator_as_stdlib(self):
        result = subprocess.run([sys.executable, "-B", str(ROOT / "scripts/macos/hope.py"), "--help"],
                                check=True, capture_output=True, text=True)
        self.assertIn("build-ros", result.stdout)
        self.assertIn("build-runner", result.stdout)

    def test_build_ros_keeps_main_sources_and_arch_specific_output_volume(self):
        run = self.invoke("build-ros", "--arch", "arm64", "--clean-cache")
        command = run.call_args.args[0]
        self.assertIn(f"{self.root}:/work", command)
        self.assertIn("hope-macos-arm64:/opt/hope", command)
        self.assertIn("HOPE_DEPLOY_DIR=/work/a3_deploy/a3_deploy_example", command)
        self.assertIn("HOPE_ROS_CLEAN_CACHE=1", command)
        self.assertEqual(command[-4:], ["-c", "# test entry snapshot\n", "hope-macos", "build-ros"])
        self.assertFalse((self.root / "agi").exists())

    def test_final_operator_build_builds_and_tests_locally_without_robot_actions(self):
        run = self.invoke("build-operator", "--arch", "arm64", "--jobs", "3", "--clean-cache")
        commands = [call.args[0] for call in run.call_args_list]
        self.assertEqual([command[2] for command in commands[:3]], ["image", "build-ros", "test-ros"])
        self.assertIn("--clean-cache", commands[1])
        self.assertEqual(commands[1][commands[1].index("--jobs") + 1], "3")
        self.assertEqual(commands[-1][:2], ["docker", "build"])
        self.assertIn("hope-operator:jazzy-arm64", commands[-1])
        self.assertFalse(any("ssh" in command or "operator.py" in map(str, command) for command in commands))

    def test_final_operator_build_has_explicit_build_only_mode(self):
        run = self.invoke("build-operator", "--no-tests")
        commands = [call.args[0] for call in run.call_args_list]
        self.assertEqual(len(commands), 3)
        self.assertIn("--no-tests", commands[1])
        self.assertFalse(any("test-ros" in command for command in commands))

    def test_final_operator_build_can_require_lint_as_well(self):
        run = self.invoke("build-operator", "--with-lint")
        self.assertIn("--with-lint", run.call_args_list[2].args[0])

    def test_exec_keeps_arguments_separate_and_ports_loopback_only(self):
        run = self.invoke("exec", "--publish-port", "8765", "--", "printf", "%s", "a b")
        command = run.call_args.args[0]
        self.assertIn("127.0.0.1:8765:8765", command)
        self.assertEqual(command[-3:], ["printf", "%s", "a b"])
        self.assertNotIn("--privileged", command)
        self.assertNotIn("--network", command)

    def test_crossbuild_mounts_only_existing_deployment_and_explicit_assets(self):
        archive = self.deploy / "thirdparty/rockchip_sysroot/rockchip-1.0-aarch64-sysroot.tar.gz"
        archive.parent.mkdir(parents=True)
        archive.touch()
        with patch.object(entry.subprocess, "run", return_value=subprocess.CompletedProcess([], 0)):
            run = self.invoke("build-rockchip", "--build-only")
        command = run.call_args.args[0]
        self.assertIn(f"{self.deploy}:/work", command)
        self.assertIn("--inside-docker", command)
        self.assertIn("--build-only", command)
        self.assertFalse(any("agibot_a3_Ultra_aimdk" in str(argument) for argument in command))
        self.assertFalse((self.root / "agi").exists())

    def test_invalid_crossbuild_request_fails_before_docker_or_source_discovery(self):
        with patch.object(entry, "source_directory") as source, \
                patch.object(entry, "run") as run, \
                patch.object(entry.subprocess, "run") as subprocess_run, \
                patch("sys.argv", ["hope.py", "build-rockchip"]), \
                redirect_stderr(StringIO()):
            with self.assertRaises(SystemExit) as error:
                entry.main()
        self.assertEqual(error.exception.code, 2)
        source.assert_not_called()
        run.assert_not_called()
        subprocess_run.assert_not_called()

    def test_runner_requires_amd64(self):
        with redirect_stderr(StringIO()), self.assertRaises(SystemExit) as error:
            self.invoke("build-runner", "--arch", "arm64", "--build-only")
        self.assertEqual(error.exception.code, 2)


class RosDiscoveryParityTests(unittest.TestCase):
    def paths(self, root):
        start = INSIDE.index("ros_source_paths() {")
        end = INSIDE.index("\n}", start) + 2
        script = INSIDE[start:end] + '\nros_source_paths "$1"\nprintf "%s\\n" "${source_paths[@]}"\n'
        result = subprocess.run(["bash", "-c", script, "test-ros-paths", str(root)],
                                check=True, capture_output=True, text=True)
        return result.stdout.splitlines()

    def test_older_layout_has_only_hope_workspace(self):
        with tempfile.TemporaryDirectory() as temporary:
            self.assertEqual(self.paths(Path(temporary)), ["src"])

    def test_main_layout_includes_sibling_natnet_workspace(self):
        with tempfile.TemporaryDirectory(prefix="hope sources ") as temporary:
            root = Path(temporary)
            (root / "NatNet2ROS2/src").mkdir(parents=True)
            self.assertEqual(self.paths(root), ["src", "../NatNet2ROS2/src"])

    def test_vrpn_workspace_is_discovered_alone_or_with_natnet(self):
        with tempfile.TemporaryDirectory(prefix="hope capture sources ") as temporary:
            root = Path(temporary)
            (root / "VRPN2ROS2/src").mkdir(parents=True)
            self.assertEqual(self.paths(root), ["src", "../VRPN2ROS2/src"])
            (root / "NatNet2ROS2/src").mkdir(parents=True)
            self.assertEqual(self.paths(root),
                             ["src", "../NatNet2ROS2/src", "../VRPN2ROS2/src"])

    def test_ros_tests_include_the_built_vrpn_backend(self):
        block = INSIDE.split("  test-ros)\n", 1)[1].split("  build-runner)\n", 1)[0]
        self.assertIn("test_packages+=(vrpn_mocap)", block)
        self.assertIn('--packages-select "${test_packages[@]}"', block)
        self.assertIn("--return-code-on-test-failure", block)
        self.assertIn("--ctest-args -LE linter", block)
        self.assertIn("ROS_DOMAIN_ID=229 ROS_LOCALHOST_ONLY=1", block)

    def test_build_and_test_both_pass_the_shared_discovery_to_colcon(self):
        for operation, following in (("build-ros", "test-ros"), ("test-ros", "build-runner")):
            with self.subTest(operation=operation):
                block = INSIDE.split(f"  {operation})\n", 1)[1].split(f"  {following})\n", 1)[0]
                self.assertIn("    ros_source_paths\n", block)
                self.assertIn('--base-paths "${source_paths[@]}"', block)


class MacPackageCompatibilityTests(unittest.TestCase):
    def legacy_package(self, architecture, config):
        start = BUILDER.index("is_legacy_rally_v10_package() {")
        end = BUILDER.index("\n}\n", start) + 2
        # JSON fixtures are valid YAML. Supply a JSON-only safe_load shim to
        # exercise the actual shell/Python selector without requiring PyYAML
        # (the real package-building environment supplies that dependency).
        python_shim = (
            "import json, sys, types\n"
            "yaml = types.ModuleType('yaml')\n"
            "yaml.safe_load = json.loads\n"
            "sys.modules['yaml'] = yaml\n"
            "sys.argv = sys.argv[1:]\n"
            "exec(compile(sys.stdin.read(), '<package-selector>', 'exec'))\n"
        )
        script = (
            'python3() { "$HOPE_TEST_PYTHON" -B -c "$HOPE_TEST_PYTHON_SHIM" "$@"; }\n'
            + BUILDER[start:end] + "\nis_legacy_rally_v10_package\n"
        )
        with tempfile.TemporaryDirectory(prefix="hope package selection ") as temporary:
            package = Path(temporary)
            (package / "config").mkdir()
            (package / "config/a3_runtime_config.yaml").write_text(json.dumps(config))
            result = subprocess.run(["bash", "-c", script], capture_output=True, text=True,
                                    env={**os.environ, "ARCH": architecture, "PKG_DIR": str(package),
                                         "HOPE_TEST_PYTHON": sys.executable,
                                         "HOPE_TEST_PYTHON_SHIM": python_shim})
        self.assertIn(result.returncode, (0, 1), result.stderr)
        return result.returncode == 0

    def test_legacy_helpers_are_selected_only_for_rockchip_v10_without_policy_dir(self):
        legacy = {"onnx": {"model_path": "models/model_10500_v10.onnx"}}
        cases = (
            ("rockchip", legacy, True),
            ("rockchip", {**legacy, "policy_dir": "policy"}, False),
            ("rockchip", {"onnx": {"model_path": "models/model_21800.onnx"}}, False),
            ("rockchip", {"policy_dir": "policy"}, False),
            ("rockchip", {}, False),
            ("x86_64", legacy, False),
            ("thor", legacy, False),
        )
        for architecture, config, expected in cases:
            with self.subTest(architecture=architecture, config=config):
                self.assertEqual(self.legacy_package(architecture, config), expected)

    def test_legacy_selection_is_shared_by_staging_and_verification(self):
        # stage_run_scripts embeds other shell functions in heredocs, so its
        # first standalone closing brace is not the outer function's end.
        for function, following in (("stage_run_scripts", "\nstage_extra_libs() {"),
                                    ("verify_package", '\nif [[ "${STAGE_ONLY}"')):
            start = BUILDER.index(function + "() {")
            end = BUILDER.index(following, start)
            self.assertIn("if is_legacy_rally_v10_package; then", BUILDER[start:end])

    def test_tbb_ipo_option_is_optional_and_passes_the_requested_value(self):
        start = BUILDER.index('  if [[ -n "${A3_TBB_ENABLE_IPO:-}" ]]; then')
        end = BUILDER.index("\n  fi", start) + len("\n  fi")
        script = 'cmake_args=(-DBASE=ON)\n' + BUILDER[start:end] + '\nprintf "%s\\n" "${cmake_args[@]}"\n'
        for value in (None, "", "OFF", "ON"):
            with self.subTest(value=value):
                environment = {key: val for key, val in os.environ.items() if key != "A3_TBB_ENABLE_IPO"}
                if value is not None:
                    environment["A3_TBB_ENABLE_IPO"] = value
                result = subprocess.run(["bash", "-c", script], env=environment,
                                        check=True, capture_output=True, text=True)
                expected = ["-DBASE=ON"]
                if value:
                    expected.append("-DTBB_ENABLE_IPO=" + value)
                self.assertEqual(result.stdout.splitlines(), expected)


if __name__ == "__main__":
    unittest.main()
