"""File-level deployment contracts; no ROS or systemd mutations."""
import ast
from contextlib import redirect_stderr
import io
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest
import sys
import fcntl
from types import SimpleNamespace
from unittest.mock import Mock, patch

A3 = Path(__file__).resolve().parents[1] / "a3"
sys.path.insert(0, str(A3))
from hope_operations_deploy import Deployment, FILES, MODULES, UNITS
import hope_operations_deploy


class OperationsDeploymentTests(unittest.TestCase):
    def test_first_install_failure_stops_setup_but_preserves_calling_shell(self):
        doc = (A3.parents[1] / "docs/operations/foxglove_lifecycle.md").read_text()
        block = doc.split("### First install only: initialize the reference owner", 1)[1].split(
            "```bash\n", 1)[1].split("```", 1)[0]
        subprocess.run(["bash", "-n"], input=block, text=True, check=True)
        with tempfile.TemporaryDirectory(prefix="operations-bootstrap-test-") as temp:
            # Execute the actual documentation block, with no sudo, ROS,
            # filesystem installation, or service operations permitted.
            stubs = '''
cd() { return 0; }
python3() {
  if [[ "$BOOTSTRAP_FAIL" == modules && "$2" == *'print(" ".join'* ]]; then return 9; fi
  printf '%s\\n' 'one.py two.py'
}
sudo() { printf 'ATTEMPT %s\\n' "$*"; return 7; }
systemctl() { printf 'UNEXPECTED systemctl\\n'; return 0; }
mktemp() { printf '%s\\n' "$BOOTSTRAP_TEMP"; }
sed() { return 0; }
'''
            for failure, code, installs in (("install", 7, 1), ("modules", 9, 0)):
                with self.subTest(failure=failure):
                    result = subprocess.run(["bash", "-c", stubs + block +
                        '\nbootstrap_result=$?\nprintf "SHELL_ALIVE status=%s\\n" "$bootstrap_result"\n'],
                        env={**os.environ, "BOOTSTRAP_FAIL": failure, "BOOTSTRAP_TEMP": temp},
                        text=True, capture_output=True, check=True)
                    self.assertIn(f"SHELL_ALIVE status={code}", result.stdout)
                    self.assertEqual(result.stdout.count("ATTEMPT install"), installs)
                    self.assertNotIn("systemctl", result.stdout)

    def test_prerequisite_installation_remains_documented_separately_from_staging(self):
        repo = A3.parents[1]
        doc = (repo / "docs/operations/foxglove_lifecycle.md").read_text()
        setup = doc.split("### Bridge, monitor and clock-coordinator prerequisites", 1)[1].split(
            "### Stage and activate", 1)[0]
        for name in ("hope_time_calibration.py", "hope_time_calibration_core.py",
                     "hope-time-calibration.service", "hope-chrony-burst.service",
                     "bridge_params_control.yaml", "hope-foxglove-control-bridge.service"):
            self.assertIn("~/foxglove_a3/" + name, setup)
            self.assertTrue((A3 / name).is_file())
        self.assertIn("../../foxglove/README.md#2-install-the-bridge-and-monitor-services", setup)
        self.assertIn("sudo systemctl enable --now hope-time-calibration.service hope-foxglove-control-bridge.service", doc)
        self.assertNotIn("enable --now", setup, "prerequisites must not activate the staged owner")
        self.assertIn("from hope_operations_deploy import MODULES", setup)
        self.assertIn("/usr/bin/python3 /usr/local/bin/hope_operations.py", setup)
        self.assertIn("sudo systemctl stop hope-lifecycle-supervisor.service", setup)
        bridge_doc = (repo / "foxglove/README.md").read_text()
        for target in ("hope-monitor", "hope-transport-gateway", "hope-base-pose-transport-relay"):
            self.assertIn("~/hope_field_native_build/" + target, bridge_doc)
        self.assertNotIn("~/hope_field_native_build/hope-runner-transport-relay-native", bridge_doc)

    @staticmethod
    def command():
        text = (A3 / "hope-lifecycle-supervisor.service").read_text()
        command = next(line.removeprefix("ExecStart=") for line in text.replace("\\\n", "").splitlines()
                       if line.startswith("ExecStart="))
        return shlex.split(command.removeprefix(":"))

    def test_unit_uses_native_owner_with_explicit_peers_and_no_duplicate_owners(self):
        text = (A3 / "hope-lifecycle-supervisor.service").read_text()
        self.assertNotIn("Conflicts=", text)
        self.assertIn("TimeoutStopSec=45min", text)
        self.assertIn("NoNewPrivileges=true", text)
        self.assertIn("KillMode=control-group", text)
        command = next(line.removeprefix("ExecStart=") for line in text.replace("\\\n", "").splitlines()
                       if line.startswith("ExecStart="))
        args = shlex.split(command.removeprefix(":"))
        self.assertEqual(args[:2], ["/bin/bash", "-lc"])
        self.assertIn("with_fastdds_unicast.sh", args[2])
        self.assertIn("ROS_STATIC_PEERS", args[2])
        self.assertIn("/usr/local/libexec/hope-operations", args[2])
        for old in ("hope_operations.py", "hope_observer.py", "hope_command_proxy.py", "hope_lifecycle_supervisor.py"):
            self.assertNotIn(old, command)
        subprocess.run(["bash", "-n", "-c", args[2]], check=True)

    def test_empty_peers_still_start_and_configured_peers_use_wrapper(self):
        with tempfile.TemporaryDirectory(prefix="operations-unit-") as temp:
            root = Path(temp)
            native, wrapper, capture = root / "native", root / "wrapper", root / "capture"
            for path, name in ((native, "native"), (wrapper, "wrapper")):
                path.write_text('#!/usr/bin/env bash\nprintf "%s\\n" ' + name + ' "$@" >"$OPERATIONS_CAPTURE"\n')
                path.chmod(0o755)
            command = self.command()[2].replace("source /opt/ros/jazzy/setup.bash", "true")
            command = command.replace("/usr/local/libexec/hope-operations", str(native))
            command = command.replace("/home/agi/hope_ws/install/hope_bringup/lib/hope_bringup/with_fastdds_unicast.sh", str(wrapper))
            for peers, expected in (("", ["native"]),
                                    ("192.168.8.132;10.42.10.12", ["wrapper", "--domain-id", "219", "--peer",
                                        "192.168.8.132", "--peer", "10.42.10.12", "--", str(native)])):
                with self.subTest(peers=peers):
                    subprocess.run(["bash", "-c", command], check=True,
                        env={**os.environ, "ROS_DOMAIN_ID": "219", "ROS_STATIC_PEERS": peers,
                             "OPERATIONS_CAPTURE": str(capture)})
                    self.assertEqual(capture.read_text().splitlines()[:len(expected)], expected)

    def test_installer_includes_transitive_hope_module_dependencies(self):
        script = (A3 / "install_native_operations.sh").read_text()
        modules = set(MODULES)
        # Installed separately because the clock coordinator imports it too.
        modules.add("hope_time_calibration_core.py")
        roots = modules | {"hope_operations_worker.py"}
        for name in roots:
            self.assertTrue((A3 / name).is_file(), name)
            tree = ast.parse((A3 / name).read_text())
            imports = set()
            for item in ast.walk(tree):
                if isinstance(item, ast.ImportFrom) and item.module:
                    imports.add(item.module.split(".")[0])
                elif isinstance(item, ast.Import):
                    imports.update(alias.name.split(".")[0] for alias in item.names)
            for module in imports:
                if module.startswith("hope_"):
                    self.assertIn(module + ".py", roots, (name, module))
        self.assertNotIn("systemctl restart", script)
        self.assertNotIn("--now", script)
        self.assertNotIn("agibot_pm", script)
        self.assertNotIn("systemctl", script, "staging must not alter loaded units or boot behavior")
        self.assertIn('hope_operations_deploy.py" stage', script)

    def test_worker_does_not_initialize_ros(self):
        # Staging a3/ alone must not require the repository-only test fixtures.
        cmake = (A3 / "native/CMakeLists.txt").read_text()
        self.assertIn('option(BUILD_TESTING "Build maintained isolated regression tools (never installed)" OFF)', cmake)
        self.assertIn('option(HOPE_MONITOR_REQUIRE_VENDOR_RPC "Require A3 vendor emergency RPC at build time" ON)', cmake)
        for target in ("hope-monitor", "hope-mocap-config-transport", "hope-operations"):
            self.assertIn("add_executable(" + target, cmake)
        self.assertLess(cmake.index("option(BUILD_TESTING"), cmake.index("find_package(ament_cmake"))
        tree = ast.parse((A3 / "hope_operations_worker.py").read_text())
        for item in ast.walk(tree):
            if isinstance(item, ast.Call):
                self.assertNotIn(ast.unparse(item.func), {"rclpy.init", "rclpy.create_node", "rclpy.spin"})

    def test_combined_worker_disables_standalone_proxy_network_restart(self):
        tree = ast.parse((A3 / "hope_operations_worker.py").read_text())
        constructors = [item for item in ast.walk(tree) if isinstance(item, ast.Call)
                        and isinstance(item.func, ast.Name) and item.func.id == "CommandProxyModule"]
        self.assertEqual(len(constructors), 1)
        keywords = {item.arg: ast.literal_eval(item.value) for item in constructors[0].keywords
                    if item.arg == "watch_interfaces"}
        self.assertEqual(keywords, {"watch_interfaces": False})
        proxy = ast.parse((A3 / "hope_command_proxy.py").read_text())
        cls = next(item for item in proxy.body if isinstance(item, ast.ClassDef) and item.name == "CommandProxyModule")
        init = next(item for item in cls.body if isinstance(item, ast.FunctionDef) and item.name == "__init__")
        for arg, default in zip(init.args.kwonlyargs, init.args.kw_defaults):
            if arg.arg == "watch_interfaces":
                self.assertTrue(ast.literal_eval(default), "standalone compatibility still watches interfaces")

    def test_native_worker_composes_field_modules_with_one_shared_state(self):
        tree = ast.parse((A3 / "hope_operations_worker.py").read_text())
        constructors = {item.func.id: item for item in ast.walk(tree)
                        if isinstance(item, ast.Call) and isinstance(item.func, ast.Name)
                        and item.func.id in {"CommandProxyModule", "LifecycleModule", "RuntimeRelay",
                                             "HopeCommandProxy", "HopeLifecycleSupervisor", "HopeObserver"}}
        self.assertEqual(set(constructors), {"CommandProxyModule", "LifecycleModule", "RuntimeRelay"})
        for name, constructor in constructors.items():
            self.assertEqual(ast.unparse(constructor.args[0]), "node")
            if name == "RuntimeRelay":
                continue
            self.assertEqual(next(ast.unparse(k.value) for k in constructor.keywords
                                  if k.arg == "shared_state"), "shared")
        for module in ("hope_operations_core.py", "hope_operations_runtime.py", "hope_telemetry.py"):
            self.assertIn(module, MODULES)


class StagedActivationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="operations-deployment-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.source = self.root / "source"
        self.source.mkdir()
        for name in FILES:
            (self.source / name).write_text("candidate " + name)
        self.run = Mock(side_effect=self.systemctl)
        self.deployment = Deployment(self.root, self.run)
        self.active = False
        self.legacy = {}
        self.before = {}
        for name, (absolute, mode) in list(FILES.items())[::2]:
            path = self.deployment.path(absolute)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("previous " + name)
            path.chmod(mode)
            self.before[absolute] = path.read_bytes()
        self.lock = self.deployment.path("/var/lib/hope-lifecycle/hardware-operation.lock")
        self.lock.parent.mkdir(parents=True)
        self.lock.touch()
        self.dropin = self.deployment.path("/etc/systemd/system/hope-lifecycle-supervisor.service.d/venue.conf")
        self.dropin.parent.mkdir(parents=True)
        self.dropin.write_text("venue settings must survive")

    def systemctl(self, args, **kwargs):
        if args[1] == "show":
            self.assertIn("--all", args, "empty hook properties must be included for stub verification")
            if args[2] in self.legacy:
                return SimpleNamespace(returncode=0, stdout="\n".join(
                    f"{key}={value}" for key, value in self.legacy[args[2]].items()) + "\n")
            if args[2] != "hope-lifecycle-supervisor.service":
                return SimpleNamespace(returncode=1, stdout="LoadState=not-found\n")
            return SimpleNamespace(returncode=0, stdout=("LoadState=loaded\nActiveState="
                + ("active" if self.active else "inactive") + "\nMainPID="
                + ("123" if self.active else "0") + "\nControlPID=0\n"))
        self.assertEqual(args, ["systemctl", "daemon-reload"])
        return SimpleNamespace(returncode=0)

    def stage(self):
        return self.deployment.stage(self.source, self.source / "hope-operations",
                                     self.source / "hope-lifecycle-supervisor.service")

    def assert_original(self):
        for absolute, _ in FILES.values():
            path = self.deployment.path(absolute)
            if absolute in self.before:
                self.assertEqual(path.read_bytes(), self.before[absolute])
            else:
                self.assertFalse(path.exists())
        self.assertEqual(self.dropin.read_text(), "venue settings must survive")

    def test_stage_changes_no_active_files_systemd_or_boot_state(self):
        self.active = True
        bundle = self.stage()
        self.deployment.verify(bundle)
        self.assert_original()
        self.run.assert_not_called()

    def test_activation_requires_attended_stopped_owner_and_free_hardware_lock(self):
        bundle = self.stage()
        with self.assertRaises(ValueError):
            self.deployment.activate(bundle)
        self.active = True
        with self.assertRaises(RuntimeError):
            self.deployment.activate(bundle, True)
        self.active = False
        with self.lock.open("r+") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with self.assertRaises(BlockingIOError):
                self.deployment.activate(bundle, True)
        self.assert_original()

    def test_matched_activation_backup_and_explicit_rollback_preserve_dropins(self):
        backup = self.deployment.activate(self.stage(), True)
        for name, (absolute, mode) in FILES.items():
            path = self.deployment.path(absolute)
            self.assertEqual(path.read_text(), "candidate " + name)
            self.assertEqual(path.stat().st_mode & 0o777, mode)
        with self.assertRaises(ValueError):
            self.deployment.rollback(backup)
        self.deployment.rollback(backup, True)
        self.assert_original()
        self.assertEqual(sum(call.args[0] == ["systemctl", "daemon-reload"] for call in self.run.call_args_list), 2)

    def test_partial_activation_failure_restores_previous_bundle(self):
        original = self.deployment.replace
        count = 0
        def failing_once(*args):
            nonlocal count
            count += 1
            if count == 4:
                raise OSError("injected install failure")
            original(*args)
        self.deployment.replace = failing_once
        with self.assertRaisesRegex(OSError, "injected"):
            self.deployment.activate(self.stage(), True)
        self.assert_original()

    def test_changed_or_incomplete_stage_cannot_activate(self):
        bundle = self.stage()
        (bundle / "hope-operations").write_text("tampered")
        with self.assertRaises(ValueError):
            self.deployment.activate(bundle, True)
        self.assert_original()
        self.run.assert_not_called()

    @staticmethod
    def legacy_state(*, stub=False, enabled="enabled"):
        executable = "/usr/bin/true" if stub else "/usr/bin/python3"
        arguments = executable if stub else executable + " /usr/local/libexec/hope_command_proxy.py"
        return dict(LoadState="loaded", ActiveState="inactive", MainPID="0", ControlPID="0",
                    UnitFileState=enabled, Type="oneshot" if stub else "simple", NeedDaemonReload="no",
                    ExecStart=f"{{ path={executable} ; argv[]={arguments} ; ignore_errors=no ; "
                              "start_time=[n/a] ; stop_time=[n/a] ; pid=0 ; code=(null) ; status=0/0 }",
                    ExecCondition="", ExecStartPre="", ExecStartPost="", ExecStop="", ExecStopPost="")

    def assert_no_activation(self, bundle, match):
        with self.assertRaisesRegex(RuntimeError, match):
            self.deployment.activate(bundle, True)
        self.assert_original()
        self.assertFalse(self.deployment.path("/var/backups/hope-operations").exists())
        self.assertFalse(any(call.args[0] == ["systemctl", "daemon-reload"] for call in self.run.call_args_list))

    def test_enabled_real_split_owners_refused_even_when_stopped(self):
        bundle = self.stage()
        for unit in UNITS[1:]:
            for enabled in ("enabled", "enabled-runtime", "static", "indirect", "alias", ""):
                with self.subTest(unit=unit, enabled=enabled):
                    self.legacy = {unit: self.legacy_state(enabled=enabled)}
                    self.assert_no_activation(bundle, unit + " is a real or unverified split-layout")

    def test_disabled_real_services_and_enabled_true_compatibility_stubs_allowed(self):
        bundle = self.stage()
        for stub, enabled in ((False, "disabled"), (True, "enabled"), (True, "static")):
            with self.subTest(stub=stub, enabled=enabled):
                self.legacy = {unit: self.legacy_state(stub=stub, enabled=enabled) for unit in UNITS[1:]}
                backup = self.deployment.activate(bundle, True)
                self.deployment.rollback(backup, True)
                self.assert_original()
        # No test permits the installer to change enablement or start anything.
        self.assertTrue(all(call.args[0][1] in {"show", "daemon-reload"} for call in self.run.call_args_list))

    def test_active_disabled_legacy_owner_still_refused(self):
        bundle = self.stage()
        for unit in UNITS[1:]:
            state = self.legacy_state(enabled="disabled")
            state.update(ActiveState="active", MainPID="123")
            self.legacy = {unit: state}
            self.assert_no_activation(bundle, "requires stopped owner: " + unit)

    def test_persistent_masked_legacy_units_allowed_and_masks_not_modified(self):
        bundle = self.stage()
        for unit in UNITS[1:]:
            mask = self.deployment.path("/etc/systemd/system/" + unit)
            mask.symlink_to("/dev/null")
            self.legacy[unit] = self.legacy_state(enabled="masked")
            self.legacy[unit].update(LoadState="masked", ExecStart="")
        backup = self.deployment.activate(bundle, True)
        self.deployment.rollback(backup, True)
        self.assert_original()
        for unit in UNITS[1:]:
            self.assertEqual(os.readlink(self.deployment.path("/etc/systemd/system/" + unit)), "/dev/null")
        self.assertTrue(all(call.args[0][1] in {"show", "daemon-reload"} for call in self.run.call_args_list))

    def test_runtime_only_masks_explain_reboot_risk(self):
        bundle = self.stage()
        for unit in UNITS[1:]:
            state = self.legacy_state(enabled="masked-runtime")
            state.update(LoadState="masked", ExecStart="")
            self.legacy = {unit: state}
            self.assert_no_activation(bundle, "runtime mask, which disappears on reboot")

    def test_mask_never_hides_an_active_owner_or_pending_reload(self):
        bundle = self.stage()
        for extra, reason in ((dict(ActiveState="active", MainPID="123"), "requires stopped owner"),
                              (dict(NeedDaemonReload="yes"), "pending unit changes")):
            state = self.legacy_state(enabled="masked")
            state.update(LoadState="masked", **extra)
            self.legacy = {UNITS[1]: state}
            self.assert_no_activation(bundle, reason)

    def test_masked_canonical_owner_is_not_implicitly_unmasked(self):
        state = self.legacy_state(enabled="masked")
        state.update(LoadState="masked")
        self.legacy = {UNITS[0]: state}
        self.assert_no_activation(self.stage(), "canonical supervisor is masked")

    def test_effective_overrides_or_unverified_stub_never_count_as_true_stub(self):
        bundle = self.stage()
        real = self.legacy_state()["ExecStart"]
        true = self.legacy_state(stub=True)["ExecStart"]
        cases = [("ExecStart", real), ("ExecStart", true + " " + real),
                 ("ExecStart", "{ path=/bin/sh ; argv[]=/bin/sh -c /usr/bin/true ; ignore_errors=no ; }"),
                 ("ExecStart", ""), ("NeedDaemonReload", "yes"), ("NeedDaemonReload", ""),
                 ("Type", "simple")]
        cases.extend((hook, real) for hook in ("ExecCondition", "ExecStartPre", "ExecStartPost", "ExecStop", "ExecStopPost"))
        for key, value in cases:
            with self.subTest(key=key, value=value):
                state = self.legacy_state(stub=True)
                state[key] = value
                self.legacy = {UNITS[1]: state}
                self.assert_no_activation(bundle, "split-layout")

    def test_layout_is_rechecked_after_hardware_lock_acquisition(self):
        bundle = self.stage()
        original = self.systemctl
        checks = 0
        def change_between_checks(args, **kwargs):
            nonlocal checks
            if args[1:3] == ["show", UNITS[1]]:
                checks += 1
                if checks == 2:
                    self.legacy[UNITS[1]] = self.legacy_state()
            return original(args, **kwargs)
        self.run.side_effect = change_between_checks
        self.assert_no_activation(bundle, "split-layout")
        self.assertEqual(checks, 2)

    def test_layout_guard_does_not_prevent_rollback_to_previous_deployment(self):
        backup = self.deployment.activate(self.stage(), True)
        self.legacy = {unit: self.legacy_state() for unit in UNITS[1:]}
        self.deployment.rollback(backup, True)
        self.assert_original()

    def test_missing_interlock_explains_initialization_without_creating_it(self):
        bundle = self.stage()
        self.lock.unlink()
        self.assert_no_activation(bundle, "missing existing hardware interlock.*initialized HOPE deployment")
        self.assertFalse(self.lock.exists())

    def test_cli_reports_activation_errors_without_a_traceback(self):
        stderr = io.StringIO()
        error = "missing existing hardware interlock: initialize the baseline supervisor before retrying"
        with patch.object(hope_operations_deploy.os, "geteuid", return_value=0), \
             patch.object(hope_operations_deploy, "Deployment") as deployment, \
             patch.object(sys, "argv", ["hope_operations_deploy.py", "activate", "/staged/candidate", \
                                        "--confirm-supported-stopped-wired-estop"]), redirect_stderr(stderr):
            deployment.return_value.activate.side_effect = RuntimeError(error)
            with self.assertRaises(SystemExit) as exited:
                hope_operations_deploy.main()
        self.assertEqual(exited.exception.code, 1)
        self.assertEqual(stderr.getvalue(), "operations activate failed: " + error + "\n")
