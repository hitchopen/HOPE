"""Exercise real supervisor argv through the Mac forced-command dispatcher."""
from contextlib import redirect_stderr
from io import StringIO
import json
import os
from pathlib import Path
import re
import runpy
import shlex
import subprocess
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "foxglove/a3"))
from hope_lifecycle_core import (
    HelperEvent, LifecycleConfig, apply_config_updates, parse_helper_event,
    validate_session_id,
)

SUPERVISOR = ROOT / "foxglove/a3/hope_lifecycle_supervisor.py"
DISPATCH = ROOT / "scripts/macos/operator-dispatch.py"
OPERATOR = runpy.run_path(str(ROOT / "scripts/macos/operator.py"))


def backend_namespace(environment):
    # Load the actual service defaults and backend without importing ROS on macOS.
    source = SUPERVISOR.read_text()
    backend = source[source.index('LIFECYCLE_HELPER = '):source.index('\nclass HopeLifecycleSupervisor')]
    namespace = dict(os=os, re=re, Path=Path, subprocess=subprocess, json=json,
                     HelperEvent=HelperEvent, LifecycleConfig=LifecycleConfig,
                     parse_helper_event=parse_helper_event, validate_session_id=validate_session_id)
    with patch.dict(os.environ, environment, clear=True):
        exec(compile(backend, str(SUPERVISOR), "exec"), namespace)
    return namespace


class OperatorProfileTests(unittest.TestCase):
    def exercise(self, backend, config):
        session = "model21800_20261018T050406Z"
        backend.start(config, session, lambda event: None)
        backend.kill_all_and_collect(config, session, lambda event: None)
        backend.exchange_recording("laptop", config, {"op": "status"})
        backend.exchange_assets("laptop", config, {"op": "list"})

    def test_default_mac_commands_reach_dispatcher_and_linux_round_trip_is_preserved(self):
        for environment, linux_user, linux_helper in (
            ({}, "operator", "/usr/local/libexec/hope-lifecycle"),
            ({"HOPE_LAPTOP_USER": "lab_operator",
              "HOPE_LAPTOP_LIFECYCLE_HELPER": "/home/lab_operator/.local/libexec/hope-lifecycle"},
             "lab_operator", "/home/lab_operator/.local/libexec/hope-lifecycle"),
        ):
            namespace = backend_namespace(environment)
            calls = []

            def run(argv, **kwargs):
                calls.append(argv)
                stdout = ("HOPE_LIFECYCLE_V1 step=COLLECT state=COMPLETE reason=LOGS_COLLECTED\n"
                          if "collect" in argv else "{}")
                return SimpleNamespace(returncode=0, stdout=stdout, stderr="")

            backend = namespace["LifecycleBackend"](run=run)
            config = LifecycleConfig(laptop_wifi_ip="10.44.1.2", hdu_wifi_ip="10.44.1.3",
                                     mdu_internal_ip="10.42.10.12", revision=1, operator_profile="mac")
            # Changing Wi-Fi address does not change the Mac identity or tunnel.
            # Linux can even reuse the old IP without being routed to the Mac.
            for profile, address in (("mac", "10.44.1.2"), ("mac", "192.168.1.8"),
                                     ("linux", "10.44.1.2"), ("mac", "192.168.1.8")):
                with self.subTest(environment=environment, profile=profile, address=address):
                    calls.clear()
                    config = apply_config_updates(config, list({**config.values(),
                        "operator_profile": profile, "laptop_wifi_ip": address}.items()))
                    self.exercise(backend, config)
                    ssh_calls = [argv for argv in calls if argv[0] == "/usr/bin/ssh"]
                    host_index = 1 + len(namespace["SSH_OPTIONS"])
                    laptop_calls = [argv for argv in ssh_calls
                                    if argv[host_index] != "agi@10.42.10.12"]
                    self.assertEqual(len(laptop_calls), 6)
                    self.assertEqual({argv[host_index + 2] for argv in laptop_calls},
                                     {"preflight-laptop", "prepare-laptop", "stop-capture-laptop",
                                      "collect", "record-control", "field-assets-laptop"})
                    for argv in laptop_calls:
                        if profile == "linux":
                            self.assertEqual(argv[host_index:host_index + 2],
                                             [f"{linux_user}@{address}", linux_helper])
                            continue
                        self.assertEqual(argv[host_index], "dongc1@hope-mac-operator")
                        remote = argv[host_index + 1:]
                        # A wrong helper would exit 64 instead of reaching execv.
                        with patch.dict(os.environ, {"SSH_ORIGINAL_COMMAND": shlex.join(remote)}), \
                             patch("os.execv", side_effect=RuntimeError("dispatched")) as execute:
                            with self.assertRaisesRegex(RuntimeError, "dispatched"):
                                runpy.run_path(str(DISPATCH))
                            execute.assert_called_once_with(remote[0], remote)
                    for argv in ssh_calls:
                        if argv[host_index] == "agi@10.42.10.12":
                            self.assertEqual(argv[host_index + 1], "/usr/local/libexec/hope-lifecycle")

    def test_original_global_helper_reproduces_exit_64(self):
        with patch.dict(os.environ, {"SSH_ORIGINAL_COMMAND": "/usr/local/libexec/hope-lifecycle preflight-laptop"}), \
             patch("os.execv") as execute, redirect_stderr(StringIO()):
            with self.assertRaises(SystemExit) as error:
                runpy.run_path(str(DISPATCH))
            self.assertEqual(error.exception.code, 64)
            execute.assert_not_called()

    def test_kernel_mode_does_not_contact_either_operator_profile(self):
        namespace = backend_namespace({})
        calls = []
        backend = namespace["LifecycleBackend"](
            run=lambda argv, **kwargs: calls.append(argv) or SimpleNamespace(returncode=0, stdout="", stderr=""))
        for profile in ("mac", "linux"):
            config = LifecycleConfig(laptop_wifi_ip="10.0.0.2", mdu_internal_ip="10.42.10.12",
                                     operator_profile=profile)
            backend.start(config, "model21800_20261018T050406Z", lambda event: None, "KERNEL")
            backend.kill_all_and_collect(config, "model21800_20261018T050406Z", lambda event: None, "KERNEL")
        self.assertEqual(len(calls), 18)
        self.assertFalse(any("preflight-laptop" in argv or "collect" in argv for argv in calls))

    def test_select_submits_profile_and_address_together(self):
        config = LifecycleConfig(laptop_wifi_ip="10.0.0.2", hdu_wifi_ip="10.0.0.3",
                                 mdu_internal_ip="10.42.10.12", revision=1)
        for profile in ("mac", "linux"):
            script = OPERATOR["_select_script"]("192.168.10.9", profile)
            source = script.split("python3 - <<'PY'\n", 1)[1].rsplit("\nPY", 1)[0]
            parameters = []
            class Node:
                def create_subscription(self, cls, topic, callback, queue):
                    callback(SimpleNamespace(data="STOPPED"))
                def create_client(self, cls, name):
                    def call(request):
                        parameters.extend((p.name, p.value.string_value) for p in request.parameters)
                        return SimpleNamespace(result=lambda: SimpleNamespace(results=[SimpleNamespace(successful=True)]))
                    return SimpleNamespace(wait_for_service=lambda **kw: True, call_async=call)
                def destroy_node(self):
                    pass
            modules = {
                "rclpy": SimpleNamespace(init=lambda: None, create_node=lambda name: Node(),
                    spin_until_future_complete=lambda *a, **kw: None, shutdown=lambda: None),
                "std_msgs.msg": SimpleNamespace(String=object),
                "rcl_interfaces.srv": SimpleNamespace(SetParameters=SimpleNamespace(Request=SimpleNamespace)),
                "rcl_interfaces.msg": SimpleNamespace(Parameter=SimpleNamespace, ParameterValue=SimpleNamespace),
            }
            with patch.dict(sys.modules, modules), \
                 patch.object(Path, "read_text", return_value=json.dumps(config.values())), patch("builtins.print"):
                exec(compile(source, "select-operator", "exec"), {})
            updated = apply_config_updates(config, parameters)
            self.assertEqual(updated.operator_profile, profile)
            self.assertEqual(updated.laptop_wifi_ip, "192.168.10.9")
            self.assertEqual(updated.hdu_wifi_ip, config.hdu_wifi_ip)

    def test_ssh_binding_uses_stable_alias_not_wifi_address(self):
        import tempfile
        with tempfile.TemporaryDirectory() as directory:
            home = Path(directory)
            ssh = home / ".ssh"
            ssh.mkdir()
            original = "Host linux-laptop\n  User lab_operator\n"
            (ssh / "config").write_text(original)
            for address in ("10.44.1.2", "192.168.1.8"):
                script = OPERATOR["_bind_script"]({"mac_ip": address, "ssh_port": 22222,
                                                    "hostkey": "ssh-ed25519 TEST"})
                source = script.split("python3 - <<'PY'\n", 1)[1].rsplit("\nPY", 1)[0]
                with patch.object(Path, "home", return_value=home), patch("builtins.print"):
                    exec(compile(source, "bind-operator", "exec"), {})
                text = (ssh / "config").read_text()
                self.assertIn("Host hope-mac-operator\n", text)
                self.assertNotIn(address, text)
                self.assertIn(original, text)
                self.assertEqual(text.count("# BEGIN HOPE MAC OPERATOR"), 1)


if __name__ == "__main__":
    unittest.main()
