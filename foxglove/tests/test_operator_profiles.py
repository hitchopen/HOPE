"""Exercise supervisor profile routing through the configurable SSH dispatcher."""
import importlib.util
import json
import os
from pathlib import Path
import re
import shlex
import stat
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "foxglove/a3"))
from hope_lifecycle_core import (
    HelperEvent, LifecycleConfig, apply_config_updates, config_to_document, parse_helper_event,
    validate_session_id,
)

SUPERVISOR = ROOT / "foxglove/a3/hope_lifecycle_supervisor.py"
DISPATCH = ROOT / "scripts/macos/operator-dispatch.py"
spec = importlib.util.spec_from_file_location("profile_dispatcher", DISPATCH)
dispatcher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dispatcher)


class Dispatched(Exception):
    pass


def backend_namespace(environment):
    # Load the actual service constants/backend without requiring ROS on macOS.
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

    def assert_dispatched(self, command, user, external_helper):
        # Execute the real dispatcher entrypoint and root-owned contract reader.
        # Only ownership/account metadata and exec are mocked; no robot needed.
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "operator.json"
            config.write_text(json.dumps({"laptop_user": user, "laptop_helper": external_helper}))
            with patch.object(dispatcher, "CONFIG_PATH", config), \
                 patch.object(dispatcher.sys, "argv", [str(DISPATCH)]), \
                 patch.dict(os.environ, {"SSH_ORIGINAL_COMMAND": shlex.join(command)}), \
                 patch.object(dispatcher.os, "fstat", return_value=SimpleNamespace(
                     st_uid=0, st_mode=stat.S_IFREG | 0o644)), \
                 patch.object(dispatcher.pwd, "getpwuid", return_value=SimpleNamespace(pw_name=user, pw_uid=1001)), \
                 patch.object(dispatcher.os, "execv", side_effect=Dispatched) as execute:
                with self.assertRaises(Dispatched):
                    dispatcher.main()
                local = f"/home/{user}/.local/libexec/hope-lifecycle"
                execute.assert_called_once_with(local, [local, *command[1:]])

    def test_defaults_and_custom_mac_contract_preserve_linux_round_trip(self):
        cases = (
            ({}, "operator", "/usr/local/libexec/hope-lifecycle",
             "operator", "/home/operator/.local/libexec/hope-lifecycle"),
            ({"HOPE_LAPTOP_USER": "lab_operator",
              "HOPE_LAPTOP_LIFECYCLE_HELPER": "/opt/linux/hope-lifecycle",
              "HOPE_MAC_USER": "team_operator",
              "HOPE_MAC_LIFECYCLE_HELPER": "/opt/mac/hope-lifecycle"},
             "lab_operator", "/opt/linux/hope-lifecycle", "team_operator", "/opt/mac/hope-lifecycle"),
        )
        for environment, linux_user, linux_helper, mac_user, mac_helper in cases:
            namespace = backend_namespace(environment)
            calls = []
            def run(argv, **kwargs):
                calls.append(argv)
                stdout = ("HOPE_LIFECYCLE_V1 step=COLLECT state=COMPLETE reason=LOGS_COLLECTED\n"
                          if "collect" in argv else "{}")
                return SimpleNamespace(returncode=0, stdout=stdout, stderr="")
            backend = namespace["LifecycleBackend"](run=run)
            config = LifecycleConfig(laptop_wifi_ip="10.44.1.2", hdu_wifi_ip="10.44.1.3",
                                     mdu_internal_ip="10.42.10.12", revision=1)
            for profile, address in (("mac", "10.44.1.2"), ("mac", "192.168.1.8"),
                                     ("linux", "10.44.1.2"), ("mac", "192.168.1.8")):
                with self.subTest(environment=environment, profile=profile, address=address):
                    calls.clear()
                    config = apply_config_updates(config, list({**config.values(),
                        "operator_profile": profile, "laptop_wifi_ip": address}.items()))
                    self.exercise(backend, config)
                    ssh_calls = [argv for argv in calls if argv[0] == "/usr/bin/ssh"]
                    host_index = 1 + len(namespace["SSH_OPTIONS"])
                    laptop_calls = [argv for argv in ssh_calls if argv[host_index] != "agi@10.42.10.12"]
                    self.assertEqual(len(laptop_calls), 6)
                    self.assertEqual({argv[host_index + 2] for argv in laptop_calls},
                                     {"preflight-laptop", "prepare-laptop", "stop-capture-laptop",
                                      "collect", "record-control", "field-assets-laptop"})
                    for argv in laptop_calls:
                        if profile == "linux":
                            self.assertEqual(argv[host_index:host_index + 2],
                                             [f"{linux_user}@{address}", linux_helper])
                        else:
                            self.assertEqual(argv[host_index:host_index + 2],
                                             [f"{mac_user}@hope-mac-operator", mac_helper])
                            self.assert_dispatched(argv[host_index + 1:], mac_user, mac_helper)
                    for argv in ssh_calls:
                        if argv[host_index] == "agi@10.42.10.12":
                            self.assertEqual(argv[host_index + 1], "/usr/local/libexec/hope-lifecycle")

    def test_selected_gripper_reaches_every_start_stage_for_both_modes(self):
        namespace = backend_namespace({})
        calls = []
        backend = namespace["LifecycleBackend"](
            run=lambda argv, **kwargs: calls.append(argv) or SimpleNamespace(returncode=0, stdout="", stderr=""))
        for mode in ("KERNEL", "OPTITRACK"):
            for profile in ("mac", "linux"):
                calls.clear()
                config = LifecycleConfig(laptop_wifi_ip="10.0.0.2", hdu_wifi_ip="10.0.0.3",
                    mdu_internal_ip="10.42.10.12", operator_profile=profile, gripper_profile="new_20261015")
                backend.start(config, "model21800_20261018T050406Z", lambda event: None, mode)
                self.assertTrue(calls)
                for argv in calls:
                    self.assertEqual(argv[-3:], [mode, "DEFAULT", "new_20261015"])

    def test_kernel_mode_never_contacts_either_operator(self):
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

    def test_invalid_mac_account_and_helper_fail_before_any_connection(self):
        for value in ("root", "bad user", "user;id", ""):
            with self.subTest(user=value), self.assertRaises(RuntimeError):
                backend_namespace({"HOPE_MAC_USER": value})
        for value in ("relative", "/bin/../helper", "/bin//helper", "/bin/helper/", "/"):
            with self.subTest(helper=value), self.assertRaises(RuntimeError):
                backend_namespace({"HOPE_MAC_LIFECYCLE_HELPER": value})

    def test_select_submits_profile_and_ip_together(self):
        spec = importlib.util.spec_from_file_location("profile_operator", ROOT / "scripts/macos/operator.py")
        operator = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(operator)
        config = LifecycleConfig(laptop_wifi_ip="10.0.0.2", hdu_wifi_ip="10.0.0.3",
                                 mdu_internal_ip="10.42.10.12", revision=1,
                                 gripper_profile="new_20261015", pants_profile="v4",
                                 mocap_source="vrpn", vrpn_ip="192.168.100.99")
        for profile in ("mac", "linux"):
            source = operator._select_script("192.168.10.9", profile).split("python3 - <<'PY'\n", 1)[1].rsplit("\nPY", 1)[0]
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
                 patch.object(Path, "read_text", return_value=json.dumps(config_to_document(config))), patch("builtins.print"):
                exec(compile(source, "select-operator", "exec"), {})
            updated = apply_config_updates(config, parameters)
            self.assertEqual((updated.operator_profile, updated.laptop_wifi_ip), (profile, "192.168.10.9"))
            self.assertEqual(updated.hdu_wifi_ip, config.hdu_wifi_ip)
            for key in ("gripper_profile", "pants_profile", "mocap_source", "vrpn_ip", "vrpn_port", "vrpn_units"):
                self.assertEqual(getattr(updated, key), getattr(config, key))
            parameters.clear()
            field_document = config_to_document(config)
            field_document["schema_version"] = 5
            field_document.pop("operator_profile")
            with patch.dict(sys.modules, modules), \
                 patch.object(Path, "read_text", return_value=json.dumps(field_document)), \
                 self.assertRaisesRegex(RuntimeError, "schema-5 venue settings were left unchanged"):
                exec(compile(source, "select-operator", "exec"), {})
            self.assertFalse(parameters)


if __name__ == "__main__":
    unittest.main()
