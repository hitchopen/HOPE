"""Exercise the SSH helper contract without a container or robot connection."""
from contextlib import redirect_stderr
import importlib.util
from io import StringIO
import json
import os
from pathlib import Path
import shlex
import stat
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

DISPATCH = Path(__file__).resolve().parents[2] / "scripts/macos/operator-dispatch.py"
SPEC = importlib.util.spec_from_file_location("hope_operator_dispatch", DISPATCH)
dispatcher = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(dispatcher)


class Dispatched(Exception):
    pass


class MacOperatorDispatchTests(unittest.TestCase):
    def dispatch(self, user, external_helper, args, *, accepted=True, login=None):
        local_helper = f"/home/{user}/.local/libexec/hope-lifecycle"
        contract = {"laptop_user": user, "laptop_helper": external_helper}
        account = SimpleNamespace(pw_name=login or user, pw_uid=1001)
        with patch.object(dispatcher.pwd, "getpwuid", return_value=account), \
             patch.object(dispatcher.os, "execv", side_effect=Dispatched) as execute:
            if accepted:
                with self.assertRaises(Dispatched):
                    dispatcher.dispatch(shlex.join(args), contract)
                execute.assert_called_once_with(local_helper, [local_helper, *args[1:]])
            else:
                with self.assertRaises(ValueError):
                    dispatcher.dispatch(shlex.join(args), contract)
                execute.assert_not_called()

    def test_generic_operator_and_default_supervisor_helper(self):
        helper = "/usr/local/libexec/hope-lifecycle"
        self.dispatch("operator", helper, [helper, "preflight-laptop"])

    def test_config_status_accepts_only_the_fixed_action(self):
        helper = "/home/operator/.local/libexec/hope-lifecycle"
        self.dispatch("operator", helper, [helper, "config-status-laptop"])
        for action in ("config-status-laptop", "operator-route"):
            self.dispatch("operator", helper, [helper, action, "extra"], accepted=False)

    def test_route_uses_local_registration_and_actual_kernel_source(self):
        helper = "/home/operator/.local/libexec/hope-lifecycle"
        with patch.object(dispatcher.pwd, "getpwuid", return_value=SimpleNamespace(pw_name="operator", pw_uid=1001)), \
                patch.object(Path, "read_text", return_value="192.168.200.2"), \
                patch.object(dispatcher.subprocess, "check_output", return_value='[{"prefsrc":"192.168.100.3"}]') as route, \
                patch("builtins.print") as output:
            dispatcher.dispatch(helper + " operator-route", {"laptop_user":"operator", "laptop_helper":helper})
        route.assert_called_once_with(["ip", "-j", "-4", "route", "get", "192.168.200.2"])
        self.assertEqual(json.loads(output.call_args.args[0]),
                         {"hdu_ip":"192.168.200.2", "laptop_ip":"192.168.100.3"})

    def test_hdu_reconnect_forwards_only_the_verified_machine_field(self):
        helper = "/home/operator/.local/libexec/hope-lifecycle"
        machine = "a" * 32
        contract = {"laptop_user":"operator", "laptop_helper":helper}
        command = ["/usr/bin/env", "HOPE_LIFECYCLE_MACHINE_ID=" + machine, helper, "reconnect-laptop", "session"]
        with patch.object(dispatcher.pwd, "getpwuid", return_value=SimpleNamespace(pw_name="operator", pw_uid=1001)), \
                patch.object(dispatcher.os, "execve", side_effect=Dispatched) as execute:
            with self.assertRaises(Dispatched):
                dispatcher.dispatch(shlex.join(command), contract)
            args = execute.call_args.args
            self.assertEqual(args[:2], (helper, [helper, "reconnect-laptop", "session"]))
            self.assertEqual(args[2]["HOPE_LIFECYCLE_MACHINE_ID"], machine)
            for replacement in (["OTHER=x", *command[2:]], [command[1], helper, "collect"],
                                [command[1], "OTHER=x", *command[2:]], ["HOPE_LIFECYCLE_MACHINE_ID=bad", *command[2:]]):
                with self.assertRaises(ValueError):
                    dispatcher.dispatch(shlex.join(["/usr/bin/env", *replacement]), contract)
            self.assertEqual(execute.call_count, 1)

    def test_original_dongc1_contract_still_works(self):
        helper = "/home/dongc1/.local/libexec/hope-lifecycle"
        self.dispatch("dongc1", helper, [helper, "record-control"])

    def test_another_user_with_custom_helper_maps_to_fixed_local_helper(self):
        helper = "/opt/team/bin/hope-lifecycle"
        self.dispatch("team_operator", helper, [helper, "collect", "model21800_20261018T050406Z",
                      "10.44.226.154", "10.44.84.104", "10.42.10.12", "NONE", "P1"])

    def test_all_supported_laptop_actions_remain_available(self):
        helper = "/usr/local/libexec/hope-lifecycle"
        for action in dispatcher.ALLOWED_ACTIONS - {"operator-route"}:
            with self.subTest(action=action):
                self.dispatch("operator", helper, [helper, action])

    def test_other_requested_helpers_are_rejected(self):
        helper = "/usr/local/libexec/hope-lifecycle"
        for wrong in ("/bin/bash", "/home/dongc1/.local/libexec/hope-lifecycle",
                      "/usr/local/libexec/../libexec/hope-lifecycle"):
            with self.subTest(helper=wrong):
                self.dispatch("operator", helper, [wrong, "collect"], accepted=False)

    def test_robot_actions_and_unknown_actions_are_rejected(self):
        helper = "/usr/local/libexec/hope-lifecycle"
        for action in ("start-runner", "start-hal", "collect-logs", "--configure", "collect;id"):
            with self.subTest(action=action):
                self.dispatch("operator", helper, [helper, action], accepted=False)

    def test_wrong_login_is_rejected(self):
        helper = "/usr/local/libexec/hope-lifecycle"
        self.dispatch("operator", helper, [helper, "collect"], login="another_user", accepted=False)

    def test_system_account_is_rejected(self):
        contract = dispatcher.validate_contract("operator", "/usr/local/libexec/hope-lifecycle")
        with patch.object(dispatcher.pwd, "getpwuid", return_value=SimpleNamespace(pw_name="operator", pw_uid=999)), \
             patch.object(dispatcher.os, "execv") as execute:
            with self.assertRaises(ValueError):
                dispatcher.dispatch("/usr/local/libexec/hope-lifecycle collect", contract)
            execute.assert_not_called()

    def test_malformed_or_incomplete_commands_are_rejected(self):
        contract = dispatcher.validate_contract("operator", "/usr/local/libexec/hope-lifecycle")
        with patch.object(dispatcher.pwd, "getpwuid", return_value=SimpleNamespace(pw_name="operator", pw_uid=1001)), \
             patch.object(dispatcher.os, "execv") as execute:
            for command in ("", "/usr/local/libexec/hope-lifecycle", "/usr/local/libexec/hope-lifecycle collect '"):
                with self.subTest(command=command), self.assertRaises(ValueError):
                    dispatcher.dispatch(command, contract)
            execute.assert_not_called()

    def test_missing_root_or_unsafe_user_is_rejected(self):
        for user in (None, "", "root", "0", "-operator", "Operator", "a/b", "user\nAllowUsers root", "a" * 33):
            with self.subTest(user=user), self.assertRaises(ValueError):
                dispatcher.validate_contract(user, "/usr/local/libexec/hope-lifecycle")

    def test_missing_relative_or_traversing_helper_is_rejected(self):
        for helper in (None, "", "/", "relative/helper", "/usr/local/../bin/helper",
                       "/usr/local/./helper", "/bin/helper/", "/bin/helper;id", "/bin/helper\n"):
            with self.subTest(helper=helper), self.assertRaises(ValueError):
                dispatcher.validate_contract("operator", helper)

    def test_configuration_requires_root_before_any_write(self):
        with patch.object(dispatcher.os, "geteuid", return_value=1001), \
             patch.object(dispatcher.tempfile, "NamedTemporaryFile") as create:
            with self.assertRaises(ValueError):
                dispatcher.configure_from_environment()
            create.assert_not_called()

    def test_configuration_requires_both_environment_values(self):
        for values in ({}, {"HOPE_OPERATOR_USER": "operator"},
                       {"HOPE_OPERATOR_HELPER": "/usr/local/libexec/hope-lifecycle"}):
            with self.subTest(values=values), patch.dict(os.environ, values, clear=True), \
                 patch.object(dispatcher.os, "geteuid", return_value=0), \
                 patch.object(dispatcher.tempfile, "NamedTemporaryFile") as create:
                with self.assertRaises(ValueError):
                    dispatcher.configure_from_environment()
                create.assert_not_called()

    def test_configuration_writes_only_validated_contract_with_safe_mode(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.json"
            with patch.object(dispatcher, "CONFIG_PATH", path), \
                 patch.object(dispatcher.os, "geteuid", return_value=0), \
                 patch.object(dispatcher.os, "fchown") as chown, \
                 patch.dict(os.environ, {"HOPE_OPERATOR_USER": "operator", "HOPE_OPERATOR_HELPER": "/usr/local/libexec/hope-lifecycle"}):
                dispatcher.configure_from_environment()
            self.assertEqual(json.loads(path.read_text()), {
                "laptop_user": "operator", "laptop_helper": "/usr/local/libexec/hope-lifecycle"})
            self.assertEqual(stat.S_IMODE(path.stat().st_mode), 0o644)
            self.assertEqual(chown.call_args.args[1:], (0, 0))
            self.assertEqual(list(Path(directory).iterdir()), [path])

    def test_configuration_must_be_root_owned_and_not_group_or_other_writable(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.json"
            path.write_text(json.dumps({"laptop_user": "operator", "laptop_helper": "/usr/local/libexec/hope-lifecycle"}))
            for uid, mode, allowed in ((0, 0o644, True), (1001, 0o644, False),
                                       (0, 0o664, False), (0, 0o646, False)):
                with self.subTest(uid=uid, mode=mode), patch.object(dispatcher, "CONFIG_PATH", path), \
                     patch.object(dispatcher.os, "fstat", return_value=SimpleNamespace(st_uid=uid, st_mode=stat.S_IFREG | mode)):
                    if allowed:
                        self.assertEqual(dispatcher.read_contract()["laptop_user"], "operator")
                    else:
                        with self.assertRaises(ValueError):
                            dispatcher.read_contract()

    def test_symlink_configuration_is_not_followed(self):
        with tempfile.TemporaryDirectory() as directory:
            real = Path(directory) / "real.json"
            real.write_text("{}")
            link = Path(directory) / "config.json"
            link.symlink_to(real)
            with patch.object(dispatcher, "CONFIG_PATH", link), self.assertRaises(OSError):
                dispatcher.read_contract()

    def test_cli_rejects_nonroot_configure_and_reports_exit_64(self):
        with patch.object(dispatcher.sys, "argv", [str(DISPATCH), "--configure"]), \
             patch.object(dispatcher.os, "geteuid", return_value=1001), redirect_stderr(StringIO()):
            self.assertEqual(dispatcher.main(), 64)


if __name__ == "__main__":
    unittest.main()
