"""Mac/HDU identity integration checks; no Docker daemon or robot required."""
from contextlib import redirect_stdout
import copy
import importlib.util
from io import StringIO
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("mac_operator", ROOT / "scripts/macos/operator.py")
operator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(operator)


def config(user="operator", helper="/home/operator/.local/libexec/hope-lifecycle"):
    return dict(root=str(ROOT), container="hope-macos-operator", ssh_user="agi",
                hdu_ip="192.168.8.182", mac_ip="192.168.8.155", linux_ip="192.168.8.156",
                mdu_ip="10.42.10.12", ssh_port=22222, laptop_user=user, laptop_helper=helper)


def contract(cfg):
    return {key: cfg[key] for key in ("laptop_user", "laptop_helper")}


def container(cfg):
    return {"Config": {"Labels": {operator.CONTRACT_LABEL: operator.CONTRACT_VERSION},
                       "Env": [f"HOPE_OPERATOR_USER={cfg['laptop_user']}",
                               f"HOPE_OPERATOR_HELPER={cfg['laptop_helper']}"]},
            "State": {"Running": True},
            "Mounts": [{"Destination": "/work", "Source": cfg["root"]},
                       {"Destination": "/operator-config", "Source": str(operator.CONFIG.resolve())},
                       {"Destination": f"/home/{cfg['laptop_user']}", "Name": "hope-macos-operator-home"}],
            "HostConfig": {"PortBindings": {"2222/tcp": [
                {"HostIp": "127.0.0.1", "HostPort": str(cfg["ssh_port"])}]}}}


class MacOperatorContractTests(unittest.TestCase):
    def test_symlinked_lifecycle_helper_is_executable(self):
        self.assertTrue((ROOT / "foxglove/helpers/hope-lifecycle").stat().st_mode & 0o111,
                        "The container dispatches the checkout helper directly")

    def test_entrypoint_handles_existing_and_new_primary_groups(self):
        source = (ROOT / "scripts/macos/operator-entrypoint.sh").read_text()
        account_setup = source.split('operator_group=hope', 1)[1].split(
            '[[ "$(id -u "$operator_user")"', 1)[0]
        account_setup = 'operator_group=hope' + account_setup
        for group_exists in (True, False):
            with self.subTest(group_exists=group_exists):
                script = '''set -euo pipefail
operator_user=operator
operator_home=/home/operator
id() { return 1; }
getent() { return GROUP_STATUS; }
groupadd() { printf 'groupadd:%s\\n' "$*"; }
useradd() { printf '%s\\n' "$@"; }
'''.replace("GROUP_STATUS", "0" if group_exists else "2") + account_setup
                result = subprocess.run(["bash", "-c", script], capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stderr)
                arguments = result.stdout.splitlines()
                self.assertIn("--create-home", arguments)
                self.assertIn("--gid", arguments)
                self.assertEqual(arguments[arguments.index("--gid") + 1], "hope")
                self.assertNotIn("--user-group", arguments)
                self.assertEqual("groupadd:hope" in arguments, not group_exists)
                self.assertEqual(arguments[-1], "operator")

    def test_direct_cli_help_works_from_mac_scripts_directory(self):
        result = subprocess.run([sys.executable, str(ROOT / "scripts/macos/operator.py"), "--help"],
                                cwd=ROOT / "scripts/macos", capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("select-mac", result.stdout)

    def test_config_requires_explicit_identity(self):
        for missing in ("laptop_user", "laptop_helper"):
            cfg = config()
            del cfg[missing]
            with self.subTest(missing=missing), self.assertRaisesRegex(ValueError, missing):
                operator.validate_config(cfg)

    def test_sequential_operators_can_reuse_an_ip_with_explicit_profiles(self):
        cfg = config()
        cfg["linux_ip"] = cfg["mac_ip"]
        self.assertEqual(operator.validate_config(cfg)["linux_ip"], cfg["mac_ip"])

    def test_default_and_original_field_contracts_are_valid(self):
        for cfg in (config(), config("dongc1", "/home/dongc1/.local/libexec/hope-lifecycle"),
                    config("alice", "/opt/hope/hope-lifecycle")):
            with self.subTest(user=cfg["laptop_user"]):
                self.assertEqual(operator.validate_config(cfg), cfg)

    def test_invalid_contracts_are_rejected(self):
        for user, helper in (("root", "/usr/local/libexec/hope-lifecycle"),
                             ("bad user", "/usr/local/libexec/hope-lifecycle"),
                             ("operator", "/tmp/../hope-lifecycle"),
                             ("operator", "relative/hope-lifecycle"),
                             ("operator", "/tmp//hope-lifecycle")):
            with self.subTest(user=user, helper=helper), self.assertRaises(ValueError):
                operator.validate_identity(user, helper)

    def test_live_identity_matches_defaults_and_custom_settings(self):
        for cfg in (config(), config("dongc1", "/home/dongc1/.local/libexec/hope-lifecycle")):
            with patch.object(operator, "remote", return_value=SimpleNamespace(stdout=json.dumps(contract(cfg)))):
                self.assertEqual(operator.verify_supervisor_contract(cfg), contract(cfg))

    def test_mismatched_identity_blocks_bind_and_mac_selection_before_writes(self):
        cfg = config()
        for key, wrong in (("laptop_user", "dongc1"), ("laptop_helper", "/home/operator/hope-lifecycle")):
            actual = {**contract(cfg), key: wrong}
            for action in (operator.bind, lambda c: operator.select(c, "mac")):
                with self.subTest(key=key, action=action), \
                        patch.object(operator, "remote", return_value=SimpleNamespace(stdout=json.dumps(actual))) as remote, \
                        patch.object(operator, "run") as run, \
                        patch.object(operator, "container_info") as inspect:
                    with self.assertRaisesRegex(RuntimeError, "differs from the running supervisor"):
                        action(cfg)
                    remote.assert_called_once_with(cfg, operator.SUPERVISOR_CONTRACT_SCRIPT, capture_output=True)
                    run.assert_not_called()
                    inspect.assert_not_called()

    def test_linux_selection_does_not_require_a_healthy_mac_container(self):
        cfg = config()
        with patch.object(operator, "verify_supervisor_contract") as identity, \
                patch.object(operator, "container_info") as inspect, \
                patch.object(operator, "remote") as remote:
            operator.select(cfg, "linux")
            identity.assert_not_called()
            inspect.assert_not_called()
            script = remote.call_args.args[1]
            self.assertIn(repr(cfg["linux_ip"]), script)
            self.assertIn("operator change requires fresh STOPPED state", script)
            self.assertNotIn("HOPE_LAPTOP_USER=", script)

    def test_live_environment_reader_uses_same_defaults_and_does_not_print_secrets(self):
        code = operator.SUPERVISOR_CONTRACT_SCRIPT.split("\n", 1)[1].rsplit("\nPY", 1)[0]
        for data, expected in ((b"SECRET=not-for-output\0", contract(config())),
                               (b"HOPE_MAC_USER=team_operator\0HOPE_MAC_LIFECYCLE_HELPER=/custom/helper\0SECRET=x\0",
                                dict(laptop_user="team_operator", laptop_helper="/custom/helper")),
                               (b"HOPE_LAPTOP_USER=linux_user\0HOPE_LAPTOP_LIFECYCLE_HELPER=/linux/helper\0",
                                contract(config()))):
            output = StringIO()
            with patch("subprocess.check_output", return_value="123\n"), \
                    patch.object(Path, "read_bytes", return_value=data) as read, redirect_stdout(output):
                exec(compile(code, "<supervisor-contract>", "exec"), {})
            self.assertEqual(json.loads(output.getvalue()), expected)
            read.assert_called_once()
            self.assertNotIn("SECRET", output.getvalue())

    def test_inactive_or_unreadable_supervisor_fails_closed(self):
        code = operator.SUPERVISOR_CONTRACT_SCRIPT.split("\n", 1)[1].rsplit("\nPY", 1)[0]
        with patch("subprocess.check_output", return_value="0\n"), \
                patch.object(Path, "read_bytes") as read, self.assertRaisesRegex(RuntimeError, "must be running"):
            exec(compile(code, "<supervisor-contract>", "exec"), {})
        read.assert_not_called()
        with patch("subprocess.check_output", return_value="123\n"), \
                patch.object(Path, "read_bytes", side_effect=PermissionError), \
                self.assertRaisesRegex(RuntimeError, "Cannot read"):
            exec(compile(code, "<supervisor-contract>", "exec"), {})

    def test_reused_container_requires_matching_identity_mounts_and_loopback_port(self):
        cfg = config()
        operator.verify_container_contract(cfg, container(cfg))
        changes = [lambda c: c["Config"].update(Labels={}),
                   lambda c: c["Config"].update(Labels={operator.CONTRACT_LABEL: "2"}),
                   lambda c: c["Config"].update(Env=["HOPE_OPERATOR_USER=dongc1"]),
                   lambda c: c["Mounts"][0].update(Source="/other/checkout"),
                   lambda c: c["Mounts"][1].update(Source="/other/config"),
                   lambda c: c["Mounts"][2].update(Destination="/home/dongc1"),
                   lambda c: c["HostConfig"]["PortBindings"]["2222/tcp"][0].update(HostIp="0.0.0.0")]
        for change in changes:
            info = copy.deepcopy(container(cfg))
            change(info)
            with self.subTest(change=change), self.assertRaisesRegex(RuntimeError, "was not changed"):
                operator.verify_container_contract(cfg, info)

    def test_up_never_replaces_mismatched_container(self):
        cfg = config()
        info = container(cfg)
        info["Config"]["Labels"] = {}
        with patch.object(operator, "container_info", return_value=info), \
                patch.object(operator, "run") as run, self.assertRaises(RuntimeError):
            operator.up(cfg)
        run.assert_not_called()

    def test_up_passes_identity_and_matching_home_to_docker(self):
        for cfg in (config(), config("dongc1", "/home/dongc1/.local/libexec/hope-lifecycle")):
            with tempfile.TemporaryDirectory() as temporary, \
                    patch.object(operator, "CONFIG", Path(temporary)), \
                    patch.object(operator, "container_info", return_value=None), \
                    patch.object(operator, "run", return_value=SimpleNamespace(stdout=json.dumps(
                        {operator.CONTRACT_LABEL: operator.CONTRACT_VERSION}))) as run, \
                    patch.object(operator.subprocess, "run", return_value=SimpleNamespace(returncode=0)), \
                    redirect_stdout(StringIO()):
                operator.up(cfg)
            argv = run.call_args_list[1].args[0]
            self.assertIn(f"HOPE_OPERATOR_USER={cfg['laptop_user']}", argv)
            self.assertIn(f"HOPE_OPERATOR_HELPER={cfg['laptop_helper']}", argv)
            self.assertIn(f"type=volume,src=hope-macos-operator-home,dst=/home/{cfg['laptop_user']}", argv)

    def test_old_image_is_rejected_before_container_creation(self):
        with tempfile.TemporaryDirectory() as temporary, \
                patch.object(operator, "CONFIG", Path(temporary)), \
                patch.object(operator, "container_info", return_value=None), \
                patch.object(operator, "run", return_value=SimpleNamespace(stdout="null")) as run, \
                self.assertRaisesRegex(RuntimeError, "Rebuild"):
            operator.up(config())
        self.assertEqual(run.call_count, 1)

    def test_tunnels_are_not_reused_across_robot_account_or_forward_port(self):
        cfg = config()
        original = operator.tunnel_socket(cfg)
        self.assertEqual(operator.tunnel_socket(dict(cfg)), original)
        self.assertIn(original, operator.ssh_args(cfg))
        for key, other in (("hdu_ip", "192.168.8.183"), ("ssh_user", "other"), ("ssh_port", 22223)):
            with self.subTest(key=key):
                self.assertNotEqual(operator.tunnel_socket({**cfg, key: other}), original)

    def test_down_stops_local_container_even_if_tunnel_is_already_gone(self):
        cfg = config()
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            (directory / "operator.json").write_text(json.dumps(cfg))
            with patch.object(operator, "CONFIG", directory), \
                    patch.object(sys, "argv", ["operator.py", "down"]), \
                    patch.object(operator, "remote") as remote, \
                    patch.object(operator, "run") as run, \
                    patch.object(operator.subprocess, "run", return_value=SimpleNamespace(returncode=255)):
                operator.main()
            self.assertTrue((directory / "disabled").exists())
            self.assertIn("Select the Linux operator before stopping", remote.call_args.args[1])
            run.assert_called_once_with(["docker", "stop", cfg["container"]])

    def test_bind_script_preserves_linux_route_and_uses_matching_user(self):
        for cfg in (config(), config("dongc1", "/home/dongc1/.local/libexec/hope-lifecycle")):
            with tempfile.TemporaryDirectory() as temporary:
                home = Path(temporary)
                (home / ".ssh").mkdir()
                linux = f"Host {cfg['linux_ip']}\n  User existing_linux_user\n"
                ssh_config = home / ".ssh/config"
                ssh_config.write_text(linux)
                settings = {**cfg, "hostkey": "ssh-ed25519 AAAATEST"}
                script = operator._bind_script(settings)
                code = script.split("\n", 1)[1].rsplit("\nPY", 1)[0]
                with patch.object(Path, "home", return_value=home), redirect_stdout(StringIO()):
                    exec(compile(code, "<bind-script>", "exec"), {})
                    exec(compile(code, "<bind-script>", "exec"), {})
                generated = ssh_config.read_text()
                self.assertIn(linux, generated)
                self.assertEqual(generated.count("# BEGIN HOPE MAC OPERATOR"), 1)
                result = subprocess.run(["ssh", "-G", "-F", str(ssh_config),
                                         f"{cfg['laptop_user']}@hope-mac-operator"],
                                        capture_output=True, text=True, check=True)
                self.assertIn(f"user {cfg['laptop_user']}\n", result.stdout)
                self.assertIn("hostname 127.0.0.1\n", result.stdout)
                self.assertNotIn(f"Host {cfg['mac_ip']}", generated)
                # Reusing the previous Mac IP on Linux must not reach the tunnel.
                linux_route = subprocess.run(["ssh", "-G", "-F", str(ssh_config),
                                              f"linux_user@{cfg['mac_ip']}"],
                                             capture_output=True, text=True, check=True)
                self.assertIn(f"hostname {cfg['mac_ip']}\n", linux_route.stdout)

    def test_down_stops_container_if_master_disappears_after_check(self):
        cfg = config()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            (path / "operator.json").write_text(json.dumps(cfg))
            def run(args):
                if args[0] == "ssh":
                    raise subprocess.CalledProcessError(255, args)
                return SimpleNamespace(returncode=0)
            with patch.object(operator, "CONFIG", path), \
                    patch.object(sys, "argv", ["operator.py", "down"]), \
                    patch.object(operator, "remote"), \
                    patch.object(operator, "run", side_effect=run) as actions, \
                    patch.object(operator.subprocess, "run", return_value=SimpleNamespace(returncode=0)):
                with self.assertRaises(subprocess.CalledProcessError):
                    operator.main()
            self.assertEqual(actions.call_args.args[0], ["docker", "stop", cfg["container"]])

    def test_mac_login_and_home_do_not_define_container_identity(self):
        for host_login in ("alice", "student", "lab_user"):
            with self.subTest(host_login=host_login), tempfile.TemporaryDirectory() as directory:
                home = Path(directory) / host_login
                checkout = home / "Projects with spaces/HOPE"
                (checkout / "foxglove/helpers").mkdir(parents=True)
                (checkout / "foxglove/helpers/hope-lifecycle").touch()
                cfg = {**config(), "root": str(checkout)}
                validated = operator.validate_config(cfg)
                self.assertEqual(validated["laptop_user"], "operator")
                self.assertEqual(validated["root"], str(checkout.resolve()))

    def test_tilde_checkout_is_expanded_for_current_user(self):
        cfg = {**config(), "root": "~/Projects/HOPE"}
        original = Path.expanduser
        def expand(path):
            return ROOT if str(path) == "~/Projects/HOPE" else original(path)
        with patch.object(Path, "expanduser", expand):
            self.assertEqual(operator.validate_config(cfg)["root"], str(ROOT.resolve()))

    def test_launcher_uses_user_home_and_launchservices_without_mac_sudo(self):
        source = (ROOT / "scripts/macos/open_foxglove.command").read_text()
        self.assertIn("Path.home()", source)
        self.assertIn("Path(cfg['root']).expanduser()", source)
        self.assertIn("'-a','Foxglove'", source)
        self.assertNotIn("/Applications/Foxglove.app", source)
        self.assertNotIn("sudo", source)
        self.assertNotIn("/Users/", source)


if __name__ == "__main__":
    unittest.main()
