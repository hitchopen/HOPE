import json
from pathlib import Path
import subprocess
import tempfile
import unittest


FOXGLOVE_DIR = Path(__file__).resolve().parents[1]
import sys

sys.path.insert(0, str(FOXGLOVE_DIR / "a3"))

from hope_lifecycle_core import (  # noqa: E402
    CONFIG_FIELDS,
    LifecycleConfig,
    apply_config_updates,
    config_from_document,
    config_to_document,
    load_config,
    parse_helper_event,
    save_config_atomic,
    release_hardware_operation_lock,
    try_acquire_hardware_operation_lock,
    validate_ipv4,
    validate_table_side,
    validate_session_id,
)


class LifecycleConfigTests(unittest.TestCase):
    @staticmethod
    def valid_config(*, revision=0):
        return LifecycleConfig(
            laptop_wifi_ip="192.168.10.2",
            hdu_wifi_ip="192.168.10.3",
            mdu_internal_ip="10.0.0.12",
            motive_ip="192.168.20.2",
            table_side="P1",
            revision=revision,
        )

    def test_defaults_require_operator_confirmation(self):
        config = LifecycleConfig()
        self.assertEqual(config.revision, 0)
        self.assertEqual(
            config.values(),
            {
                "laptop_wifi_ip": "",
                "hdu_wifi_ip": "",
                "mdu_internal_ip": "",
                "motive_ip": "",
                "table_side": "P1",
                "operator_profile": "linux",
                "gripper_profile": "legacy",
                "pants_profile": "v2",
                "mocap_source": "none",
                "vrpn_ip": "",
                "vrpn_port": "3883",
                "vrpn_units": "mm",
                "vrpn_world_frame_confirmed": "false",
            },
        )

    def test_all_fields_are_submitted_and_revision_advances(self):
        current = self.valid_config(revision=4)
        updates = [(name, current.values()[name]) for name in CONFIG_FIELDS]
        updated = apply_config_updates(current, updates)
        self.assertEqual(updated.revision, 5)
        self.assertEqual(updated.values(), current.values())

    def test_motive_address_is_optional_for_runner_only_config(self):
        current = self.valid_config(revision=4)
        updates = [(name, current.values()[name]) for name in CONFIG_FIELDS]
        updates[CONFIG_FIELDS.index("motive_ip")] = ("motive_ip", "")
        updated = apply_config_updates(current, updates)
        self.assertEqual(updated.motive_ip, "")
        self.assertEqual(
            config_from_document(config_to_document(updated)),
            updated,
        )

    def test_partial_duplicate_unknown_and_non_string_updates_fail(self):
        current = self.valid_config()
        with self.assertRaisesRegex(ValueError, "three required IPv4"):
            apply_config_updates(current, [("hdu_wifi_ip", "172.23.20.135")])
        duplicate = [(name, current.values()[name]) for name in CONFIG_FIELDS]
        duplicate[-1] = duplicate[0]
        with self.assertRaises(ValueError):
            apply_config_updates(current, duplicate)
        unknown = [(name, current.values()[name]) for name in CONFIG_FIELDS]
        unknown[-1] = ("ssh_command", "anything")
        with self.assertRaises(ValueError):
            apply_config_updates(current, unknown)
        wrong_type = [(name, current.values()[name]) for name in CONFIG_FIELDS]
        motive_index = CONFIG_FIELDS.index("motive_ip")
        wrong_type[motive_index] = ("motive_ip", 1234)
        with self.assertRaisesRegex(ValueError, "must be a string"):
            apply_config_updates(current, wrong_type)

    def test_motive_link_local_survives_confirmation_and_persistence(self):
        current = self.valid_config(revision=4)
        values = {**current.values(), "motive_ip": "169.254.10.221"}
        updated = apply_config_updates(current, list(values.items()))
        self.assertEqual(updated.revision, 5)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.json"
            save_config_atomic(path, updated)
            self.assertEqual(load_config(path), updated)
        for field in CONFIG_FIELDS[:3]:
            with self.subTest(field=field), self.assertRaises(ValueError):
                apply_config_updates(current, list({**values, field: "169.254.10.38"}.items()))

    def test_shell_helper_matches_address_scope_and_parses_motive_link_local(self):
        helper = (FOXGLOVE_DIR / "helpers/hope-lifecycle").read_text()
        # Load only the address/parser functions; never run lifecycle actions.
        functions = helper[helper.index("valid_ipv4() {"):helper.index("require_user() {")]
        for field in CONFIG_FIELDS[:4]:
            for address in ("10.44.84.152", "192.168.50.1", "172.16.0.1",
                            "169.254.10.221", "169.253.10.221", "8.8.8.8",
                            "127.0.0.1", "0.0.0.0", "224.0.0.1",
                            "255.255.255.255", "169.254.10.999", "169.254.010.1"):
                try:
                    validate_ipv4(field, address)
                    accepted = True
                except ValueError:
                    accepted = False
                result = subprocess.run(
                    ["bash", "-c", functions + '\nvalid_ipv4 "$1" "$2"',
                     "address-test", address, "motive" if field == "motive_ip" else "transport"],
                    capture_output=True, text=True, timeout=5,
                )
                with self.subTest(field=field, address=address):
                    self.assertEqual(result.returncode == 0, accepted, result.stderr)
        result = subprocess.run(
            ["bash", "-c", functions + '\nparse_common "$@"', "parse-test",
             "model21800_20261006T120000Z", "10.44.167.48", "10.44.84.152",
             "10.42.10.12", "169.254.10.221", "P1"],
            capture_output=True, text=True, timeout=5,
        )
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_ipv4_validation_is_canonical_and_rejects_special_addresses(self):
        self.assertEqual(validate_ipv4("hdu_wifi_ip", "172.23.20.135"), "172.23.20.135")
        for invalid in (
            "172.23.20.135 ",
            "hdu.local",
            "127.0.0.1",
            "0.0.0.0",
            "224.0.0.1",
            "169.254.1.1",
            "8.8.8.8",
            "10.42.10.999",
            "$(touch /tmp/no)",
        ):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                validate_ipv4("hdu_wifi_ip", invalid)

    def test_table_side_is_explicit_and_canonical(self):
        self.assertEqual(validate_table_side("P1"), "P1")
        self.assertEqual(validate_table_side("P2"), "P2")
        for invalid in ("", "P1 ", "p2", "near", "P3", 2):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                validate_table_side(invalid)

    def test_document_round_trip_and_atomic_save(self):
        config = self.valid_config(revision=2)
        self.assertEqual(config_from_document(config_to_document(config)), config)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "nested/config.json"
            save_config_atomic(path, config)
            self.assertEqual(load_config(path), config)
            self.assertEqual(path.stat().st_mode & 0o777, 0o600)
            self.assertEqual(json.loads(path.read_text())["schema_version"], 8)

    def test_unconfirmed_or_malformed_documents_fail(self):
        document = config_to_document(self.valid_config(revision=1))
        document["revision"] = 0
        with self.assertRaises(ValueError):
            config_from_document(document)
        document["revision"] = 1
        document["motive_ip"] = "motive.local"
        with self.assertRaises(ValueError):
            config_from_document(document)

    def test_schema_one_migrates_to_historical_p1_direct_mode(self):
        document = config_to_document(self.valid_config(revision=1))
        document["schema_version"] = 1
        del document["table_side"]
        loaded = config_from_document(document)
        self.assertEqual(loaded.table_side, "P1")

    def test_gripper_profiles_persist_and_legacy_documents_migrate(self):
        current = self.valid_config(revision=4)
        values = {**current.values(), "gripper_profile": "new_20261015"}
        updated = apply_config_updates(current, list(values.items()))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.json"
            save_config_atomic(path, updated)
            self.assertEqual(load_config(path).gripper_profile, "new_20261015")
        for version in (1, 2):
            old = {**config_to_document(current), "schema_version": version}
            old.pop("gripper_profile")
            self.assertEqual(config_from_document(old).gripper_profile, "legacy")
        for invalid in ("new", "", "legacy ", 4000):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                apply_config_updates(current, list({**values, "gripper_profile": invalid}.items()))

    def test_helper_preserves_profile_through_argument_parser(self):
        helper = (FOXGLOVE_DIR / "helpers/hope-lifecycle").read_text()
        functions = helper[helper.index("valid_ipv4() {"):helper.index("require_user() {")]
        args = ["model21800_20261015T010203Z", "10.42.20.1", "10.42.20.10", "10.42.10.12", "NONE", "P1", "KERNEL", "DEFAULT"]
        for selection in ([], ["legacy"], ["new_20261015"], ["invalid"]):
            result = subprocess.run(["bash", "-c", functions + '\nparse_common "$@" || exit 1; printf %s "$GRIPPER_PROFILE"', "test", *args, *selection], capture_output=True, text=True)
            self.assertEqual(result.returncode == 0, selection != ["invalid"])
            if result.returncode == 0:
                self.assertEqual(result.stdout, selection[0] if selection else "legacy")

    def test_session_and_helper_event_contracts(self):
        self.assertEqual(
            validate_session_id("model21800_20260811T120102Z"),
            "model21800_20260811T120102Z",
        )
        with self.assertRaises(ValueError):
            validate_session_id("../../bad")
        event = parse_helper_event(
            "HOPE_LIFECYCLE_V1 step=RUNNER state=COMPLETE reason=RUNNER_PASSIVE"
        )
        self.assertIsNotNone(event)
        self.assertEqual(event.step, "RUNNER")
        self.assertIsNone(parse_helper_event("arbitrary helper output"))

    def test_hardware_operation_interlock_is_exclusive_and_reusable(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "hardware-operation.lock"
            path.touch(mode=0o600)
            first = try_acquire_hardware_operation_lock(path)
            self.assertIsNotNone(first)
            self.assertIsNone(try_acquire_hardware_operation_lock(path))
            release_hardware_operation_lock(first)
            second = try_acquire_hardware_operation_lock(path)
            self.assertIsNotNone(second)
            release_hardware_operation_lock(second)


if __name__ == "__main__":
    unittest.main()


def test_pants_profile_migration_and_roundtrip(tmp_path):
    current = LifecycleConfig(
        laptop_wifi_ip="10.42.20.1", hdu_wifi_ip="10.42.20.10",
        mdu_internal_ip="10.42.10.12", motive_ip="", gripper_profile="new_20261015", revision=35)
    document = config_to_document(current)
    document["schema_version"] = 3
    document.pop("operator_profile")
    document.pop("pants_profile")
    migrated = config_from_document(document)
    assert migrated.pants_profile == "v2"
    assert migrated.gripper_profile == "new_20261015"
    for profile in ("v4", "v2"):
        updated = apply_config_updates(migrated, list({**migrated.values(), "pants_profile": profile}.items()))
        path = tmp_path / "config.json"
        save_config_atomic(path, updated)
        assert load_config(path).pants_profile == profile
    for invalid in ("v3", "V4", "", None):
        try:
            apply_config_updates(migrated, list({**migrated.values(), "pants_profile": invalid}.items()))
        except ValueError:
            pass
        else:
            raise AssertionError("invalid pants profile accepted")


def test_field_schema_3_4_5_upgrade_preserves_site_selections():
    current = LifecycleConfig(
        laptop_wifi_ip="192.168.8.182", hdu_wifi_ip="192.168.8.155",
        mdu_internal_ip="10.42.10.12", motive_ip="", table_side="P2",
        gripper_profile="new_20261015", pants_profile="v4", mocap_source="vrpn",
        vrpn_ip="192.168.8.20", vrpn_port="3884", vrpn_units="m", revision=35)
    for version in (3, 4, 5):
        document = config_to_document(current)
        document["schema_version"] = version
        document.pop("operator_profile")
        if version == 3:
            document.pop("pants_profile")
        if version < 5:
            for name in ("mocap_source", "vrpn_ip", "vrpn_port", "vrpn_units"):
                document.pop(name)
        migrated = config_from_document(document)
        assert migrated.operator_profile == "linux"
        assert migrated.gripper_profile == current.gripper_profile
        assert migrated.table_side == "P2" and migrated.revision == 35
        assert migrated.pants_profile == ("v2" if version == 3 else "v4")
        if version == 5:
            assert migrated.values() == current.values()
        assert config_from_document(config_to_document(migrated)) == migrated


def test_main_schema_3_4_upgrade_keeps_mac_identity_and_old_clients_keep_field_values():
    from hope_lifecycle_core import LEGACY_CONFIG_FIELDS, OPERATOR_CONFIG_FIELDS, FIELD_CONFIG_FIELDS
    current = LifecycleConfig(
        laptop_wifi_ip="10.0.0.2", hdu_wifi_ip="10.0.0.3", mdu_internal_ip="10.42.10.12",
        operator_profile="mac", gripper_profile="new_20261015", pants_profile="v4",
        mocap_source="vrpn", vrpn_ip="10.0.0.4", vrpn_port="3884", vrpn_units="m", revision=35)
    for version in (3, 4):
        document = {"schema_version": version, "revision": 35,
                    **{name: current.values()[name] for name in OPERATOR_CONFIG_FIELDS}}
        if version == 4:
            document["gripper_profile"] = current.gripper_profile
        migrated = config_from_document(document)
        assert migrated.operator_profile == "mac"
        assert migrated.gripper_profile == ("legacy" if version == 3 else "new_20261015")
        assert migrated.pants_profile == "v2"
    for fields in (LEGACY_CONFIG_FIELDS, OPERATOR_CONFIG_FIELDS, FIELD_CONFIG_FIELDS):
        request = {name: current.values()[name] for name in fields}
        request["laptop_wifi_ip"] = "192.168.8.182"
        updated = apply_config_updates(current, list(request.items()))
        assert updated.values() == {**current.values(), "laptop_wifi_ip": "192.168.8.182"}
        assert updated.revision == 36
