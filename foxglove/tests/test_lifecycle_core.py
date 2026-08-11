import json
from pathlib import Path
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
    validate_ipv4,
    validate_session_id,
)


class LifecycleConfigTests(unittest.TestCase):
    def test_defaults_require_operator_confirmation(self):
        config = LifecycleConfig()
        self.assertEqual(config.revision, 0)
        self.assertEqual(config.hdu_wifi_ip, "172.23.20.135")
        self.assertEqual(config.mdu_internal_ip, "10.42.10.12")

    def test_all_four_fields_are_required_and_revision_advances(self):
        current = LifecycleConfig(revision=4)
        updates = [(name, current.values()[name]) for name in CONFIG_FIELDS]
        updated = apply_config_updates(current, updates)
        self.assertEqual(updated.revision, 5)
        self.assertEqual(updated.values(), current.values())

    def test_partial_duplicate_unknown_and_non_string_updates_fail(self):
        current = LifecycleConfig()
        with self.assertRaisesRegex(ValueError, "all four"):
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
        wrong_type[-1] = ("motive_ip", 1234)
        with self.assertRaisesRegex(ValueError, "must be a string"):
            apply_config_updates(current, wrong_type)

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

    def test_document_round_trip_and_atomic_save(self):
        config = LifecycleConfig(revision=2)
        self.assertEqual(config_from_document(config_to_document(config)), config)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "nested/config.json"
            save_config_atomic(path, config)
            self.assertEqual(load_config(path), config)
            self.assertEqual(path.stat().st_mode & 0o777, 0o600)
            self.assertEqual(json.loads(path.read_text())["schema_version"], 1)

    def test_unconfirmed_or_malformed_documents_fail(self):
        document = config_to_document(LifecycleConfig(revision=1))
        document["revision"] = 0
        with self.assertRaises(ValueError):
            config_from_document(document)
        document["revision"] = 1
        document["motive_ip"] = "motive.local"
        with self.assertRaises(ValueError):
            config_from_document(document)

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


if __name__ == "__main__":
    unittest.main()
