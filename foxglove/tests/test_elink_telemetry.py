"""Deterministic telemetry diagnostic tests; no ROS, robot or vendor RPC."""
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "a3"))
from hope_elink_telemetry import (
    CHANNELS, DEFAULT_TOPICS, ElinkTelemetryTracker, TelemetryContract,
    TelemetrySample, load_telemetry_contract,
)

BOOT = "b967dc28-16f0-4a46-bad3-ca5d5fa16e50"
TRANSFER = {"boot_id": BOOT, "pid": 1, "start_ticks": 10}
HAL = {"boot_id": BOOT, "pid": 2, "start_ticks": 20}
WALL = 1_800_000_000_000_000_000


def configuration(audited=True):
    return {
        "schema_version": 1, "contract_id": "isolated-test-only", "audited": audited,
        "hardware_profile": "TEST_PROFILE", "hal_library_sha256": "a" * 64,
        "channels": {channel: {
            "required": True, "topic": DEFAULT_TOPICS[channel], "publisher_node": "/hal_elink",
            "stale_after_s": 1.0, "max_source_age_s": 0.5, "future_tolerance_s": 0.05,
            "expected_variant": "agi_hand" if channel.endswith("hand") else "",
            "acquisition_evidence": "TEST FIXTURE - NOT A HARDWARE AUDIT",
            "fault_semantics_evidence": "TEST FIXTURE - NOT A HARDWARE AUDIT",
            "not_required_reason": "",
        } for channel in CHANNELS},
    }


class TelemetryTrackerTests(unittest.TestCase):
    def setUp(self):
        self.tracker = ElinkTelemetryTracker(TelemetryContract.from_dict(configuration()))
        self.generation = self.bind()

    def bind(self, **kwargs):
        options = dict(now=10.0, wall_ns=WALL, hardware_profile="TEST_PROFILE", hal_library_sha256="a" * 64)
        options.update(kwargs)
        return self.tracker.bind_generation("test-session", TRANSFER, HAL, **options)

    def sample(self, channel="bms", seq=1, delta=0.1, **kwargs):
        values = dict(seq=seq, source_ns=WALL + round(delta * 1e9), publisher_node="/hal_elink",
                      publisher_gid="01" * 24, variant="agi_hand" if channel.endswith("hand") else "")
        values.update(kwargs)
        return TelemetrySample(**values)

    def update(self, channel="bms", seq=1, delta=0.1, **kwargs):
        return self.tracker.update(channel, self.sample(channel, seq, delta, **kwargs),
                                   now=10 + delta, wall_ns=WALL + round(delta * 1e9), generation_id=self.generation)

    def ready(self, *, bms_without_sequence=False):
        for seq, delta in ((1, .1), (2, .2)):
            for channel in CHANNELS:
                channel_seq = 0 if channel == "bms" and bms_without_sequence else seq
                self.assertTrue(self.update(channel, channel_seq, delta))
        return self.tracker.status(10.2, WALL + 200_000_000, True)

    def test_default_is_unverified_and_unbound(self):
        status = ElinkTelemetryTracker().status(10, WALL, True)
        self.assertFalse(status["functional_ready"])
        self.assertIsNone(status["valid_until_monotonic_s"])

    def test_two_post_bind_progress_samples_required(self):
        for channel in CHANNELS:
            self.update(channel)
        self.assertFalse(self.tracker.status(10.1, WALL + 100_000_000, True)["functional_ready"])
        for channel in CHANNELS:
            self.update(channel, seq=2, delta=.2)
        self.assertTrue(self.tracker.status(10.2, WALL + 200_000_000, True)["functional_ready"])

    def test_bms_unset_sequence_uses_two_advancing_source_timestamps(self):
        # Vendor BMS publishers leave the proto3 sequence at its default zero.
        # This is a regression fixture for that behavior, not a hardware audit.
        for channel in CHANNELS:
            self.assertTrue(self.update(channel, seq=0 if channel == "bms" else 1))
        self.assertFalse(self.tracker.status(10.1, WALL + 100_000_000, True)["functional_ready"])
        for channel in CHANNELS:
            self.assertTrue(self.update(channel, seq=0 if channel == "bms" else 2, delta=.2))
        status = self.tracker.status(10.2, WALL + 200_000_000, True)
        self.assertTrue(status["functional_ready"])
        self.assertEqual(status["channels"]["bms"]["sequence"], 0)

    def test_zero_sequence_is_not_enabled_for_other_channels(self):
        for channel in ("left_hand", "right_hand", "emergency"):
            with self.subTest(channel=channel):
                self.assertFalse(self.update(channel, seq=0))
                status = self.tracker.status(10.1, WALL + 100_000_000, True)
                self.assertEqual(status["channels"][channel]["reason"], "ELINK_TELEMETRY_SAMPLE_INVALID")

    def test_zero_sequence_bms_can_stay_ready_without_fabricating_a_counter(self):
        self.ready(bms_without_sequence=True)
        for seq in range(3, 203):
            delta = .2 + (seq - 2) / 100
            for channel in CHANNELS:
                self.assertTrue(self.update(channel, seq=0 if channel == "bms" else seq, delta=delta))
            status = self.tracker.status(10 + delta, WALL + round(delta * 1e9), True)
            self.assertTrue(status["functional_ready"])
            self.assertEqual(status["channels"]["bms"]["sequence"], 0)

    def test_zero_sequence_bms_duplicate_and_backward_stamps_do_not_refresh(self):
        for stamp in (WALL + 200_000_000, WALL + 190_000_000):
            with self.subTest(stamp=stamp):
                self.setUp()
                self.ready(bms_without_sequence=True)
                self.assertFalse(self.update(seq=0, delta=.3, source_ns=stamp))
                status = self.tracker.status(10.3, WALL + 300_000_000, True)
                self.assertFalse(status["functional_ready"])
                self.assertAlmostEqual(status["channels"]["bms"]["receipt_age_s"], .1)
                self.assertAlmostEqual(status["channels"]["bms"]["source_age_s"], .1)
                self.assertEqual(status["channels"]["bms"]["reason"], "ELINK_TELEMETRY_NONPROGRESSING_SAMPLE")
                # Rejection must not lower the timestamp high-water mark.
                self.assertFalse(self.update(seq=0, delta=.31, source_ns=WALL + 195_000_000))
                self.assertTrue(self.update(seq=0, delta=.4))
                self.assertFalse(self.tracker.status(10.4, WALL + 400_000_000, True)["functional_ready"])
                self.assertTrue(self.update(seq=0, delta=.5))
                self.assertTrue(self.tracker.status(10.5, WALL + 500_000_000, True)["functional_ready"])

    def test_zero_sequence_bms_still_requires_monotonic_receipt(self):
        for now in (10.2, 10.19):
            with self.subTest(now=now):
                self.setUp()
                self.ready(bms_without_sequence=True)
                self.assertFalse(self.tracker.update("bms", self.sample(seq=0, delta=.3),
                    now=now, wall_ns=WALL + 300_000_000, generation_id=self.generation))
                self.assertFalse(self.tracker.status(10.3, WALL + 300_000_000, True)["functional_ready"])

    def test_zero_sequence_bms_may_upgrade_but_must_not_downgrade_sequence(self):
        self.ready(bms_without_sequence=True)
        self.assertTrue(self.update(seq=1, delta=.3))
        self.assertTrue(self.update(seq=2, delta=.4))
        self.assertTrue(self.tracker.status(10.4, WALL + 400_000_000, True)["functional_ready"])
        for seq, delta in ((0, .41), (1, .42), (2, .43), (0, .44)):
            self.assertFalse(self.update(seq=seq, delta=delta))
        self.assertTrue(self.update(seq=3, delta=.5))
        self.assertFalse(self.tracker.status(10.5, WALL + 500_000_000, True)["functional_ready"])
        self.assertTrue(self.update(seq=4, delta=.6))
        self.assertTrue(self.tracker.status(10.6, WALL + 600_000_000, True)["functional_ready"])

    def test_new_generation_can_accept_zero_sequence_bms_after_positive_sequence(self):
        self.ready()
        old = self.generation
        self.generation = self.tracker.bind_generation("new-test-session", TRANSFER, HAL,
            now=11, wall_ns=WALL + 1_000_000_000, hardware_profile="TEST_PROFILE", hal_library_sha256="a" * 64)
        self.assertFalse(self.tracker.update("bms", self.sample(seq=0, delta=1.1), now=11.1,
            wall_ns=WALL + 1_100_000_000, generation_id=old))
        self.assertFalse(self.update(seq=0, delta=1.1, source_ns=WALL + 1_050_000_000))
        for seq, delta in ((1, 1.1), (2, 1.2)):
            for channel in CHANNELS:
                self.assertTrue(self.update(channel, seq=0 if channel == "bms" else seq, delta=delta))
        self.assertTrue(self.tracker.status(11.2, WALL + 1_200_000_000, True)["functional_ready"])

    def test_zero_sequence_bms_preserves_source_age_and_receipt_expiry(self):
        self.ready(bms_without_sequence=True)
        for now, wall in ((10.7, WALL + 700_000_000), (11.2, WALL + 200_000_000)):
            status = self.tracker.status(now, wall, True)
            self.assertFalse(status["channels"]["bms"]["message_fresh"])
            self.assertFalse(status["functional_ready"])

    def test_zero_sequence_bms_preserves_time_publisher_and_payload_validation(self):
        cases = (
            (dict(source_ns=WALL + 50_000_000), .1, "PREVIOUS_GENERATION_SAMPLE"),
            (dict(source_ns=WALL + 160_000_000), .1, "SOURCE_TIME_INVALID"),
            (dict(source_ns=WALL + 100_000_000), .7, "SOURCE_TIME_INVALID"),
            (dict(publisher_node="/other"), .1, "PUBLISHER_MISMATCH"),
            (dict(payload_valid=False), .1, "SAMPLE_INVALID"),
        )
        for kwargs, delta, reason in cases:
            with self.subTest(reason=reason):
                self.setUp()
                self.assertFalse(self.update(seq=0, delta=delta, **kwargs))
                status = self.tracker.status(10 + delta, WALL + round(delta * 1e9), True)
                self.assertIn(reason, status["channels"]["bms"]["reason"])

    def test_zero_sequence_bms_preserves_publisher_fault_latch(self):
        for kwargs, reason in ((dict(publisher_gid="02" * 24), "PUBLISHER_GENERATION_CHANGED"),):
            with self.subTest(reason=reason):
                self.setUp()
                self.ready(bms_without_sequence=True)
                self.assertFalse(self.update(seq=0, delta=.3, **kwargs))
                self.update(seq=0, delta=.4)
                self.update(seq=0, delta=.5)
                status = self.tracker.status(10.5, WALL + 500_000_000, True)
                self.assertFalse(status["functional_ready"])
                self.assertTrue(status["fault_latched"])
                self.assertIn(reason, status["reason"])

    def test_bms_device_fault_is_advisory_even_for_direct_sample_producers(self):
        self.ready(bms_without_sequence=True)
        self.assertTrue(self.update(seq=0, delta=.3, device_fault=True,
                                    diagnostics=("BMS_ADVISORY:test_undervoltage",)))
        status = self.tracker.status(10.3, WALL + 300_000_000, True)
        self.assertTrue(status["functional_ready"])
        self.assertFalse(status["fault_latched"])
        self.assertIn("BMS_DEVICE_FAULT_ADVISORY", status["channels"]["bms"]["warnings"])
        self.assertIn("BMS_ADVISORY:test_undervoltage", status["channels"]["bms"]["warnings"])

    def test_diagnostics_are_bounded_json_safe_and_never_an_admission_veto(self):
        self.ready(bms_without_sequence=True)
        for seq, diagnostics in enumerate((("\\\"\n电" * 1000,) * 1000, ["bad tuple type"],
                                           (None, float("nan"), {})), start=3):
            delta = seq / 10
            self.assertTrue(self.update(seq=0, delta=delta, device_fault=True, diagnostics=diagnostics))
            status = self.tracker.status(10 + delta, WALL + round(delta * 1e9), True)
            warnings = status["channels"]["bms"]["warnings"]
            self.assertTrue(status["functional_ready"])
            self.assertFalse(status["fault_latched"])
            self.assertLessEqual(len(warnings), 17)
            self.assertTrue(all(len(value) <= 256 and all(32 <= ord(c) < 127 for c in value)
                                for value in warnings))
            self.assertIn("BMS_DEVICE_FAULT_ADVISORY", warnings)
            self.assertLess(len(json.dumps(status, allow_nan=False)), 16384)

    def test_zero_sequence_bms_preserves_clock_discontinuity_latch(self):
        self.ready(bms_without_sequence=True)
        self.assertFalse(self.tracker.update("bms", self.sample(seq=0, delta=10.3),
            now=10.3, wall_ns=WALL + 10_300_000_000, generation_id=self.generation))
        status = self.tracker.status(10.3, WALL + 10_300_000_000, True)
        self.assertTrue(status["fault_latched"])
        self.assertIn("CLOCK_DISCONTINUITY", status["reason"])

    def test_zero_sequence_bms_does_not_bypass_audit_build_or_clock(self):
        for options in ({"audited": False}, {"hal_library_sha256": "b" * 64}, {"clock_qualified": False}):
            with self.subTest(options=options):
                self.tracker = ElinkTelemetryTracker(TelemetryContract.from_dict(configuration(options.get("audited", True))))
                self.generation = self.bind(hal_library_sha256=options.get("hal_library_sha256", "a" * 64))
                self.ready(bms_without_sequence=True)
                status = self.tracker.status(10.2, WALL + 200_000_000, options.get("clock_qualified", True))
                self.assertFalse(status["functional_ready"])

    def test_ready_expiry_bound_by_source_age(self):
        status = self.ready()
        self.assertTrue(status["message_fresh"])
        self.assertAlmostEqual(status["valid_until_monotonic_s"], 10.7)
        self.assertFalse(self.tracker.status(10.7, WALL + 700_000_000, True)["functional_ready"])

    def test_receipt_expiry_also_required_when_wall_clock_freezes(self):
        self.ready()
        self.assertFalse(self.tracker.status(11.2, WALL + 200_000_000, True)["functional_ready"])

    def test_unaudited_envelopes_never_become_functional_ready(self):
        self.tracker = ElinkTelemetryTracker(TelemetryContract.from_dict(configuration(False)))
        self.generation = self.bind()
        status = self.ready()
        self.assertTrue(status["message_fresh"])
        self.assertFalse(status["functional_ready"])
        self.assertIsNone(status["valid_until_monotonic_s"])

    def test_build_mismatch_denies_even_with_good_messages(self):
        self.bind(hal_library_sha256="b" * 64)
        status = self.ready()
        self.assertFalse(status["functional_ready"])
        self.assertIn("BUILD_MISMATCH", status["reason"])

    def test_wrong_profile_denies(self):
        self.bind(hardware_profile="wrong")
        self.assertFalse(self.ready()["functional_ready"])

    def test_clock_unqualified_or_invalid_denies(self):
        self.ready()
        for qualified in (False, None, 1, "true"):
            self.assertFalse(self.tracker.status(10.2, WALL + 200_000_000, qualified)["functional_ready"])
        for now, wall in ((math.nan, WALL), (9.0, WALL), (10.2, -1)):
            self.assertFalse(self.tracker.status(now, wall, True)["functional_ready"])

    def test_duplicates_do_not_refresh_and_require_new_progress(self):
        self.ready()
        self.assertFalse(self.update(seq=2, delta=.3, source_ns=WALL + 200_000_000))
        self.assertFalse(self.tracker.status(10.3, WALL + 300_000_000, True)["functional_ready"])
        self.assertTrue(self.update(seq=3, delta=.4))
        self.assertFalse(self.tracker.status(10.4, WALL + 400_000_000, True)["functional_ready"])
        self.assertTrue(self.update(seq=4, delta=.5))
        self.assertTrue(self.tracker.status(10.5, WALL + 500_000_000, True)["functional_ready"])

    def test_reordered_or_repeated_source_timestamp_invalidates(self):
        for kwargs in ({"seq": 1, "source_ns": WALL + 300_000_000},
                       {"seq": 3, "source_ns": WALL + 200_000_000}):
            with self.subTest(kwargs=kwargs):
                self.setUp()
                self.ready()
                self.assertFalse(self.update(delta=.3, **kwargs))
                self.assertFalse(self.tracker.status(10.3, WALL + 300_000_000, True)["functional_ready"])

    def test_old_queued_data_rejected_at_generation_boundary(self):
        for stamp in (WALL - 1, WALL, WALL + 50_000_000):
            self.assertFalse(self.update(source_ns=stamp))

    def test_future_and_delayed_first_delivery_rejected(self):
        self.assertFalse(self.update(source_ns=WALL + 160_000_000))
        sample = self.sample()
        self.assertFalse(self.tracker.update("bms", sample, now=20, wall_ns=WALL + 10_000_000_000,
                                            generation_id=self.generation))

    def test_invalid_scalar_types_and_nonfinite_rejected(self):
        for kwargs in ({"seq": -1}, {"seq": True}, {"seq": 0.0}, {"seq": 0x100000000}, {"source_ns": 0},
                       {"source_ns": 10 ** 1000}, {"source_ns": True}, {"payload_valid": False},
                       {"payload_valid": 1}, {"device_fault": "false"}, {"publisher_gid": ""}):
            with self.subTest(kwargs=str(kwargs)[:120]):
                self.assertFalse(self.update(**kwargs))

    def test_expected_publisher_and_hand_variant_enforced(self):
        self.assertFalse(self.update(publisher_node="/other"))
        self.assertFalse(self.update("left_hand", variant="claw"))

    def test_publisher_gid_change_latches_until_new_generation(self):
        self.ready()
        self.assertFalse(self.update(seq=3, delta=.3, publisher_gid="02" * 24))
        self.update(seq=4, delta=.4)
        self.update(seq=5, delta=.5)
        self.assertTrue(self.tracker.status(10.5, WALL + 500_000_000, True)["fault_latched"])
        self.bind()
        self.assertTrue(self.tracker.status(10.5, WALL + 500_000_000, True)["fault_latched"])

    def test_hand_and_emergency_device_fault_is_sticky(self):
        for channel in ("left_hand", "right_hand", "emergency"):
            with self.subTest(channel=channel):
                self.setUp()
                self.ready()
                self.assertFalse(self.update(channel, seq=3, delta=.3, device_fault=True))
                self.update(channel, seq=4, delta=.4)
                self.update(channel, seq=5, delta=.5)
                self.assertTrue(self.tracker.status(10.5, WALL + 500_000_000, True)["fault_latched"])

    def test_simultaneous_source_receiver_clock_step_latches(self):
        self.ready()
        self.assertFalse(self.tracker.update("bms", self.sample(seq=3, delta=10.3),
            now=10.3, wall_ns=WALL + 10_300_000_000, generation_id=self.generation))
        status = self.tracker.status(10.3, WALL + 10_300_000_000, True)
        self.assertTrue(status["fault_latched"])
        self.assertIn("CLOCK_DISCONTINUITY", status["reason"])

    def test_utc_rollback_or_forward_with_cached_sample_denies(self):
        self.ready()
        for wall_ns in (WALL - 10_000_000_000, WALL + 10_000_000_000):
            self.assertFalse(self.tracker.status(10.3, wall_ns, True)["functional_ready"])

    def test_new_process_generation_dumps_queue_and_late_old_callback_ignored(self):
        self.ready()
        old = self.generation
        new_hal = dict(HAL, pid=3, start_ticks=30)
        self.generation = self.tracker.bind_generation("test-session", TRANSFER, new_hal,
            now=11, wall_ns=WALL + 1_000_000_000, hardware_profile="TEST_PROFILE", hal_library_sha256="a" * 64)
        self.assertNotEqual(old, self.generation)
        self.assertFalse(self.tracker.update("bms", self.sample(seq=3, delta=1.1), now=11.1,
            wall_ns=WALL + 1_100_000_000, generation_id=old))
        self.assertFalse(self.tracker.status(11.1, WALL + 1_100_000_000, True)["functional_ready"])

    def test_unbind_revokes_readiness_and_blocks_late_callback(self):
        self.ready()
        self.tracker.unbind_generation()
        status = self.tracker.status(10.3, WALL + 300_000_000, True)
        self.assertFalse(status["functional_ready"])
        self.assertIsNone(status["valid_until_monotonic_s"])
        self.assertEqual(status["generation_id"], "")
        self.assertFalse(self.update(seq=3, delta=.3, publisher_gid="02" * 24))
        self.assertFalse(self.tracker.status(10.3, WALL + 300_000_000, True)["fault_latched"])
        self.bind(now=10.3, wall_ns=WALL + 300_000_000)
        self.assertFalse(self.tracker.status(10.3, WALL + 300_000_000, True)["functional_ready"])

    def test_unbind_preserves_fault_even_when_rebinding_same_identity(self):
        self.ready()
        self.update("emergency", seq=3, delta=.3, device_fault=True)
        self.tracker.unbind_generation()
        self.assertTrue(self.tracker.status(10.4, WALL + 400_000_000, True)["fault_latched"])
        self.bind(now=10.4, wall_ns=WALL + 400_000_000)
        self.assertTrue(self.tracker.status(10.4, WALL + 400_000_000, True)["fault_latched"])

    def test_nonrequired_hand_needs_explicit_reason_and_cannot_latch(self):
        config = configuration()
        config["channels"]["left_hand"].update(required=False, not_required_reason="absent in reviewed fixture")
        self.tracker = ElinkTelemetryTracker(TelemetryContract.from_dict(config))
        self.generation = self.bind()
        self.assertFalse(self.update("left_hand", device_fault=True))
        for seq, delta in ((1,.1),(2,.2)):
            for channel in ("right_hand", "bms", "emergency"):
                self.update(channel, seq, delta)
        self.assertTrue(self.tracker.status(10.2, WALL + 200_000_000, True)["functional_ready"])


class TelemetryContractTests(unittest.TestCase):
    def test_audited_contract_requires_exact_schema_and_evidence(self):
        for field, value in (("schema_version", True), ("audited", "true"),
                             ("hal_library_sha256", ""), ("hardware_profile", "")):
            config = configuration()
            config[field] = value
            with self.assertRaises(ValueError):
                TelemetryContract.from_dict(config)
        for field in ("acquisition_evidence", "fault_semantics_evidence", "publisher_node"):
            config = configuration()
            config["channels"]["bms"][field] = ""
            with self.assertRaises(ValueError):
                TelemetryContract.from_dict(config)

    def test_unknown_fields_invalid_thresholds_and_missing_channels_rejected(self):
        configs = []
        for field, value in (("stale_after_s", math.nan), ("max_source_age_s", 0),
                             ("future_tolerance_s", 2), ("required", "true"),
                             ("stale_after_s", 10 ** 1000)):
            config = configuration()
            config["channels"]["bms"][field] = value
            configs.append(config)
        config = configuration()
        config["typo"] = True
        configs.append(config)
        config = configuration()
        del config["channels"]["left_hand"]
        configs.append(config)
        for config in configs:
            with self.assertRaises(ValueError):
                TelemetryContract.from_dict(config)

    def test_core_channels_cannot_be_optional(self):
        for channel in ("bms", "emergency"):
            config = configuration()
            config["channels"][channel].update(required=False, not_required_reason="test")
            with self.assertRaises(ValueError):
                TelemetryContract.from_dict(config)

    def test_missing_contract_defaults_unverified(self):
        with tempfile.TemporaryDirectory() as directory:
            contract = load_telemetry_contract(Path(directory) / "absent", require_root=False)
            self.assertFalse(contract.audited)

    def test_symlink_group_writable_and_oversized_config_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config"
            path.write_text(json.dumps(configuration()))
            link = Path(directory) / "link"
            link.symlink_to(path)
            self.assertFalse(load_telemetry_contract(link, require_root=False).audited)
            path.chmod(0o666)
            self.assertFalse(load_telemetry_contract(path, require_root=False).audited)
            path.chmod(0o600)
            path.write_text(" " * 65537)
            self.assertFalse(load_telemetry_contract(path, require_root=False).audited)

    def test_valid_contract_load_has_stable_hash(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config"
            path.write_text(json.dumps(configuration()))
            path.chmod(0o600)
            contract = load_telemetry_contract(path, require_root=False)
            self.assertEqual(contract.sha256, TelemetryContract.from_dict(configuration()).sha256)
            self.assertTrue(contract.audited)

    def test_duplicate_fields_and_excessively_nested_config_are_unverified(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config"
            valid = json.dumps(configuration())
            for raw in (valid.replace('"audited": true', '"audited": false, "audited": true'),
                        '[' * 2000 + '0' + ']' * 2000):
                path.write_text(raw)
                path.chmod(0o600)
                self.assertFalse(load_telemetry_contract(path, require_root=False).audited)


if __name__ == "__main__":
    unittest.main()
