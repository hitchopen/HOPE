"""Compile the actual checked-in vendor protos and verify decoder compatibility.

Run in the existing network-isolated ROS development container (protoc and
google.protobuf required). Output is temporary; neither vendor nor robot files
are modified. These tests validate schema compatibility, not MCU freshness.
"""
import importlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "a3"))
from hope_elink_telemetry import ElinkTelemetryTracker, TelemetryContract, decode_vendor_channel
from test_elink_telemetry import configuration, TRANSFER, HAL, WALL


class ActualVendorSchemaTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        protoc = shutil.which("protoc")
        if not protoc:
            raise unittest.SkipTest("protoc required; run in network-isolated hope-dev ROS container")
        try:
            import google.protobuf  # noqa: F401
        except ImportError:
            raise unittest.SkipTest("google.protobuf required")
        root = Path(__file__).resolve().parents[2]
        cls.proto_root = root / "agi/agibot_a3_Ultra_aimdk-dev3.1/protocol/protobuf"
        targets = {
            "aimdk/protocol/hal/hand/hand_channel.proto",
            "aimdk/protocol/hal/bms/hal_bms_channel.proto",
            "aimdk/protocol/hal/state/hal_state_channel.proto",
        }
        dependencies = set()
        def include(name):
            if name in dependencies:
                return
            path = cls.proto_root / name
            if not path.is_file() or not path.resolve().is_relative_to(cls.proto_root.resolve()):
                raise AssertionError("missing or out-of-tree vendor proto: " + name)
            dependencies.add(name)
            for dependency in re.findall(r'^\s*import\s+(?:public\s+)?"([^"]+)";', path.read_text(), re.MULTILINE):
                include(dependency)
        for name in targets:
            include(name)
        cls.temp = tempfile.TemporaryDirectory(prefix="hope-elink-vendor-proto-")
        cls.addClassCleanup(cls.temp.cleanup)
        subprocess.run([protoc, "-I" + str(cls.proto_root), "--python_out=" + cls.temp.name,
                        *sorted(dependencies)], cwd=cls.proto_root, check=True,
                       capture_output=True, text=True, timeout=30)
        sys.path.insert(0, cls.temp.name)
        cls.addClassCleanup(lambda: sys.path.remove(cls.temp.name))
        existing_modules = set(sys.modules)
        def forget_generated_modules():
            for name in list(sys.modules):
                if (name == "aimdk" or name.startswith("aimdk.")) and name not in existing_modules:
                    del sys.modules[name]
        cls.addClassCleanup(forget_generated_modules)
        cls.hand = importlib.import_module("aimdk.protocol.hal.hand.hand_channel_pb2").HandStateChannel
        cls.bms = importlib.import_module("aimdk.protocol.hal.bms.hal_bms_channel_pb2").BmsStateChannel
        cls.emergency = importlib.import_module("aimdk.protocol.hal.state.hal_state_channel_pb2").EmergencyStateChannel

    def envelope(self, cls):
        message = cls()
        message.header.seq = 100
        message.header.timestamp.seconds = 1_800_000_000
        message.header.timestamp.nanos = 123_000_000
        message.header.timestamp.ms_since_epoch = 1_800_000_000_123
        return message

    def decode(self, channel, message):
        # Exercise the actual generated wire serializer and parser too.
        copy = type(message)()
        copy.ParseFromString(message.SerializeToString())
        return decode_vendor_channel(channel, copy, publisher_node="/fixture_hal", publisher_gid="01" * 24)

    def hand_message(self, variant, side="left"):
        message = self.envelope(self.hand)
        selected = getattr(getattr(message.data, side), variant)
        if variant == "inspire":
            selected.finger.thumb_pos_0 = 100
        elif variant == "claw":
            selected.position = 50
        elif variant == "agi_claw_state":
            selected.state = 0x12
            selected.pos = 50
        elif variant == "agi_hand":
            selected.finger.pos.thumb_pos_0 = 100
            selected.finger.toq.thumb_toq_0 = 100
        elif variant == "agi_o10_hand":
            selected.finger.pos.thumb_roration_pos_0 = 100
            selected.finger.toq.thumb_rotate_torque = 20
            selected.finger.pressure.thumb_pressures.extend([1] * 16)
            selected.finger.pressure.palm_pressures.extend([1] * 25)
        else:
            self.fail("unrecognized test variant")
        return message

    def test_all_five_actual_hand_variants_both_sides(self):
        for variant in ("inspire", "claw", "agi_claw_state", "agi_hand", "agi_o10_hand"):
            for side in ("left", "right"):
                with self.subTest(variant=variant, side=side):
                    decoded = self.decode(side + "_hand", self.hand_message(variant, side))
                    self.assertEqual(decoded.variant, variant)
                    self.assertTrue(decoded.payload_valid)
                    self.assertFalse(decoded.device_fault)

    def test_documented_position_and_torque_bounds(self):
        for variant, position_name, position_max, torque_name, torque_max in (
                ("agi_hand", "thumb_pos_0", 2000, "thumb_toq_0", 5700),
                ("agi_o10_hand", "thumb_roration_pos_0", 4096, "thumb_rotate_torque", 255)):
            for value in (-1, position_max + 1):
                message = self.hand_message(variant)
                setattr(getattr(message.data.left, variant).finger.pos, position_name, value)
                with self.assertRaises(ValueError):
                    self.decode("left_hand", message)
            message = self.hand_message(variant)
            setattr(getattr(message.data.left, variant).finger.toq, torque_name, torque_max + 1)
            with self.assertRaises(ValueError):
                self.decode("left_hand", message)

    def test_actual_o10_pressure_length_and_range(self):
        message = self.hand_message("agi_o10_hand")
        message.data.left.agi_o10_hand.finger.pressure.thumb_pressures.append(1)
        with self.assertRaises(ValueError):
            self.decode("left_hand", message)
        message = self.hand_message("agi_o10_hand")
        message.data.left.agi_o10_hand.finger.pressure.palm_pressures[0] = 256
        with self.assertRaises(ValueError):
            self.decode("left_hand", message)

    def test_actual_claw_failures_and_undefined_state(self):
        for code in (0x13, 0x14, 0x23):
            message = self.hand_message("agi_claw_state")
            message.data.left.agi_claw_state.state = code
            self.assertTrue(self.decode("left_hand", message).device_fault)
        message = self.hand_message("agi_claw_state")
        message.data.left.agi_claw_state.state = 0
        with self.assertRaises(ValueError):
            self.decode("left_hand", message)

    def test_actual_bms_single_multiple_and_communication_fault(self):
        message = self.envelope(self.bms)
        message.data.voltage = 48000
        self.assertFalse(self.decode("bms", message).device_fault)
        message.bms_datas.add(voltage=48000)
        self.assertFalse(self.decode("bms", message).device_fault)
        message.bms_datas[0].battery_comm_state = 1
        sample = self.decode("bms", message)
        self.assertFalse(sample.device_fault)
        self.assertIn("battery_comm_state=1", ";".join(sample.diagnostics))

    def test_actual_bms_known_adverse_health_and_startup_statuses_are_readable_advisory(self):
        for field, values in (("power_supply_health", range(2, 9)),
                              ("battery_slow_start_output_state", (1, 4)),
                              ("battery_slow_start_recharge_state", (1, 4))):
            for value in values:
                with self.subTest(field=field, value=value):
                    message = self.envelope(self.bms)
                    message.data.voltage = 58910
                    message.data.bms_state = 1
                    setattr(message.data, field, value)
                    sample = self.decode("bms", message)
                    self.assertTrue(sample.payload_valid)
                    self.assertFalse(sample.device_fault)
                    self.assertIn(field + "=" + str(value), ";".join(sample.diagnostics))
        healthy = self.envelope(self.bms)
        healthy.data.voltage = 58910
        healthy.data.bms_state = 1
        healthy.data.power_supply_health = 1
        healthy.data.battery_slow_start_output_state = 3
        healthy.data.battery_slow_start_recharge_state = 3
        self.assertEqual(self.decode("bms", healthy).diagnostics, ())

    def tracked_message(self, channel, seq, source_delta_ns):
        if channel.endswith("hand"):
            message = self.hand_message("agi_hand", channel.removesuffix("_hand"))
        elif channel == "bms":
            message = self.envelope(self.bms)
            message.data.voltage = 48000
        else:
            message = self.envelope(self.emergency)
            message.data.SetInParent()
        if channel == "bms":
            # Model the reported real publisher's unset proto3 sequence. This
            # remains absent on the wire, not a synthetic increasing counter.
            message.header.ClearField("seq")
        else:
            message.header.seq = seq
        source_ns = WALL + source_delta_ns
        message.header.timestamp.seconds, message.header.timestamp.nanos = divmod(source_ns, 1_000_000_000)
        message.header.timestamp.ms_since_epoch = source_ns // 1_000_000
        return message

    def empty_vendor_tracker(self, audited=True):
        # This deliberately artificial audit is not evidence that live
        # firmware timestamps denote MCU acquisition, rather than publishing.
        config = configuration(audited)
        for spec in config["channels"].values():
            spec["publisher_node"] = "/fixture_hal"
        tracker = ElinkTelemetryTracker(TelemetryContract.from_dict(config))
        generation = tracker.bind_generation("vendor-wire-test", TRANSFER, HAL, now=10, wall_ns=WALL,
                                             hardware_profile="TEST_PROFILE", hal_library_sha256="a" * 64)
        return tracker, generation

    def vendor_tracker_with_progress(self, audited=True):
        tracker, generation = self.empty_vendor_tracker(audited)
        for seq, source_delta_ns in ((1,100_000_000), (2,104_000_000)):
            # Representative 3.9 ms source age, not a replay of the vendor log.
            receipt_ns = source_delta_ns + 3_900_000
            for channel in ("left_hand", "right_hand", "bms", "emergency"):
                sample = self.decode(channel, self.tracked_message(channel, seq, source_delta_ns))
                if channel == "bms":
                    self.assertEqual(sample.seq, 0)
                self.assertTrue(tracker.update(channel, sample, now=10 + receipt_ns / 1e9,
                                               wall_ns=WALL + receipt_ns, generation_id=generation))
        return tracker, generation

    def dual_pack_message(self, source_delta_ns, *, connected=True):
        # Actual vendor wire schema, representative layout: unused slot 0,
        # usable slot 1. This is not a recorded live acquisition trace.
        message = self.tracked_message("bms", 0, source_delta_ns)
        message.ClearField("data")
        message.bms_datas.add(voltage=0, bms_state=0)  # BatteryStatus_NotPulgin
        message.bms_datas.add(voltage=58910 if connected else 0,
                             bms_state=1 if connected else 0)
        return message

    def test_actual_dual_pack_zero_startup_and_seq_zero_samples_count_as_transport(self):
        tracker, generation = self.empty_vendor_tracker()
        for seq, source_delta_ns in ((1,100_000_000), (2,104_000_000), (3,108_000_000)):
            receipt_ns = source_delta_ns + 3_900_000
            for channel in ("left_hand", "right_hand", "bms", "emergency"):
                message = (self.dual_pack_message(source_delta_ns, connected=seq > 1) if channel == "bms"
                           else self.tracked_message(channel, seq, source_delta_ns))
                sample = self.decode(channel, message)
                if channel == "bms":
                    self.assertEqual(sample.seq, 0)
                self.assertTrue(tracker.update(channel, sample, now=10 + receipt_ns / 1e9,
                                               wall_ns=WALL + receipt_ns, generation_id=generation))
            status = tracker.status(10 + receipt_ns / 1e9, WALL + receipt_ns, True)
            self.assertEqual(status["functional_ready"], seq >= 2)
            self.assertFalse(status["fault_latched"])
            self.assertEqual(len(status["channels"]["bms"]["warnings"]), 2 if seq == 1 else 1)
        self.assertEqual(status["channels"]["bms"]["sequence"], 0)

    def test_actual_dual_pack_nonfinite_values_and_unknown_presence_are_json_safe_advisory(self):
        for field, value in (("voltage", -1), ("voltage", float("nan")),
                             ("current", float("inf")), ("bms_state", 2)):
            with self.subTest(field=field, value=value):
                message = self.dual_pack_message(100_000_000)
                setattr(message.bms_datas[0], field, value)
                sample = self.decode("bms", message)
                self.assertTrue(sample.payload_valid)
                self.assertFalse(sample.device_fault)
                self.assertIn(field + "=", ";".join(sample.diagnostics))
                json.dumps(sample.diagnostics, allow_nan=False)
        message = self.dual_pack_message(100_000_000)
        message.bms_datas[0].bms_state = 1  # Connected but zero V, even with another usable pack.
        self.assertFalse(self.decode("bms", message).device_fault)

    def test_actual_absent_pack_reported_faults_are_advisory_with_or_without_usable_pack(self):
        for connected in (True, False):
            for field in ("abnormal_state", "battery_pack_state", "battery_comm_state"):
                with self.subTest(connected=connected, field=field):
                    tracker, generation = self.vendor_tracker_with_progress()
                    message = self.dual_pack_message(108_000_000, connected=connected)
                    setattr(message.bms_datas[0], field, 1)
                    sample = self.decode("bms", message)
                    self.assertFalse(sample.device_fault)
                    self.assertTrue(tracker.update("bms", sample, now=10.1119,
                                                   wall_ns=WALL + 111_900_000, generation_id=generation))
                    status = tracker.status(10.1119, WALL + 111_900_000, True)
                    self.assertFalse(status["fault_latched"])
                    self.assertTrue(status["functional_ready"])
                    self.assertIn(field + "=1", ";".join(status["channels"]["bms"]["warnings"]))

    def test_actual_all_sixteen_packs_worst_case_diagnostics_remain_bounded_and_advisory(self):
        tracker, generation = self.vendor_tracker_with_progress()
        message = self.tracked_message("bms", 0, 108_000_000)
        message.ClearField("data")
        for _ in range(16):
            battery = message.bms_datas.add()
            for field in battery.DESCRIPTOR.fields:
                if field.type == field.TYPE_FLOAT:
                    setattr(battery, field.name, float("nan"))
                elif field.type == field.TYPE_ENUM:
                    setattr(battery, field.name, 2147483647)
        sample = self.decode("bms", message)
        self.assertTrue(tracker.update("bms", sample, now=10.1119,
                                       wall_ns=WALL + 111_900_000, generation_id=generation))
        status = tracker.status(10.1119, WALL + 111_900_000, True)
        self.assertTrue(status["functional_ready"])
        self.assertFalse(status["fault_latched"])
        warnings = status["channels"]["bms"]["warnings"]
        self.assertEqual(len(warnings), 16)
        self.assertTrue(all(len(warning) <= 256 for warning in warnings))
        for warning in warnings:
            for field in ("abnormal_state", "battery_pack_state", "battery_comm_state", "bms_state"):
                self.assertIn(field + "=2147483647", warning)
        self.assertLess(len(json.dumps(status, allow_nan=False)), 16384)

    def test_actual_repeated_undervoltage_is_not_durable_and_emergency_still_latches(self):
        tracker, generation = self.vendor_tracker_with_progress()
        for source_delta_ns in (108_000_000,112_000_000):
            message = self.dual_pack_message(source_delta_ns)
            message.bms_datas[1].abnormal_state = 4  # PowerAbnormalStatus_UNDERVOLTAGE
            sample = self.decode("bms", message)
            receipt_ns = source_delta_ns + 3_900_000
            self.assertTrue(tracker.update("bms", sample, now=10 + receipt_ns / 1e9,
                                           wall_ns=WALL + receipt_ns, generation_id=generation))
            status = tracker.status(10 + receipt_ns / 1e9, WALL + receipt_ns, True)
            self.assertTrue(status["functional_ready"])
            self.assertFalse(status["fault_latched"])
            self.assertIn("UNDERVOLTAGE", ";".join(status["channels"]["bms"]["warnings"]))
        emergency = self.tracked_message("emergency", 3, 116_000_000)
        emergency.data.software_emergency_stop = True
        self.assertFalse(tracker.update("emergency", self.decode("emergency", emergency), now=10.1199,
                                        wall_ns=WALL + 119_900_000, generation_id=generation))
        self.assertTrue(tracker.status(10.1199, WALL + 119_900_000, True)["fault_latched"])

    def test_actual_unset_bms_sequence_can_complete_audited_readiness(self):
        tracker, _ = self.vendor_tracker_with_progress()
        status = tracker.status(10.1079, WALL + 107_900_000, True)
        self.assertTrue(status["message_fresh"])
        self.assertTrue(status["functional_ready"])
        self.assertEqual(status["channels"]["bms"]["sequence"], 0)
        self.assertAlmostEqual(status["channels"]["bms"]["source_age_s"], 0.0039)

    def test_actual_unset_bms_sequence_rejects_frozen_and_regressing_stamp(self):
        for source_delta_ns in (104_000_000, 103_000_000):
            with self.subTest(source_delta_ns=source_delta_ns):
                tracker, generation = self.vendor_tracker_with_progress()
                sample = self.decode("bms", self.tracked_message("bms", 0, source_delta_ns))
                self.assertFalse(tracker.update("bms", sample, now=10.112, wall_ns=WALL + 112_000_000,
                                                generation_id=generation))
                status = tracker.status(10.112, WALL + 112_000_000, True)
                self.assertFalse(status["functional_ready"])
                self.assertEqual(status["channels"]["bms"]["reason"],
                                 "ELINK_TELEMETRY_NONPROGRESSING_SAMPLE")

    def test_actual_unset_bms_sequence_communication_fault_is_advisory(self):
        tracker, generation = self.vendor_tracker_with_progress()
        message = self.tracked_message("bms", 0, 108_000_000)
        message.data.battery_comm_state = 1
        sample = self.decode("bms", message)
        self.assertFalse(sample.device_fault)
        self.assertTrue(tracker.update("bms", sample, now=10.1119, wall_ns=WALL + 111_900_000,
                                       generation_id=generation))
        status = tracker.status(10.1119, WALL + 111_900_000, True)
        self.assertFalse(status["fault_latched"])
        self.assertTrue(status["functional_ready"])
        self.assertIn("battery_comm_state=1", ";".join(status["channels"]["bms"]["warnings"]))

    def test_actual_unset_bms_sequence_does_not_replace_contract_audit(self):
        tracker, _ = self.vendor_tracker_with_progress(audited=False)
        status = tracker.status(10.1079, WALL + 107_900_000, True)
        self.assertTrue(status["message_fresh"])
        self.assertFalse(status["functional_ready"])

    def test_actual_emergency_defaults_and_alarm(self):
        message = self.envelope(self.emergency)
        message.data.SetInParent()
        self.assertFalse(self.decode("emergency", message).device_fault)
        message.data.software_emergency_stop = True
        self.assertTrue(self.decode("emergency", message).device_fault)
        message.data.ClearField("software_emergency_stop")
        message.data.reason = 255
        self.assertTrue(self.decode("emergency", message).device_fault)

    def test_actual_header_zero_time_and_mismatched_milliseconds(self):
        message = self.hand_message("agi_hand")
        message.header.timestamp.seconds = 0
        with self.assertRaises(ValueError):
            self.decode("left_hand", message)
        message = self.hand_message("agi_hand")
        message.header.timestamp.ms_since_epoch += 1
        with self.assertRaises(ValueError):
            self.decode("left_hand", message)


if __name__ == "__main__":
    unittest.main()
