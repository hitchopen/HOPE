"""Generated-message fixtures and passive ROS-adapter boundaries.

These descriptors model the checked-in vendor schema fields used by the
decoder. They are synthetic tests, not evidence of live firmware semantics.
"""
from pathlib import Path
import sys
import threading
import types
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "a3"))
from hope_elink_telemetry import ElinkTelemetryTracker, TelemetryContract, decode_vendor_channel
from hope_elink_telemetry_ros import ElinkTelemetrySubscriptions
from test_elink_telemetry import configuration, TRANSFER, HAL, WALL


def generated_fixture_classes():
    from google.protobuf import descriptor_pb2, descriptor_pool, message_factory
    file = descriptor_pb2.FileDescriptorProto(name="elink_fixture.proto", package="aimdk.protocol", syntax="proto3")
    F = descriptor_pb2.FieldDescriptorProto
    def message(name, fields, *, oneof=False):
        item = file.message_type.add(name=name)
        if oneof:
            item.oneof_decl.add(name="hand")
        for number, name, kind, repeated in fields:
            field = item.field.add(name=name, number=number,
                                  label=F.LABEL_REPEATED if repeated else F.LABEL_OPTIONAL)
            if isinstance(kind, str):
                field.type, field.type_name = F.TYPE_MESSAGE, ".aimdk.protocol." + kind
            else:
                field.type = kind
            if oneof:
                field.oneof_index = 0
    message("Timestamp", [(1,"seconds",F.TYPE_INT64,False),(2,"nanos",F.TYPE_INT32,False),(3,"ms_since_epoch",F.TYPE_INT64,False)])
    message("Header", [(1,"seq",F.TYPE_UINT32,False),(2,"timestamp","Timestamp",False)])
    message("FingerPos", [(1,"thumb_pos_0",F.TYPE_INT32,False)])
    message("FingerToq", [(1,"thumb_toq_0",F.TYPE_INT32,False)])
    message("FingerState", [(1,"pos","FingerPos",False),(2,"toq","FingerToq",False)])
    message("AgibotHandState", [(1,"finger","FingerState",False)])
    message("AgiClawHandState", [(1,"state",F.TYPE_UINT32,False),(2,"pos",F.TYPE_UINT32,False)])
    message("SingleHandState", [(3,"agi_claw_state","AgiClawHandState",False),(4,"agi_hand","AgibotHandState",False)], oneof=True)
    message("HandState", [(1,"left","SingleHandState",False),(2,"right","SingleHandState",False)])
    message("HandStateChannel", [(1,"header","Header",False),(2,"data","HandState",False)])
    message("BmsState", [(n,k,F.TYPE_FLOAT,False) for n,k in
            ((2,"voltage"),(3,"current"),(4,"power"),(5,"temperature"),(6,"capacity"),(7,"charge"))] +
            [(n,k,F.TYPE_UINT32,False) for n,k in
             ((12,"abnormal_state"),(14,"bms_state"),(18,"battery_pack_state"),(19,"battery_comm_state"))])
    message("BmsStateChannel", [(1,"header","Header",False),(2,"data","BmsState",False),(3,"bms_datas","BmsState",True)])
    message("EmergencyState", [(1,"active",F.TYPE_BOOL,False),(2,"reason",F.TYPE_UINT32,False),
            (3,"left_front_sensor_alarm",F.TYPE_BOOL,False),(19,"wired_emergency_stop",F.TYPE_BOOL,False),
            (20,"wireless_emergency_stop",F.TYPE_BOOL,False),(21,"software_emergency_stop",F.TYPE_BOOL,False)])
    message("EmergencyStateChannel", [(1,"header","Header",False),(2,"data","EmergencyState",False)])
    pool = descriptor_pool.DescriptorPool()
    pool.Add(file)
    get_class = getattr(message_factory, "GetMessageClass", None)
    if get_class is None:  # Ubuntu/ROS protobuf 4.21 compatibility.
        get_class = message_factory.MessageFactory(pool).GetPrototype
    return {name: get_class(pool.FindMessageTypeByName("aimdk.protocol." + name))
            for name in ("HandStateChannel", "BmsStateChannel", "EmergencyStateChannel")}


class DecoderTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            cls.classes = generated_fixture_classes()
        except ImportError:
            raise unittest.SkipTest("google.protobuf required for generated-message fixtures")

    def message(self, channel="bms", seq=1, delta_ns=100_000_000):
        name = {"bms":"BmsStateChannel", "emergency":"EmergencyStateChannel",
                "left_hand":"HandStateChannel", "right_hand":"HandStateChannel"}[channel]
        msg = self.classes[name]()
        msg.header.seq = seq
        msg.header.timestamp.seconds, msg.header.timestamp.nanos = divmod(WALL + delta_ns, 1_000_000_000)
        if channel == "bms":
            msg.data.voltage = 48000
        elif channel == "emergency":
            msg.data.SetInParent()
        else:
            msg.data.left.agi_hand.finger.pos.SetInParent()
            msg.data.right.agi_hand.finger.pos.SetInParent()
        return msg

    def decode(self, channel, message):
        return decode_vendor_channel(channel, message, publisher_node="/hal_elink", publisher_gid="01" * 24)

    def test_valid_generated_bms_and_hand_messages(self):
        for channel in ("bms", "emergency", "left_hand", "right_hand"):
            decoded = self.decode(channel, self.message(channel))
            self.assertEqual(decoded.source_ns, WALL + 100_000_000)
            self.assertFalse(decoded.device_fault)

    def test_unset_bms_sequence_survives_wire_roundtrip(self):
        message = self.message("bms", seq=0)
        # A proto3 default is omitted from the wire, as in the reported vendor
        # BMS stream. Do not fabricate a positive sequence in the decoder.
        message.header.ClearField("seq")
        copy = self.classes["BmsStateChannel"]()
        copy.ParseFromString(message.SerializeToString())
        decoded = self.decode("bms", copy)
        self.assertEqual(decoded.seq, 0)
        self.assertEqual(decoded.source_ns, WALL + 100_000_000)
        self.assertTrue(decoded.payload_valid)

    def test_missing_header_data_or_timestamp_rejected(self):
        for channel in ("bms", "emergency", "left_hand"):
            for field in ("header", "data"):
                msg = self.message(channel)
                msg.ClearField(field)
                with self.assertRaises(ValueError):
                    self.decode(channel, msg)
        msg = self.message()
        msg.header.ClearField("timestamp")
        with self.assertRaises(ValueError):
            self.decode("bms", msg)

    def test_timestamp_invalid_and_inconsistent_rejected(self):
        for name, value in (("seconds", 0), ("nanos", -1), ("nanos", 1_000_000_000), ("ms_since_epoch", 1)):
            msg = self.message()
            setattr(msg.header.timestamp, name, value)
            with self.assertRaises(ValueError):
                self.decode("bms", msg)

    def test_wrong_generated_message_type_rejected(self):
        with self.assertRaises(ValueError):
            self.decode("bms", self.message("emergency"))

    def test_missing_hand_variant_or_position_rejected(self):
        msg = self.message("left_hand")
        msg.data.left.ClearField("agi_hand")
        with self.assertRaises(ValueError):
            self.decode("left_hand", msg)
        msg.data.left.agi_hand.SetInParent()
        with self.assertRaises(ValueError):
            self.decode("left_hand", msg)

    def test_bms_nan_zero_negative_voltage_are_advisory_but_excess_slots_invalid(self):
        for voltage in (float("nan"), float("inf"), 0, -1):
            msg = self.message()
            msg.data.voltage = voltage
            sample = self.decode("bms", msg)
            self.assertTrue(sample.payload_valid)
            self.assertFalse(sample.device_fault)
            self.assertIn("voltage=" + str(float(voltage)), ";".join(sample.diagnostics))
        msg = self.message()
        for _ in range(16):
            msg.bms_datas.add(voltage=48000)
        with self.assertRaises(ValueError):
            self.decode("bms", msg)

    def test_reported_bms_communication_fault_is_advisory(self):
        msg = self.message()
        msg.data.battery_comm_state = 1
        sample = self.decode("bms", msg)
        self.assertFalse(sample.device_fault)
        self.assertIn("battery_comm_state=1", ";".join(sample.diagnostics))

    def dual_pack_message(self, seq=0, delta_ns=100_000_000, *, connected=True):
        msg = self.message("bms", seq, delta_ns)
        msg.ClearField("data")
        msg.bms_datas.add(voltage=0, bms_state=0)
        msg.bms_datas.add(voltage=58910 if connected else 0, bms_state=1 if connected else 0)
        return msg

    def test_bms_absent_slot_plus_present_pack_and_legacy_single_data(self):
        sample = self.decode("bms", self.dual_pack_message())
        self.assertTrue(sample.payload_valid)
        self.assertFalse(sample.device_fault)
        # Legacy single-data publishers omit this proto3 enum (default 0).
        legacy = self.message("bms", seq=0)
        legacy.data.ClearField("bms_state")
        sample = self.decode("bms", legacy)
        self.assertFalse(sample.device_fault)
        self.assertEqual(sample.diagnostics, ())

    def test_bms_all_zero_and_connected_zero_are_advisory(self):
        sample = self.decode("bms", self.dual_pack_message(connected=False))
        self.assertTrue(sample.payload_valid)
        self.assertEqual(len(sample.diagnostics), 2)
        self.assertTrue(all("voltage=0.0" in warning for warning in sample.diagnostics))
        msg = self.dual_pack_message()
        msg.bms_datas[0].bms_state = 1
        self.assertFalse(self.decode("bms", msg).device_fault)

    def test_bms_absent_slot_preserves_values_presence_and_faults_as_advisory(self):
        for field, value in (("voltage", -1), ("current", float("nan")),
                             ("temperature", float("inf")), ("bms_state", 2)):
            with self.subTest(field=field):
                msg = self.dual_pack_message()
                setattr(msg.bms_datas[0], field, value)
                sample = self.decode("bms", msg)
                self.assertTrue(sample.payload_valid)
                self.assertFalse(sample.device_fault)
                self.assertIn(field + "=", ";".join(sample.diagnostics))
        for connected in (True, False):
            for field in ("abnormal_state", "battery_pack_state", "battery_comm_state"):
                with self.subTest(connected=connected, field=field):
                    msg = self.dual_pack_message(connected=connected)
                    setattr(msg.bms_datas[0], field, 1)
                    sample = self.decode("bms", msg)
                    self.assertFalse(sample.device_fault)
                    self.assertIn(field + "=1", ";".join(sample.diagnostics))

    def test_emergency_alarm_or_nonzero_reason_is_fault(self):
        for name in ("active", "wired_emergency_stop", "wireless_emergency_stop", "software_emergency_stop", "left_front_sensor_alarm", "reason"):
            msg = self.message("emergency")
            setattr(msg.data, name, 1)
            self.assertTrue(self.decode("emergency", msg).device_fault)

    def test_claw_unknown_status_rejected_and_failure_reported(self):
        msg = self.message("left_hand")
        msg.data.left.agi_claw_state.SetInParent()
        with self.assertRaises(ValueError):
            self.decode("left_hand", msg)
        msg.data.left.agi_claw_state.state = 0x13
        self.assertTrue(self.decode("left_hand", msg).device_fault)

    def test_omitted_proto3_normal_flags_never_authorize_unverified_contract(self):
        tracker = ElinkTelemetryTracker(TelemetryContract.from_dict(configuration(False)))
        generation = tracker.bind_generation("session", TRANSFER, HAL, now=10, wall_ns=WALL)
        for seq, delta_ns in ((1,100_000_000),(2,200_000_000)):
            for channel in ("bms", "emergency", "left_hand", "right_hand"):
                sample = self.decode(channel, self.message(channel, 0 if channel == "bms" else seq, delta_ns))
                self.assertTrue(tracker.update(channel, sample, now=10+delta_ns/1e9,
                                               wall_ns=WALL+delta_ns, generation_id=generation))
        status = tracker.status(10.2, WALL + 200_000_000, True)
        self.assertTrue(status["message_fresh"])
        self.assertFalse(status["functional_ready"])


class AdapterTests(unittest.TestCase):
    """No ROS installation required: exercise callback and graph boundaries."""
    @classmethod
    def setUpClass(cls):
        DecoderTests.setUpClass.__func__(cls)

    message = DecoderTests.message
    dual_pack_message = DecoderTests.dual_pack_message

    def setUp(self):
        self.tracker = ElinkTelemetryTracker(TelemetryContract.from_dict(configuration()))
        self.tracker.bind_generation("session", TRANSFER, HAL, now=10, wall_ns=WALL,
                                    hardware_profile="TEST_PROFILE", hal_library_sha256="a"*64)
        self.clock = [10.1, WALL+100_000_000]
        self.callbacks = {}
        self.destroyed_subscriptions = []
        self.destroyed_timers = []
        self.graph = [types.SimpleNamespace(node_namespace="/", node_name="hal_elink", endpoint_gid=bytes([1])*24)]
        self.node = types.SimpleNamespace(
            create_subscription=lambda cls, topic, callback, qos: self.callbacks.setdefault(topic, callback),
            create_timer=lambda period, callback: callback,
            get_publishers_info_by_topic=lambda topic: self.graph,
            destroy_subscription=self.destroyed_subscriptions.append,
            destroy_timer=self.destroyed_timers.append,
        )
        qos = types.SimpleNamespace(QoSProfile=lambda **kw: kw, ReliabilityPolicy=types.SimpleNamespace(BEST_EFFORT=1),
            DurabilityPolicy=types.SimpleNamespace(VOLATILE=1), HistoryPolicy=types.SimpleNamespace(KEEP_LAST=1))
        modules = {"aimdk": types.ModuleType("aimdk"), "aimdk.protocol_pb2": types.SimpleNamespace(**self.classes),
                   "ros2_plugin_proto": types.ModuleType("ros2_plugin_proto"),
                   "ros2_plugin_proto.msg": types.SimpleNamespace(RosMsgWrapper=object),
                   "rclpy": types.ModuleType("rclpy"), "rclpy.qos": qos}
        with patch.dict(sys.modules, modules):
            self.adapter = ElinkTelemetrySubscriptions(self.node, self.tracker, threading.Lock(),
                monotonic=lambda:self.clock[0], wall_ns=lambda:self.clock[1])

    def receive(self, seq=1, delta_ns=100_000_000, *, publisher_gid=bytes([1])*24, **changes):
        self.clock[:] = [10+delta_ns/1e9, WALL+delta_ns]
        message = types.SimpleNamespace(serialization_type="pb", data=self.message("bms",seq,delta_ns).SerializeToString())
        for key,value in changes.items():
            setattr(message,key,value)
        callback = self.callbacks[configuration()["channels"]["bms"]["topic"]]
        callback(message, types.SimpleNamespace(publisher_gid=publisher_gid))

    def test_adapter_receives_generated_wrapper_without_rpc(self):
        self.assertTrue(self.adapter.available)
        self.receive()
        self.receive(2,200_000_000)
        self.assertTrue(self.tracker.status(*self.clock,True)["channels"]["bms"]["message_fresh"])

    def test_native_delayed_cdr_keeps_original_receipt_age(self):
        callback = self.callbacks[configuration()["channels"]["bms"]["topic"]]
        # Graph cache can be refreshed after ingress but before dispatch.
        self.clock[:] = [10.5, WALL+500_000_000]
        self.adapter._refresh_endpoints()
        for seq, delta in ((1,100_000_000),(2,200_000_000)):
            message = types.SimpleNamespace(serialization_type="pb",
                data=self.message("bms",seq,delta).SerializeToString())
            callback(message, types.SimpleNamespace(publisher_gid=bytes([1])*24,
                received_monotonic_s=10+delta/1e9, received_wall_ns=WALL+delta))
        channel = self.tracker.status(*self.clock,True)["channels"]["bms"]
        self.assertAlmostEqual(channel["receipt_age_s"], .3)
        self.assertEqual(channel["sequence"], 2)

    def test_adapter_accepts_unset_bms_sequence_with_source_progress(self):
        self.receive(0,100_000_000)
        self.receive(0,200_000_000)
        status = self.tracker.status(*self.clock,True)
        self.assertTrue(status["channels"]["bms"]["message_fresh"])
        self.assertEqual(status["channels"]["bms"]["sequence"], 0)
        # Other required channels are still missing; BMS alone cannot admit.
        self.assertFalse(status["functional_ready"])

    def test_adapter_both_zero_startup_counts_transport_progress_without_latching(self):
        self.receive(0,100_000_000, data=self.dual_pack_message(connected=False).SerializeToString())
        status = self.tracker.status(*self.clock,True)
        self.assertFalse(status["channels"]["bms"]["message_fresh"])
        self.assertEqual(status["channels"]["bms"]["reason"], "ELINK_TELEMETRY_NEEDS_PROGRESS")
        self.assertEqual(len(status["channels"]["bms"]["warnings"]), 2)
        self.assertFalse(status["fault_latched"])
        self.assertFalse(status["functional_ready"])
        for delta_ns, fresh in ((200_000_000,True),(300_000_000,True)):
            self.receive(0,delta_ns, data=self.dual_pack_message(delta_ns=delta_ns).SerializeToString())
            status = self.tracker.status(*self.clock,True)
            self.assertEqual(status["channels"]["bms"]["message_fresh"], fresh)
            self.assertFalse(status["fault_latched"])
        # A later all-zero sample is also advisory, not a readiness veto.
        self.receive(0,400_000_000,
                     data=self.dual_pack_message(delta_ns=400_000_000, connected=False).SerializeToString())
        self.assertTrue(self.tracker.status(*self.clock,True)["channels"]["bms"]["message_fresh"])

    def test_adapter_unset_bms_sequence_cannot_refresh_replayed_timestamps(self):
        self.receive(0,100_000_000)
        self.receive(0,200_000_000)
        for receipt_ns, source_ns in ((300_000_000,200_000_000), (400_000_000,100_000_000)):
            with self.subTest(source_ns=source_ns):
                self.receive(0,receipt_ns, data=self.message("bms",0,source_ns).SerializeToString())
                status = self.tracker.status(*self.clock,True)
                self.assertFalse(status["channels"]["bms"]["message_fresh"])
                self.assertEqual(status["channels"]["bms"]["reason"],
                                 "ELINK_TELEMETRY_NONPROGRESSING_SAMPLE")
                self.assertFalse(status["functional_ready"])

    def test_adapter_rejects_malformed_and_oversized_wrappers(self):
        for changes in ({"serialization_type":"json"},{"data":b"x"*65537},{"data":b"\xff"},{"data":[]}):
            self.receive(**changes)
            self.assertFalse(self.tracker.status(*self.clock,True)["channels"]["bms"]["message_fresh"])

    def test_graph_ambiguity_invalidates_previously_fresh_channel(self):
        self.receive()
        self.receive(2,200_000_000)
        self.graph.append(self.graph[0])
        self.adapter._refresh_endpoints()
        self.assertFalse(self.tracker.status(*self.clock,True)["channels"]["bms"]["message_fresh"])

    def test_wrong_message_info_gid_revokes_previously_fresh_channel(self):
        self.receive()
        self.receive(2,200_000_000)
        self.assertTrue(self.tracker.status(*self.clock,True)["channels"]["bms"]["message_fresh"])
        self.receive(3,300_000_000, publisher_gid=bytes([2])*24)
        status = self.tracker.status(*self.clock,True)
        self.assertFalse(status["channels"]["bms"]["message_fresh"])
        self.assertIsNone(status["valid_until_monotonic_s"])

    def test_graph_cache_expiry_cannot_admit_new_data(self):
        self.receive()
        self.receive(2,3_000_000_000)
        self.assertFalse(self.tracker.status(*self.clock,True)["channels"]["bms"]["message_fresh"])

    def test_generated_decoder_missing_is_unavailable_diagnostic(self):
        with patch.dict(sys.modules, {"aimdk.protocol_pb2": None}):
            adapter = ElinkTelemetrySubscriptions(self.node, self.tracker, threading.Lock())
        self.assertFalse(adapter.available)
        self.assertIn("DECODER_UNAVAILABLE", adapter.unavailable_reason)

    def test_incompatible_optional_decoder_does_not_prevent_startup(self):
        original_import = __import__
        for error in (RuntimeError, TypeError):
            with self.subTest(error=error.__name__):
                def incompatible(name, *args, **kwargs):
                    if name == "aimdk.protocol_pb2":
                        raise error("incompatible generated protobuf module")
                    return original_import(name, *args, **kwargs)
                with patch("builtins.__import__", side_effect=incompatible):
                    adapter = ElinkTelemetrySubscriptions(self.node, self.tracker, threading.Lock())
                self.assertFalse(adapter.available)
                self.assertIn("DECODER_UNAVAILABLE", adapter.unavailable_reason)
                self.assertFalse(self.tracker.status(*self.clock, True)["functional_ready"])

    def test_decoder_interruption_is_not_suppressed(self):
        original_import = __import__
        for error in (KeyboardInterrupt, SystemExit):
            with self.subTest(error=error.__name__):
                def interrupted(name, *args, **kwargs):
                    if name == "aimdk.protocol_pb2":
                        raise error()
                    return original_import(name, *args, **kwargs)
                with patch("builtins.__import__", side_effect=interrupted):
                    with self.assertRaises(error):
                        ElinkTelemetrySubscriptions(self.node, self.tracker, threading.Lock())

    def test_close_revokes_and_queued_callbacks_cannot_repopulate(self):
        self.receive()
        self.receive(2,200_000_000)
        timer = self.adapter._timer
        subscriptions = list(self.adapter._subscriptions)
        self.adapter.close()
        self.assertFalse(self.adapter.available)
        self.assertEqual(self.destroyed_timers, [timer])
        self.assertEqual(self.destroyed_subscriptions, subscriptions)
        # Simulate callbacks already queued in the ROS executor at close time.
        timer()
        self.receive(3,300_000_000)
        self.receive(4,400_000_000)
        self.assertEqual(self.adapter._endpoints, {})
        status = self.tracker.status(*self.clock,True)
        self.assertFalse(status["channels"]["bms"]["message_fresh"])
        self.assertIsNone(status["valid_until_monotonic_s"])
        self.adapter.close()
        self.assertEqual(self.destroyed_subscriptions, subscriptions)

    def test_close_cleans_remaining_subscriptions_after_destroy_failure(self):
        subscriptions = list(self.adapter._subscriptions)
        attempted = []
        def destroy(subscription):
            attempted.append(subscription)
            if len(attempted) == 1:
                raise RuntimeError("executor already removed first subscription")
        self.node.destroy_subscription = destroy
        self.node.destroy_timer = lambda timer: (_ for _ in ()).throw(RuntimeError("already removed"))
        self.adapter.close()
        self.assertEqual(attempted, subscriptions)
        self.assertEqual(self.adapter._subscriptions, [])
        self.assertIsNone(self.adapter._timer)
        self.assertFalse(self.adapter.available)


if __name__ == "__main__":
    unittest.main()
