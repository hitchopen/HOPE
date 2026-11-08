"""ROS-free regressions for the private native operations protocol."""
from pathlib import Path
import json
import math
import os
import socket
import struct
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "a3"))
from hope_operations_transport import WireConnection, WorkerNode, wire, MAX_FRAME
from hope_operations_core import OperationsState


class SharedRunnerTests(unittest.TestCase):
    def test_native_service_adapter_preserves_rejection_and_failure_replies(self):
        from hope_operations_core import RosComponent
        module = object.__new__(RosComponent)
        module._node = SimpleNamespace(_native_operations_transport=True)
        callback = lambda request, response: response
        for submitting, expected in ((True, "operation rejected"), (False, "operation failed")):
            worker = SimpleNamespace(try_submit=Mock())
            if submitting:
                worker.try_submit.side_effect = ValueError("fixture")
            else:
                future = Mock()
                future.result.side_effect = ValueError("fixture")
                worker.try_submit.return_value = SimpleNamespace(future=future)
            response = module._worker_callback(callback, worker)(SimpleNamespace(),SimpleNamespace(success=True,message=""))
            self.assertFalse(response.success)
            self.assertEqual(response.message, expected+": ValueError: fixture")

    def test_shared_state_keeps_native_age_and_invalidates_malformed_input(self):
        from dataclasses import asdict
        from hope_operations_native_state import NativeWire
        from hope_runner_control_core import decode_runner_state
        data = [2, 42, 1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 123]
        value = asdict(decode_runner_state(data))
        connection = Mock()
        connection.query.side_effect = [dict(value=value,received_ns=100_000_000_000,error="",decode_error="",generation=1),
            dict(value=value,received_ns=100_000_000_000,error="MALFORMED",decode_error="MALFORMED",generation=2)]
        wire = NativeWire(connection,"runner")
        first = wire.snapshot()
        self.assertEqual(first.value.run_mode,"PASSIVE")
        invalid = wire.snapshot()
        self.assertEqual(invalid.received_ns,first.received_ns)
        self.assertEqual(invalid.value,first.value)
        self.assertTrue(invalid.decode_error)


class TransportTests(unittest.TestCase):
    def test_query_deadline_includes_waiting_for_writer_lock(self):
        with self.connection.write_lock:
            started = time.monotonic()
            with self.assertRaisesRegex(TimeoutError, "before dispatch"):
                self.connection.query("wire_snapshot", name="runner", _timeout=.05)
            self.assertLess(time.monotonic()-started, .5)
        self.assertFalse(self.connection.closed.is_set())
        self.assertEqual(self.connection.pending, {})

    def test_cdr_callback_receives_original_paired_clocks(self):
        from unittest.mock import patch
        import types
        serialization = types.ModuleType("rclpy.serialization")
        serialization.deserialize_message = lambda data, cls: SimpleNamespace(data=data)
        seen, event = [], threading.Event()
        def callback(message, info):
            seen.append(info)
            event.set()
        self.connection.subscriptions[1] = object, callback, True
        with patch.dict(sys.modules, {"rclpy.serialization": serialization}):
            self.send(dict(op="sample", entity=1, received=123., received_wall_ns=456,
                           cdr=[1,2,3], publisher_gid=[7]))
            self.assertTrue(event.wait(2.))
        self.assertEqual(seen[0].received_monotonic_s, 123.)
        self.assertEqual(seen[0].received_wall_ns, 456)
        self.assertEqual(seen[0].publisher_gid, [7])

    def test_diagnostic_channel_failure_does_not_close_control_or_fail_queries(self):
        control, peer = socket.socketpair()
        samples, sample_peer = socket.socketpair()
        connection = WireConnection(control, samples)
        try:
            sample_peer.close()
            connection.sample_reader.join(timeout=1)
            self.assertFalse(connection.closed.is_set())
            seq, future = connection.request("wire_snapshot", name="runner")
            frame = json.dumps(dict(op="reply", id=seq, value="healthy")).encode()
            peer.sendall(struct.pack("!I", len(frame))+frame)
            self.assertEqual(future.result(timeout=1), "healthy")
        finally:
            connection.close();peer.close()

    def setUp(self):
        client, self.parent = socket.socketpair()
        self.connection = WireConnection(client)
        self.addCleanup(self.parent.close)
        self.addCleanup(self.connection.close)

    def send(self, message):
        encoded = json.dumps(message).encode()
        self.parent.sendall(struct.pack("!I", len(encoded)) + encoded)

    def receive(self):
        def exact(size):
            out = bytearray()
            while len(out) < size:
                out.extend(self.parent.recv(size - len(out)))
            return out
        length, = struct.unpack("!I", exact(4))
        return json.loads(exact(length))

    def subscription(self, cls, callback):
        result = []
        thread = threading.Thread(target=lambda: result.append(WorkerNode(self.connection).create_subscription(
            cls, "/isolated/player", callback, 4)))
        thread.start()
        registration = self.receive()
        self.send(dict(op="reply", id=registration["id"], value=True))
        thread.join(timeout=2.)
        self.assertFalse(thread.is_alive())
        return result[0]

    def test_pose_snapshot_preserves_default_player_argument_and_invalid_floats(self):
        class PoseStamped:
            __module__ = "geometry_msgs.msg._pose_stamped"
        PoseStamped.__qualname__ = "PoseStamped"
        event, seen = threading.Event(), []
        def callback(message, name="UCB_P2"):
            seen.append((name, message))
            event.set()
        entity = self.subscription(PoseStamped, callback)
        self.assertFalse(self.connection.subscriptions[entity][2])
        self.send(dict(op="sample", entity=entity, received=100., message={
            "header": {"frame_id": "hope_table_global", "stamp": {"sec": 12, "nanosec": 345}},
            "pose": {"position": {"x": "NaN", "y": 0., "z": 1.},
                     "orientation": {"x": 0., "y": 0., "z": 0., "w": 1.}}}))
        self.assertTrue(event.wait(2.))
        name, message = seen[0]
        self.assertEqual(name, "UCB_P2")
        self.assertEqual(message._received_monotonic_s, 100.)
        self.assertEqual(message.header.stamp.nanosec, 345)
        self.assertTrue(math.isnan(message.pose.position.x))
        self.assertEqual(message.pose.orientation.w, 1.)

    def test_mandatory_message_info_argument_is_still_supported(self):
        class String:
            __module__ = "std_msgs.msg._string"
        String.__qualname__ = "String"
        event, seen = threading.Event(), []
        def callback(message, info):
            seen.append((message.data, info.publisher_gid))
            event.set()
        entity = self.subscription(String, callback)
        self.assertTrue(self.connection.subscriptions[entity][2])
        self.send(dict(op="sample", entity=entity, received=100., publisher_gid=[7, 8], message={"data": "diagnostic"}))
        self.assertTrue(event.wait(2.))
        self.assertEqual(seen, [("diagnostic", [7, 8])])

    def test_large_field_payload_uint64_and_nonfinite_are_not_truncated(self):
        # The field API accepts 9 MiB, which is larger than ordinary control IPC.
        text = "x" * (9 * 1024 * 1024)
        values = [math.nan, math.inf, -math.inf]
        thread = threading.Thread(target=self.connection.send,
            args=({"data": text, "id": 2**64 - 1, "values": values},))
        thread.start()
        received = self.receive()
        thread.join(timeout=5)
        self.assertFalse(thread.is_alive())
        self.assertEqual(received["data"], text)
        self.assertEqual(received["id"], 2**64 - 1)
        self.assertEqual(received["values"], ["NaN", "Infinity", "-Infinity"])
        self.assertEqual(wire({"bad": math.nan}), {"bad": "NaN"})

    def test_old_snapshot_cannot_replace_newer_edge_or_reage_it(self):
        seen = []
        event, first = threading.Event(), threading.Event()
        def callback(message):
            seen.append((message.data, message._received_monotonic_s))
            first.set()
            if len(seen) == 2:
                event.set()
        self.connection.subscriptions[1] = object, callback, False
        for received in (100., 99., 101.):
            self.send(dict(op="sample", entity=1, received=received, message={"data": received}))
            if received == 100.:
                self.assertTrue(first.wait(2.))
        self.assertTrue(event.wait(2.))
        self.assertEqual(seen, [(100., 100.), (101., 101.)])

    def test_invalid_length_fails_all_pending_queries(self):
        _, future = self.connection.request("parameter", name="test")
        self.receive()
        self.parent.sendall(struct.pack("!I", MAX_FRAME + 1))
        with self.assertRaises(ConnectionError):
            future.result(timeout=2.)

    def test_query_error_is_not_success(self):
        seq, future = self.connection.request("client_call", entity=42)
        self.receive()
        self.send(dict(op="reply", id=seq, error="backend unavailable"))
        with self.assertRaisesRegex(RuntimeError, "backend unavailable"):
            future.result(timeout=2.)

    def test_non_network_timer_failure_propagates_for_supervisor_restart(self):
        def broken():
            raise RuntimeError("fatal timer failure")
        self.connection.timers.append(SimpleNamespace(active=True, next=0., period=.1, callback=broken))
        with self.assertRaisesRegex(RuntimeError, "fatal timer failure"):
            self.connection.spin(threading.Event())

    def test_idle_seal_counts_queued_and_running_normal_and_urgent_rpcs(self):
        for topic, pool in (("/hope/lifecycle/start", self.connection.normal),
                            ("/hope/runner/emergency_passive", self.connection.urgent)):
            with self.subTest(topic=topic):
                class Trigger:
                    Response = SimpleNamespace
                callback = Mock(return_value=dict(success=True, message="done"))
                self.connection.services[10] = Trigger, callback, topic
                queued = []
                original = pool.submit
                pool.submit = lambda job: queued.append(job)
                try:
                    self.connection._service(dict(entity=10, id=50, request={}))
                    self.assertEqual(self.connection.admission.pending, 1)
                    self.assertFalse(self.connection.admission.seal_if(lambda: True))
                    callback.assert_not_called()  # Queued work also prevents restart.
                    queued[0]()
                    self.assertEqual(self.receive()["id"], 50)
                    self.assertEqual(self.connection.admission.pending, 0)
                finally:
                    pool.submit = original
        self.assertTrue(self.connection.admission.seal_if(lambda: True))
        self.connection._service(dict(entity=10, id=51, request={}))
        result = self.receive()
        self.assertIn("not accepted or replayed", result["error"])
        callback.assert_called_once()

    def test_start_racing_idle_probe_invalidates_seal_even_after_callback_returns(self):
        class Trigger:
            Response = SimpleNamespace
        callback = Mock(return_value=dict(success=True))
        self.connection.services[10] = Trigger, callback, "/hope/lifecycle/start"
        checking, release, calling = threading.Event(), threading.Event(), threading.Event()
        self.addCleanup(release.set)
        sealed = []
        def quiescent():
            checking.set()
            self.assertTrue(release.wait(2.))
            return True
        seal = threading.Thread(target=lambda: sealed.append(self.connection.admission.seal_if(quiescent)))
        seal.start()
        self.assertTrue(checking.wait(2.))
        def request():
            calling.set()
            self.connection._service(dict(entity=10, id=52, request={}))
        start = threading.Thread(target=request)
        start.start()
        self.assertTrue(calling.wait(2.))
        result = self.receive()
        self.assertTrue(result["response"]["success"])
        start.join(timeout=2.)
        release.set()
        seal.join(timeout=2.)
        start.join(timeout=2.)
        self.assertFalse(seal.is_alive() or start.is_alive())
        self.assertEqual(sealed, [False])
        callback.assert_called_once()

    def test_diagnostic_burst_keeps_latest_per_subscription_and_rpc_link_alive(self):
        started, release, finished = threading.Event(), threading.Event(), threading.Event()
        seen = []
        self.addCleanup(release.set)
        def consume(message):
            seen.append((message.data, message._received_monotonic_s))
            started.set()
            release.wait(3.)
            if message.data == 699:
                finished.set()
        self.connection.subscriptions[1] = object, consume, False
        self.send(dict(op="sample", entity=1, received=1., message=dict(data=0)))
        self.assertTrue(started.wait(2.))
        for i in range(1, 700):
            self.send(dict(op="sample", entity=1, received=float(i+1), message=dict(data=i)))
        # A reply behind the entire burst is processed while the callback waits.
        seq, future = self.connection.request("player_status")
        self.receive()
        self.send(dict(op="reply", id=seq, value=True))
        self.assertTrue(future.result(timeout=2.))
        self.assertFalse(self.connection.closed.is_set())
        with self.connection.callback_condition:
            self.assertEqual(len(self.connection.callbacks), 1)
        release.set()
        self.assertTrue(finished.wait(2.))
        self.assertEqual(seen, [(0, 1.), (699, 700.)])

    def test_reconnect_probe_cannot_block_reader_player_reply_or_urgent_service(self):
        from hope_operations_native_state import NativePlayerPositions
        owner_lock = threading.Lock()
        probing, urgent = threading.Event(), threading.Event()
        result, seals = [], []
        class Trigger:
            Response = SimpleNamespace
        self.connection.services[10] = (Trigger,
            lambda req, response: (urgent.set() or dict(success=True)),
            "/hope/runner/emergency_passive")
        def player_query():
            with owner_lock:
                result.append(NativePlayerPositions(self.connection).status(0))
        def quiescent():
            probing.set()
            with owner_lock:
                return False  # An asynchronously STARTing session still owns hardware.
        query = threading.Thread(target=player_query)
        query.start()
        request = self.receive()
        probe = threading.Thread(target=lambda: seals.append(self.connection.admission.seal_if(quiescent)))
        probe.start()
        self.assertTrue(probing.wait(2.))
        # The old reader blocked on admission before reading this query reply.
        self.send(dict(op="call", entity=10, id=71, request={}))
        self.send(dict(op="reply", id=request["id"], value={"P1": True}))
        self.assertTrue(urgent.wait(1.))
        query.join(timeout=1.); probe.join(timeout=1.)
        self.assertFalse(query.is_alive() or probe.is_alive())
        self.assertEqual(result, [{"P1": True}])
        self.assertEqual(seals, [False])
        self.assertFalse(self.connection.closed.is_set())

    def test_client_cancel_releases_pending_query_and_notifies_native_owner(self):
        class Trigger:
            __module__ = "std_srvs.srv._trigger"
        Trigger.__qualname__ = "Trigger"
        result = []
        thread = threading.Thread(target=lambda: result.append(WorkerNode(self.connection).create_client(
            Trigger, "/isolated/calibration")))
        thread.start()
        registration = self.receive()
        self.send(dict(op="reply", id=registration["id"], value=True))
        thread.join(timeout=2.)
        self.assertFalse(thread.is_alive())
        future = result[0].call_async({})
        call = self.receive()
        self.assertTrue(future.cancel())
        cancel = self.receive()
        self.assertEqual(cancel, dict(op="client_cancel", call_id=call["id"]))
        self.assertNotIn(call["id"], self.connection.pending)

    @unittest.skipUnless(os.environ.get("ROS_LOCALHOST_ONLY") == "1"
                         and os.environ.get("ROS_DOMAIN_ID") == "219",
                         "generated CDR decoder requires isolated Jazzy environment")
    def test_cdr_decoder_and_publisher_gid_work_without_ros_initialization(self):
        from unittest.mock import patch
        import rclpy
        from rclpy.serialization import serialize_message
        from std_msgs.msg import String
        event, seen = threading.Event(), []
        def callback(message, info):
            seen.append((message.data, info.publisher_gid))
            event.set()
        self.connection.subscriptions[5] = String, callback, True
        with patch.object(rclpy, "init", side_effect=AssertionError("worker must not initialize ROS")):
            self.send(dict(op="sample", entity=5, received=time.monotonic(), publisher_gid=[1, 2, 3],
                           cdr=list(serialize_message(String(data="vendor diagnostic container")))))
            self.assertTrue(event.wait(2.))
        self.assertEqual(seen, [("vendor diagnostic container", [1, 2, 3])])
