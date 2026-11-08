"""Idle-only combined DDS reconnect; no networking or hardware is changed."""
import ast
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

A3 = Path(__file__).resolve().parents[1] / "a3"
sys.path.insert(0, str(A3))
from hope_operations_transport import ServiceAdmission
from hope_operations_worker import IdleNetworkReconnect, DdsReconnectRequired


# Exercise the actual state predicate without requiring an initialized ROS Node.
tree = ast.parse((A3 / "hope_lifecycle_supervisor.py").read_text())
owner = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "LifecycleModule")
method = next(n for n in owner.body if isinstance(n, ast.FunctionDef) and n.name == "quiescent")
scope = {}
exec(compile(ast.Module(body=[method], type_ignores=[]), str(A3 / "hope_lifecycle_supervisor.py"), "exec"), scope)


class IdleNetworkTests(unittest.TestCase):
    def setUp(self):
        self.lifecycle = SimpleNamespace(_lock=threading.Lock(), _state="STOPPED", _busy=False,
            _field_busy=False, _field_refresh_running=False, _hardware_operation_lock=None,
            _rpc_workers=SimpleNamespace(busy=False), _record_workers=SimpleNamespace(busy=False))
        self.lifecycle.quiescent = lambda: scope["quiescent"](self.lifecycle)
        self.proxy = SimpleNamespace(_action_lock=threading.Lock())
        self.proxy.quiescent = lambda: not self.proxy._action_lock.locked()
        self.sent = []
        self.connection = SimpleNamespace(admission=ServiceAdmission(), send=self.sent.append,
                                          query=Mock(return_value=False))
        self.current = (("eth0", "192.168.8.182"),)
        self.read = Mock(side_effect=lambda: self.current)
        self.watch = IdleNetworkReconnect(self.connection, self.lifecycle, self.proxy, Mock(), self.read)
        self.current = (("eth0", "192.168.8.183"),)

    def assert_deferred(self):
        self.watch.poll()
        self.assertTrue(self.watch.status.startswith("DDS_RECONNECT_DEFERRED"))
        self.assertFalse(self.connection.admission.sealed)
        self.assertTrue(self.connection.admission.enter(), "commands must remain admitted")
        self.connection.admission.leave()

    def test_all_owned_or_transition_states_defer_without_losing_interlock(self):
        lock = object()
        for state in ("RUNNING", "STARTING", "KILLING", "FAILED"):
            with self.subTest(state=state):
                self.lifecycle._state = state
                self.lifecycle._hardware_operation_lock = lock
                self.assert_deferred()
                self.assertIs(self.lifecycle._hardware_operation_lock, lock)
                self.assertEqual(self.lifecycle._state, state)
        self.lifecycle._state = "STOPPED"
        self.assert_deferred()  # A retained lock is never lost even if state disagrees.

    def test_busy_async_field_and_command_operations_defer(self):
        for name in ("_busy", "_field_busy", "_field_refresh_running"):
            with self.subTest(name=name):
                setattr(self.lifecycle, name, True)
                self.assert_deferred()
                setattr(self.lifecycle, name, False)
        with self.proxy._action_lock:
            self.assert_deferred()
        self.assertTrue(self.connection.admission.enter())
        self.assert_deferred()  # Also covers commands outside the proxy action lock.
        self.connection.admission.leave()

    def test_native_emergency_and_unknown_native_state_defer_reconnect(self):
        self.connection.query.return_value = True
        self.assert_deferred()
        self.connection.query.assert_called_with("runner_busy")
        self.connection.query.side_effect = TimeoutError("native reply delayed")
        self.assert_deferred()
        self.connection.query.side_effect = None
        self.connection.query.return_value = False
        with self.assertRaises(DdsReconnectRequired):
            self.watch.poll()

    def test_stopped_and_config_error_idle_reconnect_only_after_sealing(self):
        for state in ("STOPPED", "CONFIG_ERROR"):
            with self.subTest(state=state):
                self.connection.admission = ServiceAdmission()
                self.lifecycle._state = state
                with self.assertRaises(DdsReconnectRequired):
                    self.watch.poll()
                self.assertTrue(self.connection.admission.sealed)
                self.assertFalse(self.connection.admission.enter())
                self.assertTrue(self.sent[-1]["status"].startswith("DDS_RECONNECTING"))

    def test_accepted_start_retains_ownership_after_rpc_finishes(self):
        self.assertTrue(self.connection.admission.enter())
        self.assert_deferred()  # START callback has not claimed async work yet.
        self.lifecycle._state, self.lifecycle._busy = "STARTING", True
        self.lifecycle._hardware_operation_lock = object()
        self.connection.admission.leave()
        self.assert_deferred()
        self.lifecycle._state, self.lifecycle._busy = "RUNNING", False
        self.assert_deferred()  # Original RPC is done, session still owns drives.
        self.lifecycle._state, self.lifecycle._busy = "KILLING", True
        self.assert_deferred()
        self.lifecycle._state, self.lifecycle._busy = "STOPPED", False
        self.lifecycle._hardware_operation_lock = None
        with self.assertRaises(DdsReconnectRequired):
            self.watch.poll()

    def test_probe_failure_and_return_to_original_addresses_do_not_restart(self):
        self.lifecycle._state = "RUNNING"
        self.assert_deferred()
        self.current = self.watch.baseline
        self.watch.poll()
        self.assertTrue(self.sent[-1]["status"].startswith("READY:"))
        self.current = None
        self.lifecycle._state = "STOPPED"
        self.watch.poll()
        self.assertFalse(self.connection.admission.sealed)
        self.watch.baseline = None
        self.current = (("eth1", "192.168.100.132"),)
        self.watch.poll()
        self.assertEqual(self.watch.baseline, self.current)
        self.assertFalse(self.connection.admission.sealed)
