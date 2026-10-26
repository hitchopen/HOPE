"""Execute the production action method without ROS or robot transport.

Only methods are extracted; the actual command codec is used. In-memory
acknowledgements test routing, not physical safety or robot mode transitions.
"""
from __future__ import annotations

import ast
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "a3"))
from hope_observer_core import DecodeError
from hope_runner_control_core import action_succeeded, encode_runner_request


def load_methods():
    tree = ast.parse((ROOT / "a3/hope_command_proxy.py").read_text())
    cls = next(item for item in tree.body if isinstance(item, ast.ClassDef)
               and item.name == "HopeCommandProxy")
    names = {"_run_runner_action", "_runner_action_callback"}
    functions = [item for item in cls.body if isinstance(item, ast.FunctionDef)
                 and item.name in names]
    assert len(functions) == len(names)
    assert not any(isinstance(item, ast.FunctionDef) and
                   item.name == "_elink_health_callback" for item in cls.body)
    future = ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0)
    module = ast.fix_missing_locations(ast.Module(body=[future, *functions], type_ignores=[]))
    namespace = dict(Float64MultiArray=SimpleNamespace, DecodeError=DecodeError,
                     encode_runner_request=encode_runner_request, action_succeeded=action_succeeded,
                     time=time)
    exec(compile(module, str(ROOT / "a3/hope_command_proxy.py"), "exec"), namespace)
    return namespace


METHODS = load_methods()


class ProxyObserveOnlyTests(unittest.TestCase):
    def setUp(self):
        self.messages = []
        self.node = SimpleNamespace(
            _action_lock=threading.Lock(), _next_runner_request_id=lambda: 123,
            _runner_action_timeout_s=0.01, _runner_state_condition=threading.Condition(),
            _runner_state=None, _runner_state_error="",
        )

        def publish(message):
            self.messages.append(message.data)
            self.node._runner_state = SimpleNamespace(
                last_action_id=123, last_action_result="APPLIED", last_action="ENTER_MOTION",
                last_action_reason="OK", boot_id=1, state_sequence=2,
                run_mode="PD_STAND", local_role="SERVER")

        self.node._runner_request_publisher = SimpleNamespace(publish=publish)
        self.node._run_runner_action = lambda *a, **kw: METHODS["_run_runner_action"](self.node, *a, **kw)

    def act(self, action="ENTER_MOTION", urgent=False):
        return self.node._run_runner_action(action, SimpleNamespace(), urgent=urgent)

    def test_all_normal_operations_need_no_telemetry_clock_or_audit(self):
        # No observer, contract, decoder, current-session file or clock guard
        # exists on this fixture. None is an action admission prerequisite.
        for action in ("ENTER_PD_STAND", "PREPARE_SERVE", "CONFIRM_BALL_LOADED",
                       "READY_TO_SERVE", "ENTER_MOTION", "OPEN_GRIPPER",
                       "CONFIRM_GRIP_SECURE", "SET_SERVER", "SET_RECEIVER"):
            with self.subTest(action=action):
                self.setUp()
                self.assertTrue(self.act(action).success)
                self.assertEqual(self.messages, [encode_runner_request(123, action)])
                self.assertFalse(self.node._action_lock.locked())

    def test_unknown_fault_stale_clock_and_old_session_diagnostics_do_not_veto(self):
        for status in ("UNKNOWN", "FAULT", "CLOCK_UNQUALIFIED", "OLD_SESSION"):
            for action in ("ENTER_PD_STAND", "PREPARE_SERVE", "ENTER_MOTION"):
                with self.subTest(status=status, action=action):
                    self.setUp()
                    # Deliberately inert diagnostic data must never be consulted.
                    self.node._elink_diagnostic = dict(
                        state=status, functional_ready=False, fault_latched=True)
                    self.assertTrue(self.act(action).success)
                    self.assertEqual(len(self.messages), 1)

    def test_no_command_is_emitted_without_explicit_request(self):
        self.assertEqual(self.messages, [])

    def test_ordinary_actions_remain_serialized(self):
        self.node._action_lock.acquire()
        try:
            response = self.act()
            self.assertFalse(response.success)
            self.assertIn("already in progress", response.message)
            self.assertEqual(self.messages, [])
            self.assertTrue(self.node._action_lock.locked())
        finally:
            self.node._action_lock.release()

    def test_only_emergency_callback_bypasses_ordinary_waiting_action(self):
        self.node._action_lock.acquire()
        try:
            stand = METHODS["_runner_action_callback"](self.node, "ENTER_PD_STAND")
            self.assertFalse(stand(None, SimpleNamespace()).success)
            self.assertEqual(self.messages, [])
            emergency = METHODS["_runner_action_callback"](self.node, "EMERGENCY_PASSIVE")
            self.assertTrue(emergency(None, SimpleNamespace()).success)
            self.assertEqual(self.messages, [encode_runner_request(123, "EMERGENCY_PASSIVE")])
            self.assertTrue(self.node._action_lock.locked())
        finally:
            self.node._action_lock.release()

    def test_session_file_failure_is_not_a_new_motion_prerequisite(self):
        def missing():
            raise FileNotFoundError("session unavailable")
        self.node._current_session_id = missing
        self.assertTrue(self.act().success)
        self.assertFalse(self.node._action_lock.locked())

    def test_native_state_machine_denial_is_returned_not_reported_as_success(self):
        original = self.node._runner_request_publisher.publish
        def rejected(message):
            original(message)
            self.node._runner_state.last_action_result = "REJECTED_WRONG_MODE"
            self.node._runner_state.last_action_reason = "WRONG_MODE"
            self.node._runner_state.run_mode = "PASSIVE"
        self.node._runner_request_publisher.publish = rejected
        response = self.act("ENTER_PD_STAND", urgent=True)
        self.assertFalse(response.success)
        self.assertIn("REJECTED_WRONG_MODE", response.message)
        self.assertEqual(len(self.messages), 1)

    def test_missing_native_ack_is_not_success(self):
        self.node._runner_request_publisher.publish = lambda msg: self.messages.append(msg.data)
        response = self.act()
        self.assertFalse(response.success)
        self.assertIn("not confirmed", response.message)
        self.assertEqual(len(self.messages), 1)
        self.assertFalse(self.node._action_lock.locked())

    def test_invalid_action_does_not_publish(self):
        response = self.act("NOT_AN_ACTION")
        self.assertFalse(response.success)
        self.assertEqual(self.messages, [])
        self.assertFalse(self.node._action_lock.locked())


if __name__ == "__main__":
    unittest.main()
