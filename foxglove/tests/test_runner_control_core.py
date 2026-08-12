from pathlib import Path
import sys
import unittest


A3_DIR = Path(__file__).resolve().parents[1] / "a3"
sys.path.insert(0, str(A3_DIR))

from hope_observer_core import DecodeError  # noqa: E402
from hope_runner_control_core import (  # noqa: E402
    action_succeeded,
    decode_runner_state,
    encode_runner_request,
    opponent_expected_role,
    runner_session_fingerprint,
)


def valid_state() -> list[float]:
    return [
        1.0,  # schema
        1234.0,  # boot
        8.0,  # state sequence
        1.0,  # PD_STAND
        1.0,  # publishing
        1.0,  # policy native
        0.0,  # fault
        2.0,  # RECEIVER
        1.0,  # role epoch
        1.0,  # role change allowed
        1.0,  # role APPLIED
        1.0,  # ROLE_CHANGED
        0.0,  # serve unavailable
        -1.0,  # serve state unavailable
        99.0,  # last action id
        2.0,  # SET_RECEIVER
        1.0,  # APPLIED
        1.0,  # ROLE_CHANGED
        555.0,  # session fingerprint
    ]


class RunnerControlWireTests(unittest.TestCase):
    def test_fixed_request_encoder_exposes_only_seven_actions(self):
        self.assertEqual(
            encode_runner_request(42, "ENTER_MOTION"),
            [1.0, 42.0, 4.0, 0.0],
        )
        self.assertEqual(
            encode_runner_request(43, "READY_TO_SERVE"),
            [1.0, 43.0, 7.0, 0.0],
        )
        self.assertEqual(
            encode_runner_request(44, "SERVE"),
            [1.0, 44.0, 8.0, 0.0],
        )
        for forbidden in ("ENTER_SHADOW", "QUIT_RUNNER", "START_SERVE", ""):
            with self.assertRaises(DecodeError):
                encode_runner_request(42, forbidden)

    def test_decodes_authoritative_runner_state(self):
        state = decode_runner_state(valid_state())
        self.assertEqual(state.boot_id, 1234)
        self.assertEqual(state.run_mode, "PD_STAND")
        self.assertTrue(state.command_publishing)
        self.assertEqual(state.local_role, "RECEIVER")
        self.assertEqual(state.role_epoch, 1)
        self.assertEqual(state.serve_capability, "UNAVAILABLE")
        self.assertEqual(state.serve_state, "UNAVAILABLE")
        self.assertEqual(state.last_action_id, 99)
        self.assertEqual(state.last_action_result, "APPLIED")

    def test_rejects_wrong_size_schema_flags_and_unknown_codes(self):
        for mutation in (
            valid_state()[:-1],
            [2.0] + valid_state()[1:],
            valid_state()[:4] + [2.0] + valid_state()[5:],
            valid_state()[:7] + [9.0] + valid_state()[8:],
        ):
            with self.assertRaises(DecodeError):
                decode_runner_state(mutation)

    def test_serve_capability_must_match_real_serve_state(self):
        unavailable_with_idle = valid_state()
        unavailable_with_idle[13] = 0.0
        with self.assertRaisesRegex(DecodeError, "must be UNAVAILABLE"):
            decode_runner_state(unavailable_with_idle)

        available = valid_state()
        available[12] = 1.0
        available[13] = 3.0
        state = decode_runner_state(available)
        self.assertEqual(state.serve_capability, "AVAILABLE")
        self.assertEqual(state.serve_state, "AWAIT_BALL_ON_PALM")

    def test_opponent_role_is_explicitly_only_an_inference(self):
        self.assertEqual(opponent_expected_role("SERVER"), "RECEIVER")
        self.assertEqual(opponent_expected_role("RECEIVER"), "SERVER")
        self.assertEqual(opponent_expected_role("UNASSIGNED"), "UNKNOWN")
        self.assertEqual(opponent_expected_role("anything"), "UNKNOWN")

    def test_success_results_include_safe_pending_serve_abort(self):
        self.assertTrue(action_succeeded("APPLIED"))
        self.assertTrue(action_succeeded("ALREADY_SET"))
        self.assertTrue(action_succeeded("ACCEPTED_PENDING"))
        self.assertFalse(action_succeeded("REJECTED_WRONG_MODE"))

    def test_session_fingerprint_is_stable_and_bounded(self):
        first = runner_session_fingerprint("model21800_20260811T010203Z")
        second = runner_session_fingerprint("model21800_20260811T010203Z")
        different = runner_session_fingerprint("model21800_20260811T010204Z")
        self.assertEqual(first, second)
        self.assertEqual(first, 47246369472706)  # Cross-language wire fixture.
        self.assertNotEqual(first, different)
        self.assertGreater(first, 0)
        self.assertLess(first, 1 << 52)


if __name__ == "__main__":
    unittest.main()
