import math
from pathlib import Path
import sys
import unittest


A3_DIR = Path(__file__).resolve().parents[1] / "a3"
sys.path.insert(0, str(A3_DIR))

from hope_observer_core import (  # noqa: E402
    DecodeError,
    REQUIRED_BASE_FLAGS,
    decode_base_packet,
    decode_racket_packet,
    parse_planner_attempt,
    parse_positive_pid,
    parse_session_id,
    parse_x_hit_status,
    process_cmdline_matches,
)


def valid_base_packet():
    return [
        2.0,
        1.0,
        42.0,
        1_786_000_000.0,
        123_000_000.0,
        0.12,
        -0.34,
        1.01,
        1.0,
        0.0,
        0.0,
        0.0,
        0.95,
        float(REQUIRED_BASE_FLAGS),
        12345.0,
        67890.0,
    ]


def valid_racket_packet():
    return [
        2.0,
        1.0,
        1.0,
        0.70,
        -0.35,
        0.25,
        1.2,
        -0.1,
        0.4,
        0.30,
        1_786_000_000.30,
        0.0,
        1_786_000_000.0,
        100_000_000.0,
        101.0,
        9.0,
        1.0,
        64.0,
        0.18,
    ]


class SchemaDecodeTests(unittest.TestCase):
    def test_valid_base_decodes_clock_and_identity_fields(self):
        packet = decode_base_packet(valid_base_packet())
        self.assertTrue(packet.valid)
        self.assertEqual(packet.sequence, 42)
        self.assertEqual(packet.source_time_ns, 1_786_000_000_123_000_000)
        self.assertEqual(packet.position_xyz, (0.12, -0.34, 1.01))
        self.assertEqual(packet.calibration_id, 12345)
        self.assertEqual(packet.world_frame_id, 67890)

    def test_explicit_invalid_base_is_not_malformed(self):
        packet = decode_base_packet(
            [2.0, 0.0, 43.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        )
        self.assertFalse(packet.valid)
        self.assertEqual(packet.sequence, 43)
        self.assertIn("producer marked", packet.reason)

    def test_base_rejects_missing_contract_flag(self):
        values = valid_base_packet()
        values[13] = float(REQUIRED_BASE_FLAGS & ~(1 << 3))
        with self.assertRaisesRegex(DecodeError, "required base flags"):
            decode_base_packet(values)

    def test_base_rejects_nonfinite_and_bad_quaternion(self):
        values = valid_base_packet()
        values[5] = math.nan
        with self.assertRaisesRegex(DecodeError, "non-finite"):
            decode_base_packet(values)
        values = valid_base_packet()
        values[8:12] = [0.0, 0.0, 0.0, 0.0]
        with self.assertRaisesRegex(DecodeError, "quaternion norm"):
            decode_base_packet(values)

    def test_valid_racket_packet_decodes_frozen_identity(self):
        packet = decode_racket_packet(valid_racket_packet())
        self.assertTrue(packet.valid)
        self.assertEqual(packet.command_sequence, 101)
        self.assertEqual(packet.flight_id, 9)
        self.assertEqual(packet.revision_id, 1)
        self.assertEqual(packet.estimator_sample_count, 64)
        self.assertAlmostEqual(packet.estimator_span_s, 0.18)
        self.assertEqual(packet.producer_time_ns, 1_786_000_000_100_000_000)

    def test_racket_rejects_invalid_identity_or_deadline(self):
        values = valid_racket_packet()
        values[15] = 0.0
        with self.assertRaisesRegex(DecodeError, "flight id"):
            decode_racket_packet(values)
        values = valid_racket_packet()
        values[10] = 0.0
        with self.assertRaisesRegex(DecodeError, "deadline"):
            decode_racket_packet(values)

    def test_explicit_invalid_racket_preserves_command_sequence(self):
        values = [0.0] * 19
        values[0] = 2.0
        values[14] = 102.0
        packet = decode_racket_packet(values)
        self.assertFalse(packet.valid)
        self.assertEqual(packet.command_sequence, 102)


class LocalAuditParseTests(unittest.TestCase):
    def test_session_attempt_and_pid_are_narrowly_validated(self):
        self.assertEqual(
            parse_session_id("model21800_20260807T012345Z\n"),
            "model21800_20260807T012345Z",
        )
        self.assertEqual(parse_planner_attempt("planner_attempt_003"), "planner_attempt_003")
        self.assertEqual(parse_positive_pid("1234\n"), 1234)
        for invalid in ("../model21800_20260807T012345Z", "model21800_latest", ""):
            with self.assertRaises(DecodeError):
                parse_session_id(invalid)
        with self.assertRaises(DecodeError):
            parse_planner_attempt("planner_attempt_3")
        with self.assertRaises(DecodeError):
            parse_positive_pid("-1")

    def test_x_hit_status_extracts_audit_value(self):
        status = parse_x_hit_status(
            "request=1786000000000000000\n"
            "success=1\n"
            "message=CALIBRATED audit refresh base_x=-0.4300 + offset=0.5800 "
            "-> x_hit=0.1500; samples=42\n"
        )
        self.assertTrue(status.success)
        self.assertEqual(status.request_id, "1786000000000000000")
        self.assertAlmostEqual(status.x_hit_m, 0.15)
        with self.assertRaisesRegex(DecodeError, "missing fields"):
            parse_x_hit_status("success=1\n")

    def test_process_match_checks_cmdline_not_pid_alone(self):
        cmdline = b"/home/agi/hope_ws/install/hope_planner_cpp_node\0--ros-args\0"
        self.assertTrue(process_cmdline_matches(cmdline, "hope_planner_cpp_node"))
        self.assertFalse(process_cmdline_matches(cmdline, "a3_deploy_onnx_ref_pingpong"))
        self.assertFalse(process_cmdline_matches(b"", "hope_planner_cpp_node"))


if __name__ == "__main__":
    unittest.main()
