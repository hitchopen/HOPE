from pathlib import Path
import sys
import unittest


A3_DIR = Path(__file__).resolve().parents[1] / "a3"
sys.path.insert(0, str(A3_DIR))

from hope_monitor_core import (  # noqa: E402
    build_software_estop_request,
    combine_estop_results,
    cpu_load_percent,
    decode_software_estop_response,
    estop_backend_status,
    message_latency_ms,
    parse_calibration_service_sha,
    parse_chrony_status,
    parse_proc_stat_cpu,
    parse_process_stat,
    stale_sources,
    timestamp_age_s,
    top_process_cpu_load,
)


def tracking_csv(*, offset_s="0.002", skew_ppm="3.0", leap="Normal"):
    return ",".join(
        [
            "506E6C47",
            "192.0.2.10",
            "2",
            "1234567890.0",
            offset_s,
            "0.0",
            "0.0",
            "1.0",
            "0.0",
            skew_ppm,
            "0.001",
            "0.004",
            "64.0",
            leap,
        ]
    )


class ChronyStatusTests(unittest.TestCase):
    def test_ready_requires_selected_source_and_numeric_gates(self):
        result = parse_chrony_status(
            tracking_csv(),
            "MS Name/IP address Stratum Poll Reach LastRx Last sample\n"
            "^* 192.0.2.10 2 6 377 20 +2us[+3us] +/- 4ms\n",
            max_offset_ms=10.0,
            max_skew_ppm=5.0,
        )
        self.assertTrue(result.utc_qualified)
        self.assertTrue(result.gate_pass)
        self.assertAlmostEqual(result.offset_ms, 2.0)
        self.assertAlmostEqual(result.root_dispersion_ms, 4.0)

    def test_holdover_without_selected_source_is_not_qualified(self):
        result = parse_chrony_status(
            tracking_csv(),
            "^? 192.0.2.10 2 6 0 100 +2us[+3us] +/- 4ms\n",
            max_offset_ms=10.0,
            max_skew_ppm=5.0,
        )
        self.assertFalse(result.utc_qualified)
        self.assertFalse(result.gate_pass)

    def test_gate_rejects_large_offset_or_skew(self):
        result = parse_chrony_status(
            tracking_csv(offset_s="-0.011", skew_ppm="5.1"),
            "^* 192.0.2.10 2 6 377 20 +2us[+3us] +/- 4ms\n",
            max_offset_ms=10.0,
            max_skew_ppm=5.0,
        )
        self.assertTrue(result.utc_qualified)
        self.assertFalse(result.gate_pass)

    def test_malformed_tracking_is_rejected(self):
        with self.assertRaises(ValueError):
            parse_chrony_status(
                "too,few,fields",
                "^* source",
                max_offset_ms=10.0,
                max_skew_ppm=5.0,
            )


class ProbeAndFreshnessTests(unittest.TestCase):
    def test_cpu_load_uses_aggregate_proc_stat_deltas(self):
        previous = parse_proc_stat_cpu(
            "cpu 100 0 100 700 50 10 20 20 0 0\n"
        )
        current = parse_proc_stat_cpu(
            "cpu 120 0 130 750 60 10 30 20 0 0\n"
        )
        # Total time advances by 120 ticks. idle+iowait advances by 60,
        # leaving 60 busy ticks and therefore 50% aggregate utilization.
        self.assertAlmostEqual(cpu_load_percent(previous, current), 50.0)

    def test_cpu_load_rejects_missing_or_reset_counters(self):
        with self.assertRaisesRegex(ValueError, "no aggregate"):
            parse_proc_stat_cpu("intr 1 2 3\n")
        previous = parse_proc_stat_cpu("cpu 10 0 10 80 0 0 0 0\n")
        reset = parse_proc_stat_cpu("cpu 1 0 1 8 0 0 0 0\n")
        with self.assertRaisesRegex(ValueError, "monotonically"):
            cpu_load_percent(previous, reset)

    def test_top_process_cpu_load_uses_proc_deltas(self):
        previous = {
            123: parse_process_stat(
                "123 (hope planner) R " + " ".join(["0"] * 10 + ["40", "10"])
            ),
            456: parse_process_stat(
                "456 (idle helper) S " + " ".join(["0"] * 10 + ["10", "0"])
            ),
        }
        current = {
            123: parse_process_stat(
                "123 (hope planner) R " + " ".join(["0"] * 10 + ["100", "30"])
            ),
            456: parse_process_stat(
                "456 (idle helper) S " + " ".join(["0"] * 10 + ["12", "0"])
            ),
        }
        top = top_process_cpu_load(
            previous, current, total_cpu_delta=800, cpu_count=8
        )
        self.assertIsNotNone(top)
        self.assertEqual(top.pid, 123)
        self.assertEqual(top.name, "hope planner")
        self.assertAlmostEqual(top.system_percent, 10.0)
        self.assertAlmostEqual(top.core_percent, 80.0)

    def test_missing_and_old_sources_are_reported_in_configured_order(self):
        expected = ["leg", "arm", "hand"]
        stale = stale_sources(
            {"leg": 9.8, "arm": 8.0},
            expected,
            now_monotonic=10.0,
            stale_after_s=0.5,
        )
        self.assertEqual(stale, ["arm", "hand"])


class TimestampAndSafetyTests(unittest.TestCase):
    def test_estop_result_accepts_any_confirmed_path_and_reports_partial(self):
        for vendor_ok, runner_ok, expected in (
            (True, True, True),
            (True, False, True),
            (False, True, True),
            (False, False, False),
        ):
            with self.subTest(vendor_ok=vendor_ok, runner_ok=runner_ok):
                success, message = combine_estop_results(
                    vendor_accepted=vendor_ok,
                    vendor_detail="vendor-test",
                    runner_stopped=runner_ok,
                    runner_detail="runner-test",
                )
                self.assertEqual(success, expected)
                self.assertIn("physical E-stop", message)

    def test_calibration_service_sha_is_exact(self):
        sha = "a" * 64
        self.assertEqual(parse_calibration_service_sha(f"  {sha}\n"), sha)
        for invalid in ("", "A" * 64, "a" * 63, f"sha256={sha}"):
            with self.subTest(invalid=invalid):
                with self.assertRaises(ValueError):
                    parse_calibration_service_sha(invalid)

    def test_message_latency_uses_ros_header_epoch(self):
        latency = message_latency_ms(
            1_700_000_000_012_500_000,
            1_700_000_000,
            10_000_000,
        )
        self.assertAlmostEqual(latency, 2.5)

    def test_zero_message_stamp_is_rejected(self):
        with self.assertRaises(ValueError):
            message_latency_ms(1_000_000, 0, 0)

    def test_timestamp_age_is_seconds(self):
        self.assertAlmostEqual(timestamp_age_s(2_250_000_000, 2, 0), 0.25)

    def test_estop_payload_contains_only_asserted_software_command(self):
        payload = build_software_estop_request(
            1_700_000_000_123_456_789,
            "hope-test-trace",
        )
        self.assertIn(b"hope-test-trace", payload)
        self.assertIn(b"hope-foxglove", payload)
        # EmergencyCommandReq field 2 (cmd), containing EmergencyCommand field
        # 2 (software_emergency_stop) with value true.
        self.assertTrue(payload.endswith(bytes([0x12, 0x02, 0x10, 0x01])))

    def test_estop_backend_status_distinguishes_partial_and_full_readiness(self):
        full = estop_backend_status(
            vendor_ready=True,
            runner_ready=True,
            latched=False,
            vendor_protocol_available=True,
        )
        self.assertTrue(full.action_ready)
        self.assertTrue(full.full_ready)

        runner_only = estop_backend_status(
            vendor_ready=False,
            runner_ready=True,
            latched=False,
            vendor_protocol_available=True,
        )
        self.assertTrue(runner_only.action_ready)
        self.assertFalse(runner_only.full_ready)
        self.assertIn("PARTIAL ONLY", runner_only.detail)
        self.assertIn("PHYSICAL E-STOP", runner_only.detail)

        vendor_only = estop_backend_status(
            vendor_ready=True,
            runner_ready=False,
            latched=False,
            vendor_protocol_available=True,
        )
        self.assertTrue(vendor_only.action_ready)
        self.assertFalse(vendor_only.full_ready)

        latched = estop_backend_status(
            vendor_ready=True,
            runner_ready=True,
            latched=True,
            vendor_protocol_available=True,
        )
        self.assertFalse(latched.action_ready)
        self.assertFalse(latched.full_ready)

    def test_estop_response_decodes_application_header(self):
        # EmergencyCommandRsp.header { code: 17, msg: "blocked" }
        response_header = bytes([0x08, 0x11, 0x12, 0x07]) + b"blocked"
        payload = bytes([0x0A, len(response_header)]) + response_header
        self.assertEqual(
            decode_software_estop_response(payload),
            (17, "blocked"),
        )
        # A present empty proto3 ResponseHeader carries the scalar default 0.
        self.assertEqual(decode_software_estop_response(b"\x0a\x00"), (0, ""))

    def test_estop_response_rejects_missing_or_malformed_header(self):
        with self.assertRaisesRegex(ValueError, "response headers"):
            decode_software_estop_response(b"")
        with self.assertRaisesRegex(ValueError, "truncated"):
            decode_software_estop_response(b"\x0a\x05\x08")


if __name__ == "__main__":
    unittest.main()
