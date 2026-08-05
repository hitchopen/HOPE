import math
from pathlib import Path
import sys
import unittest


A3_DIR = Path(__file__).resolve().parents[1] / "a3"
sys.path.insert(0, str(A3_DIR))

from hope_monitor_core import parse_chrony_status, probe_mocap, stale_sources  # noqa: E402


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
    def test_unconfigured_mocap_does_not_run_ping(self):
        result = probe_mocap("REPLACE_WITH_MOCAP_HOST")
        self.assertFalse(result.reachable)
        self.assertTrue(math.isnan(result.rtt_ms))
        self.assertIn("not configured", result.error)

    def test_missing_and_old_sources_are_reported_in_configured_order(self):
        expected = ["leg", "arm", "hand"]
        stale = stale_sources(
            {"leg": 9.8, "arm": 8.0},
            expected,
            now_monotonic=10.0,
            stale_after_s=0.5,
        )
        self.assertEqual(stale, ["arm", "hand"])


if __name__ == "__main__":
    unittest.main()
