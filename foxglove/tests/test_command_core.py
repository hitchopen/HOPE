from pathlib import Path
import sys
import tempfile
import unittest


A3_DIR = Path(__file__).resolve().parents[1] / "a3"
sys.path.insert(0, str(A3_DIR))

from hope_command_core import (  # noqa: E402
    calibration_receipt_id,
    parse_calibration_receipt_sha,
    publish_x_hit_request,
    validate_request_id,
    wait_for_x_hit_status,
)
from hope_observer_core import DecodeError  # noqa: E402


class XHitRequestContractTests(unittest.TestCase):
    def test_calibration_receipt_sha_maps_to_schema_u52(self):
        receipt = "0123456789abc" + "d" * 51
        self.assertEqual(parse_calibration_receipt_sha(receipt), receipt)
        self.assertEqual(calibration_receipt_id(receipt), int(receipt[:13], 16))
        for invalid in ("", "A" * 64, "0" * 63, f"sha={receipt}"):
            with self.subTest(invalid=invalid), self.assertRaises(DecodeError):
                parse_calibration_receipt_sha(invalid)

    def test_request_id_is_decimal_and_bounded(self):
        self.assertEqual(validate_request_id("12345\n"), "12345")
        for invalid in ("", "-1", "1.5", "a123", "1" * 33):
            with self.assertRaises(DecodeError):
                validate_request_id(invalid)

    def test_atomic_publish_writes_one_complete_request(self):
        with tempfile.TemporaryDirectory() as directory:
            request_path = Path(directory) / "x_hit.request"
            publish_x_hit_request(request_path, "1786000000000000000")
            self.assertEqual(
                request_path.read_text(), "1786000000000000000\n"
            )
            self.assertEqual(list(Path(directory).glob(".*.tmp")), [])

    def test_existing_request_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            request_path = Path(directory) / "x_hit.request"
            request_path.write_text("older\n")
            with self.assertRaises(FileExistsError):
                publish_x_hit_request(request_path, "1786000000000000001")
            self.assertEqual(request_path.read_text(), "older\n")
            self.assertEqual(list(Path(directory).glob(".*.tmp")), [])

    def test_wait_accepts_only_matching_status(self):
        with tempfile.TemporaryDirectory() as directory:
            status_path = Path(directory) / "x_hit.status"
            status_path.write_text(
                "request=1786000000000000002\n"
                "success=1\n"
                "message=CALIBRATED -> x_hit=0.1500\n"
            )
            result = wait_for_x_hit_status(
                status_path,
                "1786000000000000002",
                timeout_s=0.0,
            )
            self.assertTrue(result.success)
            self.assertAlmostEqual(result.x_hit_m, 0.15)
            with self.assertRaisesRegex(TimeoutError, "latest status belongs"):
                wait_for_x_hit_status(
                    status_path,
                    "1786000000000000003",
                    timeout_s=0.0,
                )


if __name__ == "__main__":
    unittest.main()
