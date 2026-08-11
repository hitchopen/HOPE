from dataclasses import dataclass
from pathlib import Path
import sys
import unittest


LAPTOP_DIR = Path(__file__).resolve().parents[1] / "laptop"
sys.path.insert(0, str(LAPTOP_DIR))

from hope_marker_monitor_core import (  # noqa: E402
    count_physical_markers,
    marker_count_text,
)


@dataclass(frozen=True)
class Point:
    x: float
    y: float
    z: float


@dataclass
class Marker:
    member_id: int
    has_live_sample: bool = True
    params: int = 0x02
    position: Point = Point(1.0, 2.0, 3.0)


class MarkerMonitorCoreTests(unittest.TestCase):
    def test_counts_only_unique_physical_live_markers(self):
        markers = [Marker(index) for index in range(8)]
        markers += [Marker(8, has_live_sample=False), Marker(9, params=0x03)]
        markers += [Marker(0)]  # A duplicate member ID does not inflate count.
        self.assertEqual(count_physical_markers(markers), (8, 8))

    def test_count_is_bounded_to_operator_contract(self):
        self.assertEqual(
            count_physical_markers([Marker(index) for index in range(12)]),
            (10, 12),
        )

    def test_text_distinguishes_stale_from_a_live_zero(self):
        self.assertEqual(
            marker_count_text(0, fresh=False),
            "P1 live markers = 0/10 | NO FRESH LAPTOP DATA",
        )
        self.assertEqual(marker_count_text(0, fresh=True), "P1 live markers = 0/10")


if __name__ == "__main__":
    unittest.main()
