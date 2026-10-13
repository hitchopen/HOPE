from dataclasses import dataclass
from pathlib import Path
import sys
import unittest


LAPTOP_DIR = Path(__file__).resolve().parents[1] / "laptop"
sys.path.insert(0, str(LAPTOP_DIR))

from hope_marker_monitor_core import (  # noqa: E402
    count_physical_markers,
    marker_count_text,
    model_marker_count,
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
            count_physical_markers([Marker(index) for index in range(14)], expected_count=12),
            (12, 14),
        )

    def test_text_distinguishes_stale_from_a_live_zero(self):
        self.assertEqual(
            marker_count_text(0, fresh=False, expected_count=12),
            "UCB_P1 live markers = 0/12 | NO FRESH LAPTOP DATA",
        )
        self.assertEqual(
            marker_count_text(0, fresh=True, expected_count=12), "UCB_P1 live markers = 0/12"
        )
        self.assertEqual(
            marker_count_text(12, fresh=True, expected_count=12, asset_name="UCB_P2"),
            "UCB_P2 live markers = 12/12",
        )

    def test_auto_count_follows_17_marker_modeldef_including_occluded_members(self):
        markers = [Marker(index) for index in range(17)]
        self.assertEqual(count_physical_markers(markers), (17, 17))
        markers[-1].has_live_sample = False
        markers[-2].params = 0x03
        markers.append(Marker(0))
        self.assertEqual(model_marker_count(markers), 17)
        self.assertEqual(count_physical_markers(markers), (15, 15))
        self.assertEqual(marker_count_text(15, fresh=True, expected_count=17),
                         "UCB_P1 live markers = 15/17")
        self.assertEqual(model_marker_count([]), 0)
        self.assertEqual(count_physical_markers([]), (0, 0))
        self.assertIn("0/?", marker_count_text(0, fresh=False))


if __name__ == "__main__":
    unittest.main()
