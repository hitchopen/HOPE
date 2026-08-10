import importlib.util
import math
import sys
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "mocap_frame_probe.py"
SPEC = importlib.util.spec_from_file_location("mocap_frame_probe", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class MocapFrameProbeTest(unittest.TestCase):
    def _samples(self, cls, xyz, quat=None, count=30):
        rows = []
        for index in range(count):
            wobble = 0.0001 * math.sin(index)
            values = [index * 0.01, xyz[0] + wobble, xyz[1], xyz[2] + wobble]
            if quat is not None:
                values.extend(quat)
            rows.append(cls(*values))
        return rows

    def _evaluate(self, *, ball_z=0.02):
        return MODULE.evaluate_frame_samples(
            self._samples(MODULE.PositionSample, [0.7, -0.7, ball_z]),
            self._samples(MODULE.PoseSample, [-0.1, -0.7, 0.15], [1, 0, 0, 0]),
            min_samples=20,
            ball_radius_m=0.02,
            ball_z_tolerance_m=0.015,
            ball_speed_p95_max_mps=0.10,
            marker_to_base_xyz=[0.0, 0.0, 0.0],
            policy_z_offset_m=0.76,
            base_policy_z_min_m=0.70,
            base_policy_z_max_m=1.20,
        )

    def test_live_vertical_contract_passes_without_table_topic(self):
        report = self._evaluate()
        self.assertTrue(report["pass"], report)
        self.assertEqual(report["warnings"], [])
        self.assertFalse(report["scope"]["live_table_pose_required"])
        self.assertAlmostEqual(
            report["checks"]["policy_base_height"]["base_policy_xyz_m"][2], 0.91, places=3
        )

    def test_floor_origin_ball_is_rejected_before_double_z_offset(self):
        report = self._evaluate(ball_z=0.78)
        self.assertFalse(report["pass"])
        self.assertTrue(any("Ball centre z" in error for error in report["errors"]))

if __name__ == "__main__":
    unittest.main()
