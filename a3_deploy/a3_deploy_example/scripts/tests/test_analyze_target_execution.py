import importlib.util
from pathlib import Path
import tempfile
import unittest

import numpy as np
import pandas as pd

path = Path(__file__).resolve().parents[1]/'analyze_target_execution.py'
spec = importlib.util.spec_from_file_location('target_execution', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class TargetExecutionTest(unittest.TestCase):
    def test_opposite_paddle_face_is_same_plane(self):
        result = module.angles([-2, 0, 0], [1, 0, 0], [-3, 0, 0])
        self.assertEqual(result['normal_plane_deg'], 0.)
        self.assertEqual(result['normal_directed_deg'], 180.)
        self.assertEqual(result['normal_incoming_face_deg'], 0.)

    def test_tilt_is_measured_after_normalization(self):
        result = module.angles([1, 0, 1], [3, 0, 0])
        self.assertAlmostEqual(result['normal_plane_deg'], 45.)

    def test_invalid_normal_is_not_replaced(self):
        for invalid in [[0, 0, 0], [np.nan, 0, 1]]:
            with self.assertRaises(ValueError):
                module.angles(invalid, [1, 0, 0])

    def fixture(self, root):
        rows = []
        for t in [.98, 1.02]:
            row = {'wall_time_ns': t*1e9, 'frozen_flight_id': 7}
            for prefix, values in [('racket_', [1.02, 2, 3]), ('racket_v', [1.1, 0, 0]),
                                   ('racket_n', [-1, 0, 0])]:
                row.update({prefix+axis: value for axis, value in zip('xyz', values)})
            rows.append(row)
        pd.DataFrame(rows).to_csv(root/'pp_runner_trace.csv', index=False)
        shot = dict(wire_valid=1, flight_id=7, swing_sign=-1, strike_deadline_wall_s=1.)
        for prefix, values in [('target_actor_', [1, 2, 3]), ('racket_v', [1, 0, 0]),
                               ('target_n', [1, 0, 0]), ('strike_v', [-3, 0, 0])]:
            shot.update({prefix+axis: value for axis, value in zip('xyz', values)})
        pd.DataFrame([shot]).to_csv(root/'pp_planner_debug.csv', index=False)

    def test_all_three_targets_at_same_deadline(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.fixture(root)
            result = module.analyze(root)
            self.assertEqual(result['invalid'], [])
            row = result['rows'][0]
            self.assertAlmostEqual(row['position_cm'], 2.)
            self.assertAlmostEqual(row['velocity_mps'], .1)
            self.assertAlmostEqual(row['normal_plane_deg'], 0.)

    def test_other_flight_must_not_fill_missing_deadline(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.fixture(root)
            trace = pd.read_csv(root/'pp_runner_trace.csv')
            trace.loc[1, 'frozen_flight_id'] = 8
            trace.to_csv(root/'pp_runner_trace.csv', index=False)
            result = module.analyze(root)
            self.assertEqual(result['rows'], [])
            self.assertIn('not bracketed', result['invalid'][0]['reason'])

    def test_nonfinite_position_fails_instead_of_polluting_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.fixture(root)
            trace = pd.read_csv(root/'pp_runner_trace.csv')
            trace.loc[0, 'racket_x'] = np.nan
            trace.to_csv(root/'pp_runner_trace.csv', index=False)
            result = module.analyze(root)
            self.assertEqual(result['rows'], [])
            self.assertIn('nonfinite', result['invalid'][0]['reason'])

    def test_zero_speed_target_keeps_independent_normal_measurement(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.fixture(root)
            plans = pd.read_csv(root/'pp_planner_debug.csv')
            plans.loc[0, 'racket_vx'] = 0.
            plans.to_csv(root/'pp_planner_debug.csv', index=False)
            result = module.analyze(root)
            self.assertEqual(result['invalid'], [])
            self.assertEqual(result['rows'][0]['normal_plane_deg'], 0.)
            self.assertNotIn('normal_vs_velocity_target_deg', result['rows'][0])


if __name__ == '__main__':
    unittest.main()
