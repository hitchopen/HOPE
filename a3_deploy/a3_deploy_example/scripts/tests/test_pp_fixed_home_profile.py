import copy
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pp_fixed_home_profile import planner_profile

class FixedHomeProfileTest(unittest.TestCase):
    def setUp(self):
        self.receipt = {'station_contract': 'fixed_session_anchor_v1', 'policy_abi':'ball_clock_110_v1',
            'position_support': [[.44,.70,-.68,-.24,.8,1.45], [.44,.70,-.24,.28,.8,1.45]],
            'velocity_support': [[1.,2.9,-.8,1.2,.3,2.], [1.1,2.9,-1.,.9,0.,1.8]]}
    def test_metadata_drives_partition_and_plane(self):
        profile, plane = planner_profile(self.receipt)
        p = profile['hope_planner']['ros__parameters']
        self.assertAlmostEqual(plane,.57); self.assertEqual(p['swing_side_split_y'],-.24)
        self.assertEqual(p['swing_side_fh_y_range'],[-.68,-.24])
        self.assertEqual(p['racket_flat_schema'],4)
        self.assertFalse(p['x_hit_follow_robot'])
    def test_rejects_incoherent_or_nonfinite_support(self):
        for value in (float('nan'), float('inf'), .9, True):
            r=copy.deepcopy(self.receipt);r['position_support'][0][0]=value
            with self.assertRaises(ValueError):planner_profile(r)
    def test_rejects_unknown_contract(self):
        self.receipt['station_contract']='unknown'
        with self.assertRaises(ValueError):planner_profile(self.receipt)
    def test_direct_sim_topic_does_not_change_geometry(self):
        a,x=planner_profile(self.receipt, '/hope/runner/session_home_flat')
        self.assertEqual(a['hope_planner']['ros__parameters']['session_home_input_topic'], '/hope/runner/session_home_flat')
        self.assertAlmostEqual(x,.57)
if __name__=='__main__':unittest.main()
