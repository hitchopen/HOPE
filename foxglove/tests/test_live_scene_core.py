"""Visual shot identity uses measured events, never tracking-loss generations."""
from copy import deepcopy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "a3"))
from hope_live_scene_core import LiveSceneState  # noqa: E402


def command(stamp=100., deadline=100.2, seq=1, flight=1, x=0.):
    values = [4., 1., 1., x, -.7, 1.26, 2., 0., 1., 0., deadline, 0.,
              float(int(stamp)), (stamp-int(stamp))*1e9, float(seq), float(flight),
              0., 0., 0., 1., 0., 0.]
    return values


def status(session="live", producer="packetizer-a", generation=1, missing=False):
    return dict(session_id=session, producer_instance_id=producer, generation=str(generation),
                state="OPPONENT_RALLY", reason="ball_observation_missing" if missing else "qualified_launch",
                observation_missing=missing, forwarding=not missing)


def cross(scene, start=100.18, y=-.7, z=.5):
    assert scene.ball(start, [.04, y, z], start)
    assert scene.ball(start+.01, [-.04, y, z], start+.01)


class LiveSceneLifecycleTests(unittest.TestCase):
    def scene(self):
        scene = LiveSceneState()
        scene.status(status(), 99.9)
        scene.set_plane([1., 100., 0.], 100.)
        return scene

    def test_occlusion_state_generation_preserves_all_ghosts(self):
        scene = self.scene()
        self.assertTrue(scene.command(command(), 100.))
        self.assertTrue(scene.freeze_policy({"timestamp_s": 100.19}, [.01, -.69, .51], 100.2))
        cross(scene)
        before = deepcopy((scene.generation, scene.target, scene.frozen, scene.policy_frozen))
        scene.status(status(generation=20, missing=True), 101.)
        self.assertEqual((scene.generation, scene.target, scene.frozen, scene.policy_frozen), before)
        self.assertEqual(scene.snapshot(101.)["play_state"]["state"], "OPPONENT_RALLY")
        self.assertIsNone(scene.snapshot(101.)["ball_xyz"])
        scene.status(status(generation=21, missing=False), 101.1)
        self.assertEqual((scene.generation, scene.target, scene.frozen, scene.policy_frozen), before)

    def test_new_target_after_completed_shot_clears_all_previous_ghosts(self):
        scene = self.scene()
        scene.command(command(), 100.)
        scene.freeze_policy({"timestamp_s": 100.19}, [.01, -.7, .5], 100.2)
        cross(scene)
        old_generation = scene.generation
        self.assertTrue(scene.command(command(100.5, 100.7, 2, 2), 100.5))
        self.assertGreater(scene.generation, old_generation)
        self.assertEqual(scene.target["command_seq"], 2)
        self.assertIsNone(scene.frozen)
        self.assertIsNone(scene.policy_frozen)
        self.assertIsNone(scene.policy_sample)

    def test_solver_retry_before_strike_joins_current_visual_shot(self):
        scene = self.scene()
        scene.command(command(), 100.)
        generation = scene.generation
        scene.command(command(100.1, 100.22, 2, 2), 100.1)
        self.assertEqual(scene.generation, generation)
        self.assertEqual(scene.target["flight_id"], 2)

    def test_next_incoming_ball_refreshes_without_a_planner_command(self):
        scene = self.scene()
        scene.command(command(), 100.)
        cross(scene)
        old_generation = scene.generation
        scene.ball(101., [.50, -.7, .6], 101.)
        self.assertIsNotNone(scene.frozen)  # Loss/reappearance alone is not a new shot.
        for index in range(1, 7):
            t = index*.01
            scene.ball(101.+t, [.50-5*t, -.7, .6+.5*t-4.905*t*t], 101.+t)
        self.assertGreater(scene.generation, old_generation)
        self.assertIsNone(scene.target)
        self.assertIsNone(scene.frozen)
        cross(scene, 101.10, -.5, .7)
        self.assertEqual(scene.frozen["ball_xyz"], [0., -.5, .7])

    def test_old_ball_below_table_or_outside_table_cannot_clear_ghosts(self):
        for y, z in ((.286, -.711), (-.7, -.1), (.2, .5), (-1.8, .5)):
            with self.subTest(y=y, z=z):
                scene = self.scene()
                scene.command(command(), 100.)
                cross(scene)
                held = deepcopy((scene.generation, scene.target, scene.frozen))
                for index in range(10):
                    stamp = 100.5+index*.01
                    scene.ball(stamp, [2.96-index*.01, y, z], stamp)
                self.assertEqual((scene.generation, scene.target, scene.frozen), held)

    def test_one_reverse_exposure_after_hit_is_not_a_new_incoming_ball(self):
        scene = self.scene()
        scene.command(command(), 100.)
        cross(scene)
        held = deepcopy((scene.generation, scene.target, scene.frozen))
        scene.ball(100.5, [2.50, -.7, .5], 100.5)
        scene.ball(100.51, [2.49, -.7, .5], 100.51)
        scene.ball(100.52, [2.50, -.7, .5], 100.52)
        self.assertEqual((scene.generation, scene.target, scene.frozen), held)

    def test_old_ball_crossing_below_or_far_beside_plane_does_not_replace_ghost(self):
        for y, z in ((-.7, -.5), (-2.1, .5), (.5, .5)):
            with self.subTest(y=y, z=z):
                scene = self.scene()
                scene.command(command(), 100.)
                cross(scene)
                held = deepcopy((scene.generation, scene.target, scene.frozen))
                cross(scene, 101., y, z)
                self.assertEqual((scene.generation, scene.target, scene.frozen), held)

    def test_ball_handling_behind_opponent_end_keeps_previous_ghost_until_entry(self):
        # Real 2026-10-26 replay exposed a hand-carried segment at x~3.1 m,
        # over eleven seconds before the next Planner command.
        scene = self.scene()
        scene.command(command(), 100.)
        cross(scene)
        held = deepcopy((scene.generation, scene.target, scene.frozen))
        for index in range(9):
            stamp = 101.+index*.01
            scene.ball(stamp, [3.107-index*.01, -.747, .648], stamp)
        self.assertEqual((scene.generation, scene.target, scene.frozen), held)

    def test_every_separate_measured_plane_crossing_refreshes_without_command(self):
        scene = self.scene()
        cross(scene, 100.1, -.7, .5)
        first = deepcopy(scene.frozen)
        cross(scene, 100.4, -.6, .6)
        self.assertGreater(scene.frozen["generation"], first["generation"])
        self.assertGreater(scene.frozen["timestamp_s"], first["timestamp_s"])
        self.assertEqual(scene.frozen["ball_xyz"], [0., -.6, .6])

    def test_late_command_keeps_already_measured_crossing(self):
        for producer_stamp in (100.08, 100.105):
            with self.subTest(producer_stamp=producer_stamp):
                scene = self.scene()
                cross(scene, 100.1)
                measured = deepcopy(scene.frozen)
                generation = scene.generation
                self.assertTrue(scene.command(command(producer_stamp, 100.11), 100.15))
                self.assertEqual(scene.generation, generation)
                self.assertEqual(scene.frozen, measured)
                self.assertIsNotNone(scene.target)

    def test_gaps_fast_jumps_and_unobserved_plane_do_not_invent_crossings(self):
        for dt, first, second in ((.10, .04, -.04), (.0005, .04, -.04), (.01, 1., -1.)):
            with self.subTest(dt=dt, first=first):
                scene = self.scene()
                scene.ball(100., [first, -.7, .5], 100.)
                scene.ball(100.+dt, [second, -.7, .5], 100.+dt)
                self.assertIsNone(scene.frozen)
        scene = self.scene()
        scene.command(command(), 100.)
        scene.ball(100.2, [-.3, -.7, .5], 100.2)
        self.assertIsNone(scene.frozen)

    def test_unobserved_plane_does_not_suppress_measured_policy_at_deadline(self):
        scene = self.scene()
        scene.command(command(), 100.)
        self.assertTrue(scene.freeze_policy({"timestamp_s": 100.19}, [.01, -.7, .52], 100.2))
        self.assertIsNone(scene.frozen)
        self.assertAlmostEqual(scene.policy_frozen["sample_age_ms"], 10.)
        self.assertAlmostEqual(scene.policy_frozen["delta_xyz"][2], .02)
        first = deepcopy(scene.policy_frozen)
        self.assertFalse(scene.freeze_policy({"timestamp_s": 100.195}, [.2, -.7, .52], 100.2))
        self.assertEqual(scene.policy_frozen, first)

    def test_policy_ghost_requires_current_past_joint_sample(self):
        for sample in (None, {"timestamp_s": 100.201}, {"timestamp_s": 100.16}):
            with self.subTest(sample=sample):
                scene = self.scene()
                scene.command(command(), 100.)
                self.assertFalse(scene.freeze_policy(sample, [0., -.7, .5], 100.2))
                self.assertIsNone(scene.policy_frozen)

    def test_duplicate_out_of_order_or_old_ball_cannot_replace_crossing(self):
        scene = self.scene()
        cross(scene, 100.1)
        measured = deepcopy(scene.frozen)
        previous = deepcopy(scene.previous)
        for stamp, now, xyz in ((100.11, 100.12, [.04, -.4, .8]),
                                (100.10, 100.12, [.04, -.4, .8]),
                                (100.2, 101., [.04, -.4, .8]),
                                (100.2, 100.2, [float("nan"), 0., 0.])):
            self.assertFalse(scene.ball(stamp, xyz, now))
        self.assertEqual(scene.previous, previous)
        self.assertEqual(scene.frozen, measured)

    def test_duplicate_reordered_expired_commands_do_not_replace_target(self):
        scene = self.scene()
        scene.command(command(), 100.)
        target = deepcopy(scene.target)
        for values, now in ((command(100.1, 100.2, 1, 2), 100.1),
                            (command(99.99, 100.2, 2, 2), 100.),
                            (command(98., 100.2, 2, 2), 100.),
                            (command(100.1, 99., 2, 2), 100.1)):
            self.assertFalse(scene.command(values, now))
        self.assertEqual(scene.target, target)

    def test_session_or_packetizer_restart_clears_prior_shot_and_accepts_new_sequence(self):
        for new_status in (status(session="next"), status(producer="packetizer-b")):
            with self.subTest(new_status=new_status):
                scene = self.scene()
                scene.command(command(seq=50), 100.)
                cross(scene)
                scene.freeze_policy({"timestamp_s": 100.19}, [0., -.7, .5], 100.2)
                scene.status(new_status, 101.)
                self.assertEqual(scene.generation, 0)
                self.assertIsNone(scene.target)
                self.assertIsNone(scene.frozen)
                self.assertIsNone(scene.policy_frozen)
                self.assertIsNone(scene.previous)
                self.assertEqual(len(scene.trail), 0)
                self.assertIsNone(scene.plane_x)
                self.assertTrue(scene.command(command(101., 101.2, 1, 1), 101.))

    def test_hit_plane_update_rejects_source_time_reordering(self):
        scene = self.scene()
        scene.set_plane([1., 100.1, .1], 100.1)
        scene.set_plane([1., 100., .9], 100.2)
        self.assertEqual(scene.plane_x, .1)
        scene.set_plane([1., 97., .8], 100.2)
        self.assertEqual(scene.plane_x, .1)

    def test_trajectory_is_retained_for_one_point_eight_seconds(self):
        scene = self.scene()
        for index in range(251):
            stamp = 100.+index*.01
            scene.ball(stamp, [.5, -.7, .5], stamp)
        self.assertGreaterEqual(scene.trail[0][0], 100.7-1e-8)
        self.assertLessEqual(scene.trail[0][0], 100.71+1e-8)


if __name__ == "__main__":
    unittest.main()
