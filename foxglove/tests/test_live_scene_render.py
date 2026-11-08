"""Display lifecycle tests using real ROS marker messages, without any ROS node."""
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import sys
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'a3'))
from visualization_msgs.msg import Marker, MarkerArray
from hope_live_scene import LiveScene
from hope_live_scene_core import LiveSceneState
from hope_live_scene_geometry import RobotGeometry, SceneMarkerUpdates, marker, scene_markers


class Sink:
    def __init__(self):
        self.messages = []
    def publish(self, message):
        self.messages.append(message)


def apply_markers(scene, message):
    for m in message.markers:
        if m.action == Marker.DELETEALL:
            scene.clear()
        elif m.action == Marker.DELETE:
            scene.pop((m.ns, m.id), None)
        else:
            scene[(m.ns, m.id)] = m


class LiveSceneDisplayTests(unittest.TestCase):
    def test_expiring_ball_text_and_trail_leave_table_and_can_reappear(self):
        state = LiveSceneState()
        state.previous = (100., [.2, -.7, .4])
        state.trail.extend([(99.99, [.3, -.7, .4]), (100., [.2, -.7, .4])])
        updates, rendered = SceneMarkerUpdates(), {}
        first = updates.update(scene_markers(state, 100.01))
        apply_markers(rendered, first)
        self.assertIn(('ball', 0), rendered)
        self.assertIn(('spin', 0), rendered)
        self.assertIn(('trail', 0), rendered)
        expired = updates.update(scene_markers(state, 102.))
        self.assertNotIn(Marker.DELETEALL, [m.action for m in expired.markers])
        apply_markers(rendered, expired)
        self.assertEqual(set(rendered), {('table', 0), ('net', 0)})
        state.previous = (102.01, [.4, -.8, .5])
        restored = updates.update(scene_markers(state, 102.02))
        apply_markers(rendered, restored)
        self.assertIn(('ball', 0), rendered)
        self.assertIn(('spin', 0), rendered)
        self.assertEqual(rendered['ball', 0].pose.position.x, .4)
        self.assertNotIn(Marker.DELETEALL, [m.action for m in restored.markers])

    def geometry(self):
        geometry = RobotGeometry.__new__(RobotGeometry)
        geometry.visuals = [dict(link='pelvis_link', color=[1., 1., 1., 1.], url='http://127.0.0.1/test.STL', origin=np.eye(4))]
        return geometry

    def harness(self):
        state = LiveSceneState()
        state.session_id, state.producer = 'one', 'packetizer'
        h = SimpleNamespace(state=state, geometry=self.geometry(), scene_updates=SceneMarkerUpdates(),
            pubs={key:Sink() for key in ['robot', 'scene', 'tf', 'target', 'frozen', 'telemetry']},
            consume=lambda now:None, record=lambda *args:None, last_robot_key=None,
            last_scene_publish=0., args=SimpleNamespace(scene_hz=5.),
            last_robot_publish=0., robot_visible=False, robot_needs_clear=False,
            last_target_key=('one', 'packetizer', 0, None),
            last_frozen_key=('one', 'packetizer', 0, None, None), robot=None,
            runner_mode='PD_STAND', runner_received_s=100.)
        h.current_robot=lambda now:h.robot
        return h

    def robot_sample(self, stamp):
        return dict(timestamp_s=stamp, matrices={'pelvis_link':np.eye(4)}, racket_xyz=[0., 0., 0.], velocity=None)

    def render(self, h, now):
        with patch('hope_live_scene.time.time', return_value=now):
            LiveScene.render(h)

    def test_stable_robot_is_refreshed_without_full_clear_and_tf_is_not_duplicated(self):
        h = self.harness()
        h.robot = self.robot_sample(99.98)
        self.render(h, 100.)
        self.render(h, 100.05)
        self.assertEqual(len(h.pubs['tf'].messages), 1)
        self.assertEqual(len(h.pubs['robot'].messages), 1)
        h.robot = self.robot_sample(100.4)
        self.render(h, 100.41)
        self.assertEqual(len(h.pubs['robot'].messages), 2)
        self.assertEqual(len(h.pubs['tf'].messages), 2)
        for message in h.pubs['robot'].messages:
            self.assertEqual([(m.ns,m.id,m.action) for m in message.markers], [('robot',0,Marker.ADD)])
            self.assertEqual(message.markers[0].lifetime.sec, 1)

    def test_missing_joints_clear_once_then_recovery_reuses_robot_identity(self):
        h=self.harness()
        h.robot=self.robot_sample(100.)
        self.render(h,100.)
        h.robot=None
        self.render(h,100.3)
        self.render(h,101.)
        self.assertEqual(len(h.pubs['robot'].messages),2)
        self.assertEqual([m.action for m in h.pubs['robot'].messages[-1].markers],[Marker.DELETEALL])
        h.robot=self.robot_sample(101.1)
        self.render(h,101.12)
        self.assertEqual([m.action for m in h.pubs['robot'].messages[-1].markers],[Marker.ADD])

    def test_session_change_clears_old_geometry_even_when_new_robot_already_arrived(self):
        h=self.harness()
        h.robot=self.robot_sample(100.)
        self.render(h,100.)
        h.state.session_id='two'
        h.robot_needs_clear=True
        h.robot=self.robot_sample(100.1)
        self.render(h,100.12)
        self.assertEqual([m.action for m in h.pubs['robot'].messages[-1].markers],[Marker.DELETEALL,Marker.ADD])
        self.assertFalse(h.robot_needs_clear)

    def test_ball_and_actual_ghost_updates_do_not_resend_target(self):
        h = self.harness()
        h.geometry.paddle = []
        h.geometry.outline_url = 'http://127.0.0.1/outline.STL'
        h.state.target = dict(command_seq=1, deadline_s=150., xyz=[0., -.7, .5], velocity=[1., 0., 0.], swing_sign=1.)
        self.render(h, 100.)
        self.assertEqual(len(h.pubs['target'].messages), 1)
        h.state.frozen = dict(timestamp_s=100.1, ball_xyz=[0., -.7, .5])
        self.render(h, 100.11)
        self.assertEqual(len(h.pubs['target'].messages), 1)
        self.assertEqual(len(h.pubs['frozen'].messages), 1)
        h.state.policy_frozen = dict(timestamp_s=150.)
        h.state.policy_sample = dict(matrices={'pingpang_red_Link':np.eye(4)})
        self.render(h, 100.12)
        self.assertEqual(len(h.pubs['target'].messages), 1)
        self.assertEqual(len(h.pubs['frozen'].messages), 2)
        h.state.target['command_seq'] = 2
        self.render(h, 100.13)
        self.assertEqual(len(h.pubs['target'].messages), 2)
        self.assertEqual(len(h.pubs['frozen'].messages), 2)
        h.state.begin(100.14, 'test_next_ball')
        self.render(h, 100.15)
        self.assertEqual([m.action for m in h.pubs['target'].messages[-1].markers], [Marker.DELETEALL])
        self.assertEqual([m.action for m in h.pubs['frozen'].messages[-1].markers], [Marker.DELETEALL])

    def test_display_throttle_keeps_measurement_processing_at_every_tick(self):
        h = self.harness()
        consumed = []
        h.consume = consumed.append
        for t in (100., 100.05, 100.10, 100.15, 100.21):
            h.robot = self.robot_sample(t)
            self.render(h, t)
        self.assertEqual(len(consumed), 5)
        self.assertEqual(len(h.pubs['scene'].messages), 2)
        self.assertEqual(len(h.pubs['tf'].messages), 2)

    def test_runner_status_has_independent_expiry(self):
        import json
        h=self.harness()
        with patch('hope_live_scene.time.time', return_value=100.1):
            LiveScene.telemetry(h)
        current=json.loads(h.pubs['telemetry'].messages[-1].data)
        self.assertEqual(current['runner_mode'],'PD_STAND')
        self.assertAlmostEqual(current['runner_age_ms'],100.)
        with patch('hope_live_scene.time.time', return_value=101.1):
            LiveScene.telemetry(h)
        stale=json.loads(h.pubs['telemetry'].messages[-1].data)
        self.assertIsNone(stale['runner_mode'])
        self.assertGreater(stale['runner_age_ms'],1000.)


if __name__ == '__main__':
    unittest.main()
