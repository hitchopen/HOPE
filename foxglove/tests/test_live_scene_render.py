"""Display lifecycle tests using real ROS marker messages, without any ROS node."""
from pathlib import Path
from types import SimpleNamespace
from collections import deque
from tempfile import TemporaryDirectory
import queue
import json
from unittest.mock import patch
import sys
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'a3'))
from visualization_msgs.msg import Marker, MarkerArray
from hope_live_scene import DisplayPoseCache, LiveScene
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
        geometry.joints = []
        return geometry

    def harness(self):
        state = LiveSceneState()
        state.session_id, state.producer = 'one', 'packetizer'
        h = SimpleNamespace(state=state, geometry=self.geometry(), scene_updates=SceneMarkerUpdates(),
            pubs={key:Sink() for key in ['robot', 'scene', 'tf', 'target', 'frozen', 'telemetry']},
            swing_metadata=None, session_changed_s=0., last_status_logged=None, consume=lambda now:None, record=lambda *args:None, last_robot_key=None,
            last_scene_publish=0., args=SimpleNamespace(scene_hz=5.),
            last_robot_publish=0., display_mode=None, live_recovery_started=99.,
            last_target_key=('one', 'packetizer', 0, None),
            last_frozen_key=('one', 'packetizer', 0, None, None), robot=None,
            runner_mode='PD_STAND', runner_received_s=100.)
        h.display_pose = DisplayPoseCache(None, [], h.geometry)
        h.current_robot=lambda now:h.robot
        h.bases=[]
        h.current_base=lambda now:LiveScene.current_base(h, now)
        return h

    def robot_sample(self, stamp):
        return dict(timestamp_s=stamp, base=np.eye(4), raw={'q':{}}, matrices={'pelvis_link':np.eye(4)}, racket_xyz=[0., 0., 0.], velocity=None)

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
            self.assertEqual([(m.ns,m.id,m.action) for m in message.markers], [('robot',0,Marker.ADD), ('robot_status',0,Marker.DELETE)])
            self.assertEqual(message.markers[0].lifetime.sec, 1)

    def test_hold_world_pose_including_visual_origin_and_recover(self):
        h = self.harness()
        h.geometry.visuals[0]['origin'][0, 3] = .125
        h.robot = self.robot_sample(100.)
        h.robot['matrices']['pelvis_link'][0, 3] = 1.
        self.render(h, 100.)
        h.robot = None
        self.render(h, 100.3)
        self.render(h, 102.)
        meshes, label = h.pubs['robot'].messages[-1].markers
        self.assertEqual((meshes.ns, meshes.id, meshes.header.frame_id), ('robot', 0, 'world'))
        self.assertAlmostEqual(meshes.pose.position.x, 1.125)
        self.assertFalse(meshes.frame_locked)
        self.assertEqual((meshes.lifetime.sec, meshes.lifetime.nanosec), (0, 0))
        self.assertEqual((label.text, label.action), ('', Marker.DELETE))
        self.assertGreater(meshes.color.b, meshes.color.r)
        self.assertNotEqual(meshes.color, h.pubs['robot'].messages[0].markers[0].color)
        self.assertEqual(len(h.pubs['tf'].messages), 1)
        with patch('hope_live_scene.time.time', return_value=102.):
            LiveScene.telemetry(h)
        status = json.loads(h.pubs['telemetry'].messages[-1].data)
        self.assertFalse(status['robot_available'])
        self.assertTrue(status['display_pose_held'])
        h.robot = self.robot_sample(102.01)
        self.render(h, 102.02)
        self.assertEqual(h.display_mode, 'recovering')
        self.assertFalse(h.pubs['robot'].messages[-1].markers[0].frame_locked)
        h.robot = None
        self.render(h, 102.12)
        h.robot = self.robot_sample(102.2)
        self.render(h, 102.2)
        h.robot = self.robot_sample(102.60)
        self.render(h, 102.60)
        self.assertEqual(h.display_mode, 'recovering')
        h.robot = self.robot_sample(102.71)
        self.render(h, 102.71)
        mesh, label_delete = h.pubs['robot'].messages[-1].markers
        self.assertTrue(mesh.frame_locked)
        self.assertEqual(mesh.color, h.pubs['robot'].messages[0].markers[0].color)
        with patch('hope_live_scene.time.time', return_value=102.71):
            LiveScene.telemetry(h)
        self.assertFalse(json.loads(h.pubs['telemetry'].messages[-1].data)['display_pose_held'])
        self.assertEqual((label_delete.ns, label_delete.action), ('robot_status', Marker.DELETE))
        self.assertEqual(len(h.pubs['tf'].messages), 5)

    def test_switch_waits_for_current_session_synced_joints_and_base(self):
        h = self.harness()
        h.messages = queue.Queue()
        h.robot_samples = deque()
        h.consume = lambda now: LiveScene.consume(h, now)
        h.current_robot = lambda now: LiveScene.current_robot(h, now)
        h.geometry.joints = [('joint', 'pelvis_link', 'pingpang_red_Link', np.eye(4),
                              np.array([0., 0., 1.]), 'revolute')]
        previous = self.robot_sample(100.)
        previous['raw']['q'] = {'joint': .6}
        h.robot_samples.append(previous)
        self.render(h, 100.)
        h.messages.put(('status', 100.05, {'session_id':'two', 'producer_instance_id':'new'}))
        def joint(session, stamp, synced=True):
            h.messages.put(('joint', stamp, dict(session_id=session, source_timestamp_s=stamp,
                            sync_complete=synced, q={'joint': .4})))
        h.bases.append((100.05, np.eye(4)))
        joint('one', 100.06)
        joint('two', 100.06, False)
        joint('two', 100.1)
        self.render(h, 100.11)
        self.assertEqual(h.display_mode, 'mocap')
        self.assertFalse(h.display_pose.reference)
        self.assertEqual(h.display_pose.q, {'joint': .6})
        self.assertEqual(len(h.robot_samples), 0)
        self.assertEqual(len(h.pubs['tf'].messages), 1)
        h.bases.append((100.12, np.eye(4)))
        joint('two', 100.13)
        self.render(h, 100.14)
        self.assertEqual(h.display_mode, 'recovering')
        for stamp in [100.24, 100.34, 100.44, 100.54, 100.65]:
            h.bases.append((stamp, np.eye(4)))
            joint('two', stamp)
            self.render(h, stamp)
        self.assertEqual(h.display_mode, 'live')
        self.assertEqual(len(h.pubs['tf'].messages), 7)
        self.assertEqual(h.pubs['robot'].messages[-1].markers[-1].action, Marker.DELETE)

    def test_body_positions_are_independent_of_optional_trace_metadata(self):
        h = self.harness()
        h.messages, h.robot_samples = queue.Queue(), deque()
        h.bases.append((100., np.eye(4)))
        h.geometry.joints = [('arm', 'pelvis_link', 'pingpang_red_Link', np.eye(4),
                              np.array([0., 1., 0.]), 'revolute')]
        sample = dict(session_id='one', source_timestamp_s=100.01, sync_complete=True)
        h.messages.put(('swing', 100.01, dict(sample, swing_sign=-1, q={'arm':1.})))
        LiveScene.consume(h, 100.02)
        self.assertFalse(h.robot_samples)
        h.messages.put(('body', 100.02, dict(sample, source='body_drive', q={'arm':.2})))
        LiveScene.consume(h, 100.03)
        self.assertEqual(h.robot_samples[-1]['raw']['q'], {'arm':.2})
        self.assertEqual(h.robot_samples[-1]['raw']['source'], 'body_drive')
        self.assertEqual(h.swing_metadata, (100.01, -1))

    def test_visible_pelvis_draws_and_moves_robot_without_ball_or_joint_history(self):
        h = self.harness()
        h.state.session_id = ''
        h.runner_mode = None
        origin = np.eye(4)
        origin[0, 3] = .4
        h.geometry.joints = [('joint', 'pelvis_link', 'hand', origin, np.array([0., 1., 0.]), 'revolute')]
        with TemporaryDirectory() as tmp:
            path = Path(tmp)/'display.json'
            h.display_pose = DisplayPoseCache(path, ['field'], h.geometry)
            h.bases.append((100., np.eye(4)))
            self.render(h, 100.01)
            self.assertEqual(h.display_mode, 'mocap')
            self.assertTrue(h.display_pose.reference)
            self.assertTrue(h.pubs['robot'].messages)
            base = np.array([[0., -1., 0., 2.], [1., 0., 0., 3.], [0., 0., 1., .5], [0.,0.,0.,1.]])
            h.bases.append((100.05, base))
            self.render(h, 100.06)
            np.testing.assert_allclose(h.display_pose.matrices['hand'], base @ origin)
            marker = h.pubs['robot'].messages[-1].markers[0]
            self.assertEqual((marker.pose.position.x, marker.pose.position.y), (2., 3.))
            self.assertAlmostEqual(abs(marker.pose.orientation.z), 2**-.5)
            with patch('hope_live_scene.time.time', return_value=100.06):
                LiveScene.telemetry(h)
            v = json.loads(h.pubs['telemetry'].messages[-1].data)
            self.assertTrue(v['display_mocap_body'] and v['display_joints_reference'])
            self.assertFalse(v['robot_available'] or v['display_pose_held'])
            self.assertFalse(h.pubs['tf'].messages)
            self.render(h, 101.)
            self.assertEqual(h.display_mode, 'held')
            restored = DisplayPoseCache(path, ['field'], h.geometry)
            self.assertTrue(restored.reference)
            np.testing.assert_allclose(restored.matrices['pelvis_link'], base)

    def test_restart_restores_display_only_and_rejects_other_identity_or_corrupt_file(self):
        h = self.harness()
        with TemporaryDirectory() as tmp:
            path = Path(tmp)/'last-pose'
            identity = ['machine-a', 'P2']
            h.display_pose = DisplayPoseCache(path, identity, h.geometry)
            self.render(h, 99.)
            self.assertEqual(h.display_mode, 'empty')
            h.robot = self.robot_sample(100.)
            h.robot['base'][1, 3] = .37
            h.robot['matrices']['pelvis_link'] = h.robot['base']
            self.render(h, 100.)
            new = self.harness()
            new.display_pose = DisplayPoseCache(path, identity, new.geometry)
            self.render(new, 104.)
            self.assertEqual(new.display_mode, 'held')
            self.assertEqual(new.pubs['robot'].messages[-1].markers[0].pose.position.y, .37)
            self.assertFalse(new.pubs['tf'].messages)
            for other in (['machine-b','P2'], ['machine-a','P1']):
                self.assertIsNone(DisplayPoseCache(path, other, new.geometry).matrices)
            path.write_text('{')
            self.assertIsNone(DisplayPoseCache(path, identity, new.geometry).matrices)

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

    def test_mocap_pose_is_published_without_runner_joints(self):
        import json
        from scipy.spatial.transform import Rotation
        h=self.harness()
        base=np.eye(4)
        base[:3, :3]=Rotation.from_euler('xyz', [0., 0., 90.], degrees=True).as_matrix()
        base[:3, 3]=[-.45, -.62, 1.0]
        h.bases.append((100.0, base))
        with patch('hope_live_scene.time.time', return_value=100.05):
            LiveScene.telemetry(h)
        current=json.loads(h.pubs['telemetry'].messages[-1].data)
        self.assertFalse(current['robot_available'])
        self.assertIsNone(current['racket_speed'])
        self.assertTrue(current['base_available'])
        self.assertAlmostEqual(current['base_age_ms'], 50., places=3)
        np.testing.assert_allclose(current['robot_xyz'], [-.45, -.62, 1.0])
        np.testing.assert_allclose(current['robot_rpy_deg'], [0., 0., 90.], atol=1e-9)
        with patch('hope_live_scene.time.time', return_value=100.3):
            LiveScene.telemetry(h)
        stale=json.loads(h.pubs['telemetry'].messages[-1].data)
        self.assertFalse(stale['base_available'])
        self.assertIsNone(stale['robot_xyz'])
        self.assertAlmostEqual(stale['last_pose_age_s'], .3)
        np.testing.assert_allclose(stale['last_robot_xyz'], current['robot_xyz'])
        np.testing.assert_allclose(stale['last_robot_rpy_deg'], current['robot_rpy_deg'])


if __name__ == '__main__':
    unittest.main()
