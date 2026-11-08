#!/usr/bin/env python3
"""Laptop-only, read-only live Foxglove scene. Publishes /hope/live/* only."""
from collections import deque
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import argparse
import json
import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path
import queue
import re
import signal
import subprocess
import sys
import threading
import time
import urllib.request
from urllib.parse import urlsplit

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float64MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray
from tf2_msgs.msg import TFMessage
from hope_display_publishers import display_publisher
from hope_live_scene_core import LiveSceneState
from hope_live_scene_geometry import RobotGeometry, SceneMarkerUpdates, clear, marker, point, scene_markers


class AssetHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, outline_cache=None, **kwargs):
        self.outline_cache = Path(outline_cache) if outline_cache is not None else None
        super().__init__(*args, **kwargs)

    def translate_path(self, path):
        route = urlsplit(path).path
        if route.startswith('/outline-cache/'):
            name = route[len('/outline-cache/'):]
            if self.outline_cache is not None and re.fullmatch(r'outline-[0-9a-f]{64}\.STL', name):
                return str(self.outline_cache/name)
            return str(Path(self.directory)/'.invalid-outline-resource')
        return super().translate_path(path)

    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Cache-Control', 'public, max-age=31536000, immutable' if urlsplit(self.path).path.startswith('/outline-cache/') else 'public, max-age=3600')
        super().end_headers()

    def guess_type(self, path):
        return 'model/stl' if str(path).lower().endswith('.stl') else super().guess_type(path)

    def log_message(self, *_):
        pass


class LiveScene(Node):
    def __init__(self, args):
        super().__init__('hope_live_scene')
        self.args = args
        self.state = LiveSceneState()
        self.geometry = RobotGeometry(args.root, args.mesh_port, Path(args.log_dir)/'mesh-cache')
        self.stop = threading.Event()
        self.messages = queue.Queue(maxsize=1000)
        self.bases = deque(maxlen=500)
        self.robot_samples = deque(maxlen=200)
        self.joint_process = None
        self.last_robot_key = None
        self.robot_needs_clear = False
        self.scene_updates = SceneMarkerUpdates()
        self.runner_mode = None
        self.runner_received_s = None
        self.last_target_key = None
        self.last_frozen_key = None
        self.last_scene_publish = 0.
        self.last_robot_publish = 0.
        self.last_status_logged = None
        self.robot_visible = False
        logdir = Path(args.log_dir)
        logdir.mkdir(parents=True, exist_ok=True)
        self.audit = logging.getLogger('hope.live.scene.audit')
        self.audit.setLevel(logging.INFO)
        handler = RotatingFileHandler(logdir/'events.jsonl', maxBytes=10000000, backupCount=2)
        handler.setFormatter(logging.Formatter('%(message)s'))
        self.audit.addHandler(handler)
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscriptions_kept = [
            self.create_subscription(PoseArray, '/poses/planner_full', self.ball, qos),
            self.create_subscription(String, '/hope/runner/mode', self.runner,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)),
            self.create_subscription(Float64MultiArray, '/a3/base_pose_laptop_flat', self.base, qos),
            self.create_subscription(Float64MultiArray, '/racket/command_flat', self.command, qos),
            self.create_subscription(Float64MultiArray, '/hope/planner/hit_plane', self.plane,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        ]
        self.pubs = {}
        for name, kind in [('scene', MarkerArray), ('robot', MarkerArray), ('target', MarkerArray), ('frozen', MarkerArray), ('telemetry', String), ('tf', TFMessage)]:
            persistent = name in ('robot', 'target', 'frozen')
            pubqos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL if persistent else DurabilityPolicy.VOLATILE)
            self.pubs[name] = display_publisher(self, kind, '/hope/live/'+name, pubqos)
        self.http = ThreadingHTTPServer(('127.0.0.1', args.mesh_port),
            partial(AssetHandler, directory=str(self.geometry.urdf.parent.parent), outline_cache=self.geometry.outline_cache))
        threading.Thread(target=self.http.serve_forever, daemon=True).start()
        threading.Thread(target=self.poll_status, daemon=True).start()
        threading.Thread(target=self.read_joints, daemon=True).start()
        self.create_timer(1/20, self.render)
        self.create_timer(1/args.telemetry_hz, self.telemetry)
        self.get_logger().info('Read-only live scene ready; only /hope/live/* publishers; no control clients')

    def record(self, kind, data):
        self.audit.info(json.dumps(dict(kind=kind, receipt_s=time.time(), data=data), separators=(',', ':'), allow_nan=False))

    def poll_status(self):
        while not self.stop.is_set():
            try:
                # Read-only loopback endpoint explicitly permits no-Origin GET;
                # neither a control key nor any intent is sent over DDS.
                with urllib.request.urlopen('http://127.0.0.1:8768/ball-state', timeout=.15) as response:
                    value = json.load(response)
                self.messages.put_nowait(('status', time.time(), value))
            except Exception:
                pass
            self.stop.wait(.2)

    def read_joints(self):
        helper = Path(__file__).with_name('hope_live_joint_stream.py')
        while not self.stop.is_set():
            session = self.state.session_id
            if not session:
                self.stop.wait(.2)
                continue
            try:
                self.joint_process = subprocess.Popen([sys.executable, str(helper), '--session', session],
                    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
                for line in self.joint_process.stdout:
                    if self.stop.is_set() or self.state.session_id != session:
                        break
                    try:
                        self.messages.put_nowait(('joint', time.time(), json.loads(line)))
                    except (ValueError, queue.Full):
                        pass
            finally:
                if self.joint_process and self.joint_process.poll() is None:
                    self.joint_process.terminate()
                    try:
                        self.joint_process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        self.joint_process.kill()
            self.stop.wait(2)

    def ball(self, msg):
        if not msg.poses:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        p = msg.poses[0].position
        xyz = [p.x, p.y, p.z]
        if self.state.ball(t, xyz, time.time()):
            self.record('raw_ball', dict(timestamp_s=t, xyz=xyz, frame_id=msg.header.frame_id))

    def base(self, msg):
        a = list(msg.data)
        if len(a) < 16 or a[0] != 2 or a[1] != 1 or a[12] != 1 or not np.isfinite(a).all():
            return
        t = a[3]+a[4]*1e-9
        if not (-.1 <= time.time()-t <= .3) or (self.bases and t <= self.bases[-1][0]):
            return
        q = np.array([a[9], a[10], a[11], a[8]])
        if not .8 <= np.linalg.norm(q) <= 1.2:
            return
        base = np.eye(4)
        base[:3, :3] = Rotation.from_quat(q).as_matrix()
        base[:3, 3] = [a[5], a[6], a[7]-.76]
        self.bases.append((t, base))

    def runner(self, msg):
        self.runner_mode = msg.data
        self.runner_received_s = time.time()

    def command(self, msg):
        if self.state.command(list(msg.data), time.time()):
            self.record('target', self.state.target)

    def plane(self, msg):
        self.state.set_plane(list(msg.data), time.time())

    def consume(self, now):
        for _ in range(1000):
            try:
                kind, receipt, value = self.messages.get_nowait()
            except queue.Empty:
                break
            if kind == 'status':
                previous_session = self.state.session_id, self.state.producer
                self.state.status(value, receipt)
                if previous_session != (self.state.session_id, self.state.producer):
                    self.robot_samples.clear()
                    self.last_robot_key = None
                    self.robot_needs_clear = True
                state_key = json.dumps(value, sort_keys=True)
                if state_key != self.last_status_logged:
                    self.record('play_state', value)
                    self.last_status_logged = state_key
                continue
            if value.get('session_id') != self.state.session_id or not value.get('sync_complete'):
                continue
            stamp = value.get('source_timestamp_s', 0)
            if not 0 <= now-stamp <= .35:
                continue
            if self.robot_samples and stamp <= self.robot_samples[-1]['timestamp_s']:
                continue
            bases = [b for b in self.bases if 0 <= stamp-b[0] <= .03]
            if not bases:
                continue
            base = bases[-1][1]
            try:
                matrices = self.geometry.fk(value['q'], base)
            except (KeyError, ValueError):
                continue
            sample = dict(timestamp_s=stamp, matrices=matrices, base=base, raw=value,
                          racket_xyz=matrices['pingpang_red_Link'][:3, 3].tolist(), velocity=None)
            if self.robot_samples:
                old = self.robot_samples[-1]
                dt = stamp-old['timestamp_s']
                if .005 <= dt <= .06:
                    sample['velocity'] = [(sample['racket_xyz'][i]-old['racket_xyz'][i])/dt for i in range(3)]
            self.robot_samples.append(sample)

    def current_robot(self, now):
        return self.robot_samples[-1] if self.robot_samples and 0<=now-self.robot_samples[-1]['timestamp_s']<.25 else None

    def render(self):
        now = time.time()
        self.consume(now)
        robot = self.current_robot(now)
        # At the timer's full rate, publish every tick: wall-clock jitter must
        # not turn a slightly early 50 ms callback into a skipped frame.
        publish_scene = self.args.scene_hz >= 20 or now-self.last_scene_publish >= 1/self.args.scene_hz
        if publish_scene:
            scene = scene_markers(self.state, now, robot['racket_xyz'] if robot else None, robot['velocity'] if robot else None)
            self.pubs['scene'].publish(self.scene_updates.update(scene))
            self.last_scene_publish = now
        if robot:
            robot_key = (self.state.session_id, self.state.producer, robot['timestamp_s'])
            if publish_scene and robot_key != self.last_robot_key:
                self.pubs['tf'].publish(self.geometry.transforms(robot['timestamp_s'], robot['matrices']))
                self.last_robot_key = robot_key
        refresh_robot = robot is not None and now-self.last_robot_publish >= .4
        if self.robot_needs_clear or bool(robot) != self.robot_visible or refresh_robot:
            markers = self.geometry.robot(now).markers if robot else []
            if self.robot_needs_clear or (self.robot_visible and not robot):
                markers = [clear()]+markers
            if markers:
                self.pubs['robot'].publish(MarkerArray(markers=markers))
            self.last_robot_publish, self.robot_visible = now, bool(robot)
            self.robot_needs_clear = False
        target = self.state.target
        # Trace writes arrive after their acquisition time. Give those samples
        # time to arrive, then choose the last actual measurement BEFORE strike.
        if target and self.state.policy_frozen is None and .18 <= now-target['deadline_s'] <= .6:
            candidates = [r for r in self.robot_samples if 0 <= target['deadline_s']-r['timestamp_s'] <= .03]
            if candidates:
                r = candidates[-1]
                self.state.freeze_policy(r, r['racket_xyz'], target['deadline_s'])
                self.record('policy_frozen', self.state.policy_frozen)
        identity = (self.state.session_id, self.state.producer, self.state.generation)
        target_key = (*identity, target['command_seq'] if target else None)
        frozen_key = (*identity,
            self.state.frozen['timestamp_s'] if self.state.frozen else None,
            self.state.policy_frozen['timestamp_s'] if self.state.policy_frozen else None)
        target_changed = target_key != self.last_target_key
        frozen_changed = frozen_key != self.last_frozen_key
        if target_changed:
            tm = [clear()]
            if target:
                matrix = self.geometry.target_matrix(target)
                if matrix is not None:
                    tm += self.geometry.paddle_markers(now, matrix, 'target')
            self.pubs['target'].publish(MarkerArray(markers=tm))
            self.last_target_key = target_key
        if frozen_changed:
            fm = [clear()]
            if self.state.policy_sample:
                fm += self.geometry.paddle_markers(now, self.state.policy_sample['matrices']['pingpang_red_Link'], 'actual_frozen')
            if self.state.frozen:
                f = self.state.frozen
                m = marker('frozen_ball', 0, Marker.SPHERE, now, f['ball_xyz'], (.065, .065, .065), (1., .8, .08, 1.))
                m.lifetime.nanosec = 0
                fm.append(m)
                self.record('ball_frozen', f)
            self.pubs['frozen'].publish(MarkerArray(markers=fm))
            self.last_frozen_key = frozen_key
        if target_changed or frozen_changed:
            self.record('shot_snapshot', dict(generation=self.state.generation, frozen=self.state.frozen, policy_frozen=self.state.policy_frozen))

    def telemetry(self):
        now = time.time()
        snapshot = self.state.snapshot(now)
        runner_age = now-self.runner_received_s if self.runner_received_s is not None else None
        runner_fresh = runner_age is not None and 0 <= runner_age < 1.
        snapshot.update(runner_mode=self.runner_mode if runner_fresh else None,
            runner_age_ms=runner_age*1000 if runner_age is not None and runner_age >= 0 else None)
        r = self.current_robot(now)
        if r:
            snapshot.update(robot_available=True, robot_age_ms=(now-r['timestamp_s'])*1000,
                robot_xyz=r['base'][:3, 3].tolist(), robot_rpy_deg=Rotation.from_matrix(r['base'][:3, :3]).as_euler('xyz', degrees=True).tolist(),
                racket_xyz=r['racket_xyz'], racket_speed=float(np.linalg.norm(r['velocity'])) if r['velocity'] is not None else None,
                policy_swing_sign=r['raw'].get('swing_sign'))
        self.pubs['telemetry'].publish(String(data=json.dumps(snapshot, separators=(',', ':'), allow_nan=False)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', default=str(Path(__file__).resolve().parents[2]))
    parser.add_argument('--mesh-port', type=int, default=8769)
    parser.add_argument('--log-dir', required=True)
    parser.add_argument('--scene-hz', type=float, default=20.0)
    parser.add_argument('--telemetry-hz', type=float, default=5.0)
    args = parser.parse_args()
    if not 1 <= args.scene_hz <= 20 or not 1 <= args.telemetry_hz <= 10:
        parser.error('scene-hz must be 1..20 and telemetry-hz must be 1..10')
    def interrupted(_signal, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    rclpy.init()
    node = LiveScene(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop.set()
        if node.joint_process and node.joint_process.poll() is None:
            node.joint_process.terminate()
        node.http.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
