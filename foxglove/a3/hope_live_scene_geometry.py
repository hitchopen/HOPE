"""URDF geometry shared by the read-only live scene; meshes stay on laptop HTTP."""
from pathlib import Path
import math
import hashlib
import os
import struct
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Point, TransformStamped
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray


def point(p):
    return Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))


def marker(ns, ident, typ, now, pos=(0, 0, 0), scale=(1, 1, 1), color=(1, 1, 1, 1)):
    m = Marker()
    m.header.frame_id = 'world'
    n = round(now*1e9)
    m.header.stamp.sec, m.header.stamp.nanosec = divmod(n, 10**9)
    m.ns, m.id, m.type = ns, ident, typ
    m.pose.position = point(pos)
    m.pose.orientation.w = 1.
    m.scale.x, m.scale.y, m.scale.z = map(float, scale)
    m.color.r, m.color.g, m.color.b, m.color.a = map(float, color)
    m.lifetime.nanosec = 500000000
    return m


def clear():
    m = Marker()
    m.action = Marker.DELETEALL
    return m


class SceneMarkerUpdates:
    """Refresh live markers and remove only objects absent from this snapshot.

    Every packet includes the current ADD set so a reconnect receives a complete
    view; stable IDs let the renderer update existing objects in place.
    """
    def __init__(self):
        self.keys = set()

    def update(self, snapshot):
        current = {(m.ns, m.id) for m in snapshot.markers}
        removed = []
        for ns, ident in sorted(self.keys-current):
            m = Marker()
            m.ns, m.id, m.action = ns, ident, Marker.DELETE
            removed.append(m)
        self.keys = current
        return MarkerArray(markers=removed+snapshot.markers)


def pose(m, a):
    m.pose.position = point(a[:3, 3])
    q = Rotation.from_matrix(a[:3, :3]).as_quat()
    m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = map(float, q)


def origin(e):
    a = np.eye(4)
    if e is not None:
        a[:3, 3] = [float(x) for x in e.get('xyz', '0 0 0').split()]
        a[:3, :3] = Rotation.from_euler('xyz', [float(x) for x in e.get('rpy', '0 0 0').split()]).as_matrix()
    return a


def outline_stl(points, width=.0035, sides=8):
    """One cached mesh for the original edge segments, with a 3.5 mm diameter.

    Open eight-sided tubes retain every original segment and endpoint. The
    nominal diameter matches the old world-space line width; tubes need no
    per-camera billboarding and remain visible from both paddle faces.
    """
    edges = np.asarray(points, dtype=float).reshape(-1, 2, 3)
    delta = edges[:, 1]-edges[:, 0]
    lengths = np.linalg.norm(delta, axis=1)
    edges, delta, lengths = edges[lengths > 1e-12], delta[lengths > 1e-12], lengths[lengths > 1e-12]
    if not len(edges):
        return b'HOPE outline'.ljust(80, b'\0')+struct.pack('<I', 0)
    direction = delta/lengths[:, None]
    reference = np.eye(3)[np.argmin(np.abs(direction), axis=1)]
    u = np.cross(direction, reference)
    u /= np.linalg.norm(u, axis=1)[:, None]
    v = np.cross(direction, u)
    angle = np.arange(sides)*2*np.pi/sides
    radius = width*.5
    offset = radius*(np.cos(angle)[None, :, None]*u[:, None, :]+np.sin(angle)[None, :, None]*v[:, None, :])
    a, b = edges[:, 0, None, :]+offset, edges[:, 1, None, :]+offset
    following = np.roll(np.arange(sides), -1)
    faces = np.stack((np.stack((a, a[:, following], b), axis=2),
                      np.stack((a[:, following], b[:, following], b), axis=2)), axis=2).reshape(-1, 3, 3)
    normals = np.cross(faces[:, 1]-faces[:, 0], faces[:, 2]-faces[:, 0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1)[:, None], 1e-20)
    data = np.zeros(len(faces), dtype=np.dtype([('n', '<f4', (3,)), ('v', '<f4', (3, 3)), ('a', '<u2')]))
    data['n'], data['v'] = normals, faces
    return b'HOPE outline diameter 3.5mm; original CAD edges'.ljust(80, b'\0')+struct.pack('<I', len(faces))+data.tobytes()


class RobotGeometry:
    def __init__(self, root, port=8769, cache_dir=None):
        self.urdf = Path(os.environ.get('HOPE_LIVE_SCENE_URDF', Path(root)/'hope_training/whole_body_tracking/source/whole_body_tracking/whole_body_tracking/assets/agibot_a3/urdf/model.urdf'))
        tree = ET.parse(self.urdf).getroot()
        self.joints = []
        for j in tree.findall('joint'):
            axis = j.find('axis')
            self.joints.append((j.get('name'), j.find('parent').get('link'), j.find('child').get('link'),
                origin(j.find('origin')), np.array([float(v) for v in axis.get('xyz').split()]) if axis is not None else None, j.get('type')))
        self.visuals = []
        for link in tree.findall('link'):
            for visual in link.findall('visual'):
                mesh = visual.find('geometry/mesh')
                if mesh is None:
                    continue
                path = (self.urdf.parent/mesh.get('filename')).resolve()
                relative = path.relative_to(self.urdf.parent.parent)
                color = visual.find('material/color')
                rgba = [float(v) for v in color.get('rgba').split()] if color is not None else [.7, .7, .75, 1.]
                self.visuals.append(dict(link=link.get('name'), origin=origin(visual.find('origin')),
                    path=path, url=f'http://127.0.0.1:{port}/{relative}', color=rgba))
        q = {j[0]: 0. for j in self.joints if j[5] in ('continuous', 'revolute')}
        reference = self.fk(q, np.eye(4))
        self.paddle = []
        for v in self.visuals:
            if v['link'] not in ('right_hand_pingpang_Link', 'pingpang_red_Link', 'pingpang_black_Link'):
                continue
            rel = np.linalg.inv(reference['pingpang_red_Link']) @ reference[v['link']] @ v['origin']
            data = v['path'].read_bytes()
            count = struct.unpack_from('<I', data, 80)[0]
            faces = np.frombuffer(data, dtype=np.dtype([('n', '<f4', (3,)), ('v', '<f4', (3, 3)), ('a', '<u2')]), offset=84, count=count)['v'].astype(float)
            normals = np.cross(faces[:, 1]-faces[:, 0], faces[:, 2]-faces[:, 0])
            normals /= np.maximum(np.linalg.norm(normals, axis=1)[:, None], 1e-12)
            edges = {}
            for index, face in enumerate(faces):
                for x, y in ((0, 1), (1, 2), (2, 0)):
                    key = tuple(sorted((tuple(np.round(face[x], 6)), tuple(np.round(face[y], 6)))))
                    edges.setdefault(key, []).append((face[x], face[y], normals[index]))
            outline = []
            for entries in edges.values():
                if len(entries) == 1 or any(float(entries[0][2]@entry[2]) < .94 for entry in entries[1:]):
                    normal = sum((entry[2] for entry in entries), np.zeros(3))
                    normal /= max(np.linalg.norm(normal), 1e-12)
                    outline.extend((entries[0][0]+normal*.0008, entries[0][1]+normal*.0008))
            self.paddle.append((v, rel, outline))

        # Combine all edge meshes in paddle coordinates: one draw call, shared
        # geometry for target/actual, and no outline points carried over DDS.
        self.outline_cache = Path(cache_dir) if cache_dir is not None else Path(os.environ.get('XDG_CACHE_HOME', Path.home()/'.cache'))/'hope-foxglove'/'outlines'
        points = np.concatenate([np.asarray(outline)@rel[:3, :3].T+rel[:3, 3] for _, rel, outline in self.paddle])
        content = outline_stl(points)
        name = 'outline-'+hashlib.sha256(content).hexdigest()+'.STL'
        self.outline_cache.mkdir(parents=True, exist_ok=True)
        path = self.outline_cache/name
        if not path.exists():
            temporary = path.with_suffix('.tmp')
            temporary.write_bytes(content)
            temporary.replace(path)
        self.outline_url = f'http://127.0.0.1:{port}/outline-cache/{name}'

    def fk(self, q, base):
        out = {'pelvis_link': base}
        todo = self.joints[:]
        while todo:
            old = len(todo)
            for j in todo[:]:
                name, parent, child, o, axis, typ = j
                if parent not in out:
                    continue
                a = o.copy()
                if typ in ('revolute', 'continuous'):
                    a[:3, :3] = a[:3, :3] @ Rotation.from_rotvec(axis*q[name]).as_matrix()
                out[child] = out[parent] @ a
                todo.remove(j)
            if len(todo) == old:
                raise ValueError('disconnected URDF')
        return out

    def robot(self, now):
        # Stable IDs refresh the TTL without disposing/reloading every mesh.
        ms = []
        for i, v in enumerate(self.visuals):
            m = marker('robot', i, Marker.MESH_RESOURCE, now, color=v['color'])
            m.header.frame_id = 'hope_live_robot/'+v['link']
            m.header.stamp.sec = m.header.stamp.nanosec = 0
            m.frame_locked = True
            m.mesh_resource = v['url']
            pose(m, v['origin'])
            m.lifetime.sec, m.lifetime.nanosec = 1, 0
            ms.append(m)
        return MarkerArray(markers=ms)

    def transforms(self, now, matrices):
        result = []
        for name, a in matrices.items():
            t = TransformStamped()
            t.header.frame_id, t.child_frame_id = 'world', 'hope_live_robot/'+name
            t.header.stamp.sec, t.header.stamp.nanosec = divmod(round(now*1e9), 10**9)
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, a[:3, 3])
            q = Rotation.from_matrix(a[:3, :3]).as_quat()
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = map(float, q)
            result.append(t)
        return TFMessage(transforms=result)

    def paddle_markers(self, now, matrix, kind):
        identity = (.05, .42, 1., 1.) if kind == 'target' else (1., .12, .03, 1.)
        result = []
        for i, (v, rel, outline) in enumerate(self.paddle):
            a = matrix @ rel
            m = marker(kind, i, Marker.MESH_RESOURCE, now, color=(*identity[:3], .45) if i == 0 else v['color'])
            m.mesh_resource = v['url']
            m.lifetime.nanosec = 0
            pose(m, a)
            result.append(m)
        edge = marker(kind+'_outline', 0, Marker.MESH_RESOURCE, now, color=identity)
        pose(edge, matrix)
        edge.mesh_resource = self.outline_url
        edge.lifetime.nanosec = 0
        result.append(edge)
        return result

    def target_matrix(self, target):
        # Wire normal is not serialized; Planner constructs normal parallel to
        # target velocity. Fore/back follows local +Y/-Y; roll is illustrative.
        y = np.array(target['velocity'], dtype=float)*target['swing_sign']
        if np.linalg.norm(y) < 1e-8:
            return None
        y /= np.linalg.norm(y)
        x = np.cross(y, [0., 0., 1.])
        if np.linalg.norm(x) < 1e-8:
            x = np.cross(y, [1., 0., 0.])
        x /= np.linalg.norm(x)
        a = np.eye(4)
        a[:3, :3] = np.column_stack([x, y, np.cross(x, y)])
        a[:3, 3] = target['xyz']
        return a


def scene_markers(state, now, racket=None, velocity=None):
    ms = [marker('table', 0, Marker.CUBE, now, (1.37, -.7625, -.018), (2.74, 1.525, .036), (.08, .18, .32, 1)),
          marker('net', 0, Marker.CUBE, now, (1.37, -.7625, .07625), (.01, 1.825, .1525), (.3, .36, .4, .7))]
    plane = state.target['xyz'][0] if state.target else state.plane_x
    if plane is not None:
        ms.append(marker('plane', 0, Marker.CUBE, now, (plane, -.7625, .55), (.006, 1.525, 1.1), (.55, .25, 1., .10)))
    if state.previous and 0 <= now-state.previous[0] < .1:
        bp = state.previous[1]
        ms.append(marker('ball', 0, Marker.SPHERE, now, bp, (.04, .04, .04), (1., .42, .08, 1.)))
        # Spin estimator/display changes are deliberately deferred. Unknown is
        # not zero; the requested signed axes remain visible and unambiguous.
        text = marker('spin', 0, Marker.TEXT_VIEW_FACING, now, (bp[0]+.17, bp[1]+.08, bp[2]+.23), (0, 0, .06), (.38, .40, .44, 1.))
        text.text = '↕ ? rev/s\n↔ ? rev/s'
        ms.append(text)
    segments, segment, previous = [], [], None
    for stamp, pos in state.trail:
        if stamp < now-1.8:
            continue
        if previous and (stamp-previous[0] > .020001 or math.dist(pos, previous[1])/(stamp-previous[0]) > 35):
            if len(segment) > 1:
                segments.append(segment)
            segment = []
        segment.append(point(pos))
        previous = stamp, pos
    if len(segment) > 1:
        segments.append(segment)
    for i, points in enumerate(segments):
        m = marker('trail', i, Marker.LINE_STRIP, now, scale=(.008, 0, 0), color=(1., .42, .08, .7))
        m.points = points
        ms.append(m)
    for ident, pos, vector, color in ((0, racket, velocity, (.1, .85, .45, .85)),
        (1, state.target['xyz'] if state.target else None, state.target['velocity'] if state.target else None, (.05, .6, 1., 1.))):
        if pos is not None and vector is not None:
            m = marker('velocity', ident, Marker.ARROW, now, scale=(.012, .028, .04), color=color)
            m.points = [point(pos), point([pos[i]+vector[i]*.08 for i in range(3)])]
            ms.append(m)
    return MarkerArray(markers=ms)
