"""Outline resources preserve CAD edges while avoiding per-shot DDS point arrays."""
from pathlib import Path
from tempfile import TemporaryDirectory
from functools import partial
from http.server import ThreadingHTTPServer
from threading import Thread
import sys
import struct
import unittest
import urllib.request
import urllib.error
import hashlib
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'a3'))
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.serialization import serialize_message
from hope_live_scene import AssetHandler
from hope_live_scene_geometry import RobotGeometry, outline_stl


class OutlineTests(unittest.TestCase):
    def test_tube_retains_edge_endpoints_and_nominal_diameter(self):
        blob = outline_stl([[0., 0., -.1], [0., 0., .1]])
        count = struct.unpack_from('<I', blob, 80)[0]
        self.assertEqual(count, 16)
        triangles = np.frombuffer(blob, dtype=np.dtype([('n','<f4',(3,)),('v','<f4',(3,3)),('a','<u2')]),offset=84,count=count)
        p = triangles['v'].reshape(-1, 3)
        np.testing.assert_allclose(np.linalg.norm(p[:, :2], axis=1), .00175, atol=1e-9)
        self.assertAlmostEqual(p[:, 2].min(), -.1)
        self.assertAlmostEqual(p[:, 2].max(), .1)
        self.assertTrue(np.isfinite(triangles['n']).all())
        self.assertEqual(blob, outline_stl([[0., 0., -.1], [0., 0., .1]]))

    def test_actual_assets_use_one_cached_outline_with_distinct_colors(self):
        root = Path(__file__).resolve().parents[2]
        with TemporaryDirectory() as tmp:
            geometry = RobotGeometry(root, cache_dir=tmp)
            target = geometry.paddle_markers(100., np.eye(4), 'target')
            actual = geometry.paddle_markers(100., np.eye(4), 'actual_frozen')
            self.assertEqual(len(target), 4)
            self.assertTrue(all(m.type == Marker.MESH_RESOURCE and not m.points for m in target))
            self.assertEqual(target[-1].mesh_resource, actual[-1].mesh_resource)
            self.assertGreater(target[-1].color.b, target[-1].color.r)
            self.assertGreater(actual[-1].color.r, actual[-1].color.b)
            self.assertEqual(target[1].color, actual[1].color)
            self.assertEqual(target[2].color, actual[2].color)
            self.assertLess(len(serialize_message(MarkerArray(markers=target))), 2500)
            path = next(Path(tmp).glob('*.STL'))
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            self.assertEqual(path.name, 'outline-'+digest+'.STL')
            count = struct.unpack_from('<I', path.read_bytes(), 80)[0]
            expected_edges = sum(len(outline)//2 for _, _, outline in geometry.paddle)
            self.assertEqual(count, expected_edges*16)

    def test_hash_resource_route_and_cache_headers(self):
        with TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root/'static').mkdir()
            cache = root/'cache'
            cache.mkdir()
            blob = outline_stl([[0., 0., 0.], [1., 0., 0.]])
            name = 'outline-'+hashlib.sha256(blob).hexdigest()+'.STL'
            (cache/name).write_bytes(blob)
            server = ThreadingHTTPServer(('127.0.0.1', 0), partial(AssetHandler, directory=str(root/'static'), outline_cache=cache))
            thread = Thread(target=server.serve_forever, daemon=True)
            thread.start()
            try:
                base = f'http://127.0.0.1:{server.server_port}/outline-cache/'
                with urllib.request.urlopen(base+name, timeout=2) as response:
                    self.assertEqual(response.read(), blob)
                    self.assertEqual(response.headers['Content-Type'], 'model/stl')
                    self.assertIn('immutable', response.headers['Cache-Control'])
                    self.assertEqual(response.headers['Access-Control-Allow-Origin'], '*')
                with self.assertRaises(urllib.error.HTTPError) as failure:
                    urllib.request.urlopen(base+'../secret.STL', timeout=2)
                self.assertEqual(failure.exception.code, 404)
            finally:
                server.shutdown()
                server.server_close()
                thread.join(timeout=2)


if __name__ == '__main__':
    unittest.main()
