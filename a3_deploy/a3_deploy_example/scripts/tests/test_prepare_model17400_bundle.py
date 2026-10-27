import contextlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import prepare_model17400_bundle as bundle


class Model17400BundleTest(unittest.TestCase):
    def stage(self, machine):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        package, dist, out = root/'package', root/'dist', root/'out'
        policy = package/'policy/exported/policy.onnx'
        deploy = package/'policy/params/deploy.yaml'
        policy.parent.mkdir(parents=True)
        deploy.parent.mkdir(parents=True)
        policy.write_bytes(b'fixture policy')
        deploy.write_text('fixture: deploy\n')
        for relative in ('run_a3_pingpong.sh',
                         'config/a3_aimrt_config.pingpong_iceoryx.yaml',
                         'tools/a3p_gripper_bridge.py'):
            path = dist/relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('fixture\n')
        header = bytearray(20)
        header[:4], header[5] = b'\x7fELF', 1
        header[18:20] = machine.to_bytes(2, 'little')
        (dist/'a3_deploy_onnx_ref_pingpong').write_bytes(header)
        runtime = 'config/a3_runtime_config.pingpong.hitter_pingpong.yaml'
        generic = {'obs_debug': {'loc_mode': 'external_base'}, 'backend': {
            'aimrt_cfg_path': 'config/a3_aimrt_config.yaml',
            'sync_mode': 'min_skew_pair', 'sync_hz': 100.0}, 'policy_dir': 'policy'}
        (dist/runtime).write_text(yaml.safe_dump(generic))
        inspection = {'station_contract': 'fixed_session_anchor_v1',
                      'policy_abi': 'ball_clock_110_v1',
                      'position_support': [[.44, .70, -.68, -.24, .8, 1.45],
                                           [.44, .70, -.24, .28, .8, 1.45]],
                      'velocity_support': [[1., 2.9, -.8, 1.2, .3, 2.],
                                           [1.1, 2.9, -1., .9, 0., 1.8]]}
        argv = ['prepare_model17400_bundle.py', '--package', str(package),
                '--runner-dist', str(dist), '--output', str(out)]
        # Fake only the model identity and native inspection boundary. Exercise
        # actual staging, YAML selection, policy copying and manifest hashing.
        with mock.patch.object(bundle, 'POLICY_SHA', bundle.digest(policy)), \
             mock.patch.object(bundle, 'DEPLOY_SHA', bundle.digest(deploy)), \
             mock.patch.object(sys, 'argv', argv), \
             mock.patch.object(bundle.subprocess, 'run', return_value=
                               subprocess.CompletedProcess([], 0, yaml.safe_dump(inspection))), \
             contextlib.redirect_stdout(io.StringIO()):
            bundle.main()
        self.assertEqual((out/'deployment/policy/exported/policy.onnx').read_bytes(), policy.read_bytes())
        self.assertEqual((out/'deployment/policy/params/deploy.yaml').read_bytes(), deploy.read_bytes())
        self.assertEqual(yaml.safe_load((dist/runtime).read_text()), generic)
        entry = json.loads((out/'mdu_entry.json').read_text())
        self.assertTrue(entry['receive_only'])
        self.assertEqual(entry['files']['/agibot/a3_deploy_model17400/'+runtime],
                         bundle.digest(out/'deployment'/runtime))
        return yaml.safe_load((out/'deployment'/runtime).read_text()), generic, out

    def test_arm_bundle_restores_hardware_defaults_before_manifest(self):
        actual, generic, out = self.stage(183)
        self.assertEqual(actual['background_cpu'], 4)
        self.assertEqual(actual['driver_cpu'], 6)
        self.assertEqual(actual['backend'].pop('sync_cpu'), 5)
        self.assertEqual(actual.pop('serve_release_lead_frames'), 3)
        actual.pop('background_cpu')
        actual.pop('driver_cpu')
        self.assertEqual(actual, generic)
        self.assertEqual(json.loads((out/'receipt.json').read_text())['status'], 'STAGED_NOT_INSTALLED')

    def test_x86_simulation_keeps_generic_runtime(self):
        actual, generic, out = self.stage(62)
        self.assertEqual(actual, generic)
        self.assertEqual(json.loads((out/'receipt.json').read_text())['status'], 'SIMULATION_ONLY')


if __name__ == '__main__':
    unittest.main()
