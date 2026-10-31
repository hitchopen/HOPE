"""Exercise the real helper/asset CLI without a robot or legacy deployment."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


class LifecycleAssetBundleTest(unittest.TestCase):
    def test_gripper_wire_mismatch_rejects_without_launching_wrapper(self):
        source = (Path(__file__).resolve().parents[1] / 'helpers/hope-lifecycle').read_text()
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / 'tools').mkdir()
            helper = root / 'helper'
            helper.write_text(source.replace('/opt/ros/jazzy/setup.bash', '/dev/null'))
            bridge = root / 'tools/a3p_gripper_bridge.py'
            bridge.write_text('print("matching-wire")\n')
            native = root / 'a3_deploy_onnx_ref_pingpong'
            native.write_text('#!/bin/bash\n'
                              '[[ "$1" == --inspect-serve-release-wire ]] || exit 9\n'
                              'echo matching-wire\n')
            native.chmod(0o755)
            def check():
                return subprocess.run(['bash', '-c',
                    'source "$1"; MDU_DEPLOY="$2"; GRIPPER_PROFILE=new_20261015; check_gripper_wire_contract',
                    'bash', str(helper), str(root)],
                    env={**os.environ, 'XDG_CONFIG_HOME': str(root / 'config')},
                    capture_output=True, timeout=5).returncode
            self.assertEqual(check(), 0)
            bridge.write_text('print("other-wire")\n')
            self.assertNotEqual(check(), 0)
            bridge.write_text('raise SystemExit(1)\n')
            self.assertNotEqual(check(), 0)

    @unittest.skipIf(os.geteuid() == 0, 'writability requires an unprivileged account')
    def test_root_owned_bundle_requires_writable_log_directory(self):
        helper = Path(__file__).resolve().parents[1] / 'helpers/hope-lifecycle'
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            bundle = root / 'bundle'
            bundle.mkdir(mode=0o555)
            def check():
                return subprocess.run(['bash', '-c',
                    'source "$1"; MDU_DEPLOY="$2"; check_runner_log_directory',
                    'bash', str(helper), str(bundle)],
                    env={**os.environ, 'XDG_CONFIG_HOME': str(root / 'config')},
                    capture_output=True, timeout=5).returncode
            self.assertNotEqual(check(), 0)
            bundle.chmod(0o755)
            (bundle / 'logs').mkdir(mode=0o750)
            bundle.chmod(0o555)
            try:
                self.assertEqual(check(), 0)
                (bundle / 'logs').chmod(0o550)
                self.assertNotEqual(check(), 0)
            finally:
                bundle.chmod(0o755)
                (bundle / 'logs').chmod(0o750)

    def test_selected_bundle_validator_and_corrupt_catalog(self):
        foxglove = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            catalog = root / 'models'
            assets = root / 'assets'
            bundle = root / 'selected-bundle'
            for directory in (catalog, assets, bundle):
                directory.mkdir()
            content = b'fixture CSV\n'
            digest = hashlib.sha256(content).hexdigest()
            (assets / (digest + '.csv')).write_bytes(content)
            validator = bundle / 'a3_deploy_onnx_ref_pingpong'
            validator.write_text('#!/bin/bash\n'
                                 '[[ "$1" == --validate-serve-timeline ]] || exit 9\n'
                                 f'[[ "$LD_LIBRARY_PATH" == "{bundle}:{bundle}/lib"* ]] || exit 10\n'
                                 'echo SERVE_TIMELINE_VALID\n')
            validator.chmod(0o755)
            entry = {'deployment': str(bundle), 'files': {
                str(validator): hashlib.sha256(validator.read_bytes()).hexdigest()}}
            (catalog / 'catalog.json').write_text(json.dumps({
                'version': 1, 'models': {name: entry for name in ('model14600', 'model11999', 'model12600')}}))
            (catalog / 'selected.json').write_text('{"model":"model14600"}')
            source = (foxglove / 'helpers/hope-lifecycle').read_text()
            for original, replacement in {
                '/agibot/hope-policy-models': str(catalog),
                '/agibot/hope-field-assets': str(assets),
                '/agibot/a3_deploy_build4_serve025': str(root / 'absent-legacy'),
                '/usr/local/lib/hope-foxglove/': str(foxglove / 'a3') + '/',
                '/agibot/software/v0/entry/env/env.sh': '/dev/null',
                '/opt/ros/jazzy/setup.bash': '/dev/null',
            }.items():
                source = source.replace(original, replacement)
            helper = root / 'helper'
            helper.write_text(source)
            env = {**os.environ, 'HOPE_ROBOT_USER': subprocess.check_output(
                ['id', '-un'], text=True).strip(), 'XDG_CONFIG_HOME': str(root / 'config')}
            env.pop('HOPE_MDU_DEPLOY', None)
            def verify():
                return subprocess.run(['bash', str(helper), 'field-assets-mdu'],
                    input=json.dumps({'op': 'verify_csv', 'sha256': digest}),
                    capture_output=True, text=True, env=env, timeout=10)
            result = verify()
            self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
            self.assertEqual(json.loads(result.stdout)['sha256'], digest)
            validator.write_text('changed')
            result = verify()
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('POLICY_BUNDLE_INVALID', result.stdout)
            self.assertIn('policy bundle changed', result.stderr)


if __name__ == '__main__':
    unittest.main()
