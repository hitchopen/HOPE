"""Exercise the build/receipt boundary with an inert colcon fixture."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / 'a3/build_foxglove_bridge.sh'


class BridgeBuildReceiptTests(unittest.TestCase):
    def run_fixture(self, build_exit, missing_binary=False):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        source, rosx, scripts = root / 'sdk', root / 'rosx', root / 'scripts'
        receipt = root / 'foxglove-build.sha256'
        receipt.write_text('old receipt\n')
        files = [
            source / 'ros/src/foxglove_bridge/src/ros2_foxglove_bridge.cpp',
            source / 'ros/src/foxglove_bridge/include/foxglove_bridge/ros2_foxglove_bridge.hpp',
            source / 'compat.cpp', rosx / 'compat.cpp',
            scripts / 'build_foxglove_bridge.sh', scripts / 'apply_foxglove_display_patch.py',
            scripts / 'foxglove.patch', scripts / 'rosx.patch',
            scripts / 'patches/foxglove_bridge-display-priority.patch',
            source / 'ros/build/rosx_introspection/CMakeCache.txt',
            source / 'ros/build/foxglove_bridge/CMakeCache.txt',
            source / 'ros/install/foxglove_bridge/lib/foxglove_bridge/foxglove_bridge',
        ]
        for path in files:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('fixture bytes\n')
        if not missing_binary:
            files[-1].chmod(0o755)
        # Execute the actual final build block, omitting source/network setup.
        tail = 'cd "${FOXGLOVE_SOURCE}/ros"' + SCRIPT.read_text().split('cd "${FOXGLOVE_SOURCE}/ros"', 1)[1]
        env = {**os.environ, 'FOXGLOVE_SOURCE': str(source), 'ROSX_SOURCE': str(rosx),
               'SCRIPT_DIR': str(scripts), 'FOXGLOVE_BUILD_RECEIPT': str(receipt),
               'FOXGLOVE_PATCH': str(scripts / 'foxglove.patch'), 'ROSX_PATCH': str(scripts / 'rosx.patch'),
               'FOXGLOVE_PATCHED_FILE': 'compat.cpp', 'ROSX_PATCHED_FILE': 'compat.cpp',
               'FOXGLOVE_ROS_RELEASE': 'fixture', 'FIXTURE_BUILD_EXIT': str(build_exit)}
        result = subprocess.run(['bash', '-c', 'set -Eeuo pipefail\ncmake_args=(fixture)\n'
                                 'colcon() { return "$FIXTURE_BUILD_EXIT"; }\n'
                                 'die() { echo "$*" >&2; exit 1; }\n' + tail],
                                env=env, capture_output=True, text=True, timeout=5)
        return root, source, scripts, receipt, result

    def test_failed_build_preserves_backup_but_has_no_current_success_receipt(self):
        root, _, _, receipt, result = self.run_fixture(19)
        self.assertEqual(result.returncode, 19)
        self.assertFalse(receipt.exists())
        self.assertEqual([p.read_text() for p in root.glob('*.previous.*')], ['old receipt\n'])

    def test_success_receipt_binds_build_recipe_and_installed_files(self):
        _, source, scripts, receipt, result = self.run_fixture(0)
        self.assertEqual(result.returncode, 0, result.stderr)
        def verify():
            return subprocess.run(['sha256sum', '--check', str(receipt)], cwd=source / 'ros',
                                  capture_output=True).returncode
        self.assertEqual(verify(), 0)
        (scripts / 'build_foxglove_bridge.sh').write_text('different recipe\n')
        self.assertNotEqual(verify(), 0)

    def test_configure_only_success_cannot_write_a_receipt(self):
        _, _, _, receipt, result = self.run_fixture(0, missing_binary=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(receipt.exists())


if __name__ == '__main__':
    unittest.main()
