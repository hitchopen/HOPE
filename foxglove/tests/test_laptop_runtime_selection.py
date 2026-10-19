"""Exercise capture launch arguments for Linux and the Mac operator container."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


HELPER = Path(__file__).resolve().parents[1] / 'helpers/hope-lifecycle'


class LaptopRuntimeSelectionTests(unittest.TestCase):
    def capture_command(self, runtime):
        with tempfile.TemporaryDirectory(prefix='hope runtime ') as temporary:
            root = Path(temporary)
            env = dict(os.environ, HOPE_ROOT=str(root), XDG_CONFIG_HOME=str(root/'config'))
            env.pop('HOPE_LAPTOP_RUNTIME', None)
            if runtime is not None:
                env['HOPE_LAPTOP_RUNTIME'] = runtime
            result = subprocess.run(['bash', '-c', r'''
set -euo pipefail
source "$1"
SESSION_ID=model21800_20000101T000001Z
HDU_WIFI_IP=10.44.84.104
tmux_exists() { return 1; }
start_tmux() { printf '%s\0' "$@"; }
start_laptop_capture
''', 'runtime-test', str(HELPER)], env=env, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr.decode())
            argv = result.stdout.decode().rstrip('\0').split('\0')
            expected_tail = ['/bin/bash', str(root/'foxglove/helpers/hope-laptop-sim2real-record'),
                             str(root), 'model21800_20000101T000001Z', '10.44.84.104',
                             str(root/'real_logs/model21800_20000101T000001Z/laptop/capture_attempt_001')]
            self.assertEqual(argv[0], 'hope-capture-model21800_20000101T000001Z')
            self.assertEqual(argv[-len(expected_tail):], expected_tail)
            return argv[1:-len(expected_tail)]

    def test_linux_default_uses_existing_distrobox(self):
        self.assertEqual(self.capture_command(None), ['/usr/bin/distrobox', 'enter', 'hope', '--'])

    def test_linux_explicit_distrobox_uses_existing_distrobox(self):
        self.assertEqual(self.capture_command('distrobox'), ['/usr/bin/distrobox', 'enter', 'hope', '--'])

    def test_mac_container_runs_directly_in_its_ros_environment(self):
        self.assertEqual(self.capture_command('direct'), [])


if __name__ == '__main__':
    unittest.main()
