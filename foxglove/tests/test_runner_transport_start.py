"""Exercise the actual shell readiness gate with controlled process stubs."""
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest


class RelayStartTests(unittest.TestCase):
    def run_gate(self, fresh_log):
        source = (Path(__file__).parents[1] / 'helpers/hope-lifecycle').read_text()
        function = 'start_runner_transport() {' + source.split('start_runner_transport() {', 1)[1].split('\nrun_planner()', 1)[0]
        with tempfile.TemporaryDirectory() as directory:
            Path(directory, 'runner_transport_relay.log').write_text('RUNNER TRANSPORT HEALTHY old process\n')
            function = function.replace('"/tmp/hope_real/$SESSION_ID/hdu"', shlex.quote(directory))
            script = '''set -euo pipefail
ROBOT_USER=unused HDU_RUNNER_TRANSPORT_SESSION=test SELF=unused SESSION_ID=test
LAPTOP_WIFI_IP=none HDU_WIFI_IP=none MDU_INTERNAL_IP=none MOTIVE_IP=none TABLE_SIDE=P1 FIELD_MODE=KERNEL SERVE_CSV_SHA=DEFAULT
started=0
require_user() { :; }
tmux_exists() { [[ "$started" == 1 ]]; }
sleep() { :; }
stop_tmux() { started=0; }
emit() { echo "$*"; }
die() { echo "$*"; exit 1; }
start_tmux() { started=1; printf '%s\\n' ''' + shlex.quote(fresh_log) + ' >> ' + shlex.quote(str(Path(directory, 'runner_transport_relay.log'))) + '; }\n' + function + '\nstart_runner_transport\n'
            return subprocess.run(['bash', '-c', script], text=True, capture_output=True)

    def test_old_healthy_record_cannot_qualify_new_attempt(self):
        result = self.run_gate('RUNNER TRANSPORT WAITING new process')
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertIn('BIDIRECTIONAL_DDS_NOT_HEALTHY', result.stdout)
        self.assertNotIn('COMPLETE', result.stdout)

    def test_new_healthy_record_qualifies(self):
        result = self.run_gate('RUNNER TRANSPORT HEALTHY new process')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('COMPLETE BIDIRECTIONAL_DDS_HEALTHY', result.stdout)


if __name__ == '__main__':
    unittest.main()
