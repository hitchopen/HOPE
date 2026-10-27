"""Exercise the actual shell readiness gate with controlled process stubs."""
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest


class GatewayStartTests(unittest.TestCase):
    def run_gate(self, fresh_log):
        source = (Path(__file__).parents[1] / 'helpers/hope-lifecycle').read_text()
        function = 'start_transport_gateway() {' + source.split('start_transport_gateway() {', 1)[1].split('\nrun_planner()', 1)[0]
        with tempfile.TemporaryDirectory() as directory:
            Path(directory, 'transport_gateway.log').write_text('TRANSPORT GATEWAY HEALTHY old process\n')
            function = function.replace('"/tmp/hope_real/$SESSION_ID/hdu"', shlex.quote(directory))
            script = '''set -euo pipefail
GRIPPER_PROFILE=legacy
ROBOT_USER=unused HDU_TRANSPORT_GATEWAY_SESSION=test SELF=unused SESSION_ID=test
LAPTOP_WIFI_IP=none HDU_WIFI_IP=none MDU_INTERNAL_IP=none MOTIVE_IP=none TABLE_SIDE=P1 FIELD_MODE=KERNEL SERVE_CSV_SHA=DEFAULT
started=0
require_user() { :; }
tmux_exists() { [[ "$started" == 1 ]]; }
# Match GNU stat used on the robot, including on macOS test hosts.
stat() { wc -c < "${@: -1}"; }
sleep() { :; }
stop_tmux() { started=0; }
emit() { echo "$*"; }
die() { echo "$*"; exit 1; }
start_tmux() { started=1; printf '%s\\n' ''' + shlex.quote(fresh_log) + ' >> ' + shlex.quote(str(Path(directory, 'transport_gateway.log'))) + '; }\n' + function + '\nstart_transport_gateway\n'
            return subprocess.run(['bash', '-c', script], text=True, capture_output=True)

    def test_old_healthy_record_cannot_qualify_new_attempt(self):
        result = self.run_gate('TRANSPORT GATEWAY WAITING new process')
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertIn('BIDIRECTIONAL_DDS_NOT_HEALTHY', result.stdout)
        self.assertNotIn('COMPLETE', result.stdout)

    def test_new_healthy_record_qualifies(self):
        result = self.run_gate('TRANSPORT GATEWAY HEALTHY new process')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('COMPLETE BIDIRECTIONAL_DDS_HEALTHY', result.stdout)

    def test_legacy_stop_event_only_reports_a_session_that_existed(self):
        source = (Path(__file__).parents[1] / 'helpers/hope-lifecycle').read_text()
        function = 'stop_hdu() {' + source.split('stop_hdu() {', 1)[1].split('\nstop_laptop()', 1)[0]
        for present in ('0', '1'):
            with self.subTest(legacy_present=present):
                script = '''set -euo pipefail
ROBOT_USER=unused HDU_TRANSPORT_GATEWAY_SESSION=gateway HDU_RUNNER_TRANSPORT_SESSION=legacy
HDU_PLANNER_SESSION=planner HDU_BASE_SESSION=base
require_user() { :; }
tmux_exists() { [[ "$1" == legacy && "$LEGACY_PRESENT" == 1 ]]; }
stop_tmux() { printf 'STOP %s\\n' "$1"; }
emit() { printf '%s\\n' "$*"; }
''' + 'LEGACY_PRESENT=' + present + '\n' + function + '\nstop_hdu\n'
                result = subprocess.run(['bash', '-c', script], text=True, capture_output=True)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn('TRANSPORT_GATEWAY COMPLETE TRANSPORT_GATEWAY_STOPPED', result.stdout)
                self.assertIn('PLANNER_BASE COMPLETE HDU_STOPPED', result.stdout)
                self.assertEqual('STOP legacy\n' in result.stdout, present == '1')
                self.assertEqual('RUNNER_TRANSPORT COMPLETE RUNNER_TRANSPORT_STOPPED' in result.stdout,
                                 present == '1')


if __name__ == '__main__':
    unittest.main()
