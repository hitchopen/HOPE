"""Check the SSH Laptop action protocol without starting a real helper."""
from contextlib import redirect_stderr
from io import StringIO
import os
from pathlib import Path
import runpy
import shlex
import unittest
from unittest.mock import patch


DISPATCH = Path(__file__).resolve().parents[2] / 'scripts/macos/operator-dispatch.py'
HELPER = '/home/dongc1/.local/libexec/hope-lifecycle'


class Dispatched(Exception):
    pass


class MacOperatorDispatchTests(unittest.TestCase):
    def dispatch(self, command, expected):
        with patch.dict(os.environ, {'SSH_ORIGINAL_COMMAND': command}), \
             patch('os.execv', side_effect=Dispatched) as execute, \
             redirect_stderr(StringIO()):
            if expected is not None:
                with self.assertRaises(Dispatched):
                    runpy.run_path(str(DISPATCH))
                execute.assert_called_once_with(HELPER, expected)
            else:
                with self.assertRaises(SystemExit) as result:
                    runpy.run_path(str(DISPATCH))
                self.assertEqual(result.exception.code, 64)
                execute.assert_not_called()

    def test_supervisor_collect_request_reaches_laptop_helper(self):
        args = [HELPER, 'collect', 'model21800_20261018T050406Z',
                '10.44.226.154', '10.44.84.104', '10.42.10.12', 'NONE', 'P1']
        self.dispatch(shlex.join(args), args)

    def test_laptop_actions_used_by_supervisor_are_forwarded(self):
        for action in ('preflight-laptop', 'prepare-laptop', 'start-laptop',
                       'stop-laptop', 'stop-capture-laptop',
                       'field-assets-laptop', 'record-control'):
            with self.subTest(action=action):
                args = [HELPER, action]
                self.dispatch(shlex.join(args), args)

    def test_other_programs_and_robot_actions_are_not_dispatched(self):
        for command in ('/bin/bash -c true', HELPER+' start-runner',
                        HELPER+' collect-logs'):
            with self.subTest(command=command):
                self.dispatch(command, None)

    def test_malformed_ssh_command_is_not_dispatched(self):
        self.dispatch(HELPER+" collect '", None)


if __name__ == '__main__':
    unittest.main()
