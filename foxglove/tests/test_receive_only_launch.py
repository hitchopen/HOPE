"""Exercise the real shell prelaunch path without launching a hardware process."""
import hashlib
import os
from pathlib import Path
import subprocess

import pytest

HELPER = Path(os.environ.get('HOPE_TEST_LIFECYCLE_HELPER', str(Path(__file__).parents[1] / 'helpers/hope-lifecycle')))

@pytest.mark.parametrize('receipt,mode,csv_exists,success', [
    ('matched', 'OPTITRACK', False, True),
    ('absent', 'OPTITRACK', True, True),
    ('absent', 'OPTITRACK', False, False),
    ('wrong', 'OPTITRACK', False, False),
    ('matched', 'KERNEL', False, False),
])
def test_policy_prelaunch_csv_dependency(tmp_path, receipt, mode, csv_exists, success):
    policy = tmp_path / 'policy/exported/policy.onnx'
    policy.parent.mkdir(parents=True)
    policy.write_bytes(b'test policy identity')
    if receipt != 'absent':
        digest = hashlib.sha256(policy.read_bytes()).hexdigest() if receipt == 'matched' else '0' * 64
        (tmp_path / 'receive_only.policy_sha256').write_text(digest + '\n')
    csv = tmp_path / 'motions/a3p_op3_serve025_v23_branch110_model14600.csv'
    if csv_exists:
        csv.parent.mkdir()
        csv.write_text('matched legacy trajectory\n')
    session = tmp_path / 'session'
    session.mkdir()
    source = HELPER.read_text().split('run_runner() {', 1)[1]
    block = source[source.index('  local -a serve_runner_args='):].split('  set +e', 1)[0]
    script = ('set -euo pipefail\n'
              'die() { echo "$*" >&2; exit 1; }\n'
              'prelaunch() {\n'
              'local session_dir="$SESSION_DIR"\n' + block +
              '\nprintf "ARG=%s\\n" "${serve_runner_args[@]}" "${field_runner_args[@]}"\n}\nprelaunch\n')
    result = subprocess.run(['bash', '-c', script], cwd=tmp_path,
                            env={**os.environ, 'MDU_DEPLOY': str(tmp_path), 'SESSION_DIR': str(session),
                                 'FIELD_MODE': mode, 'SERVE_CSV_SHA': 'DEFAULT', 'GRIPPER_PROFILE': 'test'},
                            text=True, capture_output=True)
    assert (result.returncode == 0) == success, result.stderr
    if success and receipt == 'matched':
        assert '--serve' not in result.stdout
        assert 'effective_csv=NONE' in (session / 'field_selection.txt').read_text()
    elif success:
        assert 'ARG=--serve\n' in result.stdout
        assert 'ARG=--serve-timeline\n' in result.stdout
        assert hashlib.sha256(csv.read_bytes()).hexdigest() in (session / 'artifact_sha256.txt').read_text()
