from pathlib import Path
import os
import subprocess
import sys
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'a3'))
from hope_mocap_connection import require_receiver_owner

ROOT = Path(__file__).resolve().parents[2]

@pytest.mark.parametrize('content', ['', 'HOPE_LIFECYCLE_HDU_IP=192.168.8.181\n', 'HOPE_LIFECYCLE_HDU_IP=192.168.8.182\nHOPE_LIFECYCLE_HDU_IP=192.168.8.181\n'])
def test_unknown_or_other_receiver_rejected(tmp_path, content):
    receipt=tmp_path/'runtime.env'; receipt.write_text(content)
    with pytest.raises(ValueError): require_receiver_owner(receipt, '192.168.8.182')

@pytest.mark.parametrize('owner,expected', [('192.168.8.181', 44), ('192.168.8.182', 0)])
def test_actual_helper_checks_owner_before_any_stop(tmp_path, owner, expected):
    env_path=tmp_path/'.config/hope-foxglove/laptop-runtime.env'
    env_path.parent.mkdir(parents=True)
    env_path.write_text(f'HOPE_LIFECYCLE_HDU_IP={owner}\n')
    source=(ROOT/'foxglove/helpers/hope-lifecycle').read_text()
    function=source[source.index('reconnect_laptop() {'):source.index('\nstart_laptop() {')]
    script='''set -euo pipefail
require_user() { :; }
tmux_exists() { return 0; }
die() { exit 44; }
stop_laptop() { echo STOP >> "$TEST_LOG"; }
start_laptop() { echo START >> "$TEST_LOG"; }
'''+function+'\nreconnect_laptop\n'
    log=tmp_path/'calls'
    result=subprocess.run(['bash','-c',script], env={**os.environ,'HOME':str(tmp_path),
      'HOPE_ROOT':str(ROOT),'LAPTOP_USER':'test','LAPTOP_SESSION':'test',
      'HDU_WIFI_IP':'192.168.8.182','MOCAP_SOURCE':'vrpn','TEST_LOG':str(log),
      'MOTIVE_IP':'192.168.100.99','VRPN_IP':'192.168.100.99','VRPN_PORT':'3883','VRPN_UNITS':'mm','PANTS_PROFILE':'v2','TABLE_SIDE':'P1'}, capture_output=True,text=True)
    assert result.returncode==expected,result.stderr
    assert (log.read_text() if log.exists() else '') == ('STOP\nSTART\n' if expected==0 else '')


def test_geometry_confirmations_retained_only_for_exact_connection(tmp_path):
    from hope_mocap_connection import retained_confirmations
    fields=['HDU_IP','MOCAP_SOURCE','MOTIVE_IP','VRPN_IP','VRPN_PORT','VRPN_UNITS','PANTS_PROFILE','TABLE_SIDE']
    values=['192.168.8.182','vrpn','192.168.100.99','192.168.100.99','3883','mm','v2','P1']
    receipt=tmp_path/'runtime.env'
    receipt.write_text(''.join(f'HOPE_LIFECYCLE_{key}={value}\n' for key,value in zip(fields,values))+'HOPE_VRPN_PELVIS_ASSET_CONFIRMED=true\nHOPE_VRPN_WORLD_FRAME_CONFIRMED=true\n')
    assert retained_confirmations(receipt,values)=='true true'
    assert retained_confirmations(receipt,[*values[:-1],'P2'])=='false false'


def test_reconnect_releases_lock_while_detached_tmux_remains_alive(tmp_path):
    """Exercise a real, isolated tmux server: it must not inherit the flock."""
    import fcntl
    import shutil

    if not shutil.which('tmux'):
        pytest.skip('tmux is required for the detached-process regression')
    source = (ROOT / 'foxglove/helpers/hope-lifecycle').read_text()
    reconnect = source[source.index('reconnect_laptop() {'):source.index('\nstart_laptop() {')]
    start_tmux = source[source.index('start_tmux() {'):source.index('\nstop_tmux() {')]
    runtime = tmp_path / '.config/hope-foxglove/laptop-runtime.env'
    runtime.parent.mkdir(parents=True)
    runtime.write_text('HOPE_LIFECYCLE_HDU_IP=192.168.8.182\n')
    env = {**os.environ, 'HOME': str(tmp_path), 'TMUX_TMPDIR': str(tmp_path),
           'TMUX': '', 'HOPE_ROOT': str(ROOT), 'LAPTOP_USER': 'test',
           'LAPTOP_SESSION': 'lock-regression', 'HDU_WIFI_IP': '192.168.8.182',
           'MOCAP_SOURCE': 'vrpn', 'MOTIVE_IP': '192.168.100.99',
           'VRPN_IP': '192.168.100.99', 'VRPN_PORT': '3883',
           'VRPN_UNITS': 'mm', 'PANTS_PROFILE': 'v2', 'TABLE_SIDE': 'P1'}
    script = '''set -euo pipefail
require_user() { :; }
tmux_exists() { /usr/bin/tmux has-session -t "$1" 2>/dev/null; }
die() { exit 44; }
stop_laptop() { /usr/bin/tmux kill-session -t "$LAPTOP_SESSION"; }
start_laptop() {
  start_tmux "$LAPTOP_SESSION" sleep 30
  # The parent must retain mutual exclusion until reconnect itself finishes.
  if flock -n "$HOME/.config/hope-foxglove/laptop-runtime.env.connection.lock" true; then
    exit 45
  fi
}
''' + start_tmux + '\n' + reconnect + '\nreconnect_laptop\n'
    try:
        for _ in range(2):
            result = subprocess.run(['bash', '-c', script], env=env,
                                    capture_output=True, text=True, timeout=10)
            assert result.returncode == 0, result.stderr
            subprocess.run(['/usr/bin/tmux', 'has-session', '-t', 'lock-regression'],
                           env=env, check=True, capture_output=True, timeout=5)
            with Path(str(runtime) + '.connection.lock').open('w') as lock:
                fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    finally:
        subprocess.run(['/usr/bin/tmux', 'kill-server'], env=env,
                       capture_output=True, timeout=5)


@pytest.mark.parametrize('display_result', [0, 1])
def test_run_laptop_refreshes_display_after_receipt_without_gating_mocap(tmp_path, display_result):
    source = (ROOT / 'foxglove/helpers/hope-lifecycle').read_text()
    functions = source[source.index('refresh_laptop_display() {'):
                       source.index('\nvalidate_laptop_mocap() {')]
    # Redirect only the receipt location; execute the actual helper functions.
    functions = functions.replace('$HOME/.config/hope-foxglove/laptop-runtime.env',
                                  '$TEST_RECEIPT')
    receipt = tmp_path / 'runtime.env'
    receipt.write_text('HOPE_LIFECYCLE_HDU_IP=192.168.200.202\n')
    previous_inode = receipt.stat().st_ino
    runtime = tmp_path / 'foxglove/helpers/hope-laptop-container-runtime'
    runtime.parent.mkdir(parents=True)
    runtime.write_text('#!/bin/bash\nprintf "mocap\\n" >> "$TEST_LOG"\n')
    runtime.chmod(0o755)
    script = '''set -euo pipefail
require_user() { :; }
validate_laptop_mocap() { :; }
systemctl() {
  source "$TEST_RECEIPT"
  printf 'display|%s|%s|%s\\n' "$*" "$HOPE_LIFECYCLE_HDU_IP" \\
    "$HOPE_LIFECYCLE_SESSION_ID" >> "$TEST_LOG"
  return "$TEST_DISPLAY_RESULT"
}
''' + functions + '\nrun_laptop\n'
    log = tmp_path / 'calls'
    result = subprocess.run(['bash', '-c', script], capture_output=True, text=True,
        timeout=5, env={**os.environ, 'TEST_RECEIPT': str(receipt),
        'TEST_LOG': str(log), 'TEST_DISPLAY_RESULT': str(display_result),
        'HOPE_ROOT': str(tmp_path), 'HOPE_LAPTOP_RUNTIME': 'direct',
        'LAPTOP_USER': 'test', 'LAPTOP_SHARE': str(tmp_path),
        'SESSION_ID': 'new-session', 'HDU_WIFI_IP': '192.168.8.196',
        'MOCAP_SOURCE': 'vrpn', 'MOTIVE_IP': '192.168.100.99',
        'VRPN_IP': '192.168.100.99', 'VRPN_PORT': '3883',
        'VRPN_UNITS': 'mm', 'PANTS_PROFILE': 'v4', 'TABLE_SIDE': 'P2'})
    assert result.returncode == 0, result.stderr
    assert receipt.stat().st_ino != previous_inode
    assert not list(tmp_path.glob('runtime.env.tmp.*'))
    assert log.read_text().splitlines() == [
        'display|--user try-restart --no-block hope-host-bridge.service '
        'hope-live-scene.service|192.168.8.196|new-session', 'mocap']
    assert ('optional Host display refresh unavailable' in result.stderr) == bool(display_result)
