from pathlib import Path
import os
import subprocess
import sys
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'a3'))
from hope_mocap_connection import require_receiver_owner


def test_same_robot_changed_ip_is_reconnectable_but_other_identity_is_not(tmp_path):
    receipt = tmp_path / 'runtime.env'
    receipt.write_text('HOPE_LIFECYCLE_HDU_IP=192.168.8.182\nHOPE_LIFECYCLE_MACHINE_ID=' + 'a' * 32 + '\n')
    require_receiver_owner(receipt, '192.168.200.202', 'a' * 32)
    for machine in ['', 'b' * 32]:
        with pytest.raises(ValueError):
            require_receiver_owner(receipt, '192.168.8.182', machine)

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
ensure_hdu_host_key() { :; }
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
    assert retained_confirmations(receipt,values)=='true'
    assert retained_confirmations(receipt,[*values[:-1],'P2'])=='false'


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
ensure_hdu_host_key() { :; }
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
    # Keep the Host-bridge restart stamp out of the real home directory.
    functions = functions.replace('$HOME/.config/hope-foxglove/host-bridge.peer',
                                  str(tmp_path / 'host-bridge.peer'))
    receipt = tmp_path / 'runtime.env'
    receipt.write_text('HOPE_LIFECYCLE_HDU_IP=192.168.200.202\n')
    previous_inode = receipt.stat().st_ino
    runtime = tmp_path / 'foxglove/helpers/hope-laptop-container-runtime'
    runtime.parent.mkdir(parents=True)
    runtime.write_text('#!/bin/bash\nprintf "mocap\\n" >> "$TEST_LOG"\n')
    runtime.chmod(0o755)
    script = '''set -euo pipefail
require_user() { :; }
ensure_hdu_host_key() { :; }
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
        'LAPTOP_WIFI_IP': '192.168.8.150', 'MDU_INTERNAL_IP': '10.42.10.12',
        'MOCAP_SOURCE': 'vrpn', 'MOTIVE_IP': '192.168.100.99',
        'VRPN_IP': '192.168.100.99', 'VRPN_PORT': '3883',
        'VRPN_UNITS': 'mm', 'PANTS_PROFILE': 'v4', 'TABLE_SIDE': 'P2'})
    assert result.returncode == 0, result.stderr
    assert receipt.stat().st_ino != previous_inode
    assert not list(tmp_path.glob('runtime.env.tmp.*'))
    assert log.read_text().splitlines() == [
        'display|--user try-restart --no-block hope-host-bridge.service '
        'hope-live-scene.service hope-xbox-input.service|192.168.8.196|new-session', 'mocap']
    assert ('optional Host display refresh unavailable' in result.stderr) == bool(display_result)



def _reconnect_region():
    source = (ROOT / 'foxglove/helpers/hope-lifecycle').read_text()
    return source[source.index('reconnect_laptop() {'):source.index('\nstart_laptop() {')]


@pytest.mark.parametrize('session_alive,leftovers,expected', [
    (True, '', ['STOP', 'RETIRE model21800_20261118T033915Z', 'START']),
    # Leftovers of an earlier stop whose tmux session is already gone are retired too.
    (False, '4242', ['STOP', 'RETIRE model21800_20261118T033915Z', 'START']),
    # An untagged (manual) bridge is not ours: no stop/retire; start_laptop refuses it as before.
    (False, '', ['START'])])
def test_reconnect_retires_the_stopped_receiver_session_before_starting(tmp_path, session_alive, leftovers, expected):
    env_path = tmp_path / '.config/hope-foxglove/laptop-runtime.env'
    env_path.parent.mkdir(parents=True)
    env_path.write_text('HOPE_LIFECYCLE_HDU_IP=192.168.8.182\nHOPE_LIFECYCLE_SESSION_ID=model21800_20261118T033915Z\n')
    script = 'set -euo pipefail\n' + _reconnect_region() + '''
require_user() { :; }
ensure_hdu_host_key() { :; }
tmux_exists() { [[ "$SESSION_ALIVE" == 1 ]]; }
die() { exit 44; }
stop_laptop() { echo STOP >> "$TEST_LOG"; }
start_laptop() { echo START >> "$TEST_LOG"; }
retire_laptop_receiver() { echo "RETIRE $1" >> "$TEST_LOG"; }
laptop_session_pids() { [[ "$1" == model21800_20261118T033915Z ]] && printf '%s' "$LEFTOVERS"; return 0; }
reconnect_laptop
'''
    log = tmp_path / 'calls'
    result = subprocess.run(['bash', '-c', script], env={**os.environ, 'HOME': str(tmp_path),
        'HOPE_ROOT': str(ROOT), 'LAPTOP_USER': 'test', 'LAPTOP_SESSION': 'test', 'HDU_WIFI_IP': '192.168.8.182',
        'MOCAP_SOURCE': 'motive', 'TEST_LOG': str(log), 'SESSION_ALIVE': '1' if session_alive else '0',
        'LEFTOVERS': leftovers}, capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stderr
    assert log.read_text().splitlines() == expected


def test_retire_kills_only_the_stopped_receiver_tree(tmp_path):
    """Real processes: a launch that ignores SIGINT with a child that ignores SIGTERM is cleared;
    a same-session process outside the tree (like the Xbox service) and an untagged manual bridge survive."""
    import getpass
    import time
    session = 'model21800_20991231T000000Z'
    launch = tmp_path / 'optitrack_hope_bridge.launch.py'
    launch.write_text("trap '' INT\n(trap '' INT TERM; exec sleep 300) &\nwait\nwait\n")
    relay = tmp_path / 'hope_base_pose_flat_relay'
    relay.write_text("trap '' INT\nwhile :; do sleep 1; done\n")
    tagged = {**os.environ, 'HOPE_LIFECYCLE_SESSION_ID': session}
    receiver = subprocess.Popen(['bash', str(launch)], env=tagged, start_new_session=True)
    # A receiver node whose launch already died (not in any launch tree) is still retired.
    orphan = subprocess.Popen(['bash', str(relay)], env=tagged, start_new_session=True)
    bystander = subprocess.Popen(['sleep', '300'], env=tagged)
    manual = subprocess.Popen(['bash', str(launch)], env={k: v for k, v in os.environ.items()
                                                          if k != 'HOPE_LIFECYCLE_SESSION_ID'},
                              start_new_session=True)
    try:
        time.sleep(0.5)
        child = int(subprocess.run(['pgrep', '-P', str(receiver.pid)], capture_output=True, text=True).stdout.split()[0])
        script = 'set -euo pipefail\n' + _reconnect_region() + '''
sleep() { command sleep 0.01; }
retire_laptop_receiver "$SESSION"
'''
        result = subprocess.run(['bash', '-c', script], env={**os.environ, 'LAPTOP_USER': getpass.getuser(),
                                'SESSION': session}, capture_output=True, text=True, timeout=60)
        assert result.returncode == 0, result.stderr
        receiver.wait(timeout=5)
        orphan.wait(timeout=5)
        assert not Path(f'/proc/{child}').exists() or 'Z' in Path(f'/proc/{child}/stat').read_text().split()[2]
        assert bystander.poll() is None and manual.poll() is None
        # Nothing of the session left: an untagged manual bridge alone does not hold retire up.
        script_real_sleep = 'set -euo pipefail\n' + _reconnect_region() + '\nretire_laptop_receiver "$SESSION"\n'
        started = time.monotonic()
        result = subprocess.run(['bash', '-c', script_real_sleep], env={**os.environ, 'LAPTOP_USER': getpass.getuser(),
                                'SESSION': session}, capture_output=True, text=True, timeout=60)
        assert result.returncode == 0, result.stderr
        assert time.monotonic() - started < 3 and manual.poll() is None
    finally:
        import signal
        for group in (receiver.pid, manual.pid, orphan.pid):
            try:
                os.killpg(group, signal.SIGKILL)
            except ProcessLookupError:
                pass
        if bystander.poll() is None:
            bystander.kill()
        for process in (receiver, bystander, manual, orphan):
            process.wait(timeout=5)



def _host_key_function():
    source = (ROOT / 'foxglove/helpers/hope-lifecycle').read_text()
    return source[source.index('ensure_hdu_host_key() {'):source.index('\nprepare_laptop() {')]


KEY = 'ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAICS+k3UtU+EDvfu2fDnhVROStestkey'


@pytest.mark.parametrize('pinned,presented,already_known,expect_added', [
    (KEY, KEY, False, True),            # new address, pinned key: trusted
    (KEY, KEY + 'x', False, False),     # different machine at that address: never trusted
    (KEY, KEY, True, False),            # already known: no scan, unchanged
    (None, KEY, False, False),          # nothing pinned: no trust-on-first-use
    (KEY, '', False, False),            # nothing answers: unchanged
])
def test_hdu_host_key_added_only_for_a_pinned_key(tmp_path, pinned, presented, already_known, expect_added):
    config = tmp_path / '.config/hope-foxglove'
    config.mkdir(parents=True)
    if pinned is not None:
        (config / 'hdu_host_keys').write_text(pinned + '\n')
    known = tmp_path / '.ssh/known_hosts'
    known.parent.mkdir()
    known.write_text('192.168.0.126 ' + KEY + '\n')
    log = tmp_path / 'calls'
    script = 'set -euo pipefail\n' + _host_key_function() + '''
ssh-keygen() { [[ "$ALREADY" == 1 ]]; }
ssh-keyscan() { echo SCAN >> "$TEST_LOG"; [[ -n "$PRESENTED" ]] && echo "10.44.84.162 $PRESENTED"; return 0; }
ensure_hdu_host_key
'''
    result = subprocess.run(['bash', '-c', script], capture_output=True, text=True, timeout=10,
        env={**os.environ, 'HOME': str(tmp_path), 'HDU_WIFI_IP': '10.44.84.162', 'TEST_LOG': str(log),
             'PRESENTED': presented, 'ALREADY': '1' if already_known else '0'})
    assert result.returncode == 0, result.stderr
    lines = known.read_text().splitlines()
    assert ('10.44.84.162 ' + KEY in lines) == expect_added
    assert len(lines) == (2 if expect_added else 1)
    assert log.exists() == (pinned is not None and not already_known)
