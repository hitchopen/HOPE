import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('elink_scheduler', ROOT / 'helpers/hope_elink_scheduler.py')
policy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(policy)


def fake_configs(monkeypatch, tmp_path):
    for name in ('ELINK_CONFIG', 'ETHERCAT_CONFIG'):
        path = tmp_path / name
        path.write_text('fixture config')
        monkeypatch.setattr(policy, name, path)


def fake_threads(monkeypatch, tmp_path):
    snapshot = {'config': '/tmp/hope_real/test/mdu/system/hal_elink_a3_gripper.yaml',
                'pid': 100, 'start_ticks': 999, 'library_sha256': 'audited',
                'threads': [{'tid': t, 'start_ticks': t + 1000,
                             'settings': copy.deepcopy(policy.FACTORY)} for t in (101, 102, 103)]}
    actual = {t['tid']: copy.deepcopy(t['settings']) for t in snapshot['threads']}
    live = {t['tid']: True for t in snapshot['threads']}
    monkeypatch.setattr(policy, 'STATE', tmp_path)
    monkeypatch.setattr(policy, 'settings', lambda tid: copy.deepcopy(actual[tid]))
    monkeypatch.setattr(policy, 'same_thread', lambda snap, t: live[t['tid']])
    def affinity(tid, cpus):
        actual[tid]['cpus'] = list(cpus)
    def scheduler(tid, kind, param):
        actual[tid].update(policy=kind, priority=param.sched_priority)
    # These are fake scheduling operations on every host, including platforms
    # whose Python does not expose Linux's scheduler APIs.
    monkeypatch.setattr(policy.os, 'sched_setaffinity', affinity, raising=False)
    monkeypatch.setattr(policy.os, 'sched_setscheduler', scheduler, raising=False)
    monkeypatch.setattr(policy.os, 'sched_param',
                        lambda priority: SimpleNamespace(sched_priority=priority), raising=False)
    return snapshot, actual, live


def test_apply_and_restore_preserve_original_across_repeated_apply(monkeypatch, tmp_path):
    snap, actual, _ = fake_threads(monkeypatch, tmp_path)
    original = policy.original_receipt(snap, create=True)
    policy.change(snap, [policy.DESIRED] * 3)
    assert all(s == policy.DESIRED for s in actual.values())
    repeated = copy.deepcopy(snap)
    for t in repeated['threads']:
        t['settings'] = copy.deepcopy(actual[t['tid']])
    assert policy.original_receipt(repeated, create=True) == original
    policy.change(repeated, [t['settings'] for t in original['threads']])
    assert all(s == policy.FACTORY for s in actual.values())


def test_priority_failure_restores_affinity_and_previously_changed_threads(monkeypatch, tmp_path):
    snap, actual, _ = fake_threads(monkeypatch, tmp_path)
    real = policy.os.sched_setscheduler
    def fail_second(tid, kind, param):
        if tid == 102 and kind == os.SCHED_RR:
            raise PermissionError('priority denied')
        real(tid, kind, param)
    monkeypatch.setattr(policy.os, 'sched_setscheduler', fail_second)
    with pytest.raises(PermissionError, match='priority denied'):
        policy.change(snap, [policy.DESIRED] * 3)
    assert all(s == policy.FACTORY for s in actual.values())


def test_reused_tid_is_never_mutated(monkeypatch, tmp_path):
    snap, actual, live = fake_threads(monkeypatch, tmp_path)
    live[102] = False
    writes = []
    original = policy.set_settings
    def record(tid, setting):
        writes.append(tid)
        original(tid, setting)
    monkeypatch.setattr(policy, 'set_settings', record)
    with pytest.raises(RuntimeError, match='identity changed'):
        policy.change(snap, [policy.DESIRED] * 3)
    assert 102 not in writes and 103 not in writes
    assert all(s == policy.FACTORY for s in actual.values())


def test_concurrent_policy_change_is_preserved(monkeypatch, tmp_path):
    snap, actual, _ = fake_threads(monkeypatch, tmp_path)
    actual[102]['cpus'] = [1]
    with pytest.raises(RuntimeError, match='concurrently'):
        policy.change(snap, [policy.DESIRED] * 3)
    assert actual[101] == policy.FACTORY
    assert actual[102]['cpus'] == [1]


def test_readback_mismatch_rolls_back(monkeypatch, tmp_path):
    snap, actual, _ = fake_threads(monkeypatch, tmp_path)
    original = policy.os.sched_setaffinity
    def ignore_requested_affinity(tid, cpus):
        if cpus != policy.DESIRED['cpus']:
            original(tid, cpus)
    monkeypatch.setattr(policy.os, 'sched_setaffinity', ignore_requested_affinity)
    with pytest.raises(RuntimeError, match='readback failed'):
        policy.change(snap, [policy.DESIRED] * 3)
    assert all(s == policy.FACTORY for s in actual.values())


def test_stale_receipt_rejected(monkeypatch, tmp_path):
    snap, _, _ = fake_threads(monkeypatch, tmp_path)
    stale = copy.deepcopy(snap)
    stale['threads'][1]['start_ticks'] += 1
    policy.receipt_path(snap).write_text(json.dumps(stale))
    with pytest.raises(RuntimeError, match='different ELink instance'):
        policy.original_receipt(snap, create=True)


def test_stat_parser_handles_spaces_and_parentheses(tmp_path):
    (tmp_path / 'stat').write_text('123 (thread ) name) S ' + '0 ' * 18 + '456 0 0')
    assert policy.start_ticks(tmp_path) == 456


def test_unknown_model_or_build_fails_before_process_selection(monkeypatch, tmp_path):
    # Use a path-shaped object because the production config guard intentionally
    # accepts only a real managed /tmp/hope_real session, not arbitrary fixtures.
    class Config:
        def __str__(self):
            return '/tmp/hope_real/test/mdu/system/hal_elink_a3_gripper.yaml'
        def is_file(self): return True
        def is_symlink(self): return False
        def resolve(self): return self
    model = tmp_path / 'model'
    model.write_text('unsupported')
    monkeypatch.setattr(policy, 'MODEL', model)
    monkeypatch.setattr(policy, 'find_process', lambda _: pytest.fail('process selection must not run'))
    with pytest.raises(RuntimeError, match='Unsupported robot model'):
        policy.inspect(Config())
    model.write_text('A3_T3D0')
    fake_configs(monkeypatch, tmp_path)
    binary = tmp_path / 'vendor.so'
    binary.write_bytes(b'unknown build')
    monkeypatch.setattr(policy, 'HASHES', {binary: 'wrong-hash'})
    with pytest.raises(RuntimeError, match='Unaudited vendor binary'):
        policy.inspect(Config())


@pytest.mark.parametrize('count', [2, 3, 4])
def test_inspection_excludes_main_and_requires_three_audited_transport_threads(monkeypatch, tmp_path, count):
    class Config:
        def __str__(self):
            return '/tmp/hope_real/test/mdu/system/hal_elink_a3_gripper.yaml'
        def is_file(self): return True
        def is_symlink(self): return False
        def resolve(self):
            return Path('/agibot/data/bag/hope_pingpong_telemetry/sim2real_sessions/test/mdu/system/hal_elink_a3_gripper.yaml')
    config = Config()
    model = tmp_path / 'model'
    # Read the model actually admitted by the checked-in lifecycle. This catches
    # the former T3 lifecycle / P1 scheduler conflict instead of repeating it.
    lifecycle = (ROOT / 'helpers/hope-lifecycle').read_text()
    model.write_text(re.search(r'^readonly MDU_SUPPORTED_ROBOT_MODEL=(.*)$', lifecycle, re.M)[1])
    fake_configs(monkeypatch, tmp_path)
    binary = tmp_path / 'aimrt_main'
    library = tmp_path / 'vendor.so'
    binary.write_bytes(b'executable')
    library.write_bytes(b'audited library')
    proc = tmp_path / 'proc'
    process = proc / '100'
    (process / 'map_files').mkdir(parents=True)
    (process / 'exe').symlink_to(binary)
    (process / 'cmdline').write_bytes(f'aimrt_main\0--cfg_file_path={config}\0'.encode())
    (process / 'stat').write_text('100 (hal_elink) S ' + '0 ' * 18 + '999 0 0')
    # Overlayfs backing device/inode intentionally differ from visible stat().
    (process / 'maps').write_text(f'1000-2000 r-xp 00000000 fe:00 123 {library}\n')
    (process / 'map_files/1000-2000').write_bytes(library.read_bytes())
    for tid in [100] + list(range(101, 101 + count)):
        task = process / 'task' / str(tid)
        task.mkdir(parents=True)
        (task / 'comm').write_text('hal_elink\n')
        (task / 'stat').write_text(f'{tid} (hal_elink) S ' + '0 ' * 18 + f'{tid + 1000} 0 0')
    monkeypatch.setattr(policy, 'MODEL', model)
    monkeypatch.setattr(policy, 'EXECUTABLE', binary)
    monkeypatch.setattr(policy, 'LIBRARY', library)
    monkeypatch.setattr(policy, 'HASHES', {p: hashlib.sha256(p.read_bytes()).hexdigest() for p in (binary, library)})
    monkeypatch.setattr(policy, 'PROC', proc)
    monkeypatch.setattr(policy, 'settings', lambda _: copy.deepcopy(policy.FACTORY))
    if count == 3:
        result = policy.inspect(config)
        assert [t['tid'] for t in result['threads']] == [101, 102, 103]
        assert [t['role'] for t in result['threads']] == ['main', 'send', 'receive']
        (process / 'map_files/1000-2000').write_bytes(b'different loaded build')
        with pytest.raises(RuntimeError, match='mapping differs'):
            policy.inspect(config)
    else:
        with pytest.raises(RuntimeError, match='thread layout'):
            policy.inspect(config)


def test_installer_hook_matches_source_and_rejects_different_layout(tmp_path):
    installer = (ROOT / 'a3/install_mdu_elink_scheduler.sh').read_text()
    patch = installer.split("<<'PY'\n", 1)[1].split('\nPY\n', 1)[0]
    source = (ROOT / 'helpers/hope-lifecycle').read_text()
    installed = tmp_path / 'installed'
    output = tmp_path / 'new'
    installed.write_text(source)
    subprocess.run(['python3', '-c', patch, str(installed), str(output)], check=True)
    assert output.read_text() == source  # idempotent
    hook_start = source.index('  sudo -n "$(dirname "$SELF")/hope_elink_scheduler.py" apply')
    hook_end = source.index('\n', source.index('    die HAL ELINK_SCHEDULER_NOT_READY', hook_start)) + 1
    old = source[:hook_start] + source[hook_end:]
    preflight_start = old.index('  python3 "$(dirname "$SELF")/hope_elink_scheduler.py" check-hardware')
    preflight_end = old.index('  render_t3_elink_config >/dev/null', preflight_start)
    old = old[:preflight_start] + old[preflight_end:]
    installed.write_text(old)
    subprocess.run(['python3', '-c', patch, str(installed), str(output)], check=True)
    assert output.read_text() == source
    # Upgrade the previously published hook and hardware-only preflight too.
    legacy = source.replace('sudo -n "$(dirname "$SELF")/hope_elink_scheduler.py" apply --config',
                            'sudo -n python3 "$(dirname "$SELF")/hope_elink_scheduler.py" apply --config')
    permission_start = legacy.index('  sudo -n "$(dirname "$SELF")/hope_elink_scheduler.py" check-privileges')
    permission_end = legacy.index('  render_t3_elink_config >/dev/null', permission_start)
    installed.write_text(legacy[:permission_start] + legacy[permission_end:])
    subprocess.run(['python3', '-c', patch, str(installed), str(output)], check=True)
    assert output.read_text() == source
    installed.write_text('different layout')
    result = subprocess.run(['python3', '-c', patch, str(installed), str(output)], capture_output=True)
    assert result.returncode != 0


def test_start_hook_precedes_ethercat_and_runner_admission():
    source = (ROOT / 'helpers/hope-lifecycle').read_text()
    start_hal = source.split('start_hal() {', 1)[1].split('\nrun_runner()', 1)[0]
    assert start_hal.index('ELINK_SCHEDULER_NOT_READY') < start_hal.index('start_tmux "$MDU_HAL_SESSION"')
    assert start_hal.index('check_t3_hardware_config HAL') < start_hal.index('sudo -n systemctl stop')


def test_checked_in_lifecycle_matches_audited_scheduler_profile(tmp_path):
    lifecycle = ROOT / 'helpers/hope-lifecycle'
    policy.validate_lifecycle_profile(lifecycle)
    changed = tmp_path / 'hope-lifecycle'
    for before, after in (
        ('A3_T3D0_ELINK_HAND_V2', 'A3_P1D0_ELINK_HAND_V2'),
        ('MDU_SUPPORTED_ROBOT_MODEL=A3_T3D0', 'MDU_SUPPORTED_ROBOT_MODEL=A3_P1D0'),
        ('hal_elink_a3_cfg.yaml', 'hal_elink_a3_ultra_cfg.yaml'),
        ('hal_ethercat_a3_t3d0_cfg.yaml', 'hal_ethercat_a3_p1d0_cfg.yaml'),
    ):
        changed.write_text(lifecycle.read_text().replace(before, after))
        with pytest.raises(RuntimeError, match='Lifecycle hardware'):
            policy.validate_lifecycle_profile(changed)


def test_hardware_cli_needs_no_root_or_process_state(monkeypatch, capsys):
    lifecycle = ROOT / 'helpers/hope-lifecycle'
    monkeypatch.setattr(sys, 'argv', ['scheduler', 'check-hardware', '--lifecycle', str(lifecycle)])
    monkeypatch.setattr(policy, 'audit_hardware', lambda: {'hardware_profile': policy.HARDWARE_PROFILE})
    monkeypatch.setattr(policy.os, 'geteuid', lambda: pytest.fail('read-only check must not require root'))
    monkeypatch.setattr(policy, 'find_process', lambda _: pytest.fail('read-only check must not inspect processes'))
    policy.main()
    assert json.loads(capsys.readouterr().out)['hardware_profile'] == policy.HARDWARE_PROFILE


@pytest.fixture
def installer_workspace():
    if os.geteuid() != 0 or os.environ.get('HOPE_ELINK_INSTALL_TEST') != '1':
        pytest.skip('real sudo policy tests require an explicitly enabled disposable root container')
    # /tmp is world-writable; production permission checks require every parent
    # of the installed helper to be root-owned without group/other write access.
    with tempfile.TemporaryDirectory(prefix='hope-elink-test-', dir='/opt') as directory:
        root = Path(directory)
        root.chmod(0o755)
        try:
            yield root
        finally:
            (Path('/etc/sudoers.d') / root.name).unlink(missing_ok=True)


@pytest.mark.parametrize('fault', [None, 'p1-model', 'binary', 'missing-config', 'config-symlink',
                                  'lifecycle-model', 'lifecycle-elink', 'lifecycle-ethercat',
                                  'probe-failure'])
def test_installer_and_lifecycle_share_hardware_admission(installer_workspace, fault):
    """Run the actual installer and installed preflight, with filesystem fixtures.

    Only absolute deployment paths and binary contents/hashes are relocated.
    Model/profile logic, shell commands and CLI checks run unchanged. No HAL,
    service or scheduling calls are needed for this pre-start admission.
    """
    tmp_path = installer_workspace
    sudoers = Path('/etc/sudoers.d') / tmp_path.name
    package = tmp_path / 'package'
    (package / 'a3').mkdir(parents=True)
    (package / 'helpers').mkdir()
    destination = tmp_path / 'installed'
    destination.mkdir()
    lifecycle = destination / 'hope-lifecycle'
    scheduler = destination / 'hope_elink_scheduler.py'
    candidate = package / 'helpers/hope_elink_scheduler.py'
    paths = (policy.MODEL, policy.ELINK_CONFIG, policy.ETHERCAT_CONFIG,
             policy.EXECUTABLE, policy.LIBRARY)
    mapped = {path: tmp_path / path.name for path in paths}
    for path, fixture in mapped.items():
        fixture.write_text(policy.ROBOT_MODEL if path == policy.MODEL else 'fixture')
    # Minimal valid vendor YAML exercises the real lifecycle renderer too.
    mapped[policy.ELINK_CONFIG].write_text('''aimrt:
  channel:
    sub_topics_options:
      - topic_name: /body_drive/hand_joint_command
        enable_backends: [iceoryx]
  executor:
    executors:
      - name: hand_elink_server_worker
      - name: hand_control_worker
ElinkModule:
  topic:
    upper:
      hand:
        command_topic: /body_drive/hand_joint_command
        enable: false
  elink:
    upper:
      hand_enable: true
''')
    def relocate(source):
        for path, fixture in mapped.items():
            source = source.replace(str(path), str(fixture))
        return source
    source = relocate((ROOT / 'helpers/hope-lifecycle').read_text())
    source += '\n# Preserve field-local content during installation.\n'
    lifecycle.write_text(source)
    candidate_source = relocate((ROOT / 'helpers/hope_elink_scheduler.py').read_text())
    for path, digest in policy.HASHES.items():
        candidate_source = candidate_source.replace(digest, hashlib.sha256(mapped[path].read_bytes()).hexdigest())
    candidate.write_text(candidate_source)
    scheduler.write_text('# previous scheduler\n')
    if fault == 'probe-failure':
        # Syntax/hardware validation succeeds, but the installed root probe must
        # reject a non-isolated interpreter and roll back all three files.
        candidate.write_text(candidate_source.replace('#!/usr/bin/python3 -I', '#!/usr/bin/python3'))
        sudoers.write_text('# previous dedicated policy\n')
        sudoers.chmod(0o440)
    elif fault == 'p1-model':
        mapped[policy.MODEL].write_text('A3_P1D0')
    elif fault == 'binary':
        mapped[policy.LIBRARY].write_text('changed build')
    elif fault == 'missing-config':
        mapped[policy.ETHERCAT_CONFIG].unlink()
    elif fault == 'config-symlink':
        mapped[policy.ETHERCAT_CONFIG].unlink()
        mapped[policy.ETHERCAT_CONFIG].symlink_to(mapped[policy.ELINK_CONFIG])
    elif fault and fault.startswith('lifecycle-'):
        old, new = {
            'lifecycle-model': ('MDU_SUPPORTED_ROBOT_MODEL=A3_T3D0', 'MDU_SUPPORTED_ROBOT_MODEL=A3_P1D0'),
            'lifecycle-elink': (str(mapped[policy.ELINK_CONFIG]), str(tmp_path / 'ultra.yaml')),
            'lifecycle-ethercat': (str(mapped[policy.ETHERCAT_CONFIG]), str(tmp_path / 'p1d0.yaml')),
        }[fault]
        lifecycle.write_text(source.replace(old, new))
    before = lifecycle.read_bytes(), scheduler.read_bytes()
    installer = package / 'a3/install_mdu_elink_scheduler.sh'
    installer.write_text((ROOT / 'a3/install_mdu_elink_scheduler.sh').read_text()
                         .replace('/usr/local/libexec', str(destination))
                         .replace('/etc/sudoers.d/hope-elink-scheduler', str(sudoers))
                         .replace('/var/backups/hope-elink-scheduler-', str(tmp_path / 'backup-')))
    install_command = ['bash', str(installer), '--robot-user', 'nobody']
    result = subprocess.run(install_command, capture_output=True, text=True)
    if fault:
        assert result.returncode != 0, result.stdout
        assert (lifecycle.read_bytes(), scheduler.read_bytes()) == before
        if fault == 'probe-failure':
            assert 'Installation failed; restored files' in result.stderr
            assert sudoers.read_text() == '# previous dedicated policy\n'
        else:
            assert not list(tmp_path.glob('backup-*'))
        return
    assert result.returncode == 0, result.stderr
    assert lifecycle.read_text() == source
    assert scheduler.read_text() == candidate_source
    for _ in range(2):  # repeated installation preserves local content
        subprocess.run(install_command, check=True, capture_output=True)
    assert sudoers.stat().st_uid == 0
    assert sudoers.stat().st_mode & 0o777 == 0o440
    # Use a real account with no other sudo grants or cached credentials.
    unprivileged = ['runuser', '-u', 'nobody', '--']
    probe = subprocess.run([*unprivileged, 'sudo', '-n', str(scheduler), 'check-privileges'],
                           capture_output=True, text=True)
    assert probe.returncode == 0, probe.stderr
    assert 'ELINK_SCHEDULER_PRIVILEGES_OK' in probe.stdout
    managed = Path('/tmp/hope_real/sudo_probe/mdu/system/hal_elink_a3_gripper.yaml')
    managed.parent.mkdir(parents=True, exist_ok=True)
    managed.write_text('fixture session config')
    applied = subprocess.run([*unprivileged, 'sudo', '-n', str(scheduler), 'apply', '--config', str(managed)],
                             capture_output=True, text=True)
    # The exact START command reaches the process guard without a password.
    # There is deliberately no ELink process in this disposable test container.
    assert applied.returncode != 0 and 'Expected one managed ELink process, found []' in applied.stderr
    # The grant does not authorize arbitrary Python, commands, paths or actions.
    for command in (
        ['python3', '-c', 'print("unrestricted")'], ['/bin/sh', '-c', 'id'],
        [str(scheduler), 'inspect', '--config', '/tmp/hope_real/probe/mdu/system/hal_elink_a3_gripper.yaml'],
        [str(scheduler), 'apply', '--config', '/etc/passwd'],
    ):
        denied = subprocess.run([*unprivileged, 'sudo', '-n', *command], capture_output=True)
        assert denied.returncode != 0, command
    # Even arguments matching the sudoers glob must pass helper validation.
    invalid = subprocess.run([*unprivileged, 'sudo', '-n', str(scheduler), 'apply', '--config',
                              '/tmp/hope_real/../mdu/system/hal_elink_a3_gripper.yaml'],
                             capture_output=True, text=True)
    assert invalid.returncode != 0 and 'Expected managed session' in invalid.stderr
    # Execute the actual lifecycle admission with the installed scheduler.
    command = [*unprivileged, 'bash', '-c',
               'source "$1"; SESSION_ID=sudo_probe; check_t3_hardware_config PREFLIGHT', 'bash', str(lifecycle)]
    subprocess.run(command, check=True, capture_output=True, text=True)
    # A missing rule is caught by the real noninteractive sudo probe.
    rule = sudoers.read_text()
    sudoers.unlink()
    missing = subprocess.run(command, capture_output=True, text=True)
    assert missing.returncode != 0 and 'ELINK_SCHEDULER_SUDO_NOT_READY' in missing.stdout
    # Exercise START itself: only service/process queries are mocked; sudo and
    # the installed helper remain real. Never invoke an actual service command.
    start = subprocess.run([*unprivileged, 'bash', '-c', '''
source "$1"
SESSION_ID=sudo_probe
require_user() { :; }
pgrep() { return 1; }
systemctl() { return 0; }
has_unmanaged_hal() { return 1; }
sudo() {
  if [[ "$*" == *'systemctl stop'* ]]; then echo VENDOR_STOP_ATTEMPTED; return 99; fi
  command sudo "$@"
}
start_hal
''', 'bash', str(lifecycle)], capture_output=True, text=True)
    assert start.returncode != 0 and 'ELINK_SCHEDULER_SUDO_NOT_READY' in start.stdout
    assert 'VENDOR_STOP_ATTEMPTED' not in start.stdout
    # Having only permission for the probe cannot mask a missing apply grant.
    sudoers.write_text(rule.split(', ', 1)[0] + '\n')
    sudoers.chmod(0o440)
    missing_apply = subprocess.run(command, capture_output=True, text=True)
    assert missing_apply.returncode != 0 and 'ELINK_SCHEDULER_APPLY_NOT_PERMITTED' in missing_apply.stdout
    sudoers.write_text(rule)
    # Writable helper content must never pass privileged admission.
    scheduler.chmod(0o777)
    writable = subprocess.run(command, capture_output=True, text=True)
    assert writable.returncode != 0 and 'ELINK_SCHEDULER_SUDO_NOT_READY' in writable.stdout
    scheduler.chmod(0o755)
    # A post-install vendor update must fail admission before stopping services.
    mapped[policy.EXECUTABLE].write_text('vendor update')
    rejected = subprocess.run(command, capture_output=True, text=True)
    assert rejected.returncode != 0
    assert 'ELINK_HARDWARE_PROFILE_NOT_AUDITED' in rejected.stdout


def test_ack_processing_can_preempt_sender_cpu_burst():
    main, send, receive = policy.targets()
    assert receive['priority'] > main['priority'] > send['priority'] > 0
    assert all(t['policy'] == os.SCHED_RR and t['cpus'] == [1] for t in (main, send, receive))


@pytest.mark.parametrize("scheduler_exit", [0, 1])
def test_scheduler_failure_blocks_ethercat_start(tmp_path, scheduler_exit):
    source = (ROOT / "helpers/hope-lifecycle").read_text()
    start_hal = source.split("start_hal() {", 1)[1].split("\nrun_runner()", 1)[0]
    # Execute the real admission tail with command mocks. No sudo, robot,
    # processes or /tmp/hope_real files are touched by this test.
    tail = start_hal[start_hal.index("  check_t3_elink_ready HAL"):].rsplit("}", 1)[0]
    script = r'''set -e
check_t3_elink_ready() { :; }
sudo() { echo SCHEDULER; return "$1_code"; }
die() { echo "$2"; exit 42; }
emit() { :; }
start_tmux() { echo ETHERCAT_STARTED; }
wait_for_session_log() { :; }
check_t3_hal_ready() { :; }
SELF=/fake/hope-lifecycle
'''.replace('"$1_code"', str(scheduler_exit)) + tail
    result = subprocess.run(["bash", "-c", script], capture_output=True, text=True)
    if scheduler_exit:
        assert result.returncode == 42
        assert "ELINK_SCHEDULER_NOT_READY" in result.stdout
        assert "ETHERCAT_STARTED" not in result.stdout
    else:
        assert result.returncode == 0, result.stderr
        assert result.stdout.index("SCHEDULER") < result.stdout.index("ETHERCAT_STARTED")
