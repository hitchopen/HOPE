import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('elink_scheduler', ROOT / 'helpers/hope_elink_scheduler.py')
policy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(policy)


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
    monkeypatch.setattr(policy.os, 'sched_setaffinity', affinity)
    monkeypatch.setattr(policy.os, 'sched_setscheduler', scheduler)
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
    model.write_text('A3_P1D0')
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
    model.write_text('A3_P1D0')
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
    hook_start = source.index('  sudo -n python3 "$(dirname "$SELF")/hope_elink_scheduler.py"')
    hook_end = source.index('\n', source.index('    die HAL ELINK_SCHEDULER_NOT_READY', hook_start)) + 1
    old = source[:hook_start] + source[hook_end:]
    installed.write_text(old)
    subprocess.run(['python3', '-c', patch, str(installed), str(output)], check=True)
    assert output.read_text() == source
    installed.write_text('different layout')
    result = subprocess.run(['python3', '-c', patch, str(installed), str(output)], capture_output=True)
    assert result.returncode != 0


def test_start_hook_precedes_ethercat_and_runner_admission():
    source = (ROOT / 'helpers/hope-lifecycle').read_text()
    start_hal = source.split('start_hal() {', 1)[1].split('\nrun_runner()', 1)[0]
    assert start_hal.index('ELINK_SCHEDULER_NOT_READY') < start_hal.index('start_tmux "$MDU_HAL_SESSION"')


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
