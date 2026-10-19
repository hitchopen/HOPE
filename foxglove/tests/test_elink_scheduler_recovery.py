"""Interruption/recovery tests; no real scheduler, service or robot is touched."""
import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import textwrap
from types import SimpleNamespace

import pytest


SOURCE = Path(__file__).resolve().parents[1] / 'helpers/hope_elink_scheduler.py'


def load_policy():
    spec = importlib.util.spec_from_file_location('elink_recovery', SOURCE)
    policy = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(policy)
    return policy


# Signals run in a disposable child so a regression cannot terminate pytest.
# All affinity/priority operations and process identities are in-memory mocks.
SIGNAL_SCRIPT = textwrap.dedent(r'''
    import copy
    import importlib.util
    import os
    import signal
    import sys
    from types import SimpleNamespace

    spec = importlib.util.spec_from_file_location('scheduler', sys.argv[1])
    p = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(p)
    signum = getattr(signal, sys.argv[2])
    scenario = sys.argv[3]
    signals = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)
    handlers = {s: signal.getsignal(s) for s in signals}
    original = [copy.deepcopy(p.FACTORY) for _ in range(3)]
    if scenario == 'restore':
        original = p.targets()
    actual = {tid: copy.deepcopy(value) for tid, value in zip((101, 102, 103), original)}
    snapshot = {'threads': [dict(tid=tid, settings=copy.deepcopy(value))
                           for tid, value in actual.items()]}
    p.same_thread = lambda *_: True
    fired = False
    rollback_signal = False
    reads = {tid: 0 for tid in actual}

    def deliver(s):
        os.kill(os.getpid(), s)

    def settings(tid):
        global fired
        reads[tid] += 1
        # The third read is the final verification pass, after admission and
        # immediate post-update readback. Cancellation must still roll back.
        if scenario == 'final_check' and tid == 103 and reads[tid] == 3:
            fired = True
            deliver(signum)
        return copy.deepcopy(actual[tid])

    p.settings = settings

    def affinity(tid, cpus):
        global fired, rollback_signal
        actual[tid]['cpus'] = list(cpus)
        if scenario in ('apply', 'repeated') and not fired and cpus == [1]:
            if scenario == 'apply' or tid == 102:
                fired = True
                deliver(signum)
        if scenario == 'repeated' and fired and cpus == p.FACTORY['cpus']:
            rollback_signal = True
            for s in signals:
                deliver(s)

    def scheduler(tid, kind, param):
        global fired, rollback_signal
        if scenario in ('keyboard', 'system_exit') and not fired and tid == 102 and kind == p.os.SCHED_RR:
            fired = True
            if scenario == 'keyboard':
                raise KeyboardInterrupt('injected after affinity')
            raise SystemExit(37)
        if scenario == 'error_then_signal' and not fired and tid == 102 and kind == p.os.SCHED_RR:
            fired = True
            raise PermissionError('injected priority denial')
        actual[tid].update(policy=kind, priority=param.sched_priority)
        if scenario == 'restore' and not fired and kind == p.os.SCHED_OTHER:
            fired = True
            deliver(signum)
        if scenario == 'error_then_signal' and fired and kind == p.os.SCHED_OTHER:
            rollback_signal = True
            for s in signals:
                deliver(s)

    p.os.sched_setaffinity = affinity
    p.os.sched_setscheduler = scheduler
    if not hasattr(p.os, 'sched_param'):
        p.os.sched_param = lambda priority: SimpleNamespace(sched_priority=priority)
    try:
        if scenario in ('guard_before', 'guard_late'):
            with p.deferred_termination() as check_cancelled:
                deliver(signum)
                if scenario == 'guard_before':
                    check_cancelled()
            raise AssertionError('captured signal was not reported')
        p.change(snapshot, [copy.deepcopy(p.FACTORY) for _ in range(3)]
                 if scenario == 'restore' else p.targets())
        raise AssertionError('interrupted operation returned success')
    except p.SchedulingInterrupted as exc:
        assert scenario not in ('keyboard', 'system_exit', 'error_then_signal'), scenario
        assert exc.signum == signum
    except KeyboardInterrupt:
        assert scenario == 'keyboard', scenario
    except SystemExit as exc:
        assert scenario == 'system_exit', scenario
        assert exc.code == 37
    except PermissionError as exc:
        assert scenario == 'error_then_signal', scenario
        assert 'injected priority denial' in str(exc)
    assert list(actual.values()) == original, (scenario, actual, original)
    if scenario in ('repeated', 'error_then_signal'):
        assert rollback_signal, 'test did not inject a signal during rollback'
    assert {s: signal.getsignal(s) for s in signals} == handlers
    print('signal-and-rollback-ok')
''')


@pytest.mark.parametrize('signame', ['SIGINT', 'SIGTERM', 'SIGHUP'])
@pytest.mark.parametrize('scenario', ['apply', 'restore', 'repeated', 'final_check',
                                      'guard_before', 'guard_late'])
def test_catchable_signals_restore_settings_and_handlers(signame, scenario):
    result = subprocess.run([sys.executable, '-c', SIGNAL_SCRIPT, str(SOURCE), signame, scenario],
                            capture_output=True, text=True, timeout=20)
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'signal-and-rollback-ok' in result.stdout


@pytest.mark.parametrize('scenario', ['keyboard', 'system_exit', 'error_then_signal'])
def test_programmatic_interrupt_and_signal_during_error_cleanup(scenario):
    result = subprocess.run([sys.executable, '-c', SIGNAL_SCRIPT, str(SOURCE), 'SIGTERM', scenario],
                            capture_output=True, text=True, timeout=20)
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'signal-and-rollback-ok' in result.stdout


@pytest.fixture
def recovery(monkeypatch, tmp_path):
    p = load_policy()

    class Config:
        def __str__(self):
            return '/tmp/hope_real/recovery/mdu/system/hal_elink_a3_gripper.yaml'

        def is_file(self):
            return True

        def is_symlink(self):
            return False

        def resolve(self):
            return Path('/agibot/data/bag/hope_pingpong_telemetry/sim2real_sessions/'
                        'recovery/mdu/system/hal_elink_a3_gripper.yaml')

    config = Config()
    for name, value in (('MODEL', p.ROBOT_MODEL), ('ELINK_CONFIG', 'elink'),
                        ('ETHERCAT_CONFIG', 'ethercat'), ('EXECUTABLE', 'executable'),
                        ('LIBRARY', 'audited library')):
        path = tmp_path / name
        path.write_text(value)
        monkeypatch.setattr(p, name, path)
    monkeypatch.setattr(p, 'HASHES', {path: hashlib.sha256(path.read_bytes()).hexdigest()
                                    for path in (p.EXECUTABLE, p.LIBRARY)})
    proc = tmp_path / 'proc'
    process = proc / '100'
    (process / 'map_files').mkdir(parents=True)
    (process / 'exe').symlink_to(p.EXECUTABLE)
    (process / 'cmdline').write_bytes(f'aimrt_main\0--cfg_file_path={config}\0'.encode())
    (process / 'stat').write_text('100 (hal_elink) S ' + '0 ' * 18 + '999 0 0')
    (process / 'maps').write_text(f'1000-2000 r-xp 00000000 fe:00 123 {p.LIBRARY}\n')
    (process / 'map_files/1000-2000').write_bytes(p.LIBRARY.read_bytes())
    for tid in (100, 101, 102, 103):
        task = process / 'task' / str(tid)
        task.mkdir(parents=True)
        (task / 'comm').write_text('hal_elink\n')
        (task / 'stat').write_text(f'{tid} (hal_elink) S ' + '0 ' * 18 + f'{tid + 1000} 0 0')
    monkeypatch.setattr(p, 'PROC', proc)
    state = tmp_path / 'state'
    state.mkdir(mode=0o700)
    monkeypatch.setattr(p, 'STATE', state)
    actual = {tid: copy.deepcopy(p.FACTORY) for tid in (101, 102, 103)}
    writes = []
    monkeypatch.setattr(p, 'settings', lambda tid: copy.deepcopy(actual[tid]))

    def affinity(tid, cpus):
        writes.append(('affinity', tid))
        actual[tid]['cpus'] = list(cpus)

    def scheduler(tid, kind, param):
        writes.append(('scheduler', tid))
        actual[tid].update(policy=kind, priority=param.sched_priority)

    monkeypatch.setattr(p.os, 'sched_setaffinity', affinity, raising=False)
    monkeypatch.setattr(p.os, 'sched_setscheduler', scheduler, raising=False)
    if not hasattr(p.os, 'sched_param'):
        monkeypatch.setattr(p.os, 'sched_param',
                            lambda priority: SimpleNamespace(sched_priority=priority), raising=False)
    snapshot = p.inspect(config)
    p.original_receipt(snapshot, create=True)
    return SimpleNamespace(p=p, config=config, snapshot=snapshot, actual=actual, writes=writes,
                           process=process, receipt=p.receipt_path(snapshot))


@pytest.mark.parametrize('original_target', [False, True])
def test_restore_known_halfway_state_with_matching_receipt(recovery, original_target):
    r = recovery
    p = r.p
    original = copy.deepcopy(r.snapshot)
    if original_target:
        for thread, target in zip(original['threads'], p.targets()):
            thread['settings'] = target
        r.receipt.write_text(json.dumps(original))
    # The same intermediate can follow interrupted promotion or demotion.
    r.actual[101] = dict(p.FACTORY, cpus=[1])
    snapshot = p.inspect(r.config, for_restore=True)
    receipt = p.original_receipt(snapshot, create=False)
    p.change(snapshot, [t['settings'] for t in receipt['threads']])
    result = p.inspect(r.config)
    assert [t['settings'] for t in result['threads']] == [t['settings'] for t in original['threads']]


def test_partial_state_is_rejected_by_normal_inspection(recovery):
    r = recovery
    r.actual[101]['cpus'] = [1]
    with pytest.raises(RuntimeError, match='Unexpected existing thread policy'):
        r.p.inspect(r.config)
    assert not r.writes


@pytest.mark.parametrize('fault', ['missing', 'malformed', 'missing-field', 'stale-pid',
                                  'stale-process-start', 'stale-thread-start', 'wrong-library',
                                  'unsafe-original', 'wrong-role-original', 'writable', 'symlink', 'owner'])
def test_restore_refuses_untrusted_or_mismatched_receipt_before_mutation(recovery, monkeypatch, fault):
    r = recovery
    r.actual[101]['cpus'] = [1]
    receipt = json.loads(r.receipt.read_text())
    if fault == 'missing':
        r.receipt.unlink()
    elif fault == 'malformed':
        r.receipt.write_text('{')
    elif fault == 'writable':
        r.receipt.chmod(0o666)
    elif fault == 'owner':
        other_uid = os.geteuid() + 1
        monkeypatch.setattr(r.p.os, 'geteuid', lambda: other_uid)
    elif fault == 'symlink':
        target = r.receipt.with_suffix('.saved')
        r.receipt.rename(target)
        r.receipt.symlink_to(target)
    else:
        if fault == 'missing-field':
            del receipt['threads']
        elif fault == 'stale-pid':
            receipt['pid'] += 1
        elif fault == 'stale-process-start':
            receipt['start_ticks'] += 1
        elif fault == 'stale-thread-start':
            receipt['threads'][0]['start_ticks'] += 1
        elif fault == 'wrong-library':
            receipt['library_sha256'] = 'unverified'
        elif fault == 'unsafe-original':
            receipt['threads'][0]['settings']['cpus'] = [7]
        elif fault == 'wrong-role-original':
            receipt['threads'][0]['settings'] = r.p.targets()[2]
        r.receipt.write_text(json.dumps(receipt))
    with pytest.raises((RuntimeError, OSError, ValueError)):
        r.p.inspect(r.config, for_restore=True)
    assert not r.writes


@pytest.mark.parametrize('setting', ['cpu', 'priority', 'policy', 'rr-wide-affinity'])
def test_restore_refuses_unknown_current_settings_even_with_valid_receipt(recovery, setting):
    r = recovery
    if setting == 'cpu':
        r.actual[101]['cpus'] = [7]
    elif setting == 'priority':
        r.actual[101] = dict(r.p.targets()[0], priority=99)
    elif setting == 'policy':
        r.actual[101]['policy'] = 999
    else:
        r.actual[101] = dict(r.p.targets()[0], cpus=[0, 1, 7])
    with pytest.raises(RuntimeError, match='Unexpected existing thread policy'):
        r.p.inspect(r.config, for_restore=True)
    assert not r.writes


def test_restore_keeps_vendor_binary_audit(recovery):
    r = recovery
    r.actual[101]['cpus'] = [1]
    r.p.LIBRARY.write_bytes(b'new vendor build')
    with pytest.raises(RuntimeError, match='Unaudited vendor binary'):
        r.p.inspect(r.config, for_restore=True)
    assert not r.writes


def test_restore_never_mutates_reused_thread(recovery):
    r = recovery
    r.actual[101]['cpus'] = [1]
    snapshot = r.p.inspect(r.config, for_restore=True)
    task_stat = r.process / 'task/101/stat'
    task_stat.write_text('101 (hal_elink) S ' + '0 ' * 18 + '99999 0 0')
    with pytest.raises(RuntimeError, match='identity changed'):
        r.p.change(snapshot, [copy.deepcopy(r.p.FACTORY) for _ in range(3)])
    assert not r.writes


def test_failed_rollback_continues_other_touched_threads(recovery, monkeypatch):
    r = recovery
    original = r.p.os.sched_setscheduler

    def fail(tid, kind, param):
        if tid == 103:
            raise PermissionError('injected mutation and rollback failure')
        original(tid, kind, param)

    monkeypatch.setattr(r.p.os, 'sched_setscheduler', fail)
    with pytest.raises(RuntimeError, match='rollback issues'):
        r.p.change(r.snapshot, r.p.targets())
    assert r.actual[101] == r.p.FACTORY
    assert r.actual[102] == r.p.FACTORY
    assert r.actual[103] == dict(r.p.FACTORY, cpus=[1])


@pytest.mark.parametrize('action', ['apply', 'verify', 'restore'])
def test_cli_only_restore_admits_partial_state(recovery, monkeypatch, capsys, action):
    r = recovery
    p = r.p
    r.actual[101]['cpus'] = [1]
    original_inspect = p.inspect

    def inspect(config, **kwargs):
        assert str(config) == str(r.config)
        return original_inspect(r.config, **kwargs)

    monkeypatch.setattr(p, 'inspect', inspect)
    monkeypatch.setattr(p, 'check_privileges', lambda: None)
    monkeypatch.setattr(p.os, 'umask', lambda _: 0o077)
    monkeypatch.setattr(p.os, 'pidfd_open', lambda _: os.open(os.devnull, os.O_RDONLY), raising=False)
    monkeypatch.setattr(p.select, 'select', lambda *_: ([], [], []))
    monkeypatch.setattr(sys, 'argv', ['scheduler', action, '--config', str(r.config)])
    if action == 'restore':
        p.main()
        assert all(s == p.FACTORY for s in r.actual.values())
        assert json.loads(capsys.readouterr().out)['action'] == 'restore'
    else:
        with pytest.raises(RuntimeError, match='Unexpected existing thread policy'):
            p.main()
        assert not r.writes
