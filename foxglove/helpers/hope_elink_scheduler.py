#!/usr/bin/env python3
"""Scheduling policy for the three audited vendor ELink transport threads.

Does not send commands, change hand targets, or restart processes. The firmware
and thread layout are deliberately pinned: an unknown build must be audited
before this policy is applied. Thread creation order and inherited comm identify
the roles only in this exact, audited binary layout.
"""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import select
import sys

PROC = Path('/proc')
MODEL = Path('/agibot/data/info/model')
# The checked-in lifecycle and passive HAL observer use this T3 deployment.
# These hashes are also recorded in its serve_hal_clock.sha256 manifest.
# P1/Ultra uses different config paths and is not an audited startup profile.
HARDWARE_PROFILE = 'A3_T3D0_ELINK_HAND_V2'
ROBOT_MODEL = 'A3_T3D0'
ELINK_CONFIG = Path('/opt/agibot/config/hal_elink/hal_elink_a3_cfg.yaml')
ETHERCAT_CONFIG = Path('/opt/agibot/config/hal_ethercat/hal_ethercat_a3_t3d0_cfg.yaml')
LIBRARY = Path('/opt/agibot/bin/libhal_elink_pkg.so')
EXECUTABLE = Path('/opt/agibot/bin/aimrt_main')
HASHES = {
    LIBRARY: 'f83690f1c8a3d7e1f50a7c38bd5ed2ccc01543274b680d548af38f7f154225af',
    EXECUTABLE: '9ab4c3fa441b4c13fac543b8778cd650a5e1751cad26fda1f4a918912e113ad6',
}
STATE = Path('/var/lib/hope/elink-scheduler')
# CPU 1 is a housekeeping core, away from the dedicated Runner/EtherCAT cores and
# the CPU 7 network/IPC path. A CPU 7 trial retained periodic scheduling tails.
DESIRED = {'policy': os.SCHED_RR, 'priority': 20, 'cpus': [1]}
FACTORY = {'policy': os.SCHED_OTHER, 'priority': 0, 'cpus': [0, 1, 7]}
ROLES = ('main', 'send', 'receive')
PRIORITIES = (23, 20, 24)


def validate_lifecycle_profile(lifecycle):
    """Reject mixed source/field installs before installing or stopping HAL."""
    source = lifecycle.read_text()
    expected = {
        'MDU_ROBOT_MODEL_FILE': str(MODEL),
        'MDU_SUPPORTED_ROBOT_MODEL': ROBOT_MODEL,
        'MDU_ELINK_CONFIG': str(ELINK_CONFIG),
        'MDU_HAL_GRIPPER_BASE_CONFIG': str(ETHERCAT_CONFIG),
    }
    if re.findall(r'^# HOPE_HARDWARE_PROFILE=(\S+)$', source, re.M) != [HARDWARE_PROFILE]:
        raise RuntimeError('Lifecycle hardware profile differs from the audited scheduler')
    for name, value in expected.items():
        if re.findall(rf'^readonly {name}=(.*)$', source, re.M) != [value]:
            raise RuntimeError(f'Lifecycle hardware setting differs: {name}')


def audit_hardware():
    """Read-only admission, usable before the managed ELink process exists."""
    if not MODEL.is_file() or MODEL.is_symlink():
        raise RuntimeError('Robot model must be a regular file')
    if MODEL.read_text().strip() != ROBOT_MODEL:
        raise RuntimeError(f'Unsupported robot model; expected {ROBOT_MODEL}')
    for path in (ELINK_CONFIG, ETHERCAT_CONFIG):
        if not path.is_file() or path.is_symlink():
            raise RuntimeError(f'Audited hardware config must be a regular file: {path}')
    for path, expected in HASHES.items():
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise RuntimeError(f'Unaudited vendor binary: {path}')
    return {'hardware_profile': HARDWARE_PROFILE, 'robot_model': ROBOT_MODEL,
            'elink_factory_config': str(ELINK_CONFIG),
            'ethercat_config': str(ETHERCAT_CONFIG),
            'binary_sha256': {str(path): sha for path, sha in HASHES.items()}}


def targets():
    # The sender can execute continuously for several ms. Equal RT priorities
    # leave ACK processing waiting behind it until it blocks or exhausts the
    # RR time slice. Receive and protocol processing must preempt the sender.
    return [dict(DESIRED, priority=priority) for priority in PRIORITIES]


def start_ticks(path):
    # comm can contain spaces or ')'; starttime is field 22, after the last ')'.
    return int((path / 'stat').read_text().rsplit(')', 1)[1].split()[19])


def settings(tid):
    return {'policy': os.sched_getscheduler(tid),
            'priority': os.sched_getparam(tid).sched_priority,
            'cpus': sorted(os.sched_getaffinity(tid))}


def find_process(config):
    matches = []
    for path in PROC.glob('[0-9]*'):
        try:
            args = (path / 'cmdline').read_bytes().split(b'\0')
            if f'--cfg_file_path={config}'.encode() in args:
                if (path / 'exe').resolve() == EXECUTABLE:
                    matches.append(int(path.name))
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
    if len(matches) != 1:
        raise RuntimeError(f'Expected one managed ELink process, found {matches}')
    return matches[0]


def inspect(config):
    if not re.fullmatch(r'/tmp/hope_real/[A-Za-z0-9_-]+/mdu/system/hal_elink_a3_gripper\.yaml', str(config)):
        raise RuntimeError('Expected managed session ELink config path')
    # Managed recording relocates the session's mdu directory to persistent
    # storage. Accept that exact session mirror, not arbitrary symlink targets.
    mirror = Path('/agibot/data/bag/hope_pingpong_telemetry/sim2real_sessions') / Path(*Path(str(config)).parts[3:])
    if not config.is_file() or config.is_symlink() or config.resolve() not in (config, mirror):
        raise RuntimeError('Managed config must be a regular file in its session or recording mirror')
    hardware = audit_hardware()
    pid = find_process(config)
    path = PROC / str(pid)
    process_start = start_ticks(path)
    if not os.path.samefile(path / 'exe', EXECUTABLE):
        raise RuntimeError('Running executable differs from the audited file')
    mapped = []
    for line in (path / 'maps').read_text().splitlines():
        parts = line.split()
        if len(parts) == 6 and parts[5] == str(LIBRARY) and 'x' in parts[1]:
            mapped.append(path / 'map_files' / parts[0])
    # The MDU uses overlayfs: stat() on the visible path and /proc/maps can
    # report different device/inode pairs for the same backing file. Hash the
    # actual mapped backing file, which also detects replaced on-disk builds.
    if len(mapped) != 1 or hashlib.sha256(mapped[0].read_bytes()).hexdigest() != HASHES[LIBRARY]:
        raise RuntimeError('Running ELink mapping differs from the audited library')
    threads = []
    for task in (path / 'task').iterdir():
        tid = int(task.name)
        if tid != pid and (task / 'comm').read_text().strip() == 'hal_elink':
            threads.append({'tid': tid, 'start_ticks': start_ticks(task),
                            'settings': settings(tid)})
    if len(threads) != 3:
        raise RuntimeError(f'Unknown ELink transport thread layout: {len(threads)} threads')
    threads.sort(key=lambda t: t['tid'])
    # Audited library initialization creates main, send, receive in this order.
    # Reject a TID wrap/replacement rather than guessing the role assignment.
    if [t['start_ticks'] for t in threads] != sorted(t['start_ticks'] for t in threads):
        raise RuntimeError('ELink thread creation order changed')
    for thread, role, target in zip(threads, ROLES, targets(), strict=True):
        thread['role'] = role
        if thread['settings'] not in (FACTORY, target):
            raise RuntimeError(f'Unexpected existing thread policy: {thread}')
    return {**hardware, 'config': str(config), 'pid': pid, 'start_ticks': process_start,
            'library_sha256': HASHES[LIBRARY],
            'threads': threads}


def same_thread(snapshot, thread):
    path = PROC / str(snapshot['pid'])
    task = path / 'task' / str(thread['tid'])
    try:
        return (start_ticks(path) == snapshot['start_ticks']
                and start_ticks(task) == thread['start_ticks']
                and (task / 'comm').read_text().strip() == 'hal_elink')
    except (FileNotFoundError, ProcessLookupError):
        return False


def set_settings(tid, value):
    # Confine the thread before increasing its scheduling priority. During
    # rollback, demote before widening affinity to the factory CPU set.
    if value['policy'] == os.SCHED_OTHER:
        os.sched_setscheduler(tid, value['policy'], os.sched_param(value['priority']))
        os.sched_setaffinity(tid, value['cpus'])
    else:
        os.sched_setaffinity(tid, value['cpus'])
        os.sched_setscheduler(tid, value['policy'], os.sched_param(value['priority']))


def change(snapshot, targets):
    """Apply transactionally, restoring the observed settings on any failure."""
    touched = []
    try:
        for thread, target in zip(snapshot['threads'], targets, strict=True):
            if not same_thread(snapshot, thread):
                raise RuntimeError('ELink process/thread identity changed')
            if settings(thread['tid']) != thread['settings']:
                raise RuntimeError('ELink scheduling changed concurrently')
            touched.append(thread)
            set_settings(thread['tid'], target)
            if not same_thread(snapshot, thread) or settings(thread['tid']) != target:
                raise RuntimeError('ELink scheduling readback failed')
        for thread, target in zip(snapshot['threads'], targets, strict=True):
            if not same_thread(snapshot, thread) or settings(thread['tid']) != target:
                raise RuntimeError('ELink final readback failed')
    except Exception as error:
        failures = []
        for thread in reversed(touched):
            if not same_thread(snapshot, thread):
                failures.append(f"thread {thread['tid']} exited; not touching a reused TID")
                continue
            try:
                set_settings(thread['tid'], thread['settings'])
                if settings(thread['tid']) != thread['settings']:
                    raise RuntimeError('rollback readback mismatch')
            except Exception as rollback_error:
                failures.append(str(rollback_error))
        if failures:
            raise RuntimeError(f'{error}; rollback issues: {failures}') from error
        raise


def receipt_path(snapshot):
    session = Path(snapshot['config']).parts[3]
    return STATE / f"{session}-{snapshot['pid']}-{snapshot['start_ticks']}.json"


def original_receipt(snapshot, create):
    path = receipt_path(snapshot)
    if create and not path.exists():
        # Save all original settings before the first mutation. Never replace
        # this receipt on repeated apply, so restore still means pre-fix state.
        with path.open('x') as stream:
            json.dump(snapshot, stream, indent=2)
            stream.flush()
            os.fsync(stream.fileno())
    original = json.loads(path.read_text())
    keys = ('pid', 'start_ticks', 'config', 'library_sha256')
    identity = lambda s: [(t['tid'], t['start_ticks']) for t in s['threads']]
    if any(original[k] != snapshot[k] for k in keys) or identity(original) != identity(snapshot):
        raise RuntimeError('Rollback receipt belongs to a different ELink instance')
    return original


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('check-hardware', 'inspect', 'apply', 'verify', 'restore'))
    parser.add_argument('--config', type=Path)
    parser.add_argument('--lifecycle', type=Path)
    args = parser.parse_args()
    if args.action == 'check-hardware':
        if args.lifecycle is None or args.config is not None:
            parser.error('check-hardware requires --lifecycle and accepts no --config')
        validate_lifecycle_profile(args.lifecycle)
        print(json.dumps(audit_hardware(), sort_keys=True))
        return  # No root, state directory, /proc lookup or scheduling changes.
    if args.config is None or args.lifecycle is not None:
        parser.error('session actions require --config and accept no --lifecycle')
    if os.geteuid() != 0:
        raise RuntimeError('Run as root on the MDU')
    os.umask(0o077)
    STATE.mkdir(parents=True, exist_ok=True, mode=0o700)
    with (STATE / 'lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        snapshot = inspect(args.config)
        # Hold a pidfd through mutation/readback to detect process exit. Also
        # check process and task start times before touching each thread.
        fd = os.pidfd_open(snapshot['pid'])
        try:
            if select.select([fd], [], [], 0)[0]:
                raise RuntimeError('ELink process exited')
            if args.action == 'apply':
                original_receipt(snapshot, create=True)
                change(snapshot, targets())
            elif args.action == 'restore':
                original = original_receipt(snapshot, create=False)
                change(snapshot, [t['settings'] for t in original['threads']])
            result = inspect(args.config)
            if result['pid'] != snapshot['pid'] or result['start_ticks'] != snapshot['start_ticks']:
                raise RuntimeError('ELink process replaced during operation')
            if args.action in ('apply', 'verify'):
                if any(t['settings'] != target for t, target in zip(result['threads'], targets(), strict=True)):
                    raise RuntimeError('ELink scheduling policy is not active')
            result.update(action=args.action, receipt=str(receipt_path(snapshot)))
            print(json.dumps(result, sort_keys=True))
        finally:
            os.close(fd)


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print(f'ELINK_SCHEDULER_ERROR: {exc}', file=sys.stderr)
        sys.exit(1)
