#!/usr/bin/env python3
"""One-use, stopped-system PTP restoration for the guarded A3 BSP.

Ordinary boots execute the byte-identical vendor guard. Only the live clock
coordinator can grant a restoration, after its hard step and before MDU restart.
No incoming shell commands or arbitrary paths are accepted.
"""
from __future__ import annotations

import fcntl
import hashlib
import ipaddress
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import time

BACKUPS = Path('/var/backups')
HELPER = Path('/usr/local/libexec/hope-guarded-clock')
GUARD = Path('/opt/agibot/entry/system/timesync_guard.py')
VENDOR = GUARD.with_name('timesync_guard.vendor.py')
TIMESYNC = GUARD.with_name('timesync.sh')
STATE = Path('/var/lib/hope-time-calibration')
RECEIPT = STATE / 'guarded-restore.json'
ACK = STATE / 'guarded-restored.json'
STATUS = STATE / 'status.json'
LOCK = Path('/var/lib/hope-lifecycle/hardware-operation.lock')
VENDOR_SHA = '8850128968c84cadf31f920fa0b3ce76d2bb4c6aa93aa56b5bcaf5f987f278da'
TIMESYNC_SHA = '80c35a34fef2f7b7abd845e09c7c3770dd9a863ed68e9caa71f6f4161afccc5b'
WRAPPER = '''#!/usr/bin/env python3
# HOPE receipt-bound restoration; ordinary boot retains the original BSP guard.
import os
os.execv('/usr/local/libexec/hope-guarded-clock', ['hope-guarded-clock', '--launch'])
'''


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def checked(argv, timeout=10):
    result = subprocess.run(argv, capture_output=True, text=True, timeout=timeout)
    require(result.returncode == 0, f'{argv[0]}: {result.stderr.strip()[-500:] or result.stdout.strip()[-500:]}')
    return result.stdout.strip()


def secure_bytes(path, *, root_owned=True):
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    with os.fdopen(fd, 'rb') as stream:
        stat = os.fstat(stream.fileno())
        require((not root_owned or stat.st_uid == 0) and stat.st_mode & 0o022 == 0, f'unsafe file: {path}')
        return stream.read()


def check_installation():
    require(secure_bytes(GUARD) == WRAPPER.encode(), 'BSP adapter wrapper mismatch')
    require(hashlib.sha256(secure_bytes(VENDOR)).hexdigest() == VENDOR_SHA, 'vendor guard identity mismatch')
    require(hashlib.sha256(secure_bytes(TIMESYNC, root_owned=False)).hexdigest() == TIMESYNC_SHA, 'timesync identity mismatch')
    secure_bytes(HELPER)


def boot_id():
    return Path('/proc/sys/kernel/random/boot_id').read_text().strip()


def process_start(pid):
    # comm may contain spaces/parentheses. starttime is field 22 after final ')'.
    return Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[19]


def active_status():
    state = json.loads(secure_bytes(STATUS))
    require(state.get('state') == 'RUNNING' and state.get('step') == 'HDU_RESTORE'
            and state.get('hard_step_attempted') is True
            and state.get('boot_id') == boot_id(), 'not a qualified live restoration')
    require(re.fullmatch(r'timecal_[0-9]{8}T[0-9]{6}Z', state.get('operation_id', '')),
            'invalid operation identity')
    return state


def require_locked():
    with LOCK.open('r') as stream:
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return
        raise RuntimeError('hardware operation lock is not held')


def validate_target(host, user):
    require(str(ipaddress.IPv4Address(host)) == host, 'invalid MDU IPv4 address')
    require(re.fullmatch(r'[a-z_][a-z0-9_-]{0,31}', user), 'invalid robot account')


def verify_mdu_stopped(host, user):
    validate_target(host, user)
    checked(['/usr/sbin/runuser', '-u', user, '--', '/usr/bin/ssh', '-n', '-T',
             '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=3',
             '-o', 'StrictHostKeyChecking=yes', '-o', 'ServerAliveInterval=2',
             '-o', 'ServerAliveCountMax=2', f'{user}@{host}',
             'sudo', '-n', '/usr/local/libexec/hope-lifecycle',
             'time-calibration-verify-stopped-mdu'], timeout=20)


def require_no_ptp():
    for name in ('ptp4l', 'phc2sys'):
        result = subprocess.run(['/usr/bin/pgrep', '-x', name], capture_output=True, timeout=5)
        require(result.returncode == 1, f'{name} remains or process query failed')


def require_qualified():
    checked(['/usr/bin/chronyc', 'waitsync', '1', '0.010', '0', '1'], timeout=5)


def atomic_json(path, document):
    fd, temporary = tempfile.mkstemp(prefix='.' + path.name, dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(document, stream)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        Path(temporary).unlink(missing_ok=True)


def arm(host, user):
    check_installation()
    state = active_status()
    require_locked()
    parent = os.getppid()
    main_pid = checked(['/usr/bin/systemctl', 'show', 'hope-time-calibration.service', '-p', 'MainPID', '--value'])
    require(main_pid == str(parent), 'caller is not the clock coordinator')
    require_no_ptp()
    for unit in ('agibot_pm', 'agibot_ui', 'agibot_top', 'agibot_roudi'):
        active = checked(['/usr/bin/systemctl', 'show', unit, '-p', 'ActiveState', '--value'])
        require(active in ('inactive', 'failed'), f'{unit} is not stopped')
    verify_mdu_stopped(host, user)
    require_qualified()
    require(not RECEIPT.exists(), 'unconsumed restoration receipt')
    ACK.unlink(missing_ok=True)
    atomic_json(RECEIPT, dict(schema=1, boot_id=boot_id(), operation_id=state['operation_id'],
                             coordinator_pid=parent, coordinator_start=process_start(parent),
                             issued_monotonic=time.monotonic(), mdu_ip=host, robot_user=user))


def validate_receipt(receipt, state, now, current_boot, start):
    require(receipt.get('schema') == 1 and receipt.get('boot_id') == current_boot,
            'receipt belongs to another boot/schema')
    require(receipt.get('operation_id') == state['operation_id'], 'receipt belongs to another operation')
    issued = receipt.get('issued_monotonic')
    require(type(issued) in (float, int) and math.isfinite(issued) and 0 <= now - issued <= 60,
            'receipt expired or has invalid time')
    require(receipt.get('coordinator_start') == start, 'coordinator restarted')
    validate_target(receipt.get('mdu_ip'), receipt.get('robot_user'))


def launch():
    check_installation()
    if not RECEIPT.exists():
        # Same directory preserves the vendor SCRIPT_DIR / timesync.sh resolution.
        os.execv('/usr/bin/python3', ['/usr/bin/python3', str(VENDOR)])
    claimed = RECEIPT.with_name(f'guarded-restore.claimed-{os.getpid()}')
    os.rename(RECEIPT, claimed)  # Atomic single-use consumption, even on failure.
    try:
        receipt = json.loads(secure_bytes(claimed))
        state = active_status()
        pid = receipt.get('coordinator_pid')
        require(type(pid) is int and pid > 1, 'invalid coordinator pid')
        validate_receipt(receipt, state, time.monotonic(), boot_id(), process_start(pid))
        main_pid = checked(['/usr/bin/systemctl', 'show', 'hope-time-calibration.service', '-p', 'MainPID', '--value'])
        require(main_pid == str(pid), 'coordinator identity changed')
        require_locked()
        require_no_ptp()
        verify_mdu_stopped(receipt['mdu_ip'], receipt['robot_user'])
        require_qualified()
        # This child is launched by the unchanged vendor PM launcher, retaining
        # its cgroup ownership. MDU PTP/HAL remain stopped until the ACK is checked.
        # Use inherited journal FDs, not captured pipes retained by PTP workers.
        subprocess.run(['/bin/bash', str(TIMESYNC)], check=True, timeout=30)
        atomic_json(ACK, receipt)
        print('HOPE_GUARDED_CLOCK_RESTORED ' + receipt['operation_id'], flush=True)
    finally:
        claimed.unlink(missing_ok=True)


def verify():
    state = active_status()
    # Workers may become visible just before the launcher atomically writes ACK.
    # Bound the rendezvous; an absent ACK must never restore the MDU.
    for _ in range(50):
        if ACK.exists():
            break
        time.sleep(0.1)
    receipt = json.loads(secure_bytes(ACK))
    require(receipt.get('boot_id') == boot_id() and receipt.get('operation_id') == state['operation_id'],
            'missing current restoration acknowledgement')
    require(not RECEIPT.exists(), 'restoration receipt was not consumed')


def install():
    current = secure_bytes(GUARD, root_owned=False)
    require(hashlib.sha256(secure_bytes(TIMESYNC, root_owned=False)).hexdigest() == TIMESYNC_SHA, 'unknown timesync baseline')
    require(current == WRAPPER.encode() or hashlib.sha256(current).hexdigest() == VENDOR_SHA,
            'unsupported vendor guard; inspect this BSP before installing')
    if VENDOR.exists():
        require(hashlib.sha256(secure_bytes(VENDOR)).hexdigest() == VENDOR_SHA, 'original guard backup differs')
    backup = BACKUPS / f'hope-guarded-clock-{time.time_ns()}'
    backup.mkdir(mode=0o700)
    shutil.copy2(GUARD, backup / GUARD.name)
    if HELPER.exists():
        shutil.copy2(HELPER, backup / HELPER.name)
    if not VENDOR.exists():
        shutil.copy2(GUARD, VENDOR)
        os.chown(VENDOR, 0, 0)
        VENDOR.chmod(0o644)
    HELPER.parent.mkdir(parents=True, exist_ok=True)
    try:
        for destination, content, mode in ((HELPER, Path(__file__).read_bytes(), 0o755),
                                           (GUARD, WRAPPER.encode(), 0o755)):
            fd, temporary = tempfile.mkstemp(dir=destination.parent)
            try:
                with os.fdopen(fd, 'wb') as stream:
                    stream.write(content)
                os.chmod(temporary, mode)
                os.replace(temporary, destination)
            finally:
                Path(temporary).unlink(missing_ok=True)
        check_installation()
    except BaseException:
        shutil.copy2(backup / GUARD.name, GUARD)
        if (backup / HELPER.name).exists():
            shutil.copy2(backup / HELPER.name, HELPER)
        else:
            HELPER.unlink(missing_ok=True)
        raise
    print(f'GUARDED_CLOCK_ADAPTER_INSTALLED backup={backup}; no services restarted')


def main():
    require(os.geteuid() == 0, 'root required')
    args = sys.argv[1:]
    if args == ['--install']:
        install()
    elif args == ['--check']:
        check_installation()
        print('GUARDED_CLOCK_ADAPTER_V1')
    elif len(args) == 3 and args[0] == '--arm':
        arm(args[1], args[2])
    elif args == ['--launch']:
        launch()
    elif args == ['--verify']:
        verify()
    elif args == ['--revoke']:
        RECEIPT.unlink(missing_ok=True)
        ACK.unlink(missing_ok=True)
    else:
        raise RuntimeError('usage: --install | --check | --arm MDU_IP ROBOT_USER | --launch | --verify | --revoke')


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print(f'GUARDED_CLOCK_REFUSED: {exc}', file=sys.stderr, flush=True)
        sys.exit(1)
