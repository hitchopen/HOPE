#!/usr/bin/env python3
"""Nonblocking lock diagnosis; never deletes state or signals a process."""
import argparse
import errno
import fcntl
import json
import os
from pathlib import Path
import pwd
import stat
import time


def describe(path):
    try:
        s = path.lstat()
        return {'path': str(path), 'uid': s.st_uid, 'gid': s.st_gid,
                'owner': pwd.getpwuid(s.st_uid).pw_name, 'mode': oct(stat.S_IMODE(s.st_mode)),
                'inode': s.st_ino, 'device': [os.major(s.st_dev), os.minor(s.st_dev)],
                'symlink': path.is_symlink()}
    except (OSError, KeyError) as e:
        return {'path': str(path), 'error': str(e), 'errno': getattr(e, 'errno', None)}


def probe(state):
    lock = Path(str(state) + '.lock')
    result = {'observed_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()), 'state_path': str(state), 'lock_path': str(lock), 'euid': os.geteuid(),
              'egid': os.getegid(), 'ownership_before': [describe(x) for x in (state, lock, *state.parents)],
              'open': None, 'flock': None, 'holders': [], 'holder_visibility': 'current PID namespace'}
    fd = None
    try:
        fd = os.open(lock, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
        result['open'] = {'ok': True, 'errno': 0}
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            result['flock'] = {'ok': True, 'errno': 0}
            fcntl.flock(fd, fcntl.LOCK_UN)
        except OSError as e:
            result['flock'] = {'ok': False, 'errno': e.errno, 'error': e.strerror}
    except OSError as e:
        result['open'] = {'ok': False, 'errno': e.errno, 'error': e.strerror}
    finally:
        if fd is not None:
            os.close(fd)
    result['ownership_after'] = describe(lock)
    try:
        s = lock.stat()
        for line in Path('/proc/locks').read_text().splitlines():
            fields = line.split()
            if len(fields) < 6 or '->' in fields:
                continue
            major, minor, inode = fields[5].split(':')
            if (int(major,16),int(minor,16),int(inode)) != (os.major(s.st_dev),os.minor(s.st_dev),s.st_ino):
                continue
            pid = fields[4]
            try:
                command = Path('/proc', pid, 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
            except OSError as e:
                command = str(e)
            result['holders'].append({'proc_locks': line, 'pid': pid, 'command': command})
    except (OSError, ValueError) as e:
        result['holder_scan_error'] = str(e)
    return result


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('state', type=Path)
    p.add_argument('--json', type=Path)
    a = p.parse_args()
    report = probe(a.state)
    text = json.dumps(report, indent=2) + '\n'
    if a.json:
        a.json.write_text(text)
    print(text, end='')
