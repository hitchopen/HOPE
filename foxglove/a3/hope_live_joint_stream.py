#!/usr/bin/env python3
"""Read measured Runner joints through one SSH connection; never command hardware.

The running process must own the trace FD, match the current lifecycle session,
and still be recording. No old file search or synthetic zero pose is allowed.
20 Hz polling retains all fresh 50 Hz samples for contact-time interpolation.
Only stdlib is needed on either computer. JSONL stdout is telemetry only.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import time


class Unavailable(RuntimeError):
    pass


def process_identity(entry, expected_session):
    try:
        argv = entry.joinpath('cmdline').read_bytes().decode().split('\0')
        if Path(argv[0]).name != 'a3_deploy_onnx_ref_pingpong':
            return None
        if argv[argv.index('--session-id') + 1] != expected_session:
            return None
        return entry.joinpath('stat').read_text().rsplit(')', 1)[1].split()[19]
    except (OSError, ValueError, IndexError):
        return None


def runner_identity(expected_session, proc=Path('/proc'), session_path=Path('/tmp/hope_model21800_session_id')):
    if session_path.read_text().strip() != expected_session:
        raise Unavailable('lifecycle_session_changed')
    matches = []
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        started = process_identity(entry, expected_session)
        if started is not None:
            matches.append((entry, started))
    if len(matches) != 1:
        raise Unavailable('expected_one_active_runner')
    return matches[0]


def active_trace(runner, expected_session):
    matches = []
    for fd in runner.joinpath('fd').iterdir():
        try:
            target = Path(os.readlink(fd))
            if target.name == 'runner_trace.csv' and expected_session in target.parts:
                matches.append(fd)
        except OSError:
            continue
    if len(matches) != 1:
        raise Unavailable('live_trace_not_recording')
    return matches[0]


class TraceTail:
    """Bounded complete-line tail that does not replay the preexisting file."""
    def __init__(self, path, max_backlog=262144):
        self.stream = open(path, 'rb', buffering=0)
        stat = os.fstat(self.stream.fileno())
        self.identity = (stat.st_dev, stat.st_ino)
        self.header = next(csv.reader([self.stream.readline().decode().strip()]))
        needed = {'session_id', 'sync_complete', 'wall_time_ns', 'source_state_timestamp_ns',
                  'mode', 'tick', 'swing_sign', 'frozen_command_seq', 'shot_seq',
                  'frozen_producer_epoch', 'frozen_flight_id'}
        self.selected = [(i, key) for i, key in enumerate(self.header)
                         if key in needed or key.startswith('q_')]
        if 'wall_time_ns' not in self.header:
            self.selected = list(enumerate(self.header))
        self.max_backlog = max_backlog
        header_end = self.stream.tell()
        self.cursor = max(header_end, stat.st_size - 65536)
        self.initial_partial = self.cursor > header_end
        self.initial = True

    def close(self):
        self.stream.close()

    def rows(self):
        size = os.fstat(self.stream.fileno()).st_size
        if size < self.cursor:
            raise Unavailable('live_trace_truncated')
        skip_first = (self.initial and self.initial_partial) or size - self.cursor > self.max_backlog
        if size - self.cursor > self.max_backlog:
            self.cursor = size - self.max_backlog
        self.stream.seek(self.cursor)
        data = self.stream.read(self.max_backlog)
        end = data.rfind(b'\n')
        if end < 0:
            return []
        self.cursor += end + 1
        lines = data[:end].splitlines()
        if skip_first:
            lines = lines[1:]
        if self.initial:
            # A fresh last row is sufficient for the initial display; history
            # before connection must never be treated as a new live shot.
            lines = lines[-1:]
            self.initial = False
        result = []
        for line in lines:
            fields = next(csv.reader([line.decode()]))
            if len(fields) == len(self.header):
                result.append({key: fields[i] for i, key in self.selected})
        return result[-16:]


def measured_sample(row, expected_session, now_ns, max_age_ms=200.0):
    try:
        if row['session_id'] != expected_session or int(row['sync_complete']) != 1:
            return None
        wall_ns = int(row['wall_time_ns'])
        source_ns = int(row['source_state_timestamp_ns'])
        wall_age_ms = (now_ns - wall_ns) / 1e6
        source_age_ms = (now_ns - source_ns) / 1e6
        if not (-50 <= wall_age_ms <= max_age_ms and -50 <= source_age_ms <= max_age_ms):
            return None
        q = {key[2:]: float(value) for key, value in row.items() if key.startswith('q_')}
        if len(q) != 31 or not all(math.isfinite(v) for v in q.values()):
            return None
        return {
            'schema': 1, 'session_id': expected_session,
            'source_timestamp_s': source_ns / 1e9, 'wall_timestamp_s': wall_ns / 1e9,
            'source_timestamp_ns': source_ns, 'wall_timestamp_ns': wall_ns,
            'remote_source_age_ms': source_age_ms,
            'mode': int(row['mode']), 'sync_complete': True, 'q': q,
            'tick': int(row['tick']), 'swing_sign': int(float(row['swing_sign'])),
            'frozen_command_seq': int(row['frozen_command_seq']),
            'shot_seq': int(row['shot_seq']),
            'frozen_producer_epoch': int(row.get('frozen_producer_epoch', '0')),
            'frozen_flight_id': int(row.get('frozen_flight_id', '0')),
        }
    except (KeyError, TypeError, ValueError, OverflowError):
        return None


def remote_main(args):
    runner, started = runner_identity(args.session)
    fd_path = active_trace(runner, args.session)
    tail = TraceTail(fd_path)
    identity = f'{runner.name}:{started}'
    begin = time.monotonic()
    cpu_begin = time.process_time()
    next_check = begin
    deadline = begin
    emitted = payload_bytes = 0
    last_wall_ns = 0
    try:
        while not args.duration or time.monotonic() - begin < args.duration:
            now = time.monotonic()
            if now >= next_check:
                if Path('/tmp/hope_model21800_session_id').read_text().strip() != args.session:
                    raise Unavailable('lifecycle_session_changed')
                if process_identity(runner, args.session) != started:
                    raise Unavailable('runner_identity_changed')
                current_fd = active_trace(runner, args.session)
                stat = os.stat(current_fd)
                if (stat.st_dev, stat.st_ino) != tail.identity:
                    raise Unavailable('recording_segment_changed')
                next_check = now + 1.0
            now_ns = time.time_ns()
            for row in tail.rows():
                sample = measured_sample(row, args.session, now_ns, args.max_age_ms)
                if sample is None or sample['wall_timestamp_ns'] <= last_wall_ns:
                    continue
                last_wall_ns = sample['wall_timestamp_ns']
                sample['runner_identity'] = identity
                line = json.dumps(sample, separators=(',', ':'), allow_nan=False) + '\n'
                sys.stdout.write(line)
                emitted += 1
                payload_bytes += len(line)
            sys.stdout.flush()
            deadline += 1.0 / args.poll_hz
            time.sleep(max(0, deadline - time.monotonic()))
            if time.monotonic() - deadline > 1.0:
                deadline = time.monotonic()
    finally:
        tail.close()
        elapsed = time.monotonic() - begin
        print(json.dumps({'live_joint_stream_stats': {
            'elapsed_s': elapsed, 'samples': emitted, 'payload_bytes': payload_bytes,
            'samples_per_s': emitted / max(elapsed, 1e-9),
            'payload_bytes_per_s': payload_bytes / max(elapsed, 1e-9),
            'cpu_percent_one_core': 100 * (time.process_time() - cpu_begin) / max(elapsed, 1e-9),
            'runner_identity': identity,
        }}), file=sys.stderr, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='hope-newrobot-mdu')
    parser.add_argument('--session', required=True)
    parser.add_argument('--poll-hz', type=float, default=20.0)
    parser.add_argument('--max-age-ms', type=float, default=200.0)
    parser.add_argument('--duration', type=float, default=0.0)
    parser.add_argument('--remote', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if not 1 <= args.poll_hz <= 50 or not 20 <= args.max_age_ms <= 500:
        parser.error('poll-hz must be 1..50 and max-age-ms must be 20..500')
    if args.remote:
        try:
            remote_main(args)
            return 0
        except (Unavailable, OSError) as exc:
            print('live_joint_stream_unavailable: ' + str(exc), file=sys.stderr, flush=True)
            return 3
    command = shlex.join(['nice', '-n', '10', 'python3', '-u', '-', '--remote', '--session', args.session,
                          '--poll-hz', str(args.poll_hz), '--max-age-ms', str(args.max_age_ms),
                          '--duration', str(args.duration)])
    # Sending source on stdin avoids deploying a daemon or changing robot files.
    child = subprocess.Popen(['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8',
                              '-o', 'ServerAliveInterval=3', '-o', 'ServerAliveCountMax=2',
                              args.host, command], stdin=subprocess.PIPE)
    def stop(_signal, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    try:
        child.communicate(Path(__file__).read_bytes())
        return child.returncode
    except KeyboardInterrupt:
        return 130
    finally:
        if child.poll() is None:
            child.terminate()
            try:
                child.wait(timeout=2)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()


if __name__ == '__main__':
    raise SystemExit(main())
