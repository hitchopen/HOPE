#!/usr/bin/env python3
"""Five declared fresh-process trials per registered nominal profile; no retries."""
import argparse
import hashlib
import json
import os
import platform
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_hashes(repo):
    paths = subprocess.check_output(['git', '-C', str(repo), 'ls-files'], text=True).splitlines()
    prefixes = ('hope_ws/src/', 'NatNet2ROS2/src/motion_capture_tracking_interfaces/',
                'a3_deploy/a3_deploy_example/src/a3/a3_deploy_onnx_ref/include/')
    return {p: digest(repo / p) for p in paths if p.startswith(prefixes) and '/docs/' not in p}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', type=Path, required=True)
    parser.add_argument('--build', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--diagnostic-comparison', action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    def git(*values):
        return subprocess.check_output(['git', '-C', str(args.repo), *values], text=True).strip()
    assert not git('status', '--porcelain'), 'scored candidate must be a clean committed worktree'
    contract_path = args.repo / 'hope_ws/src/hope_planner_cpp/docs/integrated_planner_software_contract.json'
    contract = json.loads(contract_path.read_text())
    registered = json.loads(subprocess.check_output(
        ['ctest', '--test-dir', str(args.build), '--show-only=json-v1'], text=True))
    (args.output / 'registered-tests.json').write_text(json.dumps(registered, indent=2) + '\n')
    tests = {test['name']: test for test in registered['tests']}
    trials = []
    repetitions = 2 if args.diagnostic_comparison else contract['repetitions_per_profile']
    profiles = contract['diagnostic_comparison_profiles'] if args.diagnostic_comparison else contract['nominal_profiles']
    for repetition in range(1, repetitions + 1):
        for name in profiles:
            assert name in tests, ('missing registered profile', name)
            test = tests[name]
            command = test['command'].copy()
            output = args.output / f'repetition-{repetition}' / name / 'result.json'
            command[command.index('--output') + 1] = str(output)
            # Every declared qualification trial needs actual loss attribution.
            # The separate diagnostic pair still measures accounting on/off.
            accounting_enabled = not args.diagnostic_comparison or repetition == 1
            if accounting_enabled:
                command.append('--stage-accounting')
            environment = {}
            for prop in test.get('properties', []):
                if prop['name'] == 'ENVIRONMENT':
                    environment.update(item.split('=', 1) for item in prop['value'])
            trials.append(dict(profile=name, repetition=repetition, command=command,
                               stage_accounting=accounting_enabled, environment=environment, output=str(output)))
    before = source_hashes(args.repo)
    binaries = {str(args.build / name): digest(args.build / name)
                for name in ('hope_planner_cpp_node', 'hope_integrated_runner_probe')}
    environment = dict(platform=platform.platform(), python=sys.version,
                       affinity=sorted(os.sched_getaffinity(0)), numpy=np.__version__,
                       ros_distro=os.environ.get('ROS_DISTRO'), rmw=os.environ.get('RMW_IMPLEMENTATION'),
                       compiler=subprocess.check_output(['c++', '--version'], text=True),
                       packages=subprocess.check_output(['dpkg-query', '-W', '-f=${Package} ${Version}\n',
                                                         'ros-jazzy-rmw-fastrtps-cpp', 'libeigen3-dev',
                                                         'python3-scipy', 'cmake'], text=True))
    for filename in ('/proc/cpuinfo', '/proc/meminfo', '/proc/self/cgroup',
                     '/sys/fs/cgroup/cpu.max', '/sys/fs/cgroup/cpu.stat',
                     '/sys/fs/cgroup/cpuset.cpus.effective', '/sys/fs/cgroup/memory.max'):
        path = Path(filename)
        environment[filename] = path.read_text() if path.exists() else 'unavailable'
    host_receipt = os.environ.get('HOPE_HOST_ENVIRONMENT_RECEIPT')
    environment['parent_host_receipt'] = json.loads(Path(host_receipt).read_text()) if host_receipt else 'unavailable'
    freeze = dict(source_commit=git('rev-parse', 'HEAD'), source_tree=git('rev-parse', 'HEAD^{tree}'),
                  contract_sha256=digest(contract_path), contract=contract, source_sha256=before,
                  environment=environment, diagnostic_comparison=args.diagnostic_comparison,
                  binary_sha256=binaries, trials=trials, started_wall_ns=time.time_ns(),
                  policy='Every declared run retained; no retries or pooled passing averages')
    (args.output / 'freeze.json').write_text(json.dumps(freeze, indent=2) + '\n')
    outcomes = []
    for index, trial in enumerate(trials, 1):
        output = Path(trial['output'])
        output.parent.mkdir(parents=True, exist_ok=False)
        started = time.time_ns()
        print(f"START {index}/{len(trials)} rep={trial['repetition']} {trial['profile']}", flush=True)
        with output.with_suffix('.log').open('w') as log:
            process = subprocess.Popen(trial['command'], stdout=log, stderr=subprocess.STDOUT,
                                       env={**os.environ, **trial['environment']}, start_new_session=True)
            timeout = False
            try:
                code = process.wait(timeout=100)
            except subprocess.TimeoutExpired:
                timeout = True
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                code = 124
        required = [output, output.with_suffix('.raw.json'), output.with_suffix('.accounting.json')]
        missing = [str(path) for path in required if not path.exists()]
        result, raw, accounting = {}, {}, {}
        validation_error = None
        metrics = {}
        try:
            result, raw, accounting = [json.loads(path.read_text()) for path in required]
            assert result['status'] == accounting['status'] == 'PASS'
            assert not raw['exception'] and not accounting['exception']
            if trial['stage_accounting']:
                assert accounting['accounting_complete'], 'missing or truncated actual-stage accounting'
            source = raw['source']
            assert source['rows']
            assert all(abs(r['state_elapsed_s'] - (r['stamp_ns'] - source['clock_origin_ns']) * 1e-9)
                       < 1e-12 for r in source['rows']), 'state/acquisition mismatch'
            expected = set(source['eligible_stamps'])
            traces = raw['traces']
            observed = {int(t['source_time_ns']) for t in traces}
            assert expected and traces
            published_count = max(int(status.get('commands', 0)) for status in raw['statuses'])
            traced_sequences = {int(t['command_sequence']) for t in traces}
            wire_sequences = {int(wire[14]) for wire in raw['wire'] if wire[1] == 1.}
            assert published_count == len(traced_sequences), ('incomplete publication trace',
                                                              published_count, len(traced_sequences))
            assert wire_sequences == traced_sequences, 'valid command/trace ledger mismatch'
            coverage = len(expected & observed) / len(expected)
            assert coverage >= contract['gates']['coverage_min'], ('coverage', coverage)
            assert len(expected) == result['command_denominator'] == accounting['independent_denominator']
            publication_ms = [(int(t['publication_time_ns']) - int(t['source_time_ns'])) / 1e6
                              for t in traces]
            assert max(publication_ms) <= contract['gates']['publication_source_age_max_ms']
            by_seq = {int(t['command_sequence']): t for t in traces}
            decoder_ms = [((int(d[13]) * 10**9 + int(d[14])) -
                           int(by_seq[int(d[0])]['source_time_ns'])) / 1e6
                          for d in raw['decoded'] if d[10] == 0 and int(d[0]) in by_seq]
            assert decoder_ms and np.percentile(decoder_ms, 95) <= contract['gates']['decoder_latency_p95_max_ms']
            metrics = dict(coverage=coverage, denominator=len(expected),
                           complete_valid_publication_count=published_count,
                           publication_ms=dict(zip(('p50', 'p95', 'p99', 'max'),
                                                   map(float, np.percentile(publication_ms, [50, 95, 99, 100])))),
                           decoder_ms=dict(zip(('p50', 'p95', 'p99', 'max'),
                                               map(float, np.percentile(decoder_ms, [50, 95, 99, 100])))))
        except (OSError, ValueError, KeyError, AssertionError) as error:
            validation_error = f'{type(error).__name__}: {error}'
        passed = code == 0 and not missing and validation_error is None
        row = dict(profile=trial['profile'], repetition=trial['repetition'], exit=code,
                   status='PASS' if passed else 'FAIL', timed_out=timeout, missing_outputs=missing,
                   receipt_validation_error=validation_error, independent_metrics=metrics,
                   started_wall_ns=started, finished_wall_ns=time.time_ns(), output=str(output),
                   coverage=result.get('command_coverage', result.get('observed_coverage')),
                   denominator=result.get('command_denominator', result.get('independent_denominator')),
                   decoder_ms=result.get('actual_decoder_ms', result.get('decoder_ms')))
        output.with_suffix('.exit').write_text(str(code) + '\n')
        outcomes.append(row)
        with (args.output / 'outcomes.jsonl').open('a') as stream:
            stream.write(json.dumps(row) + '\n')
        print('RESULT ' + json.dumps(row), flush=True)
    equivalent = (before == source_hashes(args.repo) and not git('status', '--porcelain') and
                  freeze['source_commit'] == git('rev-parse', 'HEAD') and
                  all(digest(Path(path)) == sha for path, sha in binaries.items()))
    passed = equivalent and len(outcomes) == len(trials) and all(row['status'] == 'PASS' for row in outcomes)
    summary = dict(status='PASS' if passed else 'FAIL', candidate_unchanged=equivalent,
                   source_commit=freeze['source_commit'], expected_trials=len(trials),
                   executed_trials=len(outcomes), failed_trials=[r for r in outcomes if r['status'] != 'PASS'],
                   outcomes=outcomes)
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
