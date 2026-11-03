#!/usr/bin/env python3
"""Freeze and run five repetitions of all 18 incoming producer-chain profiles."""
import argparse
import json
import os
import signal
import subprocess
import time
from pathlib import Path

from qualification_campaign import digest, source_hashes


def main():
    """Keep every declared run and reject any changed source or binary."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', type=Path, required=True)
    parser.add_argument('--build', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)

    def git(*values):
        return subprocess.check_output(['git', '-C', str(args.repo), *values], text=True).strip()

    assert not git('status', '--porcelain'), 'qualification requires clean committed source'
    registered = json.loads(subprocess.check_output(
        ['ctest', '--test-dir', str(args.build), '--show-only=json-v1'], text=True))
    tests = {test['name']: test for test in registered['tests']
             if test['name'].startswith('incoming_chain_')}
    assert len(tests) == 18
    trials = []
    for repetition in range(1, 6):
        for name, test in sorted(tests.items()):
            command = test['command'].copy()
            output = args.output / f'repetition-{repetition}' / name
            command[command.index('--output') + 1] = str(output)
            environment = {}
            for prop in test.get('properties', []):
                if prop['name'] == 'ENVIRONMENT':
                    environment.update(item.split('=', 1) for item in prop['value'])
            trials.append({'profile': name, 'repetition': repetition, 'command': command,
                           'output': str(output), 'environment': environment})
    before = source_hashes(args.repo)
    binaries = {str(args.build / name): digest(args.build / name) for name in
                ('hope_planner_cpp_node', 'hope_ball_flight_packetizer',
                 'hope_integrated_runner_probe')}
    freeze = {'source_commit': git('rev-parse', 'HEAD'), 'source_sha256': before,
              'binary_sha256': binaries, 'trials': trials, 'started_ns': time.time_ns(),
              'policy': '90 fixed trials, no retries or pooled passing averages; '
                        'each freezes 10 flights and exact continuous source opportunities'}
    (args.output / 'freeze.json').write_text(json.dumps(freeze, indent=2) + '\n')
    outcomes = []
    for index, trial in enumerate(trials, 1):
        output = Path(trial['output'])
        output.parent.mkdir(parents=True, exist_ok=True)
        print(f'START {index}/90 {output.name}', flush=True)
        with output.with_suffix('.log').open('w') as log:
            process = subprocess.Popen(trial['command'], stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True,
                                       env={**os.environ, **trial['environment']})
            try:
                code = process.wait(timeout=60)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                code = 124
        error, result = None, {}
        try:
            result = json.loads((output / 'result.json').read_text())
            assert code == 0 and result['status'] == 'PASS'
            for required in ('raw.json', 'accounting.json', 'contract.json',
                             'synthetic-accuracy.json', 'relay.tsv', 'planner.tsv', 'decoder.tsv'):
                assert (output / required).is_file(), required
        except (OSError, ValueError, KeyError, AssertionError) as failure:
            error = repr(failure)
        outcome = {'profile': trial['profile'], 'repetition': trial['repetition'], 'exit': code,
                   'status': 'FAIL' if error else 'PASS', 'error': error, 'result': result}
        output.with_suffix('.exit').write_text(str(code) + '\n')
        outcomes.append(outcome)
        with (args.output / 'outcomes.jsonl').open('a') as stream:
            stream.write(json.dumps(outcome) + '\n')
        print(f"RESULT {index}/90 {outcome['status']}", flush=True)
    unchanged = (before == source_hashes(args.repo) and not git('status', '--porcelain')
                 and freeze['source_commit'] == git('rev-parse', 'HEAD')
                 and all(digest(Path(path)) == sha for path, sha in binaries.items()))
    passed = unchanged and all(row['status'] == 'PASS' for row in outcomes)
    summary = {'status': 'PASS' if passed else 'FAIL', 'candidate_unchanged': unchanged,
               'expected_trials': 90, 'executed_trials': len(outcomes),
               'failed_trials': [row for row in outcomes if row['status'] != 'PASS'],
               'outcomes': outcomes}
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
