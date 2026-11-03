#!/usr/bin/env python3
"""Compare pre-merge V2, with exact declared contact-corroboration differences."""
import argparse
import hashlib
import io
import json
from pathlib import Path
import subprocess
import tarfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', type=Path, required=True)
    parser.add_argument('--replay', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    fixtures = Path(__file__).parent / 'fixtures/pr85_cutoffs'
    manifest = json.loads((fixtures / 'manifest.json').read_text())
    corrections = json.loads((fixtures / 'contact_corroboration_deltas.json').read_text())
    root = args.output.parent / 'premerge-v2'
    root.mkdir(parents=True, exist_ok=True)
    relative = 'hope_ws/src/hope_planner_cpp'
    archive = subprocess.check_output(['git', '-C', str(args.repo), 'archive',
                                      manifest['premerge_head'], relative,
                                      'a3_deploy/a3_deploy_example/src/a3/a3_deploy_onnx_ref/include'])
    with tarfile.open(fileobj=io.BytesIO(archive)) as tar:
        tar.extractall(root / 'source', filter='data')
    build = root / 'build'
    with (root / 'build.log').open('w') as log:
        for command in ([ 'cmake', '-S', str(root / 'source' / relative / 'test/native'),
                          '-B', str(build), '-DCMAKE_BUILD_TYPE=Release'],
                        ['cmake', '--build', str(build), '--target',
                         'hope_flight_prediction_v2_replay', '-j2']):
            subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
    rows = []
    for entry in manifest['fixtures']:
        data = (fixtures / entry['path']).read_bytes()
        assert hashlib.sha256(data).hexdigest() == entry['sha256']
        options = ['--set', 'automatic_rearm_enabled=1'] if entry['rearm'] else []
        outputs = []
        for label, binary in [('premerge', build / 'hope_flight_prediction_v2_replay'),
                              ('integrated', args.replay)]:
            output = subprocess.check_output([str(binary), *options, '--observations'], input=data)
            (root / (entry['path'] + '.' + label + '.jsonl')).write_bytes(output)
            values = [json.loads(line) for line in output.splitlines()]
            for value in values:
                value.pop('compute_ms', None)  # wall duration is not algorithm state
            outputs.append(values)
        assert len(outputs[0]) == len(outputs[1]), entry['path']
        approved = {row['index']: row for row in corrections['fixtures'].get(entry['path'], [])}
        changed = []
        for index, (before, after) in enumerate(zip(*outputs)):
            if before == after:
                continue
            assert index in approved, ('unexpected cutoff change', entry['path'], index)
            expected = approved[index]
            for label, value in [('before', before), ('after', after)]:
                actual = hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()
                assert actual == expected[label + '_sha256'], (entry['path'], index, label)
            # Every exception is conservative, single-cutoff contact withholding.
            # No other state or numerical difference is admitted by this policy.
            assert before['physical_available'] and after['physical_available']
            assert not before['target_valid'] and not after['target_valid']
            assert after['reason'] == 'target_before_required_contact_conditional'
            assert after['arrival_ns'] == 0 and after['position'] is None
            assert outputs[0][index + 1] == outputs[1][index + 1], 'must reconverge next cutoff'
            changed.append(index)
        assert set(changed) == set(approved), ('declared differences changed', entry['path'])
        available = sum(value['physical_available'] for value in outputs[1])
        assert available >= 10, (entry['path'], available)
        if entry['rearm']:
            assert len({value['generation'] for value in outputs[1] if value['physical_available']}) >= 2
        rows.append(dict(fixture=entry['path'], cutoffs=len(outputs[1]),
                         physical_cutoffs=available, exact_match=not changed,
                         exact_cutoffs=len(outputs[1])-len(changed), contact_correction_cutoffs=changed))
    args.output.write_text(json.dumps(dict(status='PASS', premerge=manifest['premerge_head'],
                                          excluded_fields=['compute_ms'], cases=rows,
                                          correction_policy=corrections['reason']), indent=2) + '\n')
    print(json.dumps(rows))


if __name__ == '__main__':
    main()
