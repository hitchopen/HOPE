#!/usr/bin/env python3
"""Verify prefix-only equivalence, exact source stamps and malformed-input rejection."""
import argparse
import json
import subprocess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--executable', required=True)
    args = parser.parse_args()
    cases = 0
    for rate in (150, 300):
        for direction in (-1, 1):
            for origin in (-500000000, 0, 1792412345678901234):
                rows = []
                stamps = []
                for i in range(80):
                    time = i / rate
                    stamp = origin + round(time * 1e9) + (i % 3) * 100
                    stamps.append(stamp)
                    spin_valid = int(i >= 40)
                    rows.append(f'{stamp},{1.5 + direction * 2 * time},-.7,'
                                f'{3 + .5 * time - 4.905 * time * time},'
                                f'{spin_valid},0,30,0\n')
                command = [args.executable, '--observations', str(direction), 'complete']
                full = subprocess.check_output(command, input=''.join(rows), text=True).splitlines()
                prefix = subprocess.check_output(command, input=''.join(rows[:40]), text=True).splitlines()
                def physical_output(line):
                    value=json.loads(line);value.pop('compute_ms',None);return value
                assert [physical_output(r) for r in full[:40]] == [physical_output(r) for r in prefix], 'later spin/samples changed an earlier output'
                assert [json.loads(row)['source_ns'] for row in full] == stamps
                duplicate = subprocess.run(command, input=rows[0] + rows[0], text=True,
                                           capture_output=True)
                assert duplicate.returncode != 0 and 'duplicate' in duplicate.stderr
                cases += 1
    print(json.dumps(dict(status='PASS', causal_prefix_cases=cases, rates=[150, 300],
                          clocks=['negative_epoch', 'zero_epoch', 'exact_unix_ns'])))


if __name__ == '__main__':
    main()
