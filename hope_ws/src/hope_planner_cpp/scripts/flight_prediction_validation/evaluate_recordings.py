#!/usr/bin/env python3
"""Score explicitly matched, independently annotated flights with flight-level resampling."""
import argparse
import hashlib
import json
from collections import defaultdict
import numpy as np
from scoring import score_matched_events


def evaluate(records, repetitions=500):
    """Compare methods at declared source cutoffs; never split adjacent frames as flights."""
    by_method = defaultdict(list)
    for row in records:
        for field in ('session_id', 'ball_id', 'flight_id', 'method', 'spin_mode'):
            if field not in row:
                raise ValueError(f'missing physical-data provenance: {field}')
        by_method[row['method']].append(row)
    results = {}
    rng = np.random.default_rng(0)
    for method, rows in by_method.items():
        flights = defaultdict(list)
        for row in rows:
            flights[(row['session_id'], row['ball_id'], row['flight_id'])].append(row)
        keys = list(flights)
        coverage = []
        for _ in range(repetitions):
            chosen = rng.integers(0, len(keys), len(keys))
            sample = [row for index in chosen for row in flights[keys[index]]]
            coverage.append(sum(bool(row['target_valid']) for row in sample) / len(sample))
        results[method] = dict(spin_groups=score_matched_events(rows), physical_flights=len(keys),
                               target_coverage_flight_bootstrap_95=[float(x) for x in
                                  np.quantile(coverage, [.025, .975])] if coverage else None)
    return results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--records', required=True,
                        help='JSONL matched by physical flight/event and source-information cutoff')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    with open(args.records, 'rb') as source:
        raw = source.read()
    records = [json.loads(line) for line in raw.splitlines() if line.strip()]
    if not records:
        raise ValueError('recordings/annotations are empty')
    result = dict(input_sha256=hashlib.sha256(raw).hexdigest(),
                  matching='explicit physical-flight/event/source-cutoff annotations',
                  uncertainty='conditional model coverage; not a calibration certificate',
                  methods=evaluate(records))
    with open(args.output, 'w', encoding='utf-8') as output:
        json.dump(result, output, indent=2)
        output.write('\n')


if __name__ == '__main__':
    main()
