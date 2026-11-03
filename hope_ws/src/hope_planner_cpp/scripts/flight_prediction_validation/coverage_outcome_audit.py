"""Independent per-flight outcome audit; original per-epoch verdicts are retained."""

import argparse, csv, gzip, json, math, re, sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'test'))
from integrated_producer_chain import flight


def stream(path):
    if not path.exists():
        path = path.with_name(path.name + '.gz')
    return gzip.open(path, 'rt') if path.suffix == '.gz' else path.open()


def read(path):
    with stream(path) as file:
        return json.load(file)


def stages(path):
    with stream(path) as file:
        metadata = file.readline()
        assert 'overwritten=0' in metadata
        return list(csv.DictReader(file, delimiter='\t'))


def audit(profile, path, incoming=False):
    raw = read(path)
    src = raw['source']
    source_rows = src['rows']
    by_source = {int(row.get('source_ns', row.get('stamp_ns'))): row for row in source_rows}
    if incoming:
        received = raw['received']
        wire = received['commands']
        decoded = received['decoded']
        traces = stages(path.parent / 'planner.tsv')
        commands = [row for row in traces if row['stage'] == 'command_published']
        expected = set(range(1, 11))

        def ordinal(row):
            return row['flight'] if row['present'] else None

        def truth(row):
            return np.array(src['contact_state'][:3]), (
                src['origin_ns'] * 1e-9 + 0.5 + (row['flight'] - 1) * 0.9 + src['crossing_s']
            )

    else:
        wire = raw['wire']
        decoded = raw['decoded']
        commands = raw['traces']
        reacquisition = 'reacquisition' in profile
        if reacquisition:

            def ordinal(row):
                return row['round'] if row['opportunity'] == 'B' else None

            expected = set(range(10))
            speed = 10
            spin = profile.rsplit('_', 1)[-1]
        else:

            def ordinal(row):
                elapsed = row['state_elapsed_s'] - 0.8
                return math.floor(elapsed) if elapsed >= 0 else None

            count = re.search(r'first_(\d+)_flights', src['command_epoch_contract'])
            assert count, src['command_epoch_contract']
            expected = set(range(int(count[1])))
            match = re.search(r'review_(?:chain|refresh)_\d+_(\d+)_(\w+)', profile)
            speed = int(match[1]) if match else 5
            spin = match[2] if match else 'unknown'
        before, after, bounce = flight(
            speed, 'zero' if spin == 'zero' else 'unknown' if spin == 'unknown' else 'known'
        )
        crossing = src.get('plane_time_s', src.get('plane_s'))
        contact = after.sol(crossing)[:3]

        def truth(row):
            launch = 0.8 + row['round'] * 1.8 + 0.7 if reacquisition else 0.8 + ordinal(row)
            return contact, src['clock_origin_ns'] * 1e-9 + launch + crossing

    wires = {int(row[14]): row for row in wire if row[1] == 1}
    decoders = {}
    for row in decoded:
        if row[10] == 0:
            decoders.setdefault(int(row[0]), []).append(row)
    outcomes = {i: [] for i in sorted(expected)}
    for command in commands:
        source_ns = int(command['source_ns'] if incoming else command['source_time_ns'])
        if source_ns not in by_source:
            continue
        source = by_source[source_ns]
        i = ordinal(source)
        if i not in outcomes:
            continue
        sequence = int(command['sequence'] if incoming else command['command_sequence'])
        if sequence not in wires:
            continue
        packet = wires[sequence]
        position, arrival = truth(source)
        error = float(np.linalg.norm(np.array(packet[3:6]) - np.array([0, 0, 0.76]) - position))
        time_error = abs(packet[10] - arrival)
        matches = decoders.get(sequence, [])
        conversion_ok = bool(matches) and all(
            np.allclose(row[3:9], packet[3:9], atol=1e-8, rtol=0) for row in matches
        )
        lead = max((row[9] for row in matches), default=0)
        outcomes[i].append(
            dict(
                sequence=sequence,
                source_ns=source_ns,
                position_error_m=error,
                contact_time_error_ms=time_error * 1000,
                decoder_matches=conversion_ok,
                receiver_lead_s=lead,
                usable=conversion_ok and lead >= 0.08 and error <= 0.02 and time_error <= 0.01,
            )
        )
    return dict(
        profile=profile,
        raw_path=str(path),
        expected_flights=len(expected),
        flights=[
            dict(
                flight=i,
                commands=len(rows),
                usable_commands=sum(r['usable'] for r in rows),
                first_usable=next((r for r in rows if r['usable']), None),
                max_position_error_m=max((r['position_error_m'] for r in rows), default=None),
                max_contact_time_error_ms=max(
                    (r['contact_time_error_ms'] for r in rows), default=None
                ),
            )
            for i, rows in outcomes.items()
        ],
        assessment=(
            'CONFIRMED_DEFECT'
            if any(not any(r['usable'] for r in rows) for rows in outcomes.values())
            else 'USABLE_COMMANDS_PRESENT_REQUIRES_LIFECYCLE_AND_SIMULATION_REVIEW'
        ),
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence-root', type=Path, required=True)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text())
    results = []
    for case in manifest['cases']:
        result = audit(
            case['profile'], args.evidence_root / case['raw_path'], case.get('incoming', False)
        )
        result['run'] = case['run']
        results.append(result)
    report = dict(
        scope='Per-flight usefulness only; original verdicts preserved; no lifecycle proof inferred',
        acceptance='v2_simulation_acceptance.json; receiver lead >= existing integrated .08 s minimum',
        cases=len(results),
        results=results,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    for row in results:
        print(
            row['profile'],
            row['run'],
            sum(r['usable_commands'] > 0 for r in row['flights']),
            '/',
            row['expected_flights'],
            row['assessment'],
        )
    return 1 if any(r['assessment'] == 'CONFIRMED_DEFECT' for r in results) else 0


if __name__ == '__main__':
    raise SystemExit(main())
