"""Portable validation and reporting for the six-shard Supergate protocol."""
import csv
import hashlib
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPOSITORY = HERE.parents[3]


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_dataset(directory, count=None):
    import numpy as np
    import yaml
    directory = Path(directory).resolve()
    manifest = json.loads((directory/'SHA256.json').read_text())
    required = {'design.json'} | {f'shard-{i:02d}{suffix}' for i in range(1, 7) for suffix in ('.yaml', '-ledger.json')}
    if not required.issubset(manifest):
        raise ValueError('dataset manifest must bind design and all six YAML/ledger pairs')
    for name, digest in manifest.items():
        path = (directory/name).resolve()
        if not path.is_relative_to(directory) or sha256(path) != digest:
            raise ValueError(f'dataset integrity failure: {name}')
    result, lengths = [], []
    for shard in range(1, 7):
        scene = yaml.safe_load((directory/f'shard-{shard:02d}.yaml').read_text())
        ledger = json.loads((directory/f'shard-{shard:02d}-ledger.json').read_text())
        timing = scene['timing']
        n = len(ledger)
        lengths.append(n)
        launches = scene['clean_serves'] + scene['rapid_cycle']
        if not (scene['balls'] == len(launches) == len(scene['spin_rad_s_list']) == len(timing['flight_s_list']) == len(timing['pause_s_list']) == n):
            raise ValueError(f'shard {shard}: inconsistent scene/ledger lengths')
        rows = []
        for index, row in enumerate(ledger):
            launch, spin = row['launch'], row['initial_spin']
            if len(launch) != 6 or len(spin) != 3 or launch != launches[index] or spin != scene['spin_rad_s_list'][index]:
                raise ValueError(f'shard {shard}: launch/spin mismatch at {index}')
            values = [*launch, *spin, timing['flight_s_list'][index], timing['pause_s_list'][index]]
            if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in values):
                raise ValueError('non-finite or non-numeric dataset value')
            if values[-2] <= 0 or values[-1] < 0:
                raise ValueError('flight duration must be positive and pause nonnegative')
            rows.append(values)
        result.append(rows)
    if len(set(lengths)) != 1 or not lengths[0]:
        raise ValueError('six nonempty equally sized shards are required')
    count = lengths[0] if count is None else count
    if isinstance(count, bool) or count < 1 or count > lengths[0]:
        raise ValueError(f'cases must be between 1 and {lengths[0]}')
    return np.ascontiguousarray([rows[:count] for rows in result], dtype=np.float64)


def summarize(output, labels, count):
    summaries = {}
    for group, label in enumerate(labels):
        rows = []
        for shard in range(6):
            path = Path(output)/f'env-{6*group+shard}-results.csv'
            with path.open() as stream:
                part = list(csv.DictReader(stream))
            if len(part) != count or [int(row['shot']) for row in part] != list(range(1, count+1)):
                raise ValueError(f'incomplete or duplicate trial results: {path.name}')
            rows.extend(part)
        def stats(values):
            return {
                'trials': len(values),
                'contact_trials': sum(int(r['contacts']) > 0 for r in values),
                'legal_returns': sum(int(r['legal']) for r in values),
                'valid_plan_trials': sum(int(r['valid_plans']) > 0 for r in values),
                'naturally_completed_trials': sum(int(r['complete']) for r in values),
                'fall_indicator_trials': sum(float(r['min_z']) < .7 or float(r['max_tilt']) > math.pi/4 for r in values),
            }
        summaries[label] = stats(rows)
        summaries[label]['by_actual_swing_sign'] = {str(sign): stats([r for r in rows if float(r['side']) == sign]) for sign in (-1, 0, 1)}
    return summaries
