#!/usr/bin/env python3
"""Freeze supplied physical launches into six reproducible, hash-bound shards."""
import argparse
import json
from pathlib import Path
import random
from common import load_dataset, sha256


def freeze(samples, output, seed):
    import yaml
    rows = [dict(row) for row in samples]
    if not rows or len(rows) % 6:
        raise ValueError('provide a nonzero multiple of six physical launch records')
    identities = [tuple(row['launch']+row['initial_spin']) for row in rows]
    if len(set(identities)) != len(identities):
        raise ValueError('duplicate physical launch/spin combinations')
    random.Random(seed).shuffle(rows)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    n = len(rows)//6
    for shard in range(6):
        part = rows[shard*n:(shard+1)*n]
        for index, row in enumerate(part):
            row.update(shard_id=shard+1, shot_id=index+1)
        scene = {'balls':n, 'clean_serves':[r['launch'] for r in part[:6]], 'rapid_cycle':[r['launch'] for r in part[6:]], 'spin_rad_s_list':[r['initial_spin'] for r in part], 'timing':{'flight_s_list':[r['flight_duration_s'] for r in part], 'pause_s_list':[r.get('pause_s',0.) for r in part]}}
        (output/f'shard-{shard+1:02d}.yaml').write_text(yaml.safe_dump(scene, sort_keys=True))
        (output/f'shard-{shard+1:02d}-ledger.json').write_text(json.dumps(part,indent=2,allow_nan=False)+'\n')
    design = {'version':'custom-supergate-v1','seed':seed,'shards':6,'total_unique':len(rows),'policy_outcome_used_to_select':False,'scope':'Caller-supplied physical launch states. Freezing validates format and identity, not geometric feasibility or policy qualification.'}
    (output/'design.json').write_text(json.dumps(design,indent=2)+'\n')
    manifest = {p.name:sha256(p) for p in sorted(output.iterdir()) if p.is_file()}
    (output/'SHA256.json').write_text(json.dumps(manifest,indent=2)+'\n')
    load_dataset(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--samples',type=Path,required=True,help='JSON list: launch[6], initial_spin[3], flight_duration_s, optional pause_s')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seed',type=int,default=2026110801)
    args = parser.parse_args()
    freeze(json.loads(args.samples.read_text()),args.output,args.seed)


if __name__ == '__main__':
    main()
