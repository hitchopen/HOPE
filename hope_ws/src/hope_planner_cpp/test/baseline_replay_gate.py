#!/usr/bin/env python3
"""Check actual one-shot epochs, progressive causality, and rejected-spin CSV parity."""
import argparse
import json
import subprocess


def run(executable, args, rows):
    return [json.loads(line) for line in subprocess.check_output([executable,*args],input=''.join(rows),text=True).splitlines()]


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--baseline',required=True)
    parser.add_argument('--v2',required=True)
    args=parser.parse_args()
    rows=[f'{20_000_000_000+round(i*1e9/150)},{1.8-3*i/150},-.7,{.5-4.905*(i/150)**2},0,0,0,0\n' for i in range(45)]
    progressive=run(args.baseline,['--method','progressive'],rows)
    one=run(args.baseline,['--method','one-shot'],rows)
    assert len(progressive)==len(rows)
    assert len(one)==1 and one[0]['actual_trigger']
    assert 20_190_000_000 <= one[0]['source_ns'] <= 20_210_000_000
    assert run(args.baseline,['--method','one-shot'],rows[:20])==[]
    assert all(p['source_ns'] in [int(row.split(',')[0]) for row in rows] for p in one)
    profile=run(args.v2,['--profile'],[])[0]
    override=run(args.v2,['--profile','--set','physics.drag_k=0.15'],[])[0]
    assert profile['model_config_id']!=override['model_config_id']
    # A rejected nonzero or NaN vector cannot alter the physics/cache at fixed XYZ.
    measured=[row.rsplit(',',4)[0]+',1,0,30,0\n' if i==0 else row for i,row in enumerate(rows)]
    first=run(args.v2,['--observations','-1','complete'],measured)
    mutated=[row if i==0 else ','.join(row.strip().split(',')[:4])+',0,nan,999,-777\n' for i,row in enumerate(measured)]
    second=run(args.v2,['--observations','-1','complete'],mutated)
    for a,b in zip(first,second):
        a.pop('compute_ms',None);b.pop('compute_ms',None)
        assert a==b, 'rejected angular field changed a causal forecast'
    print(json.dumps(dict(status='PASS',progressive_epochs=len(progressive),actual_one_shot_epochs=len(one),
                          reference_ns=one[0]['source_ns'],synthetic_tooling_only=True)))


if __name__=='__main__': main()
