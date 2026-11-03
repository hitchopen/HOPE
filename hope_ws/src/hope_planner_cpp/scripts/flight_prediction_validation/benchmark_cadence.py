#!/usr/bin/env python3
"""Measure supported full-output choices independently; never waive the selected gate."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--executable',required=True)
    parser.add_argument('--smoke',required=True)
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    output=Path(args.output);output.mkdir(parents=True,exist_ok=True)
    profiles=[]
    for cadence in (50,100,150,0):
        for rate in (150,300):
            process=subprocess.run([sys.executable,args.smoke,'--executable',args.executable,
                                    '--full-rate',str(cadence),'--rates',str(rate)],text=True,capture_output=True)
            (output/f'full-{cadence}-input-{rate}.log').write_text(process.stdout+process.stderr)
            metrics=None
            for line in process.stdout.splitlines():
                if line.startswith('{"rate_hz"'):
                    metrics=json.loads(line)
            profiles.append(dict(full_rate_hz=cadence,input_rate_hz=rate,exit_code=process.returncode,
                                 status='PASS' if process.returncode==0 else 'FAIL',metrics=metrics))
    selected=[p for p in profiles if p['full_rate_hz']==50]
    receipt=dict(executable_sha256=hashlib.sha256(Path(args.executable).read_bytes()).hexdigest(),
                 selected_full_rate_hz=50,selected_profile_pass=all(p['status']=='PASS' for p in selected),
                 profiles=profiles,
                 interpretation='50 Hz retained as the conservative full-output default. Optional profiles are measured, not universally qualified. Compact source-stamp availability >=95% and <=30 ms publication freshness remain unchanged. CDR rates exclude wire headers.')
    (output/'cadence.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print(json.dumps({k:v for k,v in receipt.items() if k!='profiles'}))
    return 0 if receipt['selected_profile_pass'] else 1


if __name__=='__main__': raise SystemExit(main())
