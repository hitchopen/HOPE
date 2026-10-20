#!/usr/bin/env python3
"""Replay public HOPE methods at the same source cutoffs and score independent truth."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
from accuracy import METHODS, write_report

REQUIRED = ['independent position/event truth and occurrence uncertainty',
            'session/ball/physical-flight IDs and bounce annotations',
            'camera acquisition clock mapping and table/plane registration receipts',
            'both directions/serve contacts and speed/spin/dropout/position-only groups',
            'pinned baseline/V2 source and effective configuration, held-out split']


def replay(executable, args, csv):
    process=subprocess.run([executable,*args],input=csv,text=True,capture_output=True,check=True)
    return {row['source_ns']:row for line in process.stdout.splitlines() if line.strip()
            for row in [json.loads(line)]}


def compare(dataset, v2_executable, baseline_executable, synthetic=False):
    for field in ('frame_id','clock_mapping_id','registration_receipt_sha256','code_pins','flights'):
        if field not in dataset:
            raise ValueError('missing dataset provenance: '+field)
    if dataset['frame_id']!='table' or not dataset['clock_mapping_id']:
        raise ValueError('registered table/source-clock coordinates required')
    if not synthetic and not dataset.get('independent_truth',False):
        raise ValueError('same-sensor/proxy data cannot qualify independent real-ball accuracy')
    rows=[]
    for flight in dataset['flights']:
        observations=flight['observations']
        stamps=[r['source_ns'] for r in observations]
        if not stamps or any(b<=a for a,b in zip(stamps,stamps[1:])):
            raise ValueError('acquisition stamps must be strictly ordered')
        if flight.get('split')!='held_out' and not synthetic:
            raise ValueError('physical scoring requires a held-out whole-flight/session split')
        csv=''.join(','.join(str(v) for v in [r['source_ns'],*r['position'],int(r.get('omega_valid',False)),*r.get('omega_rad_s',[0,0,0])])+'\n' for r in observations)
        predictions={'v2_ckf':replay(v2_executable,['--observations',str(flight['direction']),
                       'complete' if flight.get('complete_history',False) else 'unknown'],csv),
                     'legacy_one_shot':replay(baseline_executable,['--method','one-shot'],csv),
                     'progressive_batch':replay(baseline_executable,['--method','progressive'],csv)}
        common_spin={}; held=False
        for observation in observations:
            angular=observation.get('omega_rad_s',[0,0,0])
            measured=observation.get('omega_valid',False) and all(__import__('math').isfinite(v) for v in angular) and any(v!=0 for v in angular)
            held=held or measured
            common_spin[observation['source_ns']]=observation.get('spin_mode', 'MEASURED' if measured else 'HELD' if held else 'POSITION_ONLY')
        for event in flight['truth_events']:
            for cutoff in flight.get('information_cutoffs_ns',stamps):
                if cutoff not in stamps:
                    raise ValueError('matched cutoff must identify an actual shared acquisition, never an interpolated baseline')
                if event.get('arrival_ns') is not None and cutoff>event['arrival_ns']:
                    continue  # Future-event eligibility ends at independently annotated occurrence.
                for method in METHODS:
                    p=predictions[method].get(cutoff)
                    kind=event['kind']
                    supported=p is not None and kind=='receiving_plane' and (method!='v2_ckf' or p.get('target_kind')==kind)
                    valid=bool(supported and p['target_valid'])
                    row=dict(session_id=flight['session_id'],ball_id=flight['ball_id'],flight_id=flight['flight_id'],
                             shot_type=flight.get('shot_type','unknown'),event_id=event['event_id'],event_kind=kind,method=method,
                             code_sha=dataset['code_pins'][method],config_id=p.get('model_config_id','not_triggered') if p else 'not_triggered',
                             frame_id=dataset['frame_id'],clock_mapping_id=dataset['clock_mapping_id'],
                             spin_mode=common_spin[cutoff],model_spin_mode=p.get('spin_mode','POSITION_ONLY') if p else 'UNAVAILABLE',
                             independent_truth=dataset.get('independent_truth',False),truth_available=event.get('truth_available',True),
                             information_cutoff_ns=cutoff,prediction_reference_time_ns=cutoff if p else None,
                             attempted=p is not None,actual_trigger=p.get('actual_trigger',False) if p else False,
                             physical_available=p.get('physical_available',False) if p else False,target_valid=valid,
                             trajectory_available=bool(p and method=='v2_ckf' and p.get('physical_available')),
                             reason=(p.get('reason','unavailable') if supported else 'unsupported_event' if p else 'not_triggered'),
                             position=p.get('position') if p else None,velocity=p.get('velocity') if p else None,
                             arrival_ns=p.get('arrival_ns') if p else None,covariance=p.get('covariance') if valid else None,
                             compute_ms=p.get('compute_ms') if p else None,b1_ns=flight.get('b1_ns'),b2_ns=flight.get('b2_ns'))
                    for name in ('position','velocity','arrival_ns'):
                        if name in event: row['truth_'+name]=event[name]
                    rows.append(row)
    return rows


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--dataset')
    parser.add_argument('--v2-executable')
    parser.add_argument('--baseline-executable')
    parser.add_argument('--output',required=True)
    parser.add_argument('--allow-synthetic',action='store_true',help='Tooling tests only; never Gate P qualification')
    args=parser.parse_args()
    output=Path(args.output);output.mkdir(parents=True,exist_ok=True)
    if not args.dataset:
        receipt=dict(gate_p='NOT_RUN_INDEPENDENT_DATA_UNAVAILABLE',dataset_ids=[],required_inputs=REQUIRED,
                     figures=['01_plane_error_vs_lead.png','02_bounce_aligned_error.png','03_coverage_latency_uncertainty.png'],
                     status='Evaluation and plotting tools are executable; no physical scores fabricated.')
        (output/'NOT_RUN.json').write_text(json.dumps(receipt,indent=2)+'\n')
        print(json.dumps(receipt));return 2
    if not args.v2_executable or not args.baseline_executable:
        parser.error('both public HOPE replay executables are required with --dataset')
    raw=Path(args.dataset).read_bytes();dataset=json.loads(raw)
    rows=compare(dataset,args.v2_executable,args.baseline_executable,args.allow_synthetic)
    (output/'matched.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    summary=write_report(rows,output,args.allow_synthetic)
    manifest=dict(dataset_sha256=hashlib.sha256(raw).hexdigest(),registration=dataset['registration_receipt_sha256'],
                  code_pins=dataset['code_pins'],input_flights=[(r['session_id'],r['ball_id'],r['flight_id']) for r in dataset['flights']],
                  executables={key:hashlib.sha256(Path(path).read_bytes()).hexdigest() for key,path in
                               [('v2',args.v2_executable),('baselines',args.baseline_executable)]},gate_p=summary['gate_p'])
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(manifest));return 0


if __name__=='__main__':
    raise SystemExit(main())
