#!/usr/bin/env python3
"""Exercise the complete C++ replay→matching→tables/figures path on labelled synthetic input."""
import argparse
import json
from pathlib import Path
import sys
import tempfile

sys.path.insert(0,str(Path(__file__).parents[1]/'scripts/flight_prediction_validation'))
from accuracy import METHODS, write_report
from compare_recordings import compare


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--baseline',required=True)
    parser.add_argument('--v2',required=True)
    parser.add_argument('--output')
    args=parser.parse_args()
    origin=20_000_000_000
    observations=[dict(source_ns=origin+round(i*1e9/150),position=[1.8-3*i/150,-.7,.5-4.905*(i/150)**2]) for i in range(45)]
    dataset=dict(frame_id='table',clock_mapping_id='synthetic',registration_receipt_sha256='0'*64,
                 independent_truth=False,code_pins={m:'0'*40 for m in METHODS},flights=[
                     dict(session_id='synthetic',ball_id='fixture',flight_id='one',direction=-1,
                          shot_type='OPPONENT_RALLY',split='held_out',complete_history=True,
                          b1_ns=origin+313000000,observations=observations,truth_events=[
                              dict(event_id='plane',kind='receiving_plane',position=[.15,-.7,.1],velocity=[-2.,0.,-1.],arrival_ns=origin+550000000),
                              dict(event_id='B1',kind='table_bounce',ordinal=1,position=[.85,-.7,.02],velocity=[-2.8,0.,-2.8],post_velocity=[-2.2,0.,2.5],arrival_ns=origin+313000000)])])
    rows=compare(dataset,args.v2,args.baseline,synthetic=True)
    assert len(rows)==45*3*2
    actual=[r for r in rows if r['method']=='legacy_one_shot' and r['attempted']]
    epochs={r['information_cutoff_ns'] for r in actual}
    assert len(epochs)==1 and origin+190_000_000<=next(iter(epochs))<=origin+210_000_000
    def check(output):
        report=write_report(rows,output,synthetic=True,repetitions=20)
        assert report['gate_p']=='NOT_RUN_SYNTHETIC_TOOLING'
        assert len(list(Path(output).glob('*/*.png')))==6
        assert len(report['paired_error_comparisons'])==3
        assert (Path(output)/'errors.csv').stat().st_size>100
        return report
    if args.output:
        report=check(args.output)
    else:
        with tempfile.TemporaryDirectory(prefix='hope-matched-tooling-') as directory:
            report=check(directory)
    print(json.dumps(dict(status='PASS',gate_p=report['gate_p'],matched_rows=len(rows),actual_one_shot_epochs=1,figures=6)))


if __name__=='__main__': main()
