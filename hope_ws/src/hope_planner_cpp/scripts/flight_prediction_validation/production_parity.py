#!/usr/bin/env python3
"""Measure fixed-input legacy command parity and disclosed forward-model changes."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import numpy as np

BASE='8d599e7d21e196275cf426488555e2648e8fd789'


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--repo',required=True)
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    repo=Path(args.repo); planner=repo/'hope_ws/src/hope_planner_cpp'
    sources={name:subprocess.check_output(['git','-C',str(repo),'show',BASE+':hope_ws/src/hope_planner_cpp/src/'+name])
             for name in ['racket_target_planner.cpp','adaptive_flight_commit.cpp','trajectory_predictor.cpp']}
    for name in ['racket_target_planner.cpp','adaptive_flight_commit.cpp']:
        assert (planner/'src'/name).read_bytes()==sources[name], 'out-of-scope production implementation changed: '+name
    with tempfile.TemporaryDirectory(prefix='hope-scope-parity-') as temporary:
        path=Path(temporary)
        for name,raw in sources.items():(path/name).write_bytes(raw)
        results={}
        for label,source in [('upstream',path),('scoped',planner/'src')]:
            executable=path/label
            subprocess.run(['/usr/bin/g++','-std=c++17','-O2','-I'+str(planner/'include'),'-I/usr/include/eigen3',
                            str(planner/'test/production_parity_probe.cpp'),str(source/'racket_target_planner.cpp'),
                            str(source/'trajectory_predictor.cpp'),str(planner/'src/common_ball_physics.cpp'),'-o',str(executable)],check=True)
            results[label]=json.loads(subprocess.check_output([str(executable)],text=True))
    assert results['upstream']['racket']==results['scoped']['racket'], 'fixed-input command parity failed'
    delta=np.asarray(results['scoped']['receiving_flight'])-results['upstream']['receiving_flight']
    receipt=dict(base=BASE,command_fixtures=18,command_max_abs_difference=0.,
                 upstream_sha256={name:hashlib.sha256(raw).hexdigest() for name,raw in sources.items()},
                 source_parity=['racket_target_planner.cpp','adaptive_flight_commit.cpp'],
                 receiving_forward_deltas=delta.tolist(),
                 max_receiving_arrival_delta_s=float(np.max(np.abs(delta[:,1]))),
                 max_receiving_position_delta_m=float(np.max(np.linalg.norm(delta[:,2:5],axis=1))),
                 max_receiving_velocity_delta_mps=float(np.max(np.linalg.norm(delta[:,5:8],axis=1))),
                 note='Fixed ball-input inversion unchanged; end-to-end downstream inputs still change with shared radius-aware RK4 flight physics.')
    Path(args.output).write_text(json.dumps(receipt,indent=2)+'\n')
    print(json.dumps(receipt))


if __name__=='__main__': main()
