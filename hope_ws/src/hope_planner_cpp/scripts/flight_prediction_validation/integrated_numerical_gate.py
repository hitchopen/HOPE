#!/usr/bin/env python3
"""Independent DOP853 outgoing event and analytic paddle reference (no C++ helpers)."""
import argparse
from collections import Counter
import csv
import io
import json
from pathlib import Path
import subprocess

import numpy as np
from scipy.integrate import solve_ivp


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rows = list(csv.DictReader(io.StringIO(subprocess.check_output([args.executable], text=True))))
    errors, times, impacts = [], [], []
    for row in rows:
        if row['valid'] != '1':
            continue
        initial = np.array([.15, float(row['y']), float(row['z'])])
        incoming = np.array([float(row['vx']), .2, float(row['vz'])])
        racket = np.array([float(row[k]) for k in ['rx', 'ry', 'rz']])
        normal = np.array([float(row[k]) for k in ['nx', 'ny', 'nz']])
        outgoing = incoming-(1+.654)*np.dot(incoming-racket, normal)*normal
        expected_outgoing = np.array([float(row[k]) for k in ['voutx', 'vouty', 'voutz']])
        closure = np.linalg.norm(outgoing-expected_outgoing)
        assert closure <= 1e-8
        def rhs(_, state):
            v = state[3:]
            return np.r_[v, np.array([0., 0., -9.81])-.12861635416914574*np.linalg.norm(v)*v]
        def landing(_, state):
            return state[2]-.02
        landing.direction = -1
        landing.terminal = True
        def net(_, state):
            return state[0]-1.37
        net.direction = 1
        solved = solve_ivp(rhs, [0, 1.5], np.r_[initial, outgoing],
                           method='DOP853', rtol=2e-12, atol=2e-14, events=[landing, net], dense_output=True)
        assert len(solved.t_events[0]) == 1
        assert len(solved.t_events[1]) == 1
        t = solved.t_events[0][0]
        position = solved.y_events[0][0, :3]
        expected = np.array([float(row[k]) for k in ['lx', 'ly', 'lz']])
        error = float(np.linalg.norm(position-expected))
        time_error = abs(t-float(row['lt']))
        assert error <= .001 and time_error <= .0005, row
        assert np.linalg.norm(position-[2.055, -.7625, .02]) <= .001
        assert solved.y_events[1][0, 2] > .1725
        errors.append(error)
        times.append(time_error)
        impacts.append(float(closure))
    assert len(errors) >= 100, 'insufficient accepted envelope coverage'
    report = {'status': 'PASS', 'total_candidates': len(rows), 'accepted': len(errors),
              'rejections': dict(Counter(r['reason'] for r in rows if r['valid'] != '1')),
              'reference': 'independent scipy DOP853 rtol=2e-12 atol=2e-14 + analytic normal impact',
              'max_position_error_m': max(errors), 'max_event_time_error_s': max(times),
              'max_impact_closure_mps': max(impacts), 'physical_accuracy': 'NOT_RUN'}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report))


if __name__ == '__main__':
    main()
