#!/usr/bin/env python3
"""Independent simulated ball -> production V2/adapter/planner/decoder audit."""
import argparse
import csv
import hashlib
import io
import itertools
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import brentq

PACKAGE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PACKAGE / 'test'))
from independent_ball_flight import flight


def vector(row, names):
    return np.array([float(row[name]) for name in names.split()])


def outgoing(initial, velocity):
    def rhs(_, state):
        speed = state[3:]
        return np.r_[speed, [0, 0, -9.81] - .12861635416914574*np.linalg.norm(speed)*speed]
    def landed(_, state):
        return state[2] - .02
    landed.direction = -1
    landed.terminal = True
    return solve_ivp(rhs, [0, 1.5], np.r_[initial, velocity], method='DOP853',
                     events=landed, dense_output=True, rtol=2e-12, atol=2e-14)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--probe', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--waypoints', action='store_true',
                        help='Retain actual production forecast waypoints for visualization')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    acceptance = json.loads((PACKAGE / 'docs/v2_simulation_acceptance.json').read_text())
    limits = acceptance['simulation_tolerances']
    epoch = 1800000000000000000
    cases, payload, plants = [], [], {}
    for index, (speed, spin, y, mode, cutoff) in enumerate(itertools.product(
            [5, 10, 12], ['unknown', 'zero', 'known'], [-1.1, -.7, -.3], range(3), [.10, .18])):
        if (speed, spin) not in plants:
            plants[speed, spin] = flight(speed, spin)
        before, after, bounce = plants[speed, spin]
        plane = [.08, .15, .20][mode]
        crossing = brentq(lambda t: after.sol(t)[0]-plane, bounce, .7)
        count = round(cutoff*300)+1
        payload.append(f'{index} {mode} {plane:.17g} {count}\n')
        for sample in range(count):
            t = sample/300
            state = before.sol(t) if t < bounce else after.sol(t)
            payload.append(f'{epoch+round(t*1e9)} {state[0]:.17g} {y:.17g} '
                           f'{state[2]:.17g} {state[6]:.17g} {int(spin!="unknown")}\n')
        truth = after.sol(crossing)[:3]; truth[1] = y
        cases.append(dict(id=index, speed=speed, spin=spin, y=y, mode=mode,
                          cutoff_s=cutoff, plane_x=plane, crossing_s=crossing,
                          truth_position=truth.tolist()))
    data = ''.join(payload)
    (args.output/'observations.txt').write_text(data)
    manifest = dict(cases=cases, acceptance=acceptance,
                    input_sha256=hashlib.sha256(data.encode()).hexdigest(),
                    probe_sha256=hashlib.sha256(args.probe.read_bytes()).hexdigest(),
                    scope='Target/ball simulation and actual C++ decoding; no robot joint dynamics')
    (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    command = [str(args.probe)]
    if args.waypoints:
        command += ['--waypoints', str(args.output / 'waypoints.csv')]
    process = subprocess.run(command, input=data, text=True, capture_output=True)
    (args.output/'probe.csv').write_text(process.stdout)
    (args.output/'probe.stderr').write_text(process.stderr)
    (args.output/'probe.exit').write_text(str(process.returncode)+'\n')
    process.check_returncode()
    rows = list(csv.DictReader(io.StringIO(process.stdout)))
    assert len(rows) == len(cases)
    results = []
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    for case, row in zip(cases, rows):
        assert int(row['id']) == case['id']
        metrics = {}
        checks = {'command_available': row['command_valid']=='1',
                  'publication_authorized': row['publication_allowed']=='1',
                  'actual_decoder_valid': row['decoded']=='1',
                  'preengagement_veto': row['revoked_before_engagement']=='1',
                  'schema': int(row['schema']) == (4 if case['mode']==0 else 5)}
        if checks['command_available']:
            contact = vector(row, 'cx cy cz'); velocity = vector(row, 'cvx cvy cvz')
            racket = vector(row, 'rx ry rz'); racket_v = vector(row, 'rvx rvy rvz')
            normal = vector(row, 'nx ny nz'); out = vector(row, 'ox oy oz')
            metrics['contact_position_error_m'] = float(np.linalg.norm(contact-case['truth_position']))
            metrics['contact_time_error_s'] = abs((int(row['contact_ns'])-epoch)*1e-9-case['crossing_s'])
            metrics['adapter_position_error_m'] = float(np.linalg.norm(contact-vector(row,'sx sy sz')))
            metrics['adapter_velocity_error_mps'] = float(np.linalg.norm(velocity-vector(row,'svx svy svz')))
            metrics['adapter_time_error_s'] = abs(float(row['strike_time'])-int(row['contact_ns'])*1e-9)
            metrics['decoder_position_error_m'] = float(np.linalg.norm(vector(row,'dx dy dz')-racket-[0,0,.76]))
            metrics['decoder_velocity_error_mps'] = float(np.linalg.norm(vector(row,'dvx dvy dvz')-racket_v))
            approach = abs(np.dot(velocity-racket_v, normal))
            restitution = np.clip(.759*np.exp(-.0441*approach), .2, .95)
            reconstructed = velocity-(1+restitution)*np.dot(velocity-racket_v,normal)*normal
            metrics['legacy_impact_residual_mps'] = float(np.linalg.norm(reconstructed-out))
            solved = outgoing(racket, reconstructed)
            checks['outgoing_landing_exists'] = len(solved.t_events[0]) == 1
            if checks['outgoing_landing_exists']:
                metrics['outgoing_landing_error_m'] = float(np.linalg.norm(solved.y_events[0][0,:3]-[2.055,-.7625,.02]))
                metrics['outgoing_landing_time_error_s'] = abs(float(solved.t_events[0][0])-.7)
            pairs = {
                'contact_position_error_m': 'committed_contact_truth_position_m',
                'contact_time_error_s': 'committed_contact_truth_time_s',
                'adapter_position_error_m': 'converted_contact_position_m',
                'adapter_velocity_error_mps': 'converted_contact_velocity_mps',
                'adapter_time_error_s': 'converted_contact_time_s',
                'decoder_position_error_m': 'converted_contact_position_m',
                'decoder_velocity_error_mps': 'converted_contact_velocity_mps',
                'outgoing_landing_error_m': 'managed_legacy_landing_simulation_m',
                'outgoing_landing_time_error_s': 'managed_legacy_landing_simulation_time_s',
            }
            checks.update({key: key in metrics and metrics[key] <= limits[limit] for key,limit in pairs.items()})
            if case['speed']==10 and case['spin']=='zero' and case['y']==-.7 and case['cutoff_s']==.18:
                before, after, bounce = plants[10, 'zero']
                times=np.linspace(0,case['crossing_s'],250)
                incoming=np.array([(before.sol(t) if t<bounce else after.sol(t))[:3] for t in times])
                following=solved.y[:3].T
                fig, axes=plt.subplots(1,2,figsize=(11,4))
                for axis, second, label in zip(axes,[2,1],['z (m above table)','y (m)']):
                    axis.plot(incoming[:,0],incoming[:,second],label='Independent incoming truth')
                    axis.plot(following[:,0],following[:,second],label='Outgoing from actual command')
                    axis.scatter([contact[0]],[contact[second]],marker='x',label='V2 contact / racket target')
                    axis.quiver(contact[0],contact[second],normal[0]*.15,normal[second]*.15,
                                angles='xy',scale_units='xy',scale=1,label='Racket face normal')
                    axis.axvline(case['plane_x'],color='gray',linestyle=':')
                    axis.set(xlabel='x (m)',ylabel=label);axis.grid(alpha=.25)
                axes[0].legend(fontsize=8);fig.suptitle(f"Mode {case['mode']}: target-level simulation, no robot motion")
                fig.tight_layout();fig.savefig(args.output/f"conversion-mode-{case['mode']}.png",dpi=150);plt.close(fig)
        else:
            uncertain = (float(row['plane_lower']) <= float(row['bounce_upper'])+.001
                         or float(row['plane_std']) > .05 or row['plane_quality']!='1'
                         or float(row['position_variance']) > .15**2
                         or float(row['min_variance']) < -1e-12 or float(row['plane_vx']) >= -.5)
            checks = {'uncertainty_explains_abstention': row['reason']=='CONTACT_ORDER_UNCERTAIN' and uncertain,
                      'no_command_or_publication': row['eligible']=='0' and row['publication_allowed']=='0' and row['decoded']=='0'}
            metrics['bounce_upper_s'] = float(row['bounce_upper'])
            metrics['plane_lower_s'] = float(row['plane_lower'])
        results.append(dict(case=case, reason=row['reason'], checks=checks, metrics=metrics,
                            command_available=row['command_valid']=='1',
                            status='PASS' if all(checks.values()) else 'FAIL'))
    groups = {}
    for result in results:
        case=result['case'];key=(case['speed'],case['spin'],case['y'],case['mode'])
        groups.setdefault(key,[]).append(result)
    covered=sum(any(r['command_available'] and r['status']=='PASS' for r in rows)
                for rows in groups.values())
    report=dict(status='PASS' if all(r['status']=='PASS' for r in results) else 'FAIL',
                passed=sum(r['status']=='PASS' for r in results), executed=len(results), results=results,
                physical_scenarios=len(groups), scenarios_with_usable_command=covered,
                commands=sum(r['command_available'] for r in results),
                simulation_level=manifest['scope'])
    if covered != len(groups): report['status']='FAIL'
    (args.output/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='results'}))
    return 0 if report['status']=='PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
