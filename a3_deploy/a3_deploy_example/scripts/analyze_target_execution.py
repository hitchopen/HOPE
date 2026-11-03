#!/usr/bin/env python3
"""Measure position, velocity and face-normal execution at the same Planner deadline.

Pure offline diagnostics. Never changes formal Gate3 criteria. Native racket
velocity is a backward difference; optional plant velocity is reported separately.
The face plane is unoriented for a two-sided paddle, so the primary angle is
acos(abs(n_actual dot n_target)); directed angles and incoming-side alignment are
also retained. Missing normals fail explicitly, never synthesized as +X.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd


def unit(vector):
    vector = np.asarray(vector, dtype=float)
    norm = np.linalg.norm(vector)
    if not np.all(np.isfinite(vector)) or norm < 1e-8:
        raise ValueError('nonfinite or zero normal/velocity direction')
    return vector / norm


def finite(vector, name):
    vector = np.asarray(vector, dtype=float)
    if not np.all(np.isfinite(vector)):
        raise ValueError(f'nonfinite {name}')
    return vector


def angles(actual, target, relative_incoming=None):
    a, b = unit(actual), unit(target)
    dot = float(np.clip(a @ b, -1., 1.))
    result = {'normal_plane_deg': float(np.degrees(np.arccos(abs(dot)))),
              'normal_directed_deg': float(np.degrees(np.arccos(dot)))}
    if relative_incoming is not None:
        incoming = finite(relative_incoming, 'incoming velocity')
        if a @ incoming > 0:
            a = -a
        if b @ incoming > 0:
            b = -b
        result['normal_incoming_face_deg'] = float(np.degrees(np.arccos(np.clip(a @ b, -1, 1))))
    return result


def summarize(rows):
    result = {}
    for side in ['ALL', 'FH', 'BH']:
        selected = [r for r in rows if side == 'ALL' or r['side'] == side]
        result[side] = {'n': len(selected)}
        if not selected:
            continue
        for key in ['position_cm', 'velocity_mps', 'normal_plane_deg', 'normal_directed_deg',
                    'normal_incoming_face_deg', 'plant_position_cm', 'plant_velocity_mps',
                    'normal_vs_velocity_target_deg']:
            values = np.array([r[key] for r in selected if key in r])
            if values.size:
                result[side][key] = {'median': float(np.median(values)),
                                     'p95': float(np.percentile(values, 95)), 'max': float(values.max())}
        for key in ['position_delta_cm', 'velocity_delta_mps']:
            values = np.array([r[key] for r in selected])
            result[side][key] = {'signed_median_xyz': np.median(values, axis=0).tolist(),
                                 'absolute_median_xyz': np.median(abs(values), axis=0).tolist(),
                                 'absolute_p95_xyz': np.percentile(abs(values), 95, axis=0).tolist()}
    return result


def analyze(root):
    root = Path(root)
    trace = pd.read_csv(root/'pp_runner_trace.csv').sort_values('wall_time_ns')
    plans = pd.read_csv(root/'pp_planner_debug.csv')
    plans = plans[plans.wire_valid == 1].drop_duplicates('flight_id')
    plant_path = root/'pp_mujoco_plant.csv'
    plant = (pd.read_csv(plant_path).sort_values('wall_time_ns').drop_duplicates('wall_time_ns', keep='last')
             if plant_path.exists() else None)
    rows, invalid = [], []
    for shot in plans.itertuples():
        try:
            deadline = float(shot.strike_deadline_wall_s)
            finite(deadline, 'deadline')
            window = trace[(trace.frozen_flight_id == shot.flight_id) &
                           (abs(trace.wall_time_ns/1e9-deadline) < .15)].drop_duplicates('wall_time_ns', keep='last')
            times = window.wall_time_ns.to_numpy()/1e9
            if len(times) < 2 or not times[0] <= deadline <= times[-1]:
                raise ValueError('deadline not bracketed by same-flight samples')
            if np.max(np.diff(times)) > .05:
                raise ValueError('same-flight trace has a gap exceeding 50 ms')
            sample = lambda prefix: np.array([np.interp(deadline, times, window[prefix+a]) for a in 'xyz'])
            target = np.array([getattr(shot, 'target_actor_'+a) for a in 'xyz'])
            velocity = np.array([getattr(shot, 'racket_v'+a) for a in 'xyz'])
            normal = np.array([getattr(shot, 'target_n'+a) for a in 'xyz'])
            actual_p, actual_v, actual_n = sample('racket_'), sample('racket_v'), sample('racket_n')
            incoming = np.array([getattr(shot, 'strike_v'+a) for a in 'xyz'])
            finite(np.r_[target, velocity, actual_p, actual_v], 'target or measured position/velocity')
            dp, dv = actual_p-target, actual_v-velocity
            row = dict(flight_id=int(shot.flight_id), side='FH' if shot.swing_sign > 0 else 'BH',
                       deadline=deadline, target_position=target.tolist(), target_velocity=velocity.tolist(),
                       target_normal=unit(normal).tolist(), actual_normal=unit(actual_n).tolist(),
                       position_cm=float(np.linalg.norm(dp)*100), velocity_mps=float(np.linalg.norm(dv)),
                       position_delta_cm=(dp*100).tolist(), velocity_delta_mps=dv.tolist(),
                       **angles(actual_n, normal, incoming-actual_v))
            if np.linalg.norm(velocity) >= 1e-8:
                row['normal_vs_velocity_target_deg'] = angles(velocity, normal)['normal_directed_deg']
            if plant is not None:
                pt = plant.wall_time_ns.to_numpy()/1e9
                if pt[0] <= deadline <= pt[-1]:
                    pp = np.array([np.interp(deadline, pt, plant['racket_'+a]) for a in 'xyz'])
                    pv = np.array([np.interp(deadline, pt, plant['racket_v'+a]) for a in 'xyz'])
                    finite(np.r_[pp, pv], 'plant position/velocity')
                    row.update(plant_position_cm=float(np.linalg.norm(pp-target)*100),
                               plant_velocity_mps=float(np.linalg.norm(pv-velocity)))
            rows.append(row)
        except (ValueError, AttributeError, KeyError) as exc:
            invalid.append({'flight_id': int(shot.flight_id), 'reason': str(exc)})
    return {'source': str(root), 'scope': 'same-deadline offline p/v/n execution; not physical qualification',
            'frame': 'world frame, original Planner target_actor position; XYZ errors retained',
            'sampling': 'same-flight Runner interpolation at strike_deadline_wall_s; optional plant derivative reported separately',
            'normal_convention': 'two-sided face plane; incoming-side and raw directed angles also retained',
            'rows': rows, 'invalid': invalid, 'summary': summarize(rows)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', action='append', required=True, help='LABEL=RUN_DIRECTORY')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = {label: analyze(path) for label, path in (item.split('=', 1) for item in args.run)}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    for label, run in result.items():
        print(label, {side: {key: round(run['summary'][side][key]['median'], 4)
                            for key in ['position_cm', 'velocity_mps', 'normal_plane_deg', 'plant_velocity_mps']
                            if key in run['summary'][side]} for side in ['FH', 'BH']}, 'invalid', len(run['invalid']))
    if any(run['invalid'] for run in result.values()):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
