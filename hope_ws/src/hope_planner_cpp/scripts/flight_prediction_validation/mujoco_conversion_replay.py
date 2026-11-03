#!/usr/bin/env python3
"""Replay actual converted command returns in the supplied A3 MuJoCo scene.

The robot is pinned. This tests ball flight after the independently reconstructed
impact, not policy tracking, joint reachability, or a simulated racket collision.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', type=Path, required=True)
    parser.add_argument('--conversion', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    app = args.repo / 'apps/a3_mujoco_serve'
    sys.path.insert(0, str(app))
    from a3_serve.mujoco_scene import A3ServeScene
    import mujoco

    config_path = app / 'config/serve.json'
    config = json.loads(config_path.read_text())
    model = (config_path.parent / config['model']['xml']).resolve()
    # The serve app's default drag fit differs from the incoming planner fit.
    # Freeze the explicit overlay before scoring; no simulator source is edited.
    config['physics']['drag']['k'] = 0.12861635416914574
    acceptance = json.loads((args.conversion / 'manifest.json').read_text())['acceptance']
    tolerance = acceptance['simulation_tolerances']['managed_legacy_landing_simulation_m']
    rows = list(csv.DictReader((args.conversion / 'probe.csv').open()))
    offered = [row for row in rows if row['command_valid'] == '1']
    manifest = dict(
        scope=__doc__,
        mujoco_version=mujoco.__version__,
        config=config,
        config_source_sha256=hashlib.sha256(config_path.read_bytes()).hexdigest(),
        model=str(model.relative_to(args.repo)),
        model_files={
            str(p.relative_to(model.parent)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(model.parent.rglob('*'))
            if p.is_file()
        },
        command_receipt_sha256=hashlib.sha256(
            (args.conversion / 'probe.csv').read_bytes()
        ).hexdigest(),
        cases=[int(row['id']) for row in offered],
        landing_tolerance_m=tolerance,
        coordinate_transform='table XYZ + [near_edge_x, table_width/2, table_height]',
        duration_s=0.9,
        expected='one receiver-side return over the net to [2.055,-.7625,.02]',
    )
    (args.output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    scene = A3ServeScene(model, config['physics'], config['model'])
    joints = (
        scene.model.key_qpos[0, scene.qpos_addresses].copy()
        if scene.model.nkey
        else np.zeros(len(scene.qpos_addresses))
    )

    def vector(row, names):
        return np.array([float(row[name]) for name in names.split()])

    results = []
    for row in offered:
        incoming = vector(row, 'cvx cvy cvz')
        racket = vector(row, 'rx ry rz')
        racket_velocity = vector(row, 'rvx rvy rvz')
        normal = vector(row, 'nx ny nz')
        relative = float(np.dot(incoming - racket_velocity, normal))
        restitution = np.clip(0.759 * np.exp(-0.0441 * abs(relative)), 0.2, 0.95)
        outgoing = incoming - (1 + restitution) * relative * normal
        replay = scene.simulate_outgoing_ball(
            joints, racket + scene.offset, outgoing, duration_s=manifest['duration_s']
        )
        landing = replay.first_bounce_table
        error = (
            float(np.linalg.norm(landing - [2.055, -0.7625, 0.02]))
            if landing is not None
            else None
        )
        checks = dict(
            landing=error is not None and error <= tolerance,
            no_net_contact=not replay.net_contact,
            ball_clears_net=replay.net_clearance_m is not None
            and replay.net_clearance_m > scene.net_height + scene.ball_radius,
            no_unplanned_racket_contact=not replay.racket_contact,
        )
        results.append(
            dict(
                id=int(row['id']),
                status='PASS' if all(checks.values()) else 'FAIL',
                checks=checks,
                landing_error_m=error,
                landing_table=landing.tolist() if landing is not None else None,
                net_center_height_m=replay.net_clearance_m,
            )
        )
    report = dict(
        status='PASS' if results and all(r['status'] == 'PASS' for r in results) else 'FAIL',
        executed=len(results),
        passed=sum(r['status'] == 'PASS' for r in results),
        simulation_level=manifest['scope'],
        results=results,
    )
    (args.output / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'results'}))
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
