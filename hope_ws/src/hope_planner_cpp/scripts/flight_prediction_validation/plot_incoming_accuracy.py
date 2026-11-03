#!/usr/bin/env python3
"""Plot recorded production forecasts and independent simulation errors."""
import argparse
import base64
import csv
import hashlib
import html
import json
from pathlib import Path
import sys

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'test'))
from independent_ball_flight import flight


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--conversion', type=Path, required=True)
    parser.add_argument('--mujoco', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    inputs = [
        args.conversion / name
        for name in ['manifest.json', 'result.json', 'probe.csv', 'waypoints.csv']
    ]
    inputs += [args.mujoco / 'manifest.json', args.mujoco / 'result.json']
    manifest = json.loads(inputs[0].read_text())
    conversion = json.loads(inputs[1].read_text())
    commands = {int(row['id']): row for row in csv.DictReader(inputs[2].open())}
    paths = {}
    for row in csv.DictReader(inputs[3].open()):
        paths.setdefault(int(row['id']), []).append(row)
    replay_manifest = json.loads(inputs[4].read_text())
    replay = json.loads(inputs[5].read_text())
    assert (
        replay_manifest['command_receipt_sha256']
        == hashlib.sha256(inputs[2].read_bytes()).hexdigest()
    ), 'MuJoCo used different commands'
    cases = {case['id']: case for case in manifest['cases']}
    issued = [row for row in conversion['results'] if row['command_available']]
    assert {row['case']['id'] for row in issued} == {row['id'] for row in replay['results']}
    assert set(cases) == set(commands)
    limits = manifest['acceptance']['simulation_tolerances']
    colors = ['#0072B2', '#D55E00', '#009E73']
    plt.rcParams.update(
        {
            'font.size': 10,
            'axes.spines.top': False,
            'axes.spines.right': False,
            'savefig.facecolor': 'white',
        }
    )
    figures = []

    def save(fig, name, caption):
        fig.savefig(args.output / (name + '.png'), dpi=170)
        fig.savefig(args.output / (name + '.svg'))
        plt.close(fig)
        figures.append(dict(name=name, caption=caption))

    selected = [
        c
        for c in cases.values()
        if c['speed'] == 10 and c['spin'] == 'known' and c['y'] == -0.7 and c['mode'] == 1
    ]
    assert len(selected) == 2, 'Expected the declared two-cutoff reference scenario'
    selected.sort(key=lambda c: c['cutoff_s'])
    before, after, bounce = flight(10, 'known')
    crossing = selected[0]['crossing_s']
    times = np.linspace(0, crossing, 500)
    truth = np.array([(before.sol(t) if t < bounce else after.sol(t))[:3] for t in times])
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8), layout='constrained')
    axes[0].plot(truth[:, 0], truth[:, 2], color='#444444', lw=2, label='Independent ball truth')
    trajectory_metrics = []
    for color, case in zip(colors, selected):
        points = paths[case['id']]
        t = np.array([case['cutoff_s'] + float(p['dt']) for p in points])
        prediction = np.array([[float(p[n]) for n in ['x', 'y', 'z']] for p in points])
        keep = t <= crossing
        t, prediction = t[keep], prediction[keep]
        reference = np.array([(before.sol(v) if v < bounce else after.sol(v))[:3] for v in t])
        reference[:, 1] = case['y']
        errors = 1000 * np.linalg.norm(prediction - reference, axis=1)
        label = f"Forecast using first {case['cutoff_s'] * 1000:.0f} ms"
        axes[0].plot(prediction[:, 0], prediction[:, 2], '--', color=color, label=label)
        axes[0].scatter(prediction[0, 0], prediction[0, 2], color=color, s=35, zorder=5)
        axes[1].plot(t * 1000, errors, color=color, label=label)
        trajectory_metrics.append(
            dict(
                id=case['id'],
                cutoff_ms=case['cutoff_s'] * 1000,
                maximum_position_error_mm=float(max(errors)),
            )
        )
    row = commands[selected[-1]['id']]
    x, z = float(row['cx']), float(row['cz'])
    axes[0].axvline(
        selected[0]['plane_x'], color='#777777', linestyle=':', label='Selected contact plane'
    )
    axes[0].scatter(
        x, z, marker='x', s=90, color='#222222', label='Converted racket contact', zorder=6
    )
    axes[0].quiver(
        x,
        z,
        0.10 * float(row['nx']),
        0.10 * float(row['nz']),
        angles='xy',
        scale_units='xy',
        scale=1,
        color='#222222',
    )
    axes[0].set(
        xlabel='Table x (m); incoming ball travels toward smaller x',
        ylabel='Ball centre above table (m)',
        title='True and predicted paths nearly overlap',
    )
    axes[1].axvline(bounce * 1000, color='#777777', linestyle=':', label='True table bounce')
    axes[1].set(
        xlabel='Time since simulated launch (ms)',
        ylabel='3D forecast position error (mm)',
        title='The error view reveals the difference',
        ylim=(0, None),
    )
    for ax in axes:
        ax.grid(alpha=0.18)
        ax.legend(fontsize=8, loc='best')
    fig.suptitle(
        'Prediction after 100 ms and 180 ms of observations\n10 m/s, measured-spin synthetic flight; per-ball plane x = 0.15 m'
    )
    save(
        fig,
        'prediction-evolution',
        'These are actual V2 forecast waypoints from two independent observation prefixes of the same noiseless flight. The arrow is the racket face normal. Time starts at simulated launch, not net crossing; these are spatial errors, not processing latency.',
    )

    position = np.array([r['metrics']['contact_position_error_m'] * 1000 for r in issued])
    timing = np.array([r['metrics']['contact_time_error_s'] * 1000 for r in issued])
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8), layout='constrained')
    for ax, metric, values, label, limit in zip(
        axes,
        ['contact_position_error_m', 'contact_time_error_s'],
        [position, timing],
        ['Contact position error (mm)', 'Contact time error (ms)'],
        [
            limits['committed_contact_truth_position_m'] * 1000,
            limits['committed_contact_truth_time_s'] * 1000,
        ],
    ):
        for cutoff, color, marker in [(0.10, colors[0], 'o'), (0.18, colors[1], '^')]:
            group = [r for r in issued if r['case']['cutoff_s'] == cutoff]
            ax.scatter(
                [r['case']['id'] for r in group],
                [max(r['metrics'][metric] * 1000, 1e-6) for r in group],
                color=color,
                marker=marker,
                s=21,
                alpha=0.7,
                label=f'{cutoff*1000:.0f} ms observation window',
            )
        ax.axhline(limit, color='#333333', linestyle='--', label=f'Declared limit: {limit:g}')
        ax.set_yscale('log')
        ax.set_xticks([26.5, 80.5, 134.5], ['5 m/s', '10 m/s', '12 m/s'])
        ax.set(
            xlabel='Configured launch speed; points enumerate spin / plane / lateral cases',
            ylabel=label,
            title=f'Max {max(values):.4g}; median {np.median(values):.4g}',
        )
        ax.grid(alpha=0.18)
        ax.legend(fontsize=8, loc='best')
    fig.suptitle(
        f"Accuracy of {len(issued)} issued commands across {len(cases)} tested prefixes\n{conversion['scenarios_with_usable_command']}/{conversion['physical_scenarios']} physical scenarios have a usable command"
    )
    save(
        fig,
        'contact-accuracy',
        'Logarithmic error axes include the predeclared acceptance limits. Three early uncertain prefixes abstained and are excluded from numeric errors, not plotted as zero. Both cutoffs are correlated samples, so these are descriptive errors rather than statistical confidence bounds.',
    )

    landings = {row['id']: row for row in replay['results']}
    landing_errors = np.array(
        [landings[r['case']['id']]['landing_error_m'] * 1000 for r in issued]
    )
    numeric = np.array([r['metrics']['outgoing_landing_error_m'] * 1000 for r in issued])
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8), layout='constrained')
    for mode, color, label in zip(
        range(3), colors, ['Fixed plane', 'Per-ball plane', 'Stable per-ball plane']
    ):
        points = np.array(
            [
                landings[r['case']['id']]['landing_table']
                for r in issued
                if r['case']['mode'] == mode
            ]
        )
        delta = (points - [2.055, -0.7625, 0.02]) * 1000
        axes[0].scatter(delta[:, 0], delta[:, 1], color=color, s=26, alpha=0.55, label=label)
    axes[0].scatter(
        [0], [0], marker='+', color='#222222', s=170, linewidth=2, label='Desired landing'
    )
    axes[0].set(
        xlabel='Landing x minus target x (mm)',
        ylabel='Landing y minus target y (mm)',
        title='MuJoCo landing positions: magnified error view',
        aspect='equal',
    )
    axes[0].legend(fontsize=8)
    maxima = [max(position), max(numeric), max(landing_errors)]
    axes[1].barh(
        ['Incoming contact', 'Outgoing numerical reference', 'Outgoing MuJoCo replay'],
        maxima,
        color=colors,
    )
    limit = limits['managed_legacy_landing_simulation_m'] * 1000
    axes[1].axvline(limit, color='#333333', linestyle='--', label=f'Position limit: {limit:g} mm')
    axes[1].set(
        xlabel='Maximum 3D position error (mm)',
        title='Maximum errors at three different stages',
        xlim=(0, max(limit * 1.15, max(maxima) * 1.2)),
    )
    for index, value in enumerate(maxima):
        axes[1].text(value + 0.2, index, f'{value:.3f} mm', va='center')
    axes[1].legend(fontsize=8)
    for ax in axes:
        ax.grid(alpha=0.18)
    fig.suptitle(
        f"Outgoing accuracy in the supplied A3 MuJoCo scene: {replay['passed']}/{replay['executed']} pass\nBall flight starts at converted contact; robot is pinned"
    )
    save(
        fig,
        'mujoco-landing',
        'The left plot zooms into millimetre deviations; the right plot shows their size against the 20 mm limit. MuJoCo simulates the outgoing ball after reconstructed impact, not a policy-driven robot swing or racket collision. Its drag coefficient is explicitly matched to the planner.',
    )

    summary = dict(
        conversion_status=conversion['status'],
        mujoco_status=replay['status'],
        commands=len(issued),
        abstentions=len(cases) - len(issued),
        contact_position_max_mm=float(max(position)),
        contact_time_max_ms=float(max(timing)),
        numerical_landing_max_mm=float(max(numeric)),
        mujoco_landing_max_mm=float(max(landing_errors)),
        trajectory_examples=trajectory_metrics,
        source_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs},
        limitations='Noiseless synthetic data. Unknown spin uses a zero-spin plant with withheld orientation. No physical accuracy or dynamic robot contact claim. Numeric accuracy does not establish ROS delivery coverage.',
    )
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    with (args.output / 'errors.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                'case_id',
                'speed_mps',
                'spin',
                'plane_mode',
                'cutoff_s',
                'contact_mm',
                'contact_ms',
                'numeric_landing_mm',
                'mujoco_landing_mm',
            ]
        )
        for r in issued:
            c = r['case']
            m = r['metrics']
            writer.writerow(
                [
                    c['id'],
                    c['speed'],
                    c['spin'],
                    c['mode'],
                    c['cutoff_s'],
                    m['contact_position_error_m'] * 1000,
                    m['contact_time_error_s'] * 1000,
                    m['outgoing_landing_error_m'] * 1000,
                    landings[c['id']]['landing_error_m'] * 1000,
                ]
            )
    document = [
        '<!doctype html><html lang="en"><meta charset="utf-8"><title>HOPE prediction accuracy</title>',
        '<style>body{font:17px/1.5 system-ui;max-width:1150px;margin:40px auto;padding:0 20px;color:#182330}img{width:100%}section{margin:40px 0}p{max-width:950px}</style>',
        '<h1>HOPE prediction and conversion accuracy</h1>',
        '<p>' + html.escape(summary['limitations']) + '</p>',
    ]
    for figure in figures:
        encoded = base64.b64encode((args.output / (figure['name'] + '.png')).read_bytes()).decode()
        document.append(
            f'<section><img alt="{html.escape(figure["name"])}" src="data:image/png;base64,{encoded}"><p>{html.escape(figure["caption"])}</p></section>'
        )
    document.append(
        '<p>Source hashes and numeric results are in summary.json and errors.csv. PNG and SVG exports are provided beside this standalone HTML file.</p></html>'
    )
    (args.output / 'index.html').write_text('\n'.join(document))
    print(
        json.dumps({k: v for k, v in summary.items() if k not in ['source_sha256', 'limitations']})
    )


if __name__ == '__main__':
    main()
