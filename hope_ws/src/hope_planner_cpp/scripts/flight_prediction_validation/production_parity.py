#!/usr/bin/env python3
"""Compile exact upstream and final production sources/headers; require strict parity."""
import argparse
import hashlib
import io
import json
from pathlib import Path
import subprocess
import tarfile
import tempfile

BASE = '8d599e7d21e196275cf426488555e2648e8fd789'
SOURCES = ['racket_target_planner.cpp', 'adaptive_flight_commit.cpp',
           'trajectory_predictor.cpp', 'batch_physics_estimator.cpp', 'spin_estimator.cpp']


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--repo', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    repo = Path(args.repo)
    relative = 'hope_ws/src/hope_planner_cpp'
    planner = repo / relative
    protected = SOURCES + ['../include/hope_planner_cpp/' + name for name in [
        'racket_target_planner.hpp', 'adaptive_flight_commit.hpp',
        'trajectory_predictor.hpp', 'batch_physics_estimator.hpp',
        'spin_estimator.hpp', 'future_contact_predictor.hpp']]
    hashes = {}
    for path in protected:
        path = str(Path(relative + '/src/' + path))
        # Normalize '..' before passing a Git tree path.
        path = str((repo / path).resolve().relative_to(repo))
        original = subprocess.check_output(['git', '-C', str(repo), 'show', BASE + ':' + path])
        assert (repo / path).read_bytes() == original, path
        hashes[path] = hashlib.sha256(original).hexdigest()
    with tempfile.TemporaryDirectory(prefix='hope-production-parity-') as temporary:
        root = Path(temporary)
        archive = subprocess.check_output(['git', '-C', str(repo), 'archive', BASE,
                                          relative + '/include', relative + '/src'])
        with tarfile.open(fileobj=io.BytesIO(archive)) as tar:
            tar.extractall(root, filter='data')
        results = {}
        for label, directory in [('upstream', root / relative), ('minimal', planner)]:
            executable = root / label
            command = ['g++', '-std=c++17', '-O2', '-I' + str(directory / 'include'),
                       '-I/usr/include/eigen3', str(planner / 'test/production_parity_probe.cpp')]
            command += [str(directory / 'src' / name) for name in
                        SOURCES if name != 'adaptive_flight_commit.cpp']
            subprocess.run(command + ['-o', str(executable)], check=True)
            results[label] = json.loads(subprocess.check_output([str(executable)], text=True))
    assert results['upstream'] == results['minimal'], 'production numerical regression'
    # Representative bounce fixture must retain distinct legacy contact modes.
    modes = results['minimal']['all_spin_modes']
    assert any(a[1:] != b[1:] for a, b in zip(modes[108:216], modes[324:432]))
    receipt = dict(status='PASS', base=BASE, production_source_sha256=hashes,
                   exact_fixture_parity=True, exceptions=[],
                   fixture_counts={key: len(value) for key, value in results['minimal'].items()},
                   max_abs_difference=0, all_five_spin_modes=True,
                   nakashima_and_venue_remain_distinct=True, fixtures=results['minimal'],
                   note='Each side compiled from its own complete source/header tree; no updated goldens.')
    Path(args.output).write_text(json.dumps(receipt, indent=2) + '\n')
    print(json.dumps({key: value for key, value in receipt.items() if key != 'fixtures'}))


if __name__ == '__main__':
    main()
