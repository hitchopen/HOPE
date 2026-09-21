#!/usr/bin/env python3
"""Verify the public installer/runtime payload in a clone or source archive.

No Git, ROS, npm, model inference or hardware access is required. --write refreshes
release hashes after rebuilding the installer; normal invocation only reads files.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[2]
EXT = ROOT / 'foxglove/extensions/hope-a3-console'
MANIFEST = ROOT / 'foxglove/public-runtime-manifest.json'
RUNTIME = ROOT / 'a3_deploy/a3_deploy_example'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def inventory():
    package = json.loads((EXT / 'package.json').read_text())
    publisher = re.sub(r'[^a-zA-Z0-9]', '', package['publisher']).lower()
    installer = EXT / f"{publisher}.{package['name']}-{package['version']}.foxe"
    if not installer.is_file():
        raise ValueError(f'missing prebuilt installer: {installer.relative_to(ROOT)}')
    with zipfile.ZipFile(installer) as archive:
        if archive.testzip() is not None:
            raise ValueError('installer CRC check failed')
        archived_package = json.loads(archive.read('package.json'))
        if archived_package != package:
            raise ValueError('installer package.json differs from source; rebuild it')
        for name in ('README.md', 'CHANGELOG.md', 'LICENSE', 'THIRD_PARTY_NOTICES.txt'):
            if archive.read(name) != (EXT / name).read_bytes():
                raise ValueError(f'installer {name} differs from source')
        main = package['main'].removeprefix('./')
        bundle = archive.read(main)
        names = set(archive.namelist())
        # Webpack emits these font URLs into the injected stylesheet.
        fonts = set(re.findall(rb'[a-f0-9]{20}\.woff2?', bundle))
        if not fonts:
            raise ValueError('installer has no bundled font references')
        for font in fonts:
            if 'dist/' + font.decode() not in names:
                raise ValueError(f'missing packaged font: {font.decode()}')
    files = {installer, EXT / 'package.json', EXT / 'package-lock.json',
             EXT / 'README.md', EXT / 'CHANGELOG.md', EXT / 'LICENSE',
             EXT / 'THIRD_PARTY_NOTICES.txt'}
    files.update((EXT / 'src').rglob('*'))
    for directory in ('a3', 'laptop', 'helpers', 'layouts'):
        for p in (ROOT / 'foxglove' / directory).rglob('*'):
            if '__pycache__' not in p.parts and p.suffix in ('', '.py', '.yaml', '.xml', '.json', '.service', '.sh', '.example'):
                files.add(p)
    assets = RUNTIME / 'assets/a3_runtime'
    motions = list((assets / 'serve/motions').glob('*.csv'))
    if len(motions) < 6:
        raise ValueError('the published serve playback set is incomplete')
    files.update(motions)
    for name in ('humanlike.yaml', 'policy.onnx', 'lin_vel_encoder.onnx'):
        p = assets / 'teleop_humanlike' / name
        if not p.is_file():
            raise ValueError(f'missing default Teleop asset: {p.relative_to(ROOT)}')
        files.add(p)
    for rel in ('models/model_21800/policy/exported/policy.onnx',
                'models/model_21800/policy/params/deploy.yaml',
                'scripts/build_a3_deploy_pkg.sh', 'scripts/a3p_gripper_bridge.py',
                'scripts/serve_gripper_presets.py',
                'src/a3/a3_deploy_onnx_ref/config/a3_runtime_config.pingpong.hitter_pingpong.yaml'):
        p = RUNTIME / rel
        if not p.is_file():
            raise ValueError(f'missing public runtime file: {p.relative_to(ROOT)}')
        files.add(p)
    return {'schema': 1, 'console_version': package['version'],
            'installer': str(installer.relative_to(ROOT)),
            'files': {str(p.relative_to(ROOT)): {'sha256': digest(p), 'bytes': p.stat().st_size}
                      for p in sorted(files) if p.is_file()}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--write', action='store_true', help='refresh the release manifest after rebuilding')
    args = parser.parse_args()
    current = inventory()
    if args.write:
        MANIFEST.write_text(json.dumps(current, indent=2) + '\n')
        print(f'Wrote {MANIFEST.relative_to(ROOT)}')
    else:
        expected = json.loads(MANIFEST.read_text())
        if current != expected:
            changed = sorted(k for k in set(current['files']) | set(expected['files'])
                             if current['files'].get(k) != expected['files'].get(k))
            raise ValueError('public release is missing/stale; rebuild and refresh: ' + ', '.join(changed))
    print(f"PUBLIC_RUNTIME_FILES_OK: {len(current['files'])} files; console {current['console_version']}; model_21800 + HumanLike")


if __name__ == '__main__':
    try:
        main()
    except (ValueError, OSError, KeyError, zipfile.BadZipFile) as error:
        print(f'PUBLIC_RUNTIME_FILES_FAIL: {error}', file=sys.stderr)
        raise SystemExit(1)
