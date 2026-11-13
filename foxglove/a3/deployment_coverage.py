#!/usr/bin/env python3
"""Offline deployment evidence coverage gate. Never connects to or changes a robot.

Evidence assertions require human review; this verifies coverage, provenance,
attachment integrity and identity/freshness, not the truth of arbitrary logs.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
INVENTORY = ROOT / 'foxglove/deployment_profiles/coverage.json'
LEVELS = ('installed', 'running', 'persistent')
SHA = re.compile(r'[0-9a-f]{64}\Z')
COMMIT = re.compile(r'[0-9a-f]{40}\Z')


def timestamp(value):
    result = datetime.fromisoformat(value.replace('Z', '+00:00'))
    if result.tzinfo is None:
        raise ValueError('timestamp requires a timezone')
    return result


def template(inventory):
    return {
        'schema': 1, 'inventory_sha256': hashlib.sha256(INVENTORY.read_bytes()).hexdigest(),
        'release_commit': '', 'started_at': '',
        'hosts': {role: {'machine_id': '', 'boot_id': '', 'before_boot_id': ''}
                  for role in sorted({c['host'] for c in inventory['components']})},
        'components': {c['id']: {
            'applicable': True, 'reason': '', 'source_paths': [],
            'stages': {level: {'status': 'not_checked',
                               'expected': {key: None for key in c['checks'][level]},
                               'observed': {}, 'captured_at': '', 'machine_id': '',
                               'boot_id': '', 'evidence': []}
                       for level in LEVELS}}
            for c in inventory['components']},
    }


def source_exists(commit, path):
    if not isinstance(path, str) or not path or Path(path).is_absolute() or '..' in Path(path).parts:
        return False
    return subprocess.run(['git', 'cat-file', '-e', f'{commit}:{path}'], cwd=ROOT,
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0


def compare_staging(staging, checkout):
    """Inventory frozen deployment sources, including files outside Git status.

    Differences need classification: newer implementations are not omissions.
    File contents are never printed or copied.
    """
    if not staging.is_dir() or not checkout.is_dir():
        raise ValueError('staging and checkout must be existing directories')
    rows = []
    for path in sorted(staging.rglob('*')):
        relative = path.relative_to(staging)
        target = checkout / relative
        if path.is_symlink():
            rows.append({'path': relative.as_posix(), 'status': 'symlink_requires_review'})
            continue
        if not path.is_file():
            continue
        historical = hashlib.sha256(path.read_bytes()).hexdigest()
        # Never follow a checkout symlink outside the source tree.
        contained = target.resolve().is_relative_to(checkout.resolve())
        current = hashlib.sha256(target.read_bytes()).hexdigest() if contained and target.is_file() and not target.is_symlink() else None
        rows.append({'path': relative.as_posix(), 'staging_sha256': historical,
                     'checkout_sha256': current,
                     'status': 'identical' if current == historical else 'missing' if current is None else 'different'})
    if not rows:
        raise ValueError('empty staging directory is not an audit')
    return rows


def validate(report, inventory, evidence_root, level, *, now=None, resolve_source=source_exists):
    errors = []
    now = now or datetime.now(timezone.utc)
    if report.get('schema') != 1:
        errors.append('unsupported receipt schema')
    if report.get('inventory_sha256') != hashlib.sha256(INVENTORY.read_bytes()).hexdigest():
        errors.append('inventory changed; regenerate/reconcile the receipt')
    commit = report.get('release_commit', '')
    valid_commit = isinstance(commit, str) and bool(COMMIT.fullmatch(commit))
    if not valid_commit:
        errors.append('release_commit must be a full Git commit')
    try:
        started = timestamp(report['started_at'])
        if not 0 <= (now - started).total_seconds() <= 86400:
            errors.append('audit must have started within the last 24 hours')
    except (KeyError, TypeError, ValueError, AttributeError):
        started = now
        errors.append('missing/invalid started_at')
    components = report.get('components', {})
    wanted = {c['id'] for c in inventory['components']}
    if set(components) != wanted:
        errors.append(f'component set mismatch: missing={sorted(wanted-set(components))}, extra={sorted(set(components)-wanted)}')
    for spec in inventory['components']:
        name = spec['id']
        item = components.get(name, {})
        if item.get('applicable') is False:
            if not spec.get('optional') or not isinstance(item.get('reason'), str) or not item['reason'].strip():
                errors.append(f'{name}: cannot omit mandatory component or omit scope reason')
            continue
        if item.get('applicable') is not True:
            errors.append(f'{name}: applicability is not declared')
        paths = item.get('source_paths')
        if not isinstance(paths, list) or not paths or not valid_commit or not all(resolve_source(commit, p) for p in paths):
            errors.append(f'{name}: missing versioned source/build recipe/hardware profile')
        host = report.get('hosts', {}).get(spec['host'], {})
        if not host.get('machine_id') or not host.get('boot_id'):
            errors.append(f'{name}: missing host machine/boot identity')
        for stage in LEVELS[:LEVELS.index(level)+1]:
            prefix = f'{name}/{stage}'
            record = item.get('stages', {}).get(stage, {})
            if record.get('status') != 'pass':
                errors.append(f'{prefix}: not passed')
            expected, observed = record.get('expected', {}), record.get('observed', {})
            required = set(spec['checks'][stage])
            if not required <= set(expected) or any(v is None or v == '' or v == {} or v == [] for v in expected.values()):
                errors.append(f'{prefix}: incomplete expected facts')
            if expected != observed:
                errors.append(f'{prefix}: expected/observed mismatch')
            if stage == 'installed':
                hashes = expected.get('artifact_hashes')
                if not isinstance(hashes, dict) or not hashes or not all(
                    isinstance(p, str) and Path(p).is_absolute() and isinstance(h, str) and SHA.fullmatch(h)
                    for p, h in hashes.items()
                ):
                    errors.append(f'{prefix}: artifact_hashes requires absolute paths and full SHA256 values')
            if not host.get('machine_id') or record.get('machine_id') != host.get('machine_id') or record.get('boot_id') != host.get('boot_id'):
                errors.append(f'{prefix}: wrong machine or stale boot')
            if stage == 'persistent' and (not host.get('before_boot_id') or host['before_boot_id'] == host.get('boot_id')):
                errors.append(f'{prefix}: no distinct pre-reboot identity')
            try:
                captured = timestamp(record['captured_at'])
                if not started <= captured <= now:
                    errors.append(f'{prefix}: evidence outside this audit window')
            except (KeyError, TypeError, ValueError, AttributeError):
                errors.append(f'{prefix}: invalid evidence timestamp')
            attachments = record.get('evidence')
            if not isinstance(attachments, list) or not attachments:
                errors.append(f'{prefix}: no evidence attachments')
                continue
            for attachment in attachments:
                try:
                    rel = Path(attachment['path'])
                    path = (evidence_root / rel).resolve()
                    expected_sha = attachment['sha256']
                    if rel.is_absolute() or not path.is_relative_to(evidence_root.resolve()) or not SHA.fullmatch(expected_sha):
                        raise ValueError('invalid attachment path/hash')
                    if hashlib.sha256(path.read_bytes()).hexdigest() != expected_sha:
                        raise ValueError('attachment hash mismatch')
                except (OSError, KeyError, TypeError, ValueError):
                    errors.append(f'{prefix}: missing/changed/invalid evidence attachment')
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('template')
    compare = sub.add_parser('compare-staging')
    compare.add_argument('staging', type=Path)
    compare.add_argument('--checkout', type=Path, default=ROOT)
    check = sub.add_parser('check')
    check.add_argument('receipt', type=Path)
    check.add_argument('--level', choices=LEVELS, required=True)
    args = parser.parse_args()
    if args.command == 'compare-staging':
        try:
            rows = compare_staging(args.staging, args.checkout)
        except (OSError, ValueError) as exc:
            print(json.dumps({'result': 'ERROR', 'error': str(exc)}))
            return 1
        unresolved = sum(row['status'] != 'identical' for row in rows)
        print(json.dumps({'result': 'DIFFERENCES_REQUIRE_REVIEW' if unresolved else 'IDENTICAL_FILES',
                          'compared': len(rows), 'different_or_missing': unresolved, 'files': rows}, indent=2))
        return 1 if unresolved else 0
    inventory = json.loads(INVENTORY.read_text())
    if args.command == 'template':
        print(json.dumps(template(inventory), indent=2, ensure_ascii=False))
        return 0
    try:
        report = json.loads(args.receipt.read_text())
        errors = validate(report, inventory, args.receipt.parent, args.level)
    except (OSError, ValueError, TypeError, AttributeError) as exc:
        errors = [f'invalid receipt: {exc}']
    print(json.dumps({'result': 'INCOMPLETE' if errors else 'EVIDENCE_COVERAGE_COMPLETE',
                      'level': args.level, 'errors': errors,
                      'limit': 'Coverage only; reviewer must verify assertions against evidence. No physical qualification.'},
                     indent=2, ensure_ascii=False))
    return 1 if errors else 0


if __name__ == '__main__':
    sys.exit(main())
