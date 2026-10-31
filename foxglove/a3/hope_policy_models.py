#!/usr/bin/env python3
"""Select preinstalled policy/deploy/Planner bundles; never accept caller paths."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import sys
from hope_field_assets import atomic_write

LEGACY_MODELS = {'model14600', 'model11999', 'model12600'}
MODELS = LEGACY_MODELS | {'model17400', 'model12000'}
MOCAP_MODELS = MODELS - {'model14600'}
SERVE_CSV_NAMES = {
    'model14600': 'a3p_op3_serve025_v23_branch110_model14600.csv',
    'model12600': 'v23-12600.csv',
    'model17400': 'v23-17400.csv',
    'model12000': 'v23-12000.csv',
}

def catalog_installed(root):
    try:
        (root / 'catalog.json').stat()
    except FileNotFoundError:
        # A surviving selection (including a broken symlink) means a damaged
        # installation, not an explicit legacy installation without a catalog.
        if (root.is_symlink() or (root / 'catalog.json').is_symlink()
                or (root / 'selected.json').exists() or (root / 'selected.json').is_symlink()):
            raise ValueError('policy catalog missing; restore the catalog before START')
        return False
    return True

def inspect(root, model=None):
    catalog = json.loads((root / 'catalog.json').read_text())
    # Fresh robots may install one reviewed bundle; never fabricate legacy entries.
    if (catalog.get('version') != 1 or not isinstance(catalog.get('models'), dict)
            or not catalog['models'] or not set(catalog['models']) <= MODELS):
        raise ValueError('invalid policy catalog')
    selected = json.loads((root / 'selected.json').read_text())['model'] if (root / 'selected.json').exists() else 'model14600'
    if selected not in MODELS or (model is not None and model not in MODELS):
        raise ValueError('unknown policy model')
    name = selected if model is None else model
    if name not in catalog['models']:
        raise ValueError(f'policy bundle not installed: {name}')
    entry = catalog['models'][name]
    if name == 'model17400' and entry.get('required_ball_model') != 'ppq004':
        raise ValueError('model17400 requires PPQ004')
    legacy_receive = name == 'model17400' and entry.get('receive_only') is True
    if legacy_receive and entry.get('serve_csv') is not None:
        raise ValueError('receive-only model17400 must not carry a serve CSV')
    if name == 'model12000' or (name == 'model17400' and not legacy_receive) or (
            name in SERVE_CSV_NAMES and entry.get('receive_only') is False):
        serve = entry.get('serve_csv')
        if (entry.get('required_ball_model') != 'ppq004' or entry.get('receive_only') is not False
                or not isinstance(serve, dict) or serve.get('name') != SERVE_CSV_NAMES[name]
                or not isinstance(serve.get('sha256'), str)
                or re.fullmatch(r'[0-9a-f]{64}', serve['sha256']) is None):
            raise ValueError(f'{name} requires PPQ004 and its matched {SERVE_CSV_NAMES[name]} serve bundle')
    for path, expected in entry['files'].items():
        if hashlib.sha256(Path(path).read_bytes()).hexdigest() != expected:
            raise ValueError(f'policy bundle changed: {path}')
    return {'model': name, 'selected': selected, **{k:v for k,v in entry.items() if k != 'files'}}

def require_stopped():
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit(): continue
        try:
            name = (proc / 'exe').resolve(strict=True).name
            if name in {'a3_deploy_onnx_ref_pingpong', 'hope_planner_cpp_node'}:
                raise ValueError('stop Runner and Planner before selecting policy')
        except (FileNotFoundError, PermissionError, ProcessLookupError): pass

def handle(root, request):
    op = request.get('op')
    if op == 'list_policy_models':
        models = {}
        for name in sorted(MODELS):
            try:
                models[name] = {**inspect(root, name), 'available': True, 'reason': ''}
            except (OSError, ValueError, KeyError, TypeError) as exc:
                models[name] = {'available': False, 'reason': str(exc)[-400:]}
        return {'models': models}
    if op == 'read_policy_model':
        return inspect(root) if catalog_installed(root) else {'catalog_installed': False, 'model': None}
    if op == 'verify_policy_model':
        result = inspect(root)
        if result['model'] != request.get('model'): raise ValueError('selected policy differs across endpoints')
        return result
    if op == 'select_policy_model':
        require_stopped()
        result = inspect(root, request.get('model'))
        if request.get('model') not in MODELS: raise ValueError('unknown policy model')
        atomic_write(root / 'selected.json', json.dumps({'model': result['model']}).encode())
        return inspect(root)
    raise ValueError('unknown policy operation')

def main():
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--field',choices=['deployment','planner_profile']);a=p.parse_args()
    try:
        if a.field == 'deployment' and not catalog_installed(a.root):
            print('NONE')
            return 0
        result = inspect(a.root) if a.field else handle(a.root,json.load(sys.stdin))
        print(result[a.field] if a.field else json.dumps(result))
    except (OSError,ValueError,KeyError,TypeError) as e:
        print(str(e),file=sys.stderr);return 1
    return 0
if __name__ == '__main__':sys.exit(main())
