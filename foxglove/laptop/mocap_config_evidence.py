"""Resolve a process's ROS startup parameters from its actual ordered inputs.

This is startup evidence, not a claim that dynamic parameter services were queried.
VRPN connection parameters are consumed when the client constructs its connection.
"""
import hashlib
from pathlib import Path
import yaml


def startup_parameters(argv, proc_path):
    node, namespace = '', ''
    for token in argv:
        if token.startswith('__node:='): node = token.split(':=', 1)[1]
        if token.startswith('__ns:='): namespace = token.split(':=', 1)[1]
    name = '/' + '/'.join(v.strip('/') for v in (namespace, node) if v)
    parameters, files = {}, []

    def collect(document, prefix=''):
        if not isinstance(document, dict):
            raise ValueError('Invalid ROS parameters document')
        for key, value in document.items():
            if key == 'ros__parameters':
                selector = '/' + prefix.strip('/')
                if selector in ('/**', name, '/' + node):
                    if not isinstance(value, dict): raise ValueError('Invalid ROS parameters block')
                    parameters.update(value)
            elif isinstance(value, dict):
                collect(value, prefix.rstrip('/') + '/' + str(key).strip('/'))

    for i, arg in enumerate(argv[:-1]):
        if arg == '--params-file':
            path = Path(argv[i+1])
            actual = proc_path/'root'/str(path).lstrip('/') if path.is_absolute() else proc_path/'cwd'/path
            data = actual.read_bytes()
            collect(yaml.safe_load(data))
            files.append({'path': str(path), 'sha256': hashlib.sha256(data).hexdigest()})
        elif arg in ('-p', '--param'):
            key, value = argv[i+1].split(':=', 1)
            parameters[key] = yaml.safe_load(value)
    return parameters, files


def endpoint_evidence(kind, parameters, receipt):
    if kind in {'vrpn', 'vrpn_relay'}:
        expected = {'server': receipt.get('HOPE_LIFECYCLE_VRPN_IP'),
                    'port': int(receipt.get('HOPE_LIFECYCLE_VRPN_PORT', '0'))}
        if kind == 'vrpn_relay':
            expected.update(position_scale=0.001 if receipt.get('HOPE_LIFECYCLE_VRPN_UNITS') == 'mm' else 1.0,
                            pants_profile=receipt.get('HOPE_LIFECYCLE_PANTS_PROFILE'),
                            table_side=receipt.get('HOPE_LIFECYCLE_TABLE_SIDE'), follow_lifecycle_config=True)
    elif kind == 'base' and receipt.get('HOPE_LIFECYCLE_MOCAP_SOURCE') == 'vrpn':
        expected = {'vrpn_pelvis_asset_confirmed': receipt.get('HOPE_VRPN_PELVIS_ASSET_CONFIRMED') == 'true',
                    'world_frame_calibrated': receipt.get('HOPE_VRPN_WORLD_FRAME_CONFIRMED') == 'true'}
    elif kind == 'motive':
        expected = {'hostname': receipt.get('HOPE_LIFECYCLE_MOTIVE_IP')}
    else:
        return {}
    return {'expected': expected, 'startup_parameters': {k: parameters.get(k) for k in expected},
            'matches': all(parameters.get(k) == v for k, v in expected.items())}
