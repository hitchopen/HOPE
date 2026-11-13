#!/usr/bin/env python3
"""Read-only evidence of laptop receipt and consumer environments."""
import json
import os
from pathlib import Path
import shlex
import subprocess
import xml.etree.ElementTree as ET
from mocap_config_evidence import startup_parameters, endpoint_evidence

KINDS = {'hope_live_scene.py': 'scene', 'hope_xbox_preview.py': 'xbox', 'client_node': 'vrpn',
         'foxglove_bridge': 'display', 'optitrack_mct_relay': 'motive_relay',
         'avatar_pro_vrpn_relay': 'vrpn_relay', 'motion_capture_tracking_node': 'motive',
         'hope_base_pose_flat_relay': 'base', 'hope_ball_flight_packetizer': 'packetizer'}


def read_receipt(path):
    receipt = {}
    for line in path.read_text().splitlines():
        if '=' in line and not line.startswith('#'):
            key, value = line.split('=', 1)
            parsed = shlex.split(value)
            if len(parsed) > 1 or key in receipt:
                raise ValueError('Malformed or duplicate runtime receipt key: ' + key)
            receipt[key] = parsed[0] if parsed else ''
    return receipt


def probe(receipt, proc=Path('/proc'), run=subprocess.run):
    target = receipt.get('HOPE_LIFECYCLE_HDU_IP', '')
    allowed = {target, receipt.get('HOPE_LIFECYCLE_LAPTOP_IP', ''), '127.0.0.1'} - {''}
    processes, errors = [], []
    for p in proc.glob('[0-9]*'):
        try:
            if p.stat().st_uid != os.getuid():
                continue
            argv = (p/'cmdline').read_bytes().decode().split('\0')
            # Match the actual executable/script, not a distrobox/bash launcher
            # whose argv merely mentions the eventual ROS consumer.
            program = argv[0]
            if Path(program).name.startswith('python'):
                program = next((x for x in argv[1:] if x and not x.startswith('-')), '')
            kind = KINDS.get(Path(program).name)
            if not kind:
                continue
            actual = p/'root'/program.lstrip('/') if program.startswith('/') else p/'cwd'/program
            result = {'pid': int(p.name), 'kind': kind, 'matches': False, 'program': str(actual.resolve())}
            processes.append(result)
            env = dict(v.split('=', 1) for v in (p/'environ').read_bytes().decode().split('\0') if '=' in v)
            peers = set(env.get('ROS_STATIC_PEERS', '').split(';')) - {''}
            result['peers'] = sorted(peers)
            result['matches'] = bool(target) and target in peers and peers <= allowed
            # Exported receipt values make a restarted receiver distinguishable
            # from one still using a previous side, source, or pants profile.
            if kind in {'vrpn', 'vrpn_relay', 'motive', 'base'}:
                keys = ('HDU_IP', 'TABLE_SIDE', 'PANTS_PROFILE', 'MOCAP_SOURCE', 'VRPN_IP', 'VRPN_PORT', 'VRPN_UNITS', 'MOTIVE_IP')
                result['runtime_values'] = {k: env.get('HOPE_LIFECYCLE_' + k) for k in keys}
                result['matches'] = result['matches'] and all(
                    env.get('HOPE_LIFECYCLE_' + k) == receipt.get('HOPE_LIFECYCLE_' + k) for k in keys)
            if kind in {'vrpn', 'vrpn_relay', 'motive', 'base'}:
                parameters, files = startup_parameters(argv, p)
                result['parameter_files'] = files
                result['endpoint'] = endpoint_evidence(kind, parameters, receipt)
                result['matches'] = result['matches'] and result['endpoint'].get('matches', True)
            profile = env.get('FASTDDS_DEFAULT_PROFILES_FILE') or env.get('FASTRTPS_DEFAULT_PROFILES_FILE')
            if profile:
                addresses = {a.text for peers_node in ET.parse(profile).iter()
                             if peers_node.tag.split('}')[-1] == 'initialPeersList'
                             for a in peers_node.iter() if a.tag.split('}')[-1] == 'address'}
                result['profile_peers'] = sorted(addresses)
                if addresses:
                    result['matches'] = result['matches'] and target in addresses and addresses <= allowed
        except (OSError, ValueError, UnicodeError, ET.ParseError) as exc:
            # Fail closed when an identified consumer disappears mid-observation.
            if 'result' in locals() and result.get('pid') == int(p.name):
                result.update(matches=False, error=str(exc))
            elif not isinstance(exc, (FileNotFoundError, ProcessLookupError)):
                errors.append(str(exc))
    r = run(['systemctl', '--user', 'is-active', 'hope-host-bridge.service'], capture_output=True, text=True, timeout=2)
    return {'receipt': receipt, 'processes': processes, 'errors': errors,
            'mocap_setup': {'source': 'laptop-runtime.env (current generated receiver receipt)',
                'pelvis_asset_confirmed': receipt.get('HOPE_VRPN_PELVIS_ASSET_CONFIRMED') == 'true',
                'world_frame_confirmed': receipt.get('HOPE_VRPN_WORLD_FRAME_CONFIRMED') == 'true',
                'packetizer_running': any(p['kind'] == 'packetizer' for p in processes)},
            'display_active': r.stdout.strip() == 'active' and not errors}


if __name__ == '__main__':
    print(json.dumps(probe(read_receipt(Path.home()/'.config/hope-foxglove/laptop-runtime.env'))))
