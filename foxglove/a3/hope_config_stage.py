#!/usr/bin/env python3
"""Prepare drop-ins from captured `systemctl cat` output. Never installs/restarts.

This deliberately takes saved unit text: preparing a bundle during a field test
must not need access to the robot. Activation must recheck each source hash.
"""
import argparse
import hashlib
import json
import re
from pathlib import Path
from hope_config_runtime import UNITS
from hope_lifecycle_core import config_from_document, config_to_document
from dataclasses import replace
import shlex

WRAPPER = '/usr/bin/python3 /usr/local/lib/hope-foxglove/hope_config_runtime.py exec -- '


def wrap_unit(text):
    section, command, logical = '', None, ''
    for raw in text.splitlines():
        line = raw.strip()
        if line.startswith(('#', ';')):
            continue
        logical += line[:-1] + ' ' if line.endswith('\\') else line
        if line.endswith('\\'):
            continue
        line, logical = logical, ''
        if line.startswith('[') and line.endswith(']'):
            section = line
        elif section == '[Service]' and line.startswith('ExecStart='):
            value = line.split('=', 1)[1].strip()
            if not value:
                command = None
            elif command is not None:
                raise ValueError('Multiple ExecStart commands require manual review')
            else:
                command = value
    if logical or not command:
        raise ValueError('Missing or incomplete ExecStart')
    if 'hope_config_runtime.py exec -- ' in command:
        raise ValueError('Unit already wrapped; use the saved pre-migration unit')
    # ':' preserves shell variables in the original mocap service; other
    # systemd command prefixes have different semantics and are not guessed.
    prefix = ':' if command.startswith(':') else ''
    original = command[1:] if prefix else command
    if not original.startswith('/'):
        raise ValueError('Unsupported ExecStart prefix or executable')
    return '[Service]\nExecStart=\nExecStart=' + prefix + WRAPPER + original + '\n'


def metadata_env(text):
    # Addresses only live in config.json. Retain installation paths/usernames.
    return '\n'.join(line for line in text.splitlines()
                     if not line.lstrip().startswith('ROS_STATIC_PEERS=')) + '\n'


def attestation_free_env(text):
    """Remove retired editable flags only; preserve site/hardware metadata."""
    return '\n'.join(line for line in text.splitlines() if not re.match(
        r'\s*(?:export\s+)?HOPE_VRPN_(?:PELVIS_ASSET|WORLD_FRAME)_CONFIRMED=', line)) + '\n'


def migrate_attested_config(config_bytes, runtime_bytes, evidence_bytes, current_runtime_bytes=None):
    """Offline migration of an existing operator attestation, never a new one.

    Returns a candidate and source hashes which MUST be checked again before
    activation. Only a session-id change is allowed after the attested snapshot;
    all other receipt bytes must match and both snapshots are hash recorded.
    Nothing is installed and no robot/service actions are performed here.
    """
    document = json.loads(config_bytes)
    if document.get('schema_version') != 6:
        raise ValueError('Expected the captured schema-6 config')
    config = config_from_document(document)
    evidence = json.loads(evidence_bytes)
    receipt = {}
    for line in runtime_bytes.decode().splitlines():
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        key, value = line.split('=', 1)
        tokens = shlex.split(value)
        if key in receipt or len(tokens) > 1:
            raise ValueError('Ambiguous runtime receipt')
        receipt[key] = tokens[0] if tokens else ''
    receipt_hash = hashlib.sha256(runtime_bytes).hexdigest()
    changes = evidence.get('configuration_changes', [])
    if not any(row.get('path', '').endswith('/laptop-runtime.env') and
               row.get('after_sha256') == receipt_hash for row in changes):
        raise ValueError('Runtime receipt is not the attested snapshot')
    current_runtime_bytes = runtime_bytes if current_runtime_bytes is None else current_runtime_bytes
    def without_session(data):
        lines = data.decode().splitlines(keepends=True)
        sessions = [line for line in lines if line.startswith('HOPE_LIFECYCLE_SESSION_ID=')]
        if len(sessions) > 1 or any(not re.fullmatch(
                r'HOPE_LIFECYCLE_SESSION_ID=model21800_\d{8}T\d{6}Z\n?', line) for line in sessions):
            raise ValueError('Invalid runtime session')
        return [line for line in lines if not line.startswith('HOPE_LIFECYCLE_SESSION_ID=')]
    if without_session(runtime_bytes) != without_session(current_runtime_bytes):
        raise ValueError('Current receipt differs from the attested setup')
    attestation = evidence.get('operator_confirmation', {})
    if (evidence.get('status') != 'REPAIRED_MANAGED_PASSIVE_VERIFIED' or
            attestation.get('verification_type') != 'operator_visual_attestation; VRPN geometry not independently verified' or
            not attestation.get('mocap_operator_response_followup') or
            attestation.get('expected_profile') != config.pants_profile or
            attestation.get('expected_side') != config.table_side or
            attestation.get('expected_body') != 'UCB_' + config.table_side or
            not re.fullmatch('[0-9a-f]{64}', attestation.get('expected_asset_sha256', '')) or
            not attestation.get('expected_world') or config.mocap_source != 'vrpn'):
        raise ValueError('Operator attestation does not describe this setup')
    binding = {'HDU_IP': config.hdu_wifi_ip, 'MOTIVE_IP': config.motive_ip,
               'TABLE_SIDE': config.table_side, 'PANTS_PROFILE': config.pants_profile,
               'MOCAP_SOURCE': config.mocap_source, 'VRPN_IP': config.vrpn_ip,
               'VRPN_PORT': config.vrpn_port, 'VRPN_UNITS': config.vrpn_units}
    if any(receipt.get('HOPE_LIFECYCLE_' + key) != value for key, value in binding.items()):
        raise ValueError('Config differs from the attested receiver binding')
    if any(receipt.get(key) != 'true' for key in
           ('HOPE_VRPN_PELVIS_ASSET_CONFIRMED', 'HOPE_VRPN_WORLD_FRAME_CONFIRMED')):
        raise ValueError('Existing receiver is not attested')
    migrated = replace(config, revision=config.revision + 1,
                       vrpn_pelvis_asset_confirmed='true', vrpn_world_frame_confirmed='true')
    return {'config': config_to_document(migrated), 'activation_requires_source_hash_match': True,
            'source_sha256': {name: hashlib.sha256(data).hexdigest() for name, data in
                              [('config.json', config_bytes), ('laptop-runtime.env', current_runtime_bytes),
                               ('attested-runtime.env', runtime_bytes), ('attestation.json', evidence_bytes)]},
            'attestation': attestation, 'state': 'PREPARED_NOT_INSTALLED'}


def laptop_compat_env(text):
    aliases = {'LAPTOP_WIFI_IP','HDU_WIFI_IP','LAPTOP_WIRED_IP','HDU_WIRED_IP','MDU_INTERNAL_IP',
               'VRPN_IP','VRPN_PORT','MOTIVE_IP','ROBOT_IF','ROBOT_WIRED_IF','VRPN_IF','LAPTOP_VRPN_IP'}
    kept = []
    for line in text.splitlines():
        match = re.match(r"\s*(?:export\s+)?([A-Z_]+)=", line)
        if not match or match.group(1) not in aliases:
            kept.append(line)
    kept += ['# Derived aliases; configure addresses in Foxglove.',
             'source "$HOME/.config/hope-foxglove/laptop-runtime.env"',
             'source "$HOPE_LIFECYCLE_ROOT/foxglove/helpers/hope-config-compat"']
    return '\n'.join(kept) + '\n'


def stage(source, output):
    records, payload = {}, {}
    for unit in UNITS:
        raw = (source / unit).read_bytes()
        records[unit] = hashlib.sha256(raw).hexdigest()
        payload[unit + '.d/40-authoritative-config.conf'] = wrap_unit(raw.decode())
    raw = (source / 'network.env').read_bytes()
    records['network.env'] = hashlib.sha256(raw).hexdigest()
    payload['network.env'] = metadata_env(raw.decode())
    # No partial directory on validation failure; never overwrite an old stage.
    output.mkdir(parents=True, exist_ok=False)
    for name, text in payload.items():
        path = output / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)
    (output / 'source-sha256.json').write_text(json.dumps(records, indent=2) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--captured-unit-dir', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    stage(args.captured_unit_dir, args.output)
