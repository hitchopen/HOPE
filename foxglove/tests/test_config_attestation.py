"""Configuration/attestation boundaries without ROS, services, or robot IO."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
from dataclasses import replace
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'a3'))
from hope_lifecycle_core import (LifecycleConfig, V6_CONFIG_FIELDS, apply_config_updates,
                                 config_to_document, config_from_document)
from hope_config_stage import migrate_attested_config, attestation_free_env


def config():
    return LifecycleConfig(laptop_wifi_ip='192.168.8.150', hdu_wifi_ip='192.168.8.182',
        mdu_internal_ip='10.42.10.12', motive_ip='192.168.100.99', vrpn_ip='192.168.100.99',
        mocap_source='vrpn', pants_profile='v4', table_side='P1', revision=54,
        vrpn_pelvis_asset_confirmed='true', vrpn_world_frame_confirmed='true')


@pytest.mark.parametrize('key,value', [('hdu_wifi_ip','192.168.8.183'),('mocap_source','none'),
    ('motive_ip','192.168.100.98'),('vrpn_ip','192.168.100.98'),('vrpn_port','3884'),
    ('vrpn_units','m'),('pants_profile','v2'),('table_side','P2')])
@pytest.mark.parametrize('legacy', [False, True])
def test_setup_change_cannot_carry_old_confirmations(key, value, legacy):
    current = config()
    values = {**current.values(), key:value}
    if legacy: values = {k:v for k,v in values.items() if k in V6_CONFIG_FIELDS}
    saved = apply_config_updates(current, list(values.items()))
    assert saved.vrpn_pelvis_asset_confirmed == saved.vrpn_world_frame_confirmed == 'false'
    assert saved.revision == 55
    # Attestation is a separate save against the already saved binding.
    if saved.mocap_source == 'vrpn':
        attested = apply_config_updates(saved, list({**saved.values(),
            'vrpn_pelvis_asset_confirmed':'true','vrpn_world_frame_confirmed':'true'}.items()))
        assert attested.vrpn_world_frame_confirmed == 'true'


@pytest.mark.parametrize('key,value', [('laptop_wifi_ip','192.168.8.151'),('mdu_internal_ip','10.42.10.13')])
def test_transport_change_preserves_physical_setup_confirmation(key,value):
    current = config()
    saved = apply_config_updates(current,list({**current.values(), key:value}.items()))
    assert saved.vrpn_pelvis_asset_confirmed == saved.vrpn_world_frame_confirmed == 'true'


def test_old_schema_never_invents_confirmation_and_schema7_requires_valid_flags():
    document = config_to_document(config())
    assert config_from_document(document) == config()
    assert config_from_document({**document,'schema_version':6}).vrpn_world_frame_confirmed == 'false'
    for value in [True,1,'yes','',None]:
        with pytest.raises(ValueError):
            config_from_document({**document,'vrpn_world_frame_confirmed':value})
    document.pop('operator_profile')
    with pytest.raises(ValueError): config_from_document(document)


def test_helper_arguments_override_retired_environment_flags():
    helper = (Path(__file__).resolve().parents[1]/'helpers/hope-lifecycle').read_text()
    funcs = helper[helper.index('valid_ipv4() {'):helper.index('require_user() {')]
    args = ['model21800_20261109T010203Z','192.168.8.150','192.168.8.182','10.42.10.12',
            '192.168.100.99','P1','OPTITRACK','DEFAULT','legacy','vrpn','192.168.100.99','3883','mm','v4']
    for extra, expected in [([], 'false,false'),(['false','true'],'false,true'),(['true','true'],'true,true')]:
        r = subprocess.run(['bash','-c',funcs+'\nparse_common "$@" || exit 1; printf "%s,%s" "$HOPE_VRPN_PELVIS_ASSET_CONFIRMED" "$HOPE_VRPN_WORLD_FRAME_CONFIRMED"',
                            'test',*args,*extra],capture_output=True,text=True,
                           env={**os.environ,'HOPE_VRPN_PELVIS_ASSET_CONFIRMED':'true','HOPE_VRPN_WORLD_FRAME_CONFIRMED':'true'})
        assert r.returncode == 0, r.stderr
        assert r.stdout == expected


def migration_inputs():
    c = config()
    binding = {'HDU_IP':c.hdu_wifi_ip,'MOTIVE_IP':c.motive_ip,'TABLE_SIDE':c.table_side,
        'PANTS_PROFILE':c.pants_profile,'MOCAP_SOURCE':c.mocap_source,'VRPN_IP':c.vrpn_ip,
        'VRPN_PORT':c.vrpn_port,'VRPN_UNITS':c.vrpn_units}
    receipt = ''.join(f'HOPE_LIFECYCLE_{k}={v}\n' for k,v in binding.items())
    receipt += 'HOPE_VRPN_PELVIS_ASSET_CONFIRMED=true\nHOPE_VRPN_WORLD_FRAME_CONFIRMED=true\n'
    receipt = receipt.encode()
    evidence = {'status':'REPAIRED_MANAGED_PASSIVE_VERIFIED',
        'configuration_changes':[{'path':'/saved/laptop-runtime.env','after_sha256':hashlib.sha256(receipt).hexdigest()}],
        'operator_confirmation':{'verification_type':'operator_visual_attestation; VRPN geometry not independently verified',
            'mocap_operator_response_followup':'看着都是对的','expected_profile':'v4','expected_side':'P1',
            'expected_body':'UCB_P1','expected_asset_sha256':'a'*64,'expected_world':'table z=0'}}
    document = {**config_to_document(c),'schema_version':6}
    return document,receipt,evidence


def test_migration_preserves_evidenced_confirmation_and_requires_exact_sources():
    document,receipt,evidence = migration_inputs()
    def migrate(doc=document, raw=receipt, ev=evidence):
        return migrate_attested_config(json.dumps(doc).encode(),raw,json.dumps(ev).encode())
    candidate = migrate()
    assert candidate['config']['revision'] == 55
    assert candidate['config']['vrpn_world_frame_confirmed'] == 'true'
    assert candidate['activation_requires_source_hash_match'] is True
    for key,value in [('hdu_wifi_ip','192.168.8.183'),('vrpn_ip','192.168.100.98'),('table_side','P2')]:
        with pytest.raises(ValueError): migrate(doc={**document,key:value})
    with pytest.raises(ValueError): migrate(raw=receipt+b'# changed\n')
    with pytest.raises(ValueError): migrate(ev={**evidence,'operator_confirmation':{}})


def test_retired_env_flags_removed_without_touching_hardware_metadata():
    assert attestation_free_env('HOPE_HARDWARE=A3_P1D0\nexport HOPE_VRPN_WORLD_FRAME_CONFIRMED=true\nHOPE_VRPN_PELVIS_ASSET_CONFIRMED=true\n') == 'HOPE_HARDWARE=A3_P1D0\n'


def test_migration_allows_only_session_change_after_recorded_attestation():
    document,raw,evidence = migration_inputs()
    raw = b'HOPE_LIFECYCLE_SESSION_ID=model21800_20261109T102209Z\n' + raw
    evidence['configuration_changes'][0]['after_sha256'] = hashlib.sha256(raw).hexdigest()
    current = raw.replace(b'102209Z',b'104408Z')
    result = migrate_attested_config(json.dumps(document).encode(),raw,json.dumps(evidence).encode(),current)
    assert result['source_sha256']['laptop-runtime.env'] == hashlib.sha256(current).hexdigest()
    with pytest.raises(ValueError):
        migrate_attested_config(json.dumps(document).encode(),raw,json.dumps(evidence).encode(),current.replace(b'192.168.100.99',b'192.168.100.98'))
