from dataclasses import replace
import json
from types import SimpleNamespace
from unittest.mock import Mock
import pytest
from hope_lifecycle_core import LifecycleConfig
from hope_connection_identity import control_connections, validate_network_migration

IDENTITY = {'machine_id': 'a' * 32, 'boot_id': 'b' * 36,
            'connection': {'hdu_ip': '192.168.200.202', 'laptop_ip': '192.168.100.89'}}


def test_established_control_pairs_are_deduplicated_and_exclude_other_ports():
    rows = 'header\n0: CAC8A8C0:223E 5964A8C0:ABCD 01\n1: CAC8A8C0:223E 5964A8C0:ABCE 01\n'
    assert control_connections(rows) == [IDENTITY['connection']]
    assert not control_connections(rows.replace('223E', '223D'))
    assert not control_connections(rows.replace(' 01', ' 0A'))


@pytest.fixture
def migration(monkeypatch):
    monkeypatch.setenv('HOPE_LAPTOP_MACHINE_ID', 'c' * 32)
    monkeypatch.setenv('HOPE_LAPTOP_USER', 'user')
    current = LifecycleConfig(hdu_wifi_ip='192.168.8.196', laptop_wifi_ip='192.168.8.250', revision=4)
    updated = replace(current, hdu_wifi_ip='192.168.200.202', laptop_wifi_ip='192.168.100.89')
    proof = {'machine_id': 'a' * 32, 'boot_id': 'b' * 36, 'revision': 4}
    run = Mock(return_value=SimpleNamespace(stdout='c' * 32 + '\n'))
    return current, updated, proof, run


def test_same_machine_new_network_verifies_registered_operator(migration):
    current, updated, proof, run = migration
    validate_network_migration(current, updated, json.dumps(proof), observed=IDENTITY, run=run)
    assert 'StrictHostKeyChecking=yes' in run.call_args.args[0]
    assert 'user@192.168.100.89' in run.call_args.args[0]


@pytest.mark.parametrize('change', [{'machine_id': 'd' * 32}, {'boot_id': 'e' * 36},
                                  {'revision': 3}, {'revision': True}, {'extra': 'x'}])
def test_stale_or_other_robot_proof_never_contacts_operator(migration, change):
    current, updated, proof, run = migration
    with pytest.raises(ValueError):
        validate_network_migration(current, updated, json.dumps({**proof, **change}), observed=IDENTITY, run=run)
    run.assert_not_called()


def test_ambiguous_connection_and_arbitrary_address_refused(migration):
    current, updated, proof, run = migration
    for observed, candidate in [({**IDENTITY, 'connection': None}, updated),
                                (IDENTITY, replace(updated, hdu_wifi_ip='192.168.200.203'))]:
        with pytest.raises(ValueError):
            validate_network_migration(current, candidate, json.dumps(proof), observed=observed, run=run)
    run.assert_not_called()


def test_wrong_operator_identity_refused(migration):
    current, updated, proof, run = migration
    run.return_value.stdout = 'd' * 32
    with pytest.raises(ValueError, match='registered operator'):
        validate_network_migration(current, updated, json.dumps(proof), observed=IDENTITY, run=run)


def test_mac_takeover_requires_authenticated_route_and_preserves_linux_return(migration):
    current, linux, proof, run = migration
    mac = replace(linux, operator_profile='mac')
    run.return_value.stdout = json.dumps(IDENTITY['connection'])
    validate_network_migration(current, mac, json.dumps(proof), observed=IDENTITY, run=run)
    assert run.call_args.args[0][-3:] == ['operator@hope-mac-operator',
                                          '/home/operator/.local/libexec/hope-lifecycle', 'operator-route']
    run.return_value.stdout = json.dumps({**IDENTITY['connection'], 'laptop_ip': '192.168.100.2'})
    with pytest.raises(ValueError, match='route differs'):
        validate_network_migration(current, mac, json.dumps(proof), observed=IDENTITY, run=run)
    run.return_value.stdout = 'c' * 32
    validate_network_migration(replace(current, operator_profile='mac'), linux, json.dumps(proof),
                               observed=IDENTITY, run=run)
    assert 'user@192.168.100.89' in run.call_args.args[0]
