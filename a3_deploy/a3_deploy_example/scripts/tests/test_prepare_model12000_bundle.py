from pathlib import Path
import sys

import pytest
import yaml

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))
import prepare_model12000_bundle as bundle


@pytest.fixture
def receipt(tmp_path):
    csv = tmp_path / bundle.CSV_NAME
    csv.write_bytes(b'validated CSV')
    evidence = {'policy_sha256': bundle.POLICY_SHA, 'deploy_sha256': bundle.DEPLOY_SHA,
                'csv_sha256': bundle.digest(csv), 'cases': {}}
    for name in ('normal', 'kernel', 'normal_yaw', 'pitch_plus', 'pitch_minus'):
        evidence['cases'][name] = {'status': 'PASS', 'clock': 'simulation', 'cycles': [{}, {}, {}]}
    return csv, evidence


def test_receipt_binds_exact_csv_bytes(receipt):
    csv, evidence = receipt
    bundle.validate_receipt(evidence, csv)
    csv.write_bytes(b'another model CSV')
    with pytest.raises(ValueError, match='bytes'):
        bundle.validate_receipt(evidence, csv)


@pytest.mark.parametrize('key', ['policy_sha256', 'deploy_sha256'])
def test_other_model_validation_is_not_reusable(receipt, key):
    csv, evidence = receipt
    evidence[key] = 'a' * 64
    with pytest.raises(ValueError, match='bytes'):
        bundle.validate_receipt(evidence, csv)


@pytest.mark.parametrize('change', ['failed', 'missing', 'wall-clock', 'single-cycle'])
def test_incomplete_or_failed_simulation_cannot_enable_serve(receipt, change):
    csv, evidence = receipt
    if change == 'failed': evidence['cases']['normal']['status'] = 'FAIL'
    if change == 'missing': del evidence['cases']['pitch_minus']
    if change == 'wall-clock': evidence['cases']['normal']['clock'] = 'wall_clock_unqualified'
    if change == 'single-cycle': evidence['cases']['normal']['cycles'] = [{}]
    with pytest.raises(ValueError):
        bundle.validate_receipt(evidence, csv)


def test_arm_runtime_restoration_preserves_other_settings(tmp_path):
    config = tmp_path / 'config/a3_runtime_config.pingpong.hitter_pingpong.yaml'
    config.parent.mkdir()
    config.write_text(yaml.safe_dump({'background_cpu': -1, 'driver_cpu': -1, 'backend': {'sync_cpu': -1, 'type': 'iceoryx'}, 'unrelated': 123}))
    policy = tmp_path / 'policy.onnx';policy.write_bytes(b'unchanged model')
    bundle.restore_hardware_runtime(tmp_path)
    actual = yaml.safe_load(config.read_text())
    assert (actual['background_cpu'], actual['backend']['sync_cpu'], actual['driver_cpu']) == (4, 5, 6)
    assert actual['serve_release_lead_frames'] == 3
    assert actual['unrelated'] == 123 and actual['backend']['type'] == 'iceoryx'
    assert policy.read_bytes() == b'unchanged model'
