"""Run lifecycle parsing/dispatch with hardware calls replaced by local probes."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'a3'), str(ROOT.parent / 'hope_ws/src/hope_bringup/scripts')]
from hope_field_operations import FieldOperations
from hope_lifecycle_core import LifecycleConfig, parse_helper_event
from hope_policy_models import handle


@pytest.fixture
def catalogs(tmp_path):
    roots = {h: tmp_path / h for h in ('hdu', 'mdu')}
    for root in roots.values():
        root.mkdir()
        models = {}
        for name in ('model14600', 'model11999', 'model12600', 'model17400'):
            artifact = root / (name + '.bin')
            artifact.write_text(name)
            digest = hashlib.sha256(artifact.read_bytes()).hexdigest()
            models[name] = dict(files={str(artifact): digest}, policy_sha256=digest,
                                deploy_sha256=digest, deployment=str(root / name))
            if name == 'model17400':
                models[name].update(receive_only=True, required_ball_model='ppq004')
        (root / 'catalog.json').write_text(json.dumps(dict(version=1, models=models)))
        (root / 'selected.json').write_text(json.dumps(dict(model='model17400')))
    return roots


def corrupt(root, damage):
    if damage == 'changed':
        (root / 'model17400.bin').write_text('changed')
    elif damage == 'missing':
        (root / 'model17400.bin').unlink()
    elif damage == 'malformed':
        (root / 'catalog.json').write_text('{')
    elif damage == 'unreadable':
        (root / 'catalog.json').unlink()
        (root / 'catalog.json').mkdir()  # fails to read even under root
    elif damage == 'missing_catalog':
        (root / 'catalog.json').unlink()


def shell_helper(tmp_path, root, missing_module=False):
    source = (ROOT / 'helpers/hope-lifecycle').read_text()
    source = source.replace('/agibot/hope-policy-models', str(root))
    source = source.replace('/usr/local/lib/hope-foxglove/hope_policy_models.py',
                            str(tmp_path / 'missing.py' if missing_module else ROOT / 'a3/hope_policy_models.py'))
    helper = tmp_path / 'helper'
    helper.write_text(source)
    return helper


def dispatch(helper, action, tmp_path):
    # Keep the actual parser, dispatcher and stop_mdu; mock only external effects.
    script = '''source "$1"
require_user() { :; }
stop_tmux() { printf 'STOP %s\\n' "$1"; }
sudo() { :; }
systemctl() { :; }
check_elink_process_presence() { :; }
start_runner() { echo START_REACHED; }
main "$2" model21800_20261026T120000Z 192.168.8.194 192.168.8.182 10.42.10.12 NONE P1
'''
    return subprocess.run(['bash', '-c', script, 'probe', str(helper), action],
                          env={**os.environ, 'XDG_CONFIG_HOME': str(tmp_path / 'config')},
                          capture_output=True, text=True, timeout=12)


@pytest.mark.parametrize('damage', ['changed', 'missing', 'malformed', 'unreadable', 'missing_catalog', 'missing_module'])
def test_damaged_bundle_cannot_block_stop_but_blocks_start(tmp_path, catalogs, damage):
    if damage != 'missing_module':
        corrupt(catalogs['mdu'], damage)
    helper = shell_helper(tmp_path, catalogs['mdu'], damage == 'missing_module')
    result = dispatch(helper, 'stop-mdu', tmp_path)
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'STOP hope-runner' in result.stdout
    assert 'STOP hope-hal' in result.stdout
    assert 'AGIBOT_PM_RESTORED' in result.stdout
    assert result.stderr  # integrity problem is still reported separately
    start = dispatch(helper, 'start-runner', tmp_path)
    assert start.returncode != 0
    assert 'START_REACHED' not in start.stdout


def field(tmp_path, catalogs, physics='ppq004'):
    calls = []
    def exchange(host, config, request):
        calls.append((host, request.copy()))
        if request['op'] == 'verify_ball_model':
            return {'configured_model': physics}
        return handle(catalogs[host], request)
    return FieldOperations(tmp_path / 'field', exchange), calls


@pytest.mark.parametrize('saved_state', ['absent', 'migrated'])
@pytest.mark.parametrize('physics,mode,other,error', [
    ('ppq004', 'OPTITRACK', 'model17400', None),
    ('ppq002', 'OPTITRACK', 'model17400', 'PPQ004'),
    ('ppq004', 'KERNEL', 'model17400', 'receive mode'),
    ('ppq002', 'OPTITRACK', 'model12600', 'differ'),
])
def test_start_resolves_installed_policy_without_saved_selection(tmp_path, catalogs, physics, mode, other, error, saved_state):
    (catalogs['mdu'] / 'selected.json').write_text(json.dumps({'model': other}))
    if saved_state == 'migrated':
        (tmp_path / 'field').mkdir()
        (tmp_path / 'field/selection.json').write_text(json.dumps({'mode': mode}))
    manager, calls = field(tmp_path, catalogs, physics)
    manager._data['mode'] = mode
    assert manager.status()['policy_model'] is None
    if error:
        with pytest.raises(ValueError, match=error):
            manager.runner_selection(LifecycleConfig(revision=1))
    else:
        assert manager.runner_selection(LifecycleConfig(revision=1)) == ('OPTITRACK', 'DEFAULT')
        assert manager.status()['policy_model'] == 'model17400'
        assert [h for h, q in calls if q['op'] == 'verify_policy_model'] == ['hdu', 'mdu']


@pytest.mark.parametrize('damage', ['changed', 'missing', 'malformed', 'unreadable', 'missing_catalog'])
def test_fresh_state_rejects_policy_read_failures(tmp_path, catalogs, damage):
    corrupt(catalogs['mdu'], damage)
    manager, _ = field(tmp_path, catalogs)
    with pytest.raises(ValueError, match='mdu.*policy'):
        manager.runner_selection(LifecycleConfig(revision=1))


@pytest.mark.parametrize('absent_hosts,success', [(('hdu', 'mdu'), True), (('mdu',), False)])
def test_only_explicit_two_endpoint_catalog_absence_allows_legacy(tmp_path, catalogs, absent_hosts, success):
    for host in absent_hosts:
        (catalogs[host] / 'catalog.json').unlink()
        (catalogs[host] / 'selected.json').unlink()
    manager, _ = field(tmp_path, catalogs)
    if success:
        assert manager.runner_selection(LifecycleConfig(revision=1)) == ('OPTITRACK', 'DEFAULT')
    else:
        with pytest.raises(ValueError, match='catalog'):
            manager.runner_selection(LifecycleConfig(revision=1))


def test_native_serve_failure_keeps_diagnostics_and_parseable_event(tmp_path):
    source = (ROOT / 'helpers/hope-lifecycle').read_text()
    emit = source[source.index('emit() {'):source.index('valid_ipv4() {')]
    block = source[source.index('  local validation_output\n'):source.index('  if grep -q', source.index('  local validation_output\n'))]
    setup = tmp_path / 'setup.bash'
    setup.write_text(':\n')
    block = block.replace('/opt/ros/jazzy/setup.bash', str(setup))
    script = 'set -euo pipefail\n' + emit + '''
timeout() { printf 'native detail line one\\nnative detail line two\\n' >&2; return 7; }
probe() {
local -a validation_args=(--validate-serve-timeline fake.csv)
MDU_DEPLOY=/unused
''' + block + '\n}\nprobe\n'
    result = subprocess.run(['bash', '-c', script], capture_output=True, text=True, timeout=5)
    assert result.returncode != 0
    assert 'native detail line one\nnative detail line two' in result.stderr
    events = [parse_helper_event(line) for line in result.stdout.splitlines()]
    assert any(e and e.reason == 'SERVE_TIMELINE_INVALID' and e.state == 'FAILED' for e in events)


def test_same_model_different_bundle_hashes_are_rejected(tmp_path, catalogs):
    path = catalogs['mdu'] / 'catalog.json'
    catalog = json.loads(path.read_text())
    catalog['models']['model17400']['deploy_sha256'] = '0' * 64
    path.write_text(json.dumps(catalog))
    manager, _ = field(tmp_path, catalogs)
    with pytest.raises(ValueError, match='bundles differ'):
        manager.runner_selection(LifecycleConfig(revision=1))


def test_fresh_receive_only_selection_ignores_unused_saved_csv(tmp_path, catalogs):
    manager, calls = field(tmp_path, catalogs)
    manager._data['csv'] = {'sha256': 'unused legacy CSV'}
    assert manager.runner_selection(LifecycleConfig(revision=1)) == ('OPTITRACK', 'DEFAULT')
    assert manager.status()['csv'] is None
    assert not any(q['op'] == 'verify_csv' for _, q in calls)


def test_damaged_catalog_link_is_not_legacy_absence(tmp_path, catalogs):
    for root in catalogs.values():
        (root / 'catalog.json').unlink()
        (root / 'selected.json').unlink()
        (root / 'catalog.json').symlink_to(root / 'missing.json')
    manager, _ = field(tmp_path, catalogs)
    with pytest.raises(ValueError, match='catalog missing'):
        manager.runner_selection(LifecycleConfig(revision=1))


def test_valid_and_legacy_deployment_dispatch(tmp_path, catalogs):
    helper = shell_helper(tmp_path, catalogs['mdu'])
    assert 'START_REACHED' in dispatch(helper, 'start-runner', tmp_path).stdout
    stopped = dispatch(helper, 'stop-mdu', tmp_path)
    assert stopped.returncode == 0 and not stopped.stderr
    (catalogs['mdu'] / 'catalog.json').unlink()
    (catalogs['mdu'] / 'selected.json').unlink()
    assert 'START_REACHED' in dispatch(helper, 'start-runner', tmp_path).stdout


@pytest.mark.parametrize('physics,other', [('ppq002', 'model17400'), ('ppq004', 'model12600')])
def test_supervisor_rejects_before_starting_hardware(tmp_path, catalogs, monkeypatch, physics, other):
    pytest.importorskip('rclpy')
    import threading
    from types import SimpleNamespace
    from unittest.mock import Mock
    import hope_lifecycle_supervisor as supervisor
    monkeypatch.setattr(supervisor, 'release_hardware_operation_lock', Mock())
    (catalogs['mdu'] / 'selected.json').write_text(json.dumps({'model': other}))
    manager, _ = field(tmp_path, catalogs, physics)
    node = SimpleNamespace(_lock=threading.Lock(), _field=manager, _backend=Mock(),
                           _on_event=Mock(), _hardware_operation_lock=object(),
                           get_logger=Mock(return_value=Mock()))
    supervisor.HopeLifecycleSupervisor._run_start(
        node, LifecycleConfig(revision=1), 'model21800_20261026T120000Z')
    node._backend.start.assert_not_called()
    assert node._state == 'STOPPED'
    assert node._last_result.startswith('PREFLIGHT_REJECTED:')
