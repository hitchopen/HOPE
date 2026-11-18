import sys
import threading
from pathlib import Path
from types import SimpleNamespace
from concurrent.futures import ThreadPoolExecutor
from unittest.mock import Mock
import pytest
pytest.importorskip('rclpy')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'a3'))
import hope_lifecycle_supervisor as module
from hope_lifecycle_core import LifecycleConfig, load_config
from rclpy.parameter import Parameter

@pytest.fixture
def node(tmp_path,monkeypatch):
    monkeypatch.setattr(module,'CONFIG_PATH',tmp_path/'config.json')
    monkeypatch.setattr(module,'try_acquire_hardware_operation_lock',lambda: object())
    monkeypatch.setattr(module,'release_hardware_operation_lock',lambda _: None)
    config=LifecycleConfig(laptop_wifi_ip='192.168.8.155',hdu_wifi_ip='192.168.8.182',mdu_internal_ip='10.42.10.12',motive_ip='192.168.100.99',vrpn_ip='192.168.100.99',mocap_source='vrpn',revision=1)
    with ThreadPoolExecutor(1) as pool:
        yield SimpleNamespace(_lock=threading.Lock(),_busy=False,_field_busy=False,_state='STOPPED',_config=config,_last_result='',_field_pool=pool,_backend=Mock())

def confirm(node, **changes):
    values={**node._config.values(),**changes}
    req=SimpleNamespace(parameters=[Parameter(k,value=v).to_parameter_msg() for k,v in values.items()])
    return module.HopeLifecycleSupervisor._apply_config(node,req,SimpleNamespace())

def test_other_robot_ip_never_saved_or_reconnected(node):
    result=confirm(node,hdu_wifi_ip='192.168.8.183')
    assert not result.results[0].successful
    assert not module.CONFIG_PATH.exists()
    node._backend.reconnect_mocap.assert_not_called()

def test_mocap_save_and_connect_then_reload(node):
    result=confirm(node,vrpn_ip='192.168.100.100',pants_profile='v4')
    assert result.results[0].successful
    node._backend.reconnect_mocap.assert_called_once_with(node._config)
    assert load_config(module.CONFIG_PATH).vrpn_ip=='192.168.100.100'
    assert load_config(module.CONFIG_PATH).pants_profile=='v4'
    assert not node._field_busy

def test_failed_connection_remains_saved_and_retry_reconnects(node):
    node._backend.reconnect_mocap.side_effect=module.LifecycleFailure('OTHER_ROBOT_RECEIVER_UNTOUCHED')
    result=confirm(node,vrpn_ip='192.168.100.100')
    assert not result.results[0].successful
    assert 'Configuration saved' in result.results[0].reason
    assert load_config(module.CONFIG_PATH).vrpn_ip=='192.168.100.100'
    node._backend.reconnect_mocap.side_effect=None
    assert confirm(node).results[0].successful
    assert node._backend.reconnect_mocap.call_count==2

@pytest.mark.parametrize('state', ['RUNNING', 'STARTING', 'FAILED', 'KILLING'])
def test_running_robot_never_reconfigured(node, state):
    node._state=state
    assert not confirm(node,table_side='P2').results[0].successful
    node._backend.reconnect_mocap.assert_not_called()
    assert not module.CONFIG_PATH.exists()


def test_operator_switch_reconnects_even_with_same_addresses(node):
    result = confirm(node, operator_profile='mac')
    assert result.results[0].successful
    node._backend.reconnect_mocap.assert_called_once_with(node._config)
    assert load_config(module.CONFIG_PATH).operator_profile == 'mac'


def test_confirmation_changes_reconnect_even_when_network_is_applied(node,monkeypatch):
    monkeypatch.setattr(module,'read_status',lambda _: {'applied':True})
    result=confirm(node,vrpn_world_frame_confirmed='true')
    assert result.results[0].successful
    node._backend.reconnect_mocap.assert_called_once_with(node._config)
    saved=load_config(module.CONFIG_PATH)
    assert saved.vrpn_world_frame_confirmed == 'true'


def test_unchanged_confirm_repairs_receiver_without_new_revision(node, monkeypatch):
    monkeypatch.setattr(module, 'read_status', lambda _: {'applied': False})
    before = node._config
    result = confirm(node)
    assert result.results[0].successful
    assert result.results[0].reason.startswith('CONFIG_UNCHANGED:')
    node._backend.reconnect_mocap.assert_called_once_with(before)
    assert node._config is before and node._config.revision == 1
    assert not module.CONFIG_PATH.exists()
    assert node._last_result == 'CONFIG_UNCHANGED_REVISION_1'


def test_unchanged_confirm_when_applied_is_a_no_op(node, monkeypatch):
    monkeypatch.setattr(module, 'read_status', lambda _: {'applied': True})
    result = confirm(node)
    assert result.results[0].successful
    node._backend.reconnect_mocap.assert_not_called()
    assert node._config.revision == 1 and not module.CONFIG_PATH.exists()


def test_unchanged_confirm_reconnect_failure_is_reported_and_retried(node, monkeypatch):
    monkeypatch.setattr(module, 'read_status', lambda _: {'applied': True})
    node._last_result = 'CONFIG_SAVED_MOCAP_FAILED: helper failed: UNMANAGED_BRIDGE_PRESENT'
    node._backend.reconnect_mocap.side_effect = module.LifecycleFailure('UNMANAGED_BRIDGE_PRESENT')
    result = confirm(node)
    assert not result.results[0].successful
    assert result.results[0].reason.startswith('CONFIG_UNCHANGED: mocap connection failed')
    assert 'MOCAP_FAILED' in node._last_result and not module.CONFIG_PATH.exists()
    node._backend.reconnect_mocap.side_effect = None
    assert confirm(node).results[0].successful
    assert node._backend.reconnect_mocap.call_count == 2 and node._config.revision == 1


def test_changed_confirm_still_creates_one_new_revision(node, monkeypatch):
    monkeypatch.setattr(module, 'read_status', lambda _: {'applied': True})
    assert confirm(node, gripper_profile='new_20261015').results[0].successful
    assert load_config(module.CONFIG_PATH).revision == 2 and node._config.revision == 2
    node._backend.reconnect_mocap.assert_not_called()


@pytest.mark.parametrize('mode,laptop', [('KERNEL', False), ('OPTITRACK', True)])
def test_start_gate_skips_laptop_receiver_only_for_kernel(node, monkeypatch, mode, laptop):
    calls = []
    monkeypatch.setattr(module, 'require_applied', lambda config, **kw: calls.append(kw.get('laptop', True)))
    submitted = []
    node._field = SimpleNamespace(status=lambda: {'mode': mode})
    node._pool = SimpleNamespace(submit=lambda *args: submitted.append(args))
    node._run_start = object()
    node._session_id = ''
    node._session_mode = 'OPTITRACK'
    node._hardware_operation_lock = None
    node._step = 'IDLE'
    response = module.HopeLifecycleSupervisor._start(node, None, SimpleNamespace())
    assert response.success, response.message
    assert calls == [laptop, laptop]
    assert submitted[0][-1] is (not laptop)


def test_kernel_admitted_start_rechecks_full_audit_if_mode_changed(monkeypatch):
    calls = []
    def require(config, **kw):
        calls.append(kw.get('laptop', True))
        raise ValueError('CONFIG_NOT_APPLIED: laptop receiver absent')
    monkeypatch.setattr(module, 'require_applied', require)
    node = SimpleNamespace(_lock=threading.Lock(), _state='STARTING', _session_id='', _step='', _elink_audit={},
                           _field=SimpleNamespace(runner_selection=lambda config: ('OPTITRACK', 'DEFAULT')),
                           _backend=Mock(), _session_mode='KERNEL', _hardware_operation_lock=None,
                           get_logger=lambda: Mock(), _busy=True, _last_result='')
    try:
        module.HopeLifecycleSupervisor._run_start(node, LifecycleConfig(revision=3), 'model21800_20261118T000000Z',
                                                  kernel_admission=True)
    except Exception:
        pass
    assert calls == [True]
    node._backend.start.assert_not_called()


@pytest.mark.parametrize('previous_mode,laptop', [('OPTITRACK', True), ('KERNEL', False)])
def test_failed_session_cleanup_needs_full_audit_unless_it_was_kernel(node, monkeypatch, previous_mode, laptop):
    calls = []
    monkeypatch.setattr(module, 'require_applied', lambda config, **kw: calls.append(kw.get('laptop', True)))
    node._state = 'FAILED'
    node._field = SimpleNamespace(status=lambda: {'mode': 'KERNEL'})
    node._pool = SimpleNamespace(submit=lambda *args: None)
    node._run_start = object()
    node._session_id = 'model21800_20261118T000000Z'
    node._session_mode = previous_mode
    node._hardware_operation_lock = None
    node._step = 'IDLE'
    response = module.HopeLifecycleSupervisor._start(node, None, SimpleNamespace())
    assert response.success, response.message
    assert calls == [laptop, laptop]
