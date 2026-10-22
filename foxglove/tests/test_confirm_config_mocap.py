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

def test_running_robot_never_reconfigured(node):
    node._state='RUNNING'
    assert not confirm(node,vrpn_ip='192.168.100.100').results[0].successful
    node._backend.reconnect_mocap.assert_not_called()
    assert not module.CONFIG_PATH.exists()
