"""Network recovery must never become a clock action or interrupt one."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'a3'))
import hope_time_calibration_core as core

UP = ((5, 'wifi_hdu', True, '192.168.8.182'),)
DOWN = ((5, 'wifi_hdu', False, '192.168.8.182'),)


def test_boot_waits_for_late_address_before_initializing_dds():
    clock = [0.0]
    read = Mock(side_effect=[(), DOWN, UP])
    result = core.wait_for_calibration_network('192.168.8.182', read=read,
        clock=lambda: clock[0], sleep=lambda dt: clock.__setitem__(0, clock[0]+dt))
    assert result == UP and clock[0] == 1.0


def test_missing_network_has_bounded_wait_and_recovers_on_later_arrival():
    clock = [0.0]
    signature = core.wait_for_calibration_network('192.168.8.182', timeout_s=2,
        read=lambda: (), clock=lambda: clock[0],
        sleep=lambda dt: clock.__setitem__(0, clock[0]+dt))
    assert signature == () and clock[0] == 2.0
    watch = core.CalibrationNetworkWatch(signature)
    assert not watch.needs_reconnect(UP, 10)
    assert watch.needs_reconnect(UP, 13)


def test_same_ip_return_is_not_forgotten_and_flaps_are_debounced():
    watch = core.CalibrationNetworkWatch(UP)
    assert not watch.needs_reconnect(UP, 0)
    assert not watch.needs_reconnect(DOWN, 1)
    assert not watch.needs_reconnect(UP, 2)
    assert not watch.needs_reconnect(DOWN, 3)
    assert not watch.needs_reconnect(UP, 4)
    assert not watch.needs_reconnect(UP, 6.9)
    assert watch.needs_reconnect(UP, 7)


def test_bad_probe_does_not_trigger_reconnect():
    watch = core.CalibrationNetworkWatch(UP)
    assert not watch.needs_reconnect(None, 10)
    assert not watch.needs_reconnect(UP, 20)


def test_interface_replacement_with_same_address_requires_reconnect():
    watch = core.CalibrationNetworkWatch(UP)
    current = ((8, 'wifi_hdu', True, '192.168.8.182'),)
    assert not watch.needs_reconnect(current, 10)
    assert watch.needs_reconnect(current, 13)


def test_network_probe_includes_carrier_and_excludes_loopback(monkeypatch):
    data = [dict(ifindex=5, ifname='wifi_hdu', flags=['UP'],
                 addr_info=[dict(family='inet', scope='global', local='192.168.8.182')]),
            dict(ifindex=1, ifname='lo', flags=['UP','LOWER_UP'],
                 addr_info=[dict(family='inet', scope='host', local='127.0.0.1')])]
    monkeypatch.setattr(core.subprocess, 'run', Mock(return_value=SimpleNamespace(stdout=json.dumps(data))))
    assert core.read_calibration_network() == DOWN
    data[0]['flags'].append('LOWER_UP')
    core.subprocess.run.return_value.stdout = json.dumps(data)
    assert core.read_calibration_network() == UP


@pytest.mark.parametrize('change', [dict(boot_id='old'), dict(state='RUNNING'), dict(busy=True),
    dict(monotonic_s=8.4), dict(monotonic_s=10.1), dict(monotonic_s=float('nan'))])
def test_owner_stale_active_or_other_boot_cannot_authorize_recycle(tmp_path, change):
    p=tmp_path/'owner.json'
    owner=dict(boot_id='boot',state='STOPPED',busy=False,monotonic_s=10)
    p.write_text(json.dumps(owner))
    assert core.calibration_owner_stopped(p,'boot',10)
    owner.update(change);p.write_text(json.dumps(owner))
    assert not core.calibration_owner_stopped(p,'boot',10)
