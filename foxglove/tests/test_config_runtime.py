"""Offline contracts: no SSH, ROS, real systemctl, or robot operations."""
from dataclasses import replace
import json
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'a3'))
import hope_config_runtime as m
from hope_lifecycle_core import LifecycleConfig
from hope_config_stage import wrap_unit, metadata_env, stage

@pytest.fixture
def config():
    return LifecycleConfig(laptop_wifi_ip='192.168.8.150', hdu_wifi_ip='192.168.8.182',
                           mdu_internal_ip='10.42.10.12', table_side='P1', revision=54)

def report(config):
    return dict(boot_id=m.BOOT.read_text().strip(), monotonic_s=100, config_sha256=m.identity(config),
                applied=True, state='APPLIED', reason='matched', laptop={'matches':True},
                services=[dict(service=u, matches=True, active='active') for u in m.UNITS])

def test_environment_overrides_legacy_peer_and_bootstrap_can_serve_ui(config):
    env=m.environment(config, {'ROS_STATIC_PEERS':'192.168.8.194', 'KEEP':'yes'})
    assert env['ROS_STATIC_PEERS']=='192.168.8.150;10.42.10.12' and env['KEEP']=='yes'
    assert m.environment(LifecycleConfig(), env)['ROS_STATIC_PEERS']==''

@pytest.mark.parametrize('mutation', [lambda r:r.update(monotonic_s=80), lambda r:r.update(monotonic_s=102),
    lambda r:r.update(boot_id='previous'), lambda r:r.update(config_sha256='old'),
    lambda r:r.update(services=[]), lambda r:r['services'][0].update(matches=False),
    lambda r:r.update(applied='true'), lambda r:r['laptop'].update(matches=False)])
def test_stale_malformed_or_mixed_report_never_applied(config,tmp_path,mutation):
    path=tmp_path/'status.json'; value=report(config); mutation(value); path.write_text(json.dumps(value))
    assert not m.read_status(config,path,101)['applied']

def test_current_report_atomic_values(config,tmp_path):
    path=tmp_path/'status.json'; path.write_text(json.dumps(report(config)))
    status=m.read_status(config,path,101)
    assert status['applied'] and status['revision']==54 and status['values']['table_side']=='P1'
    changed=replace(config,table_side='P2',revision=55)
    status=m.read_status(changed,path,101)
    assert not status['applied'] and status['values']['table_side']=='P2'

def process(proc,pid,env,children=''):
    p=proc/str(pid); (p/'task'/str(pid)).mkdir(parents=True)
    (p/'environ').write_bytes(b'\0'.join(f'{k}={v}'.encode() for k,v in env.items()))
    (p/'stat').write_text(f'{pid} (name with spaces) '+' '.join(['0']*30))
    (p/'task'/str(pid)/'children').write_text(children)

def test_actual_child_old_peers_not_hidden_by_wrapper(config,tmp_path):
    good=m.environment(config,{})
    process(tmp_path,100,good,'101'); process(tmp_path,101,dict(good,ROS_STATIC_PEERS='192.168.8.194;10.42.10.12'))
    run=Mock(return_value=SimpleNamespace(stdout='LoadState=loaded\nActiveState=active\nMainPID=100\n'))
    result=m.service_status(m.UNITS[0],config,run,tmp_path)
    assert not result['matches'] and result['processes'][1]['actual_peers'].startswith('192.168.8.194')

def test_dds_profile_override_is_audited(config,tmp_path):
    xml=tmp_path/'dds.xml'; xml.write_text('<profiles><initialPeersList><address>192.168.8.194</address></initialPeersList></profiles>')
    process(tmp_path,100,m.environment(config,{'FASTRTPS_DEFAULT_PROFILES_FILE':str(xml)}))
    assert not m.process_status(100,config,tmp_path)['matches']

def owner(config,**kw):
    return dict(boot_id='test',monotonic_s=100,config_sha256=m.identity(config),state='STOPPED',busy=False,**kw)

@pytest.mark.parametrize('state',['RUNNING','STARTING','FAILED','KILLING','CONFIG_ERROR'])
def test_running_states_forbid_refresh(config,state):
    o=owner(config); o['state']=state
    assert not m.can_reconcile(o,{'state':'IDLE'},101,'test',m.identity(config))

@pytest.mark.parametrize('change',[{'busy':True},{'monotonic_s':95},{'monotonic_s':102},{'boot_id':'old'},{'config_sha256':'old'}])
def test_owner_must_be_fresh_idle_and_same_generation(config,change):
    o=owner(config); o.update(change)
    assert not m.can_reconcile(o,{'state':'COMPLETE'},101,'test',m.identity(config))

def test_calibration_running_forbids_refresh(config):
    assert not m.can_reconcile(owner(config),{'state':'RUNNING'},101,'test',m.identity(config))
    assert m.can_reconcile(owner(config),{'state':'COMPLETE'},101,'test',m.identity(config))

@pytest.fixture
def reconciliation(config,tmp_path,monkeypatch):
    o=tmp_path/'owner'; c=tmp_path/'cal'; o.write_text(json.dumps(owner(config))); c.write_text('{"state":"IDLE"}')
    monkeypatch.setattr(m,'OWNER',o); monkeypatch.setattr(m,'CALIBRATION',c)
    monkeypatch.setattr(m,'time',SimpleNamespace(monotonic=lambda:101))
    monkeypatch.setattr(m,'load_config',Mock(return_value=config))
    monkeypatch.setattr(m,'no_control_processes',Mock(return_value=True))
    monkeypatch.setattr(m,'no_mdu_control_processes',Mock(return_value=True))
    acquire=Mock(return_value=object()); release=Mock()
    monkeypatch.setattr(m,'try_acquire_hardware_operation_lock',acquire)
    monkeypatch.setattr(m,'release_hardware_operation_lock',release)
    r=report(config); r['boot_id']='test'; r['services'][0]['matches']=False
    return SimpleNamespace(report=r,acquire=acquire,release=release,owner=o,cal=c)

@pytest.mark.parametrize('block',['lock','control','mdu','generation','calibration','owner'])
def test_reconcile_cannot_touch_services_when_blocked(config,reconciliation,block):
    v=reconciliation
    if block=='lock':v.acquire.return_value=None
    elif block=='control':m.no_control_processes.return_value=False
    elif block=='mdu':m.no_mdu_control_processes.return_value=False
    elif block=='generation':m.load_config.return_value=replace(config,revision=55)
    elif block=='calibration':v.cal.write_text('{"state":"RUNNING"}')
    else:v.owner.write_text(json.dumps(dict(owner(config),state='RUNNING')))
    run=Mock(); m.reconcile(config,v.report,run); run.assert_not_called()

def test_only_active_stale_allowlisted_unit_and_once_per_generation(config,reconciliation):
    v=reconciliation; v.report['services'][1].update(matches=False,active='inactive')
    attempts=set(); run=Mock(); m.reconcile(config,v.report,run,attempts); m.reconcile(config,v.report,run,attempts)
    run.assert_called_once_with(['systemctl','try-restart',m.UNITS[0]],check=True,timeout=30)
    assert v.release.call_count==2

def test_laptop_empty_process_list_never_matches(config):
    assert not m.laptop_matches(config,{'receipt':{},'processes':[],'display_active':True})['matches']

def test_unit_wrapper_preserves_colon_and_shell_variables():
    text='[Service]\nExecStart=:/bin/bash -lc \'echo "${ROS_STATIC_PEERS}" && \\\n exec /bin/program\'\n'
    result=wrap_unit(text)
    assert 'ExecStart=:/usr/bin/python3' in result and '${ROS_STATIC_PEERS}' in result and 'exec /bin/program' in result
    with pytest.raises(ValueError):wrap_unit('[Service]\nExecStart=/one\nExecStart=/two\n')
    assert metadata_env('ROS_STATIC_PEERS=old\nHOPE_LAPTOP_USER=user\n')=='HOPE_LAPTOP_USER=user\n'

def test_staging_only_writes_requested_output(tmp_path):
    source=tmp_path/'input'; source.mkdir()
    for unit in m.UNITS:(source/unit).write_text('[Service]\nExecStart=/bin/program\n')
    (source/'network.env').write_text('ROS_STATIC_PEERS=old\n')
    out=tmp_path/'out'; stage(source,out)
    assert len(list(out.glob('*.service.d/*.conf')))==len(m.UNITS)
    assert (source/'network.env').read_text()=='ROS_STATIC_PEERS=old\n'
    with pytest.raises(FileExistsError):stage(source,out)


@pytest.mark.parametrize('code,expected',[(0,False),(1,True),(2,False),(255,False)])
def test_mdu_running_or_unreachable_is_not_idle(config,code,expected):
    run=Mock(return_value=SimpleNamespace(returncode=code))
    assert m.no_mdu_control_processes(config,run) is expected



def test_legacy_env_has_no_independent_robot_or_mocap_values():
    from hope_config_stage import laptop_compat_env
    output=laptop_compat_env('export VRPN_IP=192.168.100.111\nexport HDU_WIFI_IP=192.168.8.196\nHOPE_ROOT=/workspace\n')
    assert '192.168.' not in output and 'HOPE_ROOT=/workspace' in output
    assert 'hope-config-compat' in output


def test_kernel_without_children_file_still_audits_real_descendants(config,tmp_path):
    good=m.environment(config,{})
    process(tmp_path,100,good); process(tmp_path,101,dict(good,ROS_STATIC_PEERS='192.168.8.194'))
    for pid,parent in [(100,1),(101,100)]:
        (tmp_path/str(pid)/'task'/str(pid)/'children').unlink()
        (tmp_path/str(pid)/'status').write_text(f'Name:\tpython3\nPPid:\t{parent}\n')
    run=Mock(return_value=SimpleNamespace(stdout='LoadState=loaded\nActiveState=active\nMainPID=100\n'))
    result=m.service_status(m.UNITS[0],config,run,tmp_path)
    assert not result['matches'] and {p['pid'] for p in result['processes']}=={100,101}
