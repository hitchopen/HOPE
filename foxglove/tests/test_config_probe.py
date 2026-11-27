import json
import os
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'laptop'))
from hope_config_probe import read_receipt, probe

@pytest.fixture
def receipt():
    return {'HOPE_LIFECYCLE_HDU_IP':'192.168.8.182','HOPE_LIFECYCLE_LAPTOP_IP':'192.168.8.150',
            **{'HOPE_LIFECYCLE_'+k:v for k,v in dict(TABLE_SIDE='P1',PANTS_PROFILE='v4',MOCAP_SOURCE='vrpn',
               VRPN_IP='192.168.100.99',VRPN_PORT='3883',VRPN_UNITS='mm',MOTIVE_IP='').items()}}

def proc(root,name,env):
    p=root/'100';p.mkdir(); (p/'root').mkdir(); (p/'root'/'params').write_text('/**:\n  ros__parameters:\n    server: 192.168.100.99\n    port: 3883\n')
    (p/'cmdline').write_bytes(name.encode()+b'\0--params-file\0/params\0')
    (p/'environ').write_bytes(b'\0'.join((k+'='+v).encode() for k,v in env.items()))

@pytest.mark.parametrize('bad',[{'ROS_STATIC_PEERS':'192.168.8.194;127.0.0.1'},
                               {'ROS_STATIC_PEERS':'192.168.8.182;192.168.8.196'},
                               {'HOPE_LIFECYCLE_TABLE_SIDE':'P2'},
                               {'HOPE_LIFECYCLE_PANTS_PROFILE':'v2'}])
def test_stale_receiver_or_extra_robot_peer_is_exposed(receipt,tmp_path,bad):
    env=dict(receipt,ROS_STATIC_PEERS='192.168.8.182;127.0.0.1');env.update(bad)
    proc(tmp_path,'/opt/vrpn/client_node',env)
    value=probe(receipt,tmp_path,Mock(return_value=SimpleNamespace(stdout='active\n')))
    assert len(value['processes'])==1 and not value['processes'][0]['matches']

def test_current_receiver_matches(receipt,tmp_path):
    proc(tmp_path,'/opt/vrpn/client_node',dict(receipt,ROS_STATIC_PEERS='192.168.8.182;127.0.0.1'))
    value=probe(receipt,tmp_path,Mock(return_value=SimpleNamespace(stdout='active\n')))
    assert value['processes'][0]['matches']

def test_receipt_is_data_and_duplicates_rejected(tmp_path):
    p=tmp_path/'receipt';p.write_text("# generated\nKEY='a b'\nEMPTY=''\n")
    assert read_receipt(p)=={'KEY':'a b','EMPTY':''}
    p.write_text('KEY=a\nKEY=b\n')
    with pytest.raises(ValueError):read_receipt(p)


def test_effective_params_override_correct_environment(receipt,tmp_path):
    proc(tmp_path,'/opt/vrpn/client_node',dict(receipt,ROS_STATIC_PEERS='192.168.8.182;127.0.0.1'))
    (tmp_path/'100/root/params').write_text('/**:\n  ros__parameters:\n    server: 192.168.100.111\n    port: 3884\n')
    value=probe(receipt,tmp_path,Mock(return_value=SimpleNamespace(stdout='active\n')))
    endpoint=value['processes'][0]['endpoint']
    assert not endpoint['matches'] and endpoint['startup_parameters']=={'server':'192.168.100.111','port':3884}
    assert not value['processes'][0]['matches']


def test_ordered_ros_params_resolve_selected_node_and_cli_override(tmp_path):
    from mocap_config_evidence import startup_parameters
    (tmp_path/'root').mkdir()
    (tmp_path/'root'/'base').write_text('/**:\n  ros__parameters:\n    server: old\n    port: 3883\n/other:\n  ros__parameters:\n    server: wrong_node\n')
    (tmp_path/'root'/'late').write_text('/vrpn_mocap/client_node:\n  ros__parameters:\n    server: current\n')
    params,files=startup_parameters(['--ros-args','-r','__node:=client_node','-r','__ns:=/vrpn_mocap',
       '--params-file','/base','--params-file','/late','-p','port:=3884',''],tmp_path)
    assert params=={'server':'current','port':3884} and len(files)==2


def test_ros2_launch_tuple_params_file_is_read_and_other_python_tags_rejected(tmp_path):
    # ros2 launch writes list parameters with yaml.dump, i.e. as !!python/tuple.
    import yaml
    from mocap_config_evidence import startup_parameters
    (tmp_path/'root').mkdir()
    (tmp_path/'root'/'launch_params').write_text(yaml.dump({'/**': {'ros__parameters': {
        'world_transform.translation_m': (0.1, -0.2, 0.0), 'hostname': '169.254.10.221'}}}))
    assert '!!python/tuple' in (tmp_path/'root'/'launch_params').read_text()
    params,_=startup_parameters(['--ros-args','--params-file','/launch_params',''],tmp_path)
    assert params=={'world_transform.translation_m':[0.1,-0.2,0.0],'hostname':'169.254.10.221'}
    (tmp_path/'root'/'evil').write_text('/**:\n  ros__parameters:\n    x: !!python/object/apply:os.system ["true"]\n')
    with pytest.raises(yaml.YAMLError):
        startup_parameters(['--ros-args','--params-file','/evil',''],tmp_path)



def test_generated_receipt_uses_canonical_empty_ips_and_same_revision_values(tmp_path):
    import subprocess
    source=(Path(__file__).resolve().parents[1]/'helpers/hope-lifecycle').read_text()
    function=source[source.index('write_laptop_runtime() {'):source.index('\nrun_laptop() {')]
    env={**os.environ,'HOME':str(tmp_path),'SESSION_ID':'model21800_20261109T000000Z',
         'HDU_WIFI_IP':'192.168.8.182','LAPTOP_WIFI_IP':'192.168.8.150','MDU_INTERNAL_IP':'10.42.10.12',
         'MOTIVE_IP':'NONE','VRPN_IP':'NONE','HOPE_ROOT':'/workspace','LAPTOP_SHARE':'/share',
         'TABLE_SIDE':'P1','MOCAP_SOURCE':'none','VRPN_PORT':'3883','VRPN_UNITS':'mm','PANTS_PROFILE':'v4'}
    subprocess.run(['bash','-c','set -euo pipefail\n'+function+'\nwrite_laptop_runtime'],env=env,check=True)
    receipt=read_receipt(tmp_path/'.config/hope-foxglove/laptop-runtime.env')
    assert receipt['HOPE_LIFECYCLE_VRPN_IP']==receipt['HOPE_LIFECYCLE_MOTIVE_IP']==''
    assert receipt['HOPE_LIFECYCLE_LAPTOP_IP']=='192.168.8.150'
    from hope_mocap_connection import retained_confirmations
    values=['192.168.8.182','none','NONE','NONE','3883','mm','v4','P1']
    path=tmp_path/'.config/hope-foxglove/laptop-runtime.env'
    path.write_text(path.read_text().replace('HOPE_VRPN_WORLD_FRAME_CONFIRMED=false','HOPE_VRPN_WORLD_FRAME_CONFIRMED=true'))
    assert retained_confirmations(path,values)=='true'
    assert retained_confirmations(path,[*values[:-1],'P2'])=='false'


def test_world_confirmation_evidence_is_separate_from_raw_vrpn(receipt):
    from mocap_config_evidence import endpoint_evidence
    false=endpoint_evidence('base',{'world_frame_calibrated':False},receipt)
    assert false['matches']  # configuration consistency is not localization readiness
    confirmed=dict(receipt,HOPE_VRPN_WORLD_FRAME_CONFIRMED='true')
    stale=endpoint_evidence('base',{'world_frame_calibrated':False},confirmed)
    assert not stale['matches']


@pytest.mark.parametrize('argv',[
    ['/bin/bash','/usr/bin/distrobox','enter','hope','--','ros2','run','foxglove_bridge','foxglove_bridge'],
    ['/bin/bash','-c','python3','/workspace/hope_live_scene.py'],
    ['/usr/bin/python3','/opt/ros/jazzy/bin/ros2','run','foxglove_bridge','foxglove_bridge']])
def test_launch_wrappers_are_not_mistaken_for_ros_consumers(receipt,tmp_path,argv):
    proc(tmp_path,'/placeholder',{})
    (tmp_path/'100/cmdline').write_bytes(b'\0'.join(x.encode() for x in argv))
    assert probe(receipt,tmp_path,Mock(return_value=SimpleNamespace(stdout='active\n')))['processes']==[]


def test_field_display_must_own_listening_socket(receipt, tmp_path, monkeypatch):
    monkeypatch.setenv('HOPE_FIELD_DISPLAY', '1')
    proc(tmp_path, '/opt/ros/jazzy/lib/foxglove_bridge/foxglove_bridge',
         dict(ROS_STATIC_PEERS=receipt['HOPE_LIFECYCLE_HDU_IP']))
    pid = next(tmp_path.glob('[0-9]*'))
    (pid/'fd').mkdir()
    (pid/'fd/5').symlink_to('socket:[3333]')
    (tmp_path/'net').mkdir()
    tcp = tmp_path/'net/tcp'
    tcp.write_text('header\n0: 0100007F:223F 00000000:0000 0A 0 0 0 1000 0 4444\n')
    assert not probe(receipt, tmp_path)['display_active']
    tcp.write_text(tcp.read_text().replace('4444','3333'))
    assert probe(receipt, tmp_path)['display_active']
    tcp.write_text(tcp.read_text().replace('0100007F','00000000'))
    assert not probe(receipt, tmp_path)['display_active']
