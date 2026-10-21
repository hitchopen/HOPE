from pathlib import Path
import os
import subprocess
import sys
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'a3'))
from hope_mocap_connection import require_receiver_owner

ROOT = Path(__file__).resolve().parents[2]

@pytest.mark.parametrize('content', ['', 'HOPE_LIFECYCLE_HDU_IP=192.168.8.181\n', 'HOPE_LIFECYCLE_HDU_IP=192.168.8.182\nHOPE_LIFECYCLE_HDU_IP=192.168.8.181\n'])
def test_unknown_or_other_receiver_rejected(tmp_path, content):
    receipt=tmp_path/'runtime.env'; receipt.write_text(content)
    with pytest.raises(ValueError): require_receiver_owner(receipt, '192.168.8.182')

@pytest.mark.parametrize('owner,expected', [('192.168.8.181', 44), ('192.168.8.182', 0)])
def test_actual_helper_checks_owner_before_any_stop(tmp_path, owner, expected):
    env_path=tmp_path/'.config/hope-foxglove/laptop-runtime.env'
    env_path.parent.mkdir(parents=True)
    env_path.write_text(f'HOPE_LIFECYCLE_HDU_IP={owner}\n')
    source=(ROOT/'foxglove/helpers/hope-lifecycle').read_text()
    function=source[source.index('reconnect_laptop() {'):source.index('\nstart_laptop() {')]
    script='''set -euo pipefail
require_user() { :; }
tmux_exists() { return 0; }
die() { exit 44; }
stop_laptop() { echo STOP >> "$TEST_LOG"; }
start_laptop() { echo START >> "$TEST_LOG"; }
'''+function+'\nreconnect_laptop\n'
    log=tmp_path/'calls'
    result=subprocess.run(['bash','-c',script], env={**os.environ,'HOME':str(tmp_path),
      'HOPE_ROOT':str(ROOT),'LAPTOP_USER':'test','LAPTOP_SESSION':'test',
      'HDU_WIFI_IP':'192.168.8.182','MOCAP_SOURCE':'vrpn','TEST_LOG':str(log),
      'MOTIVE_IP':'192.168.100.99','VRPN_IP':'192.168.100.99','VRPN_PORT':'3883','VRPN_UNITS':'mm','PANTS_PROFILE':'v2','TABLE_SIDE':'P1'}, capture_output=True,text=True)
    assert result.returncode==expected,result.stderr
    assert (log.read_text() if log.exists() else '') == ('STOP\nSTART\n' if expected==0 else '')


def test_geometry_confirmations_retained_only_for_exact_connection(tmp_path):
    from hope_mocap_connection import retained_confirmations
    fields=['HDU_IP','MOCAP_SOURCE','MOTIVE_IP','VRPN_IP','VRPN_PORT','VRPN_UNITS','PANTS_PROFILE','TABLE_SIDE']
    values=['192.168.8.182','vrpn','192.168.100.99','192.168.100.99','3883','mm','v2','P1']
    receipt=tmp_path/'runtime.env'
    receipt.write_text(''.join(f'HOPE_LIFECYCLE_{key}={value}\n' for key,value in zip(fields,values))+'HOPE_VRPN_PELVIS_ASSET_CONFIRMED=true\nHOPE_VRPN_WORLD_FRAME_CONFIRMED=true\n')
    assert retained_confirmations(receipt,values)=='true true'
    assert retained_confirmations(receipt,[*values[:-1],'P2'])=='false false'
