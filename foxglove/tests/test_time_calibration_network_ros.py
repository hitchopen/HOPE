"""Real Jazzy timer/executor teardown; synthetic network inputs, no robot peers."""
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get('ROS_DOMAIN_ID') != '218' or os.environ.get('ROS_LOCALHOST_ONLY') != '1',
    reason='requires isolated localhost domain 218')


def test_real_executor_recycles_only_after_owner_stops_then_replacement_stays_up(tmp_path):
    for mode in ['reconnect','replacement']:
        report=tmp_path/(mode+'.json')
        result=subprocess.run([sys.executable,__file__,'--child',mode,str(report)],
            capture_output=True,text=True,timeout=15)
        assert report.exists(),result.stdout+result.stderr
        data=json.loads(report.read_text())
        assert data['closed'] and data['destroyed']
        assert data['clock_actions']==0
        if mode=='reconnect':
            assert result.returncode==75,result.stdout+result.stderr
            assert data['recycle'] and data['elapsed']>=5
        else:
            assert result.returncode==0,result.stdout+result.stderr
            assert not data['recycle'] and data['elapsed']>=5


def child(mode,report):
    import functools
    import time
    from types import SimpleNamespace
    from unittest.mock import Mock
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'a3'))
    import hope_time_calibration as c
    from hope_lifecycle_core import try_acquire_hardware_operation_lock
    from hope_time_calibration_core import read_boot_id
    start=time.monotonic()
    root=report.parent
    c.STATUS_PATH=root/(mode+'-status.json')
    c.OWNER_PATH=root/(mode+'-owner.json')
    lock=root/(mode+'-lock');lock.touch()
    c.try_acquire_hardware_operation_lock=functools.partial(try_acquire_hardware_operation_lock,lock)
    c.load_config=lambda _:SimpleNamespace(revision=1,hdu_wifi_ip='192.0.2.1')
    old=((5,'fixture',False,'192.0.2.1'),)
    current=((5,'fixture',True,'192.0.2.1'),)
    c.wait_for_calibration_network=lambda _:old if mode=='reconnect' else current
    c.read_calibration_network=lambda:current
    backend=Mock()
    c.TimeCalibrationBackend=Mock(return_value=backend)
    data={'closed':False,'destroyed':False}
    base=c.HopeTimeCalibration
    class Node(base):
        def __init__(self,*a):
            super().__init__(*a)
            def owner():
                elapsed=time.monotonic()-start
                c.OWNER_PATH.write_text(json.dumps(dict(boot_id=read_boot_id(),
                    state='RUNNING' if elapsed<5 else 'STOPPED',busy=False,monotonic_s=time.monotonic())))
                if mode=='replacement' and elapsed>6:
                    c.rclpy.try_shutdown()
            self.create_timer(.2,owner)
        def close(self):
            super().close();data.update(closed=True,recycle=self.recycle_requested,
                elapsed=time.monotonic()-start,clock_actions=backend.calibrate.call_count)
        def destroy_node(self):
            super().destroy_node();data['destroyed']=True
            report.write_text(json.dumps(data))
    c.HopeTimeCalibration=Node
    c.main()

if __name__=='__main__':
    child(sys.argv[2],Path(sys.argv[3]))
