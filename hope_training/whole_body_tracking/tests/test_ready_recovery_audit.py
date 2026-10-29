import csv
import json
import math
from pathlib import Path

from test_gaussian_action_projection import load_module

ROOT = Path(__file__).resolve().parents[1]
audit = load_module("ready_recovery_audit_test", ROOT / "scripts/audit_ready_recovery.py")


def test_dwell_cannot_bridge_failed_samples_or_telemetry_gaps():
    assert audit.longest_dwell([0, .01, .02, 1, 1.01], [1, 1, 0, 1, 1]) < .02
    assert audit.longest_dwell([0, .01, .02, 1, 1.01], [1]*5) < .03


def test_station_pose_audit_keeps_original_anchor_verdict_and_rejects_crouch(tmp_path):
    reference = ROOT / "cfg/references/hold_pose_B_32_5_K20.json"
    pose = json.loads(reference.read_text())
    refs = dict(zip(pose['joint_order'], pose['actual_q_reference_rad']))
    report = {'recovered_engaged_serves': 0, 'rows': [{
        'serve': 1, 'flight_id': 1, 'shot_id': 1, 'command_station': [.2, 0],
        'runner_physical_contact_pose': {'physical_contact_wall_time_ns': 100_000_000}}]}
    original = json.dumps(report)
    (tmp_path/'pp_rally_report.json').write_text(original)
    (tmp_path/'pp_mujoco_plant_report.json').write_text(json.dumps({'runner': {'motion_wall_time_last_ns': 2_000_000_000}}))
    r, p = [math.radians(v)/2 for v in pose['metrics']['torso_rpy_deg'][:2]]
    template = dict(base_x=.2, base_y=0, base_z=pose['metrics']['pelvis_height_cm']/100,
                    base_vx=0, base_vy=0, base_qw=1, base_qx=0, base_qy=0, base_qz=0,
                    torso_qw=math.cos(r)*math.cos(p), torso_qx=math.sin(r)*math.cos(p),
                    torso_qy=math.cos(r)*math.sin(p), torso_qz=-math.sin(r)*math.sin(p),
                    q_waist_pitch_joint=refs['waist_pitch_joint'],
                    q_left_knee_joint=refs['left_knee_joint'], q_right_knee_joint=refs['right_knee_joint'],
                    left_foot_x=.2, left_foot_y=.16, right_foot_x=.2, right_foot_y=-.16)
    template.update({'q_'+name: ref for name,ref in refs.items()})
    for side in ('left','right'):
        template.update({side+'_foot_q'+axis: float(axis=='w') for axis in 'wxyz'})
        template[side+'_wrist_z']=pose['metrics'][side+'_wrist_height_cm']/100
    def write_plant():
        with (tmp_path/'pp_mujoco_plant.csv').open('w') as stream:
            writer = csv.DictWriter(stream, ['wall_time_ns', *template]);writer.writeheader()
            for i in range(200): writer.writerow({'wall_time_ns': i*10_000_000, **template})
    write_plant()
    result = audit.audit(tmp_path, reference)
    assert result['station_recovered'] == result['full_ready_pose_recovered'] == 1
    assert result['full_body_evidence_complete'] and result['full_joint_pose_recovered']==1
    assert result['original_session_anchor_recovered'] == 0
    template['q_left_knee_joint'] = math.radians(40)
    write_plant()
    result = audit.audit(tmp_path, reference)
    assert result['station_recovered'] == 1 and result['full_ready_pose_recovered'] == 0
    template['q_left_knee_joint']=refs['left_knee_joint']
    template['q_right_elbow_joint']=refs['right_elbow_joint']+math.radians(35)
    write_plant()
    result=audit.audit(tmp_path,reference)
    assert result['legacy_partial_pose_recovered']==1
    assert result['full_ready_pose_recovered']==result['full_joint_pose_recovered']==0
    template['q_right_elbow_joint']=refs['right_elbow_joint']
    del template['left_wrist_z']
    write_plant()
    result=audit.audit(tmp_path,reference)
    assert result['full_joint_pose_recovered']==1
    assert result['full_ready_pose_recovered'] is None and not result['full_body_evidence_complete']
    assert (tmp_path/'pp_rally_report.json').read_text() == original
    # Early next-ball arrival gives only 0.1 s after protected follow-through;
    # a 0.3 s dwell is impossible and must not be labeled a posture failure.
    report['rows'].append({'serve':2,'flight_id':2,'shot_id':2,'command_station':[.2,0],
        'physical_outcome':{'first_stamp_ns':700_000_000},
        'runner_physical_contact_pose':{'physical_contact_wall_time_ns':1_000_000_000}})
    (tmp_path/'pp_rally_report.json').write_text(json.dumps(report))
    result=audit.audit(tmp_path,reference)
    assert result['interrupted_by_next_ball']==1
    assert result['eligible_recovery_windows']==1 and result['station_recovered']==1
