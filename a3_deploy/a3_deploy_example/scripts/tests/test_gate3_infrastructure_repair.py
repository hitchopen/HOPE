import argparse
import csv
import fcntl
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys

import pytest

SCRIPTS=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(SCRIPTS))
from pp_gate3_plant_contracts import add_cli, evaluate
from pp_wire_identity_probe import probe
from pp_fixed_home_profile import planner_profile
from run_model19000_gate3_repair import preflight_gate, reusable_stage, ORCHESTRATION_CHANGES
import run_gate3


def fixture(tmp):
    args=argparse.ArgumentParser();add_cli(args)
    a=args.parse_args(['--require-home-support-frame','--require-torso-hold','--require-collision-ledger',
                      '--max-home-signed-width-m','0.31'])
    a.plant=tmp/'plant.csv';a.runner_trace=tmp/'runner.csv'
    trace=[dict(wall_time_ns=1_000_000_000+i*20_000_000,mode=3,level=0,reach_level=0) for i in range(101)]
    rows=[]
    for i in range(420):
        row=dict(wall_time_ns=950_000_000+i*5_000_000,base_x=-.5,base_y=-.7625)
        for body in ('base','torso','left_foot','right_foot'):
            row.update({body+'_q'+axis:float(axis=='w') for axis in 'wxyz'})
        row.update(left_foot_x=-.5,left_foot_y=-.62,right_foot_x=-.5,right_foot_y=-.91,
                   robot_collision_ledger_v1=1,robot_table_contact_count=0,
                   robot_net_contact_count=0,robot_self_contact_count=0)
        rows.append(row)
    save(a.runner_trace,trace);save(a.plant,rows)
    return a,rows


def save(path,rows):
    with path.open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=rows[0]);writer.writeheader();writer.writerows(rows)


def test_all_required_checks_pass_on_measured_quiet_pose(tmp_path):
    a,_=fixture(tmp_path);r=evaluate(a)
    assert r['extended_checks_pass']
    assert r['home_support_frame']['anchor_wall_time_ns']<1_000_000_000
    assert r['torso_hold']['windows']


@pytest.mark.parametrize('change,group', [('support','home_support_frame'),('torso','torso_hold'),('collision','collision_ledger')])
def test_required_check_detects_actual_violation(tmp_path,change,group):
    a,rows=fixture(tmp_path)
    for row in rows:
        if row['wall_time_ns']<1_500_000_000:continue
        if change=='support':row['left_foot_y']+=.15
        if change=='torso':row['torso_qw']=math.cos(.2);row['torso_qy']=math.sin(.2)
        if change=='collision':row['robot_table_contact_count']=1
    save(a.plant,rows);r=evaluate(a)
    assert not r['extended_checks_pass'] and not r[group]['pass']


def test_missing_collision_evidence_never_passes(tmp_path):
    a,rows=fixture(tmp_path)
    for row in rows:row.pop('robot_collision_ledger_v1')
    save(a.plant,rows)
    with pytest.raises(ValueError,match='missing required columns'):evaluate(a)


def test_wire_probe_distinguishes_open_and_actual_lock_holder(tmp_path):
    missing=probe(tmp_path/'missing-parent/state')
    assert missing['open']['errno']==2 and missing['flock'] is None
    path=tmp_path/'state';lock=Path(str(path)+'.lock')
    with lock.open('w') as f:
        fcntl.flock(f,fcntl.LOCK_EX|fcntl.LOCK_NB)
        r=probe(path)
        assert r['open']['ok'] and not r['flock']['ok']
        assert r['flock']['errno'] in (11,13)
        assert any(x['pid']==str(__import__('os').getpid()) for x in r['holders'])
    assert probe(path)['flock']['ok']
    assert lock.exists() # no stale-lock deletion


def test_saved_native_inspection_has_model19000_plane():
    repo=SCRIPTS.parents[2]
    # Published metadata candidate inspection is captured in the original formal evidence.
    model=repo/'a3_deploy/a3_deploy_example/models/model_19000_native_v2/manifest.json'
    assert json.loads(model.read_text())['checkpoint_sha256'].startswith('744526d2')
    r={'policy_abi':'ball_clock_110_v1','station_contract':'fixed_session_anchor_v1',
       'position_support':[[.44,.70,-.68,-.24,.8,1.45],[.44,.70,-.24,.28,.8,1.45]],
       'velocity_support':[[1,2.9,-.8,1.2,.3,2],[1.1,2.9,-1,.9,0,1.8]]}
    assert planner_profile(r)[1]==pytest.approx(.57)


def preflight_fixture(folder):
    def write(name,data):(folder/name).write_text(json.dumps(data))
    from run_model19000_gate3_repair import PREFLIGHT_SCENE,sha
    (folder/'scene.yaml').write_bytes(PREFLIGHT_SCENE.read_bytes())
    write('run.json',{'scene_sha256':sha(PREFLIGHT_SCENE),'evaluation_kind':'integration_preflight_not_certification', 'scene':{'balls':26},'resolved_scene_environment':{'PP_SERVES':'26'}})
    write('planner.pid.monitor-complete.json',{'pid':12345,'alive':True})
    write('pp_rally_report.json',{'rows':[{'t':20}]*26,
          'runner_trace_engagement':{'engaged_flight_ids':[1],'completed_flight_ids':[1]}})
    write('pp_planner_cpp_report.json',{'valid_commands':26,'rows_observed':26})
    write('pp_mujoco_plant_report.json',{
        **{name:{'evidence_complete':True,'failures':[]} for name in ('home_support_frame','torso_hold','collision_ledger')},
        'idle_smoothness':{'steady':{'measured':True},'failures':['measured behavioral failure retained']}})
    write('pp_physical_ball_report.json',{'rows':[{'shot_id':i,'telemetry_complete':True,'trajectory_complete':True} for i in range(1,27)]})
    write('pp_gate3_drain_report.json',{'launched_shot_ids':list(range(1,27)),
          'status':'complete','evidence_complete':True,'pending_shot_ids':[]})
    (folder/'console.log').write_text('Reports produced; missed shot is a behavioral failure.')
    (folder/'pp_runner.log').write_text('[pp shared-home] match=1 wire_command=1 receiver_epoch=1 revision=1 flight=1 schema=4 epoch=123 expected_epoch=123 frame=0 target_x=0.070 expected_x=0.070 plane=0.57 base_fresh=1 calibration=55 expected_calibration=55 world=66 expected_world=66\n')
    save(folder/'pp_planner_debug.csv',[{'flight_id':str(i),'wire_valid':'1','command_seq':str(i),'revision_id':'1','session_home_epoch':'123','strike_x':'.07'} for i in range(1,27)])


def test_preflight_accepts_complete_infrastructure_without_faking_behavior_pass(tmp_path):
    preflight_fixture(tmp_path);r=preflight_gate(tmp_path)
    assert r['infrastructure_pass'] and r['behavioral_failures_retained']['idle_smoothness']
    assert r['certification'] is False


@pytest.mark.parametrize('failure', ['no_completion','wrong_epoch','missing_plant_evidence','dead_planner'])
def test_preflight_prevents_formal_when_proof_missing(tmp_path,failure):
    preflight_fixture(tmp_path)
    if failure=='no_completion':
        p=tmp_path/'pp_rally_report.json';r=json.loads(p.read_text());r['runner_trace_engagement']['completed_flight_ids']=[];p.write_text(json.dumps(r))
    if failure=='wrong_epoch':
        p=tmp_path/'pp_runner.log';p.write_text(p.read_text().replace('expected_epoch=123','expected_epoch=456'))
    if failure=='missing_plant_evidence':
        p=tmp_path/'pp_mujoco_plant_report.json';r=json.loads(p.read_text());r['home_support_frame']['evidence_complete']=False;p.write_text(json.dumps(r))
    if failure=='dead_planner':(tmp_path/'infrastructure_failure.json').write_text('{}')
    assert not preflight_gate(tmp_path)['readiness_pass']


@pytest.mark.parametrize('normal_cleanup',[False,True])
def test_planner_watchdog_distinguishes_crash_from_post_drain_cleanup(tmp_path,monkeypatch,normal_cleanup):
    class Process:
        calls=0
        def wait(self,timeout):
            self.calls+=1
            if self.calls==1:raise subprocess.TimeoutExpired('fixture',timeout)
            return 0
    (tmp_path/'planner.pid').write_text('999999999') # deliberately nonexistent; never signalled
    if normal_cleanup:(tmp_path/'planner.pid.monitor-complete.json').write_text('{"alive":true}')
    stopped=[]
    monkeypatch.setattr(run_gate3,'stop_engine_preserving_flights',lambda *args:stopped.append(True))
    rc=run_gate3.wait_for_engine(Process(),tmp_path,0)
    assert rc==(0 if normal_cleanup else 2)
    assert bool(stopped)==(not normal_cleanup)


@pytest.mark.parametrize('prefix',[False,True])
def test_checked_scene_keeps_formal100_and_uses_standard_26_preflight(tmp_path,monkeypatch,prefix):
    # Composition-only check: no transport/backend or actual policy is initialized.
    import yaml
    dist=tmp_path/'dist';sim=tmp_path/'sim';ws=tmp_path/'ws'
    for p in [dist/'a3_deploy_onnx_ref_pingpong',dist/'run_a3_pingpong.sh',sim/'bin/aimrt_main',ws/'install/local_setup.bash']:
        p.parent.mkdir(parents=True,exist_ok=True);p.write_text('test fixture')
    physics=ws/'src/hope_planner_cpp/config/ball_physics_ppq004_20261020.yaml'
    physics.parent.mkdir(parents=True);physics.write_text('hope_planner:\n  ros__parameters:\n    drag_k: 0.1245\n')
    runtime=tmp_path/'runtime.yaml';runtime.write_text(json.dumps({'runner_dist':str(dist),'sim_install':str(sim),'hope_ws':str(ws)}))
    policy=tmp_path/'policy'
    for name in ['params/deploy.yaml','exported/policy.onnx']:
        p=policy/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text('composition fixture, not an actor')
    inspection={'policy_abi':'ball_clock_110_v1','station_contract':'fixed_session_anchor_v1',
        'position_support':[[.44,.70,-.68,-.24,.8,1.45],[.44,.70,-.24,.28,.8,1.45]],
        'velocity_support':[[1,2.9,-.8,1.2,.3,2],[1.1,2.9,-1,.9,0,1.8]]}
    monkeypatch.setattr(run_gate3,'native_inspect',lambda *args:inspection)
    folder=tmp_path/'check'
    argv=['--runtime',str(runtime),'--policy-dir',str(policy),'--check','--output',str(folder),
          '--scenario',str(run_gate3.CONFIG/'scenarios'/('fixed-home.yaml' if prefix else 'fixed-home-100.yaml'))]
    if prefix:argv.append('--integration-preflight')
    assert run_gate3.execute(run_gate3.parser().parse_args(argv))==0
    r=json.loads((folder/'run.json').read_text());env=r['resolved_scene_environment']
    assert env['PP_SERVES']==('26' if prefix else '100')
    assert env['PP_MOTION_IDLE_S']=='20.0' and env['PP_REQUIRED_POST_COMPLETION_TAIL_S']=='3.0'
    assert r['evaluation_kind']==('integration_preflight_not_certification' if prefix else 'formal_gate3')
    for key,value in run_gate3.scene_environment(r['scene']).items():assert env[key]==value
    assert json.loads(env['PP_SERVES_LIST'])[:36]==sum(r['scene']['clean_serves'],[])
    profile=yaml.safe_load((folder/'fixed_home_ppq004.yaml').read_text())['hope_planner']['ros__parameters']
    assert profile['wire_identity_state_file']=='' and profile['drag_k']==.1245


def test_behavioral_early_termination_is_not_infrastructure_failure(tmp_path):
    preflight_fixture(tmp_path)
    p=tmp_path/'pp_rally_report.json';r=json.loads(p.read_text());r['falls']=1;r['rows']=r['rows'][:2];p.write_text(json.dumps(r))
    p=tmp_path/'pp_gate3_drain_report.json';r=json.loads(p.read_text());r['launched_shot_ids']=[1,2];p.write_text(json.dumps(r))
    p=tmp_path/'pp_physical_ball_report.json';r=json.loads(p.read_text());r['rows']=r['rows'][:2];p.write_text(json.dumps(r))
    p=tmp_path/'pp_planner_cpp_report.json';r=json.loads(p.read_text());r['rows_observed']=2;r['valid_commands']=2;p.write_text(json.dumps(r))
    p=tmp_path/'pp_planner_debug.csv';rows=list(csv.DictReader(p.open()));save(p,rows[:2])
    proof=preflight_gate(tmp_path)
    assert proof['infrastructure_pass'] and not proof['readiness_pass']
    assert {'check':'all_26_launched','category':'behavioral_early_termination'} in proof['failure_categories']


def test_only_verified_unchanged_native_stages_migrate(tmp_path):
    from run_model19000_gate3_repair import sha
    binary=tmp_path/'binary';binary.write_bytes(b'verified native artifact')
    receipt={'complete':True,'binding':'old','outputs':{str(binary):sha(binary)}}
    argv=['unchanged','native','command']
    def reuse(name='Runner-build',changes=ORCHESTRATION_CHANGES,old_argv=argv):
        return reusable_stage(name,receipt,'new',argv,old_argv,'old',changes)
    assert reuse()
    assert not reuse(name='integration-preflight')
    assert not reuse(changes={'hope_ws/src/hope_planner_cpp/src/planner_node.cpp'})
    assert not reuse(old_argv=['changed','native','command'])
    binary.write_bytes(b'changed output')
    assert not reuse()


def test_minimum_ball_guard_still_rejects_six():
    import yaml
    scene=yaml.safe_load((run_gate3.CONFIG/'scenarios/fixed-home.yaml').read_text())
    with pytest.raises(ValueError,match='even.*26'):run_gate3.scene_environment(scene,6)
