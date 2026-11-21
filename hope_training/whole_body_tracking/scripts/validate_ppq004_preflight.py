"""Infrastructure gate for an explicitly authorized reference-sequence diagnostic.

Hitting qualification is reported, never used as a substitute for infrastructure
checks or silently promoted to success. Reads an existing actual-planner attempt.
"""
import argparse,csv,hashlib,json,re
from pathlib import Path
import numpy as np
from mujoco_eval_recurrent import ROOT,dump
from reference_gru_diagnostic import reference_ledger


def load(path):return json.loads(Path(path).read_text())
def lines(path):
    with Path(path).open() as f:return [json.loads(x) for x in f]

def require(value,message):
    if not value:raise ValueError('Preflight infrastructure gate: '+message)


def validate(session,attempt='attempt-8aUCdxAm'):
    s=Path(session);base=s/attempt;p=base/'preflight'
    summary=load(p/'summary.json');cfg=load(p/'configuration.json');rows=load(p/'per-ball.json')
    snapshot=load(s/'snapshot.json');manifest=load(s/'bundle/manifest.json');physical=load(p/'physical-evidence.json')
    require(summary['phase']=='preflight','not a preflight')
    require(summary['iteration']==snapshot['iteration']==manifest['checkpoint_iteration']==29500,'wrong iteration')
    checkpoint='b53a031b6d2dacceeebe7301412d3d2c5fc456f445c6aebb357e4159871683d0'
    require(summary['checkpoint_sha256']==snapshot['sha256']==manifest['checkpoint_sha256']==checkpoint,'checkpoint identity')
    require(snapshot['run_id']=='gru-formal-2b8c09938a','wrong W&B run')
    require(summary['onnx_sha256']==manifest['onnx_sha256']==cfg['policy_manifest']['onnx_sha256'],'ONNX identity')
    profile=manifest['task']['ball_physics'];sha=hashlib.sha256((ROOT/profile['path']).read_bytes()).hexdigest()
    require(sha==profile['sha256']=='df55e82102e0d5624732ea7ea03f6d98053f45d7f036367e3df3551a1c88a6f9','profile hash')
    require(cfg['table_impact_model']=='ppq004','wrong table impact mode')
    require(cfg['physics']==load_yaml(ROOT/profile['path']),'recorded physics differs from frozen profile')
    require(cfg['pair_settings']['table']['impact_model']=='ppq004_capped_slip_actual_contact_v1','wrong impact handler')
    require(cfg['control_dt']==.02 and cfg['physics_dt']==.001,'control/physics timing')
    require(cfg['initial_idle_s']==20 and cfg['final_tail_s']==3 and cfg['reset_rule']=='initial only; fall ends rally','schedule/reset contract')
    require(summary['error'] is None and summary['rally_failure'] is None,'runtime/fall error')
    require(summary['resets_after_initial']==0 and not any(summary['mujoco_warning_counts']),'reset/numerical warning')
    require(not summary['remaining_live_ids'] and summary['drain_pass'],'remaining live flights')
    require(summary['peak_concurrent_balls']>=2,'rapid concurrency not exercised')
    require([r['shot_id'] for r in rows]==[1,2,7,8,9],'wrong preflight sequence')
    require(physical['trajectory_complete_count']==5 and not physical['trajectory_incomplete_shot_ids'],'incomplete physical telemetry')
    require(summary['simulated_seconds']>=summary['final_tail_end_s'],'final tail incomplete')
    require(all(abs(c['error_s'])<=.1 for c in summary['cadence']),'cadence violation')
    ledger,_=reference_ledger(s/'fixed-home-100.yaml')
    expected_time=20.
    for row,index in zip(rows,[0,1,6,7,8]):
        require(row['launched'] and not row['unlaunched'] and row['trajectory_complete'] and not row['interrupted'],'unlaunched/interrupted serve')
        require(row['terminal_kind'] in ('ground_crossing','settled_on_table'),'nonphysical flight terminal')
        require(abs(row['launched_at_s']-expected_time)<1e-6,'preflight launch time')
        require(row['position_w']==ledger[index]['position_w'] and row['velocity_w']==ledger[index]['velocity_w'],'serve geometry changed')
        expected_time+=row['flight_s']+row['pause_s']
        table=[e for e in row['events'] if e['kind']=='table_contact']
        require(table and table[0].get('impact_model')=='ppq004_capped_slip_actual_contact_v1','missing actual PPQ004 impact witness')
        require(table[0]['normal_force']>0 and table[0].get('position_modified') is False,'nonphysical/snapped impact')
    commands=lines(p/'planner-commands.jsonl');trace=lines(p/'policy-trace.jsonl');audit=list(csv.DictReader((base/'preflight-ros/planner-audit.csv').open()))
    delivered=[]
    for row in rows:
        applied=[v for v in commands if v['shot_id']==row['shot_id'] and v['applied']]
        require(len(applied)==1 and applied[0]['decode_reason']=='accepted','missing/repeated accepted planner command')
        cmd=applied[0];target=cmd['decoded'];deadline=row['planned_deadline_s']
        require(row['planner_on_time'] and target['lead_at_receipt_s']>=.04,'late planner command')
        require(any(a['command_seq']==str(target['identity'][0]) and a['wire_valid']=='1' for a in audit),'command absent from actual planner audit')
        consumed=[]
        for tick in trace:
            if not cmd['sim_t']<=tick['sim_t']<deadline or tick['observation'] is None:continue
            if tick['target_source']!='actual_live_planner' or not np.allclose(tick['target_position_w'],target['position_w'],atol=1e-8):continue
            obs=np.array(tick['observation']);base_pos=np.array(tick['base_at_observation'])
            require(np.allclose(obs[103:106],np.array(target['position_w'])-base_pos,atol=2e-7,rtol=0),'actor target position fields')
            require(np.allclose(obs[106:109],target['velocity_w'],atol=2e-7,rtol=0),'actor velocity fields')
            require(abs(obs[109]-(deadline-tick['sim_t']))<2e-7,'actor countdown')
            require(tick['actions'] is not None and tick['policy_reset_count']==1,'actor inactive/reset')
            consumed.append(tick['sim_t'])
        require(consumed,'planner command never consumed by actor')
        delivered.append({'shot_id':row['shot_id'],'actual_actor_ticks':len(consumed),'first_tick_s':consumed[0],'side':target['planned_side']})
    require(all(v['policy_reset_count']==1 and not v['rally_failed'] for v in trace),'recurrent reset/failure')
    sensors=lines(base/'preflight-ros/sensor-and-planner.jsonl')
    require(sensors,'no transport trace')
    stamps=[v['input']['stamp_ns'] for v in sensors]
    require(all(b>a for a,b in zip(stamps,stamps[1:])),'nonmonotonic exposure stamps')
    skews=[abs(v['output']['reply_unix_ns']-v['input']['stamp_ns'])/1e9 for v in sensors]
    require(max(skews)<.05,'exposure/host clock skew')
    require({v['input']['serve_id'] for v in sensors if v['input']['ball_active']}=={1,2,7,8,9},'wrong planner ball ownership')
    log=(base/'preflight-ros/planner.log').read_text();launcher=(base/'launcher.log').read_text()
    require(not re.search(r'\[(ERROR|FATAL)\]|Traceback|BrokenPipeError',log+'\n'+launcher),'planner/launcher infrastructure error')
    health=re.findall(r'planner health[^\n]*',log)
    require(health and all(token in health[-1] for token in ['drops=0','deadline_miss=0','logger_drops=0']),'planner drops/deadline errors')
    require('Preflight replay:' in launcher and 'Phase preflight failed' not in launcher,'launcher did not finish cleanly')
    hitting={'raw_contacts':sum(any(e['kind']=='racket_contact' and e.get('normal_force',0)>0 for e in row['events']) for row in rows),
        'qualified_contacts':sum(row['contact_pass'] for row in physical['rows']),
        'legal_landings':sum(row['landing_pass'] for row in physical['rows']),
        'recovery_dwell_pass':sum(row['recovery_settled'] for row in rows)}
    require(all(summary['groups']['all'][k]==v for k,v in hitting.items()),'hitting report disagrees with per-ball physical evidence')
    files=['preflight/summary.json','preflight/configuration.json','preflight/per-ball.json','preflight/physical-evidence.json','preflight/planner-commands.jsonl','preflight/policy-trace.jsonl','preflight-ros/planner-audit.csv','preflight-ros/sensor-and-planner.jsonl','preflight-ros/planner.log','launcher.log']
    return {'infrastructure_gate':'PASS','validated_attempt':str(base),'iteration':29500,'checkpoint_sha256':checkpoint,'onnx_sha256':manifest['onnx_sha256'],
        'table_impact_model':'ppq004','profile_sha256':sha,'actor_deliveries':delivered,'all_flights_drained':True,'resets_after_initial':0,
        'maximum_transport_clock_skew_s':max(skews),'peak_concurrent_balls':summary['peak_concurrent_balls'],
        'hitting_observed_not_a_gate':summary['groups']['all'],'independent_hitting_evidence':hitting,'formal_gate3_certification':'NOT_APPLICABLE',
        'source_hashes':{name:hashlib.sha256((base/name).read_bytes()).hexdigest() for name in files},
        'scheduled_diagnostic_balls':len(ledger),'initial_idle_s':20,'first_six_flight_s':[r['flight_s'] for r in ledger[:6]],
        'rapid_flight_s':[r['flight_s'] for r in ledger[6:]],'no_inter_ball_resets':True,'fall_ends_rally':True}


def load_yaml(path):
    import yaml
    return yaml.safe_load(Path(path).read_text())

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--session',type=Path,required=True);p.add_argument('--attempt',default='attempt-8aUCdxAm');p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=validate(a.session,a.attempt)
    if a.output.exists():raise FileExistsError(a.output)
    dump(a.output,result)
    h=result['independent_hitting_evidence']
    print(f"Validated actual-planner infrastructure: PASS; raw contacts={h['raw_contacts']}, qualified={h['qualified_contacts']}, legal landings={h['legal_landings']} (not a hitting pass). Scheduled diagnostic balls={result['scheduled_diagnostic_balls']}.",flush=True)
