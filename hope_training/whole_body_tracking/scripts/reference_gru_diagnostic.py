"""Audited fixed-home-100 sequence with carried GRU and concurrent real balls.

Diagnostic only: actual ROS C++ Planner required; native Runner ABI/certification
is not emulated. Fall ends inference/launching, then physical flights drain.
"""
import argparse,csv,hashlib,json,math,socket,sys,time
from pathlib import Path
import numpy as np
from mujoco_eval_recurrent import Plant,RecurrentOnnxPolicy,ObsTarget,dump,ROOT
from recurrent_physical_ball import VideoRecorder
from concurrent_physical_balls import ConcurrentBalls,PoolFull
from planner_rally_protocol import OFFSET,decode_command,valid_delivery,physical_outcome
sys.path.insert(0,str(ROOT/'a3_deploy/a3_deploy_example/scripts'))
from pp_recovery_dwell import update_recovery_dwell
from pp_gate3_core import join_physical_evidence_by_side


def planner_target(position,velocity,deadline_sim_s,observation_sim_s):
    """Absolute exposure-clock deadline, without clipping or a second latency offset.

    The frozen training observation exposes the remaining strike time directly;
    its sampled preparation range is .35--.95 s. A .65 s cap changes that contract.
    """
    return ObsTarget(np.asarray(position),np.asarray(velocity),
                     float(deadline_sim_s-observation_sim_s))


def reference_ledger(scene_path):
    import yaml
    from run_gate3 import scene_environment
    scene=yaml.safe_load(Path(scene_path).read_text());env=scene_environment(scene)
    if scene['balls']!=100:raise ValueError('Audited reference requires 100 balls')
    states=np.array(json.loads(env['PP_SERVES_LIST'])).reshape(100,6)
    fs=json.loads(env['PP_FLIGHT_S_LIST']);ps=json.loads(env['PP_PAUSE_S_LIST'])
    rows=[];t=20.
    for i,(state,flight,pause) in enumerate(zip(states,fs,ps)):
        rows.append({'shot_id':i+1,'position_table':state[:3].tolist(),
            'position_w':(state[:3]+OFFSET).tolist(),'velocity_w':state[3:].tolist(),
            'spin_w':[0.,0.,0.],'flight_s':flight,'pause_s':pause,
            'nominal_launch_at_s':round(t,6),'lane':'clean' if i<6 else 'rapid'})
        t+=flight+pause
    return rows,env


def advance_without_actor(plant):
    """Hold last actuator controls unchanged while draining a failed rally."""
    for i in range(plant.substeps):
        t=plant.episode_time+i*plant.model.opt.timestep
        plant.ball_physics.before(t);plant.mj.mj_step(plant.model,plant.data)
        plant.ball_physics.after(t+plant.model.opt.timestep)
    plant.episode_time+=plant.dt
    if not np.isfinite(plant.data.qpos).all() or not np.isfinite(plant.data.qvel).all():raise FloatingPointError('Nonfinite drain physics')


def end_rally(pool,t):
    """Stop planner ownership only; every live body remains in physics."""
    pool.release(t)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    for flag in ['bundle','scene','output']:ap.add_argument('--'+flag,type=Path,required=True)
    ap.add_argument('--socket',required=True);ap.add_argument('--preflight',action='store_true')
    ap.add_argument('--view',action='store_true');ap.add_argument('--video',action='store_true')
    ap.add_argument('--table-impact-model',choices=['soft','ppq004'],default='soft')
    a=ap.parse_args();out=a.output;out.mkdir(parents=True,exist_ok=False)
    ledger,env=reference_ledger(a.scene);full_ledger=ledger
    if a.preflight:
        # Separate clean FH/BH + actual rapid FF/BH handoffs. No scored outcomes.
        ledger=[dict(full_ledger[i]) for i in [0,1,6,7,8]]
    rows={s['shot_id']:{**s,'launched':False,'planner_on_time':False,'planner_side':None,
            'positive_planner_observation_ticks':0,'commands':[], 'recovery_settled':False,
            'recovery_dwell_observed_s':0.,'recovery_max_dwell_observed_s':0.,
            'raw_contact':False,'qualified_contact':False,'legal_landing':False,
            'trajectory_complete':False,'end_to_end_success':False} for s in ledger}
    dump(out/'ledger.json',ledger);dump(out/'reference-environment.json',env)
    policy=RecurrentOnnxPolicy(a.bundle);plant=Plant(policy.manifest,physical=True,ball_slots=9,table_impact_model=a.table_impact_model)
    trajectory=(out/'ball-trajectories.jsonl').open('x')
    pool=ConcurrentBalls(plant,rows.keys(),trajectory=lambda x:trajectory.write(json.dumps(x)+'\n'))
    policy.reset_episode()  # Exactly once for this process, never per ball/handoff.
    if a.view:plant.open_viewer()
    # Record actual states cheaply; render software video AFTER planner timing is over.
    recorder=None
    render_states=(out/'render-states.jsonl').open('x') if a.video else None
    render_count=0
    dump(out/'configuration.json',{'policy_manifest':policy.manifest,'scene_sha256':hashlib.sha256(a.scene.read_bytes()).hexdigest(),
        'physics':pool.cfg,'pair_settings':pool.pair_settings,'table_impact_model':a.table_impact_model,'slots':9,'control_dt':plant.dt,
        'physics_dt':plant.model.opt.timestep,'initial_idle_s':20,'final_tail_s':3,
        'wall_timeout_s':150 if a.preflight else 900,'seed':0,
        'episode_timeouts_disabled':True,'reset_rule':'initial only; fall ends rally',
        'recovery':{'xy_m':.05,'speed_ema_mps':.12,'speed_ema_alpha':.2,'tilt_deg':12,'yaw_deg':5,'pelvis_min_m':.8,'continuous_dwell_s':.3},
        'host_completion_clock_proxy_after_strike_s':.5,'minimum_command_lead_s':.04,'maximum_wall_clock_lag_s':.05,
        'certification':'NOT_APPLICABLE_REFERENCE_SEQUENCE_DIAGNOSTIC','command':sys.argv})
    conn=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);conn.settimeout(.25);conn.connect(a.socket)
    channel=conn.makefile('rwb');obslog=(out/'policy-trace.jsonl').open('x');cmdlog=(out/'planner-commands.jsonl').open('x')
    begin=time.monotonic();unix_anchor=time.time();subcount=0;current_id=None
    next_index=0;next_launch=20.;release_at=None;tail_end=None;all_released=False
    last_target=None;last_deadline=None;accepted_current=False;spawn_ns=0
    failure=None;fall_time=None;rally_stop_time=None;stop_launches=False;peak_lag=0.;cadence=[]
    home_q=plant.state().base_quat_w.copy();home_yaw=math.atan2(2*(home_q[0]*home_q[3]+home_q[1]*home_q[2]),1-2*(home_q[2]**2+home_q[3]**2))
    ema_speed=0.

    def before(t):
        nonlocal next_launch,next_index,current_id,release_at,tail_end,all_released,accepted_current,spawn_ns
        if stop_launches:return
        if current_id is not None and t+1e-8>=release_at:
            row=rows[current_id];row['released_at_s']=t;pool.release(t);current_id=None
            next_launch=t+row['pause_s']
            if next_index==len(ledger):
                all_released=True
                completion_proxy=max(t,last_deadline+.5 if last_deadline is not None else t)
                tail_end=completion_proxy+3.
        if current_id is None and not all_released and t+1e-8>=next_launch:
            serve=ledger[next_index]
            try:pool.launch(serve,t)
            except PoolFull:
                rows[serve['shot_id']]['slot_wait_s']=t-next_launch
                return
            current_id=serve['shot_id'];row=rows[current_id];row['launched']=True;row['launched_at_s']=t
            if next_index:
                prev=rows[ledger[next_index-1]['shot_id']]
                gap=t-prev['launched_at_s'];expected=prev['flight_s']+prev['pause_s']
                cadence.append({'shot_id':current_id,'gap_s':gap,'expected_s':expected,'error_s':gap-expected})
            next_index+=1;accepted_current=False;spawn_ns=int((unix_anchor+t)*1e9)
            release_at=t+serve['flight_s']
            print(f'LAUNCH shot={current_id} t={t:.3f} active={pool.live} hidden=CARRIED',flush=True)

    def after(t):
        nonlocal subcount,last_target,last_deadline,accepted_current,peak_lag
        subcount+=1
        if subcount%5:return
        lag=time.monotonic()-(begin+t);peak_lag=max(peak_lag,lag)
        if lag>.05:raise RuntimeError(f'Realtime preflight/diagnostic clock lag {lag:.3f}s >50ms; wall-deadline validity not established')
        time.sleep(max(0,begin+t-time.monotonic()))
        state=plant.state();ball=pool.planner_ball()
        request={'stamp_ns':int((unix_anchor+t)*1e9),'sim_t':t,'serve_id':ball.shot_id if ball else None,
            'base_table':(state.base_pos_w-OFFSET).tolist(),'base_quat_wxyz':state.base_quat_w.tolist(),
            'ball_active':ball is not None,'ball_table':(ball.state()[0]-OFFSET).tolist() if ball else [0.,0.,0.],
            'ball_quat_wxyz':plant.data.qpos[ball.q+3:ball.q+7].tolist() if ball else [1.,0.,0.,0.]}
        channel.write((json.dumps(request)+'\n').encode());channel.flush();line=channel.readline()
        if not line:raise RuntimeError('Planner bridge closed')
        reply=json.loads(line)
        for raw in reply['commands']:
            cmd,reason=decode_command(raw['values'],spawn_ns,time.time_ns(),.04)
            apply=bool(ball is not None and current_id is not None and cmd is not None and not accepted_current and not stop_launches)
            record={'sim_t':t,'shot_id':current_id,'decode_reason':reason,'applied':apply,'decoded':cmd,**raw}
            cmdlog.write(json.dumps(record)+'\n')
            if current_id is not None:rows[current_id]['commands'].append(record)
            if apply:
                accepted_current=True;last_target=ObsTarget(np.array(cmd['position_w']),np.array(cmd['velocity_w']),0.)
                last_deadline=cmd['deadline_unix_s']-unix_anchor
                row=rows[current_id];row['planner_on_time']=True;row['planner_side']=cmd['planned_side']
                row['planned_deadline_s']=last_deadline;row['host_completion_clock_proxy_s']=last_deadline+.5
    pool.before_hook=before;pool.after_hook=after
    wall_limit=150 if a.preflight else 900
    t=0.;error=None
    try:
        while time.monotonic()-begin<wall_limit:
            t=plant.episode_time;actor_action=None
            base_at_observation=plant.state().base_pos_w.copy()
            if plant.viewer is not None and not plant.viewer.is_running():raise RuntimeError('Viewer closed')
            target=ObsTarget(plant.data.site_xpos[plant.racket].copy(),np.zeros(3),-1.) if last_target is None else planner_target(last_target.pos_w,last_target.vel_w,last_deadline,t)
            source='idle_no_plan' if last_target is None else 'actual_live_planner'
            if not stop_launches:
                obs=plant.observe(target)
                if current_id is not None and accepted_current and obs[109]>0:rows[current_id]['positive_planner_observation_ticks']+=1
                np.testing.assert_allclose(obs[103:106],target.pos_w-base_at_observation,atol=2e-7)
                np.testing.assert_allclose(obs[106:109],target.vel_w,atol=2e-7)
                action=policy.infer(obs);actor_action=action.tolist();plant.step(action)
            else:
                obs=None;advance_without_actor(plant)
            metric=plant.metrics();state=plant.state()
            if not stop_launches and (metric['base_z']<.8 or metric['tilt_deg']>60 or plant.data.warning.number.any()):
                failure='fall' if metric['base_z']<.8 or metric['tilt_deg']>60 else 'numerical_warning'
                rally_stop_time=plant.episode_time
                fall_time=rally_stop_time if failure=='fall' else None
                stop_launches=True;end_rally(pool,rally_stop_time);current_id=None
                print(f'RALLY FAILED {failure}; no reset/rescue/new launch; draining {pool.live}',flush=True)
            if stop_launches and plant.data.warning.number.any():raise FloatingPointError('Unsafe numerical drain; preserve unresolved flights')
            q=state.base_quat_w;yaw=math.atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]**2+q[3]**2))
            yaw_error=abs(math.degrees(math.atan2(math.sin(yaw-home_yaw),math.cos(yaw-home_yaw))))
            ema_speed=.2*metric['base_speed']+.8*ema_speed
            settled=(metric['drift_m']<=.05 and ema_speed<=.12 and metric['tilt_deg']<=12 and yaw_error<=5 and metric['base_z']>=.8 and not stop_launches)
            for row in rows.values():
                proxy=row.get('host_completion_clock_proxy_s')
                if proxy is not None and t>=proxy:update_recovery_dwell(row,now=t,settled_now=settled,required_dwell_s=.3)
            obslog.write(json.dumps({'sim_t':t,'observation':obs.tolist() if obs is not None else None,'target_source':source,
                'actions':actor_action,'base_at_observation':base_at_observation.tolist(),
                'target_position_w':target.pos_w.tolist(),'target_velocity_w':target.vel_w.tolist(),'time_to_strike':target.time_to_strike,
                'executed_qdes':plant.previous_qdes.tolist(),'hidden_norm':float(np.linalg.norm(policy.hidden)),'policy_reset_count':policy.reset_count,
                'base_z':metric['base_z'],'drift_m':metric['drift_m'],'tilt_deg':metric['tilt_deg'],
                'yaw_error_deg':yaw_error,'speed_ema':ema_speed,'rally_failed':stop_launches})+'\n')
            if render_states is not None and plant.episode_time+1e-8>=render_count/20.:
                render_states.write(json.dumps({'sim_t':plant.episode_time,'qpos':plant.data.qpos.tolist(),
                    'target_position':last_target.pos_w.tolist() if last_target is not None else None,
                    'target_velocity':last_target.vel_w.tolist() if last_target is not None else None})+'\n')
                render_count+=1
            if a.view and subcount%50==0:pool.draw(last_target)
            if stop_launches and not pool.live:break
            if all_released and not pool.live and t>=tail_end:break
        else:raise TimeoutError('Bounded wall timeout; unresolved/unlaunched serves remain failures')
    except (Exception,KeyboardInterrupt) as exc:
        error=repr(exc);stop_launches=True
        if rally_stop_time is None:rally_stop_time=plant.episode_time
        end_rally(pool,plant.episode_time)
        # Transport/render interruption does not erase already launched balls.
        # Stop actuator updates and transport; drain safe engine state offline.
        pool.before_hook=None;pool.after_hook=None
        drain_deadline=min(begin+wall_limit,time.monotonic()+30.)
        try:
            while pool.live and time.monotonic()<drain_deadline and not plant.data.warning.number.any():
                advance_without_actor(plant)
                if render_states is not None and plant.episode_time+1e-8>=render_count/20.:
                    render_states.write(json.dumps({'sim_t':plant.episode_time,'qpos':plant.data.qpos.tolist(),
                        'target_position':last_target.pos_w.tolist() if last_target is not None else None,
                        'target_velocity':last_target.vel_w.tolist() if last_target is not None else None})+'\n')
                    render_count+=1
        except Exception as drain_error:error+='; drain='+repr(drain_error)
    finally:
        try:plant.close_viewer()
        except Exception as e:error=(error or '')+'; viewer_close='+repr(e)
        if recorder is not None:
            try:recorder.close()
            except Exception as e:error=(error or '')+'; video_close='+repr(e)
        try:channel.write(b'{"close":true}\n');channel.flush()
        except OSError:pass
        channel.close();conn.close();trajectory.close();obslog.close();cmdlog.close()
        if render_states is not None:render_states.close()
        physical=pool.score.report();by_id={r['shot_id']:r for r in physical['rows']}
        for shot_id,row in rows.items():
            ph=by_id[shot_id];record=pool.records.get(shot_id)
            row['physical']=ph;row['trajectory_complete']=ph['trajectory_complete']
            row['qualified_contact']=ph['contact_pass'];row['legal_landing']=ph['landing_pass']
            row['raw_contact']=ph['racket_contact_count']>0
            row['raw_net_contact']=bool(ph['incoming_net_events'] or ph['post_racket_net_events'])
            row['events']=record['events'] if record else []
            row['valid_delivery']=valid_delivery(row['events']) if row['launched'] else False
            row['diagnostic_geometric_return']=physical_outcome(row['events'])['legal_return'] if row['launched'] else False
            row['landing_after_rally_failure']=bool(rally_stop_time is not None and ph['landing_event'] is not None and ph['landing_event']['stamp_ns']*1e-9>rally_stop_time)
            row['end_to_end_success']=bool(row['valid_delivery'] and row['planner_on_time'] and row['qualified_contact'] and row['legal_landing'] and row['trajectory_complete'] and not row['landing_after_rally_failure'])
            row['interrupted']=bool(row['launched'] and not row['trajectory_complete'])
            row['unlaunched']=not row['launched']
            row['planner_failure']=bool(row['launched'] and not row['planner_on_time'])
            row['unresolved']=row['interrupted']
            row['resolved_miss']=bool(row['launched'] and row['trajectory_complete'] and not row['raw_contact'])
            row['failed_return']=bool(row['launched'] and row['trajectory_complete'] and row['raw_contact'] and not row['legal_landing'])
            if record:
                for k in ['slot','released_at_s','terminal_at_s','terminal_kind']:row[k]=record[k]
            row.pop('_recovery_dwell_start',None)
        joined=[{'shot_id':r['shot_id'],'command_side':r['planner_side'],'lane':r['lane']} for r in rows.values()]
        thresholds=join_physical_evidence_by_side(joined,physical,min_samples_per_side=49,min_contact_rate=5/6,
            min_landing_rate=5/6,min_contacts_per_side=41,min_landings_per_side=41,
            min_landing_observations_per_side=41,min_global_contacts=84,min_global_landings=84,
            min_global_landing_observations=84,allowed_landing_censor_lanes=()) if not a.preflight else {'status':'NOT_APPLICABLE: separate five-serve integration preflight, no benchmark threshold verdict'}
        groups={}
        for label,group in [('nominal_forehand_lane',[r for r in rows.values() if r['position_table'][1]<-1.0125]),('nominal_backhand_lane',[r for r in rows.values() if r['position_table'][1]>=-1.0125]),('all',list(rows.values())),('forehand',[r for r in rows.values() if r['planner_side']=='forehand']),('backhand',[r for r in rows.values() if r['planner_side']=='backhand']),('unassigned',[r for r in rows.values() if r['planner_side'] is None])]:
            success=sum(r['end_to_end_success'] for r in group);delivery=sum(r['valid_delivery'] for r in group);contacts=sum(r['raw_contact'] for r in group);legal=sum(r['legal_landing'] for r in group)
            def rate(n,d):return {'numerator':n,'denominator':d,'rate':n/d if d else None}
            groups[label]={'scheduled_or_assigned':len(group),'launched':sum(r['launched'] for r in group),
                'unlaunched':sum(r['unlaunched'] for r in group),'raw_contacts':contacts,
                'qualified_contacts':sum(r['qualified_contact'] for r in group),'legal_landings':legal,
                'net_contacts':sum(r['raw_net_contact'] for r in group),'misses':sum(r['launched'] and r['trajectory_complete'] and not r['raw_contact'] for r in group),
                'valid_deliveries':delivery,'planner_on_time':sum(r['launched'] and r['planner_on_time'] for r in group),
                'planner_failures':sum(r['planner_failure'] for r in group),
                'failed_returns':sum(r['failed_return'] for r in group),'unresolved':sum(r['unresolved'] for r in group),
                'interrupted':sum(r['interrupted'] for r in group),'recovery_dwell_pass':sum(r['recovery_settled'] for r in group),
                'end_to_end':rate(success,len(group)),'valid_delivery_conditioned':rate(success,delivery),
                'contact_conditioned_legal':rate(legal,contacts)}
        streak=best=0
        for row in rows.values():streak=streak+1 if row['end_to_end_success'] else 0;best=max(best,streak)
        report={'label':'FIXED_HOME_100_REFERENCE_SEQUENCE_GRU_DIAGNOSTIC','formal_gate3_certification':'NOT_APPLICABLE','video_mode':'record actual states now; offline render after timing-critical evaluation','render_frames':render_count,
            'phase':'preflight' if a.preflight else '100-serve','iteration':policy.manifest['checkpoint_iteration'],
            'checkpoint_sha256':policy.manifest['checkpoint_sha256'],'onnx_sha256':policy.manifest['onnx_sha256'],
            'table_impact_model':a.table_impact_model,'profile_sha256':pool.slots[0].profile['sha256'],
            'scheduled_ball_count':len(ledger),'falls':int(failure=='fall'),
            'groups':groups,'rally_failure':failure,'fall_time_s':fall_time,'rally_stop_time_s':rally_stop_time,'error':error,
            'mujoco_warning_counts':plant.data.warning.number.tolist(),'post_failure_drain':'last controls held unchanged; no actor/planner action updates; bounded safe physical drain',
            'resets_after_initial':policy.reset_count-1,'peak_concurrent_balls':pool.peak_concurrent,
            'remaining_live_ids':pool.live,'longest_success_streak':best,'peak_clock_lag_s':peak_lag,
            'reset_free_segment':{'count':sum(r['launched'] for r in rows.values()),'launched_shot_ids':[r['shot_id'] for r in rows.values() if r['launched']],'ends_at_fall':failure=='fall'},
            'initial_idle_s':20.,'final_tail_end_s':tail_end,
            'integration_preflight_pass':bool(a.preflight and error is None and failure is None and pool.peak_concurrent>=2 and all(r['trajectory_complete'] and r['valid_delivery'] and r['planner_on_time'] and r['positive_planner_observation_ticks']>0 for r in rows.values())),
            'cadence':cadence,'cadence_within_reference_0_10s':all(abs(c['error_s'])<=.10 for c in cadence),
            'reference_physical_thresholds':thresholds,'simulated_seconds':plant.episode_time,
            'wall_seconds':time.monotonic()-begin,'last_recovery_dwell_pass':list(rows.values())[-1]['recovery_settled'],
            'lifecycle_difference':'Host deadline+0.5s completion proxy, not native preempt/native completion telemetry; no reset or rescue.',
            'physics_difference':('MuJoCo soft contacts; table uses saved PPQ004 law only at actual table-contact edge, 1ms quantized with no position change; racket remains MuJoCo soft mesh contact.' if a.table_impact_model=='ppq004' else 'MuJoCo soft contact; native instrument overwrites velocity with fitted impact map.')+' Reference strict edge-velocity qualification is NOT relaxed.',
            'drain_pass':all(r['trajectory_complete'] for r in rows.values() if r['launched'])}
        with (out/'events.jsonl').open('x') as f:
            for row in rows.values():
                for e in row['events']:f.write(json.dumps({'shot_id':row['shot_id'],'slot':row.get('slot'),**e})+'\n')
        dump(out/'physical-evidence.json',physical);dump(out/'per-ball.json',list(rows.values()));dump(out/'summary.json',report)
        simplekeys=['shot_id','lane','launched','unlaunched','planner_side','planner_on_time','planner_failure','valid_delivery','raw_contact','qualified_contact','raw_net_contact','legal_landing','resolved_miss','failed_return','trajectory_complete','interrupted','unresolved','recovery_settled','recovery_max_dwell_observed_s','end_to_end_success']
        with (out/'per-ball.csv').open('x') as f:
            w=csv.DictWriter(f,fieldnames=simplekeys,extrasaction='ignore');w.writeheader();w.writerows(rows.values())
        print(json.dumps({k:v for k,v in report.items() if k not in ['reference_physical_thresholds','cadence']},indent=2),flush=True)
    if error or failure:raise SystemExit(1)
    if a.preflight:
        if pool.peak_concurrent<2 or not report['drain_pass'] or not all(r['planner_on_time'] and r['positive_planner_observation_ticks']>0 and r['valid_delivery'] for r in rows.values()):
            raise SystemExit('Integration preflight failed: require concurrent handoff, natural drain, live on-time planner inputs and physical delivery for every preflight serve')

if __name__=='__main__':main()
