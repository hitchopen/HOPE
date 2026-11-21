"""100 scheduled serves through the actual ROS planner; no training/hardware IO.

Separate preflight and scored processes. Absolute planner deadlines require real
wall pacing: abort rather than report a benchmark with a warped time base.
"""
import argparse,csv,hashlib,json,socket,sys,time
from collections import deque
from pathlib import Path
import numpy as np
from mujoco_eval_recurrent import Plant,RecurrentOnnxPolicy,ObsTarget,dump,ROOT
from planner_rally_protocol import OFFSET,decode_command,physical_outcome,summarize,valid_delivery,finalize_row


def blank(serve,reason='not_started'):
    return {**serve,'started':False,'valid_delivery':False,'planner_on_time':False,'contact':False,
        'miss':False,'net_contact':False,'legal_return':False,'failed_return':False,
        'fall_reset':False,'numerical_reset':False,'ready_proxy':False,
        'interrupted':True,'end_to_end_success':False,'reason':reason,
        'racket_contacts':0,'landing_position':None,'commands':[],'events':[]}


def spawn(pb,serve,episode):
    if pb.active:raise RuntimeError('Previous ball has not resolved or been marked interrupted')
    pb.park();pb.contacts=set();pb.history.clear();pb.reference=np.empty((0,3))
    p=np.array(serve['position_w']);v=np.array(serve['velocity_w']);w=np.array(serve['spin_w'])
    pb.current={'episode':episode,'bank_row':None,'serve_id':serve['id'],'side':serve['side'],
        'racket_contacts':0,'net_contacts':0,'floor_contacts':0,'table_contacts':0,
        'net_clear':False,'first_landing':None,'legal_return':False,'arrival_error_m':None}
    pb.target_position=p.copy();pb.strike_at=1e99;pb.arrival_done=True
    pb.pending=(pb.plant.episode_time,p,v,w)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bundle',type=Path,required=True);p.add_argument('--protocol',type=Path,required=True)
    p.add_argument('--socket',required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--preflight',action='store_true');p.add_argument('--view',action='store_true')
    a=p.parse_args();protocol=json.loads(a.protocol.read_text())
    raise SystemExit('DRAFT SUPERSEDED: fixed-home-100 reference audit requires multi-ball handoff and matched recurrent lifecycle/scoring; see docs/GRU_FIXED_HOME_100_AUDIT.md. No evaluation started.')
    serves=protocol['serves']
    if len(serves)!=100 or sum(s['side']==0 for s in serves)!=50:raise ValueError('Requires frozen 50/50, 100-serve protocol')
    if a.preflight:
        # Both sides, independent scored state and output. Avoid selecting outcomes.
        serves=[next(s for s in serves if s['side']==side).copy() for side in [0,1]]
        for i,s in enumerate(serves):s['launch_at_s']=2.+5*i
    out=a.output;out.mkdir(parents=True,exist_ok=False)
    policy=RecurrentOnnxPolicy(a.bundle);plant=Plant(policy.manifest,physical=True)
    dump(out/'policy-manifest.json',policy.manifest)
    dump(out/'evaluation-config.json',{'protocol':protocol,'physics':plant.ball_physics.cfg,
        'pair_settings':plant.ball_physics.pair_settings,'physics_dt':plant.model.opt.timestep,
        'control_dt':plant.dt,'action':policy.manifest['action'],'actuator':policy.manifest['actuator'],
        'episode_timeouts_disabled':True,'reset_delay_rule':'initial0; full resets alternate0/max saved delay','real_planner_required':True})
    plant.reset(0);policy.reset_episode();pb=plant.ball_physics
    pb.history=deque(maxlen=5000)  # Entire bounded serve; display decimates, trace does not.
    if a.view:plant.open_viewer()
    conn=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);conn.settimeout(.25);conn.connect(a.socket);stream=conn.makefile('rwb')
    rows=[blank(s) for s in serves];resets=[];active=None;next_index=0;epoch=0
    last_target=None;last_deadline=None;current_command=None;spawn_ns=0;event_start=0
    ready_dwell=0.;peak_lag=0.;sensor_count=0;clock_anchor=time.time();begin=time.monotonic()
    sim_end=serves[-1]['launch_at_s']+protocol['slot_s'];wall_limit=90 if a.preflight else protocol['wall_timeout_s']
    command_trace=(out/'commands.jsonl').open('x');trace=(out/'trace.jsonl').open('x')
    completed=(out/'completed-serves.jsonl').open('x');global_t=0.;error=None
    sub_tick=0;control_start=0.;plant_start=0.
    original_after=pb.after
    def observe(subtime):
        nonlocal current_command,last_target,last_deadline,peak_lag,sub_tick,sensor_count
        was_active=pb.active
        original_after(subtime);sub_tick+=1
        if was_active:
            after,_=pb.state();before=pb.before_p
            for plane,kind in [(1.87,'net_crossing_incoming'),(.58,'hit_plane_incoming')]:
                if before[0]>plane>=after[0]:
                    f=(plane-before[0])/(after[0]-before[0]);cross=before+f*(after-before)
                    pb.event(kind,subtime,cross,clear=bool(cross[2]>.9325 and abs(cross[1])<=.7625))
            if after[2]<-.1:pb.event('out_of_play',subtime,after)
        if active is not None and 'resolution_snapshot' not in rows[active]:
            outcome=physical_outcome(pb.events[event_start:])
            if outcome['resolved']:
                rows[active]['resolution_snapshot']=outcome
                rows[active]['physical_resolved_at_s']=control_start+subtime-plant_start
        if sub_tick%5:return
        t=control_start+subtime-plant_start
        target_wall=begin+t;lag=time.monotonic()-target_wall;peak_lag=max(peak_lag,lag)
        if lag>protocol['max_realtime_lag_s']:raise RuntimeError(f'Realtime capacity failed: lag {lag:.4f}s > 50ms; planner wall deadlines cannot be validated')
        time.sleep(max(0,target_wall-time.monotonic()))
        state=plant.state();pos,vel=pb.state();stamp=int((clock_anchor+t)*1e9)
        request={'stamp_ns':stamp,'sim_t':t,'serve_id':rows[active]['id'] if active is not None else None,
            'base_table':(state.base_pos_w-OFFSET).tolist(),'base_quat_wxyz':state.base_quat_w.tolist(),
            'ball_active':pb.active,'ball_table':(pos-OFFSET).tolist(),
            'ball_quat_wxyz':plant.data.qpos[pb.q+3:pb.q+7].tolist()}
        stream.write((json.dumps(request)+'\n').encode());stream.flush()
        line=stream.readline()
        if not line:raise RuntimeError('ROS planner bridge closed unexpectedly')
        reply=json.loads(line);sensor_count+=1
        if 'error' in reply:raise RuntimeError(reply['error'])
        for raw in reply['commands']:
            command,reason=decode_command(raw['values'],spawn_ns,time.time_ns(),protocol['min_command_lead_s'])
            item={'sim_t':t,'serve_index':active,'reason':reason,**raw,'decoded':command,
                'applied':bool(active is not None and command is not None and current_command is None and rows[active]['reason']=='in_flight')}
            command_trace.write(json.dumps(item)+'\n')
            if active is None:continue
            rows[active]['commands'].append(item)
            if command is not None and current_command is None and rows[active]['reason']=='in_flight':
                current_command=command;last_target=ObsTarget(np.array(command['position_w']),np.array(command['velocity_w']),0.)
                last_deadline=command['deadline_unix_s']-clock_anchor;rows[active]['planner_on_time']=True
                rows[active]['accepted_command']=command
    pb.after=observe
    def close_serve(reason):
        nonlocal active
        if active is None:return
        row=rows[active];ev=[{**e,'serve_id':row['id']} for e in pb.events[event_start:]]
        finalize_row(row,ev,reason,ready_dwell)
        completed.write(json.dumps(row)+'\n');completed.flush()
        print('SERVE',json.dumps({k:row[k] for k in ['id','side_name','reason','valid_delivery','planner_on_time','contact','legal_return','interrupted','ready_proxy']}),flush=True)
        pb.finish(reason);active=None
    try:
        for tick in range(int(round(sim_end/.02))):
            global_t=tick*.02
            if time.monotonic()-begin>wall_limit:raise TimeoutError('Total evaluation wall timeout')
            if plant.viewer is not None and not plant.viewer.is_running():raise RuntimeError('Viewer closed; remaining serves interrupted')
            if next_index<len(serves) and global_t+1e-8>=serves[next_index]['launch_at_s']:
                if active is not None:close_serve('slot_deadline_unresolved')
                active=next_index;next_index+=1;row=rows[active];row['reason']='in_flight';row['started']=True
                current_command=None;spawn_ns=int((clock_anchor+global_t)*1e9);event_start=len(pb.events);ready_dwell=0.
                spawn(pb,row,epoch)
            control_start=global_t;plant_start=plant.episode_time
            if last_target is None:target=ObsTarget(plant.data.site_xpos[plant.racket].copy(),np.zeros(3),-1.)
            else:target=ObsTarget(last_target.pos_w,last_target.vel_w,min(.65,last_deadline-global_t))
            try:
                obs_base=plant.state().base_pos_w.copy()
                source_before='planner' if last_target is not None else 'idle_no_plan'
                obs=plant.observe(target)
                np.testing.assert_allclose(obs[103:106],target.pos_w-obs_base,atol=2e-7)
                np.testing.assert_allclose(obs[106:109],target.vel_w,atol=2e-7)
                if active is not None and current_command is not None and obs[109]>0:
                    rows[active]['positive_planner_observation_ticks']=rows[active].get('positive_planner_observation_ticks',0)+1
                action=policy.infer(obs)
                plant.step(action)
                metric=plant.metrics();warning=bool(plant.data.warning.number.any())
                fallen=metric['base_z']<.8 or metric['tilt_deg']>60
                if fallen or warning:
                    if active is not None:rows[active]['fall_reset']=fallen;rows[active]['numerical_reset']=warning
                    close_serve('fall' if fallen else 'mujoco_warning')
                    resets.append({'sim_t':global_t,'serve_id':next_index-1,'reason':'fall' if fallen else 'mujoco_warning'})
                    epoch+=1;plant.reset((epoch%2)*int(policy.manifest['action']['delay_max']));policy.reset_episode();last_target=last_deadline=None
                    continue
            except FloatingPointError as e:
                if active is not None:rows[active]['numerical_reset']=True
                close_serve('numerical_failure');resets.append({'sim_t':global_t,'serve_id':next_index-1,'reason':repr(e)})
                epoch+=1;plant.reset((epoch%2)*int(policy.manifest['action']['delay_max']));policy.reset_episode();last_target=last_deadline=None
                continue
            if active is not None:
                row=rows[active];events=pb.events[event_start:]
                row['valid_delivery']=valid_delivery(events)
                # Actual implemented position/speed proxy, continuous dwell;
                # do not call this the full training or hardware READY gate.
                current_outcome=physical_outcome(events)
                if current_outcome['resolved']:
                    proxy=metric['drift_m']<=.1 and metric['base_speed']<=.2
                    ready_dwell=ready_dwell+.02 if proxy else 0.
                    if ready_dwell>=.3 and 'ready_proxy_first_at_s' not in row:row['ready_proxy_first_at_s']=global_t+.02
                if global_t+.02>=row['launch_at_s']+protocol['slot_s']-1e-8:
                    close_serve('resolved' if current_outcome['resolved'] else 'slot_deadline_unresolved')
            pos,vel=pb.state()
            trace.write(json.dumps({'sim_t':global_t,'serve_id':rows[active]['id'] if active is not None else None,
                'ball_position':pos.tolist(),'ball_velocity':vel.tolist(),'ball_active':pb.active,
                'observation':obs.tolist(),'hidden_norm':float(np.linalg.norm(policy.hidden)),
                'policy_reset_count':policy.reset_count,'base_z':metric['base_z'],'tilt_deg':metric['tilt_deg'],
                'target_source':source_before,'base_position_at_observation':obs_base.tolist(),
                'target_position':target.pos_w.tolist(),'target_velocity':target.vel_w.tolist(),'time_to_strike':target.time_to_strike})+'\n')
            # Draw at 20 Hz; physical observation transport remains 200 Hz.
            if a.view and tick%3==0:
                history=pb.history
                pb.history=deque(list(history)[::3],maxlen=5000)
                try:
                    side=rows[active]['side'] if active is not None else 0
                    if current_command is not None:plant.draw(target,side)
                    else:
                        with plant.viewer.lock():
                            pb.decorate(plant.viewer.user_scn,target,side)
                            # Hide target cross/label when no new planner command exists.
                            plant.viewer.user_scn.ngeom=max(0,plant.viewer.user_scn.ngeom-4)
                        plant.viewer.set_texts((plant.mj.mjtFontScale.mjFONTSCALE_150,
                            plant.mj.mjtGridPos.mjGRID_TOPRIGHT,
                            'ACTUAL PHYSICAL BALL | PLANNER PENDING\nOrange incoming; cyan after force contact\nNo new planned strike target to display',''))
                        plant.viewer.sync()
                finally:pb.history=history
    except (Exception,KeyboardInterrupt) as e:
        error=repr(e)
        close_serve('evaluation_interrupted')
    finally:
        close_serve('evaluation_end')
        plant.close_viewer();trace.close();command_trace.close();completed.close()
        try:stream.write(b'{"close":true}\n');stream.flush()
        except OSError:pass
        stream.close();conn.close()
        dump(out/'per-serve.json',rows)
        keys=sorted(set().union(*(r.keys() for r in rows))-{'commands','events'})
        with (out/'per-serve.csv').open('x') as f:
            w=csv.DictWriter(f,fieldnames=keys,extrasaction='ignore');w.writeheader()
            for row in rows:w.writerow({k:json.dumps(row.get(k)) if isinstance(row.get(k),(list,dict)) else row.get(k) for k in keys})
        with (out/'events.jsonl').open('x') as f:
            for row in rows:
                for event in row['events']:f.write(json.dumps(event)+'\n')
        report={'phase':'preflight' if a.preflight else 'scored','error':error,'summary':summarize(rows),
            'resets':resets,'total_reset_events_including_warmup':len(resets),'fall_reset_events':sum(r['reason']=='fall' for r in resets),'policy_reset_count_including_initial':policy.reset_count,'peak_realtime_lag_s':peak_lag,
            'sensor_messages':sensor_count,'simulated_seconds':global_t+.02,'wall_seconds':time.monotonic()-begin,
            'checkpoint':policy.manifest['checkpoint'],'iteration':policy.manifest['checkpoint_iteration'],
            'checkpoint_sha256':policy.manifest['checkpoint_sha256'],'bundle':str(a.bundle),
            'protocol_sha256':hashlib.sha256(a.protocol.read_bytes()).hexdigest(),'command':sys.argv,
            'clock_anchor_unix_s':clock_anchor,'actor_contract':'unchanged 110D; target only from accepted live planner command, neutral idle before first command',
            'ready_definition':'drift<=0.1m and base planar speed<=0.2m/s for continuous 0.3s after physical resolution; partial proxy only',
            'episode_timeouts':'12s task timeouts disabled by evaluation-only driver; no resets per serve',
            'preflight_checks':'strict frame/schema/time/identity checks; 200Hz poses; <=50ms wall lag; deadline >=40ms at receipt'}
        dump(out/'summary.json',report)
        print(json.dumps(report,indent=2),flush=True)
    if error:raise SystemExit(1)
    if a.preflight and not all(r['valid_delivery'] and r['planner_on_time'] and r.get('positive_planner_observation_ticks',0)>0 and not r['interrupted'] for r in rows):
        raise SystemExit('Integration preflight failed: both serves must deliver, receive on-time live plans, and resolve; scored run forbidden')

if __name__=='__main__':main()
