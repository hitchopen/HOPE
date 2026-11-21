"""Compare the actual C++ prediction with independently reconstructed ball states."""
import argparse,csv,hashlib,json,subprocess
from pathlib import Path
import numpy as np
from mujoco_eval_recurrent import ROOT,dump
from diagnose_strike_alignment import read_rows
OFFSET=np.array([.5,.7625,.76])


def state_at(rows,t):
    times=np.array([r['sim_t'] for r in rows]);i=int(np.searchsorted(times,t,side='right'))
    if i==0:
        if abs(t-times[0])>1e-6:return None
        i=1
    if i==len(rows):
        if abs(t-times[-1])>1e-6:return None
        return {k:np.array(rows[-1][k]) for k in ['p','v','w']}
    a,b=rows[i-1],rows[i];span=b['sim_t']-a['sim_t']
    u=(t-a['sim_t'])/span if span else 1.
    return {k:(1-u)*np.array(a[k])+u*np.array(b[k]) for k in ['p','v','w']}


def run(source,alignment,executable,out):
    source=Path(source);alignment=Path(alignment);executable=Path(executable);out=Path(out);out.mkdir(parents=True,exist_ok=False)
    receipt=json.loads((executable.parent/'cpp-trace-receipt.json').read_text())
    for p,digest in receipt['source_sha256'].items():
        if hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=digest:raise ValueError('C++ source changed: '+p)
    if hashlib.sha256(executable.read_bytes()).hexdigest()!=receipt['artifact_sha256'][executable.name]:raise ValueError('Trace executable changed')
    report=json.loads((alignment/'alignment.json').read_text());rows=json.loads((source/'per-ball.json').read_text())
    cfg=json.loads((source/'configuration.json').read_text());parameters=json.loads((ROOT/'a3_deploy/a3_deploy_example/models/model_29999/policy/params/planner-parameters.json').read_text())
    for k,v in {'drag_k':.12453915600992894,'magnus_k':.00399279130564425,'restitution_v':.9373319241133231,'post_net_future_bounce_tangential_gain':.41642144985036184,'table_friction_cap_mu':.218520906499864,'dt_integrate':.001}.items():
        if parameters[k]!=v:raise ValueError('Trace driver physics constants do not match recorded planner')
    result=[]
    for r in report['results']:
        sid=r['shot_id'];audit=r['planner_audit_row'];anchor=r['planned_deadline_unix_s']-r['planned_deadline_sim_s']
        args=[audit['source_time_s']]+[audit[k] for k in ['est_x','est_y','est_z','est_vx','est_vy','est_vz','spin_wx_rad_s','spin_wy_rad_s','spin_wz_rad_s']]
        traces={}
        for label,plane in [('original',None),('continued',-.02)]:
            p=out/f'shot-{sid}-cpp-{label}.csv'
            with p.open('x') as f:
                f.write('kind,unix_s,x,y,z,vx,vy,vz,wx,wy,wz\n');f.flush()
                subprocess.run([str(executable),*args,*([str(plane)] if plane is not None else [])],stdout=f,check=True)
            traces[label]=list(csv.DictReader(p.open()))
        crossing=traces['original'][-1]
        check_p=np.array([float(crossing[k]) for k in ['x','y','z']]);expected=np.array([float(audit[k]) for k in ['strike_x','strike_y','strike_z']])
        if np.max(np.abs(check_p-expected))>1e-9 or abs(float(crossing['unix_s'])-r['planned_deadline_unix_s'])>1e-6:raise ValueError('Instrumented C++ predictor did not reproduce recorded command')
        predicted=[{'sim_t':float(v['unix_s'])-anchor,'p':(np.array([float(v[k]) for k in ['x','y','z']])+OFFSET).tolist(),
            'v':[float(v[k]) for k in ['vx','vy','vz']],'w':[float(v[k]) for k in ['wx','wy','wz']],'kind':v['kind']} for v in traces['continued']]
        predicted.sort(key=lambda x:x['sim_t'])
        measured=read_rows(alignment/f'shot-{sid}-surface-samples.jsonl')
        actual=[{'sim_t':v['sim_t'],'p':v['ball_w'],'v':v['ball_velocity_w'],'w':v['ball_omega_w']} for v in measured]
        physical=next(x for x in rows if x['shot_id']==sid)
        event=next(x for x in physical['events'] if x['kind']=='table_contact')
        pre=next(x for x in predicted if x['kind']=='pre_bounce');post=next(x for x in predicted if x['kind']=='post_bounce')
        separation=next(x['sim_t']+.001 for x in actual if x['sim_t']>event['episode_t'] and x['p'][2]>.78 and x['v'][2]>0)
        checkpoints=[('planner_input',float(audit['source_time_s'])-anchor),('shared_pre_bounce',pre['sim_t']-.001),
            ('planner_impact_pre',pre['sim_t']),('planner_impact_post',post['sim_t']),
            ('actual_contact_pre',event['episode_t']-.001),('actual_contact_post',event['episode_t']),
            ('actual_separated',separation),
            ('planned_strike_deadline',r['planned_deadline_sim_s'])]
        if r['actual_strike_plane_crossing'] is not None:
            checkpoints.insert(-1,('actual_strike_plane_crossing',r['actual_strike_plane_crossing']['sim_t']))
        first_racket=min((e['episode_t'] for e in physical['events'] if e['kind']=='racket_contact'),default=None)
        evidence=[]
        for kind,t in checkpoints:
            a=state_at(actual,t);b=state_at(predicted,t)
            if kind=='planner_impact_pre':b={k:np.array(pre[k]) for k in ['p','v','w']}
            if kind=='planner_impact_post':b={k:np.array(post[k]) for k in ['p','v','w']}
            evidence.append({'event':kind,'sim_t':t,'unix_s':t+anchor,'after_actual_racket_contact':first_racket is not None and t>=first_racket,'actual':{k:v.tolist() for k,v in a.items()} if a else None,'planner':{k:v.tolist() for k,v in b.items()} if b else None,
                'position_error_m':(a['p']-b['p']).tolist() if a and b else None,
                'velocity_error_mps':(a['v']-b['v']).tolist() if a and b else None})
        stream=[]
        for a in actual:
            b=state_at(predicted,a['sim_t'])
            if b is None:continue
            stream.append({'sim_t':a['sim_t'],'actual_p':a['p'],'planner_p':b['p'].tolist(),'actual_v':a['v'],'planner_v':b['v'].tolist(),'actual_w':a['w'],'planner_w':b['w'].tolist(),
                'position_error_m':float(np.linalg.norm(np.array(a['p'])-b['p'])),'velocity_error_mps':float(np.linalg.norm(np.array(a['v'])-b['v']))})
        dump(out/f'shot-{sid}-comparison-stream.json',stream)
        before=[v for v in stream if v['sim_t']<pre['sim_t']-.001]
        result.append({'shot_id':sid,'side':r['side'],'cpp_recorded_command_reproduction':'PASS','events':evidence,'actual_contact':event,
            'incoming_prediction_valid_until_racket_contact_s':first_racket,
            'actual_strike_plane_crossing':r['actual_strike_plane_crossing'],
            'pre_bounce_max_position_error_m':max(v['position_error_m'] for v in before),
            'pre_bounce_max_velocity_error_mps':max(v['velocity_error_mps'] for v in before),
            'first_over_1mm_position_error_sim_s':next((v['sim_t'] for v in stream if v['position_error_m']>.001),None),
            'first_over_0_05mps_velocity_error_sim_s':next((v['sim_t'] for v in stream if v['velocity_error_mps']>.05),None)})
    dump(out/'comparison.json',{'iteration':report['iteration'],'checkpoint_sha256':report['checkpoint_sha256'],'onnx_sha256':report['onnx_sha256'],
        'source':str(source),'table_impact_model':cfg.get('table_impact_model','soft'),'frame':'MuJoCo world, metres/seconds/rad per second; P to W [.5,.7625,.76]',
        'cpp_trace_receipt':receipt,'reconstruction_max_qpos_error':report['maximum_qpos_error'],'results':result})
    fields=['shot_id','event','sim_t','unix_s','actual_p','planner_p','actual_v','planner_v','actual_w','planner_w','position_error_m','velocity_error_mps']
    with (out/'checkpoints.csv').open('x') as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader()
        for r in result:
            for e in r['events']:
                w.writerow({'shot_id':r['shot_id'],**{k:e[k] for k in ['event','sim_t','unix_s','position_error_m','velocity_error_mps']},**{f'{prefix}_{k}':e[label][k] if e[label] else None for prefix,label in [('actual','actual'),('planner','planner')] for k in ['p','v','w']}})
    for r in result:print('shot',r['shot_id'],'pre-bounce max',r['pre_bounce_max_position_error_m'],r['pre_bounce_max_velocity_error_mps'],'first 1mm',r['first_over_1mm_position_error_sim_s'])

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for k in ['source','alignment','trace-executable','output']:p.add_argument('--'+k,type=Path,required=True)
    a=p.parse_args();run(a.source,a.alignment,a.trace_executable,a.output)
