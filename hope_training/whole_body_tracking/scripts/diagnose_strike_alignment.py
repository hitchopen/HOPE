"""Reconstruct a recorded preflight and measure actual collision surfaces at 1 kHz.

Recorded actions and launch/release events only. No policy inference, planner
replacement, new target, collision override or clock compensation. Reconstruction
must match every saved qpos frame before any measurement is accepted.
"""
import argparse,csv,hashlib,json,math
from pathlib import Path
import numpy as np
from mujoco_eval_recurrent import Plant,dump
from concurrent_physical_balls import ConcurrentBalls


def read_rows(path):
    with Path(path).open() as f:return [json.loads(x) for x in f]


def geometry(plant,data,ball,t):
    ft=np.zeros(6)
    gap=plant.mj.mj_geomDistance(plant.model,data,ball.gid,ball.racket,100.,ft)
    return {'sim_t':float(t),'ball_w':data.geom_xpos[ball.gid].tolist(),
        'racket_geom_origin_w':data.geom_xpos[ball.racket].tolist(),
        'racket_site_w':data.site_xpos[plant.racket].tolist(),
        'racket_rotation_w':data.geom_xmat[ball.racket].reshape(3,3).tolist(),
        'racket_site_rotation_w':data.site_xmat[plant.racket].reshape(3,3).tolist(),
        'surface_gap_m':float(gap),'ball_to_racket_nearest_points_w':ft.tolist(),
        'ball_velocity_w':data.qvel[ball.v:ball.v+3].tolist(),
        'ball_omega_w':(data.xmat[ball.bid].reshape(3,3)@data.qvel[ball.v+3:ball.v+6]).tolist()}


def bracket(samples,t,key):
    times=np.array([x['sim_t'] for x in samples]);i=int(np.searchsorted(times,t))
    if not 0<i<len(times):raise ValueError('Timestamp outside flight')
    a,b=samples[i-1],samples[i];u=(t-times[i-1])/(times[i]-times[i-1])
    return (1-u)*np.array(a[key])+u*np.array(b[key]),[a['sim_t'],b['sim_t']]


def reconstruct(source,bundle,out):
    source=Path(source);out=Path(out);out.mkdir(parents=True,exist_ok=False)
    manifest=json.loads((Path(bundle)/'manifest.json').read_text())
    cfg=json.loads((source/'configuration.json').read_text())
    assert cfg['policy_manifest']['onnx_sha256']==manifest['onnx_sha256']
    assert cfg['policy_manifest']['checkpoint_sha256']==manifest['checkpoint_sha256']
    p=Plant(manifest,physical=True,ball_slots=cfg['slots'],table_impact_model=cfg.get('table_impact_model','soft'))
    rows=json.loads((source/'per-ball.json').read_text());pool=ConcurrentBalls(p,[r['shot_id'] for r in rows])
    trace=read_rows(source/'policy-trace.jsonl');renders=read_rows(source/'render-states.jsonl')
    render_by={round(x['sim_t'],6):x for x in renders};commands=read_rows(source/'planner-commands.jsonl')
    launched=set();released=set();samples={r['shot_id']:[] for r in rows};errors=[]
    view=p.mj.MjData(p.model)
    def before(t):
        for r in rows:
            sid=r['shot_id']
            if not r['launched']:continue
            if sid in launched and sid not in released and r.get('released_at_s') is not None and t+1e-8>=r['released_at_s']:
                pool.release(t);released.add(sid)
            if sid not in launched and t+1e-8>=r['launched_at_s']:
                pool.launch(r,t);launched.add(sid)
    def after(t):
        # Derived poses after mj_step refer to the pre-integration configuration.
        # Refresh separate scratch data; leave simulation warmstart/forces untouched.
        p.mj.mj_copyData(view,p.model,p.data);p.mj.mj_forward(p.model,view)
        for sid in launched:
            b=pool.slots[pool.records[sid]['slot']]
            if b.shot_id!=sid or b.retired:continue
            v=geometry(p,view,b,t);v['qpos']=p.data.qpos.tolist();samples[sid].append(v)
    pool.before_hook=before;pool.after_hook=after
    for tick in trace:
        if tick['actions'] is None:raise ValueError('Action-free failure drain needs explicit reconstruction support')
        assert abs(tick['sim_t']-p.episode_time)<1e-8
        p.step(tick['actions'])
        frame=render_by.get(round(p.episode_time,6))
        if frame is not None:errors.append(float(np.max(np.abs(p.data.qpos-np.array(frame['qpos'])))))
    if len(errors)!=len(renders) or max(errors)>1e-9:raise ValueError(f'Reconstruction mismatch: {len(errors)}/{len(renders)}, error={max(errors)}')
    results=[]
    for row in rows:
        sid=row['shot_id'];data=samples[sid]
        if not data:continue
        with (out/f'shot-{sid}-surface-samples.jsonl').open('x') as f:
            for v in data:f.write(json.dumps(v)+'\n')
        applied=[c for c in commands if c['shot_id']==sid and c['applied']]
        if len(applied)!=1:raise ValueError('Expected exactly one accepted planner command per flight')
        command=applied[0];t=row['planned_deadline_s'];b=pool.slots[row['slot']]
        q,interval=bracket(data,t,'qpos');view.qpos[:]=q;p.mj.mj_normalizeQuat(p.model,view.qpos);p.mj.mj_forward(p.model,view)
        at=geometry(p,view,b,t);at.pop('ball_velocity_w');at.pop('ball_omega_w') # scratch qvel is not interpolated
        at['interpolation_bracket_s']=interval
        near=min(data,key=lambda x:x['surface_gap_m']);near={k:v for k,v in near.items() if k!='qpos'}
        target=np.array(command['decoded']['position_w']);times=np.array([x['sim_t'] for x in data]);ball=np.array([x['ball_w'] for x in data])
        ix=np.flatnonzero((ball[:-1,0]>=target[0])&(ball[1:,0]<target[0]))
        crossing=None
        if len(ix):
            i=ix[0];u=(target[0]-ball[i,0])/(ball[i+1,0]-ball[i,0]);crossing={'sim_t':float(times[i]+u*(times[i+1]-times[i])),'ball_w':(ball[i]+u*(ball[i+1]-ball[i])).tolist()}
            crossing['relative_to_planned_deadline_s']=crossing['sim_t']-t
        obs=next(x for x in trace if x['sim_t']>=command['sim_t'] and x['target_source']=='actual_live_planner' and np.allclose(x['target_position_w'],target,atol=1e-9))
        anchor=command['decoded']['deadline_unix_s']-t
        normal=np.array(at['racket_rotation_w'])[:,0] # compiled mesh's thin axis
        wanted=np.array(command['decoded']['velocity_w']);wanted/=np.linalg.norm(wanted)
        angle=math.degrees(math.acos(float(np.clip(abs(normal@wanted),0,1))))
        point_error=np.array(at['ball_w'])-target;track_error=np.array(at['racket_site_w'])-target
        result={'shot_id':sid,'side':row['planner_side'],'frame':'host MuJoCo world; x toward far table, y robot left, z up; metres/seconds',
            'planner_command':command,'planned_deadline_sim_s':t,'planned_deadline_unix_s':command['decoded']['deadline_unix_s'],
            'at_deadline':at,'ball_minus_planner_point_m':point_error.tolist(),'racket_site_minus_planner_point_m':track_error.tolist(),
            'racket_site_tracking_error_m':float(np.linalg.norm(track_error)),
            'unsigned_surface_normal_vs_requested_velocity_deg':angle,
            'actual_strike_plane_crossing':crossing,'closest_surface':{**near,'unix_s':anchor+near['sim_t']},
            'closest_distance_definition':'signed ball sphere surface to convex collision mesh surface, mj_geomDistance; 1ms sampled global minimum before natural terminal',
            'first_actor_observation':{k:obs[k] for k in ['sim_t','base_at_observation','target_position_w','target_velocity_w','time_to_strike','observation']},
            'first_actor_true_deadline_countdown_s':t-obs['sim_t'],
            'first_actor_countdown_error_s':obs['time_to_strike']-(t-obs['sim_t'])}
        results.append(result)
    affected=[]
    audit_path=source.parent/(source.name+'-ros')/'planner-audit.csv'
    audits=list(csv.DictReader(audit_path.open())) if audit_path.exists() else []
    for result in results:
        for tick in trace:
            if not result['planner_command']['sim_t']<=tick['sim_t']<=result['planned_deadline_sim_s']:continue
            if tick['target_source']!='actual_live_planner' or not np.allclose(tick['target_position_w'],result['planner_command']['decoded']['position_w'],atol=1e-9):continue
            correct=result['planned_deadline_sim_s']-tick['sim_t'];error=tick['time_to_strike']-correct
            if abs(error)>1e-8:affected.append({'shot_id':result['shot_id'],'sim_t':tick['sim_t'],'actual_observed_tts':tick['time_to_strike'],'correct_deadline_countdown':correct,'error_s':error})
        identity=result['planner_command']['decoded']['identity'][0]
        audit=next((v for v in audits if v['command_seq']==str(identity)),None)
        if audit is not None:
            result['planner_audit_row']=audit
            anchor=result['planned_deadline_unix_s']-result['planned_deadline_sim_s']
            result['fit_source_sim_t']=float(audit['source_time_s'])-anchor
            sample=min(samples[result['shot_id']],key=lambda v:abs(v['sim_t']-result['fit_source_sim_t']))
            result['actual_ball_at_fit_source_nearest_1ms']=sample['ball_w']
    receipt={'source':str(source),'iteration':manifest['checkpoint_iteration'],'checkpoint_sha256':manifest['checkpoint_sha256'],'onnx_sha256':manifest['onnx_sha256'],
        'source_file_sha256':{f:hashlib.sha256((source/f).read_bytes()).hexdigest() for f in ['configuration.json','per-ball.json','planner-commands.jsonl','policy-trace.jsonl','render-states.jsonl','ball-trajectories.jsonl']},
        'maximum_qpos_error':max(errors),'matched_saved_frames':len(errors),'sampling_dt_s':.001,
        'measurement_method':'Recorded-action deterministic reconstruction verified against ALL saved qpos frames; scratch forward geometry; linear qpos interpolation with normalized quaternions at deadline; no live planner or actor rerun',
        'results':results,'affected_countdown_ticks':affected}
    dump(out/'alignment.json',receipt)
    write_svg(out,results,samples)
    print(json.dumps({k:v for k,v in receipt.items() if k!='results'},indent=2))
    for x in results:print(x['shot_id'],x['side'],'prediction_error',x['ball_minus_planner_point_m'],'tracking',x['racket_site_tracking_error_m'],'closest',x['closest_surface']['surface_gap_m'])


def write_svg(out,results,samples):
    # Standalone scientific projections, no plotting-package installation required.
    chosen=[next(x for x in results if x['side']==side) for side in ['forehand','backhand']]
    svg=['<svg xmlns="http://www.w3.org/2000/svg" width="1280" height="950" viewBox="0 0 1280 950"><rect width="1280" height="950" fill="white"/>',
         '<style>text{font:15px sans-serif}.title{font-size:20px;font-weight:bold}</style>',
         '<text x="30" y="30" class="title">Actual ball / collision surface at planned strike — frozen model29500</text>',
         '<text x="30" y="56">Host world metres. Orange: actual ball; blue: racket site; magenta: intended strike. No predicted path drawn.</text>']
    for row,x in enumerate(chosen):
        t=x['planned_deadline_sim_s'];a=[v for v in samples[x['shot_id']] if t-.4<=v['sim_t']<=t+.15]
        for col,(axis,label,bounds) in enumerate([(2,'x–z',(0,1.35,.65,1.5)),(1,'x–y',(0,1.35,-.5,.3))]):
            left=70+col*620;top=120+row*410;w=500;h=285;xlo,xhi,ylo,yhi=bounds
            def xy(p):return (left+(p[0]-xlo)/(xhi-xlo)*w,top+h-(p[axis]-ylo)/(yhi-ylo)*h)
            svg.append(f'<text x="{left}" y="{top-30}" class="title">{x["side"]}, shot {x["shot_id"]}: {label}</text>')
            svg.append(f'<rect x="{left}" y="{top}" width="{w}" height="{h}" fill="#fafafa" stroke="#555"/>')
            for k,color in [('ball_w','#d97800'),('racket_site_w','#1468c0')]:
                points=' '.join(f'{px:.2f},{py:.2f}' for px,py in map(lambda v:xy(v[k]),a));svg.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="2"/>')
            for k,color in [('ball_w','#d97800'),('racket_site_w','#1468c0')]:
                px,py=xy(x['at_deadline'][k]);svg.append(f'<circle cx="{px}" cy="{py}" r="6" fill="{color}"/>')
            target=x['planner_command']['decoded']['position_w'];px,py=xy(target)
            svg.append(f'<path d="M{px-9},{py}h18 M{px},{py-9}v18" stroke="#c000a0" stroke-width="3"/>')
            # Collision mesh normal axis, sign deliberately unsigned (two-sided face).
            origin=np.array(x['at_deadline']['racket_geom_origin_w']);normal=np.array(x['at_deadline']['racket_rotation_w'])[:,0];p0=xy(origin-normal*.12);p1=xy(origin+normal*.12)
            svg.append(f'<line x1="{p0[0]}" y1="{p0[1]}" x2="{p1[0]}" y2="{p1[1]}" stroke="#1468c0" stroke-width="4"/>')
            svg.append(f'<text x="{left}" y="{top+h+24}">x {xlo} → {xhi} m; {label[-1]} {ylo} → {yhi} m. Dots at t={t:.6f}s</text>')
            svg.append(f'<text x="{left}" y="{top+h+47}">closest surface gap {x["closest_surface"]["surface_gap_m"]:.4f}m at {x["closest_surface"]["sim_t"]:.6f}s</text>')
    svg.append('</svg>');(out/'alignment.svg').write_text('\n'.join(svg))


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    for k in ['source','bundle','output']:ap.add_argument('--'+k,type=Path,required=True)
    args=ap.parse_args();reconstruct(args.source,args.bundle,args.output)
