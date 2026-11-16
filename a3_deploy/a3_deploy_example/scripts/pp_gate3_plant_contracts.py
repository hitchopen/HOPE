"""Required Gate3 checks missing from the older plant-report CLI.

Definitions/limits follow the archived professor report
docs/gates/model14600_gate3_30balls_20261013/pp_mujoco_plant_report.json:
strict pre-MOTION foot anchors, immutable HOME-yaw frame, actual HOLD windows,
and instrumented robot/table/net/self contact counters. No success is inferred
from absent columns. Optional-reach certification remains explicitly unsupported
here; the fixed-HOME model19000 profile disables optional reach.
"""
import bisect
import csv
import hashlib
import json
import math
import statistics
from pathlib import Path


LIMITS = {
    'home-base-yaw-peak-deg':12., 'home-base-yaw-end-deg':5.,
    'home-support-yaw-peak-deg':12., 'home-support-yaw-end-deg':5.,
    'home-support-midpoint-peak-m':.10, 'home-support-midpoint-end-m':.03,
    'home-foot-anchor-peak-m':.12, 'home-foot-anchor-end-m':.03,
    'home-foot-heading-peak-deg':15., 'home-foot-heading-end-deg':3.,
    'home-foot-tilt-peak-deg':12., 'home-foot-tilt-end-deg':5.,
    'home-level2-width-allowance-m':.08, 'home-fore-aft-stagger-m':.10,
    'torso-hold-pitch-median-deg':3., 'torso-hold-pitch-p95-deg':5., 'torso-hold-pitch-peak-deg':8.,
    'torso-hold-roll-median-deg':3., 'torso-hold-roll-p95-deg':5., 'torso-hold-roll-peak-deg':8.,
    'torso-hold-yaw-median-deg':3., 'torso-hold-yaw-p95-deg':5., 'torso-hold-yaw-peak-deg':8.,
}


def add_cli(p):
    for name in ('home-support-frame','torso-hold','torso-hold-three-axis','collision-ledger','optional-reach'):
        p.add_argument('--require-'+name, action='store_true')
    for name, default in LIMITS.items():
        p.add_argument('--max-'+name, type=float, default=default)
    p.add_argument('--min-home-signed-width-m',type=float,default=.26)
    p.add_argument('--max-home-signed-width-m',type=float,default=.32)
    p.add_argument('--min-torso-hold-window-s',type=float,default=.20)
    p.add_argument('--torso-hold-settle-trim-s',type=float,default=.12)
    p.add_argument('--physical-ball-report',type=Path)
    p.add_argument('--optional-reach-distribution-contract',default='stress_balanced_10_8_8_v1')
    p.add_argument('--optional-reach-trajectory-qualification',default='NOT_PROVEN')
    p.add_argument('--optional-reach-expected-flights',type=int,default=26)


def angle(x):
    return math.degrees(math.atan2(math.sin(x), math.cos(x)))


def rpy(row, name):
    w,x,y,z=(row[name+'_q'+c] for c in 'wxyz')
    norm=w*w+x*x+y*y+z*z
    if abs(norm-1)>1e-3:
        raise ValueError(f'non-unit {name} quaternion')
    return (math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),
            math.asin(max(-1.,min(1.,2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))


def quantile(values, q):
    values=sorted(values)
    at=(len(values)-1)*q;lo=int(at);hi=math.ceil(at)
    return values[lo]+(values[hi]-values[lo])*(at-lo)


def required_rows(path, names):
    with path.open(newline='') as f:
        reader=csv.DictReader(f)
        missing=set(names)-set(reader.fieldnames or ())
        if missing: raise ValueError(f'{path.name} missing required columns: {sorted(missing)}')
        for row in reader:
            result={k:float(row[k]) for k in names if k!='wall_time_ns'}
            result['wall_time_ns']=int(row['wall_time_ns'])
            if not all(math.isfinite(v) for v in result.values()):
                raise ValueError(f'{path.name}: nonfinite required evidence')
            yield result


def evaluate(args):
    requested = [name for name in ('home_support_frame','torso_hold','collision_ledger','optional_reach')
                 if getattr(args,'require_'+name)]
    if args.require_torso_hold_three_axis and not args.require_torso_hold:
        raise ValueError('--require-torso-hold-three-axis requires --require-torso-hold')
    if not requested: return {}
    for name,value in vars(args).items():
        if name.startswith(('max_home_','min_home_','max_torso_','min_torso_','torso_hold_settle_')):
            if not math.isfinite(value) or value<0: raise ValueError('invalid limit '+name)
    if args.max_home_signed_width_m<args.min_home_signed_width_m:
        raise ValueError('unordered signed-width bounds')
    trace=list(required_rows(args.runner_trace,['wall_time_ns','mode','level','reach_level']))
    if not trace: raise ValueError('empty Runner evidence')
    times=[x['wall_time_ns'] for x in trace]
    if any(a>=b for a,b in zip(times,times[1:])): raise ValueError('nonmonotonic Runner clock')
    motion=[x for x in trace if x['mode']==3]
    if not motion: raise ValueError('no actual MOTION samples')
    start,end=motion[0]['wall_time_ns'],motion[-1]['wall_time_ns']
    fields=['wall_time_ns','base_x','base_y']
    for body in ('base','torso','left_foot','right_foot'):
        fields += [body+'_q'+a for a in 'wxyz']
    fields += [side+'_foot_'+a for side in ('left','right') for a in 'xy']
    fields += ['robot_collision_ledger_v1']+['robot_'+kind+'_contact_count' for kind in ('table','net','self')]
    anchor=None; plant=[]
    for row in required_rows(args.plant,fields):
        stamp=row['wall_time_ns']
        if stamp<start: anchor=row
        elif stamp<=end: plant.append(row)
    if anchor is None or (start-anchor['wall_time_ns'])/1e9>.25:
        raise ValueError('missing/farther-than-0.25s strict pre-MOTION plant anchor')
    if len(plant)<2: raise ValueError('insufficient synchronized MOTION plant samples')
    # Source coverage, not policy success: missing endpoints/gaps must stay visible.
    stamps=[row['wall_time_ns'] for row in plant]
    if (abs(stamps[0]-start)>50_000_000 or abs(stamps[-1]-end)>50_000_000 or
            any(b<=a or b-a>50_000_000 for a,b in zip(stamps,stamps[1:]))):
        raise ValueError('MOTION plant coverage gap exceeds 50ms')
    out={}
    home_yaw=rpy(anchor,'base')[2]
    if args.require_home_support_frame:
        def geometry(row):
            lx,ly,rx,ry=(row[k] for k in ('left_foot_x','left_foot_y','right_foot_x','right_foot_y'))
            return lx,ly,rx,ry,math.atan2(rx-lx,ly-ry)
        alx,aly,arx,ary,syaw=geometry(anchor)
        ah=(rpy(anchor,'left_foot')[2],rpy(anchor,'right_foot')[2])
        series={k:[] for k in ('base_yaw','support_yaw','support_midpoint','foot_anchor','foot_heading','foot_tilt','fore_aft_stagger','signed_width')}
        for row in plant:
            lx,ly,rx,ry,yaw=geometry(row);l=rpy(row,'left_foot');r=rpy(row,'right_foot')
            values=(abs(angle(rpy(row,'base')[2]-home_yaw)),abs(angle(yaw-syaw)),
                    math.hypot((lx+rx-alx-arx)/2,(ly+ry-aly-ary)/2),
                    max(math.hypot(lx-alx,ly-aly),math.hypot(rx-arx,ry-ary)),
                    max(abs(angle(l[2]-ah[0])),abs(angle(r[2]-ah[1]))),
                    math.degrees(max(abs(l[0]),abs(l[1]),abs(r[0]),abs(r[1]))),
                    abs((lx-rx)*math.cos(syaw)+(ly-ry)*math.sin(syaw)),
                    -(lx-rx)*math.sin(syaw)+(ly-ry)*math.cos(syaw))
            for k,v in zip(series,values):series[k].append(v)
        checks={}
        for metric in ('base_yaw','support_yaw','support_midpoint','foot_anchor','foot_heading','foot_tilt'):
            unit='deg' if metric in ('base_yaw','support_yaw','foot_heading','foot_tilt') else 'm'
            for phase,actual in [('peak',max(series[metric])),('end',series[metric][-1])]:
                checks[metric+'_'+phase]=actual<=getattr(args,f'max_home_{metric}_{phase}_{unit}')
        checks['signed_width_min']=min(series['signed_width'])>=args.min_home_signed_width_m
        # No authorization is synthesized from a label. Model19000 has disabled
        # optional reach; a future nonzero reach run needs its genuine provider.
        checks['signed_width_max_authorized']=max(series['signed_width'])<=args.max_home_signed_width_m
        checks['fore_aft_stagger']=max(series['fore_aft_stagger'])<=args.max_home_fore_aft_stagger_m
        out['home_support_frame']={'contract':'pre_motion_world_foot_anchor_support_frame_v2',
            'anchor_contract':'last_strict_pre_motion_plant_sample_v2', 'anchor_wall_time_ns':anchor['wall_time_ns'],
            'anchor_age_s':(start-anchor['wall_time_ns'])/1e9,'evidence_complete':True,
            'checks':checks,'pass':all(checks.values()),'failures':[k for k,v in checks.items() if not v],
            'rows':len(plant),'peak':{k:max(v) for k,v in series.items()},
            'final':{k:v[-1] for k,v in series.items()},
            'signed_width_min_m':min(series['signed_width']),'signed_width_max_m':max(series['signed_width'])}
    if args.require_torso_hold:
        windows=[]; current=[]
        for row in trace:
            if row['mode']==3 and row['level']==0:
                if current and row['wall_time_ns']-current[-1]['wall_time_ns']>50_000_000:
                    windows.append(current);current=[]
                current.append(row)
            elif current:windows.append(current);current=[]
        if current:windows.append(current)
        selected=[]
        for window in windows:
            lo=window[0]['wall_time_ns']+int(args.torso_hold_settle_trim_s*1e9)
            hi=window[-1]['wall_time_ns']
            if (hi-lo)/1e9<args.min_torso_hold_window_s:continue
            samples=[row for row in plant if lo<=row['wall_time_ns']<=hi]
            if not samples:continue
            angles=[rpy(row,'torso') for row in samples]
            metrics={}
            for i,axis in enumerate(('roll','pitch','yaw')):
                values=[abs(angle(a[i]-(home_yaw if axis=='yaw' else 0))) for a in angles]
                metrics[axis]={'median':statistics.median(values),'p95':quantile(values,.95),'peak':max(values)}
            selected.append({'start_ns':lo,'end_ns':hi,'rows':len(samples),'metrics':metrics})
        axes=('roll','pitch','yaw') if args.require_torso_hold_three_axis else ('pitch',)
        failures=[]
        if not selected:failures.append('no actual HOLD window after required trim/minimum duration')
        for i,window in enumerate(selected):
            for axis in axes:
                for metric,actual in window['metrics'][axis].items():
                    if actual>getattr(args,f'max_torso_hold_{axis}_{metric}_deg'):
                        failures.append(f'window{i}:{axis}_{metric}')
        out['torso_hold']={'classification_contract':'runner_mode3_level0_initial_interball_final_v2',
            'orientation_frame_contract':'table_world_immutable_home_yaw_zero_v1','checked_axes':list(axes),
            'evidence_complete':True,'pass':not failures,'failures':failures,'windows':selected}
    if args.require_collision_ledger:
        complete=all(row['robot_collision_ledger_v1']==1 for row in plant)
        counts={k:sum(row['robot_'+k+'_contact_count']>0 for row in plant) for k in ('table','net','self')}
        failures=[k for k,v in counts.items() if v]
        if not complete:failures.append('missing instrumented collision contract')
        out['collision_ledger']={'contract':'robot_table_net_self_contact_v1','evidence_complete':complete,
            'contact_rows':counts,'failures':failures,'pass':not failures}
    if args.physical_ball_report:
        raw=args.physical_ball_report.read_bytes();physical=json.loads(raw)
        if not isinstance(physical.get('rows'),list):raise ValueError('invalid physical-ball report')
        out['physical_ball_binding']={'sha256':hashlib.sha256(raw).hexdigest(),'rows':len(physical['rows'])}
    nonzero=any(row['reach_level']!=0 for row in trace)
    if args.require_optional_reach or nonzero:
        raise ValueError('nonzero/required optional reach needs authoritative trajectory qualification; fixed-HOME provider does not certify it')
    out['required_contracts']=requested
    out['extended_checks_pass']=all(out[name]['pass'] for name in requested)
    return out
