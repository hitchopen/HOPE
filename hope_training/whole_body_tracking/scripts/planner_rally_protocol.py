"""Pure protocol, schema-2 adapter and physical scoring for GRU/planner evaluation."""
import math
import numpy as np

OFFSET=np.array([.5,.7625,.76])

def make_protocol(seed=0):
    # Reuse clean serve geometry/velocities from fixed-home-100.yaml, with
    # stationary-task table translation; do not use its prerecorded targets.
    rng=np.random.default_rng(seed)
    sides=np.array([0]*50+[1]*50);rng.shuffle(sides)
    lanes={0:[-1.0225,-1.0575,-1.0925],1:[-.7125,-.6575,-.6125]}
    serves=[]
    for i,side in enumerate(sides):
        table=np.array([2.4,float(rng.choice(lanes[int(side)])),.49])
        serves.append({'id':i,'side':int(side),'side_name':'forehand' if side==0 else 'backhand',
            'launch_at_s':2.+5.*i,'position_w':(table+OFFSET).tolist(),
            'velocity_w':[-3.,0.,2.2],'spin_w':[0.,0.,0.]})
    return {'schema':'hope_gru_real_planner_100_v1','seed':seed,'serves':serves,
        'warmup_s':2.,'slot_s':5.,'sim_timeout_s':502.,'wall_timeout_s':900.,
        'min_command_lead_s':.04,'max_realtime_lag_s':.05,'sensor_hz':200,
        'ready_proxy':{'drift_m':.1,'speed_mps':.2,'continuous_dwell_s':.3},
        'frame_translation_world_minus_planner':OFFSET.tolist(),
        'supported_envelope':'Existing fixed-home-100 clean serves; speed 3.72 m/s, zero spin, table +x incoming to -x. Venue fit: speed 1-7 m/s, spin <=15 rev/s.',
        'episode_timeout_override':'Disable evaluator 12s timeout ONLY in this evaluation driver. Robot/GRU reset only on fall, numerical warning or nonfinite state.',
        'resolution_rule':'First actual return table contact with first landing, or floor/out-of-play. Otherwise interrupt at 5s; never clear an unresolved ball as success.'}


def decode_command(values,spawn_ns,now_ns,min_lead=.04):
    a=np.asarray(values,dtype=float)
    if a.shape!=(19,) or not np.isfinite(a).all():return None,'malformed'
    if a[0]!=2 or a[1]!=1 or a[2] not in [-1,1]:return None,'planner_invalid'
    producer=a[12]+a[13]*1e-9;now=now_ns*1e-9
    if producer<spawn_ns*1e-9-1e-6 or producer>now+.05:return None,'wrong_flight_or_clock'
    if abs((a[10]-producer)-a[9])>.003:return None,'timestamp_contract'
    if a[10]-now<min_lead:return None,'late_command'
    if np.any(a[14:17]<1):return None,'missing_identity'
    # Schema2 publisher already adds policy_z_offset=.76. Translate XY only.
    return {'position_w':(a[3:6]+[.5,.7625,0.]).tolist(),
        'velocity_w':a[6:9].tolist(),'deadline_unix_s':float(a[10]),
        'producer_unix_s':float(producer),'lead_at_receipt_s':float(a[10]-now),
        'planned_side':'forehand' if a[2]==1 else 'backhand',
        'identity':a[14:17].astype(int).tolist()},'accepted'


def physical_outcome(events):
    racket=[e for e in events if e['kind']=='racket_contact']
    hit_time=racket[0]['episode_t'] if racket else math.inf
    land=[e for e in events if e['kind']=='first_return_landing' and e['episode_t']>=hit_time]
    table=[e for e in events if e['kind']=='table_contact' and e['episode_t']>=hit_time]
    net=[e for e in events if e['kind']=='net_contact']
    floor=any(e['kind']=='floor_contact' for e in events)
    landing=land[0] if land else (table[0] if table else None)
    stop=landing['episode_t'] if landing else math.inf
    crossing=any(e['kind']=='net_crossing_outgoing' and e.get('after_racket') and e.get('clear') and hit_time<=e['episode_t']<=stop for e in events)
    actual_table=bool(table and landing and abs(table[0]['episode_t']-stop)<.08)
    p=landing['position'] if landing else [0,0,0]
    on_opp=1.87<p[0]<=3.24 and abs(p[1])<=.7625
    legal=bool(len(racket)==1 and crossing and actual_table and on_opp and not any(e['episode_t']<=stop for e in net))
    resolved=bool((landing and actual_table) or floor or any(e['kind']=='out_of_play' for e in events))
    return {'contact':bool(racket),'racket_contacts':len(racket),'net_contact':bool(net),
        'landing_position':p if landing else None,'legal_return':legal,
        'net_contact_events':len(net),'resolved':resolved,'miss':bool(resolved and not racket),
        'failed_return':bool(resolved and racket and not legal)}


def valid_delivery(events):
    incoming=any(e['kind']=='net_crossing_incoming' and e.get('clear') for e in events)
    hit_time=min((e['episode_t'] for e in events if e['kind']=='racket_contact'),default=math.inf)
    own_bounce=any(e['kind']=='table_contact' and e['episode_t']<hit_time and .5<=e['position'][0]<1.87 for e in events)
    arrival=any(e['kind'] in ('hit_plane_incoming','racket_contact') for e in events)
    inbound_net=any(e['kind']=='net_contact' and e['episode_t']<hit_time for e in events)
    return bool(incoming and own_bounce and arrival and not inbound_net)


def finalize_row(row,events,reason,ready_dwell):
    row.update(physical_outcome(events));row['events']=events
    row['valid_delivery']=valid_delivery(events)
    if 'resolution_snapshot' in row:
        for key in ['legal_return','resolved','miss','failed_return','landing_position']:
            row[key]=row['resolution_snapshot'][key]
    row['slot_interrupted']=reason!='resolved'
    row['interrupted']=not row['resolved']
    row['end_to_end_success']=bool(row['valid_delivery'] and row['planner_on_time'] and row['legal_return'] and not row['interrupted'] and not row['fall_reset'] and not row['numerical_reset'])
    row['reason']=reason
    row['ready_proxy']=bool(ready_dwell>=.3 and not row['fall_reset'] and not row['numerical_reset'])
    return row


def summarize(rows):
    result={}
    for name,subset in [('all',rows),('forehand',[x for x in rows if x['side']==0]),('backhand',[x for x in rows if x['side']==1])]:
        n=len(subset);valid=sum(x['valid_delivery'] for x in subset);contact=sum(x['contact'] for x in subset)
        success=sum(x['end_to_end_success'] for x in subset);legal=sum(x['legal_return'] and not x['interrupted'] for x in subset)
        def rate(a,b):return {'numerator':a,'denominator':b,'rate':a/b if b else None}
        result[name]={'scheduled':n,'valid_delivery':valid,'on_time_planner':sum(x['planner_on_time'] for x in subset),
            'contacts':contact,'racket_contact_events':sum(x['racket_contacts'] for x in subset),'net_contact_events':sum(x.get('net_contact_events',0) for x in subset),'serve_delivery_failures':n-valid,'no_on_time_command_total':n-sum(x['planner_on_time'] for x in subset),'planner_failures_after_valid_delivery':sum(x['valid_delivery'] and not x['planner_on_time'] for x in subset),'misses':sum(x['miss'] for x in subset),'net_contacts':sum(x['net_contact'] for x in subset),
            'legal_returns':legal,'failed_returns':sum(x['failed_return'] for x in subset),
            'fall_resets':sum(x['fall_reset'] for x in subset),'numerical_resets':sum(x['numerical_reset'] for x in subset),
            'ready_proxy_pass':sum(x['ready_proxy'] for x in subset),'unresolved_interrupted':sum(x['interrupted'] for x in subset),
            'end_to_end':rate(success,n),'valid_delivery_conditioned':rate(success,valid),
            'contact_conditioned_physical_legal':rate(legal,contact)}
    best=streak=0;segments=[];start=0
    for i,row in enumerate(rows):
        streak=streak+1 if row['end_to_end_success'] else 0;best=max(best,streak)
        if row['fall_reset'] or row['numerical_reset']:
            segments.append({'first_serve':start,'last_serve':i-1,'count':i-start});start=i+1
    started=sum(x.get('started',True) for x in rows)
    segments.append({'first_serve':start,'last_serve':started-1,'count':max(0,started-start)})
    result['longest_success_streak']=best;result['reset_free_segments']=segments
    result['longest_reset_free_segment']=max((s['count'] for s in segments),default=0)
    return result
