"""270 requested strata spanning source x/y, side/reach, speed/height and spin.
Feasibility and closure are computed before native tests; no outcome-based pruning.
"""
from pathlib import Path
import json,time
import numpy as np,yaml
from scipy.optimize import least_squares
from flight_kernel import flight
W=Path(__file__).resolve().parent;rng=np.random.default_rng(20261108);seeds=json.loads((W/'expanded-horizontal-ledger.json').read_text());requested=[];accepted=[];st=time.time()
for origin_index,ly in enumerate([-1.6,-1.2,-.7625,-.3,.3]):
 for target_index,dy in enumerate([-.62,-.44,-.27,-.18,.02,.22]):
  for vb,sp in enumerate([2.3,3.25,4.5]):
   for hb,height in enumerate([.9,1.1,1.3]):
    idx=len(requested);lx=[2.,2.4,2.7][(origin_index+target_index+vb+hb)%3];cy=-.7625+dy;cx=.07;vec=np.array([lx-cx,ly-cy]);unit=vec/np.linalg.norm(vec);spin_case=(origin_index+2*target_index+vb+hb)%3;spin=np.array([-unit[1],unit[0],0.])*[-20.,0.,20.][spin_case];spin[2]=[-12.,0.,12.][(origin_index+target_index+2*vb+hb)%3]
    seed=np.array(seeds[(vb*3+hb)*3]['launch']);q0=np.array([seed[3]*unit[0],seed[3]*unit[1],seed[5],seed[2]])
    def state(q):return np.array([lx,ly,q[3],q[0],q[1],q[2]])
    def residual(q):
     s,w,t,b,bounce,net=flight(state(q),spin);return np.r_[(s[1]-cy)*6,(s[2]+.76-height)*6,np.linalg.norm(s[3:])-sp,max(0,s[3]+[1.0,1.6,2.8][vb])*2,max(0,.18-net[1])*3,max(0,-1.50-net[0])*2,max(0,net[0]+.025)*2,max(0,bounce[0]-1.35)*2,abs(b-1)*2,abs(s[0]-cx)*2, .001*(q-q0)]
    opts=[]
    for seed0 in [q0,q0+np.array([-.3,0.,-.5,.2]),q0+np.array([-.3,0.,.7,.1])]:
     q=np.clip(seed0,[-15,-7,-9,.035],[-.5,7,9,1.8]);fit=least_squares(residual,q,bounds=([-16,-8,-10,.03],[-.3,8,10,1.9]),diff_step=.012,max_nfev=100);opts.append((np.linalg.norm(residual(fit.x)),fit.x))
     if opts[-1][0]<.04:break
    error,q=min(opts,key=lambda p:p[0]);s,w,t,b,bounce,net=flight(state(q),spin);ok=bool(abs(s[0]-cx)<1e-6 and abs(s[1]-cy)<.025 and abs(s[2]+.76-height)<.025 and abs(np.linalg.norm(s[3:])-sp)<.10 and b==1 and net[1]>.17 and -1.5<net[0]<-.025 and 0<bounce[0]<1.37)
    row={'requested_id':idx,'origin_x':lx,'origin_y':ly,'origin_index':origin_index,'target_relative_y':dy,'side':'FH' if dy<-.24 else 'BHfront' if abs(dy)<=.15 else 'BHother','speed_bin':vb,'height_bin':hb,'launch':state(q).tolist(),'initial_spin':spin.tolist(),'predicted_contact':s.tolist(),'predicted_contact_spin':w.tolist(),'predicted_contact_speed':float(np.linalg.norm(s[3:])),'predicted_contact_world_height':float(s[2]+.76),'predicted_angle_deg':float(np.degrees(np.arctan2(s[4],-s[3]))),'flight_s':float(t),'bounce':bounce.tolist(),'net':net.tolist(),'closure_error':float(error),'feasible':ok};requested.append(row)
    if ok:accepted.append(row)
    if idx%15==14:print('requested',idx+1,'accepted',len(accepted),'seconds',round(time.time()-st,1),flush=True);(W/'super-grid-progress.json').write_text(json.dumps(requested,indent=2))
# Retain every failed requested cell. Retry deficits in a later explicit revision; never count them as tested.
if len(accepted)%2:accepted=accepted[:-1]
order=rng.permutation(len(accepted));rows=[accepted[i] for i in order]
for i,r in enumerate(rows):r['shot_id']=i+1;r['aim']=[[1.85,-1.25],[1.85,-.275],[2.055,-.7625],[2.4,-1.25],[2.4,-.275],[2.4,-.7625]][i%6]
scene=yaml.safe_load((W/'branch-gate3/fixed-home.yaml').read_text());scene.update(description='Stratified source-origin/diagonal/speed/height/spin suite; 270 requested unique physics questions, retained feasible cells. Actual coverage audited from telemetry.',balls=len(rows),clean_serves=[r['launch'] for r in rows[:6]],rapid_cycle=[r['launch'] for r in rows[6:]],spin_rad_s_list=[r['initial_spin'] for r in rows]);scene['timing'].update(flight_s_list=[max(1.15,r['flight_s']+.55) for r in rows],pause_s_list=[.8]*6+[0.]*(len(rows)-6));scene['timing']['flight_s_list'][-1]=3.
(W/'super-gate3.yaml').write_text(yaml.safe_dump(scene));(W/'super-gate3-aim.csv').write_text(''.join(f"{r['aim'][0]},{r['aim'][1]}\n" for r in rows));(W/'super-gate3-ledger.json').write_text(json.dumps({'requested':requested,'selected':rows,'excluded_odd_tail':len(accepted)!=sum(r['feasible'] for r in requested),'design_seed':20261108,'scope':'Independent physical combinations; synthetic, not measured throws. Origin range informed by historical observed x=2.4 crossings. No test outcome used for fixture selection.'},indent=2));print('DONE',len(rows),len(requested),flush=True)
