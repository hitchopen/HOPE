"""Additional geometric coverage of the recorded slow tail; no policy scores."""
from pathlib import Path
import json,hashlib
import numpy as np,yaml
from scipy.optimize import least_squares
from flight_kernel import flight
W=Path(__file__).resolve().parent;D=W.parent/'outputs/super-gate3-v1';O=W.parent/'outputs/super-gate3-slow-tail';O.mkdir(exist_ok=False)
real=json.loads((W/'real-plan-ledger.json').read_text());real=[r for r in real if .8<=r['height']<=1.45 and -.68<=r['relative_y']<=.28 and r['incoming_speed']<1.8014331306667368]
seeds=json.loads((D/'samples.json').read_text());attempts=[];selected=[]
def solve(r,index,jitter=False):
 rng=np.random.default_rng(2026110803+index);dy=r['relative_y'];speed=r['incoming_speed'];height=r['height'];angle=r['incoming_angle_deg']
 if jitter:dy+=rng.uniform(-.012,.012);height+=rng.uniform(-.015,.015);speed+=rng.uniform(-.04,.04);angle+=rng.uniform(-1,1)
 cy=-.7625+dy;target=np.array([cy,height,speed,angle]);ranked=sorted(seeds,key=lambda s:((s['target_relative_y']-dy)*2)**2+((s['predicted_contact_world_height']-height)*3)**2+(s['predicted_contact_speed']-speed)**2+((s['predicted_angle_deg']-angle)/30)**2)[:8]
 best=None
 for si,seed in enumerate(ranked):
  spin=np.array(seed['initial_spin']);lx=float(seed['origin_x']);q0=np.array(seed['launch'])[ [3,4,5,2,1] ];q0[4]=cy-np.tan(np.radians(angle))*(lx-.07)
  def initial(q):return np.array([lx,q[4],q[3],q[0],q[1],q[2]])
  def residual(q):
   s,w,t,b,bounce,net=flight(initial(q),spin);a=np.degrees(np.arctan2(s[4],-s[3]))
   return np.r_[(s[1]-cy)*8,(s[2]+.76-height)*8,(np.linalg.norm(s[3:])-speed)*3,(a-angle)*.15,max(0,s[3]+.5)*2,max(0,.185-net[1])*6,max(0,-1.5-net[0])*4,max(0,net[0]+.025)*4,max(0,bounce[0]-1.34)*2,max(0,.01-bounce[0])*2,abs(b-1)*3,abs(s[0]-.07)*3,.0001*(q-q0)]
  bounds=([-16,-8,-10,.03,-2.3],[-.3,8,10,1.4,.8]);fit=least_squares(residual,np.clip(q0,np.array(bounds[0])+1e-6,np.array(bounds[1])-1e-6),bounds=bounds,diff_step=.008,max_nfev=140)
  score=np.linalg.norm(residual(fit.x));s,w,t,b,bounce,net=flight(initial(fit.x),spin);a=float(np.degrees(np.arctan2(s[4],-s[3])))
  checks={'plane':abs(s[0]-.07)<1e-6,'y':abs(s[1]-cy)<.025,'height':abs(s[2]+.76-height)<.025,'speed':abs(np.linalg.norm(s[3:])-speed)<.06,'angle':abs(a-angle)<1.5,'one_bounce':b==1,'net_clearance':net[1]>.18,'net_width':-1.5<net[0]<-.025,'bounce_half':0<bounce[0]<1.37}
  row={'sample_id':f'SLOW-{index:03d}','section':'slow_tail','source_session':r['session_id'],'source_flight_id':r['flight_id'],'source_is_planner_prediction':True,'jittered':jitter,'seed_index':si,'target_relative_y':dy,'target_speed':speed,'target_height':height,'target_angle_deg':angle,'origin_x':lx,'origin_y':float(fit.x[4]),'launch':initial(fit.x).tolist(),'initial_spin':spin.tolist(),'predicted_contact':s.tolist(),'predicted_contact_speed':float(np.linalg.norm(s[3:])),'predicted_contact_world_height':float(s[2]+.76),'predicted_angle_deg':a,'flight_s':float(t),'bounce':bounce.tolist(),'net':net.tolist(),'closure_error':score,'feasibility_checks':{k:bool(v) for k,v in checks.items()},'feasible':bool(all(checks.values())),'aim':[2.055,-.7625],'cell_id':None,'lateral_bin':int(np.digitize(dy,[-.32,-.15])),'speed_bin':0,'height_bin':int(np.digitize(height,[1,1.2])),'angle_bin':int(np.digitize(angle,[-5,5])),'side':'FH' if dy<-.24 else 'BHfront' if abs(dy)<=.15 else 'BHother'}
  if best is None or score<best['closure_error']:best=row
  if row['feasible']:return row
 return best
for index in range(150):
 r=solve(real[index%len(real)],index,index>=len(real));attempts.append(r)
 if r['feasible']:selected.append(r)
 print(index,'feasible',r['feasible'],'selected',len(selected),flush=True)
 (O/'attempts.json').write_text(json.dumps(attempts,indent=2))
 if len(selected)==54 and index>=len(real)-1:break
assert len(selected)==54
rng=np.random.default_rng(2026110803);rng.shuffle(selected);base=yaml.safe_load((D/'shard-01.yaml').read_text());allrows=[]
for i in range(6):
 rows=selected[i*9:(i+1)*9]
 for k,r in enumerate(rows):r.update(shard_id=i+1,shot_id=k+1)
 scene=dict(base);scene.update(description='Slow-tail supplement, fitted to recorded Planner predictions; synthetic launches and spin, not literal field replay.',balls=9,clean_serves=[r['launch'] for r in rows[:6]],rapid_cycle=[r['launch'] for r in rows[6:]],spin_rad_s_list=[r['initial_spin'] for r in rows]);scene['timing']=dict(base['timing']);scene['timing'].update(flight_s_list=[max(1.5,r['flight_s']+.95) for r in rows],pause_s_list=[.8]*6+[0.]*3);scene['timing']['flight_s_list'][-1]=3.
 (O/f'shard-{i+1:02d}.yaml').write_text(yaml.safe_dump(scene));(O/f'shard-{i+1:02d}-ledger.json').write_text(json.dumps(rows,indent=2));allrows.extend(rows)
(O/'samples.json').write_text(json.dumps(allrows,indent=2));(O/'design.json').write_text(json.dumps({'real_requests':len(real),'real_requests_feasible':sum(r['feasible'] for r in attempts[:len(real)]),'total':len(allrows),'min_speed':min(r['predicted_contact_speed'] for r in allrows),'max_speed':max(r['predicted_contact_speed'] for r in allrows),'policy_outcome_used':False,'scope':'Synthetic feasible near-matches to historical slow predicted contacts. Initial spin is a geometric construction variable, not measured field spin. Position/speed/angle matching tolerances retained per attempt; not exact field replay.'},indent=2));(O/'SHA256.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in O.iterdir() if p.is_file()},indent=2))
