"""81 target height/lateral/speed/angle cells, four distinct physical trials each.
All feasibility checks precede model tests. Save failures instead of changing gates.
"""
from pathlib import Path
import json,time,concurrent.futures,itertools
import numpy as np,yaml
from scipy.optimize import least_squares,differential_evolution
from flight_kernel import flight
W=Path(__file__).resolve().parent
SEEDS=json.loads((W/'expanded-horizontal-ledger.json').read_text())
def solve(spec):
 lateral,vb,hb,ab,variant=spec;lx=[2.,2.4,2.7,2.4][variant];dy=[[-.46,-.42,-.38,-.44],[-.28,-.26,-.24,-.22],[-.02,.06,.14,.22]][lateral][variant];cy=-.7625+dy;sp=[[2.15,2.35,2.2,2.4],[2.9,3.2,3.5,3.8],[4.15,4.4,4.65,4.85]][vb][variant];height=[[.87,.90,.93,.96],[1.04,1.08,1.12,1.16],[1.24,1.29,1.34,1.39]][hb][variant];angle=np.radians([-10.,0.,10.][ab]);unit=np.array([np.cos(angle),-np.sin(angle)]);spin=np.r_[-unit[1]*[-20.,0.,20.,0.][variant],unit[0]*[-20.,0.,20.,0.][variant],[-12.,0.,12.,0.][variant]];seed=np.array(SEEDS[(vb*3+hb)*3]['launch']);q0=np.array([seed[3]*unit[0],seed[3]*unit[1],seed[5],seed[2],cy-np.tan(angle)*(lx-.07)])
 def initial(q):return np.array([lx,q[4],q[3],q[0],q[1],q[2]])
 def residual(q):
  s,w,t,b,bounce,net=flight(initial(q),spin);a=np.arctan2(s[4],-s[3]);return np.r_[(s[1]-cy)*8,(s[2]+.76-height)*8,(np.linalg.norm(s[3:])-sp)*2,(a-angle)*6,max(0,s[3]+[.9,1.6,2.8][vb])*2,max(0,.19-net[1])*5,max(0,-1.49-net[0])*4,max(0,net[0]+.035)*4,max(0,bounce[0]-1.33)*2,max(0,.02-bounce[0])*2,abs(b-1)*3,abs(s[0]-.07)*3,.0005*(q-q0)]
 bounds=([-16,-8,-10,.03,-2.1],[-.3,8,10,1.3,.6]);opts=[]
 for seed0 in [q0,q0+np.array([-.3,0,-.8,.2,0]),q0+np.array([-.8,0,1,.3,0]),q0+np.array([0,0,-2,.6,0])]:
  fit=least_squares(residual,np.clip(seed0,np.array(bounds[0])+1e-6,np.array(bounds[1])-1e-6),bounds=bounds,diff_step=.008,max_nfev=140);opts.append((np.linalg.norm(residual(fit.x)),fit.x))
  if opts[-1][0]<.02:break
 error,q=min(opts,key=lambda x:x[0]);s,w,t,b,bounce,net=flight(initial(q),spin);ok=bool(abs(s[0]-.07)<1e-6 and abs(s[1]-cy)<.025 and abs(s[2]+.76-height)<.025 and abs(np.linalg.norm(s[3:])-sp)<.10 and abs(np.arctan2(s[4],-s[3])-angle)<np.radians(1.5) and b==1 and net[1]>.18 and -1.5<net[0]<-.025 and 0<bounce[0]<1.37)
 return {'requested_id':'core-'+ '-'.join(map(str,spec)),'section':'factorial81','lateral_bin':lateral,'speed_bin':vb,'height_bin':hb,'angle_bin':ab,'variant':variant,'origin_x':lx,'origin_y':float(q[4]),'target_relative_y':dy,'side':'FH' if dy<-.24 else 'BHfront' if abs(dy)<=.15 else 'BHother','target_speed':sp,'target_height':height,'target_angle_deg':float(np.degrees(angle)),'launch':initial(q).tolist(),'initial_spin':spin.tolist(),'predicted_contact':s.tolist(),'predicted_contact_spin':w.tolist(),'predicted_contact_speed':float(np.linalg.norm(s[3:])),'predicted_contact_world_height':float(s[2]+.76),'predicted_angle_deg':float(np.degrees(np.arctan2(s[4],-s[3]))),'flight_s':float(t),'bounce':bounce.tolist(),'net':net.tolist(),'closure_error':float(error),'feasible':ok}
if __name__=='__main__':
 specs=list(itertools.product(range(3),range(3),range(3),range(3),range(4)));requested=[];start=time.time()
 with concurrent.futures.ProcessPoolExecutor(max_workers=4) as pool:
  for i,row in enumerate(pool.map(solve,specs)):
   requested.append(row)
   if i%27==26:print(i+1,'accepted',sum(r['feasible'] for r in requested),'seconds',round(time.time()-start,1),flush=True)
 (W/'super-factorial-requested.json').write_text(json.dumps(requested,indent=2));print('DONE',len(requested),sum(r['feasible'] for r in requested),flush=True)
