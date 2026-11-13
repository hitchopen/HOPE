"""Freeze 12 distinct feasible physical samples in each of 81 coverage cells.
Feasibility is purely geometric/dynamic and precedes policy tests. All attempted
samples and failures are preserved. This is stratified coverage, not i.i.d. play.
"""
from pathlib import Path
import json,time,concurrent.futures,itertools
import numpy as np
from scipy.optimize import least_squares
from flight_kernel import flight
W=Path(__file__).resolve().parent.parent
SEEDS=json.loads((W/'expanded-horizontal-ledger.json').read_text())
V1=json.loads((W/'super-gate3-ledger.json').read_text())['selected']

def solve(spec):
 lateral,vb,hb,ab,variant=spec
 cell=((lateral*3+vb)*3+hb)*3+ab
 rng=np.random.default_rng(20261109007+cell*10000+variant)
 # Four axes define the cell. Twelve within-cell trials use stratified quantiles
 # with independently permuted dimensions and fixed seeds; retries are recorded.
 u=rng.random(10)
 if variant<12:
  lrng=np.random.default_rng(20261108+cell)
  u=np.array([(lrng.permutation(12)[variant]+u[j])/12 for j in range(10)])
 dy=np.interp(u[0],[0,1],[[-.84,-.70],[.30,.44],[.46,.59]][lateral])
 sp=np.interp(u[1],[0,1],[[1.8,2.45],[2.6,3.9],[4.1,5.1]][vb])
 height=np.interp(u[2],[0,1],[[.82,.98],[1.02,1.18],[1.22,1.43]][hb])
 angle=np.radians(np.interp(u[3],[0,1],[[-20,-6],[-4,4],[6,20]][ab]))
 lx=np.interp(u[4],[0,1],[2.,2.8]);cy=-.7625+dy
 unit=np.array([np.cos(angle),-np.sin(angle)])
 spin=np.r_[-unit[1]*np.interp(u[5],[0,1],[-25,25]),unit[0]*np.interp(u[5],[0,1],[-25,25]),np.interp(u[6],[0,1],[-15,15])]
 seed=np.array(SEEDS[(vb*3+hb)*3]['launch'])
 q0=np.array([seed[3]*unit[0],seed[3]*unit[1],seed[5],seed[2],cy-np.tan(angle)*(lx-.07)])
 def initial(q):return np.array([lx,q[4],q[3],q[0],q[1],q[2]])
 def residual(q):
  s,w,t,b,bounce,net=flight(initial(q),spin);a=np.arctan2(s[4],-s[3])
  return np.r_[(s[1]-cy)*8,(s[2]+.76-height)*8,(np.linalg.norm(s[3:])-sp)*2,(a-angle)*6,max(0,s[3]+[.7,1.5,2.7][vb])*2,max(0,.19-net[1])*5,max(0,-1.49-net[0])*4,max(0,net[0]+.035)*4,max(0,bounce[0]-1.33)*2,max(0,.02-bounce[0])*2,abs(b-1)*3,abs(s[0]-.07)*3,.0005*(q-q0)]
 bounds=([-16,-8,-10,.03,-2.3],[-.3,8,10,1.35,.8]);opts=[]
 # Seed the opposite vertical branch as well; do not require all balls to rise.
 for seed0 in [q0,q0+np.array([-.3,0,-.8,.2,0]),q0+np.array([-.8,0,1,.3,0]),q0+np.array([0,0,-2,.6,0])]:
  fit=least_squares(residual,np.clip(seed0,np.array(bounds[0])+1e-6,np.array(bounds[1])-1e-6),bounds=bounds,diff_step=.008,max_nfev=100)
  opts.append((np.linalg.norm(residual(fit.x)),fit.x))
  if opts[-1][0]<.025:break
 error,q=min(opts,key=lambda x:x[0]);s,w,t,b,bounce,net=flight(initial(q),spin)
 actual_angle=float(np.degrees(np.arctan2(s[4],-s[3])))
 actual_angle_bin=int(np.digitize(actual_angle,[-5.,5.]));actual_speed_bin=int(np.digitize(np.linalg.norm(s[3:]),[2.5,4.]));actual_height_bin=int(np.digitize(s[2]+.76,[1.,1.2]));actual_lateral_bin=int(np.digitize(s[1]+.7625,[-.68,.45]))
 checks={'plane':abs(s[0]-.07)<1e-6,'y':abs(s[1]-cy)<.025,'height':abs(s[2]+.76-height)<.025,'speed':abs(np.linalg.norm(s[3:])-sp)<.10,'angle':abs(actual_angle-np.degrees(angle))<1.5,'one_bounce':b==1,'net_clearance':net[1]>.18,'net_width':-1.5<net[0]<-.025,'bounce_half':0<bounce[0]<1.37,'exact_cell':(actual_lateral_bin,actual_speed_bin,actual_height_bin,actual_angle_bin)==(lateral,vb,hb,ab)}
 return {'requested_id':'core-'+ '-'.join(map(str,spec)),'section':'factorial81','cell_id':cell,'lateral_bin':lateral,'speed_bin':vb,'height_bin':hb,'angle_bin':ab,'variant':variant,'origin_x':float(lx),'origin_y':float(q[4]),'target_relative_y':float(dy),'side':'FH' if dy<-.24 else 'BHfront' if abs(dy)<=.15 else 'BHother','target_speed':float(sp),'target_height':float(height),'target_angle_deg':float(np.degrees(angle)),'launch':initial(q).tolist(),'initial_spin':spin.tolist(),'predicted_contact':s.tolist(),'predicted_contact_spin':w.tolist(),'predicted_contact_speed':float(np.linalg.norm(s[3:])),'predicted_contact_world_height':float(s[2]+.76),'predicted_angle_deg':actual_angle,'flight_s':float(t),'bounce':bounce.tolist(),'net':net.tolist(),'closure_error':float(error),'feasibility_checks':{k:bool(v) for k,v in checks.items()},'feasible':bool(all(checks.values()))}

