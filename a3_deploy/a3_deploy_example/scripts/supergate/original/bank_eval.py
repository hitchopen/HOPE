"""Frozen stratified V5 conditional-target closed-loop probe, no ball/Planner/HAL.
Same measured native WAIT state for all actors, 0807 floating-base plant.
Incoming speed stratifies targets; this probe does not simulate incoming flight.
"""
from pathlib import Path
import sys,json,hashlib,time
import numpy as np,onnxruntime as ort,mujoco,yaml
from scipy.spatial.transform import Rotation
W=Path(__file__).resolve().parent;R=Path('/home/user/Desktop/workspace/nohope/agi/a3_deploy_example/scripts');sys.path.insert(0,str(R))
import validate_spin001_serve_mujoco as helper
from runner_sim_profile import apply_hope_contact_profile
B=W/'training-v5/assets/stationary_reach_v5_coverage_20261104';XML=W/'model-0807/a3_pingpong.xml'
C=yaml.safe_load((W/'download/model_14200_policy/params/deploy.yaml').read_text());names=C['actions']['JointPositionAction']['joint_names'];default=np.array(C['default_joint_pos']);scale=np.array(C['actions']['JointPositionAction']['scale']);limits=np.array(C['actions']['JointPositionAction']['clip']);head=np.array(['head_' in n for n in names]);opt=ort.SessionOptions();opt.intra_op_num_threads=1;opt.inter_op_num_threads=1
model,_=helper._compile_model(XML);apply_hope_contact_profile(model)
qa=np.array([model.jnt_qposadr[model.joint(n).id] for n in names]);va=np.array([model.jnt_dofadr[model.joint(n).id] for n in names]);ac=np.array([model.actuator(n+'_motor').id for n in names]);pelvis=model.body('pelvis_link').id;racket=model.site('right_racket').id

def fixtures():
 out={};used=set()
 for split,count in [('dev',2),('test',4)]:
  z=np.load(B/(split+'.npz'));speed=np.linalg.norm(z['incoming_velocity'],axis=1);h=z['target_position_offset'][:,2];sb=np.digitize(speed,[2.5,4]);hb=np.digitize(h,[1.,1.2]);front=abs(z['target_position_offset'][:,1])<=.15;rows=[];rng=np.random.default_rng(20261108+(split=='test'))
  for side in ['FH','BHfront','BHother']:
   for v in range(3):
    for height in range(3):
     mask=(z['clip']==(0 if side=='FH' else 1))&(sb==v)&(hb==height)&(h>=.8)&(h<=1.45)
     if side!='FH':mask&=(front if side=='BHfront' else ~front)
     selected=[]
     for idx in rng.permutation(np.flatnonzero(mask)):
      par=int(z['parent_id'][idx])
      if par in used:continue
      used.add(par);selected.append(int(idx))
      if len(selected)==count:break
     assert len(selected)==count,(split,side,v,height)
     for j,idx in enumerate(selected):rows.append({'split':split,'row':idx,'parent':int(z['parent_id'][idx]),'side':side,'speed_bin':v,'height_bin':height,'speed_mps':float(speed[idx]),'height_m':float(h[idx]),'lead_s':.4 if split=='dev' else [.3,.4,.5,.6][j],**{k:z[k][idx].tolist() for k in ['target_position_offset','target_velocity','target_normal','incoming_velocity','incoming_spin','intended_landing_xy']}})
  out[split]=rows
 (W/'bank-fixtures.json').write_text(json.dumps(out,indent=2));return out

def summary(rows):
 out={}
 for label,group in [('ALL',rows)]+[(s,[r for r in rows if r['side']==s]) for s in ['FH','BHfront','BHother']]+[(f'{v}/{h}',[r for r in rows if r['speed_bin']==v and r['height_bin']==h]) for v in range(3) for h in range(3)]:
  if not group:continue
  p=np.array([r['p_cm'] for r in group]);v=np.array([r['v_mps'] for r in group]);n=np.array([r['n_deg'] for r in group]);out[label]={'n':len(group),'p_cm_median':float(np.median(p)),'p_cm_p90':float(np.quantile(p,.9)),'v_mps_median':float(np.median(v)),'n_deg_median':float(np.median(n)),'falls':sum(r['fall'] for r in group),'p_le_5cm':int(sum(p<=5)),'pvn_within_5cm_0p5mps_10deg':int(sum((p<=5)&(v<=.5)&(n<=10))),'objective':float(np.mean(vector(group)**2))}
 return out

def vector(rows):return np.concatenate([np.r_[np.array(r['delta_p'])/.025,np.array(r['delta_v'])/.25,np.array(r['delta_n'])/np.sin(np.radians(5))] for r in rows])

def run(policy,split='dev',quick=False):
 fixture=json.loads((W/'bank-fixtures.json').read_text())[split]
 if quick:fixture=fixture[::2] if split=='dev' else fixture[:3]
 state=np.load(W/'serve-14200-0807/ready_reference.npz');sdk=helper.JOINTS if hasattr(helper,'JOINTS') else None
 # Capture stores SDK ordering; address lookup converts to actor ordering.
 command=state['command'][-1];ix={int(q):i for i,q in enumerate(state['qa'])};order=[ix[int(q)] for q in qa];kp=command[3,order];kd=command[4,order];ready_des=command[0,order];previous0=(ready_des-default)/scale;previous0[head]=0
 sess=ort.InferenceSession(str(policy/'exported/policy.onnx'),opt,providers=['CPUExecutionProvider']);rows=[]
 for f in fixture:
  d=mujoco.MjData(model);d.qpos[:]=state['qpos'][-1];d.qvel[:]=state['qvel'][-1]
  for j in range(model.njnt):
   if 'gate3_ball_free_joint' in model.joint(j).name:d.qpos[model.jnt_qposadr[j]:model.jnt_qposadr[j]+3]=[100+j,0,-10]
  mujoco.mj_forward(model,d);home=d.qpos[:2].copy();target=np.array(f['target_position_offset']);target[:2]+=home;vt=np.array(f['target_velocity']);nt=np.array(f['target_normal']);nt/=np.linalg.norm(nt);lead=f['lead_s'];previous=previous0.copy();minz=float(d.qpos[2]);maxtilt=0.;maxstep=0.;maxratio=0.;last=ready_des.copy();hit=None
  for tick in range(int(round((lead+.6)/.02))+1):
   t=tick*.02;rot=Rotation.from_quat(d.xquat[pelvis][[1,2,3,0]]);fw=rot.apply([1,0,0])[:2];fw/=np.linalg.norm(fw);gyro=np.zeros(6);mujoco.mj_objectVelocity(model,d,mujoco.mjtObj.mjOBJ_BODY,pelvis,gyro,1)
   x=np.r_[gyro[:3],d.qpos[qa]-default,d.qvel[va],previous,rot.inv().apply([0,0,-1]),fw,home-d.xpos[pelvis,:2],target-d.xpos[pelvis],vt,lead-t]
   raw=sess.run(['actions'],{'obs':x[None].astype(np.float32),'time_step':np.array([[tick]],np.float32)})[0][0];des=np.clip(default+raw*scale,limits[:,0],limits[:,1]);des[head]=ready_des[head];previous=(des-default)/scale;previous[head]=0;maxstep=max(maxstep,float(abs(des-last).max()));last=des.copy()
   if abs(t-lead)<1e-8:
    normal=d.site_xmat[racket].reshape(3,3)[:,1].copy();normal*=1 if normal@nt>=0 else -1
    # Both policies scored against intended per-face contact point, plus legacy site distance.
    off=d.site_xmat[racket].reshape(3,3)@np.array([0,-.013208 if f['side']!='FH' else 0,0]);pos=d.site_xpos[racket]+off;jac=np.zeros((3,model.nv));mujoco.mj_jac(model,d,jac,None,pos,int(model.site_bodyid[racket]));vel=jac@d.qvel
    hit={'delta_p':(pos-target).tolist(),'delta_v':(vel-vt).tolist(),'delta_n':(normal-nt).tolist(),'p_cm':float(np.linalg.norm(pos-target)*100),'v_mps':float(np.linalg.norm(vel-vt)),'n_deg':float(np.degrees(np.arccos(np.clip(normal@nt,-1,1)))),'legacy_site_p_cm':float(np.linalg.norm(d.site_xpos[racket]-target)*100)}
   for sub in range(20):
    tau=kp*(des-d.qpos[qa])-kd*d.qvel[va];maxratio=max(maxratio,float((abs(tau)/model.actuator_ctrlrange[ac,1]).max()));d.ctrl[ac]=np.clip(tau,model.actuator_ctrlrange[ac,0],model.actuator_ctrlrange[ac,1]);mujoco.mj_step(model,d)
   mujoco.mj_forward(model,d);minz=min(minz,float(d.qpos[2]));tilt=np.arccos(np.clip(d.xmat[pelvis].reshape(3,3)[2,2],-1,1));maxtilt=max(maxtilt,float(tilt))
  assert hit is not None
  rows.append({**f,**hit,'min_z':minz,'max_tilt_deg':float(np.degrees(maxtilt)),'fall':bool(minz<.75 or maxtilt>.75),'max_command_step_rad':maxstep,'max_requested_torque_ratio':maxratio})
 return {'scope':'Offline conditional-target closed-loop MuJoCo; no ball flight, contact, Planner or native Runner. Incoming speed only stratifies correlated bank targets. Not a Gate3 pass.','model_sha256':hashlib.sha256(XML.read_bytes()).hexdigest(),'policy_sha256':hashlib.sha256((policy/'exported/policy.onnx').read_bytes()).hexdigest(),'fixture_sha256':hashlib.sha256((W/'bank-fixtures.json').read_bytes()).hexdigest(),'rows':rows,'summary':summary(rows)}

if __name__=='__main__':
 if not (W/'bank-fixtures.json').exists():fixtures()
 if len(sys.argv)<2:raise SystemExit()
 from build_candidates import SOURCES
 for name in sys.argv[1].split(','):
  st=time.time();split=sys.argv[2] if len(sys.argv)>2 else 'dev';r=run(SOURCES.get(name,W/'models'/name),split);(W/f'bank-{split}-{name}.json').write_text(json.dumps(r,indent=2));print(name,split,r['summary']['ALL'],'seconds',time.time()-st,flush=True)
