from pathlib import Path
import json, math, hashlib, shutil
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
W=Path(__file__).resolve().parent; O=W.parent/'outputs/super-gate3-evaluation'; O.mkdir(exist_ok=True)
D=W.parent/'outputs/super-gate3-v1'; samples=pd.DataFrame(json.loads((D/'samples.json').read_text()))
RUNS=[('13800','super-internal-native',0),('14200','super-internal-native',6),('fusion','super-internal-fusions',0),('fusion_weighted','super-internal-fusions',6),('14200_weighted','super-internal-edited',0)]
frames=[]
def plain(v):
 if isinstance(v,dict):return {str(k):plain(x) for k,x in v.items()}
 if isinstance(v,(list,tuple)):return [plain(x) for x in v]
 if isinstance(v,np.integer):return int(v)
 if isinstance(v,(float,np.floating)):return float(v) if math.isfinite(v) else None
 return v
def save(name,v):(O/name).write_text(json.dumps(plain(v),ensure_ascii=False,indent=2))
for label,folder,offset in RUNS:
 if not (W/folder/'run.json').exists():continue
 run=json.loads((W/folder/'run.json').read_text()); assert run['return_code']==0
 assert run['dataset_manifest_sha256']==hashlib.sha256((D/'SHA256.json').read_bytes()).hexdigest()
 parts=[]
 for shard in range(1,7):
  r=pd.read_csv(W/folder/f'env-{offset+shard-1}-results.csv');assert len(r)==200
  r['shard_id']=shard;r=r.rename(columns={'shot':'shot_id','side':'actual_side'})
  r=r.merge(samples,on=['shard_id','shot_id'],validate='one_to_one');parts.append(r)
 r=pd.concat(parts,ignore_index=True);r['model']=label;r['contact']=r.contacts>0
 r['landing_error_m']=np.hypot(r.landing_x-2.055,r.landing_y+.7625)
 r['fall_indicator']=(r.min_z<.7)|(r.max_tilt>np.pi/4)
 r['planned_speed']=np.linalg.norm(r[['strike_vx','strike_vy','strike_vz']],axis=1)
 r['planned_height']=r.strike_z+.76;r['planned_angle']=np.degrees(np.arctan2(r.strike_vy,-r.strike_vx))
 r['planned_lateral_bin']=np.digitize(r.strike_y+.7625,[-.32,-.15]);r['planned_speed_bin']=np.digitize(r.planned_speed,[2.5,4]);r['planned_height_bin']=np.digitize(r.planned_height,[1.,1.2]);r['planned_angle_bin']=np.digitize(r.planned_angle,[-5.,5.])
 r['planned_cell_agrees']=np.logical_and.reduce([r['planned_'+a+'_bin']==r[a+'_bin'] for a in ['lateral','speed','height','angle']])
 r['planned_side_name']=np.where(r.actual_side==1,'FH','BH')
 frames.append(r)
df=pd.concat(frames,ignore_index=True)
df.drop(columns=[c for c in df.columns if any(isinstance(x,(dict,list)) for x in df[c])]).to_csv(O/'all-trials.csv',index=False)
def stats(r):
 landing=r.loc[r.legal==1,'landing_error_m']
 return dict(n=len(r),contacts=int(r.contact.sum()),legal=int(r.legal.sum()),legal_pct=100*float(r.legal.mean()),valid_plans=int((r.valid_plans>0).sum()),complete=int(r.complete.sum()),fall_indicators=int(r.fall_indicator.sum()),min_pelvis_z=float(r.min_z.min()),max_tilt_deg=float(np.degrees(r.max_tilt.max())),max_requested_torque_ratio=float(r.max_torque_ratio.max()),trials_requesting_torque_clip=int((r.max_torque_ratio>1).sum()),min_lead_s=float(r.lead.min()),median_lead_s=float(r.lead.median()),landing_error_legal_median_m=float(landing.median()),landing_error_legal_p90_m=float(landing.quantile(.9)),landing_error_legal_p95_m=float(landing.quantile(.95)),multiple_contact_trials=int((r.contacts>1).sum()),uncompleted_ids=r.loc[r.complete==0,'sample_id'].tolist())
summary={};groups=[];cells=[]
for model,r in df.groupby('model',sort=False):
 summary[model]=stats(r);summary[model]['core']=stats(r[r.section=='factorial81']);summary[model]['boundary']=stats(r[r.section=='origin_boundary']);summary[model]['planned_cell_agreement_core']=int(r.loc[r.section=='factorial81','planned_cell_agrees'].sum())
 for axis in ['section','side','planned_side_name','speed_bin','height_bin','angle_bin','lateral_bin','shard_id']:
  for key,g in r.groupby(axis):groups.append({'model':model,'axis':axis,'group':key,**stats(g)})
 for cell,g in r[r.section=='factorial81'].groupby('cell_id'):
  assert len(g)==12
  cells.append({'model':model,'cell_id':int(cell),**{a:int(g.iloc[0][a]) for a in ['lateral_bin','speed_bin','height_bin','angle_bin']},**stats(g)})
save('summary.json',summary);save('groups.json',groups);save('cells.json',cells)
pd.DataFrame(cells).to_csv(O/'81-cells.csv',index=False);pd.DataFrame(groups).to_csv(O/'groups.csv',index=False)
paired={}
for model in summary:
 if model=='14200':continue
 a=df[df.model=='14200'].set_index('sample_id');b=df[df.model==model].set_index('sample_id').loc[a.index]
 paired[model+'_vs_14200']={'both_legal':int(((a.legal==1)&(b.legal==1)).sum()),'candidate_only_legal':int(((a.legal==0)&(b.legal==1)).sum()),'14200_only_legal':int(((a.legal==1)&(b.legal==0)).sum()),'both_fail':int(((a.legal==0)&(b.legal==0)).sum()),'different_planned_side':int((a.actual_side!=b.actual_side).sum())}
save('paired.json',paired)
real=pd.DataFrame(json.loads((W/'real-plan-ledger.json').read_text()));support=real[(real.height>=.8)&(real.height<=1.45)&(real.relative_y>=-.68)&(real.relative_y<=.28)]
coverage={'real_valid_plans':len(real),'real_within_position_support':len(support),'real_outside_position_support':len(real)-len(support),'real_support_below_v1_min_speed':int((support.incoming_speed<samples.predicted_contact_speed.min()).sum()),'real_support_below_v1_min_height':int((support.height<samples.predicted_contact_world_height.min()).sum()),'real_support_above_v1_max_height':int((support.height>samples.predicted_contact_world_height.max()).sum()),'real_speed_min':float(support.incoming_speed.min()),'limitations':'The 81 bins are covered, continuous range is not exhaustive. Historical low-speed tail below 1.80 m/s is outside v1; out-of-support plans retained. Real plans are predictions, not contacts.'}
save('real-coverage-limits.json',coverage)
fig,ax=plt.subplots(1,3,figsize=(15,4.6),constrained_layout=True)
core=samples[samples.section=='factorial81'];extra=samples[samples.section!='factorial81']
ax[0].scatter(core.origin_x,core.origin_y,s=7,alpha=.45,label='972 core');ax[0].scatter(extra.origin_x,extra.origin_y,s=12,alpha=.6,label='228 boundary');ax[0].set(xlabel='Launch x (m)',ylabel='Launch y (m)',title='Launch positions vary');ax[0].legend()
ax[1].scatter(support.incoming_speed,support.height,s=10,c='black',alpha=.45,label='268 real Planner predictions');ax[1].scatter(core.predicted_contact_speed,core.predicted_contact_world_height,s=7,alpha=.35,label='972 core');ax[1].set(xlabel='Incoming speed norm (m/s)',ylabel='Contact world height (m)',title='Height and speed are crossed');ax[1].legend(fontsize=8)
for v in [2.5,4]:ax[1].axvline(v,c='grey',lw=.6)
for v in [1,1.2]:ax[1].axhline(v,c='grey',lw=.6)
ax[2].scatter(samples.predicted_angle_deg,samples.target_relative_y,s=8,alpha=.5);ax[2].set(xlabel='Incoming horizontal angle (deg)',ylabel='Target y relative to HOME (m)',title='Angle and target are crossed')
fig.savefig(O/'coverage.png',dpi=180);plt.close(fig)
fig,axes=plt.subplots(2,3,figsize=(12,10),constrained_layout=True)
cc=pd.DataFrame(cells)
for mi,model in enumerate(['13800','14200']):
 for ai in range(3):
  grid=np.zeros((9,3),int)
  for _,r in cc[(cc.model==model)&(cc.angle_bin==ai)].iterrows():grid[int(r.lateral_bin)*3+int(r.height_bin),int(r.speed_bin)]=int(r.legal)
  a=axes[mi,ai];a.imshow(grid,vmin=0,vmax=12,cmap='YlGn');a.set_title(f'{model} | angle '+['left','straight','right'][ai]);a.set_xticks(range(3),['slow','medium','fast']);a.set_yticks(range(9),[f'{y}/{h}' for y in ['left','middle','right'] for h in ['low','mid','high']]);a.set_ylabel('target lateral / height')
  for y in range(9):
   for x in range(3):a.text(x,y,f'{grid[y,x]}/12',ha='center',va='center',fontsize=9)
fig.suptitle('Legal returns in each frozen core cell (12 distinct trajectories)');fig.savefig(O/'baseline-81-cells.png',dpi=180);plt.close(fig)
print(json.dumps(plain({'summary':summary,'paired':paired,'coverage':coverage}),indent=2))
