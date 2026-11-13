from pathlib import Path
import json,math,sys
import numpy as np,pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,Ellipse
W=Path(__file__).resolve().parent;O=W.parent/'outputs/super-gate3-detailed-20261108';O.mkdir(exist_ok=True)
def clean(x):
 if isinstance(x,dict):return {str(k):clean(v) for k,v in x.items()}
 if isinstance(x,(list,tuple)):return [clean(v) for v in x]
 if isinstance(x,np.integer):return int(x)
 if isinstance(x,(float,np.floating)):return float(x) if np.isfinite(x) else None
 return x
def put(name,x):(O/name).write_text(json.dumps(clean(x),ensure_ascii=False,indent=2))
def describe(x):
 a=np.asarray(x,float);a=a[np.isfinite(a)]
 return {'n':len(a),'mean':float(a.mean()) if len(a) else None,'std':float(a.std(ddof=1)) if len(a)>1 else None,'median':float(np.median(a)) if len(a) else None,'p90':float(np.quantile(a,.9)) if len(a) else None,'p95':float(np.quantile(a,.95)) if len(a) else None,'max':float(a.max()) if len(a) else None}
groups=json.loads((W/'detailed-policy-groups.json').read_text());sets=[('main','detailed-main','super-gate3-v1',groups),('slow','detailed-slow','super-gate3-slow-tail',groups)]
if (W/'detailed-candidates/run.json').exists():sets.append(('main','detailed-candidates','super-gate3-v1',json.loads((W/'new-weight-groups.json').read_text())))
if (W/'detailed-candidates-slow/run.json').exists():sets.append(('slow','detailed-candidates-slow','super-gate3-slow-tail',json.loads((W/'new-weight-groups.json').read_text())))
trials=[];events=[];postures=[];landings=[];warnings=[]
for dataset,folder,fixture,gg in sets:
 root=W/folder
 if not (root/'run.json').exists():continue
 manifest=pd.DataFrame(json.loads((W.parent/'outputs'/fixture/'samples.json').read_text()))[['sample_id','shard_id','shot_id','section','lateral_bin','speed_bin','height_bin','angle_bin']]
 for gi,g in enumerate(gg):
  for shard in range(6):
   env=gi*6+shard;r=pd.read_csv(root/f'env-{env}-results.csv').rename(columns={'shot':'shot_id'});r['shard_id']=shard+1;r=r.merge(manifest,on=['shard_id','shot_id'],validate='one_to_one')
   audit=pd.read_csv(root/f'env-{env}-audit.csv').rename(columns={'shot':'shot_id'});r=r.merge(audit,on='shot_id',validate='one_to_one');r['model']=g['label'];r['dataset']=dataset;r['side_name']=np.where(r.side>0,'FH','BH');r['contact']=r.contacts>0;r['fall']=(r.min_z<.7)|(r.max_tilt>np.pi/4);r['clipped_time_fraction']=r.clipped_steps/r.control_steps
   kin=pd.read_csv(root/f'env-{env}-kinematics.csv').rename(columns={'shot':'shot_id'});kin=kin.merge(r[['shot_id','sample_id','dataset','model','side_name','legal','contact','section','speed_bin','height_bin','lateral_bin','angle_bin']],on='shot_id',validate='many_to_one')
   for c in ['p_error_m','dp_x','dp_y','dp_z']:kin[c.replace('_m','_cm') if c=='p_error_m' else c+'_cm']=kin[c]*100
   kin['surface_position_error_cm']=100*np.linalg.norm(kin[[f'surface_p_{a}' for a in 'xyz']].to_numpy()-kin[[f'target_p_{a}' for a in 'xyz']].to_numpy(),axis=1)
   kin['legacy_site_position_error_cm']=100*np.linalg.norm(kin[[f'legacy_site_p_{a}' for a in 'xyz']].to_numpy()-kin[[f'target_p_{a}' for a in 'xyz']].to_numpy(),axis=1)
   kin['surface_velocity_error_mps']=np.linalg.norm(kin[[f'surface_v_{a}' for a in 'xyz']].to_numpy()-kin[[f'target_v_{a}' for a in 'xyz']].to_numpy(),axis=1)
   body=pd.read_csv(root/f'env-{env}-posture.csv').rename(columns={'shot':'shot_id'});body['model']=g['label'];body['dataset']=dataset;body['shard_id']=shard+1
   recovery=[]
   for _,k in kin[kin.event=='planned'].iterrows():
    bb=body[body.shot_id==k.shot_id];delta=np.abs(bb.t-(k.deadline+.6));v=bb.iloc[int(delta.argmin())] if len(bb) else None
    recovery.append({'shot_id':k.shot_id,'recovery_available':v is not None and delta.min()<=.012,'recovery_torso_yaw_deg':v.torso_yaw_deg if v is not None and delta.min()<=.012 else np.nan,'recovery_pelvis_yaw_deg':v.pelvis_yaw_deg if v is not None and delta.min()<=.012 else np.nan})
   r=r.merge(pd.DataFrame(recovery),on='shot_id',how='left',validate='one_to_one');trials.append(r);events.append(kin);postures.append(body)
   warn=pd.read_csv(root/f'env-{env}-warnings.csv');warnings.append({'model':g['label'],'dataset':dataset,'shard_id':shard+1,'warnings':int(warn.number.sum())})
   # First descending crossing of the table-height plane, including off-table returns.
   ball=pd.read_csv(root/f'env-{env}-ball.csv')
   for _,row in r.iterrows():
    rr={'model':g['label'],'dataset':dataset,'sample_id':row.sample_id,'contact':bool(row.contact),'legal':int(row.legal),'side_name':row.side_name,'source':'no_contact','x':np.nan,'y':np.nan}
    if row.contact:
     bb=ball[(ball.shot==row.shot_id)&(ball.racket_contacts>0)].sort_values('t').drop_duplicates('t')
     values=bb[['t','x','y','z','vz']].to_numpy();found=False
     for i in range(1,len(values)):
      a,b=values[i-1],values[i]
      if a[3]>.7801 and b[3]<=.7801 and b[4]<0 and b[0]-a[0]<=.0081:
       f=(a[3]-.7801)/(a[3]-b[3]);pos=a+f*(b-a);rr.update(source='height_plane_projection',x=pos[1],y=pos[2]);found=True;break
     # Prefer the physical first post-racket table contact; projections are only fallback.
     if np.isfinite(row.landing_x):rr.update(source='physical_table_contact',x=row.landing_x,y=row.landing_y)
     elif not found:rr['source']='no_recorded_crossing'
    landings.append(rr)
df=pd.concat(trials,ignore_index=True);ev=pd.concat(events,ignore_index=True);body=pd.concat(postures,ignore_index=True);land=pd.DataFrame(landings)
land['error_x']=land.x-2.055;land['error_y']=land.y+.7625;land['radial_error_m']=np.hypot(land.error_x,land.error_y)
df.to_csv(O/'trials.csv',index=False);ev.to_csv(O/'racket-errors.csv',index=False);land.to_csv(O/'landing-points.csv',index=False);put('warning-counts.json',warnings)
metrics=['p_error_cm','v_error_mps','n_error_deg','dp_x_cm','dp_y_cm','dp_z_cm','dv_x','dv_y','dv_z','dn_x','dn_y','dn_z','surface_position_error_cm','surface_velocity_error_mps','legacy_site_position_error_cm','dt_ms','pelvis_yaw_deg','torso_yaw_deg','twist_deg']
summaries=[]
for (dataset,model,event),g in ev.groupby(['dataset','model','event'],sort=False):
 for axis in ['ALL','side_name','speed_bin','height_bin','angle_bin','lateral_bin','legal']:
  for key,part in ([('ALL',g)] if axis=='ALL' else g.groupby(axis)):
   for metric in metrics:summaries.append({'dataset':dataset,'model':model,'event':event,'axis':axis,'group':key,'metric':metric,**describe(part[metric])})
put('racket-error-statistics.json',summaries);pd.DataFrame(summaries).to_csv(O/'racket-error-statistics.csv',index=False)
posture_summary=[];landing_summary=[];outcomes=[]
for (dataset,model),g in df.groupby(['dataset','model'],sort=False):
 row={'dataset':dataset,'model':model,'n':len(g),'contacts':int(g.contact.sum()),'legal':int(g.legal.sum()),'falls':int(g.fall.sum()),'min_pelvis_height_m':float(g.min_z.min()),'max_tilt_deg':float(np.degrees(g.max_tilt.max())),'planned_samples':int(g.planned_sample.sum()),'contact_samples':int(g.contact_sample.sum()),'torso_peak_above45_trials':int((g.peak_torso_yaw_deg>45).sum()),'torso_peak_above60_trials':int((g.peak_torso_yaw_deg>60).sum()),'recovery_abs_torso_above15_trials':int((abs(g.recovery_torso_yaw_deg)>15).sum()),'recovery_available':int(g.recovery_available.sum())};outcomes.append(row)
 for axis in ['ALL','side_name']:
  for key,part in ([('ALL',g)] if axis=='ALL' else g.groupby(axis)):
   for metric in ['peak_pelvis_yaw_deg','peak_torso_yaw_deg','peak_twist_deg','end_pelvis_yaw_deg','end_torso_yaw_deg','recovery_pelvis_yaw_deg','recovery_torso_yaw_deg','peak_roll_deg','peak_pitch_deg','max_base_xy_drift_m','clipped_time_fraction']:
    posture_summary.append({'dataset':dataset,'model':model,'axis':axis,'group':key,'metric':metric,**describe(part[metric])})
 ll=land[(land.model==model)&(land.dataset==dataset)];known=ll[np.isfinite(ll.x)];legal=ll[ll.legal==1]
 landing_summary.append({'dataset':dataset,'model':model,'n':len(g),'contacts':int(g.contact.sum()),'measurable_landings':len(known),'physical_table_contacts':int((ll.source=='physical_table_contact').sum()),'projected_off_table':int((ll.source=='height_plane_projection').sum()),'no_crossing_after_contact':int((ll.source=='no_recorded_crossing').sum()),'x_mean_std':describe(known.x),'y_mean_std':describe(known.y),'dx_mean_std':describe(known.error_x),'dy_mean_std':describe(known.error_y),'radial_all_measurable':describe(known.radial_error_m),'radial_legal_only':describe(legal.radial_error_m),'cov_xy_m2':known[['x','y']].cov().to_numpy().tolist(),'short_x_below_1p37':int((known.x<1.37).sum()),'long_x_above_2p74':int((known.x>2.74).sum()),'y_below_minus1p525':int((known.y< -1.525).sum()),'y_above0':int((known.y>0).sum())})
put('outcomes.json',outcomes);put('posture-statistics.json',posture_summary);put('landing-statistics.json',landing_summary);pd.DataFrame(posture_summary).to_csv(O/'posture-statistics.csv',index=False)
# Conditional failure patterns, paired outcomes, and reproducible metric identities.
planned=ev[ev.event=='planned'];detail=df.merge(planned[['dataset','model','sample_id','p_error_cm','v_error_mps','n_error_deg','torso_yaw_deg','pelvis_yaw_deg']],on=['dataset','model','sample_id'],validate='one_to_one')
condition=[]
for (dataset,model),g in detail.groupby(['dataset','model'],sort=False):
 for axis in ['ALL','side_name','speed_bin','height_bin','angle_bin','lateral_bin','section']:
  for key,part in ([('ALL',g)] if axis=='ALL' else g.groupby(axis)):
   condition.append({'dataset':dataset,'model':model,'axis':axis,'group':key,'n':len(part),'contacts':int(part.contact.sum()),'legal':int(part.legal.sum()),'falls':int(part.fall.sum()),'position_cm':describe(part.p_error_cm),'velocity_mps':describe(part.v_error_mps),'normal_deg':describe(part.n_error_deg)})
put('condition-statistics.json',condition)
cell=detail[(detail.dataset=='main')&(detail.section=='factorial81')].groupby(['model','lateral_bin','height_bin','speed_bin','angle_bin']).agg(n=('legal','size'),legal=('legal','sum'),contacts=('contact','sum'),position_mean_cm=('p_error_cm','mean'),position_std_cm=('p_error_cm','std'),velocity_mean_mps=('v_error_mps','mean'),velocity_std_mps=('v_error_mps','std'),normal_mean_deg=('n_error_deg','mean'),normal_std_deg=('n_error_deg','std'),torso_peak_mean_deg=('peak_torso_yaw_deg','mean')).reset_index();cell.to_csv(O/'81-cells.csv',index=False)
paired=[]
from scipy.stats import binomtest
for dataset in df.dataset.unique():
 base=df[(df.dataset==dataset)&(df.model=='14200')][['sample_id','legal']].rename(columns={'legal':'baseline_legal'})
 for model in df.model.unique():
  g=df[(df.dataset==dataset)&(df.model==model)].merge(base,on='sample_id',validate='one_to_one');wins=int(((g.legal==1)&(g.baseline_legal==0)).sum());losses=int(((g.legal==0)&(g.baseline_legal==1)).sum())
  paired.append({'dataset':dataset,'model':model,'n':len(g),'wins_vs_14200':wins,'losses_vs_14200':losses,'net':wins-losses,'discordant_exact_binomial_p_unadjusted':float(binomtest(wins,wins+losses).pvalue) if wins+losses else 1.,'caution':'Descriptive paired check; multiple candidate selection, reused benchmark, trajectory sequence dependence. Not an independent generalization test.'})
put('paired-outcomes.json',paired)
checks={}
checks['unique_trial_ids']=not df.duplicated(['dataset','model','sample_id']).any();checks['one_planned_sample_per_trial']=len(planned)==len(df) and not planned.duplicated(['dataset','model','sample_id']).any()
checks['first_contact_count_matches']=int((ev.event=='contact').sum())==int(df.contact.sum());checks['planned_time_within_1ms']=bool(planned.dt_ms.between(-.0001,1.0001).all())
checks['position_norm_identity']=bool(np.allclose(np.linalg.norm(ev[['dp_x','dp_y','dp_z']],axis=1),ev.p_error_m,atol=1e-8));checks['velocity_norm_identity']=bool(np.allclose(np.linalg.norm(ev[['dv_x','dv_y','dv_z']],axis=1),ev.v_error_mps,atol=1e-8))
checks['target_normals_unit']=bool(np.allclose(np.linalg.norm(ev[['target_n_x','target_n_y','target_n_z']],axis=1),1,atol=1e-6));checks['measured_normals_unit']=bool(np.allclose(np.linalg.norm(ev[['racket_n_x','racket_n_y','racket_n_z']],axis=1),1,atol=1e-6))
checks['normal_angle_identity']=bool(np.allclose(np.degrees(np.arccos(np.clip(np.sum(ev[['target_n_x','target_n_y','target_n_z']].to_numpy()*ev[['racket_n_x','racket_n_y','racket_n_z']].to_numpy(),axis=1),-1,1))),ev.n_error_deg,atol=1e-5));checks['all_legal_have_physical_table_contact']=bool((land[land.legal==1].source=='physical_table_contact').all())
checks['core_81_cells_12_each']=len(cell)==81*df.model.nunique() and bool((cell.n==12).all());put('analysis-validation.json',{'checks':checks,'passed':all(checks.values()),'trial_rows':len(df),'kinematic_event_rows':len(ev),'std_definition':'sample standard deviation, ddof=1; normal errors in degrees, position norms in cm, velocity norms in m/s'})
assert all(checks.values()),checks
models=list(dict.fromkeys(df.model));main=ev[(ev.dataset=='main')&(ev.event=='planned')]
fig,axes=plt.subplots(1,3,figsize=(15,5),constrained_layout=True)
for ax,metric,title in zip(axes,['p_error_cm','v_error_mps','n_error_deg'],['Position error (cm)','Velocity error (m/s)','Face-plane angle error (deg)']):
 data=[main[main.model==m][metric].dropna() for m in models];ax.boxplot(data,tick_labels=models,showfliers=False);ax.set_ylabel(title);ax.tick_params(axis='x',rotation=50);ax.grid(axis='y',alpha=.25)
fig.suptitle('Original planned deadline: all trials, including misses (boxes hide outliers; full tails in data)');fig.savefig(O/'racket-errors.png',dpi=170);plt.close(fig)
ncols=3;nrows=math.ceil(len(models)/3);fig,axes=plt.subplots(nrows,ncols,figsize=(15,4.2*nrows),constrained_layout=True);axes=np.asarray(axes).ravel()
for ax,model in zip(axes,models):
 ll=land[(land.dataset=='main')&(land.model==model)];ax.add_patch(Rectangle((0,-1.525),2.74,1.525,facecolor='#ecf5ed',edgecolor='grey'));ax.axvline(1.37,color='grey',lw=.8)
 for mask,label,color in [(ll.legal==1,'legal','#168451'),((ll.legal==0)&(ll.source=='physical_table_contact'),'table, illegal','#e69722'),(ll.source=='height_plane_projection','off-table projection','#ce4052')]:
  x=ll[mask];ax.scatter(x.x,x.y,s=9,alpha=.4,label=label,color=color)
 ax.scatter([2.055],[-.7625],marker='+',s=100,c='black');ax.set(xlim=(-.5,4.5),ylim=(-2.3,.8),xlabel='x (m)',ylabel='y (m)',title=f'{model}: {int(ll.legal.sum())}/{len(ll)} legal');ax.set_aspect('equal');ax.legend(fontsize=7,loc='lower left')
 outside=((ll.x<-.5)|(ll.x>4.5)|(ll.y< -2.3)|(ll.y>.8)).sum();ax.text(.02,.97,f'{int((~ll.contact).sum())} misses; {outside} outside view',transform=ax.transAxes,va='top',fontsize=8)
for ax in axes[len(models):]:ax.axis('off')
fig.suptitle('Landing distributions: physical table contacts + measured descending height-plane crossings');fig.savefig(O/'landing-distributions.png',dpi=170);plt.close(fig)
fig,axes=plt.subplots(1,3,figsize=(15,5),constrained_layout=True)
for ax,metric,title in zip(axes,['peak_torso_yaw_deg','peak_pelvis_yaw_deg','recovery_torso_yaw_deg'],['Peak torso turn |yaw| (deg)','Peak pelvis turn |yaw| (deg)','Torso yaw at deadline +0.6 s (deg)']):
 d=df[df.dataset=='main'];ax.boxplot([d[d.model==m][metric].dropna() for m in models],tick_labels=models,showfliers=False);ax.set_ylabel(title);ax.tick_params(axis='x',rotation=50);ax.grid(axis='y',alpha=.25)
fig.suptitle('Body orientation relative to each environment\'s initial WAIT; turning is distinct from falling');fig.savefig(O/'body-orientation.png',dpi=170);plt.close(fig)
fig,axes=plt.subplots(math.ceil(len(models)/3),3,figsize=(13,3.7*math.ceil(len(models)/3)),constrained_layout=True);axes=np.asarray(axes).ravel()
for ax,m in zip(axes,models):
 g=df[(df.dataset=='main')&(df.model==m)];rates=g.groupby(['height_bin','speed_bin']).legal.agg(['sum','size']);arr=np.array([[100*rates.loc[(h,s),'sum']/rates.loc[(h,s),'size'] for s in range(3)] for h in range(3)])
 im=ax.imshow(arr,vmin=0,vmax=100,cmap='RdYlGn',origin='lower')
 for h in range(3):
  for s in range(3):ax.text(s,h,f'{arr[h,s]:.1f}%\n{int(rates.loc[(h,s),"sum"])}/{int(rates.loc[(h,s),"size"])}',ha='center',va='center',fontsize=10)
 ax.set_xticks(range(3),['slow','medium','fast']);ax.set_yticks(range(3),['low','medium','high']);ax.set_title(m);ax.set_xlabel('Incoming speed');ax.set_ylabel('Contact height')
for ax in axes[len(models):]:ax.axis('off')
fig.colorbar(im,ax=list(axes[:len(models)]),shrink=.6,label='Legal return rate (%)');fig.suptitle('Height × incoming speed: main dataset, all lateral targets and incoming angles');fig.savefig(O/'height-speed-coverage.png',dpi=170);plt.close(fig)
print(json.dumps(clean(outcomes),indent=2))
