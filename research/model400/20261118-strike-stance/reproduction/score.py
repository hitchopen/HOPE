from pathlib import Path
import sys,json
import pandas as pd,numpy as np
root=Path(sys.argv[1])
manifest=pd.read_json(Path(sys.argv[2])/'samples.json').rename(columns={'shard_id':'shard','shot_id':'shot','side':'dataset_side'})
def stat(d):
    p=d.p_error_m*100
    return {'n':len(d),'planned':int(p.notna().sum()),'completed':int(d.complete.sum()),'contacts':int((d.contacts>0).sum()),'legal':int(d.legal.sum()),'collisions':int(d.collision.sum()),'position_cm':[p.mean(),p.var(ddof=0)],'velocity_mps':[d.v_error_mps.mean(),d.v_error_mps.var(ddof=0)],'normal_deg':[d.n_error_deg.mean(),d.n_error_deg.var(ddof=0)]}
out={}
for i,g in enumerate(json.loads((root/'run.json').read_text())['policy_groups']):
    trials=[];ends=[]
    for j in range(6):
        pre=str(root/f'env-{i*6+j}')
        load=lambda name:pd.read_csv(pre+'-'+name+'.csv')
        r=load('results');k=load('kinematics').query('event=="planned"')
        r=r.merge(k[['shot','p_error_m','v_error_mps','n_error_deg']],on='shot',how='left',validate='1:1')
        r['collision']=r.shot.isin(load('collisions').query('normal_force>1').shot);r['shard']=j+1;trials.append(r)
        f=load('stance');start=f.loc[f.shot==0].iloc[-1];end=f.iloc[-1]
        end_drift=[float(np.hypot(end[s+'_x']-start[s+'_x'],end[s+'_y']-start[s+'_y'])) for s in ('left','right')]
        yaw=[float(abs(np.degrees(np.arctan2(np.sin(end[s+'_yaw']-start[s+'_yaw']),np.cos(end[s+'_yaw']-start[s+'_yaw']))))) for s in ('left','right')]
        ends.append({'end_drift_cm':100*max(end_drift),'end_width_cm':100*(end.left_y-end.right_y),'end_heading_deg':max(yaw),'max_width_cm':100*(f.left_y-f.right_y).max(),'min_width_cm':100*(f.left_y-f.right_y).min(),'minimum_base_height':float(r.min_z.min())})
    d=pd.concat(trials).merge(manifest[['shard','shot','height_bin','dataset_side']],on=['shard','shot'],validate='1:1')
    v={'all':stat(d),'by_side':{a:stat(b) for a,b in d.groupby('dataset_side')},'by_height':{str(a):stat(b) for a,b in d.groupby('height_bin')},'shards':ends,'mean_end_drift_cm':np.mean([e['end_drift_cm'] for e in ends]),'max_end_drift_cm':max(e['end_drift_cm'] for e in ends),'max_end_heading_deg':max(e['end_heading_deg'] for e in ends)}
    out[g['label']]=v;d.to_csv(root/(g['label']+'-shots.csv'),index=False)
    print(g['label'],v['all'],'drift_cm',v['mean_end_drift_cm'],v['max_end_drift_cm'],'yaw',v['max_end_heading_deg'])
(root/'score.json').write_text(json.dumps(out,indent=2))
