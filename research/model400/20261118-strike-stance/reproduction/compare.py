import itertools,json,sys
from pathlib import Path
import numpy as np,pandas as pd
w=Path(__file__).resolve().parent
selected=sys.argv[2] if len(sys.argv)>2 else json.loads((w/'gate-pick.json').read_text())['rank'][0]
stage=sys.argv[1];r=w/stage
a=pd.read_csv(r/'v5-shots.csv').set_index(['shard','shot']).sort_index()
b=pd.read_csv(r/(selected+'-shots.csv')).set_index(['shard','shot']).sort_index()
assert a.index.equals(b.index) and a['dataset_side'].equals(b['dataset_side'])
resamples=np.array(list(itertools.product(range(6),repeat=6)))
counts=np.stack([(resamples==s).sum(axis=1) for s in range(6)],axis=1)
def moments(stats,weights):
    t=weights@stats;n=t[:,0];mean=t[:,1]/n
    return np.stack((mean,t[:,2]/n-mean**2),axis=1)
report={}
for name,mask in [('all',np.ones(len(a),bool)),('FH',a.dataset_side=='FH'),('low',a.height_bin==0),('middle',a.height_bin==1),('high',a.height_bin==2)]:
    out={'n':int(sum(mask)),'v5_legal':int(a[mask].legal.sum()),'candidate_legal':int(b[mask].legal.sum()),'v5_collisions':int(a[mask].collision.sum()),'candidate_collisions':int(b[mask].collision.sum())}
    for key,col,scale in [('position_cm','p_error_m',100),('velocity_mps','v_error_mps',1),('normal_deg','n_error_deg',1)]:
        shared=mask & a[col].notna() & b[col].notna()
        sums=[]
        for d in [a,b]:
            stats=[]
            for i in range(1,7):
                v=d.loc[shared & (a.index.get_level_values('shard')==i),col].to_numpy()*scale
                stats.append([len(v),v.sum(),(v*v).sum()])
            sums.append(np.array(stats))
        paired=[moments(t,np.ones((1,6)))[0].tolist() for t in sums]
        intervals=np.quantile(moments(sums[1],counts)-moments(sums[0],counts),[.025,.975],axis=0)
        out[key]={'paired':int(shared.sum()),'v5':paired[0],'candidate':paired[1],'mean_delta_interval':intervals[:,0].tolist(),'variance_delta_interval':intervals[:,1].tolist()}
    report[name]=out
(r/'paired.json').write_text(json.dumps(report,indent=2))
