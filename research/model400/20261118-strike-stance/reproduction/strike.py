from pathlib import Path
import json,hashlib
import numpy as np,pandas as pd,onnx,onnxruntime as ort,yaml
from onnx import helper as H,numpy_helper as N
w=Path(__file__).resolve().parent;src=w/'screen'
groups=json.loads((w/'groups.json').read_text());variants=[(label,alpha) for label in ['width3','foot2'] for alpha in [.35,.7]]
out=[groups[0]]
for label,alpha in variants:
    g=next(g for g in groups if g['label']==label);index=groups.index(g);base=Path(g['path']);m=onnx.load(base/'exported/policy.onnx');orig=list(m.graph.node);del m.graph.node[:]
    rows=[]
    for j in range(6):
        stem=str(src/f'env-{6*index+j}')
        kin=pd.read_csv(stem+'-kinematics.csv').query('event=="planned"')
        rows.append(kin.merge(pd.read_csv(stem+'-results.csv')[['shot','side']],on='shot',validate='1:1'))
    d=pd.concat(rows)
    def op(k,*inputs,**attr):
        name='strike_cal_'+str(len(m.graph.node));m.graph.node.append(H.make_node(k,inputs,[name],name=name,**attr));return name
    def const(a,dtype=np.float32):
        n='strike_const_'+str(len(m.graph.initializer));m.graph.initializer.append(N.from_array(np.asarray(a,dtype=dtype),n));return n
    def idx(i):return op('Gather','obs',const([i],np.int64),axis=1)
    def ramp(x,center,span):return op('Clip',op('Div',op('Sub',x,const(center)),const(span)),const(0),const(1))
    z=idx(105);y=op('Sub',idx(104),idx(102));fh=ramp(op('Neg',y),.22,.04)
    low=ramp(op('Neg',z),.01,.12);high=ramp(z,.01,.15);mid=op('Sub',op('Sub',const(1),low),high)
    clock=idx(109);phase=op('Mul',ramp(clock,-.22,.14),ramp(op('Neg',clock),-.65,.15))
    correction=const(np.zeros(110));bias=[]
    for sign,side in [(1,fh),(-1,op('Sub',const(1),fh))]:
        for band,weight,mask in [('low',low,d.target_p_z<.97),('mid',mid,(d.target_p_z>=.97)&(d.target_p_z<1.17)),('high',high,d.target_p_z>=1.17)]:
            part=d[(d.side==sign)&mask]
            err=part[['dp_x','dp_y','dp_z','dv_x','dv_y','dv_z']].median().to_numpy()
            assert np.isfinite(err).all()
            shift=np.zeros(110);shift[103:109]=-alpha*np.clip(err,[-.035]*3+[-.16]*3,[.035]*3+[.16]*3)
            correction=op('Add',correction,op('Mul',op('Mul',side,weight),const(shift)))
            bias.append({'side':sign,'height':band,'n':len(part),'obs_delta':shift[103:109].tolist()})
    adjusted=op('Add','obs',op('Mul',phase,correction))
    for n in orig:
        for i,k in enumerate(n.input):
            if k=='obs':n.input[i]=adjusted
        m.graph.node.append(n)
    name=label+'-aim'+str(alpha);dest=w/name;(dest/'exported').mkdir(parents=True,exist_ok=True);(dest/'params').mkdir(exist_ok=True)
    onnx.checker.check_model(m);path=dest/'exported/policy.onnx';onnx.save(m,path)
    o=ort.SessionOptions();o.intra_op_num_threads=1;a=ort.InferenceSession(str(base/'exported/policy.onnx'),o);b=ort.InferenceSession(str(path),o)
    data=pd.read_csv(w.parent/'opt400-support/gate-control/pp_obs.csv')
    for i in np.linspace(0,len(data)-1,36,dtype=int):
        x=data.iloc[i][[f'obs_{k}' for k in range(110)]].to_numpy('f');feed={'obs':x[None,:],'time_step':np.array([[int(data.ts.iloc[i])]],'f')}
        y0=a.run(None,feed);y1=b.run(None,feed);assert np.isfinite(y1[0]).all()
        if x[109]>=.65 or x[109]<=-.22:np.testing.assert_array_equal(y0[0],y1[0])
        for v,u in zip(y0[1:],y1[1:]):np.testing.assert_array_equal(v,u)
    cfg=yaml.safe_load((base/'params/deploy.yaml').read_text());cfg['provenance']['policy_sha256']=hashlib.sha256(path.read_bytes()).hexdigest();cfg['provenance']['strike_calibration']={'design':'bounded side-height median tracking correction learned only on 120 development shots','parent':label,'magnitude':alpha,'coefficients':bias}
    (dest/'params/deploy.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False));out.append({'label':name,'path':str(dest)})
(w/'strike-groups.json').write_text(json.dumps(out,indent=2))
