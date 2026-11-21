from pathlib import Path
import json,hashlib,sys
import numpy as np,pandas as pd,onnx,onnxruntime as ort,yaml
from onnx import helper as H,numpy_helper as N
w=Path(__file__).resolve().parent
base=w.parents[1]/'outputs/model400-supergate-v5/policy'
groups=[{'label':'v5','path':str(base)}]
obs=pd.read_csv(w.parent/'opt400-support/gate-control/pp_obs.csv')
options=ort.SessionOptions();options.intra_op_num_threads=1
def session(m):return ort.InferenceSession(m.SerializeToString(),options)
def make(name,width_gain,yaw_gain,ankle,limit):
    m=onnx.load(base/'exported/policy.onnx')
    orig=session(m); cfg=yaml.safe_load((base/'params/deploy.yaml').read_text())
    action=cfg['actions']['JointPositionAction'];names=action['joint_names'];sc=np.asarray(action['scale'])
    source=next(n for n in m.graph.node if 'actions' in n.output);source.output[list(source.output).index('actions')]='native_action'
    for n in m.graph.node:
        for i,x in enumerate(n.input):
            if x=='actions':n.input[i]='native_action'
    def op(kind,*ins,**attr):
        out='stance_'+str(len(m.graph.node));m.graph.node.append(H.make_node(kind,list(ins),[out],name=out,**attr));return out
    def const(val,integer=False):
        key='stance_k'+str(len(m.graph.initializer));m.graph.initializer.append(N.from_array(np.array(val,dtype=np.int64 if integer else np.float32),key));return key
    def idx(x,i,axis=0):return op('Gather',x,const([i],True),axis=axis)
    # Horizontal Y component in the base-heading frame, corrected for body roll.
    upy=op('Neg',idx('obs',97,1));upz=op('Neg',idx('obs',98,1))
    norm=op('Max',op('Sqrt',op('Add',op('Mul',upy,upy),op('Mul',upz,upz))),const(.01))
    def lateral(vec):
        return op('Div',op('Sub',op('Mul',upz,idx(vec,1)),op('Mul',upy,idx(vec,2))),norm)
    lp,rp='clear_jointp_156','clear_jointp_317'
    upx=op('Neg',idx('obs',96,1));headx=idx('obs',99,1);heady=idx('obs',100,1)
    deltax=idx('obs',101,1);deltay=idx('obs',102,1)
    def front(vec):
        yz=op('Add',op('Mul',upy,idx(vec,1)),op('Mul',upz,idx(vec,2)))
        return op('Sub',op('Mul',norm,idx(vec,0)),op('Div',op('Mul',upx,yz),norm))
    def weighted(value,weights):
        gains=np.zeros(31)
        for joint,gain in weights.items():gains[names.index(joint+'_joint')]=gain
        return op('Mul',value,const(gains))
    correction=const(np.zeros(31))
    for side,sgn,pos,R in [('left',1,lp,'clear_jointR_157'),('right',-1,rp,'clear_jointR_318')]:
        # Desired world foot position relative to the fixed HOME, then expressed in the heading frame.
        wx=op('Add',deltax,const(-.002));wy=op('Add',deltay,const(sgn*.136))
        goalx=op('Add',op('Mul',headx,wx),op('Mul',heady,wy))
        goaly=op('Sub',op('Mul',headx,wy),op('Mul',heady,wx))
        dy=op('Sub',goaly,lateral(pos));dx=op('Sub',goalx,front(pos))
        correction=op('Add',correction,weighted(dy,{side+'_hip_roll':width_gain,side+'_ankle_roll':-width_gain*ankle}))
        correction=op('Add',correction,weighted(dx,{side+'_hip_pitch':-.6*width_gain,side+'_ankle_pitch':.35*width_gain}))
        forward=op('Squeeze',idx(R,0,1),axes=[1])
        heading=op('Add',op('Mul',heady,front(forward)),op('Mul',headx,lateral(forward)))
        correction=op('Add',correction,weighted(heading,{side+'_hip_yaw':-yaw_gain}))
    bounded=op('Clip',correction,const(-limit),const(limit))
    m.graph.node.append(H.make_node('Add',['native_action',op('Div',bounded,const(sc))],['actions']))
    onnx.checker.check_model(m);test=session(m)
    audit=[]
    for i in np.linspace(0,len(obs)-1,48,dtype=int):
        feed={'obs':obs.iloc[i][[f'obs_{k}' for k in range(110)]].to_numpy('f')[None,:],'time_step':np.array([[int(obs.ts.iloc[i])]],'f')}
        a=orig.run(None,feed);b=test.run(None,feed);delta=(b[0]-a[0])[0]*sc
        assert np.isfinite(delta).all() and abs(delta).max()<limit+1e-6
        assert all(np.array_equal(x,y) for x,y in zip(a[1:],b[1:]))
        assert all(x==0 for n,x in zip(names,delta) if not any(k in n for k in ('hip_roll','ankle_roll','hip_yaw','hip_pitch','ankle_pitch')))
        audit.append(float(abs(delta).max()))
    dest=w/name;(dest/'exported').mkdir(parents=True,exist_ok=True);(dest/'params').mkdir(exist_ok=True)
    path=dest/'exported/policy.onnx';onnx.save(m,path);sha=hashlib.sha256(path.read_bytes()).hexdigest()
    cfg['provenance']['policy_sha256']=sha;cfg['provenance']['supergate_optimization']={'name':name,'method':'ankle-FK world HOME foot position and heading feedback','width_gain_rad_m':width_gain,'yaw_gain':yaw_gain,'ankle_ratio':ankle,'limit_rad':limit,'target_width_m':.272,'parent_sha256':hashlib.sha256((base/'exported/policy.onnx').read_bytes()).hexdigest()}
    (dest/'params/deploy.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
    groups.append({'label':name,'path':str(dest)})
for spec in [('home2',2,.5,.7,.10),('home5',5,1,.7,.14),('home9',9,1.5,.7,.19),('home4soft',4,1,0,.12)]:
    make(*spec)
(w/'home-groups.json').write_text(json.dumps(groups,indent=2))
