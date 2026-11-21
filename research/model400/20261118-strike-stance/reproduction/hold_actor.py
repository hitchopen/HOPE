"""Blend selected lower-body action outputs with the same actor's side-specific waiting command."""
from pathlib import Path
import copy,json,hashlib
import numpy as np, pandas as pd,onnx,onnxruntime as ort,yaml
from onnx import helper as H,numpy_helper as N
w=Path(__file__).resolve().parent;base=w.parents[1]/'outputs/model400-supergate-v5/policy'
groups=[{'label':'v5','path':str(base)}]
for name,amount,all_legs in [('hold15',.15,True),('hold40',.4,True),('hold70',.7,True),('hold40lat',.4,False)]:
    m=onnx.load(base/'exported/policy.onnx');trunk=[copy.deepcopy(n) for n in m.graph.node if n.name.startswith('/actor/actor.')];assert len(trunk)==7
    p=next(n for n in m.graph.node if 'actions' in n.output);p.output[list(p.output).index('actions')]='swing_command'
    def op(k,*inputs,**attr):
        n='hold_actor_'+str(len(m.graph.node));m.graph.node.append(H.make_node(k,inputs,[n],name=n,**attr));return n
    def const(v,i=False):
        n='hold_weight_'+str(len(m.graph.initializer));m.graph.initializer.append(N.from_array(np.asarray(v,dtype=np.int64 if i else np.float32),n));return n
    def idx(i):return op('Gather','obs',const([i],True),axis=1)
    fh=op('Less',op('Sub',idx(104),idx(102)),const(-.24))
    cmd=op('Where',fh,const([.57,-.46,.056,1.95,.2,1.15,.82]),const([.57,.02,.056,2.,-.05,.9,.96]))
    mapping=np.zeros((2,7));mapping[:2,:2]=np.eye(2)
    ready=op('Add',cmd,op('MatMul',op('Gather','obs',const([101,102],True),axis=1),const(mapping)))
    h=op('Concat',op('Gather','obs',const(np.arange(103),True),axis=1),ready,axis=1)
    for j,n in enumerate(trunk):
        n.name=f'hold_trunk_{j}';n.input[0]=h;h=n.name+'_out';n.output[0]=h;m.graph.node.append(n)
    c=yaml.safe_load((base/'params/deploy.yaml').read_text());a=c['actions']['JointPositionAction']
    mask=[any(key in nm for key in (('hip','knee','ankle') if all_legs else ('hip_roll','ankle_roll','hip_yaw'))) for nm in a['joint_names']]
    diff=op('Mul',op('Sub',h,'swing_command'),const(np.array(mask)*amount));lim=.12/np.array(a['scale']);bounded=op('Max',const(-lim),op('Min',diff,const(lim)))
    active=op('Clip',op('Div',op('Sub',const(.76),idx(109)),const(.16)),const(0),const(1))
    delta=op('Mul',bounded,active)
    m.graph.node.append(H.make_node('Add',['swing_command',delta],['actions']))
    dest=w/name;(dest/'exported').mkdir(parents=True,exist_ok=True);(dest/'params').mkdir(exist_ok=True)
    onnx.checker.check_model(m);path=dest/'exported/policy.onnx';onnx.save(m,path)
    o=ort.SessionOptions();o.intra_op_num_threads=1;b=ort.InferenceSession(str(path),o);orig=ort.InferenceSession(str(base/'exported/policy.onnx'),o)
    df=pd.read_csv(w.parent/'opt400-support/gate-control/pp_obs.csv')
    for i in np.linspace(0,len(df)-1,64,dtype=int):
        x=df.iloc[i][[f'obs_{z}' for z in range(110)]].to_numpy('f')[None,:];feed={'obs':x,'time_step':np.array([[int(df.ts.iloc[i])]],'f')}
        yy=b.run(None,feed);zz=orig.run(None,feed);d=(yy[0]-zz[0])[0]
        assert not np.any(d[np.logical_not(mask)]) and np.max(np.abs(d*a['scale']))<.12001
        if x[0,109]>.76:assert np.array_equal(yy[0],zz[0])
        for v,u in zip(yy[1:],zz[1:]):np.testing.assert_array_equal(v,u)
    c['provenance']['policy_sha256']=hashlib.sha256(path.read_bytes()).hexdigest();c['provenance']['support_actor']={'parent':groups[0]['path'],'blend':amount,'joint_names':[joint for joint,k in zip(a['joint_names'],mask) if k],'max_delta_rad':.12}
    (dest/'params/deploy.yaml').write_text(yaml.safe_dump(c,sort_keys=False));groups.append({'label':name,'path':str(dest)})
(w/'hold-groups.json').write_text(json.dumps(groups,indent=2))
