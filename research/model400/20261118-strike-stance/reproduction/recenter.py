import hashlib,json
from pathlib import Path
import numpy as np,onnx,onnxruntime as ort,yaml
from onnx import helper as H,numpy_helper as N
w=Path(__file__).resolve().parent;p=w/'width3-aim0.35';group=w/'strike-groups.json';groups=json.loads(group.read_text())
for gain,cap in [(1.5,.045),(3.,.075)]:
    m=onnx.load(p/'exported/policy.onnx');nodes=list(m.graph.node);del m.graph.node[:]
    def op(kind,ins,out,**kw):
        m.graph.node.append(H.make_node(kind,ins,[out],name=out,**kw));return out
    def constant(name,v):
        m.graph.initializer.append(N.from_array(np.asarray(v),name));return name
    station=op('Gather',['obs',constant('home_indices',[101,102])],'home_error',axis=1)
    t=op('Gather',['obs',constant('time_index',[109])],'home_time',axis=1)
    gate=op('Greater',[t,constant('hold_threshold',np.float32(.76))],'home_active')
    scaled=op('Mul',[station,constant('hold_gain',np.float32(gain))],'home_scaled')
    limited=op('Clip',[scaled,constant('min_extra',np.float32(-cap)),constant('max_extra',np.float32(cap))],'home_limited')
    extra=op('Where',[gate,limited,constant('hold_zero',np.float32(0))],'home_extra')
    mix=np.zeros((2,110),'f');mix[0,[101,103]]=1;mix[1,[102,104]]=1
    expanded=op('MatMul',[extra,constant('hold_xy_map',mix)],'home_expanded')
    x=op('Add',['obs',expanded],'recentered_obs')
    for node in nodes:
        for i,arg in enumerate(node.input):
            if arg=='obs':node.input[i]=x
        m.graph.node.append(node)
    name=f'center{gain:g}';dest=w/name;(dest/'exported').mkdir(parents=True,exist_ok=True);(dest/'params').mkdir(exist_ok=True)
    onnx.checker.check_model(m);path=dest/'exported/policy.onnx';onnx.save(m,path)
    o=ort.SessionOptions();o.intra_op_num_threads=1
    a=ort.InferenceSession(str(p/'exported/policy.onnx'),o);b=ort.InferenceSession(str(path),o)
    for phase in [-.12,.4,.82]:
        x0=np.zeros((1,110),'f');x0[0,98]=-1;x0[0,99]=1;x0[0,109]=phase;x0[0,101:103]=[-.015,.030]
        x1=x0.copy()
        if phase>.76:x1+=np.clip(x1[:,101:103]*gain,-cap,cap)@mix
        pred=lambda s,x:s.run(None,{'obs':x,'time_step':np.array([[3]],'f')})
        for left,right in zip(pred(a,x1),pred(b,x0)):np.testing.assert_allclose(left,right,rtol=1e-5,atol=1e-6)
    cfg=yaml.safe_load((p/'params/deploy.yaml').read_text());cfg['provenance']['policy_sha256']=hashlib.sha256(path.read_bytes()).hexdigest();cfg['provenance']['recenter']={'extra_xy_gain':gain,'max_axis_extra_m':cap,'active_when_tts_above_s':.76,'coupled_channels':[101,102,103,104]}
    (dest/'params/deploy.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False));groups.append({'label':name,'path':str(dest)})
group.write_text(json.dumps(groups,indent=2))
