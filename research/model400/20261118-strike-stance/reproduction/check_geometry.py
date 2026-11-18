import numpy as np,pandas as pd,onnx,onnxruntime as ort,json
from pathlib import Path
w=Path(__file__).resolve().parent
r=w.parent/'opt400-support/gate-control'
d=pd.read_csv(r/'pp_obs.csv');f=pd.read_csv(r/'pp_mujoco_plant.csv',usecols=['wall_time_ns','left_foot_x','left_foot_y','right_foot_x','right_foot_y'])
joined=pd.merge_asof(d.sort_values('wall_time_ns'),f.sort_values('wall_time_ns'),on='wall_time_ns',direction='nearest',tolerance=6000000).dropna(subset=['right_foot_x'])
m=onnx.load(w/'foot2/exported/policy.onnx')
for name in ['clear_jointp_156','clear_jointp_317']:
    m.graph.output.append(onnx.helper.make_tensor_value_info(name,onnx.TensorProto.FLOAT,[3]))
o=ort.SessionOptions();o.intra_op_num_threads=1;s=ort.InferenceSession(m.SerializeToString(),o);error=[]
for i in np.linspace(0,len(joined)-1,300,dtype=int):
    row=joined.iloc[i];x=row[[f'obs_{k}' for k in range(110)]].to_numpy(dtype=np.float32);left,right=s.run(['clear_jointp_156','clear_jointp_317'],{'obs':x[None,:],'time_step':np.asarray([[row.ts]],dtype=np.float32)})
    horizontal=np.array([0,-x[98],x[97]]);horizontal/=np.linalg.norm(horizontal)
    observed=(left-right)@horizontal;correct=(row.left_foot_y-row.right_foot_y)*x[99]-(row.left_foot_x-row.right_foot_x)*x[100]
    error.append(100*(observed-correct))
receipt={'paired_samples':len(error),'width_difference_cm_mean':np.mean(error),'width_difference_abs_cm_p95':np.quantile(np.abs(error),.95),'scope':'Archived native Runner observation vs nearest plant timestamp; code reuses native policy FK in pelvis frame'}
print(receipt);(w/'width-check.json').write_text(json.dumps(receipt,indent=2))
