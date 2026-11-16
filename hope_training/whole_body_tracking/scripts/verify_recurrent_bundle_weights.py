"""Independent checkpoint -> ONNX initializer verification, in existing grasping."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
import onnx
from onnx import numpy_helper
import torch
from portable_model_config import portable_motion_config

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 p=argparse.ArgumentParser(description=__doc__)
 for k in ['checkpoint','bundle','baseline','report']:p.add_argument('--'+k,type=Path,required=True)
 p.add_argument('--iteration',type=int,required=True);p.add_argument('--sha256',required=True);a=p.parse_args()
 torch.set_num_threads(2)
 assert sha(a.checkpoint)==a.sha256
 saved=torch.load(a.checkpoint,map_location='cpu',weights_only=False)
 assert saved['iter']==a.iteration and saved['hope_exact_resume_state']['wandb_run_id']=='gru-formal-2b8c09938a'
 state=saved['model_state_dict'];keys=sorted(k for k in state if k.startswith(('actor.','memory_a.rnn.','actor_obs_normalizer.')))
 arrays={x.name:numpy_helper.to_array(x) for x in onnx.load(str(a.bundle/'actor.onnx')).graph.initializer};matched={};h=hashlib.sha256()
 def gates(x):
  r,z,n=np.split(x,3,axis=0);return np.concatenate([z,r,n],axis=0)
 for k in keys:
  x=state[k].detach().cpu().contiguous().numpy();assert np.isfinite(x).all(),k
  h.update(k.encode());h.update(str(x.shape).encode());h.update(x.tobytes())
  if k.startswith('memory_a.rnn.'):continue
  names=[name for name,y in arrays.items() if x.shape==y.shape and np.array_equal(x,y)];assert names,k;matched[k]=names
 for suffix in ['weight_ih_l0','weight_hh_l0']:
  x=gates(state['memory_a.rnn.'+suffix].numpy())[None];assert any(x.shape==y.shape and np.array_equal(x,y) for y in arrays.values()),suffix
 x=np.concatenate([gates(state['memory_a.rnn.bias_ih_l0'].numpy()),gates(state['memory_a.rnn.bias_hh_l0'].numpy())])[None]
 assert any(x.shape==y.shape and np.array_equal(x,y) for y in arrays.values()),'GRU bias'
 m=json.loads((a.bundle/'manifest.json').read_text());b=json.loads(a.baseline.read_text())
 assert m['checkpoint_iteration']==a.iteration and m['checkpoint_sha256']==a.sha256
 assert m['wandb_run_id']=='gru-formal-2b8c09938a' and sha(a.bundle/'actor.onnx')==m['onnx_sha256']
 fields=['normalization','normalization_keys','joint_names','observation_layout','hidden_shape','action','actuator','root_default','physics_dt','decimation','control_dt','task','motion_cfg','visual_table','racket_site']
 for k in fields:
  actual=portable_motion_config(m[k]) if k=='motion_cfg' else m[k]
  expected=portable_motion_config(b[k]) if k=='motion_cfg' else b[k]
  assert actual==expected,f'Incompatible evaluator/training contract: {k}'
 report={'result':'PASS','iteration':a.iteration,'checkpoint_sha256':a.sha256,'onnx_sha256':m['onnx_sha256'],'actor_parameter_sha256':h.hexdigest(),'loaded_initializers':matched,'gru_weights_and_bias_match':True,'finite_actor_weights':True,'identical_environment_contract_fields':fields}
 a.report.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k!='loaded_initializers'}),flush=True)
if __name__=='__main__':main()
