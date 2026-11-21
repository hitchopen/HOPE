"""Standalone exact-actor64-step parity and recurrent reset verification."""
from pathlib import Path
import json
import numpy as np
from runtime import RecurrentOnnxPolicy
root=Path(__file__).resolve().parent
p=RecurrentOnnxPolicy(root);f=np.load(root/'parity_vectors.npz');p.hidden=f['initial_h'].copy()
assert len(f['obs'])==64 and np.any(p.hidden) and int(f['reset_at'])==32
aerr=herr=0.
for i,obs in enumerate(f['obs']):
 if i==int(f['reset_at']):p.reset_episode()
 action=p.infer(obs)
 aerr=max(aerr,float(np.max(abs(action-f['actions'][i,0]))));herr=max(herr,float(np.max(abs(p.hidden-f['hidden'][i]))))
 np.testing.assert_allclose(action,f['actions'][i,0],rtol=2e-4,atol=2e-5)
 np.testing.assert_allclose(p.hidden,f['hidden'][i],rtol=2e-4,atol=2e-5)
p.reset_episode();a=p.infer(f['obs'][0]);h=p.hidden.copy()
for obs in f['obs'][1:17]:p.infer(obs)
p.reset_episode();np.testing.assert_array_equal(a,p.infer(f['obs'][0]));np.testing.assert_array_equal(h,p.hidden)
print(json.dumps({'result':'PASS','iteration':p.manifest['checkpoint_iteration'],'checkpoint_sha256':p.manifest['checkpoint_sha256'],'onnx_sha256':p.manifest['onnx_sha256'],'sequence_steps':64,'reset_at':32,'action_max_abs_error':aerr,'hidden_max_abs_error':herr,'atol':2e-5,'rtol':2e-4,'zero_reset_repeat':'PASS','behavioral_evaluation_performed':False}))
