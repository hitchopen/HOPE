"""Standalone published actor sequence/reset check; no repository or torch needed."""
from pathlib import Path
import json
import numpy as np
from runtime import RecurrentOnnxPolicy
root=Path(__file__).resolve().parent
p=RecurrentOnnxPolicy(root);f=np.load(root/'parity_vectors.npz');p.hidden=f['initial_h'].copy()
for i,obs in enumerate(f['obs']):
 if i==int(f['reset_at']):p.reset_episode()
 action=p.infer(obs)
 np.testing.assert_allclose(action,f['actions'][i,0],rtol=2e-4,atol=2e-5)
 np.testing.assert_allclose(p.hidden,f['hidden'][i],rtol=2e-4,atol=2e-5)
p.reset_episode();a=p.infer(f['obs'][0]);h=p.hidden.copy()
for obs in f['obs'][1:17]:p.infer(obs)
p.reset_episode();np.testing.assert_array_equal(a,p.infer(f['obs'][0]));np.testing.assert_array_equal(h,p.hidden)
print(json.dumps({'result':'PASS','iteration':p.manifest['checkpoint_iteration'],'onnx_sha256':p.manifest['onnx_sha256'],'sequence_steps':64,'zero_reset_repeat':'PASS'}))
