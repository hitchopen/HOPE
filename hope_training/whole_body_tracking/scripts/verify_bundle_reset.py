"""Bounded repeat-after-reset check against the same exported recurrent actor."""
import argparse,json
from pathlib import Path
import numpy as np
from mujoco_eval_recurrent import RecurrentOnnxPolicy,dump

def verify(bundle,output):
 policy=RecurrentOnnxPolicy(bundle);obs=np.load(Path(bundle)/'parity_vectors.npz')['obs']
 policy.reset_episode();assert not np.any(policy.hidden)
 a=policy.infer(obs[0]);h=policy.hidden.copy()
 for x in obs[1:17]:policy.infer(x)
 policy.reset_episode();assert not np.any(policy.hidden)
 b=policy.infer(obs[0]);np.testing.assert_array_equal(a,b);np.testing.assert_array_equal(h,policy.hidden)
 dump(output,{'result':'PASS','iteration':policy.manifest['checkpoint_iteration'],'onnx_sha256':policy.manifest['onnx_sha256'],'reset_zeroes_hidden':True,'repeat_after_reset_action_and_hidden_exact':True,'training_hidden_used':False})
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--bundle',required=True);p.add_argument('--output',required=True);a=p.parse_args();verify(a.bundle,a.output)
