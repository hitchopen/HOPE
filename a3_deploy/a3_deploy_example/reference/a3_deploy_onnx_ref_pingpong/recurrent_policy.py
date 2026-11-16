"""Explicit-state CPU GRU inference; independent of the stateless deploy path."""
from pathlib import Path
import hashlib
import json

import numpy as np
import onnxruntime as ort


class RecurrentOnnxPolicy:
    def __init__(self, bundle):
        self.bundle = Path(bundle)
        self.manifest = json.loads((self.bundle/'manifest.json').read_text())
        if self.manifest['schema'] != 'hope_stationary_gru_mujoco_v1':
            raise ValueError('unsupported recurrent bundle schema')
        model = self.bundle/'actor.onnx'
        if hashlib.sha256(model.read_bytes()).hexdigest() != self.manifest['onnx_sha256']:
            raise ValueError('ONNX SHA mismatch')
        options = ort.SessionOptions()
        options.intra_op_num_threads = 2
        options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(str(model),options,providers=['CPUExecutionProvider'])
        if {x.name for x in self.session.get_inputs()} != {'obs','h_in'}:
            raise ValueError('expected obs and h_in')
        if {x.name for x in self.session.get_outputs()} != {'actions','h_out'}:
            raise ValueError('expected actions and h_out')
        self.hidden = np.zeros(self.manifest['hidden_shape'],dtype=np.float32)
        self.reset_count = 0
        self.steps = 0

    def reset_episode(self):
        self.hidden.fill(0)
        self.reset_count += 1

    def infer(self, obs):
        obs = np.asarray(obs,dtype=np.float32).reshape(1,110)
        if not np.isfinite(obs).all():
            raise FloatingPointError('non-finite observation')
        action, hidden = self.session.run(['actions','h_out'],{'obs':obs,'h_in':self.hidden})
        if action.shape != (1,31) or hidden.shape != tuple(self.manifest['hidden_shape']):
            raise ValueError('incorrect recurrent output shapes')
        if not np.isfinite(action).all() or not np.isfinite(hidden).all():
            raise FloatingPointError('non-finite recurrent actor output')
        self.hidden = hidden
        self.steps += 1
        return action[0]
