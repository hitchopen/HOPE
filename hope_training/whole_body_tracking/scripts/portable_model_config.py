"""Normalize training motion provenance paths without changing numerical settings."""
from copy import deepcopy

def portable_motion_config(config):
 result=deepcopy(config)
 files=result.get('motion_file')
 if isinstance(files,(list,tuple)):
  marker='hope_training/motions/preprocessed/'
  result['motion_file']=[marker+str(p).split(marker,1)[1] if marker in str(p) else str(p) for p in files]
 return result
