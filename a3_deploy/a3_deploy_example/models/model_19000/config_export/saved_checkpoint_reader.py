"""Restricted, lazy read of known Torch ZIP tensor storages without Torch/Isaac.

Only the enumerated storage/NumPy reconstruction globals are accepted.
The caller must bind the trusted checkpoint SHA256 before reading.
"""
import pickle,zipfile,io,collections,codecs
import numpy as np
class StorageType:
 def __init__(self,dtype):self.dtype=dtype
class Storage:
 def __init__(self,z,key,dtype):self.z=z;self.key=key;self.dtype=dtype
class Tensor:
 def __init__(self,s,offset,size,stride,*args):self.s=s;self.offset=offset;self.size=size;self.stride=stride
 def numpy(self):
  a=np.frombuffer(self.s.z.read(self.s.key),dtype=self.s.dtype)
  return np.lib.stride_tricks.as_strided(a[self.offset:],shape=self.size,strides=tuple(x*a.dtype.itemsize for x in self.stride))
class Reader(pickle.Unpickler):
 def __init__(self,z,prefix,payload):self.z=z;self.prefix=prefix;super().__init__(io.BytesIO(payload))
 def find_class(self,module,name):
  if module=='torch' and name in ['FloatStorage','LongStorage','BoolStorage','ByteStorage']:return StorageType({'FloatStorage':'<f4','LongStorage':'<i8','BoolStorage':'?','ByteStorage':'u1'}[name])
  if (module,name)==('torch._utils','_rebuild_tensor_v2'):return Tensor
  if (module,name)==('collections','OrderedDict'):return collections.OrderedDict
  if (module,name)==('_codecs','encode'):return codecs.encode
  if module=='numpy' and name in ['dtype','ndarray']:return getattr(np,name)
  if (module,name)==('numpy.core.multiarray','_reconstruct'):return np._core.multiarray._reconstruct
  raise ValueError('Unapproved pickle global: '+module+'.'+name)
 def persistent_load(self,obj):
  assert obj[0]=='storage';return Storage(self.z,self.prefix+'/data/'+str(obj[2]),obj[1].dtype)
