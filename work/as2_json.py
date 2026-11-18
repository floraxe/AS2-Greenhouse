"""JSON for experiment records, including NumPy scalar/array diagnostics."""
import json as _json

loads=_json.loads
JSONDecodeError=_json.JSONDecodeError

def _native(value):
    import numpy as np
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,np.generic):return value.item()
    raise TypeError(f'Unsupported experiment record: {type(value).__name__}')

def dumps(value,*args,**kwargs):
    kwargs.setdefault('default',_native)
    return _json.dumps(value,*args,**kwargs)
