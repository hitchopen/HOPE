# Policy models

Place your exported actor policy here:

```
models/hope_pingpong.onnx      # obs[1,111] -> raw_action[1,31], single output
models/policy_manifest.json    # optional: contract name, dims, control rate, joint order
```

The reference runtime config (`../config/hope_pingpong_runtime.yaml`) points
`policy.onnx_path` at `../models/hope_pingpong.onnx`. Override on the command line
with `--onnx /path/to/hope_pingpong.onnx`.

Export a policy from the training package (`export_onnx.py`). The sample motion
clips that ship with training are reference examples only and are not
performance-tuned; replace them with your own before expecting a competitive
policy. No policy binary is distributed in this repository.
