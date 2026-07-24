# Policy models (Unitree G1)

Place your exported G1 actor policy here:

```
models/hope_pingpong_g1.onnx   # obs[1,105] -> raw_action[1,29], single output
models/policy_manifest.json    # optional: contract name, dims, control rate, joint order
```

The reference runtime config (`../config/hope_pingpong_runtime.yaml`) points
`policy.onnx_path` at `../models/hope_pingpong_g1.onnx`. Override on the command line
with `--onnx /path/to/hope_pingpong_g1.onnx`.

Export a G1 policy from the training package:

```bash
python scripts/export_onnx.py --task HOPE-PingPong-UnitreeG1-v0 \
  --checkpoint logs/rsl_rl/hope_pingpong_g1/<run>/model_<iter>.pt \
  --onnx-name hope_pingpong_g1.onnx
```

The sample motion clips that ship with training are reference examples only and are not
performance-tuned; replace them with your own before expecting a competitive policy. No policy
binary is distributed in this repository.
