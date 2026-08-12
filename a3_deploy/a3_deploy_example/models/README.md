# Policy models

The published build_1 policy is complete and selected by the default runtime:

```text
models/model_21800/policy/
├── exported/policy.onnx       # obs[1,110] + time_step[1,1] -> actions[1,31]
└── params/deploy.yaml         # joint/action/PD/timing contract
```

Run it with `../scripts/run_pingpong_sim.sh`, or pass
`--policy-dir models/model_21800/policy` to the native runner. See
[`../../../docs/MODEL_21800.md`](../../../docs/MODEL_21800.md).

To exercise another compatible actor in the Python harness, pass
`--onnx /path/to/policy.onnx`. New native-runner exports should keep the same
`policy/{exported,params}` directory layout.
