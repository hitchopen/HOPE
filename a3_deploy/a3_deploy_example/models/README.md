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

<!-- four-frozen-gru-exports -->

## Additional frozen recurrent exports

**export verified; no new behavioral evaluation performed.** Run `gru-formal-2b8c09938a`. These bundles have no new standing/ball results or videos.

| Iteration | Portable bundle | Loading and verification |
|---|---|---|
| 13000 | [ZIP](model_13000/model_13000_gru_bundle.zip) | [README](model_13000/README.md) |
| 14000 | [ZIP](model_14000/model_14000_gru_bundle.zip) | [README](model_14000/README.md) |
| 14600 | [ZIP](model_14600/model_14600_gru_bundle.zip) | [README](model_14600/README.md) |
| 20000 | [ZIP](model_20000/model_20000_gru_bundle.zip) | [README](model_20000/README.md) |
