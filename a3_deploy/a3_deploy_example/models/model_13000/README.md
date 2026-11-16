# model_13000: recurrent StationaryRally actor

**export verified; no new behavioral evaluation performed.**

[Download portable ZIP](model_13000_gru_bundle.zip) · [ONNX](policy/exported/actor.onnx) · [Manifest](policy/exported/manifest.json) · [Runtime configuration](policy/params/runtime.json) · [Model index](../README.md)

Run `gru-formal-2b8c09938a`; exact iteration **13000**, no substitution. Training source `33c609022efbb4e85fe6a58a5a8947b0fbd46c76`.

- Checkpoint SHA256: `d9c0ea9c3401a40fb958c14e531a197544d3fcb2bfc57b981443adcbf240b41e`.
- ONNX SHA256: `6e822a73ea97b440dd2ca78b8b0d18112a8fb1b1165d3ab6fa95ce6094822b77`.
- Actor parameter fingerprint: `c75c883914e9e17d195966dd42272d185b31d6f0d39e7acccc66e5528795a2de`.

Original large PT and full private training YAML remain in the local snapshot; no PT is distributed. This package contains its own actual ONNX, saved PyTorch sequence fixtures, exact initializer-verification evidence, host parity/reset report, normalization/joint/action/actuator/control configuration, standalone runtime, pinned dependency lists and per-file hashes. No evidence from another checkpoint is relabelled.

## Verify and load

Use an existing compatible Python with the versions in `policy/exported/requirements-runtime.txt` (NumPy/ONNX Runtime); no automatic installation is performed. From this repository's root:

```bash
python a3_deploy/a3_deploy_example/models/model_13000/policy/exported/verify.py
```

After extracting the ZIP, from the extracted root:

```bash
python policy/exported/verify.py
cd policy/exported
python - <<'PYTHON'
from pathlib import Path
from runtime import RecurrentOnnxPolicy
actor = RecurrentOnnxPolicy(Path('.'))
actor.reset_episode()
print(actor.manifest['checkpoint_iteration'], actor.hidden.shape)
# Each real control tick: action = actor.infer(observation_110)
# The observation must follow this bundle's saved contract; carry memory.
PYTHON
```

The standalone test checks64 consecutive saved PyTorch/ONNX outputs with arbitrary nonzero initial memory, a zero reset at step32, and exact repeated inference after resetting from different histories. `host-verification.json` records observed errors; the manifest records the exporter PyTorch stateful-oracle error. Export verification is not standing/ball/robot performance evidence.

## Recurrent runtime contract

Inputs: `obs:float32[1,110]`, `h_in:float32[1,1,110]`; outputs: `actions:float32[1,31]`, `h_out:float32[1,1,110]`. Initialize hidden state to zero once at genuine episode start; carry the returned hidden state through all control ticks/serves/recovery. Reset only for a real new episode. Never reuse training-batch memory or reset every tick/ball. Normalization is `identity` and owned by the graph. No new actor inputs or side labels are added.

The manifest/runtime JSON define all observation slices, saved joint order, passive columns, affine action scale/offset/clamp, delay FIFO, actuator parameters and20ms control timing. Original training metadata is4×5ms; the compatible existing physical MuJoCo evaluator uses the documented20×1ms override. Actor inference itself needs no MuJoCo, ROS or training libraries. Native feed-forward/hardware runners are incompatible with this hidden-state ABI and must not be pointed at this actor without an adapter.

Shared nominal MuJoCo/planner configuration is included only for compatibility, not as evidence of a new evaluation. No new standing trials, ball evaluations, comparisons or videos were run. Do not infer performance from model19000/29999 or prior results. Remaining PhysX/MuJoCo sensing, drive/contact, lifecycle and scoring differences remain documented in the existing evaluation docs. No real-robot deployment claim is made.
