# Model 14200 Best

Complete local policy bundles for Foxglove `model14200best`.

- `best-v1/`: Best v1, policy SHA256 `5344f9a220b953ca4cf4c5c57b536be6f04b5a88d13c0ac19b3f3c00c70adba2`, deploy SHA256 `94bd36a8fa190e9cdb150d68a3b06045e43952d58343c659e1c21acef629feed`.
- `earlier-staged/`: the earlier local bundle, policy SHA256 `95e63b544763ee52e9d6dfa45e0584e4205479a09eba2ba31cf2572e9e9be7e8`.

Each variant includes the ONNX policy, deploy YAML, matched serve CSV and Planner profile. Keep the four files from the same variant together. In particular, the older `agi/a3_deploy_example/assets/a3_runtime/serve/motions/v23-14200-best.csv` belongs to `earlier-staged`, not `best-v1`.

`manifest.json` records exact hashes and original source identities. All assets are ordinary Git blobs, with no extra LFS download needed. This is asset publication only: it does not change the selected policy, install files on a robot, or qualify physical behavior.
