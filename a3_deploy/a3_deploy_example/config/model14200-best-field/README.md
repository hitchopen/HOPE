# Frozen 14200 Best field admission

These paired deltas preserve the exported training metadata and station input.
The Runner and Planner both admit relative to the measured live base, with
forehand [-0.68, -0.24] m and backhand [-0.24, +0.40] m. Apply both deltas only
to the matching bundle described in `qualification-receipt.json`; they are not
a default for other policies or later research weights. The evidence describes
sampled simulation acceptance, not universal physical stability.

The source branch contains configuration and serve CSV recipes, not ONNX weights,
installed binary packages, raw captures or current robot selections. Bundle
preparation still checks policy/deploy/CSV hashes and matched simulation evidence.
Rebuilding a package does not apply these optional deltas automatically.

No target-body rotation was added: the inspected training contract uses world
position deltas together with separate body-heading observations. Ball physics
coefficients were not refitted by these changes.

`v23-14200-best.csv` is the final frozen Best v1 handoff (original generator
name `v23-14200-weight-v3`), SHA-256
`14c75e8e2f17234e26d2498c7b75da9b2f2b8aad48f8e31659901c56fa856630`.
It supersedes the earlier Best candidate CSV; its recipe preserves the original
name so regeneration retains the exact validated bytes.
