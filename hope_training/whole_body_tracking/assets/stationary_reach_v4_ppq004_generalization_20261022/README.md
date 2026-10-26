# 3star / PPQ004 stationary-rally question bank

This is the default bank for `task=HOPEPingPongStationaryRally`, recipe revision
21. The task also pins `configs/ball_physics_ppq004_training.yaml` and its SHA256;
`scripts/train.py` selects that profile automatically. Manual bank-path, receipt,
hash, or `HOPE_BALL_PHYSICS_YAML` overrides are not required. The task profile
takes precedence over a stale shell environment variable. Other tasks retain
their existing profile selection.

From the repository's training directory, inside the Isaac environment:

```bash
source setup_train_env.sh
hope_isaac_py scripts/train.py \
  task=HOPEPingPongStationaryRally algo=ppo_balanced_hold \
  headless=true device=cuda:0 num_envs=6144 max_iterations=30000 seed=0 \
  checkpoint_path=null warm_start_actor_checkpoint=null \
  run_name=stationary_reach_ppq004_generalization_scratch
```

The command starts a new training run. Updating the YAML does not change an
already-running process or its saved `params/task_resolved.yaml`.

| Split | Retained questions | New synthetic questions | Total |
| --- | ---: | ---: | ---: |
| train | 183,274 | 318,779 | 502,053 |
| dev | 21,852 | 39,963 | 61,815 |
| test | 22,452 | 39,651 | 62,103 |

All three `.npz` files and their receipts are stored directly in Git, not as
Git LFS pointers. Each file is below GitHub's 100 MiB file limit. A normal Git
checkout supplies the complete training and evaluation question arrays.
SHA256 values are recorded in the receipts; the task additionally pins the
training bank and receipt hashes. The NPZ payloads have not been regenerated
or changed for publication.

The operator identifies PPQ004 as the 3star calibration. Flight and table
coefficients derive from the upstream HOPE PPQ004 profile at commit
`8d24642474205f6cfb48263cd22896306feeac47`; paddle contact retains the prior venue
model. New questions are synthetic pre-bounce trajectories with fitted return
targets, not additional measured 3star recordings. Robot right is negative Y.

`coverage.json` gives the split distributions; `generation_report.json` records
the bounded search and unfilled cells. Split parent identities are disjoint.
Physical qualification does not establish robot feasibility or policy success.

Receipts retain the original generator hashes and historical absolute source
paths as provenance. Those paths are not runtime dependencies. The large
`*.fresh_trajectories.npz` diagnostic sidecars, progress snapshots, and the local
`bank_identity.json` are not needed by the loader and are not included here.
