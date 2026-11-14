# Local Foxglove policy assets

This directory publishes the local ONNX policies and matching deploy YAML,
serve CSV and Planner profiles for all **12 policy IDs** exposed by the installed
Console 1.9.20-recovery.0. It contains **17 named variants** because Fusion and
14200 Best have multiple local versions. The files are byte-preserved ordinary
Git blobs: cloning this branch downloads the models themselves, without an
additional LFS download.

`catalog.json` binds every asset to its SHA256 and source, and maps each model ID
to its variants. It is a publication index, not a robot installation catalog.
No default selection is changed. Scope was explicitly limited to local files;
robot availability and the currently selected robot policy were not verified.

| Console ID | Published variants | Source status |
| --- | --- | --- |
| `model11999` | `catalog` | Local catalog snapshot; receive-only, no serve CSV |
| `model12000` | `catalog` | Matched local serve bundle |
| `model12600` | `catalog` | Matched local serve bundle |
| `model13800` | `catalog` | Matched local serve bundle |
| `model13800astra` | `catalog` | Matched local serve bundle |
| `model14200` | `catalog` | Matched local serve bundle |
| `model14200best` | `best-v1`, `earlier-staged` | Different weights/configs/CSVs; keep each set together |
| `model14600` | `catalog` | Matched local serve bundle |
| `model17400` | `catalog` | Local serve-capable bundle; a historical receive-only installation uses the same weights/config |
| `model14200fusion` | `staged` | Local staged bundle |
| `model15200body` | `offline-candidate` | Original status records falls in all six continuous test environments; remains a candidate |
| `modelfusion` | `original-wrapper`, `mix25`, `phase`, `clock`, `joint-astra` | Locally preserved variants, not a claim they are all installed |

The `modelfusion/joint-astra` actor is byte-identical to the named Astra policy;
its original alias and matched CSV are retained. The original Fusion wrapper has
only a parity record and no matched serve CSV or Planner bundle; later staged
Fusion variants include those assets. The earlier 14200 Best CSV in the source
checkout differs from best-v1; both exact pairs are published separately.

## Layout and verification

Each variant contains `policy/exported/policy.onnx` and
`policy/params/deploy.yaml`. Matched serve assets live under `serve/` and Planner
profiles under `planner/`. Original candidate status records are under `evidence/`.
Training checkpoints, installed runtime libraries, recordings and raw trajectories
are not required by this asset collection.

From the repository root:

```sh
ASSETS=a3_deploy/a3_deploy_example/models/foxglove_policies
python "$ASSETS/verify.py"
# Optional ONNX structural and CPU inference checks:
python "$ASSETS/verify.py" --onnx
```

The first command uses only the Python standard library and verifies every hash,
all 12 model IDs, Planner JSON structure and the policy/deploy hashes embedded in
each serve CSV. The second also requires `onnx`, `onnxruntime`, `numpy` and `PyYAML`,
checks self-contained ONNX weights, and performs a finite-output CPU smoke test.
Neither command starts the robot or assesses physical policy performance.

Pass a chosen variant's `policy/` directory to a compatible native Runner or to
the [Supergate runner](../../scripts/supergate/README.md). Runtime/source ABI,
robot geometry and ready-state compatibility still depend on that evaluation.
The Planner profiles retain their original per-variant settings; publishing them
does not install them or prove they match an online machine.
