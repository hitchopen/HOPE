# Supergate: reusable offline policy evaluation

Supergate runs supplied A3 native policy packages against frozen physical ball
launches. It uses one process, MuJoCo's internal batch pool, the checkout's native
policy and production Gate3 ball-contact implementation, and a pinned offline
Planner dependency.
Each policy receives the same six continuous shards.

This directory contains reusable code and test definitions. It contains no
personal model weights, historical run results, raw recordings, installed
libraries. Supply your own policy, robot geometry and matching ready state.
Raw time-series recording is off by default in the portable runner. The
[original research scripts](original/README.md) are also preserved verbatim;
those historical scripts retain workstation paths and their original behavior.

## Included suites

* `data/main`: 1,200 distinct physical launches; 972 samples cover 81 cells
  (lateral position × height × incoming speed × horizontal angle), 12 per cell,
  plus 228 origin-boundary samples. Six shards contain 200 launches each.
* `data/slow-tail`: 54 synthetic slow-tail launches, nine per shard. These are
  constructed near historical predicted contact conditions, not exact replay of
  measured ball spin or racket contact.

* `data/outer`: 402 outer-band launches, 67 per shard. Of 81 requested coverage
  cells, 67 produced six feasible samples each; 14 remain generation holes.
  `attempts.json` preserves successful and failed construction attempts.
* `data/mixed-long`: 1,200 launches, 200 continuous trials per shard: 402 outer,
  486 fresh inner holdout and 312 main. The original order seed is 202611090711;
  there are no duplicate launch/spin pairs within this suite.

The YAML/ledger pairs are byte-preserved frozen inputs. Their SHA256 manifests
also bind the design metadata. Outer and mixed-long include their original
`samples.json`, design and SHA256 files unchanged. Main, slow-tail and outer
construction precedes policy outcomes. Mixed-long was used for empirical range
selection informed by fall outcomes; it overlaps outer/main and is not an
independent holdout or a general safety claim. All suites are coverage tests,
not representative success-rate estimates. A modified suite must receive a new
manifest and identity.

## Dependencies and build

The tested environment is Linux, Python 3.12, MuJoCo **3.17.0**, NumPy, PyYAML,
C++20, Eigen3, yaml-cpp and ONNX Runtime C++ **1.19.2**. The MuJoCo package must
provide `mujoco/mjbatch.h` and its matching shared library. Build and run with the
same Python/MuJoCo environment. Python dependencies are listed in
`requirements.txt`; install native development packages through your normal
environment. ROS/DDS services are not needed for this backend.

From the repository root:

```sh
SG=a3_deploy/a3_deploy_example/scripts/supergate
python "$SG/build.py" --ort-root /path/to/onnxruntime-linux-x64-1.19.2 \
  --output /path/to/supergate-build
python -m unittest discover -s "$SG" -p test_supergate.py -v
```

The builder takes the native policy and production ball physics from the current
checkout. The offline Planner is bundled under `native/planner`, with source
hashes in `SOURCE_MANIFEST.json`: this is the Supergate diagnostic v2 snapshot,
including its return-physics solver, and differs from the runtime Planner. It
does not replace the runtime Planner. The builder generates an isolated,
thread-local simulation-clock header and extracts the production ball methods
into the build directory. It does not modify production headers or
use a previously installed robot binary. `build.json` records the compiler
command, source hashes, MuJoCo version and resulting library hash.

## Run a policy or compare policies

A policy directory must contain `exported/policy.onnx` and
`params/deploy.yaml`. Use a Gate3 XML with the nine-flight pool, SDK31 motor
bindings, required racket/table/net geometry and at least 63 userdata entries.
The ready-state NPZ must come from that same model and contain `qpos`, `qvel`,
`command` and `qa`; the existing `validate_serve_segments_mujoco.py` trace output
and `adapt_fusion_serve.py` ready-state output provide this format.

```sh
python "$SG/run.py" --library /path/to/supergate-build/native_batch.so \
  --model-xml /path/to/a3_pingpong.xml --ready-state /path/to/ready_reference.npz \
  --policy baseline=/path/to/baseline_policy \
  --policy candidate=/path/to/candidate_policy \
  --output /path/to/new-evaluation --threads 12
```

Use `--cases 1` for a six-launch-per-policy smoke test. Omit `--cases` for the
full suite. Use `--dataset "$SG/data/slow-tail"` for the slow set. Output must be
a new directory. `--config-dir` can select an explicit Planner configuration
directory; it must contain the four files named in `run.py`. Select the added
suites with `--dataset "$SG/data/outer"` or `--dataset "$SG/data/mixed-long"`.
Changing the dataset does not change the portable runner's admission profile.
The original outer/mixed-long range sweep used the v6 backend and explicit
support profiles; see [original scripts and protocol differences](original/README.md).
Policy/deploy,
dataset, ready-state, model XML, configuration and library hashes are recorded.
Geometry includes/meshes remain caller-managed model dependencies; the XML hash
alone is not a complete geometry-package hash.

Each policy group shares one explicitly supplied ready state for a paired
comparison. Run separate invocations when model-specific ready states are needed,
and account for that change when comparing results.

Outputs include per-trial result/audit CSVs, event-level racket diagnostics,
MuJoCo warnings, `run.json` and `summary.json`. The summary separates actual swing
signs and counts contacts, legal returns, valid plans and fall-indicator trials.
Fall indicators mean pelvis height below 0.7 m or tilt above 45 degrees during a
trial; they are not counts of independent physical falls. Missing or duplicate
trial rows fail reporting rather than silently improving the denominator.
`--raw-traces` explicitly enables larger per-step ball/posture CSVs when needed.

## Freeze another physical-launch suite

Provide a JSON list whose length is a nonzero multiple of six. Each record needs
`launch: [x,y,z,vx,vy,vz]`, `initial_spin: [wx,wy,wz]`, `flight_duration_s` and
optional `pause_s`. Launch z is table-relative; the runtime adds the 0.76 m table
height. All other launch positions/velocities use the existing Gate3 world axes.
The spin is in rad/s. Duplicate launch/spin pairs are rejected.

```sh
python "$SG/freeze_dataset.py" --samples /path/to/launches.json \
  --seed 2026110801 --output /path/to/new-suite
```

This deterministically shuffles and freezes supplied launches; it does not solve
for feasible trajectories or certify physical coverage. Built-in datasets retain
their original per-cell design and do not need to be regenerated before use.

## Scope and provenance

The protocol uses nominal HOME `(-0.5,-0.7625)`, fixed aim `(2.055,-0.7625)`, a
20 s WAIT warmup, 1 kHz physics, 50 Hz policy, 250 Hz observations, a fixed 8 ms
in-process delivery model and continuous state within each shard. Its lateral
admission profile is the historical Supergate profile; it is not automatically
derived from every new policy's training distribution.

The native backend is the diagnostic v2 Supergate implementation, adapted to
explicit inputs and current-checkout builds. Event diagnostics use separate
`mjData`, avoiding perturbation of the integration state. The frozen main and
slow-tail YAML/ledgers originate from the 2026-11-08 Supergate evaluation; an old
design label claiming a full Runner was corrected in the published metadata.

This tests policy, Planner and physics cores in simulation. It does not exercise
the outer Runner state machine, DDS, transport, HAL, managed lifecycle or robot
hardware, and it does not qualify deployment or physical hitting.
