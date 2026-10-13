# HitterPingPong two-file deployment

Current commands were checked against build_6 on 2026-10-11. Follow the
[complete deployment/Gate 3 sequence](../../docs/operations/deploy_and_gate3.md)
for x86 packaging, ROS/native simulator installation, policy inspection,
fixed-home execution, saved reports, cross-building and hardware package staging.

The policy-owned files are `params/deploy.yaml` and `exported/policy.onnx`.
They own observation/action semantics, joint mapping, PD arrays and provenance;
the runtime YAML owns transport/localization. Native inspection validates the
actual pair. This branch supports multiple policy ABIs, including 110-D
ball-clock, 112-D SmallStation and 324-D Compact in the unified Gate 3 entry;
observation size alone does not establish compatibility.

Use the selected task's exporter in [run_training.md](../../QUICKSTART_A3_ISAAC.md).
The standard HitterPingPong exporter and Compact exporters are different entry
points. Retain the exact run-local inputs and selected checkpoint. Do not edit
C++ or hand-change metadata to rename an old policy into a compatible ABI.

The current command sequence is:

1. Select the exact export and build a complete package with explicit
   `--runtime-cfg` and `--policy-dir`.
2. Build the intended ROS dependency closure and install the current native
   MuJoCo source in a separate ignored build directory.
3. Configure the real install paths with root `run_gate3.sh --configure`, then
   use `--check` before an explicit simulation run.
4. Read exit status and the saved `run.json` plus rally/physical-ball/plant/
   Planner/drain evidence; simulator certification and hardware qualification
   remain separate.
5. Cross-build and stage a new immutable robot slot, then follow the current
   [Foxglove/lifecycle procedure](../../docs/operations/foxglove_first_hardware_test.md).

`build_6/scripts/pp_gate3_hitter_pingpong.sh` is the internal backend; it does
not parse `--help` or `--preflight-only`. Use root `run_gate3.sh --help` and
`--check`. Public main's dedicated model_21800 backend has its own preflight
option; do not transfer that switch to this branch.

The material below is the dated August 2026 workflow and execution receipt.
Its fixed checkpoint, old build directory, extra manual contact gate and
standalone backend commands do not replace the current CLI workflow above.
The archived shell examples are displayed as text.

---

# HitterPingPong: Unitree-style deploy workflow

This section records the 2026-08-03 Build/HitterPingPong policy workflow.  A
deployable policy is one directory with exactly these policy-owned files:

```text
<run>/
├── exported/policy.onnx
└── params/deploy.yaml
```

`deploy.yaml` owns the policy contract: control period, joint names and mapping,
default pose, action scale/offset/safe interval, Kp/Kd, ordered observation
terms, dimensions, and contract identifiers.  The A3 runtime YAML owns only
transport/localization settings.  Changing checkpoints must not require editing
C++ or rewriting `onnx.model_path`.

The 110-D ping-pong observation is task-specific, so the generic loader selects
the `a3_pingpong_hitter_pure` observation plugin named by `task_plugin`.  MuJoCo
and hardware use the same runner, plugin, policy directory, and command decoder;
only the RobotIO transport/backend changes.

## Historical 2026-08-03 workflow

### 1. Export the two-file policy directory

Run in the training environment.  The formal export script now writes ONNX and
then generates/validates `params/deploy.yaml`:

```text
cd /home/dongc1/workspace/Hope_v11/hope_training/whole_body_tracking
source setup_train_env.sh

RUN=/absolute/path/to/logs/rsl_rl/.../<run>
bash scripts/export_onnx_hitter_pure.sh "$RUN" model_N.pt
```

For an ONNX that already exists, generate only the YAML:

```text
python3 scripts/export_deploy_cfg.py \
  --run-dir "$RUN" \
  --policy "$RUN/exported/policy.onnx"
```

Do not hand-edit the generated YAML.  Re-export it from the frozen run-local
`params/env.yaml` and ONNX metadata.

## 2. Validate without copying or changing source config

Run in Distrobox `hope`:

```text
distrobox enter hope -- bash -lc '
cd /home/dongc1/workspace/Hope_v11/a3_deploy/a3_deploy_example
bash scripts/sync_pingpong_model.sh /absolute/path/to/<run>
'
```

This verifies the ONNX SHA256 recorded by `deploy.yaml` and runs C++/Python
parity.  It does not copy the model into `assets/` or modify a runtime YAML.

## 3. Build a self-contained x86 package

```text
distrobox enter hope -- bash -lc '
cd /home/dongc1/workspace/Hope_v11/a3_deploy/a3_deploy_example
bash scripts/build_a3_deploy_pkg.sh \
  --arch x86_64 \
  --runtime-cfg src/a3/a3_deploy_onnx_ref/config/a3_runtime_config.pingpong.hitter_pingpong.yaml \
  --policy-dir /absolute/path/to/<run> \
  --jobs "$(nproc)"
'
```

The package contains:

```text
dist/a3_deploy_x86_64/
├── a3_deploy_onnx_ref_pingpong
├── libonnxruntime.so*
├── policy/exported/policy.onnx
├── policy/params/deploy.yaml
└── config/a3_runtime_config.pingpong.hitter_pingpong.yaml
```

The packaged runtime config contains only `policy_dir: policy` plus transport
and localization fields.  The runner verifies the YAML/ONNX hash and requires
their joint, observation, action, gain, limit, and contract values to agree
before it can start.

## 4. Relink and install the official AGI MuJoCo path

Never validate a new runner against the historical, Git-tracked
`build_v11/install`. Build into the ignored
`cmake-build-hitter-pingpong-v11` directory and install inside Distrobox
`hope`:

```text
distrobox enter hope -- bash -lc '
cd /home/dongc1/workspace/Hope_v11/a3_deploy/A3_MuJoCo_Sim/aimrt_mujoco_sim
source /opt/ros/jazzy/setup.bash
set -euo pipefail
cmake -S . -B cmake-build-hitter-pingpong-v11 \
  -DCMAKE_BUILD_TYPE=Release \
  -DAIMRT_MUJOCO_SIM_INSTALL=ON \
  -DCMAKE_INSTALL_PREFIX="$PWD/cmake-build-hitter-pingpong-v11/install" \
  -DAIMRT_MUJOCO_SIM_BUILD_WITH_ROS2=ON \
  -DAIMRT_MUJOCO_SIM_BUILD_TESTS=OFF \
  -DAIMRT_MUJOCO_SIM_BUILD_EXAMPLES=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3
cmake --build cmake-build-hitter-pingpong-v11 --config Release --target install --parallel "$(nproc)"
'
```

This target must relink `libmujoco_sim_pkg.so` and reinstall the model/config
files used by Gate3.  Merely recompiling a source object is not enough.

## 5. Contact test before Gate3

Compile and run `scripts/pp_scripted_racket_contact_ab.cc` against the newly
installed MuJoCo.  The gate is PASS only when every explicit-map case has one
rising-edge contact, separates from the racket, matches the discrete impulse
map, remains finite, and reports an outgoing speed no greater than `8 m/s`.

```text
distrobox enter hope -- bash -lc '
set -euo pipefail
SIM=/home/dongc1/workspace/Hope_v11/a3_deploy/A3_MuJoCo_Sim/aimrt_mujoco_sim
OUT=/tmp/pp_scripted_racket_contact_ab
cd "$SIM"
g++ -std=c++20 -O2 \
  -I"$SIM/cmake-build-hitter-pingpong-v11/install/include" \
  -I"$SIM/src/module/mujoco_sim_module" \
  "$SIM/scripts/pp_scripted_racket_contact_ab.cc" \
  -L"$SIM/cmake-build-hitter-pingpong-v11/install/lib" \
  -Wl,-rpath,"$SIM/cmake-build-hitter-pingpong-v11/install/lib" \
  -lmujoco -o "$OUT"
"$OUT" | tee /tmp/pp_scripted_racket_contact_ab.json
python3 - <<"PY"
import json
report = json.load(open("/tmp/pp_scripted_racket_contact_ab.json"))
assert report["explicit_contact_energy_pass"] is True
print("explicit outgoing speeds (m/s):", [
    row["outgoing_speed_mps"] for row in report["results"]
    if row["mode"] == "explicit_map"
])
PY
'
```

The test must be compiled after the install step.  Reusing an old diagnostic
binary does not prove that the current MuJoCo headers/libraries were selected.

Full Gate3 is forbidden if this test fails.  When it passes, run the sole formal
entrypoint, again inside `hope`:

Gate3 and the HDU production launcher use `hope_planner_cpp_node`. Build and
test it from the same ROS workspace before running Gate3:

```text
distrobox enter hope -- bash -lc '
source /opt/ros/jazzy/setup.bash
cd /home/dongc1/workspace/Hope_v11/hope_ws
colcon build --symlink-install --packages-select hope_planner_cpp \
  --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select hope_planner_cpp
colcon test-result --test-result-base build/hope_planner_cpp --verbose
'
```

The formal runtime chain is raw mocap → calibrated schema-2 base relay →
`hope_planner_cpp_node` (post-net one-shot, net crossing + 50 ms) → C++ runner.
Gate3 creates its Planner receipt from the live C++ debug CSV after the
physical serves. It does not run a Python Planner preflight or compare against
a Python physics reimplementation. The obsolete Gate3 Python envelope
entrypoint and its tests were removed so it cannot silently return to the
certification path.

```text
distrobox enter hope -- bash -lc '
cd /home/dongc1/workspace/Hope_v11/a3_deploy/a3_deploy_example
bash scripts/pp_gate3_hitter_pingpong.sh
'
```

Gate3 is an evaluation, not a repair loop.  Preserve its complete evidence and
report stability, recovery, strike completion, racket contact, outgoing ball
state, net clearance, legal landing, and q_des/actual-q audits separately.

## Non-negotiable rules

- Do not hand-copy an ONNX into `dist/`.
- Do not add a checkpoint-specific `model_path` to runtime YAML.
- Do not change C++ for each checkpoint.
- Do not claim MuJoCo/hardware parity from formula similarity alone: the exact
  YAML, ONNX hash, binary, installed simulator, and trace must be bound together.
- Do not run full Gate3 until the freshly linked fixed-racket contact test passes.

The 2026-08-03 R4/model-21400 execution receipt and complete traces are under
`logs/eval_deterministic/hitter_pingpong_r4_unitree_deploy_20260803` inside the
frozen R4 run directory.  That receipt records a passing fixed-contact gate and
a failing full Gate3; a physical-ball PASS must not hide a runner/smoothness
failure.
