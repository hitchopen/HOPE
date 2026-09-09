# A3 MuJoCo Serve Application

This folder is a self-contained engineering workflow for deterministic Agibot
A3 ping-pong serves. It starts with measured ball/table/racket physics, computes
a legal serve in MuJoCo, converts the resulting Cartesian racket motion to A3
right-arm joint angles with damped-least-squares (DLS) inverse kinematics,
exports every SDK CSV joint column, and packages that CSV with the validated
PR #18 high-level A3 arm runtime.

No strike joint pose is guessed or hand-authored in the generator. The retained
CSV under `assets/validated/` is the exact PR #18 reference artifact and remains
the default deployment input. The project developer confirms that this exact
PR #18 application was fully tested on A3 and is executable and safe. Its
[demo video](assets/validated/pr18_a3_serve_demo.mp4) is included as a reference
source.

## End-to-end workflow

```text
serve.json
  target bounces + measured physics
        |
        v
official A3 MuJoCo MJCF + racket collision mesh
  outgoing-ball grid search under drag, gravity, table bounce and net contact
        |
        v
racket impact velocity + two-sided face plane normal
  moving-plane restitution inversion
        |
        v
Cartesian ready / acceleration / impact / follow-through / return path
        |
        v
DLS IK using MuJoCo site Jacobians
  exact A3 right-arm radians, high-level limits enforced
        |
        v
full MuJoCo joint replay
  racket contact + first own-half bounce + net clearance + opponent-half bounce
        |
        v
37-column, 200 Hz A3 SDK CSV + SHA-256 manifest + validation report
        |
        v
native MDU CMake build
  PR #18 14-arm /motion/control/arm_joint_command runtime
        |
        v
read-only preflight -> guarded motion_player handoff -> A3 execution
```

The vendor controller retains ownership of the waist, neck and legs. The app
publishes only the 14 high-level arm positions. The left-wrist-roll handling,
process ownership handoff, operator trigger, and restoration logic are retained
from PR #18.

## Layout

| Path | Purpose |
|---|---|
| `a3_serve/` | Physics search, SO(3) math, DLS IK, replay, CSV export and CLI. |
| `config/serve.json` | Reproducible physics, targets, timing, IK and validation gates. |
| `config/approved_motions.json` | Source-controlled hardware approvals keyed by CSV SHA-256. |
| `assets/validated/` | Fully A3-tested PR #18 CSV, runtime contract and demo reference video. |
| `runtime/` | Native-MDU C++ high-level arm runner, guarded wrapper and build script. |
| `tests/` | Portable solver/export/runtime tests plus an optional MuJoCo model smoke test. |

The application references the shared official A3 model at
`a3_deploy/A3_MuJoCo_Sim/.../a3_pingpong.xml`. It does not copy the vendor
meshes into this folder. The model already contains the `right_racket` site and
`right_racket_collision` mesh; the racket-local `+Y` axis is the face normal.
Because the collision face is two-sided, the IK layer selects `+normal` or
`-normal` whichever requires less rotation from the A3 READY pose.

The current physics contract is deliberately **no-spin**. Its measured drag
and contact coefficients come from the repository's
`configs/ball_physics.yaml`; the generated manifest records that file's hash.
This workflow does not claim to model Magnus lift or spin-dependent racket and
table contacts.

## Local setup and generation

This is an offline host-Python workflow, not an Isaac, ROS, or robot runtime.
On a new workstation, first complete the host prerequisites in
[`docs/DISTROBOX_SETUP.md`](../../docs/DISTROBOX_SETUP.md), then use the
isolated virtual environment below from this directory:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[test]'

hope-a3-serve inspect --config config/serve.json
pytest -q
hope-a3-serve generate \
  --config config/serve.json \
  --output build/generated/default
```

The output directory contains:

- `serve_policy.csv`: all six root and all 31 joint columns at 200 Hz;
- `serve_vendor_arm_manifest.json`: model/config/template/output hashes and
  planner/IK/replay evidence;
- `validation_report.json`: search, IK, collision and legal-serve metrics.

Generation is offline and does not create a ROS publisher or send a robot
command. If no legal candidate or converged/collision-free IK replay is found,
the command exits without producing a deployable artifact.

With MuJoCo 3.2.7, the checked-in default configuration currently produces a
complete 3,878-row CSV with these replay gates satisfied:

- racket contact observed and no net contact;
- own-side bounce at approximately `(1.010, -0.812)` m;
- opponent-side bounce at approximately `(2.049, -0.719)` m;
- approximately `0.308` m ball-centre height over the net plane;
- zero robot self-collisions at the configured 10-frame scan stride;
- zero unconverged IK frames and a `4.36 rad/s` peak right-arm joint speed,
  below the configured `5.2 rad/s` source limit.

These are reproducibility checks, not hardware-safety evidence. The command
recomputes them and stores the exact values in `validation_report.json`.

## Build the executable A3 application on the MDU

The final C++ binary must be built in the installed A3 ROS 2 environment:

```bash
cd /path/to/HOPE/apps/a3_mujoco_serve
source /agibot/software/v0/entry/env/env.sh
bash runtime/scripts/build_a3_app.sh \
  --motion build/generated/default/serve_policy.csv \
  --manifest build/generated/default/serve_vendor_arm_manifest.json \
  --jobs 2
```

For the A3-validated PR #18 reference, omit `--motion` and `--manifest`; the
build defaults to `assets/validated/`. The result is
`dist/a3_serve_vendor_arm/`. Its package manifest binds the executable,
runtime scripts, CSV, motion manifest, fixed-profile qualification record and
approval registry by SHA-256. The default PR #18 CSV is reported as `approved`
and real modes remain enabled. A different CSV can pass the fixed safety
profile and be packaged as a `candidate`, but real modes remain disabled until
its SHA-256 receives a reviewed approval-registry entry. The runtime pins the
reviewed registry hash, so changing the CSV and its adjacent manifest together
cannot manufacture hardware approval.

Check the complete input qualification without compiling the AArch64
executable:

```bash
bash runtime/scripts/build_a3_app.sh --verify-inputs-only
```

Run the packaged app using the procedure in [runtime/README.md](runtime/README.md).
The default invocation is read-only preflight. Real modes retain PR #18's
literal `--confirm-real-commands` and interactive operator requirements.

## Generalize to another serve direction

Copy `config/serve.json`, then change only the task-level inputs at first:

| Setting | Meaning |
|---|---|
| `first_bounce_target_table` | Desired own-side ball-centre contact, in the table frame. |
| `second_bounce_target_table` | Desired opponent-side contact; change its lateral coordinate to aim left/right. |
| `speed_m_s`, `elevation_deg`, `azimuth_deg` | Candidate outgoing-ball grid searched in MuJoCo. |
| `max_racket_speed_m_s` | Planning bound; the separate joint-speed gate still applies after IK. |

The table frame is right-handed: `+x` points toward the opponent, `+y` points
left, and the playing surface is `z = 0`. The default targets use negative `y`
because the table spans `y in [-width, 0]`. Then rerun `generate`; the impact
inversion, DLS IK, collision scan and full MuJoCo replay all recompute from the
new inputs.

Do not casually change `source_hz`, frame count, READY/stroke/strike frames,
CSV schema, joint order, command topic or the 100 Hz runtime stride. Those are
part of the PR #18 runtime contract; changing one requires coordinated runtime
work and renewed qualification.

A new generated CSV is a new motion application. It inherits the checked
transport and handoff implementation, but not the PR #18 trajectory's hardware
validation. The builder first applies the non-relaxable
`a3_high_level_arm_v1` envelope. A passing external artifact is a read-only
`candidate`; after robot qualification, add its hash and approval scope to the
source-controlled registry through review to enable real execution.

## Scope and licensing

New Python workflow code is contributed under the repository's Apache-2.0
license. The official A3 MuJoCo package is Mulan PSL v2. The retained runtime
source carries its original AgiBot copyright header and terms. See the root
`THIRD_PARTY_NOTICES.md`; placing these components in one application folder
does not relicense vendor material.
