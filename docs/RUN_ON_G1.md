# Running HOPE on the Unitree G1

The Unitree G1 (29 DOF) is supported in parallel with the Agibot A3. The G1 is structurally the A3
minus its two passive neck joints, so the same task machinery drives both — only the DOF count
(29 vs 31), the body names (lowercase `_link`, root `pelvis`), and the racket mount differ. The
A3 path is unchanged; everything G1 lives in new files.

| Piece | G1 location |
|-------|-------------|
| Robot articulation cfg | [`robots/g1.py`](../hope_training/whole_body_tracking/source/whole_body_tracking/whole_body_tracking/robots/g1.py) |
| Canonical joint order (29) | [`hope_training/config/joint_order_unitree_g1.yaml`](../hope_training/config/joint_order_unitree_g1.yaml) |
| Task cfg (tracking) | `tasks/tracking/config/unitree_g1/` → `HOPE-PingPong-UnitreeG1-v0` |
| Task cfg (table-tennis scene) | `tasks/table_tennis/config/unitree_g1/` → `HOPE-TableTennis-UnitreeG1-v0` |
| Launcher task YAML | [`cfg/task/HOPEPingPongG1.yaml`](../hope_training/whole_body_tracking/cfg/task/HOPEPingPongG1.yaml) (`task=HOPEPingPongG1`) |
| Placeholder motion generator | [`scripts/make_g1_placeholder_motions.py`](../hope_training/whole_body_tracking/scripts/make_g1_placeholder_motions.py) |
| Deploy contract + reference runner | [`g1_deploy/g1_deploy_example/`](../g1_deploy/g1_deploy_example/) |
| Shared action adapter (train + deploy) | [`g1_deploy/g1_deploy_example/config/action_adapter.yaml`](../g1_deploy/g1_deploy_example/config/action_adapter.yaml) |

## 1. Point at the G1 USD asset

The training env spawns a **pre-converted USD** (the one made from
`g1_with_racket_adapter_short_ball_throwing.urdf`). The default path in `robots/g1.py` points at the
TTRL asset tree (note the space in the directory name is part of the real path). Override it with an
env var if you keep the asset elsewhere:

```bash
export HOPE_G1_USD_PATH="/abs/path/to/g1_with_racket_adapter_short _ball_throwing/g1_with_racket_adapter_short _ball_throwing.usd"
```

Copy the **whole** `g1_with_racket_adapter_short _ball_throwing/` directory if you relocate it — the
top-level `.usd` references `configuration/*.usd` by relative path.

## 2. Generate the placeholder motion clips

The shipped A3 clips are 31-DOF and cannot be reused. Generate 29-DOF G1 placeholders (numpy-only
URDF forward kinematics — no Isaac needed):

```bash
python3 hope_training/whole_body_tracking/scripts/make_g1_placeholder_motions.py
# writes hope_training/motions/preprocessed/hope_g1_{forehand,backhand}.npz
```

These are **non-performant placeholders** (a gentle synthetic swing) so training imports and shape
checks pass. Replace them with real retargeted G1 swings before training a policy you deploy (see
[REPLACE_MOTIONS.md](REPLACE_MOTIONS.md)).

## 3. Train → export

```bash
cd hope_training/whole_body_tracking
source setup_train_env.sh
python scripts/train.py task=HOPEPingPongG1 algo=ppo headless=true
python scripts/export_onnx.py --task HOPE-PingPong-UnitreeG1-v0 \
  --checkpoint logs/rsl_rl/hope_pingpong_g1/<run>/model_<iter>.pt \
  --onnx-name hope_pingpong_g1.onnx \
  --motion-file   hope_training/motions/preprocessed/hope_g1_forehand.npz \
  --motion-file-2 hope_training/motions/preprocessed/hope_g1_backhand.npz
```

`train.py` runs a **joint-order gate** on startup: it prints the articulation's actual Isaac
enumeration and fails if it differs from `joint_order_unitree_g1.yaml`. If it differs, paste the
printed order into that one YAML (it is the single source of truth) and re-run. The export produces
a `105 -> 29` ONNX with the G1 `joint_order` embedded in the metadata.

## 4. Deploy contract

The deploy reference runner is the G1 twin of the A3 one:

```bash
cp logs/rsl_rl/hope_pingpong_g1/<run>/exported/hope_pingpong_g1.onnx g1_deploy/g1_deploy_example/models/
cd g1_deploy/g1_deploy_example
# NOTE: a G1 MuJoCo MJCF is not shipped yet (see below) — pass --model-xml your own G1 model.
bash scripts/run_pingpong_sim.sh --model-xml /path/to/g1_pingpong.xml --view --realtime
```

Training and the deploy runner read the **same** `action_adapter.yaml`, so the raw-action → joint
targets transform is identical (asserted by `tests/test_g1_contract.py`).

## Tuning checklist (before real deployment)

- **Racket mount FK** — `G1_MOUNT_OFFSET` / `G1_MOUNT_QUAT` in `robots/g1.py` are seeds from the
  URDF joint (`xyz=(0.1485,0,0)`, `rpy=(π/2,0,0)`); tune them in the Isaac/MuJoCo viewer so the
  racket-target reward tracks the true paddle center. The URDF itself flags the mount pose as
  provisional.
- **Actuator gains** — `robots/g1.py` carries the real motor-derived G1 gains (armature-based,
  overdamped ζ=2). Retune if your hardware differs.
- **Default stance / clamp** — `action_adapter.yaml` `default_q` is a ready stance and
  `joint_position_clamp` uses the URDF mechanical limits; adjust to taste (one file, read by both
  training and deploy).

## Deferred follow-up

The MuJoCo **real-ball evaluator** needs a G1 MJCF with a racket collision geom + racket site (plus
a `pelvis` free-joint and a gyro sensor for the sim bridge). You can adapt one from
`TTRL-ICRA2026/Beyondmimic_Deploy_G1/mjmodel.xml`. Until then, `mujoco_eval_onnx.py` and the deploy
sim bridge remain A3-only.
