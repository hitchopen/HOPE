## HOPE Agibot A3 Ping-Pong (this repo)

Status: Partial — A3 training here demonstrates pipeline viability, NOT an accepted quality baseline.

This package is the BeyondMimic motion-tracking trainer (upstream G1 docs below), extended in HOPE
to train an [Agibot A3](../../agibot/) (31 actuated DOF) ping-pong swing policy. Unlike the
upstream `argparse` entries, HOPE drives training through **Hydra** entry points:

- `scripts/train.py` and `scripts/play.py` with `task=HitterPingPong algo=ppo`.
- The `HitterPingPong` task maps to the gym task `HOPE-HitterPingPong-AgibotA3-v0` (`experiment_name agibot_a3_hitter_pingpong`).
- Overrides are layered from the `cfg/` tree: `cfg/task` (env/task), `cfg/algo` (PPO), `cfg/base` (shared defaults).
- `HitterPingPong` trains one unified HITTER-style policy: clip 0 = forehand, clip 1 = backhand, on the 110-D `hitter_pure` actor contract (no swing-side observation — the side is inferred outside the policy).
- Local video-generated `.npz` clips are first-class inputs: pass `motion_file=...` plus `motion_file_2=...`; the HOPE launcher does not require a remote artifact registry.

**The authoritative guides are [docs/TRAIN_POLICY.md](../../docs/TRAIN_POLICY.md) and
[QUICKSTART_A3_ISAAC.md](../../QUICKSTART_A3_ISAAC.md).**
A from-scratch Isaac Sim/Lab install is out of scope here — follow the upstream
[Isaac Lab installation guide](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html).

### HOPE environment (GPU/Isaac box)

- Isaac Sim 4.5.0, Isaac Lab 2.1.0, Python 3.10, NVIDIA CUDA GPU; `rsl_rl` comes via Isaac Lab.
- Install into the Isaac Lab python: `python -m pip install -e source/whole_body_tracking`.
- Extra pip deps NOT in `setup.py` `install_requires` (must be importable in the Isaac Lab python):
  `hydra` and `omegaconf`.
- `source setup_train_env.sh` (must be **sourced**, in the GPU/Isaac shell) to get the `hope_isaac_py`
  launcher. Edit its site-specific paths, or provide a (git-ignored)
  `setup_train_env.local.sh` override that it auto-sources.

### A3 asset and motions

- The A3 ping-pong URDF lives at
  `source/whole_body_tracking/whole_body_tracking/assets/agibot_a3/urdf/model.urdf`, generated from
  `agibot/URDF/A3T2.5-URDF-std-pingpang/` with `python3 scripts/prepare_a3_isaac_asset.py --force`
  (the script ships in this package's `scripts/`).
- Motion flow: GVHMR (video → SMPL-X) → GMR (`--robot agibot_a3`; the default robot is `g1`, A3 NEEDS
  `--robot agibot_a3`) → convert the retargeted trajectory into the documented `.npz` + `.yaml`
  schema ([docs/REPLACE_MOTIONS.md](../../docs/REPLACE_MOTIONS.md)) → train directly with
  `motion_file=...` / `motion_file_2=...`.
- `HitterPingPong.yaml` pins the recipe: the `hitter_pure` racket-target mode,
  forehand/backhand-conditioned target ranges, per-clip strike phases, and the continuous-rally
  settings. Keep local clip order aligned with `strike_phase_per_clip`: `motion_file` = forehand,
  `motion_file_2` = backhand.
- Train against forehand/backhand clips expressed in the corrected **HOPE +X** world frame
  ([docs/REPLACE_MOTIONS.md](../../docs/REPLACE_MOTIONS.md)).
- `max_iterations` defaults to a train-forever sentinel — pass `max_iterations=` on the CLI and stop
  manually when `strike_success` plateaus.

---

# BeyondMimic Motion Tracking Code

> The sections below are the **upstream BeyondMimic (Unitree G1) baseline** documentation, retained
> for reference. For the HOPE Agibot A3 ping-pong workflow, use the section above plus
> `docs/TRAIN_POLICY.md`; the HOPE path consumes local `.npz` files. The upstream
> helper/entry scripts referenced below
> (`csv_to_npz.py`, `replay_npz.py`, `scripts/rsl_rl/*`) ship in the **upstream repository**,
> not in this fork — produce HOPE clips per `docs/REPLACE_MOTIONS.md` instead.

[![IsaacSim](https://img.shields.io/badge/IsaacSim-4.5.0-silver.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
[![Isaac Lab](https://img.shields.io/badge/IsaacLab-2.1.0-silver)](https://isaac-sim.github.io/IsaacLab)
[![Python](https://img.shields.io/badge/python-3.10-blue.svg)](https://docs.python.org/3/whatsnew/3.10.html)
[![Linux platform](https://img.shields.io/badge/platform-linux--64-orange.svg)](https://releases.ubuntu.com/20.04/)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit&logoColor=white)](https://pre-commit.com/)
[![License](https://img.shields.io/badge/license-MIT-yellow.svg)](https://opensource.org/license/mit)

[[Website]](https://beyondmimic.github.io/)
[[Arxiv]](https://arxiv.org/abs/2508.08241)
[[Video]](https://youtu.be/RS_MtKVIAzY)

## Overview

BeyondMimic is a versatile humanoid control framework that provides highly dynamic motion tracking with the
state-of-the-art motion quality on real-world deployment and steerable test-time control with guided diffusion-based
controllers.

This repo covers the motion tracking training in BeyondMimic. **You should be able to
train any sim-to-real-ready motion in the LAFAN1 dataset, without tuning any parameters**.

For sim-to-sim and sim-to-real deployment, please refer to
the [motion_tracking_controller](https://github.com/HybridRobotics/motion_tracking_controller).

### Alternative Implementations

- There is an alternative reproduction of BeyondMimic in [mjlab](https://github.com/mujocolab/mjlab), a new Isaac Lab-style manager API powered by MuJoCo-Warp for RL and robotics research. See the implementation [here](https://github.com/mujocolab/mjlab/blob/main/src/mjlab/tasks/tracking/tracking_env_cfg.py).

## Installation

- Install Isaac Lab v2.1.0 by following
  the [installation guide](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html). We recommend
  using the conda installation as it simplifies calling Python scripts from the terminal.

- Clone this repository separately from the Isaac Lab installation (i.e., outside the `IsaacLab` directory):

```bash
# Option 1: SSH
git clone git@github.com:HybridRobotics/whole_body_tracking.git

# Option 2: HTTPS
git clone https://github.com/HybridRobotics/whole_body_tracking.git
```

- Pull the robot description files from GCS

```bash
# Enter the repository
cd whole_body_tracking
# Rename all occurrences of whole_body_tracking (in files/directories) to your_fancy_extension_name
curl -L -o unitree_description.tar.gz https://storage.googleapis.com/qiayuanl_robot_descriptions/unitree_description.tar.gz && \
tar -xzf unitree_description.tar.gz -C source/whole_body_tracking/whole_body_tracking/assets/ && \
rm unitree_description.tar.gz
```

- Using a Python interpreter that has Isaac Lab installed, install the library

```bash
python -m pip install -e source/whole_body_tracking
```

## Motion Tracking

### Motion Preprocessing & Registry Setup

In order to manage the large set of motions we used in this work, we leverage the WandB registry to store and load
reference motions automatically.
Note: The reference motion should be retargeted and use generalized coordinates only.

- Gather the reference motion datasets (please follow the original licenses), we use the same convention as .csv of
  Unitree's dataset

    - Unitree-retargeted LAFAN1 Dataset is available
      on [HuggingFace](https://huggingface.co/datasets/lvhaidong/LAFAN1_Retargeting_Dataset)
    - Sidekicks are from [KungfuBot](https://kungfu-bot.github.io/)
    - Christiano Ronaldo celebration is from [ASAP](https://github.com/LeCAR-Lab/ASAP).
    - Balance motions are from [HuB](https://hub-robot.github.io/)


- Log in to your WandB account; access Registry under Core on the left. Create a new registry collection with the name "
  Motions" and artifact type "All Types".


- Convert retargeted motions to include the maximum coordinates information (body pose, body velocity, and body
  acceleration) via forward kinematics,

```bash
python scripts/csv_to_npz.py --input_file {motion_name}.csv --input_fps 30 \
    --output_file ./motions/{motion_name}.npz --output_name {motion_name} --headless
```

This writes the processed motion file and registers it with the WandB registry under the output name {motion_name}.

- Test if the WandB registry works properly by replaying the motion in Isaac Sim:

```bash
python scripts/replay_npz.py --registry_name={your-organization}-org/wandb-registry-motions/{motion_name}
```

- Debugging
    - Make sure to export WANDB_ENTITY to your organization name, not your personal username.

### Policy Training

- Train policy by the following command:

```bash
python scripts/rsl_rl/train.py --task=Tracking-Flat-G1-v0 \
--registry_name {your-organization}-org/wandb-registry-motions/{motion_name} \
--headless --logger wandb --log_project_name {project_name} --run_name {run_name}
```

### Policy Evaluation

- Play the trained policy by the following command:

```bash
python scripts/rsl_rl/play.py --task=Tracking-Flat-G1-v0 --num_envs=2 --wandb_path={wandb-run-path}
```

The WandB run path can be located in the run overview. It follows the format {your_organization}/{project_name}/ along
with a unique 8-character identifier. Note that run_name is different from run_path.

## Code Structure

Below is an overview of the code structure for this repository:

- **`source/whole_body_tracking/whole_body_tracking/tasks/tracking/mdp`**
  This directory contains the atomic functions to define the MDP for BeyondMimic. Below is a breakdown of the functions:

    - **`commands.py`**
      Command library to compute relevant variables from the reference motion, current robot state, and error
      computations. This includes pose and velocity error calculation, initial state randomization, and adaptive
      sampling.

    - **`rewards.py`**
      Implements the DeepMimic reward functions and smoothing terms.

    - **`events.py`**
      Implements domain randomization terms.

    - **`observations.py`**
      Implements observation terms for motion tracking and data collection.

    - **`terminations.py`**
      Implements early terminations and timeouts.

- **`source/whole_body_tracking/whole_body_tracking/tasks/tracking/tracking_env_cfg.py`**
  Contains the environment (MDP) hyperparameters configuration for the tracking task.

- **`source/whole_body_tracking/whole_body_tracking/tasks/tracking/config/g1/agents/rsl_rl_ppo_cfg.py`**
  Contains the PPO hyperparameters for the tracking task.

- **`source/whole_body_tracking/whole_body_tracking/robots`**
  Contains robot-specific settings, including armature parameters, joint stiffness/damping calculation, and action scale
  calculation.

- **`scripts`**
  Includes utility scripts for preprocessing motion data, training policies, and evaluating trained policies.

This structure is designed to ensure modularity and ease of navigation for developers expanding the project.
