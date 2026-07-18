# Third-party notices

HOPE PingPong is licensed under Apache-2.0 (see [LICENSE](LICENSE)). It redistributes the
third-party materials listed below, each under its own license. This file covers only material
that is actually included in this repository.

---

## whole_body_tracking training framework (BeyondMimic)

- Location: `hope_training/whole_body_tracking/`
- License: MIT
- Copyright: Copyright (c) 2024, The Isaac Lab Project Developers
- Origin: Derived from BeyondMimic (`HybridRobotics/whole_body_tracking`), an Isaac Lab
  motion-tracking reinforcement-learning extension.

The HOPE PingPong training package is a fork/derivative of the BeyondMimic
`whole_body_tracking` project, adapted to the table-tennis task. The upstream MIT license text
is retained in `hope_training/whole_body_tracking/LICENCE`. The SMPL-X → robot retargeting and
video → SMPL-X extraction stages referenced in the docs use GMR (`YanjieZe/GMR`, MIT) and GVHMR
respectively; those tools are not redistributed here.

---

## vrpn_mocap

- Location: `hope_ws/src/vrpn_mocap/`
- License: MIT
- Copyright: Copyright (c) 2022 Alvin Sun
- Origin: VRPN motion-capture client for ROS 2 (ChingMu VRPN ROS 2 plugin).

The MIT license text is retained in `hope_ws/src/vrpn_mocap/LICENSE`. This package is vendored
so the planner can be brought up against a VRPN motion-capture server; it is otherwise
unmodified except for documentation links.

---

## AimRT MuJoCo simulation

- Location: `a3_deploy/A3_MuJoCo_Sim/aimrt_mujoco_sim/`
- License: Mulan Permissive Software License, Version 2 (Mulan PSL v2)
- Origin: AgiBot AimRT-based MuJoCo simulation.

The Mulan PSL v2 license text is retained in
`a3_deploy/A3_MuJoCo_Sim/aimrt_mujoco_sim/LICENSE`. As published, this package bundles:

- **`joint_msgs`** (AgiBot ROS/AimRT message definitions), distributed under the same Mulan
  PSL v2 license as part of the simulation package.
- The **`a3_pingpong` MuJoCo model and meshes**, which are AgiBot A3 robot CAD geometry
  distributed as part of this Mulan-licensed simulation package. This is the runnable robot
  model shipped with the repository. The equivalent standalone URDF/meshes are **not**
  redistributed here (see `a3_deploy/URDF/README.md`).

---

## Table + net USD asset

- Location: `hope_training/whole_body_tracking/source/whole_body_tracking/whole_body_tracking/tasks/table_tennis/table_usd/`
- License: MIT
- Copyright: Copyright (c) 2026 purdue-tracelab
- Origin: A table-tennis table + net USD mesh used to build the training scene.

The MIT license text is retained alongside the asset in
`table_usd/LICENSE-PACE-ICRA2026-MIT.txt`.

---

## Agibot A3 materials (`agibot/`)

- Location: `agibot/`
- Origin: Agibot A3 robot deployment example, URDF/meshes, AimRT MuJoCo simulation, and
  open-source hardware add-ons.

The `agibot/` tree contains Agibot A3 robot materials, each component under its own terms:

- `agibot/A3_MuJoCo_Sim/aimrt_mujoco_sim/` — the AimRT MuJoCo simulation, under the **Mulan
  Permissive Software License, Version 2 (Mulan PSL v2)**, retained in its `LICENSE`.
- `agibot/code_deployment/` — the A3 deploy example. It bundles third-party runtime SDKs (for
  example Unitree SDK2, ONNX Runtime, RKNN runtime, RapidJSON), each governed by its own license
  retained in-tree alongside the respective component. The Agibot-authored deploy sources carry
  `Agibot Inc.` copyright headers and are included under Agibot's terms.
- `agibot/pku/` — open-source hardware (hip marker shell, wrist racket adapter); see its README.
- `agibot/URDF/` — Agibot A3 URDF and meshes, carrying Agibot's copyright.

---

## Runtime dependencies (not redistributed)

The training, planner, evaluation, and reference-runner code depend on third-party Python and
ROS 2 packages (for example NumPy, PyYAML, PyTorch, Isaac Lab, ONNX Runtime, MuJoCo, rclpy).
These are installed from their own distribution channels and are **not** redistributed in this
repository; each remains under its own license. See the per-component `requirements*.txt` and
`package.xml` files.
