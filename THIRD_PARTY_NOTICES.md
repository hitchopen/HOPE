# Third-party notices

HOPE is licensed under Apache-2.0 (see [LICENSE](LICENSE)). It redistributes the
third-party materials listed below, each under its own license. This file covers only material
that is actually included in this repository.

---

## whole_body_tracking training framework (BeyondMimic)

- Location: `hope_training/whole_body_tracking/`
- License: MIT
- Copyright: Copyright (c) 2024, The Isaac Lab Project Developers
- Origin: Derived from BeyondMimic (`HybridRobotics/whole_body_tracking`), an Isaac Lab
  motion-tracking reinforcement-learning extension.

The HOPE training package is a fork/derivative of the BeyondMimic
`whole_body_tracking` project, adapted to the table-tennis task. The upstream MIT license text
is retained in `hope_training/whole_body_tracking/LICENCE`. The SMPL-X → robot retargeting and
video → SMPL-X extraction stages referenced in the docs use GMR (`YanjieZe/GMR`, MIT) and GVHMR
respectively; those tools are not redistributed here.

---

## vrpn_mocap

- Location: `VRPN2ROS2/src/vrpn_mocap/`
- License: MIT
- Copyright: Copyright (c) 2022 Alvin Sun
- Origin: VRPN motion-capture client for ROS 2 (ChingMu VRPN ROS 2 plugin).

The MIT license text is retained in `VRPN2ROS2/src/vrpn_mocap/LICENSE`. This
package is vendored so the planner can be brought up against a VRPN
motion-capture server. Local changes add HOPE topic/QoS behavior, preservation
and strict validation of the server-provided timestamp, an operational
timestamp probe, tests, and standalone-workspace documentation; see
`VRPN2ROS2/src/vrpn_mocap/PIN.md`.

---

## motion_capture_tracking (IMRCLab)

- Location: `NatNet2ROS2/src/motion_capture_tracking/`, `NatNet2ROS2/src/motion_capture_tracking_interfaces/`
- License: MIT
- Copyright: Copyright (c) 2021 Wolfgang Hönig (submodules: (c) 2016 USC-ACTLab, (c) 2014 whoenig)
- Origin: OptiTrack/NatNet motion-capture driver for ROS 2
  (`IMRCLab/motion_capture_tracking`, v1.0.9), with its `libmotioncapture` and
  `librigidbodytracker` submodules materialized in-tree.

The MIT license text is retained in `NatNet2ROS2/src/motion_capture_tracking/LICENSE` (and in
`deps/libmotioncapture/LICENSE` / `deps/librigidbodytracker/LICENSE`). This package is vendored
so the planner can be brought up against an OptiTrack/Motive rig via the open-source NatNet
depacketizer. Local modifications: non-OptiTrack vendor SDK trees removed, non-OptiTrack
backends disabled, and NatNet unicast/model-definition fixes applied — the complete
provenance and patch list is `NatNet2ROS2/src/motion_capture_tracking/PIN.md`.

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

The high-level A3 arm runner consolidated under
`apps/a3_mujoco_serve/runtime/src/` retains its original `AgiBot Inc.`
copyright header and terms. Consolidation into the HOPE application does not
relicense that source. The new planner, IK, exporter, tests and documentation
around it are HOPE contributions under Apache-2.0.

The developer-supplied PR #18 visual demo is retained at
`apps/a3_mujoco_serve/assets/validated/pr18_a3_serve_demo.mp4` as project
reference material; it is not third-party runtime code.

---

## Runtime dependencies (not redistributed)

The training, planner, evaluation, and reference-runner code depend on third-party Python and
ROS 2 packages (for example NumPy, PyYAML, PyTorch, Isaac Lab, ONNX Runtime, MuJoCo, rclpy).
These are installed from their own distribution channels and are **not** redistributed in this
repository; each remains under its own license. See the per-component `requirements*.txt` and
`package.xml` files.
