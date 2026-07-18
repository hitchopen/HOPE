# A3 URDF / meshes — optional user-supplied copy

The Agibot A3 URDF description and its mesh assets (`A3T2.5-URDF-std-pingpang`) are
the robot vendor's hardware CAD-derived geometry and carry **no open-source
license**. This repository ships the **Agibot-provided** package with the starter
under `agibot/URDF/A3T2.5-URDF-std-pingpang/` (see `A3_ASSETS.md` at the repo
root), and the asset-prep tooling uses that copy by default.

This directory is an **optional override location**: if you prefer (or are
required) to use your own vendor-supplied copy of the A3 model instead of the
bundled one, place it here and point the tooling at it with `--source-root`.

## What you provide (optional)

Obtain the `A3T2.5-URDF-std-pingpang` package from your Agibot A3 robot vendor /
robot support materials and place it here so the tree looks like:

```
a3_deploy/URDF/
  README.md                          # this file (the only file shipped here)
  A3T2.5-URDF-std-pingpang/
    urdf/
      URDF-JOINT-LINK.urdf           # the URDF the asset-prep step reads
    meshes/
      *.STL                          # visual meshes referenced by the URDF
    config/
      joint_names_*.yaml             # joint name list (optional)
    package.xml                      # ROS package wrapper (optional)
    CMakeLists.txt                   # ROS package wrapper (optional)
    launch/
      *.launch                       # display/gazebo helpers (optional)
```

The exact folder name (`A3T2.5-URDF-std-pingpang`) and the URDF filename are what
the tooling looks for; keep them as your vendor provides, or update the paths where
you invoke the tools.

## What consumes it

- **Training asset preparation.** The training package's asset-prep step
  (`prepare_a3_isaac_asset.py`) reads a source URDF package and builds the Isaac
  Lab robot asset. By default it uses the bundled
  `agibot/URDF/A3T2.5-URDF-std-pingpang/`; to use the copy you place HERE
  instead, pass `--source-root a3_deploy/URDF/<your_a3_package>`.
- **The MuJoCo sim.** `a3_deploy/A3_MuJoCo_Sim` uses its own `a3_pingpong`
  MJCF and a bundled copy of the robot meshes, so it runs **without** this URDF
  directory. Note, however, that the sim's bundled STL meshes are the **same**
  physical-robot assets as the URDF meshes here — see the licensing note in the
  MuJoCo sim README if you fork or redistribute that package.

## Joint order

Whatever URDF you place here must expose the 31 controllable joints in the
canonical order used across training, export, the reference runner, and the
planner. That order is defined in
`../a3_deploy_example/reference/a3_deploy_onnx_ref_pingpong/joint_order.py`
(waist 3, neck 2, left arm 7, right arm 7, left leg 6, right leg 6). The racket is
mounted on the right wrist.
