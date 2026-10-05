# HOPE integration

This is the supplied A3 Ultra AimDK 3.1 bundle, consolidated from the old
numbered download directory into `agi/agibot_a3_Ultra_aimdk-dev3.1`.
`protocol/`, `examples/` and `prebuilt/` retain their relative layout.
`hand_command.py` and `hand_state.py` are alongside the existing `grip.py`.
Local password notes and Python bytecode caches are excluded.

The bundle includes its Python wheel and AArch64 ROS 2 protocol libraries.
Vendor examples require their documented ROS 2/Python environment; copying this
folder does not install those dependencies or validate them on another platform.

On `build_6`, the deployment builder reads this directory's `grip.py` as data
and copies it into `tools/`. The gripper bridge validates its hash and extracts
presets without executing that script. Its original bytes are preserved.

Public `main` continues to package
`a3_deploy/a3_deploy_example/scripts/serve_gripper_presets.py` as `tools/grip.py`.
The body Runner uses its existing local protocol sources. Adding this SDK does
not redirect Runner, change command ownership, launch ROS nodes or move hardware.
