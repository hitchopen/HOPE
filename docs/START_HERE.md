# Start here: public runtime

Use the public `a3_deploy/`, `agibot/` and `NatNet2ROS2/` layout.
The October 11–13 runtime updates retain the published model and training tree.

| Work | Entry |
| --- | --- |
| New-machine installation and container selection | [DISTROBOX_SETUP.md](DISTROBOX_SETUP.md) |
| Isaac training, source launcher and motion inputs | [public Isaac quickstart](../QUICKSTART_A3_ISAAC.md) |
| ROS workspace and native Runner builds | [build_and_test.md](operations/build_and_test.md) |
| Complete deployment and native Gate 3 command sequence | [deploy_and_gate3.md](operations/deploy_and_gate3.md) |
| Restore local motions/checkpoints/vendor assets | [setup_local_sync.md](operations/setup_local_sync.md) |
| Supplied ONNX/deploy pair in native MuJoCo Gate 3 | [gate3_two_file_runner.md](operations/gate3_two_file_runner.md) |
| Runner, serving and operator controls | [runtime_xbox.md](operations/runtime_xbox.md), [Foxglove lifecycle](operations/foxglove_lifecycle.md) |
| Hardware session procedure | [run_hardware_session.md](operations/run_hardware_session.md) |
| Policy input/action and joint order | [policy_observation_action.md](interfaces/policy_observation_action.md), [joint order](interfaces/joint_order.md) |
| Simulation/hardware gates and retained evidence | [G06](gates/G06_isaac_to_mujoco.md), [G07](gates/G07_mujoco_to_real.md), [PROGRESS.md](PROGRESS.md) |
| Artifact handling | [asset inventory](../A3_ASSETS.md) |

Use the environment guide for installation, the selected task YAML for the
training recipe, exported metadata for policy IO, and each run's reports for
its result. Dated gate notes retain historical evidence;
their old package paths, commands and verdicts do not describe a new run.
Environment inspection and `--check` do not establish simulator certification
or hardware qualification. Training launch remains an operator action.
