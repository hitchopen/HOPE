# Runtime updates: October 11–13, 2026

This public integration starts from `hitchopen/HOPE main` at `4ebeee4d7`
(including the 17-station shell from PR #43). It ports the runtime changes
through `build_6` commit `a85b812dd` into the public `a3_deploy/`, `agibot/`
and `NatNet2ROS2/` layout. The `hope_training` tree and published policy files
are unchanged. Training history and training-only procedures are excluded.

- Default `motive_live` localization accepts the selected named body's live
  definition without a fixed file hash, marker count or reference layout.
  The supplied 17-station body is named `UCB_P1`; measured calibration and
  file-pinned asset modes remain explicit options.
- The NatNet driver applies the venue world transform once at publication.
  Public bounded decoders, the output limiter and body allowlist are retained.
- Distributed recording covers Laptop/HDU/MDU artifacts and asynchronous
  Runner/Planner CSV writes, with export helpers and operator controls.
- Shared fitted ball physics, adaptive flight handling and target-turn guards
  update Planner, packetizer and native MuJoCo/Gate 3 paths.
- Console 1.8.13 includes live asset status and recording controls. Runtime
  fixes cover Motive link-local networking and gripper cold-start timing.

Integration validation built ROS/Planner/NatNet and the x86 Runner, packaged
the Console, and passed the focused Python, Planner, NatNet and ball-contact
tests. The native suite passed 429 tests with four unavailable-policy skips;
its legacy `FK.TestFKAndGlobalVelocities` case requires an absent
`a3/a3_29dof.xml` fixture and was excluded after confirming that failure.
The x86 Runner build used AimRT and ROS backend integration disabled. No ARM
package, full physical-ball Gate 3 qualification or hardware session was run
for this integration, and it makes no new policy-certification claim.

Use [START_HERE.md](START_HERE.md) and the
[deployment guide](operations/deploy_and_gate3.md) for current commands.
