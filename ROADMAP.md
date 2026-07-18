# Roadmap

Scope and direction for HOPE PingPong. For what the focused rewrite removed and
why, see [`docs/reference/REMOVED_FROM_STARTER.md`](docs/reference/REMOVED_FROM_STARTER.md).

## Shipped

- One unified forehand/backhand policy for the Agibot A3 (31 actuated DOF),
  with automatic per-ball side selection and continuous multi-rally play.
- A fixed station and base heading; legs used only for in-place balance and
  recovery.
- A single no-spin ball model fitted from real data
  ([`configs/ball_physics.yaml`](configs/ball_physics.yaml)), shared by
  training, planner, and eval.
- Isaac Lab + PPO training, actor-only ONNX export, and one public metric,
  `success_rate`.
- Two deploy paths: the clean-room reference runner in
  [`a3_deploy/`](a3_deploy) and Agibot's own example under
  [`agibot/code_deployment/`](agibot/code_deployment).
- A ROS 2 workspace ([`hope_ws/`](hope_ws)) with the planner, `RacketCommand`,
  and the vendored `vrpn_mocap` driver, wired end to end into the reference
  runner via `--planner`.

## Out of scope, by design

Station movement, footstep planning, locomotion, ball spin, motion
retiming/TOPP, opponent adaptation, and shot strategy. Also excluded: internal
shadow/gate/debug/replay machinery, failure checks, and checkpoint promotion.
The shipped motions, rewards, action adapter, side selector, and physics
constants are documented examples meant to be replaced — see
[`docs/EXTENDING_HOPE_PINGPONG.md`](docs/EXTENDING_HOPE_PINGPONG.md).

## Next

- **A motion converter.** [`docs/REPLACE_MOTIONS.md`](docs/REPLACE_MOTIONS.md)
  asks teams to bring their own retargeted clips, but no retargeted-CSV → `.npz`
  tool ships today. This is the clearest gap; revival steps are recorded in
  [`docs/reference/REMOVED_FROM_STARTER.md`](docs/reference/REMOVED_FROM_STARTER.md).
- **Performance-tuned reference motions.** The two clips under
  `hope_training/motions/preprocessed/` are physically-neutral placeholders.
- **Validated reward defaults and training recipes**, so `success_rate` is
  reproducible from a clean clone.
- **Ball and table physics in the reference runner's MuJoCo scene.** The MJCF
  bundled with the deploy runner is robot-only, so that interactive sim path
  validates policy execution and joint control but not rally outcomes. The
  authoritative `success_rate` already comes from
  `hope_training/whole_body_tracking/scripts/mujoco_eval_onnx.py`, which builds
  a full MuJoCo ball + table + net scene around the same robot model
  (`scripts/evaluate.py` is only a fast in-Isaac analytic estimate); this
  roadmap item is about folding that rally physics into the interactive runner
  scene too.
- **A real-robot deployment checklist** with reproduced dry-run, joint-order,
  command-scale, low-gain, e-stop, and safe-halt verification.
- **CI** for the non-Isaac checks, and optional GPU smoke jobs.
