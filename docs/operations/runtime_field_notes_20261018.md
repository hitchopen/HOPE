# A3 runtime field notes — 2026-10-18

These are historical deployment observations preserved from the development
record in PR #53 at commit `c845e5ccdfec5f2e253f331127212a73df1743de`.
They are not new tests performed while separating the pull requests, current
robot status, or a deployment recipe. The original record used the `build_6`
layout; this repository's Runner/simulator sources are under `a3_deploy/`.
Evidence archives named below were local deployment artifacts, not files
distributed in this repository.

For current installation and operation use [Foxglove lifecycle](foxglove_lifecycle.md)
and [deployment and Gate 3](deploy_and_gate3.md). The field v20 CSV has its own
hash, runtime prerequisites and validation limits in the
[asset manifest](../../a3_deploy/a3_deploy_example/assets/a3_runtime/serve/motions/a3p_op3_serve025_edge_center_v20.manifest.json).

## PD Stand deployment

The recorded change gave `a3_deploy_onnx_ref_pingpong` an independent
`--pd-stand-blend-sec`, default **5.0 seconds**. From PASSIVE, measured joint
position/velocity and zero gains seed the quintic transition to the existing
standing pose and Kp/Kd. Entry velocity decays within 0.25 seconds. The existing
motion/policy transition duration is independent. This controls the command
transition; the physical standing response still requires attended validation.

The original field build also removed two Planner pose-age admission checks.
That Planner experiment was subsequently reverted in source by
`98402e43836b06f6d721890f8462f2b7d7620dd2`: the
`target_turn_max_pose_age_s: 0.2` receive-age and source-age checks are retained
in this PR. The historic binary below therefore must not be treated as an
exact build of the current Planner. The source-revert record did not establish
that the robot's Planner binary had also been replaced.

The deployment receipt reported:

- MDU active link `/agibot/a3_deploy_build4_serve025` pointed to
  `/agibot/a3_deploy_stand5s_98724f730cc4`; Runner SHA-256 prefix
  `98724f730cc4`.
- HDU Planner in `install_model21800_fix/hope_planner_cpp` had SHA-256 prefix
  `6bfa6c18acf47`. Its source/config were compared with that deployment's payload.
- Managed restart created session `model21800_20261018T050406Z`, reporting
  `RUNNING / PASSIVE`, `alive=1`, `session_matches=1`, `fault=0` and six ready HAL
  feedback groups. `/proc/22568/exe` matched the deployed Runner; the startup log
  showed `pose_gain_blend_sec=5.000`.
- The site retained KERNEL mode, its existing model 14600 (ONNX SHA-256 prefix
  `a987240610bb`) and serving CSV. The session-name prefix `model21800` was not
  evidence of the loaded model. KERNEL mode did not start Planner.
- No PD_STAND, Motion, Serve or gripper action was sent during this receipt.
  PASSIVE startup is not a physical stand/serve acceptance test.
- Nine command-transition tests and twelve Planner ROS integration tests passed
  for the then-current experimental source. The latter do not validate the
  later reverted Planner state. An isolated HDU Planner startup check also passed.

The prior MDU package remained at
`/agibot/a3_deploy_model14600_recordfix_f0932ed1`; HDU backups were under
`/home/agi/hope_updates/stand5s-98724f730cc4/backup`. Source payloads, hashes,
build logs and runtime evidence were kept in the original deployment's
`stand-and-planner/` archive. These paths identify a past deployment, not
portable install defaults.

## Failed START and HDU reboot observations

An earlier attended START at 2026-10-18 04:00:36 UTC used the robot's existing
`/agibot/a3_deploy_model14600_recordfix_f0932ed1` package and session
`model21800_20261018T040036Z`. HAL reported six ready groups and Runner stayed
PASSIVE at approximately 500 Hz with `command_fault_latched=0`; no motion or
gripper action was sent.

Around 04:02 UTC the HDU rebooted while the MDU remained running. Persistent
`/var/lib/systemd/pstore/dmesg-ramoops-0` recorded
`[636.478670] Kernel panic - not syncing: RCU Stall` on vendor kernel
`6.1.118 PREEMPT_RT`. A stalled `ktimers/6` stack involved `rt_spin_lock` and
`__hrtimer_run_queues`; another CPU showed `pl330_irq_handler`. SPI busy and RT
throttling were also recorded. The panic establishes the reboot cause, but the
record does not identify a specific driver fix.

The HDU reboot lost Planner and supervisor session memory. **Complete START
acceptance failed.** Recovery observed fresh PASSIVE Runner state and the
existing preflight guard entered `FAILED / MANAGED_RECOVERY_REQUIRED`, rather
than treating the robot as stopped. A subsequent reboot did not resolve the
problem: a new pstore entry reported an RCU Stall at uptime 692.48 seconds.
The matching vendor kernel/firmware fix was not established in this record.
Do not infer standing/serving readiness from a reachable Foxglove connection.
The earlier receipt also recorded an approximately -6.2-second HDU NTP offset;
that is a historical value, not a current measurement.

The historical evidence files included `passive-live.json`,
`runner-passive-evidence.log`, `hdu-kernel-panic.txt`, `reboot-diagnostic.log`,
`reboot-syslog.log`, `reboot-vendor.log`, `post-recurrence-live.json` and
`hdu-reboot-diagnostics.zip`.

## Startup cleanup and supervisor privileges

The field checkout incorporated upstream background-cleanup commit
`58108499672dcf47bf76d4ef9f7b65f1b652bad2` as local commit
`f1c93c16f14124c0fd360903470c3cc0511e2116`. The helper and supervisor updates
were installed on the HDU with backups, initially without submitting START.

The deployed supervisor had `NoNewPrivileges=true` (`NoNewPrivs=1`), which
prevented its new cleanup command, `sudo -n systemctl stop agibot_ui.service`,
from elevating. Ordinary SSH sudo access did not prove that the confined
supervisor could run it. The preflight consequently failed with
`UNRELATED_BACKGROUND_NOT_STOPPED` while the vendor UI was active.

The field operator then retained the original upstream sudo-based cleanup and
installed a supervisor-only drop-in
`/etc/systemd/system/hope-lifecycle-supervisor.service.d/70-upstream-startup-sudo.conf`
setting `NoNewPrivileges=false`. The historical record reports `NoNewPrivs=0`
after restart, no new sudoers rules for that change, removal of an interim
polkit rule, and unchanged settings for other services. This is a description
of a specific privilege change, not a request to disable service hardening on
another robot. Use the current reviewed installation procedure and narrowly
scoped permissions instead of copying a historical workaround blindly.

Reported verification was limited to 27 cleanup/lifecycle tests, Bash syntax,
the exact sudo command executed as `agi` in the service mount namespace, and
KERNEL/OPTITRACK preflights returning COMPLETE with PASS cleanup receipts.
Recording, process manager, bridge and chrony PIDs were unchanged. No START or
joint motion was sent in that validation. It did not prove that RCU Stall was
fixed. Backups included
`/var/backups/hope-foxglove/581084996-20261018T043022Z` and
`/var/backups/hope-foxglove/main-startup-20261018T044112Z`; the historical archives
were named `foxglove-581084996/`, `main-startup-alignment/` and
`foxglove-cleanup-permissions/`.
