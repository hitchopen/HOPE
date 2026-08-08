// Ping-pong policy front-end as a drop-in A3PolicyDriver CommandFn. Per tick:
//   scripted/planner racket target -> reference clock -> ONNX refs ->
//   180/177/175-D obs (auto-selected from the model input dim) ->
//   action -> target_q (Isaac) -> scatter to 31 SDK slots -> RobotCommand.
// NECK PASSIVE: head slots [3,4] are held at nominal, clamped into a bounded
// contract's current selected interval, with AGI's fixed PD (kp=40, kd=2);
// the model's neck outputs are ignored for hardware command.
//
// ===================== WHAT IS SCRIPTED vs LEARNED =====================
// The swing JOINT TRAJECTORY is NOT hard-coded. Every tick the learned ONNX
// policy emits a fresh 31-DOF action; q_des = default_q + action*action_scale.
// The forehand/backhand BODY POSTURE is learned (encoded in the policy weights +
// the baked reference clip the ONNX carries as obs-independent side-outputs).
//   LEARNED (ONNX, per tick):  31 joint actions -> q_des; kp/kd from metadata.
//   REFERENCE (ONNX side-out): command[0:62] ref joint_pos/vel + tracked body
//                              poses, indexed by time_step (the strike clock).
//   SCRIPTED (this file, C++):  the racket TARGET (pos/vel/normal-sign), the
//                              strike clock (time_to_strike -> time_step), and
//                              the forehand/backhand SELECT (swing_dir_). There
//                              is NO live ball tracker / planner -- ScriptedTarget
//                              is a fixed front-right TEST target. Pressing f/b
//                              only flips the target y-sign + swing_type and picks
//                              the matching baked clip; it does not load new poses.
//   OVERWRITTEN AFTER ONNX:    neck slots [3,4] forced passive; legs forced to
//                              nominal iff --legs-passive; q_des clamped to A3
//                              joint limits (safety). Nothing else is overridden.
// To hit REAL balls, replace ScriptedTarget with planner output (pos/vel/normal/
// hit-time from a ball-trajectory estimator); the policy/obs/decode stay as-is.
// ======================================================================
//
// Depends only on Eigen + onnxruntime + robot_io_backend.hpp (plain structs),
// so it is unit-testable off-robot. The CommandFn signature matches
// a3_deploy::CommandFn exactly (assignable without including the AimRT driver).
#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

#include "a3_deploy/numeric_safety.hpp"
#include "a3_pingpong/pp_base_estimator.hpp"
#include "a3_pingpong/pp_joint_limits.hpp"
#include "a3_pingpong/pp_joint_map.hpp"
#include "a3_pingpong/pp_obs_builder.hpp"
#include "a3_pingpong/pp_onnx_policy.hpp"
#include "a3_pingpong/pp_oracle_pose.hpp"
#include "a3_pingpong/pp_planner_lifecycle.hpp"
#include "a3_pingpong/pp_planner_input.hpp"
#include "a3_pingpong/pp_qdes_contract.hpp"
#include "a3_pingpong/pp_reference_clock.hpp"
#include "a3_pingpong/pp_velocity_gate.hpp"
#include "a3_pingpong/pp_static_handoff.hpp"
#include "a3_policy_parameters.hpp"      // ::a3_pd_stand_kps / kds (official robust-stand gains)
#include "robot_io/a3_layout_extra.hpp"  // robot_io::kA3PolicyToSdkIdx (29->31 scatter)
#include "robot_io/robot_io_backend.hpp"

namespace a3_pingpong {

// AGI neck-passive constants (from expand_to_backend.hpp).
constexpr int kHeadSlot0 = 3;
constexpr int kHeadSlot1 = 4;
constexpr double kHeadPosRad = 0.0;
constexpr double kHeadKp = 40.0;
constexpr double kHeadKd = 2.0;
// Backend MuJoCo slot layout: legs are slots [19..30] (12 DOF).
constexpr int kLegSlotStart = 19;
constexpr int kLegSlotCount = 12;
// Waist is slots [0..2] (waist_yaw, waist_roll, waist_pitch).
constexpr int kWaistSlotStart = 0;
constexpr int kWaistSlotCount = 3;
// Arms are slots [5..18]: left shoulder p/r/y, elbow, wrist r/p/y (5..11), then right (12..18).
constexpr int kArmSlotStart = 5;
constexpr int kArmSlotCount = 14;

// torso_Link's frame origin relative to the pelvis: offset (-0.02,0,0.005) carried
// through waist_yaw (Isaac q[2], +Z) and waist_roll (Isaac q[5], +X). The torso
// anchor sits ~at the waist (≈ base + 5 mm up), NOT 0.25 m up — verified against
// the model's reference body_pos_w[7] ≈ pelvis + 0.005. Using a fixed (0,0,1.20)
// torso put a ~0.25 m bias into the motion_anchor_pos observation.
inline Vec3 torso_pos_from_base(const Vec3& base_pos, const Vec4& base_quat,
                                const Eigen::VectorXd& q_isaac) {
  const double wy = q_isaac.size() > 2 ? q_isaac[2] : 0.0;  // waist_yaw
  const double wr = q_isaac.size() > 5 ? q_isaac[5] : 0.0;  // waist_roll
  const Mat3 Rwaist = (Eigen::AngleAxisd(wy, Vec3::UnitZ()) *
                       Eigen::AngleAxisd(wr, Vec3::UnitX())).toRotationMatrix();
  const Vec3 off = Rwaist * Vec3(-0.02, 0.0, 0.005);
  return base_pos + mat_from_quat(base_quat) * off;
}

// How the localization-dependent obs terms (motion_anchor_pos_b,
// racket_target_pos_b, base_target_pos_b) get their robot world pose. The 180-D
// obs LAYOUT is identical in all three modes; only the *values* of these terms
// change. See SIM_DEPLOY_REHEARSAL.md.
enum class LocMode {
  kFabricated,       // A (legacy): nominal frozen base pose + waist-FK torso.
                     //   -> motion_anchor_pos_b is a FICTIONAL tracking error.
  kPerfectTracking,  // B (hardware-safe): assume position tracking is perfect.
                     //   torso_pos_w := ref anchor (motion_anchor_pos_b == 0);
                     //   base_pos_w := ref pelvis (racket/base target relative to
                     //   where we SHOULD be). Real IMU still drives orientation.
  kOracle,           // C (SIMULATION ONLY): true MuJoCo pelvis pose from the shm
                     //   bridge. NEVER available on hardware (shm file absent).
  kExternalBase,     // HARDWARE planner mode: calibrated base POSE from live mocap
                     //   localizer (PpBasePoseInput / /a3/base_pose_flat), in the SAME
                     //   world frame as the planner's racket target. V17 requires schema 2
                     //   and never falls back to an IMU-derived absolute heading. Stale ->
                     //   hold last mocap pose, block engage and warn loudly.
};

struct PpPolicyConfig {
  // Unitree-style policy directory contract. Empty keeps legacy ONNX-metadata
  // loading for old artifacts; formal HitterPingPong deploy always sets it.
  std::string deploy_cfg_path;
  int level = 1;                 // 0 = hold wind-up (quasi-stand), 1 = periodic forehand
  bool legs_passive = false;     // hold leg joints at nominal (firm PD) — for a HOISTED demo
                                 // where balance isn't needed; stops leg twitch from the
                                 // nominal-base-position obs gap. Arm+waist still swing.
  bool waist_passive = false;    // ALSO hold the waist (slots 0..2) at nominal. The policy
                                 // commands waist_pitch to its forward limit (+0.419) which,
                                 // with the forehand arms reaching forward, pushes the CoM past
                                 // the feet → a STATIC leg hold can't rebalance → tips forward.
                                 // Freezing the waist keeps the torso CoM over the feet for an
                                 // ARMS-ONLY ground swing (with --official-stand → official gains).
  bool auto_leg_hold = false;    // dynamic hold: at level 0 (ready/windup) HOLD legs+waist (stable
                                 // stand, avoids the frozen-windup OOD foot-lift); at level 1 (swing)
                                 // RELEASE them (full-body self-balancing swing). The driver flips
                                 // set_legs_passive/set_waist_passive each tick from the level.
  bool arm_hold_nominal = false; // stage-cosmetics override (--arm-hold-nominal): during level-0
                                 // holds, after the robot has been CONTINUOUSLY quiet for
                                 // arm_hold_min_quiet_s, RAMP the ARM q_des (slots 5..18) to nominal
                                 // over arm_hold_blend_s and hold; any non-quiet tick (or level 1)
                                 // RELEASES the arms to the policy instantly. Fixes model_17400's
                                 // hold arm-twist (left-down/right-up into the q_des clamp — hold
                                 // arm posture is reward-free in that training generation) WITHOUT
                                 // retraining. History: an instant-quiet 1 s ramp toppled the
                                 // post-swing recovery (P3b z≈0.25 twice) — the sustained-quiet
                                 // floor + slow ramp + instant-release is the variant that survives
                                 // it (g25-verified). The policy still sees the measured (held) arm
                                 // q in obs — the legs_passive-proven mismatch class. Opt-in;
                                 // retire once an arm_hold-retrained model ships.
  double arm_hold_blend_s = 2.5; // ramp time to nominal once the sustained-quiet floor is met.
  double arm_hold_min_quiet_s = 1.0;  // continuous upright+still time required before ramping.
  double leg_smooth_alpha = 1.0; // EMA low-pass on the POLICY-DRIVEN leg q_des: out = a*in + (1-a)*prev.
                                 // 1.0 = off (no smoothing). <1 removes the tick-to-tick jitter that
                                 // stiff weight-bearing gains (--leg-stand-gains) amplify into a TWITCH;
                                 // ~0.2-0.3 = moderate (tau ~3-4 ticks @50Hz). Seeded from nominal so the
                                 // release does not jump; no-op when legs are HELD. See --leg-smooth-alpha.
  double leg_clamp_rad = 0.0;    // 0 = off. >0 clamps each POLICY-DRIVEN leg slot (level-1
                                 // released swing) to nominal ± this band, capping the deep
                                 // crouch-and-lean the trained swing commands (hip_pitch -0.6..
                                 // -0.77, ankle_pitch -0.7..-0.9 rad) that the real robot cannot
                                 // hold standing -> knees sink. Keeps legs near the proven upright
                                 // stand while leaving room for small balance moves. No-op when
                                 // legs are HELD (already nominal). See --leg-clamp-rad.
  bool use_base_estimator = false;  // leg-FK + IMU pelvis-height estimate (planted feet).
                                    // ON for the ground test; OFF on the hoist (feet hang ->
                                    // planted assumption invalid -> use nominal height).
                                    // Only affects kFabricated mode.
  // HARDWARE-SAFE DEFAULT: perfect_tracking. kFabricated synthesizes a fictional
  // world-tracking error (the documented "deploy buzz" mode) and must NOT be the
  // default on hardware. The A/B/C rehearsal selects fabricated explicitly via
  // --loc-mode. See LocMode + SIM_DEPLOY_REHEARSAL.md.
  LocMode loc_mode = LocMode::kPerfectTracking;  // A/B/C localization mode (see LocMode).
  // DEFAULT FLIPPED TO TRUE (2026-07-03): with yaw_align (below, default on) the base yaw is
  // expressed relative to the ENGAGE heading — it starts at identity and then tracks the robot's
  // REAL turning, which is exactly what training saw (targets rotated by the current base yaw,
  // hope_commands.racket_target_pos_b_rel). The old false default (identity yaw) predates
  // yaw_align and silently mixed frames: the racket-FK world conversion uses the full
  // yaw-aligned quat while the target rotation ignored yaw — fine only while the robot never
  // turns. model_9000 TURNS ~84 deg by design (v4 clips are baked facing world +Y), so identity
  // yaw would rotate the target obs ~84 deg OOD mid-motion. Revert with --no-imu-yaw (only
  // sensible for a non-turning model, e.g. p4).
  bool use_imu_yaw_for_targets = true;  // see build_obs_180(use_base_yaw_for_targets)
  double oracle_max_age_s = 0.1;    // reject oracle samples older than this (stale bridge/sim).
  double dt = 0.02;              // 50 Hz
  double strike_period = 3.0;    // seconds between strikes (level 1)
  double strike_lead_frac = 0.7; // strike occurs at this fraction of each cycle
  // SINGLE-SWING / REST mode: the periodic level-1 clock WRAPS every strike_period —
  // the reference SNAPS from the clip's end pose back to the windup frame mid-stance.
  // Training never tracks that transition (clip wraps TELEPORT the robot in Isaac), and
  // the backhand end->windup pose gap is large enough that the snap topples the free
  // base (observed: p4 backhand survives swing 1, collapses right after the first wrap;
  // forehand's smaller gap survives). single_swing: after the clip has fully played
  // (tts below the clip's end), auto-drop to level 0 (held stand / windup hold) instead
  // of wrapping — press 1 to swing again from a clean windup start (which the policy
  // provably handles). swing_rest_s >= 0: additionally auto re-arm level 1 after that
  // many seconds of rest (continuous demo without ever snapping).
  bool single_swing = false;
  double swing_rest_s = -1.0;    // <0 = no auto re-arm (manual '1' per swing)
  // ===================== LIVE PLANNER MODE (Path B, official) =====================
  // When planner_mode: the racket target is NO LONGER the scripted per-clip box center.
  // A real planner feeds PpRacketTargetInput (over AimRT /racket/command_flat) and a mocap
  // localizer feeds PpBasePoseInput (LocMode::kExternalBase). Each ComputeCommand tick,
  // PlannerEngageStep_ reproduces the proven Python wbc_runner._tick engage machine:
  // gate a fresh VALID command (timeout / invalid-flutter grace / min-tts / base-low /
  // reachability), then set_swing_dir + set_level(1) + FREEZE the target. The existing
  // swing clock, tts clamps, single-swing completion and mid-swing latch execute the swing
  // UNCHANGED. planner_mode implies single_swing (one clip per engage, then a held stand).
  bool planner_mode = false;
  // Field execution matching the learned policy contract. The ball clock owns
  // release; station READY and yaw are telemetry rather than runner-added hard
  // admission gates. Freshness, finite checks, timing and one-shot consumption
  // remain mandatory input/lifecycle correctness checks.
  bool policy_native = false;
  // x86 Gate3 policy audit only.  Keep the actor's finite final q_des unchanged
  // even when it escapes the exported safe interval or the backend hard limits,
  // and record the would-be intervention instead of throwing/clamping.  The
  // production runner never enables this flag.
  bool gate3_qdes_audit_only = false;
  double shot_reuse_tolerance_s = 0.25;
  double engage_min_tts_s = 1.0;      // never START a swing later than this (deep-clip snap -> fall)
  // DEEP PREFIX-SKIP (2026-07-13, 110-D only): the v13 windups open with a near-static
  // ready prefix (measured from the clip npz: joint |vel| <= 0.5 rad/s / bodies <= 0.2 m/s
  // until ~0.53*windup fh / ~0.46*windup bh; the dynamic swing onset is fh 0.48 s of 0.82 s,
  // bh 0.44 s of 0.96 s). Engaging with tts0 < windup starts the reference clock mid-prefix
  // (time_step_for already supports it — the old 0.9 gate allowed a 10% band), and a
  // mid-PREFIX start is ~the trained ready stand, so it is not OOD the way a mid-SWING
  // start is. For model_21800 hardware this is also the deterministic target-snapshot
  // delay: keep consuming planner revisions until tts reaches windup - skip. Other 110-D
  // contracts retain the historical behavior where it only widens the late boundary.
  // The commit time is windup - skip, where
  // skip = clamp(engage_prefix_skip_s, 0.10*windup, 0.45*windup) per clip. The 0.45 cap
  // keeps the start strictly before the measured dynamic onset (>=0.46*windup) on BOTH v13
  // clips; the 0.10 floor preserves the historical 0.9*windup gate when this is set to 0.
  // v13 effect at the 0.20 default: fh late-cutoff 0.738 -> 0.62 s, bh 0.864 -> 0.76 s.
  double engage_prefix_skip_s = 0.20;
  double planner_invalid_grace_s = 0.25;  // a valid cmd still engages if an invalid arrived within this
  double command_timeout_s = 0.5;     // no fresh VALID command within this -> stand
  double pending_expire_after_strike_s = 0.25;  // retain station while blocked; expire only after ball passes
  double base_low_z = 0.7;            // base below this (fallen/crouched) -> refuse to engage
  double hold_anchor_x_b = 0.40;      // base-rel x of the ready-hold target between swings (racket-reach)
  // Post-swing hold budget: run the POLICY hold this long after a completed swing (it must
  // actively balance out of the follow-through — a static stand cannot), then blend to the
  // STATIC official stand until the next engage. The model's level-0 policy hold only has
  // ~5 s of margin (Gate 2.5: scripted m0 hold falls at ~5 s; closed-loop: post-swing hold
  // degrades at ~5-10 s) — never park on it.
  double hold_recover_s = 2.5;
  double hold_blend_s = 0.8;          // q_des ramp measured-pose -> nominal at the switch
  double external_base_max_age_s = 0.2;   // reject base-pose samples older than this (stale mocap)
  double external_base_gyro_propagation_max_s = 0.05;  // short age bridge, mocap-anchored
  // Reachability gate (base-relative x,y + world z + speed); mirrors wbc_runner target_gate.
  bool target_gate_enable = true;
  double gate_x_lo = 0.20, gate_x_hi = 0.90;
  double gate_y_abs = 0.85;
  double gate_z_lo = 0.55, gate_z_hi = 1.40;
  double gate_speed_max = 3.5;
  // ============== 110-D hitter_pure additions (2026-07-07, HITTER-paper deploy) =============
  // The 110 engage gate is METADATA-driven (per-clip z bands + station geometry from the
  // ONNX hitter_pure boxes), replacing the fixed base-relative box above. These bound the
  // remaining free parameters:
  double gate_station_step_max = 0.85;  // max |derived station − current base| xy (m); trained
                                        // stations span ±0.40 vs spawn and up to ~0.8 m between
                                        // consecutive swings (paper Fig. 4 goes to ±0.75-0.8)
  double gate_station_step_margin = 0.05;  // Final metadata hi + this tolerance is the max gate
  // Per-AXIS x readiness gate (2026-07-09, official-G3 serve-8 fall): the norm-only step
  // gate above is calibrated for LATERAL station steps (trained y-footwork). It let a swing
  // engage with the base +0.19 m FORWARD of the station mid-walk-back — a compressed-reach
  // view (target 0.32 m ahead vs the trained 0.51 strike view; training wander is ±0.10)
  // the policy never swung from: the forehand diverged 1.44 m and fell. x-locked models
  // train ZERO x-station steps, so ANY x offset at engage is untrained geometry — gate it
  // separately and tightly: reject while |station_x − base_x| > this, let the ACTIVE policy
  // hold finish walking back, and take the next serve ("move to station, WAIT, then
  // strike"). 0.15 = the deploy x-lock assertion threshold (1.5x trained wander, under the
  // ~0.19-0.20 OOD edge). <= 0 disables (legacy x-free generations use the norm gate only).
  double gate_station_x_max = 0.15;
  // RallyFinal pending-station rhythm. The explicit ONNX rally_final_v1 marker plus validated
  // station-step metadata enables this path: while level 0, feed the predicted station into the fixed-windup
  // policy hold, wait until lateral error and estimated planar speed are both small for a
  // sustained interval, and only then let the clip clock arm. Legacy ONNX files have no
  // runtime marker and retain the historical immediate-engage behavior byte-for-byte.
  double station_ready_x_max = 0.10;       // |base_x - pending_station_x| [m]
  double station_ready_y_max = 0.10;       // |base_y - pending_station_y| [m]
  double station_ready_speed_max = 0.20;   // localization finite-difference speed [m/s]
  double station_ready_hold_s = 0.12;      // both conditions must persist this long [s]
  double station_takeover_blend_s = 0.15;  // static nominal -> level-0 policy; fits ball horizon
  bool station_ready_enable = true;        // --no-station-ready is diagnostic-only
  // RallyV8 mostly-stationary inversion (recipe-gated; see stay_if_reachable_ member).
  // --no-stay-if-reachable forces legacy band-center inversion for an A/B.
  bool stay_if_reachable_enable = true;
  // Gate3A deploy-plant rehearsal: exercise the exact pending-station move/settle state
  // machine, but stop after its READY dwell and categorically forbid a swing release.
  // Default false preserves the continuous-rally Gate3 and hardware behavior.
  bool station_only = false;
  // x86-only, non-certifying MuJoCo replay for the exact V17 recipe-v1/model_16600
  // artifact. Keep the full learned balance controller active, but bind every
  // accepted shot to the first fresh MOTION-session base anchor. A target whose
  // inverse reach geometry asks for a larger station change is skipped.
  PpOnnxLoadProfile onnx_load_profile = PpOnnxLoadProfile::kProductionStrict;
  bool fixed_station_replay = false;
  bool moving_station_replay = false;
  // Isolated fixed-station A/B: the external station remains immutable, but
  // post-strike lateral error is amplified in the actor's coupled base/racket
  // hold-target channels so the already-trained y footwork closes back onto
  // that same origin. x is deliberately untouched and remains policy-native
  // self-recovery.
  bool fixed_y_homing_replay = false;
  double fixed_y_homing_enter_m = 0.030;
  double fixed_y_homing_exit_m = 0.020;
  double fixed_y_homing_gain = 2.0;
  double fixed_y_homing_max_delta_m = 0.240;
  double fixed_station_tolerance_m = 0.020;
  double gate_z_margin = 0.05;          // slack around the per-clip trained z band (m)
  // Per-clip trained VELOCITY box gate (2026-07-08, from the first rally-gate fall): the old
  // |v|<=3.5 speed cap accepted a planner demand of (0.9,+0.18,0.7) — vy 0.18 vs the trained
  // fh box [0.96,1.96] — and the swing executed on an out-of-distribution velocity command
  // (follow-through charged +0.57 m off-station; trained follow-through drift is 0.01-0.02 m).
  // Engage + mid-swing streaming now also require vel_w inside the per-clip
  // hitter_pure_vel_range_per_clip metadata box, per axis, +- this margin (m/s).
  // Raise via --vel-gate-margin if a venue's demanded returns sit just outside the box —
  // but read the REJECT(110) vel print first: a far-out demand is a planner mistuning
  // (delta_t_flight / target_land aim), not a gate problem.
  double gate_vel_margin = 0.30;
  // STREAM-until-contact (paper Fig. 3: the planner refines the prediction to ~0 error at
  // contact; the paper's WBC consumes the stream — there is NO lock-at-engage). While a swing
  // flies, same-side commands passing the band gate keep updating WHERE (pos/vel); WHEN stays
  // the engage-latched clip clock (training never varies tts mid-swing — the training analog
  // of streaming WHERE is racket.midswing_resample_prob, whose tts floor this mirrors).
  // ⚠ DEFAULT OFF (2026-07-08): the deployed baseline TRAINED with midswing_resample_prob
  // = 0.0 (HOPEPingPongHitterPure.yaml:187) — for it, every mid-swing target update is an
  // untrained obs transition, and streaming also moves the derived STATION mid-swing, which
  // even the training-side resample contract holds fixed. Enable with --stream-target ONLY
  // for a model actually trained with midswing_resample_prob > 0.
  bool stream_target = false;           // 110-D models only; other contracts keep the lock
  double stream_tts_floor_s = 0.30;     // freeze the target inside the last 0.3 s before strike
  // DEMO-ROBUSTNESS velocity mode (--demo; --vel-box-center is a compatibility alias):
  // command the per-clip
  // TRAINED BOX-CENTER velocity (== the reference swing's strike velocity, the manifold
  // the policy is most robust on) instead of the planner's solved velocity. The planner
  // still owns WHERE (pos) and WHEN (tts); only the outgoing-shot aim precision is given
  // up (the return goes roughly where the human demo's returns went). Rationale: the
  // planner's physically-solvable velocities intersect the trained box only near its
  // low-z corner (rally-gate measurement: demanded vz 0.08-0.10 vs trained center 0.71),
  // and off-center vel commands erode the swing margin in the stricter AGI sim.
  bool vel_cmd_box_center = false;
  // ENGAGE HEADING GATE (2026-07-08, rally run-3 fall): training swings always START facing
  // ~+x (episode resets; reference clips yaw at most ±20° MID-swing and END back at ~0-6°),
  // but a divergent follow-through can leave the real robot 30-70° off heading, and the
  // 110-D obs are world-frame — an engage from a yawed stand is far outside the trained
  // start distribution (measured: engage at ~-30° yaw -> |act| 58, 2 m sprint, violent
  // fall). Refuse to engage while the (yaw-aligned) base heading is off by more than this;
  // status shows "yawed". Recovery = the operator re-stand ('s' -> square the robot -> 'm').
  double engage_yaw_max_deg = 20.0;
  // Stricter heading bound for the STATIC-stand handoff (rally run 5: a +17° handoff —
  // legal under the 20° engage gate — tipped ~3 s after the gains froze; the static stand
  // needs a genuinely square stance, while an engage merely needs a near-trained start).
  double static_handoff_yaw_max_deg = 10.0;
  // MOTION-ENTRY SETTLE (2026-07-08): run 3 engaged a leftover in-flight serve on the SAME
  // tick MOTION started (the robot was seconds off the stand-gain catch). Give the stand
  // this long before the first engage of a MOTION session.
  double engage_settle_s = 1.0;
  // 110-D LEVEL-0 station semantics (2026-07-07 fix): false (nominal) = the 177-style
  // fixed-world hold anchor — idle actively station-keeps, pulling the base back after every
  // follow-through so displacement can NOT accumulate across swings (the Gate-2.5 P7 creep).
  // true = the legacy Δ=0 idle (station := current base), kept ONLY for A/B: it let the robot
  // free-creep between swings (12200 P7 fall) and diverges outright for hold-trained rally
  // models (18000 P2 fall) — see the level-0 branch comment.
  bool idle_station_dzero_110 = false;
  // YAW-ALIGN (hardware fix, 2026-07-02): the pelvis AND torso IMU yaws are NOT
  // world-referenced on the real robot (boot-to-boot drift; MDU captures show a constant
  // fictional -12/-15/-38.5 deg yaw error in motion_anchor_ori_b while training reset noise
  // is only +-11 deg). Two obs terms consume the raw quats: motion_anchor_ori_b (torso vs
  // clip-frame reference anchor) and the racket-FK world conversion (R(base_quat)*fk vs the
  // identity-yaw target frame) — the old use_imu_yaw_for_targets=false fix only bypassed the
  // TARGET rotation. With yaw_align, each IMU's yaw is captured at the moment the policy
  // engages (SHADOW/MOTION entry; robot standing, facing its operational forward = the clip
  // world +x) and its inverse is left-multiplied onto every subsequent sample, so attitudes
  // are expressed relative to the entry heading. No-op in sim (yaw ~ 0 at spawn).
  bool yaw_align = true;
  double swing_speed = 1.0;      // <1.0 stretches the swing in real time so the
                                 // hardware actuators can actually track it
                                 // (native speed under-shoots + strains loudly).
                                 // The clip frame AND obs time_to_strike slow
                                 // together, so the (frame,tts) pair stays on the
                                 // training manifold — just evolves slower.
  // Scripted swing direction at startup: false=forehand (clip 0), true=backhand
  // (clip 1). Toggle live with the f/b keys. No live planner — this is the
  // scripted TEST path.
  bool start_backhand = false;
  // Legacy fallback scripted targets, PER CLIP, chosen inside the model_18400 sampling boxes.
  // A 110-D export with hitter_pure pos/vel boxes overwrites both arrays with its own per-clip
  // box centers in PpPolicy's constructor; metadata-less legacy models retain these values.
  // RE-SYNCED 2026-07-09 to the model_18400_xlock generation (HOPEPingPongHitterPure,
  // run 2026-07-08_23-03-18; the 110-D hitter_pure contract). This generation moved the striking
  // plane to x=0.51 RELATIVE to the station (the demo blade midpoint; the prior 0.70 forced a
  // forward lunge) AND locked the station in x (base_target_x_range [0,0]) — a TRUE x-locked /
  // y-footwork striker (G1 composite 0.998, drift_fwd 0.008 m/swing). Samples only y/z on x=0.51:
  //   clip0 forehand: pos x[0.51,0.51] y[-0.65,-0.15] z[0.67,0.97]  vel x[1.05,2.05] y[ 0.96, 1.96] z[0.31,1.11]
  //   clip1 backhand: pos x[0.51,0.51] y[-0.05, 0.45] z[0.88,1.18]  vel x[1.61,2.61] y[-1.21,-0.21] z[0.00,0.71]
  // Targets sit at the BOX CENTERS (fh (0.51,-0.40,0.82) vel (1.55,1.46,0.71); bh (0.51,0.20,1.03)
  // vel (2.11,-0.71,0.36)); y-centers ≈ the ref reach y (fh −0.409 / bh +0.185). The 110-D station
  // geometry the RUNNER uses is derived from the ONNX hitter_pure_pos_range_per_clip box
  // (plane_x=0.51). These constants are only the fallback when those boxes are absent.
  // ⚠ model_9000 is a WALK-AND-STRIKE policy: with these world-fixed targets it turns ~84 deg and
  // displaces its base 0.4-0.65 m before contact (measured in the deploy-faithful MuJoCo gate).
  // That footwork only closes the loop when the localization source reports the REAL base motion
  // (sim: --oracle-pelvis; hardware: mocap). Under perfect_tracking the base obs stays pinned to
  // the reference pelvis and the strike loop runs OPEN — validate in sim with BOTH loc modes.
  // NOTE (RESOLVED 2026-07-09): the earlier warning here — that pulling x below 0.70 with a
  // RUNNER constant fails Gate 2.5 P3b (model_19400_holdfix2 at x=0.58/0.63 fell post-swing
  // because recovery was only trained near the 0.70 blade point) — no longer applies. The 0.51
  // plane is now the TRAINED reference blade point (retrain rider = base-relative fixed-reach-x
  // sampling, done in run 2026-07-08_23-03-18), so the policy recovers around 0.51. Do NOT
  // re-introduce a runner-side plane offset on top of the trained 0.51.
  Vec3 racket_pos_w_clip[2] = {Vec3(0.51, -0.40, 0.82), Vec3(0.51, 0.20, 1.03)};
  Vec3 racket_vel_w_clip[2] = {Vec3(1.55, 1.46, 0.71), Vec3(2.11, -0.71, 0.36)};
  // sim2real localisation gap: no global base/torso pose -> nominal (matches
  // the Python wbc_runner shadow behavior). base orientation uses the real IMU.
  Vec3 nominal_base_pos_w = Vec3(0.0, 0.0, 0.95);
  Vec3 nominal_torso_pos_w = Vec3(0.0, 0.0, 1.20);
  Vec4 nominal_torso_quat_w = Vec4(1.0, 0.0, 0.0, 0.0);
};

class PpPolicy {
 public:
  PpPolicy(const std::string& onnx_path, PpPolicyConfig cfg = {})
      : onnx_(onnx_path, cfg.onnx_load_profile, cfg.deploy_cfg_path),
        cfg_(cfg), level_(cfg.level),
        swing_speed_(cfg.swing_speed), swing_dir_(cfg.start_backhand ? -1 : 1),
        legs_passive_(cfg.legs_passive), waist_passive_(cfg.waist_passive),
        leg_clamp_rad_(cfg.leg_clamp_rad), leg_smooth_alpha_(cfg.leg_smooth_alpha),
        last_action_(Eigen::VectorXd::Zero(kNumJoints)) {
    if (onnx_.is_rally_v17_recipe() &&
        !onnx_.is_v17_r1_stationary_replay() &&
        !onnx_.uses_authoritative_mocap_pose())
      throw std::runtime_error(
          "pingpong: calibrated V17 requires full-pose mocap metadata");
    if (onnx_.uses_authoritative_mocap_pose()) {
      cfg_.external_base_max_age_s =
          onnx_.base_localization_max_age_s();
      cfg_.external_base_gyro_propagation_max_s =
          onnx_.base_localization_max_propagation_s();
      if (cfg_.loc_mode != LocMode::kExternalBase)
        throw std::runtime_error(
            "pingpong: authoritative full-pose observations require "
            "external-base mocap localization");
    }
    if (cfg_.gate3_qdes_audit_only) {
      if (!cfg_.planner_mode || !cfg_.policy_native)
        throw std::runtime_error(
            "pingpong: Gate3 q_des audit-only mode requires planner + policy-native");
      std::fprintf(
          stderr,
          "[pp qdes-audit] X86 GATE3 ONLY: finite policy q_des is published "
          "unchanged; safe/hard-limit exceedances are telemetry, not fail-fast "
          "or runner clamps\n");
    }
    if (onnx_.is_v17_r10_p0_gate3()) {
      if (!cfg_.planner_mode || !cfg_.policy_native || cfg_.station_only ||
          cfg_.fixed_station_replay || cfg_.moving_station_replay ||
          cfg_.stream_target)
        throw std::runtime_error(
            "pingpong: V17-r10 x86 Gate3 requires planner + policy-native, "
            "strike enabled, immutable session station, and frozen target");
    }
    if (onnx_.is_v17_r12_v11_qdes_tuple_hardware() && cfg_.stream_target)
      throw std::runtime_error(
          "pingpong: V17-r12 freezes one schema-2 planner target at engage; "
          "mid-swing target streaming is outside the training contract");
    if (cfg_.fixed_station_replay || cfg_.moving_station_replay) {
      if (!onnx_.is_v17_r1_stationary_replay())
        throw std::runtime_error(
            "pingpong: isolated V17-r1 replay requires the exact isolated "
            "V17 recipe-v1/model_16600 load profile");
      if (!cfg_.planner_mode || !cfg_.policy_native ||
          cfg_.loc_mode != LocMode::kExternalBase ||
          !cfg_.station_ready_enable || cfg_.station_only ||
          !cfg_.stay_if_reachable_enable ||
          !std::isfinite(cfg_.fixed_station_tolerance_m) ||
          cfg_.fixed_station_tolerance_m <= 0.0 ||
          cfg_.fixed_station_tolerance_m > 0.020 + 1.0e-12)
        throw std::runtime_error(
            "pingpong: isolated V17-r1 replay requires planner/policy-native, "
            "fresh external-base localization, readiness, stay-if-reachable, "
            "strike enabled, and fixed-station tolerance <= 0.020 m");
      if (cfg_.fixed_station_replay == cfg_.moving_station_replay)
        throw std::runtime_error(
            "pingpong: isolated V17-r1 replay must select exactly one of "
            "fixed-station or moving-station mode");
      if (cfg_.fixed_y_homing_replay &&
          (!cfg_.fixed_station_replay || cfg_.moving_station_replay))
        throw std::runtime_error(
            "pingpong: fixed-y homing requires the immutable fixed-station "
            "replay mode");
      if (cfg_.fixed_y_homing_replay &&
          (!std::isfinite(cfg_.fixed_y_homing_enter_m) ||
           !std::isfinite(cfg_.fixed_y_homing_exit_m) ||
           !std::isfinite(cfg_.fixed_y_homing_gain) ||
           !std::isfinite(cfg_.fixed_y_homing_max_delta_m) ||
           cfg_.fixed_y_homing_exit_m <= 0.0 ||
           cfg_.fixed_y_homing_enter_m <= cfg_.fixed_y_homing_exit_m ||
           cfg_.fixed_y_homing_gain <= 1.0 ||
           cfg_.fixed_y_homing_max_delta_m <
               cfg_.fixed_y_homing_enter_m ||
           cfg_.fixed_y_homing_max_delta_m > 0.35))
        throw std::runtime_error(
            "pingpong: fixed-y homing thresholds/gain must be finite, "
            "0 < exit < enter, gain > 1, and max delta in [enter,0.35] m");
    }
    if (!build_src_to_sdk(onnx_.joint_names(), isaac_to_sdk_))
      throw std::runtime_error("pingpong: ONNX joint_names do not map onto the backend layout");
    // LOAD-TIME SAFE-RANGE vs HARDWARE-LIMIT GATE (2026-07-23 audit): the runner hard-clamps
    // every published q_des to kSdkJointPosLo/Hi (pp_joint_limits.hpp) after the policy
    // decode. The training safe range baked into the ONNX must sit INSIDE that table —
    // otherwise a future Isaac asset limit revision would silently make the deploy clamp
    // trim q_des every tick (an invisible train/deploy contract break, not the pathological-
    // spike catch the clamp exists for). Refuse to run instead.
    if (onnx_.has_safe_qdes_interval_contract()) {
      for (int i = 0; i < kNumJoints; ++i) {
        const int sdk = isaac_to_sdk_[i];
        if (!(kSdkJointPosLo[sdk] <= onnx_.qdes_safe_lo()[i] &&
              onnx_.qdes_safe_hi()[i] <= kSdkJointPosHi[sdk]))
          throw std::runtime_error(
              "pingpong: ONNX qdes safe range escapes the hardware position limits for "
              "joint '" + onnx_.joint_names()[i] + "' — the deploy hard clamp would trim "
              "q_des every tick; refusing to run");
      }
    }
    if (onnx_.is_rally_v17_recipe()) {
      for (int i = 0; i < kNumJoints; ++i) {
        const int sdk = isaac_to_sdk_[i];
        if (std::fabs(kSdkJointPosLo[sdk] - onnx_.qdes_hard_lo()[i]) > 1e-4 ||
            std::fabs(kSdkJointPosHi[sdk] - onnx_.qdes_hard_hi()[i]) > 1e-4)
          throw std::runtime_error(
              "pingpong: V17 exported hard limits do not match the Gate3 plant "
              "for joint '" + onnx_.joint_names()[i] + "'");
      }
    }
    // Planner-mode pre-engage hold target: seed pos/vel from the forehand box center (the
    // same in-training values the SCRIPTED hold uses) so the level-0 hold obs before the
    // first serve is on-manifold; each engage overwrites them with the frozen command.
    // The y seed matters: a centered (y=0) hold target sits ~0.3 m LEFT of the forehand
    // ready racket -> the policy leans/reaches toward it, sinks, and tips (observed in the
    // headless closed-loop). Box-center y keeps the hold at the trained ready stance.
    planner_frozen_pos_w_ = cfg_.racket_pos_w_clip[0];
    planner_frozen_vel_w_ = cfg_.racket_vel_w_clip[0];
    planner_hold_z_w_ = cfg_.racket_pos_w_clip[0][2];
    planner_hold_pos_b_engage_ =
        Vec3(cfg_.hold_anchor_x_b, cfg_.racket_pos_w_clip[0][1], 0.0);
    // Reference-clock layout: prefer the ONNX-baked per-clip metadata (new exports carry
    // clip_seg_lengths/clip_strike_phases). The ClipLayout default is the LEGACY v1 layout
    // ({95,105}/{0.36,0.50}, model_15200-era); driving a v2-baked model with it serves the
    // wrong reference frames every tick (forehand strike ~0.6 s early, follow-through clamped,
    // "backhand" spliced across the clip boundary) — the 2026-07-02 stale-clock deploy bug.
    if (onnx_.has_clip_layout()) {
      if (onnx_.clip_seg_lengths().size() != 2)
        throw std::runtime_error("pingpong: ONNX clip layout metadata does not have 2 clips");
      clip_.seg_len[0] = static_cast<int>(std::lround(onnx_.clip_seg_lengths()[0]));
      clip_.seg_len[1] = static_cast<int>(std::lround(onnx_.clip_seg_lengths()[1]));
      clip_.strike_phase[0] = onnx_.clip_strike_phases()[0];
      clip_.strike_phase[1] = onnx_.clip_strike_phases()[1];
      std::fprintf(stderr,
          "[pp] clip layout from ONNX metadata: seg_len={%d,%d} strike_phase={%.3f,%.3f} "
          "(strike frames %d/%d)\n",
          clip_.seg_len[0], clip_.seg_len[1], clip_.strike_phase[0], clip_.strike_phase[1],
          clip_.strike_frame(0), clip_.strike_frame(1));
      // Baked-clip GROUNDING check (2026-07-03): the runner (and training's actor obs!) consume
      // the RAW clip-world reference. A properly re-grounded clip (scripts/reground_hope_frame.py,
      // e.g. the hopex lineage / p4) has frame-0 pelvis yaw == 0 -> refs coincide with the engage
      // heading and a strike-in-place policy deploys cleanly under perfect_tracking. A NON-re-
      // grounded clip (e.g. registry v4, pelvis yaw ~+82/+86 deg) trains a TURN-AND-WALK policy
      // whose footwork perfect_tracking cannot observe (base obs pinned to the ref pelvis) ->
      // open strike loop on hardware. Print per-clip baked yaw so a raw-clip model is never a
      // silent surprise again.
      for (int c = 0; c < 2; ++c) {
        const auto r0 = onnx_.refs(clip_.seg_start(c));
        const Vec4& q0 = r0.anchor_quat_w;
        const double yaw0 = std::atan2(2.0 * (q0[0] * q0[3] + q0[1] * q0[2]),
                                       1.0 - 2.0 * (q0[2] * q0[2] + q0[3] * q0[3]));
        const double yaw0_deg = yaw0 * 180.0 / M_PI;
        std::fprintf(stderr, "[pp] clip %d baked frame-0 anchor yaw = %+.1f deg%s\n", c, yaw0_deg,
                     (std::fabs(yaw0_deg) > 20.0)
                         ? "  ** NOT RE-GROUNDED: policy will TURN toward the clip heading and "
                           "step to its target; that footwork is INVISIBLE under perfect_tracking "
                           "(open strike loop). Use oracle/mocap localization, or deploy a model "
                           "trained on re-grounded (+X, yaw~0) clips. **"
                         : "");
      }
      // Periodic-wrap guard (2026-07-03): in the default periodic mode the tts clock wraps
      // (1-strike_lead_frac)*strike_period seconds after the strike. If that is shorter than a
      // clip's follow-through, the reference SNAPS back to the windup MID-FOLLOW-THROUGH — an
      // untracked transition that topples the free base (the p4 backhand failure signature).
      // v4 clips have LONG follow-throughs (fh 1.46 s / bh 1.74 s vs the 0.9 s default budget).
      if (!cfg_.single_swing && cfg_.swing_rest_s < 0.0) {
        const double post_budget = (1.0 - cfg_.strike_lead_frac) * cfg_.strike_period;
        for (int c = 0; c < 2; ++c) {
          const double follow_s =
              (clip_.seg_len[c] - 1 - (clip_.strike_frame(c) - clip_.seg_start(c))) * cfg_.dt;
          if (post_budget < follow_s - 1e-9) {
            std::fprintf(stderr,
                "[pp WARN] periodic mode wraps %.2f s after the strike but clip %d's "
                "follow-through is %.2f s -> the reference will SNAP mid-follow-through "
                "(untrained; topples the free base). Run with --single-swing or --swing-rest S, "
                "or raise strike_period.\n",
                post_budget, c, follow_s);
          }
        }
      }
    } else {
      std::fprintf(stderr,
          "[pp WARN] ONNX carries NO clip layout metadata -> using the hardcoded LEGACY v1 "
          "layout seg_len={%d,%d} strike_phase={%.2f,%.2f}. Only correct for v1-clip models "
          "(model_15200); a v2-baked model will swing against the WRONG reference frames.\n",
          clip_.seg_len[0], clip_.seg_len[1], clip_.strike_phase[0], clip_.strike_phase[1]);
    }
    // 177-D hitter_footwork: resolve the per-clip base-station reach offsets. The runner
    // derives the deploy-time base STATION from the racket target as
    //   station_xy = target_xy - reach_offset_xy[clip]
    // (training base_couple_mode=reference_reach: standing AT the station puts the racket
    // target at the clip's reference reach). Prefer the ONNX-baked ref_reach_offset_xy
    // metadata (exports since 2026-07-06); else compute from the baked refs at each clip's
    // strike frame (same arithmetic as training _ensure_reference_strike_state). The station
    // channel is the whole point of the 177 contract — refuse to run without it rather than
    // silently feeding a garbage station.
    if (onnx_.obs_dim() == kObsDim177) {
      if (onnx_.has_reach_offsets()) {
        if (onnx_.reach_offsets().size() < 2)
          throw std::runtime_error(
              "pingpong: 177 model's ref_reach_offset_xy metadata does not have 2 clips");
        reach_offset_clip_[0] = onnx_.reach_offsets()[0];
        reach_offset_clip_[1] = onnx_.reach_offsets()[1];
        std::fprintf(stderr,
            "[pp] 177 hitter: reach offsets from ONNX metadata: fh=(%+.3f,%+.3f) "
            "bh=(%+.3f,%+.3f)\n",
            reach_offset_clip_[0][0], reach_offset_clip_[0][1],
            reach_offset_clip_[1][0], reach_offset_clip_[1][1]);
      } else if (onnx_.has_clip_layout()) {
        for (int c = 0; c < 2; ++c)
          reach_offset_clip_[c] = onnx_.reach_offset_from_refs(clip_.strike_frame(c));
        std::fprintf(stderr,
            "[pp WARN] 177 hitter: ONNX lacks ref_reach_offset_xy metadata -> computed from "
            "the baked refs: fh=(%+.3f,%+.3f) bh=(%+.3f,%+.3f). Re-export with the patched "
            "exporter (scripts/export_onnx_hitter.sh) to bake it.\n",
            reach_offset_clip_[0][0], reach_offset_clip_[0][1],
            reach_offset_clip_[1][0], reach_offset_clip_[1][1]);
      } else {
        throw std::runtime_error(
            "pingpong: 177 hitter model without clip layout OR reach-offset metadata — "
            "cannot derive the base station; re-export with scripts/export_onnx_hitter.sh");
      }
    }
    // 110-D hitter_pure (2026-07-07): resolve the per-side station geometry from the baked
    // sampling boxes — station_xy = target_xy − (plane_x, y_band_center)[side] (the paper's
    // §V-B-3 heuristic computes p̂_base downstream of the ball planner; here = the runner).
    // The per-clip z bands also drive the engage gate. Preference order: hitter_pure box
    // metadata (exports via scripts/export_onnx_hitter_pure.sh) → ref_reach_offset_xy
    // (numerically ≈ the box centers by construction: fh (0.699,−0.409) / bh (0.706,+0.185)
    // vs box (0.70,−0.40)/(0.70,+0.20)) → refs-FK fallback. Refuse to run blind.
    if (onnx_.is_hitter_pure_obs()) {
      if (onnx_.has_hitter_pure_boxes() && onnx_.hp_pos_boxes().size() >= 2) {
        for (int c = 0; c < 2; ++c) {
          const auto& b = onnx_.hp_pos_boxes()[c];  // {x_lo,x_hi,y_lo,y_hi,z_lo,z_hi}
          cfg_.racket_pos_w_clip[c] = Vec3(
              0.5 * (b[0] + b[1]), 0.5 * (b[2] + b[3]), 0.5 * (b[4] + b[5]));
          reach_offset_clip_[c] = Vec2(
              cfg_.racket_pos_w_clip[c][0], cfg_.racket_pos_w_clip[c][1]);
          hp_y_band_[c] = Vec2(b[2], b[3]);
          hp_z_band_[c] = Vec2(b[4], b[5]);
        }
        if (onnx_.hp_vel_boxes().size() >= 2) {  // per-clip trained vel box -> engage/stream gate
          for (int c = 0; c < 2; ++c) {
            hp_vel_box_[c] = onnx_.hp_vel_boxes()[c];
            const auto& b = hp_vel_box_[c];
            cfg_.racket_vel_w_clip[c] = Vec3(
                0.5 * (b[0] + b[1]), 0.5 * (b[2] + b[3]), 0.5 * (b[4] + b[5]));
          }
          hp_vel_box_set_ = true;
        }
        if (onnx_.hp_vel_core_boxes().size() >= 2 &&
            onnx_.hp_vel_planner_boxes().size() >= 2) {
          for (int c = 0; c < 2; ++c) {
            hp_vel_core_box_[c] = onnx_.hp_vel_core_boxes()[c];
            hp_vel_planner_box_[c] = onnx_.hp_vel_planner_boxes()[c];
          }
          hp_vel_components_set_ = true;
        }
        if (onnx_.requires_component_velocity_gate() && !hp_vel_components_set_)
          throw std::runtime_error(
              "pingpong: runtime contract v2 requires exact core/planner velocity components");
        if (cfg_.vel_cmd_box_center && hp_vel_components_set_) {
          for (int c = 0; c < 2; ++c) {
            const Vec3& center = cfg_.racket_vel_w_clip[c];
            if (!velocity_in_box(hp_vel_planner_box_[c], center[0], center[1], center[2],
                                 kVelocityBoxContractTolerance))
              throw std::runtime_error(
                  "pingpong: --demo union-box center is outside the trained planner component");
          }
        }
        // These seeds were initialized from PpPolicyConfig before ONNX geometry was resolved.
        // Refresh them now so pre-first-engage idle and --demo cannot retain stale
        // centers from a prior motion generation. ScriptedTarget reads the same updated cfg_.
        planner_frozen_pos_w_ = cfg_.racket_pos_w_clip[0];
        planner_frozen_vel_w_ = cfg_.racket_vel_w_clip[0];
        planner_hold_z_w_ = cfg_.racket_pos_w_clip[0][2];
        planner_hold_pos_b_engage_ = Vec3(
            cfg_.hold_anchor_x_b, cfg_.racket_pos_w_clip[0][1], 0.0);
        std::fprintf(stderr,
            "[pp] 110 hitter_pure: station geometry from ONNX boxes: plane_x=%.2f "
            "fh y[%.2f,%.2f] z[%.2f,%.2f]  bh y[%.2f,%.2f] z[%.2f,%.2f]\n",
            reach_offset_clip_[0][0], hp_y_band_[0][0], hp_y_band_[0][1], hp_z_band_[0][0],
            hp_z_band_[0][1], hp_y_band_[1][0], hp_y_band_[1][1], hp_z_band_[1][0],
            hp_z_band_[1][1]);
        if (hp_vel_box_set_)
          std::fprintf(stderr,
              "[pp] 110 hitter_pure: target centers from ONNX boxes: "
              "fh pos=(%+.3f,%+.3f,%+.3f) vel=(%+.3f,%+.3f,%+.3f)  "
              "bh pos=(%+.3f,%+.3f,%+.3f) vel=(%+.3f,%+.3f,%+.3f)\n",
              cfg_.racket_pos_w_clip[0][0], cfg_.racket_pos_w_clip[0][1],
              cfg_.racket_pos_w_clip[0][2], cfg_.racket_vel_w_clip[0][0],
              cfg_.racket_vel_w_clip[0][1], cfg_.racket_vel_w_clip[0][2],
              cfg_.racket_pos_w_clip[1][0], cfg_.racket_pos_w_clip[1][1],
              cfg_.racket_pos_w_clip[1][2], cfg_.racket_vel_w_clip[1][0],
              cfg_.racket_vel_w_clip[1][1], cfg_.racket_vel_w_clip[1][2]);
        if (hp_vel_components_set_)
          std::fprintf(stderr,
              "[pp] 110 hitter_pure: velocity gate uses exact core OR planner components "
              "(planner mix %.2f, ramp %d); union retained for coarse safety only\n",
              onnx_.hp_vel_planner_mix_prob(), onnx_.hp_vel_range_ramp_steps());
      } else if (onnx_.has_reach_offsets() && onnx_.reach_offsets().size() >= 2) {
        reach_offset_clip_[0] = onnx_.reach_offsets()[0];
        reach_offset_clip_[1] = onnx_.reach_offsets()[1];
        for (int c = 0; c < 2; ++c) {
          hp_y_band_[c] = Vec2(reach_offset_clip_[c][1] - 0.25, reach_offset_clip_[c][1] + 0.25);
          hp_z_band_[c] = Vec2(cfg.gate_z_lo, cfg.gate_z_hi);
        }
        std::fprintf(stderr,
            "[pp WARN] 110 hitter_pure: ONNX lacks hitter_pure box metadata -> station from "
            "ref_reach_offset_xy fh=(%+.3f,%+.3f) bh=(%+.3f,%+.3f), WIDE z gate. Re-export "
            "with scripts/export_onnx_hitter_pure.sh to bake the trained boxes.\n",
            reach_offset_clip_[0][0], reach_offset_clip_[0][1], reach_offset_clip_[1][0],
            reach_offset_clip_[1][1]);
      } else if (onnx_.has_clip_layout()) {
        for (int c = 0; c < 2; ++c) {
          reach_offset_clip_[c] = onnx_.reach_offset_from_refs(clip_.strike_frame(c));
          hp_y_band_[c] = Vec2(reach_offset_clip_[c][1] - 0.25, reach_offset_clip_[c][1] + 0.25);
          hp_z_band_[c] = Vec2(cfg.gate_z_lo, cfg.gate_z_hi);
        }
        std::fprintf(stderr,
            "[pp WARN] 110 hitter_pure: no box/reach metadata -> refs-FK fallback "
            "fh=(%+.3f,%+.3f) bh=(%+.3f,%+.3f). Re-export to bake the trained boxes.\n",
            reach_offset_clip_[0][0], reach_offset_clip_[0][1], reach_offset_clip_[1][0],
            reach_offset_clip_[1][1]);
      } else {
        throw std::runtime_error(
            "pingpong: 110 hitter_pure model without box/reach/clip metadata — cannot derive "
            "the base station; re-export with scripts/export_onnx_hitter_pure.sh");
      }
      // Idle-hold seeds on the trained manifold: ready racket at the fixed plane in front of
      // the fh band center, at the fh band-center height (hitter_pure trains NO hold — idle
      // must look like 'standing at station, next target at comfortable reach, tts pinned').
      planner_hold_pos_b_engage_ =
          Vec3(reach_offset_clip_[0][0], reach_offset_clip_[0][1], 0.0);
      planner_hold_z_w_ = 0.5 * (hp_z_band_[0][0] + hp_z_band_[0][1]);
      if (onnx_.is_rally_final_contract()) {
        if (onnx_.hp_pos_boxes().size() < 2 || !onnx_.has_hitter_pure_base_range() ||
            onnx_.hp_vel_boxes().size() < 2)
          throw std::runtime_error(
              "pingpong: Rally runtime contract requires complete hitter_pure "
              "pos/vel/base metadata; refusing an ambiguous deploy contract");
        hp_base_target_range_ = onnx_.hp_base_range();
        rally_final_station_control_ = true;
        if (onnx_.is_v17_r10_p0_gate3()) {
          if (onnx_.has_hitter_pure_station_y_step_range())
            throw std::runtime_error(
                "pingpong: V17-r10 fixed station forbids station-step metadata");
          cfg_.gate_station_step_max = 0.0;
          std::fprintf(stderr,
              "[pp] V17-r10 fixed-station contract: immutable MOTION-entry "
              "session anchor; READY is telemetry; release follows ball clock; "
              "target freezes at engage\n");
        } else {
          if (!onnx_.has_hitter_pure_station_y_step_range())
            throw std::runtime_error(
                "pingpong: moving RallyFinal contract requires station-step metadata");
          hp_station_y_step_range_ = onnx_.hp_station_y_step_range();
          // Metadata is the train/deploy contract. A CLI may tighten the legacy cap but cannot
          // silently widen a Final policy beyond its trained transition range.
          cfg_.gate_station_step_max = std::min(
              cfg_.gate_station_step_max,
              hp_station_y_step_range_[1] +
                  std::max(0.0, cfg_.gate_station_step_margin));
          std::fprintf(stderr,
              "[pp] 110 RallyFinal station contract: step_y=[%.2f,%.2f] m, gate<=%.2f m; "
              "pending readiness |dx|<=%.2f |dy|<=%.2f m speed<=%.2f m/s dwell>=%.2f s%s\n",
              hp_station_y_step_range_[0], hp_station_y_step_range_[1],
              cfg_.gate_station_step_max, cfg_.station_ready_x_max,
              cfg_.station_ready_y_max, cfg_.station_ready_speed_max,
              cfg_.station_ready_hold_s,
              cfg_.station_ready_enable ? "" : " (READINESS DISABLED by CLI)");
        }
      }
    }
    if (onnx_.is_rally_final_v3_recipe())
      std::fprintf(stderr, "[pp] hitter_pure training_recipe=rally_final_v3\n");
    if (!onnx_.runtime_contract().empty())
      std::fprintf(stderr, "[pp] hitter_pure runtime_contract=%s\n",
                   onnx_.runtime_contract().c_str());
    if (onnx_.has_finite_lateral_gait())
      std::fprintf(stderr,
          "[pp] V15 finite gait from ONNX/YAML: freq=%.2f Hz duty=%.2f deadband=%.2f m "
          "step=%.2f m cycles<=%d |vy|<=%.2f m/s; intervention deploy value=0\n",
          onnx_.gait_frequency_hz(), onnx_.gait_duty_factor(),
          onnx_.gait_move_deadband(), onnx_.gait_step_distance(),
          onnx_.gait_max_cycles(), onnx_.gait_velocity_max());
    if (onnx_.is_rally_station_recipe()) {
      stay_if_reachable_ = cfg_.stay_if_reachable_enable;
      std::fprintf(stderr,
          "[pp] hitter_pure training_recipe=%s (stay-if-reachable %s: fh y band "
          "[%.2f,%.2f] bh [%.2f,%.2f] about the held station)\n",
          onnx_.is_rally_v17_recipe() ? "rally_v17" :
          (onnx_.is_rally_v15_recipe() ? "rally_v15" :
          (onnx_.is_rally_v14_recipe() ? "rally_v14" :
          (onnx_.is_rally_v13_recipe() ? "rally_v13" :
          (onnx_.is_rally_v12_recipe() ? "rally_v12" :
          (onnx_.is_rally_v11_recipe() ? "rally_v11" :
          (onnx_.is_rally_v10_recipe() ? "rally_v10" :
          (onnx_.is_rally_v9_recipe() ? "rally_v9" : "rally_v8"))))))),
          stay_if_reachable_ ? "ON" : "OFF (CLI)",
          hp_y_band_[0][0], hp_y_band_[0][1], hp_y_band_[1][0], hp_y_band_[1][1]);
    }
    if (cfg_.fixed_station_replay)
      std::fprintf(
          stderr,
          "[pp stationary-replay] NON-CERTIFYING MUJOCO ONLY: exact "
          "V17 recipe-v1/model_16600 accepted; immutable session station, "
          "requested step <= %.3f m, full-body balance enabled\n",
          cfg_.fixed_station_tolerance_m);
    if (cfg_.fixed_y_homing_replay)
      std::fprintf(
          stderr,
          "[pp fixed-y-homing] NON-CERTIFYING MUJOCO ONLY: immutable y origin; "
          "activate |ey|>=%.3f m, finish |ey|<=%.3f m, coupled hold-target "
          "gain=%.2f, |dy_cmd|<=%.3f m; x recovery unchanged\n",
          cfg_.fixed_y_homing_enter_m, cfg_.fixed_y_homing_exit_m,
          cfg_.fixed_y_homing_gain, cfg_.fixed_y_homing_max_delta_m);
    if (cfg_.moving_station_replay)
      std::fprintf(
          stderr,
          "[pp moving-recovery] NON-CERTIFYING MUJOCO ONLY: exact "
          "V17 recipe-v1/model_16600 accepted; trained lateral station "
          "transitions and strict READY release enabled\n");
    if (onnx_.is_rally_v10_recipe() || onnx_.is_rally_v11_recipe() ||
        onnx_.is_rally_v12_recipe() || onnx_.is_rally_v13_recipe() ||
        onnx_.is_rally_v14_recipe() || onnx_.is_rally_v15_recipe() ||
        onnx_.is_rally_v17_recipe()) {
      const auto& joint_names = onnx_.joint_names();
      const auto& default_q = onnx_.default_q();
      for (std::size_t i = 0; i < joint_names.size(); ++i) {
        if (joint_names[i] == "right_elbow_joint") {
          std::fprintf(stderr,
              "[pp] hitter_pure joint_default right_elbow_joint=%.6f\n",
              default_q[static_cast<Eigen::Index>(i)]);
          break;
        }
      }
    }
    if (cfg_.station_only && !rally_final_station_control_)
      throw std::runtime_error(
          "pingpong: --station-only requires a 110-D RallyFinal runtime contract; "
          "refusing a mode that could fall through to swing engage");
    nominal_q_sdk_ = to_sdk_order(onnx_.default_q(), isaac_to_sdk_);  // nominal pose in SDK order
    qdes_projected_isaac_ = onnx_.default_q();
    leg_qdes_smooth_ = nominal_q_sdk_;  // seed the leg q_des EMA at nominal (no jump on first release)
    // Official robust-stand PD gains (a3_pd_stand_*, 29-DOF policy view) scattered
    // to the 31 SDK slots via kA3PolicyToSdkIdx; neck slots get the fixed head PD.
    official_kp_sdk_ = Eigen::VectorXd::Zero(kNumJoints);
    official_kd_sdk_ = Eigen::VectorXd::Zero(kNumJoints);
    for (int i = 0; i < robot_io::kA3PolicyDof; ++i) {
      const int sdk = robot_io::kA3PolicyToSdkIdx[i];
      official_kp_sdk_[sdk] = a3_pd_stand_kps[i];
      official_kd_sdk_[sdk] = a3_pd_stand_kds[i];
    }
    for (int s : {kHeadSlot0, kHeadSlot1}) { official_kp_sdk_[s] = kHeadKp; official_kd_sdk_[s] = kHeadKd; }
    last_q_des_ = last_q_meas_ = last_qd_meas_ = Eigen::VectorXd::Zero(kNumJoints);
  }

  // Attach a SIM-ONLY oracle pelvis-pose source (shared by main; only consulted
  // when loc_mode == kOracle). On hardware the shm file is absent so the reader
  // fails to open and oracle mode falls back with a loud warning.
  void SetOracle(std::shared_ptr<PpOraclePose> oracle) { oracle_ = std::move(oracle); }

  // Attach LIVE planner inputs (Path B). racket_in feeds the racket target; base_in feeds
  // LocMode::kExternalBase. Both are written by the AimRT subscriber thread and read here
  // from the driver thread (each is internally lock-guarded + age-gated). Only consulted
  // when cfg_.planner_mode is set; absent/stale streams degrade to a held stand.
  void SetRacketInput(std::shared_ptr<PpRacketTargetInput> r) { racket_in_ = std::move(r); }
  void SetBasePoseInput(std::shared_ptr<PpBasePoseInput> b) { base_in_ = std::move(b); }
  bool planner_mode() const { return cfg_.planner_mode; }
  std::string planner_status() const {
    std::lock_guard<std::mutex> lk(planner_mu_);
    return planner_status_;
  }

  struct PlannerTraceSnapshot {
    std::string status;
    std::string lifecycle_event;
    std::string lifecycle_reason;
    bool localization_fresh = false;
    bool base_speed_valid = false;
    double base_speed_xy = 0.0;
    bool pending_active = false;
    int pending_clip = -1;
    double pending_station_x = 0.0;
    double pending_station_y = 0.0;
    double pending_strike_time = 0.0;
    double current_strike_time = 0.0;
    double current_tts = 0.0;
    double engage_raw_tts = 0.0;
    double engage_clock_tts0 = 0.0;
    double engage_requested_phase_s = 0.0;
    double engage_actual_phase_s = 0.0;
    double engage_expected_strike_lateness_s = 0.0;
    bool late_phase_clamped = false;
    double valid_age_s = -1.0;
    bool ready_timer_active = false;
    bool ready_reported = false;
    double ready_dwell_s = 0.0;
    std::uint64_t lifecycle_seq = 0;
    std::uint64_t shot_seq = 0;
    std::uint64_t planner_msg_seq = 0;
    std::uint64_t planner_flight_id = 0;
    std::uint64_t planner_revision_id = 0;
    int planner_stable_revision_count = 0;
    std::uint64_t frozen_command_seq = 0;
    std::uint64_t frozen_flight_id = 0;
    std::uint64_t frozen_revision_id = 0;
    double frozen_strike_time = 0.0;
    double frozen_raw_tts = 0.0;
    Vec3 base_pos_w = Vec3::Zero();
    Vec4 base_quat_w = Vec4(1.0, 0.0, 0.0, 0.0);
    Vec3 target_pos_w = Vec3::Zero();
    Vec3 target_vel_w = Vec3::Zero();
    bool racket_fk_valid = false;
    Vec3 racket_pos_w = Vec3::Zero();
    Vec3 racket_vel_w = Vec3::Zero();
    Vec3 racket_normal_w = Vec3::Zero();
  };

  PlannerTraceSnapshot planner_trace_snapshot(std::uint64_t tick_idx) const {
    PlannerTraceSnapshot d;
    {
      std::lock_guard<std::mutex> lk(planner_mu_);
      d.status = planner_status_;
    }
    d.lifecycle_event = planner_lifecycle_event_;
    d.lifecycle_reason = planner_lifecycle_reason_;
    d.localization_fresh =
        (cfg_.loc_mode == LocMode::kExternalBase && base_fresh_) ||
        (cfg_.loc_mode == LocMode::kOracle && oracle_fresh_) ||
        (cfg_.loc_mode != LocMode::kExternalBase && cfg_.loc_mode != LocMode::kOracle);
    d.base_speed_valid = base_speed_xy_valid_;
    d.base_speed_xy = base_speed_xy_est_;
    d.pending_active = planner_pending_station_active_;
    d.pending_clip = planner_pending_station_active_ ? planner_pending_clip_ : -1;
    d.pending_station_x = planner_pending_station_w_[0];
    d.pending_station_y = planner_pending_station_w_[1];
    d.pending_strike_time = planner_pending_strike_time_;
    d.current_strike_time = planner_current_strike_time_;
    d.current_tts = planner_current_tts_;
    d.engage_raw_tts = planner_engage_raw_tts_;
    d.engage_clock_tts0 = planner_engage_clock_tts0_;
    d.engage_requested_phase_s = planner_engage_requested_phase_s_;
    d.engage_actual_phase_s = planner_engage_actual_phase_s_;
    d.engage_expected_strike_lateness_s =
        planner_engage_expected_strike_lateness_s_;
    d.late_phase_clamped = planner_late_phase_clamped_;
    d.valid_age_s = planner_valid_age_s_;
    d.ready_timer_active = planner_station_ready_timer_active_;
    d.ready_reported = planner_station_ready_reported_;
    d.ready_dwell_s = planner_station_ready_timer_active_
        ? (tick_idx - planner_station_ready_since_tick_) * std::max(cfg_.dt, 1e-6)
        : 0.0;
    d.lifecycle_seq = planner_lifecycle_seq_;
    d.shot_seq = planner_shot_seq_;
    d.planner_msg_seq = planner_current_msg_seq_;
    d.planner_flight_id = planner_current_flight_id_;
    d.planner_revision_id = planner_current_revision_id_;
    d.planner_stable_revision_count = planner_current_stable_revision_count_;
    d.frozen_command_seq = planner_frozen_command_seq_;
    d.frozen_flight_id = planner_frozen_flight_id_;
    d.frozen_revision_id = planner_frozen_revision_id_;
    d.frozen_strike_time = planner_frozen_strike_time_;
    d.frozen_raw_tts = planner_frozen_raw_tts_;
    d.base_pos_w = last_base_pos_;
    d.base_quat_w = last_base_quat_w_;
    d.target_pos_w = last_target_pos_w_;
    d.target_vel_w = last_target_vel_w_;
    d.racket_fk_valid = last_racket_fk_valid_;
    d.racket_pos_w = last_racket_pos_w_;
    d.racket_vel_w = last_racket_vel_w_;
    d.racket_normal_w = last_racket_normal_w_;
    return d;
  }

  LocMode loc_mode() const { return cfg_.loc_mode; }
  const char* loc_mode_name() const {
    switch (cfg_.loc_mode) {
      case LocMode::kFabricated: return "fabricated(A)";
      case LocMode::kPerfectTracking: return "perfect_tracking(B)";
      case LocMode::kOracle: return "oracle(C)";
      case LocMode::kExternalBase: return "external_base(mocap)";
    }
    return "?";
  }

  // --- obs-debug snapshot (full 180-D obs + flags), read by the status thread ---
  struct ObsDebug {
    Eigen::VectorXd obs;          // last 180-D observation
    bool valid = false;
    bool oracle_enabled = false;  // loc_mode == kOracle
    bool oracle_fresh = false;    // a fresh oracle sample was used this tick
    double oracle_age_s = -1.0;   // age of last oracle sample (s), -1 if n/a
    std::uint64_t sync_miss = 0;  // ticks seen with sync_aligned == false (cumulative)
  };
  ObsDebug take_obs_debug() {
    std::lock_guard<std::mutex> lk(obs_mu_);
    ObsDebug d;
    d.obs = last_obs_;
    d.valid = last_obs_.size() == onnx_.obs_dim();
    d.oracle_enabled = (cfg_.loc_mode == LocMode::kOracle);
    d.oracle_fresh = oracle_fresh_;
    d.oracle_age_s = oracle_age_s_;
    d.sync_miss = sync_miss_;
    return d;
  }
  const Eigen::VectorXd& last_obs_unsafe() const { return last_obs_; }

  // --- live per-joint diagnostics (SDK order), read by the status thread ---
  struct DiagSnapshot {
    Eigen::VectorXd q_des, q_meas, qd_meas;                    // instantaneous
    Eigen::VectorXd des_range, meas_range, err_peak, qd_peak;  // over the window
    bool valid = false;                                        // window had samples
  };
  // Copy out diagnostics and reset the rolling window. Thread-safe.
  DiagSnapshot take_diag() {
    std::lock_guard<std::mutex> lk(diag_mu_);
    DiagSnapshot d;
    d.q_des = last_q_des_; d.q_meas = last_q_meas_; d.qd_meas = last_qd_meas_;
    if (ranges_init_) {
      d.des_range = des_hi_ - des_lo_;
      d.meas_range = meas_hi_ - meas_lo_;
      d.err_peak = err_peak_;
      d.qd_peak = qd_peak_;
      d.valid = true;
    }
    ranges_init_ = false;  // start a fresh window
    return d;
  }

  // Official robust stand (matches AGI's PD_STAND): pose = nominal (== a3_default_angles),
  // gains = production a3_pd_stand_*. All in 31-DOF SDK order.
  const Eigen::VectorXd& official_stand_q() const { return nominal_q_sdk_; }
  const Eigen::VectorXd& official_stand_kp() const { return official_kp_sdk_; }
  const Eigen::VectorXd& official_stand_kd() const { return official_kd_sdk_; }

  // Runtime swing level: 0 = hold wind-up (quasi-stand), 1 = periodic forehand.
  // Any EXTERNAL level change (keyboard, safety guard) cancels a pending swing-rest
  // auto re-arm — a guard trip must never re-enter the swing on its own.
  void set_level(int lvl) { rest_rearm_armed_.store(false); level_.store(lvl); }
  int level() const { return level_.load(); }

  // Live swing-speed tuning (real-time stretch; <1.0 slower). Clamped to a sane range.
  void set_swing_speed(double s) { swing_speed_.store(std::max(0.05, std::min(2.0, s))); }
  double swing_speed() const { return swing_speed_.load(); }

  // Live swing DIRECTION (scripted test path; no live planner). +1 = forehand
  // (target -y, baked clip 0), -1 = backhand (target +y, baked clip 1). Flips the
  // scripted target's y-sign and swing_type; the reference clock then selects the
  // matching baked clip via clip_id_from_swing_sign.
  //
  // MID-SWING LATCH (2026-07-04): applying a dir flip while a swing is in progress
  // snaps the 62-D reference obs from clip A's mid-swing frame to clip B's windup
  // while the BODY is mid-swing — the exact OOD transition training never contains
  // (clips only switch at a completed wrap + hold; see the Python runner's
  // active-swing lock and the free-base 'b'-key falls). At level 1 the request is
  // QUEUED and applied at the next safe boundary (level 0, or the next windup start
  // of the periodic/single-swing clock) in ComputeCommand.
  void set_swing_dir(int d) {
    const int want = d >= 0 ? 1 : -1;
    if (level_.load() == 1 && swing_dir_.load() != want) {
      pending_swing_dir_.store(want);
      std::fprintf(stderr, "[pp] swing dir -> %s QUEUED (mid-swing switch is OOD; "
                   "applies at the next windup/hold)\n", want > 0 ? "FOREHAND" : "BACKHAND");
      return;
    }
    swing_dir_.store(want);
  }
  int swing_dir() const { return swing_dir_.load(); }
  const char* swing_dir_name() const { return swing_dir_.load() >= 0 ? "FOREHAND" : "BACKHAND"; }

  // Re-capture the yaw-align offsets on the next policy tick. Called by the driver
  // whenever the mode transitions INTO SHADOW/MOTION from PASSIVE/PD_STAND (the robot
  // may have been turned/moved between engagements). Also drops the 177-D hold-station
  // anchor so it re-captures at the robot's NEW spot (a stale anchor from before the
  // move would command a walk back to the old position), and restarts the arm-hold
  // sustained-quiet clock (--arm-hold-nominal) for the fresh MOTION entry.
  void rearm_yaw_align() {
    yaw_align_pending_.store(true);
    hold_station_set_ = false;
    arm_quiet_ticks_ = 0;    // fresh MOTION entry: restart the arm-hold sustained-quiet clock
    arm_hold_armed_ = true;  // ...and re-arm the pre-swing arm hold
    // Never carry a previous rally's recurrent action history across a
    // non-policy controller.  V17/V11 affine trained its static handoff with
    // previous action == 0; bounded contracts replace this zero with measured
    // executed-q_des feedback on the actual first policy tick below.
    const RuntimeHandoffReset handoff =
        runtime_handoff_reset(onnx_.has_bounded_qdes_contract());
    last_action_.setZero();
    last_action_seed_pending_ = handoff.seed_measured_qdes_feedback;
    if (handoff.seed_measured_qdes_feedback) {
      qdes_projector_initialized_ = false;
      // ENGAGE FEEDBACK SEED (2026-07-23): training's first post-reset tick reports the
      // MEASURED posture feedback, never a zero action history. Arm the seed; it is taken
      // from the measured joint state on the next policy tick (zeros are kept only until a
      // valid joint state has been received).
    }
    // PLANNER-MODE swing-state reset (2026-07-08): SHADOW and MOTION run ComputeCommand on
    // DIFFERENT clock domains (SHADOW = a free-running local counter, MOTION = the publish-
    // gated driver tick). A swing engaged during a SHADOW preview leaves level 1 + a
    // swing_clock_origin_ from the SHADOW clock; entering MOTION then resumes that phantom
    // swing against an incoherent clock (tts frozen or deeply negative — engage locked out
    // or an instant snap). Every SHADOW/MOTION entry from PASSIVE/PD_STAND must start from
    // a clean stand: level 0, no engage latch, no stale rest timer. The next valid planner
    // command re-engages normally. (Scripted mode is untouched — the operator owns level.)
    if (cfg_.planner_mode) {
      pending_swing_dir_.store(0);
      rest_rearm_armed_.store(false);
      level_.store(0);
      planner_engaged_ = false;
      planner_pending_station_active_ = false;
      planner_station_ready_timer_active_ = false;
      planner_station_ready_reported_ = false;
      planner_pending_strike_time_ = 0.0;
      planner_current_strike_time_ = 0.0;
      planner_current_tts_ = 0.0;
      planner_engage_raw_tts_ = 0.0;
      planner_engage_clock_tts0_ = 0.0;
      planner_engage_requested_phase_s_ = 0.0;
      planner_engage_actual_phase_s_ = 0.0;
      planner_engage_expected_strike_lateness_s_ = 0.0;
      planner_late_phase_clamped_ = false;
      planner_frozen_command_seq_ = 0;
      planner_frozen_flight_id_ = 0;
      planner_frozen_revision_id_ = 0;
      planner_frozen_strike_time_ = 0.0;
      planner_frozen_raw_tts_ = 0.0;
      planner_valid_age_s_ = -1.0;
      RecordPlannerLifecycle_("clear", "mode_rearm");
      prefirst_active_station_tracking_started_ = false;
      base_motion_initialized_ = false;
      base_speed_xy_valid_ = false;
      base_velocity_xy_valid_ = false;
      base_velocity_xy_est_.setZero();
      ResetFiniteLateralGait_();
      station_session_origin_set_ = false;
      planner_static_active_ = false;
      static_settle_ticks_ = 0;
      planner_policy_takeover_active_ = false;
      planner_entry_pending_.store(true);  // restart the engage settle clock (engage_settle_s)
      fixed_y_homing_active_ = false;
      fixed_y_homing_start_tick_ = 0;
      fixed_y_homing_peak_error_m_ = 0.0;
      fixed_y_homing_activation_count_ = 0;
      fixed_y_homing_completion_count_ = 0;
    }
    serve_static_handoff_pending_ = false;
  }

  // A serve is a stronger transition than a generic mode re-entry.  The
  // deterministic clip has already held the exact V17 default for 0.5 s, so
  // preserve that command on the first policy tick and force the planner's
  // sticky static hold before any learned action can run.  This makes the
  // q_des boundary exactly continuous while retaining the policy's own
  // planner/observation reset path.
  void rearm_static_policy_handoff() {
    rearm_yaw_align();
    const RuntimeHandoffReset handoff =
        runtime_handoff_reset(onnx_.has_bounded_qdes_contract());
    serve_static_handoff_pending_ = handoff.force_exact_default_static;
    last_q_des_ = nominal_q_sdk_;
    if (cfg_.auto_leg_hold) {
      legs_passive_.store(true);
      waist_passive_.store(true);
    }
  }

  // Full-body gate: true => leg q_des is overwritten to nominal (NOT a full-body
  // test); false => the policy's leg actions pass through (31-DOF command check).
  // Atomic so --auto-leg-hold can flip it per-tick from the driver thread while the
  // status thread reads it. Initialised from cfg in the constructor.
  bool legs_passive() const { return legs_passive_.load(); }
  bool waist_passive() const { return waist_passive_.load(); }
  void set_legs_passive(bool v) { legs_passive_.store(v); }
  void set_waist_passive(bool v) { waist_passive_.store(v); }

  PpRacketTarget ScriptedTarget(std::uint64_t tick_idx) const {
    PpRacketTarget tg;
    const int dir = swing_dir_.load();  // +1 forehand (clip0) / -1 backhand (clip1)
    const int clip = dir >= 0 ? 0 : 1;
    // Per-clip target from the trained sampling boxes (see PpPolicyConfig) — no mirroring.
    tg.pos_w = cfg_.racket_pos_w_clip[clip];
    tg.vel_w = cfg_.racket_vel_w_clip[clip];
    tg.swing_sign = (dir >= 0 ? 1.0 : -1.0);  // +1 fore / -1 back
    tg.base_target_xy = Vec2::Zero();
    // Swing clock measured from the origin set on each level->1 entry (see ComputeCommand),
    // so a release from a long level-0 hold starts the swing at the WINDUP (matching the held
    // pose) instead of snapping into the free-running mid-cycle phase, which would mismatch the
    // body and lurch the robot. swing_speed<1 stretches the clock.
    const std::uint64_t origin = swing_clock_origin_.load();
    const double t = (tick_idx >= origin ? tick_idx - origin : 0) * cfg_.dt * swing_speed_.load();
    if (level_.load() == 0) {
      tg.time_to_strike = 5.0;  // far away -> clock holds at clip start (wind-up)
    } else if (cfg_.planner_mode) {
      // LIVE PLANNER: linear clock seeded from the ENGAGE-time tts (clamped to the clip's
      // windup length at engage) so the reference strike aligns with the ball's arrival.
      // Same no-wrap semantics as single_swing; completion still trips on tts < min_tts.
      tg.time_to_strike = planner_tts0_ - t;
    } else if (cfg_.single_swing || cfg_.swing_rest_s >= 0.0) {
      // SINGLE-SWING: linear clock, NO fmod wrap. The periodic schedule bounds tts to
      // [-(1-lead)*period, lead*period] = [-0.9, 2.1], which (a) never reaches the clip's
      // end (backhand needs tts=-1.76) so the follow-through frames 227..270 never play,
      // and (b) SNAPS the reference end->windup every period. Linear tts plays the WHOLE
      // clip once; ComputeCommand then drops to level 0 when the clip has fully played.
      tg.time_to_strike = cfg_.strike_lead_frac * cfg_.strike_period - t;
    } else {
      const double cyc = std::fmod(t, cfg_.strike_period);
      tg.time_to_strike = cfg_.strike_lead_frac * cfg_.strike_period - cyc;  // windup->strike->follow-through
    }
    return tg;
  }

  // CommandFn body. Fills a full 31-slot RobotCommand (SDK order). Always valid.
  bool ComputeCommand(std::uint64_t tick_idx, const robot_io::RobotState& state,
                      robot_io::RobotCommand& cmd) {
    // LIVE PLANNER (Path B): decide engage/hold from the latest planner command and drive
    // the EXISTING swing controls (set_swing_dir/set_level + freeze). Runs before the swing
    // clock logic so the 0->1 edge below resets the clock to the windup as usual. No-op in
    // the scripted/keyboard path (planner_mode == false).
    if (cfg_.planner_mode && planner_entry_pending_.exchange(false))
      planner_entry_tick_ = tick_idx;  // first tick of this SHADOW/MOTION session (settle clock)
    if (cfg_.planner_mode) PlannerEngageStep_(tick_idx);
    // Reset the swing clock to its windup on level 0->1 (release from hold) OR on a
    // forehand<->backhand switch. Either way the swing must (re)start from its WINDUP
    // (tts -> clip start, matching the current near-stand body) rather than snap into the
    // free-running mid-cycle phase. Pressing 'b' mid-forehand WITHOUT this reset jumps the
    // backhand reference straight to a mid-swing frame while the body is still in a
    // forehand-end pose -> reference/body mismatch -> lurch -> FALL (forehand is fine only
    // because it gets this clean windup start at MOTION entry).
    // Apply a QUEUED dir flip (set_swing_dir latch) only at a safe boundary: held stand,
    // or while the swing clock still sits at the windup start (tts clamped at max — early
    // cycle / just released). There the flip re-selects the OTHER clip's windup, the same
    // reference-pose family the clock reset produces anyway; mid-swing it would snap the
    // obs reference across clips (the 'b'-mid-forehand OOD fall).
    {
      const int pend = pending_swing_dir_.load();
      if (pend != 0 && (level_.load() == 0 || last_tts_at_windup_)) {
        pending_swing_dir_.store(0);
        swing_dir_.store(pend);
        std::fprintf(stderr, "[pp] queued swing dir applied -> %s\n",
                     pend > 0 ? "FOREHAND" : "BACKHAND");
      }
    }
    const int swing_lvl_now = level_.load();
    const int swing_dir_now = swing_dir_.load();
    if ((swing_lvl_now == 1 && swing_level_prev_ != 1) || swing_dir_now != swing_dir_prev_)
      swing_clock_origin_.store(tick_idx);
    // ANY 1->0 edge restarts the planner post-swing recovery clock, not just the normal
    // completion path (which also sets it, idempotently). Without this, an EXTERNAL
    // set_level(0) mid-swing (squat/tilt guard, operator key) leaves the clock stale from
    // the PREVIOUS hold — post_recovery reads instantly true and the stiff static stand
    // freezes onto a tilted, still-moving robot, skipping the policy recovery entirely.
    if (cfg_.planner_mode && swing_lvl_now == 0 && swing_level_prev_ == 1)
      planner_hold_start_tick_ = tick_idx;
    swing_level_prev_ = swing_lvl_now;
    swing_dir_prev_ = swing_dir_now;
    PpRacketTarget tg = ScriptedTarget(tick_idx);
    const int clip_id = clip_id_from_swing_sign(tg.swing_sign);
    // Clamp time_to_strike to the clip's IN-TRAINING maximum. Training computes
    // tts = (strike_frame - current_frame)*dt from the actual clip frame, so its max is
    // (strike_frame - seg_start)*dt (backhand 0.86 s, forehand 1.30 s). The scripted schedule
    // instead feeds raw 2.1 s at cycle start / 5.0 s at hold — an OOD (tts, windup-frame)
    // pairing the policy never saw (worst for backhand: 1.24 s of OOD input right before the
    // swing; observed to precede the free-base backhand fall). Clamping makes the windup state
    // exactly the training state "at windup frame, tts=max". The reference clock is unaffected
    // (it already clamps ts to seg_start for any tts >= this bound).
    const double max_tts =
        (clip_.strike_frame(clip_id) - clip_.seg_start(clip_id)) * clip_.step_dt;
    if (tg.time_to_strike > max_tts) tg.time_to_strike = max_tts;
    last_tts_at_windup_ = (tg.time_to_strike >= max_tts - 1e-9);
    // SINGLE-SWING / REST (see PpPolicyConfig): once the clip has fully played, drop to
    // level 0 (held stand) instead of letting the periodic clock WRAP the reference from
    // the end pose back to windup (an untracked-in-training snap that topples the backhand).
    // min_tts = tts at the clip's last frame; below it the clock is clamped at the end.
    if ((cfg_.single_swing || cfg_.swing_rest_s >= 0.0) && swing_lvl_now == 1) {
      const double min_tts = (clip_.strike_frame(clip_id) -
                              (clip_.seg_start(clip_id) + clip_.seg_len[clip_id] - 1)) *
                             clip_.step_dt;
      if (tg.time_to_strike < min_tts) {
        level_.store(0);
        if (cfg_.planner_mode) planner_hold_start_tick_ = tick_idx;  // recovery-window clock
        if (cfg_.swing_rest_s >= 0.0) {
          rest_rearm_tick_ = tick_idx + static_cast<std::uint64_t>(
              std::max(0.0, cfg_.swing_rest_s) / std::max(cfg_.dt, 1e-6));
          rest_rearm_armed_ = true;
        }
        std::fprintf(stderr, "[pp] swing complete -> level 0 (held stand)%s\n",
                     cfg_.swing_rest_s >= 0.0 ? " (auto re-arm after rest)" : "; press 1 to swing again");
      }
    }
    // Auto re-arm after the rest (only if WE dropped the level; a manual '0' clears it).
    // NOT in planner mode: there a swing re-engages only on a fresh VALID command
    // (PlannerEngageStep_); rest_rearm_tick_ is reused there purely as the rest timer.
    if (!cfg_.planner_mode && rest_rearm_armed_ && level_.load() == 0 &&
        tick_idx >= rest_rearm_tick_) {
      rest_rearm_armed_ = false;
      level_.store(1);  // next tick's 0->1 edge resets the swing clock to windup
    }
    // MIN-side OBS clamp (2026-07-04): the reference clock clamps the FRAME at the clip
    // end, but the raw tts kept decreasing into values training never paired with the
    // frozen end frame (periodic mode with a raised strike_period). Clamp the OBS tts to
    // the in-training minimum, symmetric with the max clamp above. AFTER the completion
    // check on purpose — that check needs the raw sub-minimum tts to detect clip end.
    {
      const double min_tts_clip = (clip_.strike_frame(clip_id) -
                                   (clip_.seg_start(clip_id) + clip_.seg_len[clip_id] - 1)) *
                                  clip_.step_dt;
      if (tg.time_to_strike < min_tts_clip) tg.time_to_strike = min_tts_clip;
    }
    const int time_step = clip_.time_step_for(clip_id, tg.time_to_strike);

    PpRefs refs = onnx_.refs(time_step);
    // HOLD = a STATIONARY reference (2026-07-05, train==deploy lockstep): clip frame 0
    // is a mid-crouch TRANSIENT (knee +7.8 rad/s, torso -1.11 m/s down) — feeding its
    // raw velocities through the whole hold taught the policy to fight a phantom squat
    // (the Gate 2.5 P2 3-5 s bare-hold tip). Training now zeroes the reference joint
    // velocities on held envs (commands.py joint_vel); mirror it in every policy-hold
    // state (level 0 = scripted hold AND the planner post-swing recovery hold).
    if (level_.load() == 0) {
      refs.joint_vel.setZero();
      // ...and the hold JOINT reference is the READY STAND, not the windup crouch
      // (2026-07-05 lockstep with training commands.joint_pos): frame 0 is an
      // asymmetric mid-crouch — imitating it during hold produced the splayed-feet
      // crouch-stand. The release into the swing is the trained stand_start regime.
      refs.joint_pos = onnx_.default_q();
    }

    if (!state.sync_aligned) ++sync_miss_;  // dropped/unaligned state packet count

    PpRobotState st;
    st.q = from_sdk_order(state.q, isaac_to_sdk_);    // SDK -> Isaac
    st.qd = from_sdk_order(state.dq, isaac_to_sdk_);
    if (st.q.size() != kNumJoints || st.qd.size() != kNumJoints ||
        !PpAllFinite(st.q) || !PpAllFinite(st.qd)) {
      throw std::runtime_error(
          "ping-pong measured joint state has wrong size or contains NaN/Inf");
    }
    for (int i = 0; i < kNumJoints; ++i) {
      const int sdk = isaac_to_sdk_[i];
      const bool exported_limit_contract =
          onnx_.has_safe_qdes_interval_contract();
      const double lo = exported_limit_contract ? onnx_.qdes_hard_lo()[i]
                                                : kSdkJointPosLo[sdk];
      const double hi = exported_limit_contract ? onnx_.qdes_hard_hi()[i]
                                                : kSdkJointPosHi[sdk];
      const double tolerance = exported_limit_contract
          ? onnx_.qdes_actual_q_hard_tolerance_rad() : 0.0;
      const auto disposition = classify_actual_q_hard_limit(
          st.q[i], lo, hi, tolerance,
          exported_limit_contract && onnx_.qdes_actual_q_hard_audit_only());
      if (disposition == ActualQHardLimitDisposition::kFault) {
        throw std::runtime_error(
            "ping-pong PHYSICAL SAFETY FAULT: measured q exceeds hard limit for joint '" +
            onnx_.joint_names()[i] + "' (q=" + std::to_string(st.q[i]) +
            ", interval=[" + std::to_string(lo) + "," +
            std::to_string(hi) + "], tolerance=" +
            std::to_string(tolerance) + ")");
      }
      if (disposition == ActualQHardLimitDisposition::kTelemetry) {
        if (!actual_q_hard_audit_active_[i]) {
          std::fprintf(
              stderr,
              "[pp actual-q audit] joint '%s' measured q=%+.6f exceeds "
              "exported interval=[%+.6f,%+.6f] tolerance=%.6f; "
              "audit-only, runner remains active\n",
              onnx_.joint_names()[i].c_str(), st.q[i], lo, hi, tolerance);
        }
        actual_q_hard_audit_active_[i] = true;
      } else if (actual_q_hard_audit_active_[i]) {
        std::fprintf(
            stderr,
            "[pp actual-q audit] joint '%s' recovered inside the exported "
            "hard-limit tolerance; runner remained active\n",
            onnx_.joint_names()[i].c_str());
        actual_q_hard_audit_active_[i] = false;
      }
    }
    st.base_quat_w = state.imu_quat_wxyz;             // real pelvis IMU orientation
    st.base_ang_vel_b = state.imu_gyro;               // real pelvis gyro (body frame)
    // torso ORIENTATION from the real secondary (torso) IMU when available
    // (identity was wrong -> broke the anchor term -> robot fell). Measurable in
    // every mode, so always use the real value.
    st.torso_quat_w = state.has_secondary_imu ? state.sec_imu_quat_wxyz
                                              : cfg_.nominal_torso_quat_w;

    const bool mocap_authoritative =
        onnx_.uses_authoritative_mocap_pose() &&
        cfg_.loc_mode == LocMode::kExternalBase;

    // YAW-ALIGN (see PpPolicyConfig::yaw_align). Capture each IMU's yaw on the first
    // policy tick after (re)engage, then express every subsequent attitude relative to
    // that entry heading. Fixes the boot-drift yaw polluting motion_anchor_ori_b and the
    // racket-FK world conversion on hardware; no-op in sim where spawn yaw ~ 0.
    // V17 external-base has a calibrated absolute mocap heading. An engage-time
    // IMU yaw reset would silently rotate that canonical world frame.
    if (cfg_.yaw_align && !mocap_authoritative) {
      if (yaw_align_pending_.load()) {
        // UPRIGHT + STATIONARY GUARD (2026-07-04): capturing while the robot is tilted,
        // turning, or fallen bakes a garbage offset into EVERY subsequent obs (yaw of a
        // fallen quat is ill-defined; observed in the ROS runner: all base-relative
        // targets rotated ~125 deg, magnitude untouched). Defer the capture until the
        // robot is upright (proj gravity ~[0,0,-1]) and still (|gyro| small); warn while
        // waiting so a hoisted/leaning engage is visible instead of silently wrong.
        // body-frame gravity z from the raw base quat (w,x,y,z): R(q)^T·[0,0,-1] |_z
        const double gz = 2.0 * (st.base_quat_w[1] * st.base_quat_w[1] +
                                 st.base_quat_w[2] * st.base_quat_w[2]) - 1.0;
        const double gyro_n = st.base_ang_vel_b.norm();
        if (gz > -0.95 || gyro_n > 0.5) {
          if (++yaw_align_defer_ticks_ % 50 == 1) {
            std::fprintf(stderr,
                "[pp WARN] yaw-align DEFERRED: robot not upright/still (gravZ=%+.2f "
                "|gyro|=%.2f); stand the robot at its heading to capture.\n", gz, gyro_n);
          }
        } else {
          yaw_align_pending_.store(false);
          yaw_align_defer_ticks_ = 0;
          yaw0_base_inv_ = quat_inv(yaw_quat(st.base_quat_w));
          yaw0_torso_inv_ = quat_inv(yaw_quat(st.torso_quat_w));
        const auto yaw_deg = [](const Vec4& q) {
          return std::atan2(2.0 * (q[0] * q[3] + q[1] * q[2]),
                            1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3])) * 180.0 / M_PI;
        };
        std::fprintf(stderr,
            "[pp] yaw-align captured at policy engage: base_yaw=%+.1f deg torso_yaw=%+.1f deg "
            "(subtracted from all subsequent IMU attitudes; robot heading at engage == clip +x)\n",
            yaw_deg(st.base_quat_w), yaw_deg(st.torso_quat_w));
        }
      }
      st.base_quat_w = quat_mul(yaw0_base_inv_, st.base_quat_w);
      st.torso_quat_w = quat_mul(yaw0_torso_inv_, st.torso_quat_w);
    }
    if (!state.has_secondary_imu && !sec_imu_warned_ &&
        !mocap_authoritative) {
      sec_imu_warned_ = true;
      std::fprintf(stderr,
          "[pp WARN] secondary (torso) IMU ABSENT -> torso orientation falls back to "
          "identity; motion_anchor_ori_b (and the anchor frame feeding "
          "motion_anchor_pos_b) will be WRONG. Do NOT run MOTION on hardware without "
          "a working torso IMU.\n");
    }

    // --- localization-dependent world pose (3 modes; obs LAYOUT unchanged) ---
    oracle_fresh_ = false;
    oracle_age_s_ = -1.0;
    std::uint64_t localization_seq = 0;
    switch (cfg_.loc_mode) {
      case LocMode::kOracle: {  // ===== C: SIMULATION ONLY (true MuJoCo pose) =====
        PpOracleSample s;
        if (oracle_ && oracle_->Latest(s, cfg_.oracle_max_age_s)) {
          st.base_pos_w = s.pos;       // true world pelvis position
          st.base_quat_w = s.quat;     // true world pelvis orientation
          oracle_fresh_ = true;
          localization_seq = s.seq;
          oracle_age_s_ = s.age_s;
          st.torso_pos_w = torso_pos_from_base(st.base_pos_w, st.base_quat_w, st.q);
        } else {  // stale/missing oracle -> SAFE fallback to perfect-tracking, warn LOUDLY
          // 2026-07-03: this used to warn ONCE and then silently degrade — an --oracle-pelvis
          // A/B run with the bridge down produced a perfect_tracking run in disguise (the two
          // "different" loc-mode tests were identical). Now: repeat the warning every ~2 s and
          // mark the fallback in oracle_fresh_ so the [obs] status line shows fresh=0.
          oracle_fresh_ = false;
          if ((oracle_warn_tick_++ % 100) == 0) {
            std::fprintf(stderr,
                "[pp ORACLE] NO FRESH SAMPLE (bridge down / stale shm?) -> running as "
                "perfect-tracking. This is NOT an oracle run — start "
                "scripts/run_oracle.sh first and require 'fresh=1' in the [obs] line.\n");
          }
          st.base_pos_w = refs.ref_pelvis_pos_w;
          st.torso_pos_w = refs.anchor_pos_w;
        }
        break;
      }
      case LocMode::kPerfectTracking: {  // ===== B: assume perfect position tracking =====
        // racket/base-target relative to where the pelvis SHOULD be (the reference),
        // and zero the anchor POSITION error so the policy is not fed a fictional
        // world-tracking error. Orientation terms stay real (IMU above).
        st.base_pos_w = refs.ref_pelvis_pos_w;
        st.torso_pos_w = refs.anchor_pos_w;   // -> motion_anchor_pos_b == 0
        break;
      }
      case LocMode::kExternalBase: {  // ===== HARDWARE planner: calibrated mocap base POSE =====
        PpBaseSample s;
        if (base_in_ &&
            base_in_->Latest(
                s, cfg_.external_base_max_age_s,
                /*require_authoritative=*/mocap_authoritative)) {
          st.base_pos_w = s.pos;
          if (s.authoritative) {
            // The high-rate pelvis gyro only propagates the most recent mocap
            // attitude over a short transport interval. It never supplies the
            // absolute yaw or survives a long mocap dropout.
            const double propagation_s =
                std::clamp(s.age_s, 0.0,
                           cfg_.external_base_gyro_propagation_max_s);
            st.base_quat_w = quat_integrate_body_rate(
                s.quat, st.base_ang_vel_b, propagation_s);
          }
          if (mocap_authoritative)
            st.torso_quat_w = st.base_quat_w;  // torso IMU is non-load-bearing in 110-D
          st.torso_pos_w = torso_pos_from_base(st.base_pos_w, st.base_quat_w, st.q);
          base_fresh_ = true;
          have_last_external_base_ = true;
          last_external_base_pos_w_ = s.pos;
          last_external_base_quat_w_ = st.base_quat_w;
          localization_seq = s.seq;
          oracle_age_s_ = s.age_s;
        } else {  // stale/absent mocap -> hold last measured base + loud warn; never invent reference pose.
          base_fresh_ = false;
          if ((base_warn_tick_++ % 100) == 0) {
            const bool policy_native_stale_is_advisory =
                cfg_.policy_native &&
                !cfg_.fixed_station_replay &&
                !cfg_.moving_station_replay;
            std::fprintf(stderr,
                "[pp EXT-BASE] NO FRESH authoritative mocap base pose "
                "(relay/stamp/calibration/staleness fault) -> holding the last "
                "mocap pose; %s.\n",
                policy_native_stale_is_advisory
                    ? "policy-native MOTION and Planner release continue"
                    : "new planner engage is blocked");
          }
          st.base_pos_w = have_last_external_base_ ? last_external_base_pos_w_ : last_base_pos_;
          if (mocap_authoritative) {
            st.base_quat_w = have_last_external_base_
                ? last_external_base_quat_w_ : last_base_quat_w_;
            st.torso_quat_w = st.base_quat_w;
          }
          st.torso_pos_w = torso_pos_from_base(st.base_pos_w, st.base_quat_w, st.q);
        }
        break;
      }
      case LocMode::kFabricated:  // ===== A: legacy fabricated nominal pose =====
      default: {
        st.base_pos_w = cfg_.nominal_base_pos_w;
        if (cfg_.use_base_estimator)  // leg-FK + IMU pelvis height (planted-foot stance)
          st.base_pos_w[2] = estimate_base_height(st.q, st.base_quat_w);
        // torso POSITION = base + waist-FK offset (~base + 5 mm up).
        st.torso_pos_w = torso_pos_from_base(st.base_pos_w, st.base_quat_w, st.q);
        break;
      }
    }
    const bool localized_base =
        (cfg_.loc_mode == LocMode::kOracle && oracle_fresh_) ||
        (cfg_.loc_mode == LocMode::kExternalBase && base_fresh_);
    UpdateBaseMotion_(tick_idx, st.base_pos_w, localized_base, localization_seq);
    st.base_velocity_xy_w = base_velocity_xy_valid_ ? base_velocity_xy_est_ : Vec2::Zero();
    if (onnx_.uses_position_mocap_obs()) {
      const double max_age = onnx_.base_localization_max_age_s();
      st.localization_age = (localized_base && oracle_age_s_ >= 0.0 && max_age > 0.0)
          ? std::clamp(oracle_age_s_ / max_age, 0.0, 1.0)
          : 1.0;
    }
    if (rally_final_station_control_ && localized_base && !station_session_origin_set_) {
      station_session_origin_w_ = Vec2(st.base_pos_w[0], st.base_pos_w[1]);
      station_session_origin_set_ = true;
      std::fprintf(stderr,
          "[pp station] session origin=(%+.3f,%+.3f); absolute trained box "
          "x[%.2f,%.2f] y[%.2f,%.2f]\n",
          station_session_origin_w_[0], station_session_origin_w_[1],
          hp_base_target_range_[0], hp_base_target_range_[1],
          hp_base_target_range_[2], hp_base_target_range_[3]);
    }
    last_base_pos_ = st.base_pos_w;
    last_base_quat_w_ = st.base_quat_w;  // yaw-aligned; PlannerEngageStep_ gates on it next tick

    // LIVE PLANNER static stand at level 0 — in TWO regimes:
    //   (a) pre-FIRST-engage (the Python runner's proven _stand-until-engage design;
    //       running the policy hold from a cold stand knelt the robot within ~2 s), and
    //   (b) POST-RECOVERY: hold_recover_s after a completed swing. The policy hold must
    //       run first (it actively balances out of the follow-through — a static stand
    //       cannot), but the model's level-0 hold only has ~5 s of margin (Gate 2.5 +
    //       closed-loop falls), so after the recovery window we blend to the static
    //       official stand and stay there until the next engage.
    // Localization/engage above still run every tick; an engage (level 0->1) exits this
    // branch and main's blend covers the stand -> swing transition (the Gate-2-proven
    // MOTION-entry path). q_des ramps measured -> nominal over hold_blend_s so the stiff
    // official gains never snap onto a displaced pose (the kp-2000 catapult class).
    // Policy-native field mode deliberately leaves the learned level-0 policy in
    // control before and after a strike; no fixed-pose handoff is inserted.
    if (!cfg_.policy_native) {
      // Handoff is QUIESCENCE-GATED, not time-only: a timed switch fell 0.6 s after the
      // blend began (the robot still carried follow-through momentum — the documented
      // "blended static stand cannot balance out of the follow-through" failure). The
      // policy hold keeps actively balancing until the robot is upright AND still; a
      // force-switch at recover+3 s bounds the stay inside the fragile ~5-10 s window.
      const double t_since =
          (tick_idx - planner_hold_start_tick_) * cfg_.dt;
      const bool upright_still =
          projected_gravity_body(st.base_quat_w)[2] < -0.95 &&
          st.base_ang_vel_b.norm() < 0.4 &&
          (st.qd.size() == 0 || st.qd.cwiseAbs().maxCoeff() < 1.0);
      // BASE-SETTLE guard (2026-07-11, rally-gate no-engage-transition fall): never hand
      // the stiff STATIC stand a robot whose base is still translating. near_station goes
      // true at the END of a station walk while the base still carries ~0.5 m/s (the hold
      // anchor was already re-stamped to the NEW station at gate-accept), and the +3 s
      // force path below checks no robot state at all — both froze a walking robot on
      // official gains (kp-2000 catapult, 4/4 repro on 0711 across two models). Reuse the
      // engage settle estimator + knobs (--ready-speed-max / --ready-dwell). A fresh,
      // localization-backed speed estimate is mandatory before a NEW STATIC handoff in
      // external-base/oracle modes: treating a one-frame localization gap as "settled"
      // can freeze a moving robot. Modes without a localization feed keep legacy behavior.
      const bool static_needs_localization =
          cfg_.loc_mode == LocMode::kExternalBase || cfg_.loc_mode == LocMode::kOracle;
      const bool static_localization_fresh =
          (cfg_.loc_mode == LocMode::kExternalBase && base_fresh_) ||
          (cfg_.loc_mode == LocMode::kOracle && oracle_fresh_);
      const bool base_lin_quiet = static_needs_localization
          ? (static_localization_fresh && base_speed_xy_valid_ &&
             base_speed_xy_est_ <= cfg_.station_ready_speed_max)
          : (!base_speed_xy_valid_ ||
             base_speed_xy_est_ <= cfg_.station_ready_speed_max);
      static_settle_ticks_ = base_lin_quiet ? static_settle_ticks_ + 1 : 0;
      const bool base_settled = static_needs_localization
          ? (static_localization_fresh && base_speed_xy_valid_ &&
             static_settle_ticks_ * cfg_.dt >= cfg_.station_ready_hold_s)
          : (!base_speed_xy_valid_ ||
             static_settle_ticks_ * cfg_.dt >= cfg_.station_ready_hold_s);
      // NEAR-STATION guard (2026-07-08, from the rally-gate fall): never hand the stiff
      // STATIC stand a robot that is parked far off its hold station — the walked stance
      // is staggered/leaning and the official gains freeze it there (measured: forced
      // switch at 0.83 m off-station -> tip). Off-station, the POLICY hold keeps actively
      // walking home; the switch waits until it arrives. No-op when no anchor is set
      // (cold boot before an anchor exists: near_station true -> legacy behavior).
      const bool near_station = !hold_station_set_ ||
          (Vec2(st.base_pos_w[0], st.base_pos_w[1]) - hold_station_w_).norm() < 0.3;
      // ...and NEAR-HEADING (2026-07-08 rally run 4): the backhand follow-through can leave
      // the robot yawed 35-55° (execution over-rotation; the reference ends at ~0°). The
      // static stand FROZE that yawed/staggered stance and it tipped seconds later, while
      // the POLICY hold both balances actively and — being the trained pre-strike state
      // (which always faces +x in training) — is the only thing in the chain with a
      // heading-restoring feedback loop. Off-heading: stay on the policy hold (g25-proven
      // to 20 s); the engage heading gate keeps swings blocked until square.
      const double hold_yaw = std::atan2(
          2.0 * (st.base_quat_w[0] * st.base_quat_w[3] +
                 st.base_quat_w[1] * st.base_quat_w[2]),
          1.0 - 2.0 * (st.base_quat_w[2] * st.base_quat_w[2] +
                       st.base_quat_w[3] * st.base_quat_w[3]));
      const bool near_heading =
          std::fabs(hold_yaw) < cfg_.static_handoff_yaw_max_deg * M_PI / 180.0;
      const bool post_recovery = planner_have_hold_ && near_station && near_heading &&
          base_settled &&
          ((t_since > cfg_.hold_recover_s && upright_still) ||
           t_since > cfg_.hold_recover_s + 3.0);
      // PRE-FIRST-ENGAGE quiescence gate (2026-07-12 run2 fall): the old bare
      // !planner_have_hold_ disjunct entered STATIC with NO state checks. Before the
      // first completed swing, an explicitly expired/cancelled station walk may drop the
      // pending shield. Transient planner/localization gaps now retain the pending target
      // while blocking release; otherwise the next tick can hand a robot still carrying
      // walk momentum to the stiff official
      // stand (the kp-2000 catapult class the 0711 guards were added for; run2 fell
      // exactly here when the planner stream went stale mid-walk). Mirror the
      // post_recovery stack: hard floor = base_settled && near_station (never freeze a
      // translating base), upright_still for the fast path, and the t_since>3 s arm
      // bounds the stay on the fragile policy hold (ClearPendingStation_ restarts the
      // clock on aborted-walk edges). A true cold boot is different: before ACTIVE station
      // tracking has ever been released there is no walk momentum to prove with mocap, so an
      // upright/quiet encoder+IMU stand may enter the official static hold immediately. Once any
      // pending station has released the policy, this exception is permanently closed for the
      // MOTION session and a fresh localization-backed base settle is mandatory after an abort.
      // near_heading is deliberately NOT required here — pre-first-swing a drifted IMU
      // yaw must not deadlock the handoff (static stand is yaw-agnostic; the engage
      // heading gate + yawed-deadlock release own heading).
      const bool prefirst_static = prefirst_static_allowed(
          planner_have_hold_, near_station, prefirst_active_station_tracking_started_,
          base_settled, upright_still, t_since > 3.0);
      if (cfg_.planner_mode && !planner_engaged_ && !planner_pending_station_active_ &&
          level_.load() == 0 && !planner_static_active_ && !planner_have_hold_ &&
          !prefirst_static && prefirst_active_station_tracking_started_ &&
          (prefirst_warn_tick_++ % 100) == 0)
        std::fprintf(stderr,
            "[pp] pre-first-engage STATIC stand DEFERRED (base unsettled after an aborted "
            "station walk) -> staying on the ACTIVE policy hold until quiet\n");
      // STICKY: once static engages it stays until the next swing (level 1). A quiescence
      // condition that flaps re-enters the branch every few ticks — policy/static command
      // CHATTER with a restarted blend each time (observed: 9 re-entries then a fall).
      if (cfg_.planner_mode && !planner_engaged_ && !planner_pending_station_active_ &&
          level_.load() == 0 &&
          (serve_static_handoff_pending_ || prefirst_static || post_recovery ||
           planner_static_active_)) {
        if (!planner_static_active_) {
          planner_static_active_ = true;
          planner_static_start_tick_ = tick_idx;
          // Generic entries blend from measured q.  Serve handoff is already
          // commanding the exact default and must not introduce a new
          // default->measured command step at the controller boundary.
          planner_static_q0_ = serve_static_handoff_pending_
              ? nominal_q_sdk_
              : (state.q.size() == kNumJoints ? state.q : nominal_q_sdk_);
          const bool from_serve = serve_static_handoff_pending_;
          serve_static_handoff_pending_ = false;
          // The policy is out of control from here until the next engage: arm the
          // measured-posture feedback seed so the engage's first policy tick reads the
          // training-style post-reset obs (MEASURED posture in executed-q_des feedback
          // coordinates, taken fresh on that tick) instead of a seconds-stale action.
          const RuntimeHandoffReset handoff =
              runtime_handoff_reset(onnx_.has_bounded_qdes_contract());
          last_action_.setZero();
          last_action_seed_pending_ = handoff.seed_measured_qdes_feedback;
          if (handoff.seed_measured_qdes_feedback)
            qdes_projector_initialized_ = false;
          if (from_serve)
            std::fprintf(stderr,
                "[pp] SERVE -> V17 exact-default STATIC handoff; "
                "action history reset and learned action blocked until engage\n");
          if (planner_have_hold_)
            std::fprintf(stderr,
                "[pp] post-swing recovery done -> STATIC official stand until next engage\n");
        }
        const double a = std::min(1.0,
            (tick_idx - planner_static_start_tick_) * cfg_.dt /
                std::max(cfg_.hold_blend_s, 1e-3));
        cmd.q_des = (1.0 - a) * planner_static_q0_ + a * nominal_q_sdk_;
        cmd.dq_des = Eigen::VectorXd::Zero(kNumJoints);
        cmd.tau_ff = Eigen::VectorXd::Zero(kNumJoints);
        cmd.kp = official_kp_sdk_;
        cmd.kd = official_kd_sdk_;
        last_q_des_ = cmd.q_des;  // seed a continuous static->policy takeover blend
        return true;
      }
      // Reset the sticky latch only when a swing actually runs (engage exited the branch);
      // never mid-hold, or quiescence flapping chatters between policy and static commands.
      if (level_.load() == 1) planner_static_active_ = false;
    }

    // LIVE PLANNER target override (Path B): the swing clock already set tg.time_to_strike,
    // swing_sign, clip_id and time_step above; here we only swap the REACH POINT. During a
    // swing use the FROZEN world target; while holding, use a base-anchored ready target at
    // racket-reach x (so the footwork policy is not commanded to walk to a fixed world point
    // during the hold — the wbc_runner rest-hold semantics). Untouched when not planner_mode.
    if (cfg_.planner_mode) {
      if (planner_engaged_) {   // active swing -> frozen world target
        tg.pos_w = planner_frozen_pos_w_;
        tg.vel_w = planner_frozen_vel_w_;
      } else if (onnx_.is_hitter_pure_obs()) {
        // 110 hitter_pure idle (2026-07-08 fix, from the first rally-gate fall): the hold
        // target must be WORLD-FIXED at the hold-station anchor — the same obs family as
        // the Gate-2.5-proven scripted hold (world-fixed box-center target + box-center
        // vel; P2 held 20 s on it). The first design anchored it to the LIVE base (0.70
        // ahead of wherever the robot is, re-anchored per tick): a moving carrot with NO
        // positional feedback — racket_target_rel_base never closes however far the robot
        // walks, and the rally gate measured the policy hold charging +0.83 m off-station
        // in ~1 s on it (then the forced static handoff tipped from the walked stance).
        // Anchoring at the station makes a forward drift SHORTEN the observed reach, so
        // the trained pre-strike response pulls the robot back. Geometry is PER-SIDE
        // (last swing side = the side the hold tts clamp already assumes): plane_x +
        // y-band center at the anchor, z-band mid height, trained box-center velocity
        // (the frozen streamed vel could be out-of-band — it is what the LAST swing flew).
        // Offset in WORLD axes, deliberately NOT rotated by the base yaw (2026-07-08 review):
        // training's hold target is world-fixed (station + the +x plane offset in WORLD),
        // and the engage-side station derivation already uses world-frame reach offsets.
        // Rotating by the live yaw would make the target ORBIT the station while a yawed
        // robot re-squares — an obs the (rally2-)trained recovery never sees.
        const int hc = clip_id_from_swing_sign(swing_dir_.load() >= 0 ? 1.0 : -1.0);
        const Vec2 anchor_xy = hold_station_set_
            ? hold_station_w_
            : Vec2(st.base_pos_w[0], st.base_pos_w[1]);  // pre-anchor tick / dropout
        tg.pos_w = Vec3(anchor_xy[0] + reach_offset_clip_[hc][0],
                        anchor_xy[1] + reach_offset_clip_[hc][1],
                        planner_pending_station_active_
                            ? planner_hold_z_w_
                            : 0.5 * (hp_z_band_[hc][0] + hp_z_band_[hc][1]));
        tg.vel_w = planner_pending_station_active_
            ? planner_pending_vel_w_ : cfg_.racket_vel_w_clip[hc];
      } else {                  // idle/rest (incl. before the first engage) -> base-anchored hold
        const Vec4 base_yaw = yaw_quat(st.base_quat_w);
        Vec3 hb(cfg_.hold_anchor_x_b, planner_hold_pos_b_engage_[1], 0.0);
        tg.pos_w = st.base_pos_w + quat_rotate(base_yaw, hb);
        tg.pos_w[2] = planner_hold_z_w_;
        tg.vel_w = planner_frozen_vel_w_;
      }
    }

    last_proj_grav_ = projected_gravity_body(st.base_quat_w);

    // 177-D hitter_footwork base-station channel (base_target_pos_b = yaw-frame Δxy from the
    // current base to the commanded station). During a swing the station rides the SAME reach
    // point the swing uses (scripted box center or frozen planner target) minus the per-clip
    // reference reach — training's base_couple_mode=reference_reach coupling. During level-0
    // holds the station is a FIXED WORLD ANCHOR (captured at hold entry / carried over from
    // the completed swing): the live Δ to that anchor is the policy's balance signal —
    // training pays pbase through every hold, so the policy leans on this channel to stay
    // put. Feeding Δ=0 through holds was the first design and is WRONG as the nominal path:
    // it removes the only anchor and the policy free-wanders meters during holds, then falls
    // off-station (2026-07-06 MuJoCo deploy-faithful CSV phase analysis: falls at |torso|
    // 1-2 m with ±0.1 m stations; live-station holds: model_17400 0 falls x 3 seeds).
    // Δ=0 remains ONLY the localization-dropout fallback (perfect_tracking / fabricated /
    // stale mocap/oracle), where any nonzero Δ would be fictional and chased open-loop.
    // 2026-07-07: the fixed-world anchor now applies to 110-D hitter_pure TOO (it was Δ=0
    // at idle — see the idle_station_dzero_110 branch below for the refuting evidence).
    if (onnx_.obs_dim() == kObsDim177 || onnx_.is_hitter_pure_obs()) {
      const bool base_real =
          (cfg_.loc_mode == LocMode::kOracle && oracle_fresh_) ||
          (cfg_.loc_mode == LocMode::kExternalBase && base_fresh_);
      if (!base_real) {
        tg.base_target_xy = Vec2(st.base_pos_w[0], st.base_pos_w[1]);  // dropout: Δ=0
        // Preserve the last verified world anchor through a transient/stale sample.  The
        // current tick still receives the safe fictional-motion fallback (Δ=0), while
        // recovery resumes toward the SAME station once localization becomes fresh again.
        // Session/runtime resets remain the only paths that intentionally discard the anchor.
      } else if (level_.load() == 1) {
        const int c = clip_id_from_swing_sign(tg.swing_sign);
        // stay-if-reachable (rally_v8): the in-swing station channel must agree with the
        // engage-derived station — an in-band target keeps the held station instead of
        // re-centering, so the policy sees the SAME "stay" command it trained on.
        tg.base_target_xy = station_from_target_(Vec2(tg.pos_w[0], tg.pos_w[1]), c);
        hold_station_w_ = tg.base_target_xy;  // post-swing hold recovers AT the strike station
        hold_station_set_ = true;
      } else if (onnx_.obs_dim() == kObsDim110 && cfg_.idle_station_dzero_110) {
        // LEGACY 110 idle: Δ=0 (station := current base). First design, justified as
        // "hitter_pure trains NO hold so idle never demands station-keeping" — REFUTED by the
        // 2026-07-07 Gate-2.5 evidence: with Δ=0 there is NO pull-back between swings, so the
        // follow-through displacement ACCUMULATES across cycles against the world-fixed
        // scripted target (the model_12200 P7 fall), and a hold-TRAINED rally model
        // (model_18000) outright DIVERGES in it — walked +0.94 m THROUGH the target
        // (racket_rel_base x +0.69 -> -0.27 while base_target_dxy pinned 0), yawed ~70°, obs
        // blow-up, fell 12 s into the P2 hold. Training-side truth: hitter_pure stations are
        // x ±0.10 / y ±0.40 with drift 0.01-0.02 m/swing — the policy NEVER trains forward
        // locomotion; an unanchored idle walks it straight out of distribution (the observed
        // pigeon-toed creep). Kept ONLY as a compile-time A/B fallback; the nominal 110 path
        // is the 177-style fixed-world anchor below (idle at an anchor == "pre-strike at the
        // station", which hitter_pure trains every swing).
        tg.base_target_xy = Vec2(st.base_pos_w[0], st.base_pos_w[1]);
      } else {
        if (!hold_station_set_) {  // fresh hold (cold boot / explicit runtime reset)
          hold_station_w_ = Vec2(st.base_pos_w[0], st.base_pos_w[1]);
          hold_station_set_ = true;
        }
        tg.base_target_xy = hold_station_w_;
      }
    }

    // The fixed-y homing A/B keeps the true station at the immutable MOTION
    // origin. The actor-facing base and ready-racket y channels are shifted
    // together while post-strike drift is outside the hysteresis band; x
    // remains exactly policy-native.
    ApplyFixedYHomingObservation_(
        tg, Vec2(st.base_pos_w[0], st.base_pos_w[1]),
        level_.load() == 0, tick_idx);

    // Reproduce the V15 training command after the final station/hold state is known.
    // level 0 is the pre/post-swing hold; level 1 deliberately publishes mode=-1 and pauses
    // the finite gait clock.  The training-only intervention indicator remains exactly zero.
    UpdateFiniteLateralGaitObservation_(tg, level_.load() == 0);

    // Measured racket FK for field diagnostics. This uses the same measured
    // Isaac-order q and localized base pose as the observation builder, but it
    // is not fed back into the actor. Local +Y is the A3 racket face normal.
    {
      const RacketPosePelvis racket_b = racket_pose_pelvis(st.q);
      const Mat3 base_rotation = mat_from_quat(st.base_quat_w);
      const Vec3 racket_pos_w = st.base_pos_w + base_rotation * racket_b.position;
      const Vec3 racket_normal_w = (base_rotation * racket_b.rotation.col(1)).normalized();
      if (last_racket_fk_valid_ && tick_idx > last_racket_fk_tick_) {
        const double elapsed =
            (tick_idx - last_racket_fk_tick_) * std::max(cfg_.dt, 1.0e-6);
        last_racket_vel_w_ = (racket_pos_w - last_racket_pos_w_) / elapsed;
      } else {
        last_racket_vel_w_.setZero();
      }
      last_racket_pos_w_ = racket_pos_w;
      last_racket_normal_w_ = racket_normal_w;
      last_racket_fk_tick_ = tick_idx;
      last_racket_fk_valid_ = true;
      last_target_pos_w_ = tg.pos_w;
      last_target_vel_w_ = tg.vel_w;
    }

    // 175-D deploy_parity vs 177-D hitter_footwork vs 180-D full (model_15200) vs 110-D
    // hitter_pure. Auto-selected from the loaded ONNX input dim. build_obs_175 drops
    // motion_anchor_pos_b + base_target_pos_b and reframes the racket target relative to the
    // CURRENT racket FK (pp_racket_fk.hpp) — no world base pos. build_obs_177 = the 175 layout
    // + base_target_pos_b(2) re-inserted (above). build_obs_110 = HITTER Table-I exact: NO
    // reference stream/swing_type, WORLD-frame deltas + e_base,x (refs never enter the obs —
    // the clip clock above only schedules tts and the graph's time_step input).
    // ENGAGE FEEDBACK SEED (2026-07-23): the first policy tick after a mode rearm or the
    // static-stand handoff mirrors training's post-reset obs — before the projector's first
    // action the executed-q_des feedback slot reports the MEASURED posture
    // ((clamp(q, safe_lo, safe_hi) - default_q) / feedback_scale, clamped to [-1, 1]),
    // not zeros. Seeded HERE (not at the arming site) so the value is the posture of the
    // actual first policy tick, not a stale snapshot from before a static hold.
    if (last_action_seed_pending_ && state.q.size() == kNumJoints) {
      if (onnx_.has_bounded_qdes_contract()) {
        last_action_ = onnx_.measured_qdes_feedback(st.q);
        if (onnx_.last_action_head_is_zero()) {
          for (int i = 0; i < kNumJoints; ++i)
            if (isaac_to_sdk_[i] == kHeadSlot0 || isaac_to_sdk_[i] == kHeadSlot1)
              last_action_[i] = 0.0;
        }
      }
      last_action_seed_pending_ = false;
    }
    const Eigen::VectorXd obs = (onnx_.obs_dim() == kObsDim118)
        ? build_obs_118(st, tg, last_action_, onnx_.default_q())
        : (onnx_.obs_dim() == kObsDim113)
        ? build_obs_113(st, tg, last_action_, onnx_.default_q())
        : (onnx_.obs_dim() == kObsDim110)
        ? build_obs_110(st, tg, last_action_, onnx_.default_q())
        : (onnx_.obs_dim() == kObsDim175)
        ? build_obs_175(refs, st, tg, last_action_, onnx_.default_q(), cfg_.use_imu_yaw_for_targets)
        : (onnx_.obs_dim() == kObsDim177)
        ? build_obs_177(refs, st, tg, last_action_, onnx_.default_q(), cfg_.use_imu_yaw_for_targets)
        : build_obs_180(refs, st, tg, last_action_, onnx_.default_q(), cfg_.use_imu_yaw_for_targets);
    if (!PpAllFinite(obs)) {
      throw std::runtime_error("ping-pong observation contains NaN/Inf");
    }
    { std::lock_guard<std::mutex> lk(obs_mu_); last_obs_ = obs; }  // for obs-debug
    const Eigen::VectorXd action = onnx_.mean_action(obs, time_step);
    if (action.size() != kNumJoints || !PpAllFinite(action)) {
      throw std::runtime_error("ping-pong ONNX action has wrong size or contains NaN/Inf");
    }
    const Eigen::VectorXd tq_nominal_isaac = onnx_.target_q(action);
    if (tq_nominal_isaac.size() != kNumJoints || !PpAllFinite(tq_nominal_isaac)) {
      throw std::runtime_error("ping-pong decoded target_q has wrong size or contains NaN/Inf");
    }
    const Eigen::VectorXd tq_isaac =
        ProjectBoundedQdes_(tq_nominal_isaac, st.q, st.qd);
    if (!PpAllFinite(tq_isaac))
      throw std::runtime_error("ping-pong projected target_q contains NaN/Inf");

    Eigen::VectorXd q_sdk = to_sdk_order(tq_isaac, isaac_to_sdk_);
    Eigen::VectorXd kp_sdk = to_sdk_order(onnx_.kp(), isaac_to_sdk_);
    Eigen::VectorXd kd_sdk = to_sdk_order(onnx_.kd(), isaac_to_sdk_);

    // NECK PASSIVE: ignore model neck outputs; hold head at nominal w/ fixed PD.
    for (int s : {kHeadSlot0, kHeadSlot1}) {
      // Bounded models already replace each head target with
      // clamp(default_q, selected_interval) in ProjectBoundedQdes_. Preserve
      // that exact target so transient constraints and Python parity survive.
      if (!onnx_.has_bounded_qdes_contract()) q_sdk[s] = kHeadPosRad;
      kp_sdk[s] = kHeadKp;
      kd_sdk[s] = kHeadKd;
    }

    // LEGS PASSIVE (hoisted demo): hold legs at nominal stand with the trained
    // leg PD. Removes leg twitch caused by balance corrections against the
    // nominal-base-position obs (no localisation). Arm + waist still swing.
    if (legs_passive_.load()) {
      // Hold legs at nominal with the TRAINED leg PD (ran clean on the hoist).
      // The stiff official ground-stand gains buzz/swing a hoisted robot, so
      // they are NOT used here; they live behind --official-stand for Step 2.
      // With --auto-leg-hold this flips true at level 0 / false at level 1.
      for (int s = kLegSlotStart; s < kLegSlotStart + kLegSlotCount; ++s)
        q_sdk[s] = nominal_q_sdk_[s];
    }

    // WAIST PASSIVE: hold the waist (slots 0..2) at nominal so the torso stays
    // upright. The policy drives waist_pitch to its forward limit which (with the
    // forehand arms forward) shifts the CoM past the feet — a static leg hold can't
    // rebalance that. Freezing the waist makes the swing ARMS-ONLY but keeps the
    // CoM over the base of support. Gains: official ground-stand kp/kd applied in
    // a3_pingpong_main when --official-stand is set (else the trained waist PD).
    if (waist_passive_.load()) {
      for (int s = kWaistSlotStart; s < kWaistSlotStart + kWaistSlotCount; ++s)
        q_sdk[s] = nominal_q_sdk_[s];
    }

    // ARM HOLD (stage cosmetics, --arm-hold-nominal): PRE-SWING level-0 holds ONLY
    // (MOTION entered, no swing yet since the last mode entry): after a sustained-
    // quiet floor, ramp the arm q_des (slots 5..18) to nominal and hold. The
    // POST-SWING recovery hold is NEVER touched — the policy balances out of the
    // follow-through WITH its arms, and every override variant tested topples it
    // (2026-07-06, g25 oracle P3b: instant-quiet 1 s ramp z=0.26/0.23; sustained-
    // quiet 1 s floor + 2.5 s ramp + instant-release z=0.38/0.35, 1-of-3 pass —
    // CLOSED as unsafe, do not retry runner-side). Post-swing arm cleanup instead
    // comes from the STATIC official stand: planner mode auto-hands-off after
    // hold_recover_s (now CLI-tunable, --hold-recover); scripted mode = operator 's'.
    // Fixes the pre-swing hold arm-twist without retraining; the policy keeps seeing
    // the measured (held) arm q in obs — the legs_passive-proven mismatch class.
    // Runs BEFORE the joint-limit clamp.
    if (cfg_.arm_hold_nominal) {
      if (level_.load() == 1) arm_hold_armed_ = false;  // swung: arms stay policy-owned
      const bool quiet =
          level_.load() == 0 && arm_hold_armed_ &&
          projected_gravity_body(st.base_quat_w)[2] < -0.92 &&
          st.base_ang_vel_b.norm() < 0.5 &&
          (st.qd.size() == 0 || st.qd.cwiseAbs().maxCoeff() < 1.5);
      arm_quiet_ticks_ = quiet ? arm_quiet_ticks_ + 1 : 0;
      const double t_quiet = arm_quiet_ticks_ * cfg_.dt;
      if (t_quiet > cfg_.arm_hold_min_quiet_s) {
        const double a = std::min(1.0, (t_quiet - cfg_.arm_hold_min_quiet_s) /
                                           std::max(cfg_.arm_hold_blend_s, 1e-3));
        for (int s = kArmSlotStart; s < kArmSlotStart + kArmSlotCount; ++s)
          q_sdk[s] = (1.0 - a) * q_sdk[s] + a * nominal_q_sdk_[s];
      }
    }

    // LEG q_des CLAMP (released full-body swing only): the trained swing commands a
    // deep crouch-and-lean (hip_pitch -0.6..-0.77, knee +0.6, ankle_pitch -0.7..-0.9
    // rad) that assumes planted-foot contact dynamics. On the real robot that posture
    // is not a stable static stand -> tracking it sinks the knees / pitches forward.
    // Clamp each policy-driven leg slot to nominal ± leg_clamp_rad_ to keep the legs
    // near the proven upright stand while leaving room for small balance moves. The
    // policy still sees the true measured q in obs, so its feedback loop is intact.
    // No-op when legs are HELD (already nominal) or when the band is 0 (off).
    if (leg_clamp_rad_ > 0.0 && !legs_passive_.load()) {
      for (int s = kLegSlotStart; s < kLegSlotStart + kLegSlotCount; ++s) {
        const double lo = nominal_q_sdk_[s] - leg_clamp_rad_;
        const double hi = nominal_q_sdk_[s] + leg_clamp_rad_;
        q_sdk[s] = std::min(std::max(q_sdk[s], lo), hi);
      }
    }

    // LEG q_des LOW-PASS (released swing only): EMA-smooth the leg q_des so stiff
    // weight-bearing gains (--leg-stand-gains, kp~2000) track a SMOOTH reference
    // instead of the policy's tick-to-tick jitter (which they amplify into a TWITCH).
    // Runs AFTER the clamp, so the EMA of in-band values stays in band. Seeded from
    // nominal; while legs are HELD it tracks nominal so the next release starts smooth.
    if (leg_smooth_alpha_ < 1.0 && leg_qdes_smooth_.size() == kNumJoints) {
      const double a = leg_smooth_alpha_;
      const bool released = !legs_passive_.load();
      for (int s = kLegSlotStart; s < kLegSlotStart + kLegSlotCount; ++s) {
        leg_qdes_smooth_[s] = released ? (a * q_sdk[s] + (1.0 - a) * leg_qdes_smooth_[s])
                                       : q_sdk[s];  // held: track nominal, re-seed for release
        q_sdk[s] = leg_qdes_smooth_[s];
      }
    }

    // Final pending-station takeover: the static official stand cannot locomote, but handing
    // its stiff nominal command directly to the level-0 policy creates a one-tick pose/gain snap.
    // Blend both desired pose and gains into policy control before readiness dwell may begin.
    if (planner_policy_takeover_active_) {
      const double a = std::min(1.0,
          (tick_idx - planner_policy_takeover_start_tick_) * cfg_.dt /
              std::max(cfg_.station_takeover_blend_s, 1e-3));
      if (planner_policy_takeover_q0_.size() == kNumJoints)
        q_sdk = (1.0 - a) * planner_policy_takeover_q0_ + a * q_sdk;
      kp_sdk = (1.0 - a) * official_kp_sdk_ + a * kp_sdk;
      kd_sdk = (1.0 - a) * official_kd_sdk_ + a * kd_sdk;
      if (a >= 1.0 - 1e-9) planner_policy_takeover_active_ = false;
    }

    // The full plant target includes passive-head/limb overrides and takeover blending, not just
    // the policy projector.  Fail closed against the ONNX safe interval before the legacy hard
    // clamp; silently trimming here would hide a Python/C++ contract break.
    last_safe_interval_violation_count_ = 0;
    if (onnx_.has_safe_qdes_interval_contract()) {
      ++safe_interval_audit_ticks_;
      for (int i = 0; i < kNumJoints; ++i) {
        const int sdk = isaac_to_sdk_[i];
        if (!std::isfinite(q_sdk[sdk])) {
          throw std::runtime_error(
              "ping-pong QDES NUMERIC FAULT: final plant q_des is non-finite for "
              "joint '" + onnx_.joint_names()[i] + "'");
        }
        const double excess = std::max(
            onnx_.qdes_safe_lo()[i] - q_sdk[sdk],
            q_sdk[sdk] - onnx_.qdes_safe_hi()[i]);
        if (excess > 0.0) {
          ++last_safe_interval_violation_count_;
          ++safe_interval_count_[sdk];
          safe_interval_max_excess_[sdk] =
              std::max(safe_interval_max_excess_[sdk], excess);
          if (!cfg_.gate3_qdes_audit_only) {
            throw std::runtime_error(
                "ping-pong QDES SAFETY FAULT: final plant q_des escapes ONNX safe "
                "interval for joint '" + onnx_.joint_names()[i] + "'");
          }
        }
      }
      if (last_safe_interval_violation_count_ > 0 &&
          cfg_.gate3_qdes_audit_only && !safe_interval_warned_) {
        safe_interval_warned_ = true;
        std::fprintf(
            stderr,
            "[pp qdes-audit] final q_des escaped the ONNX safe interval on %d "
            "joint(s); Gate3 continues with the unmodified command\n",
            last_safe_interval_violation_count_);
      }
    }

    // SAFETY: clamp q_des to the MJCF/URDF joint position limits before publish.
    // In-range commands are untouched (no-op for in-distribution actions); a
    // nonzero count means the policy commanded out-of-limit -> a red flag we warn
    // about once (check gains / targets / loc mode before continuing on hardware).
    const Eigen::VectorXd q_preclamp = q_sdk;  // raw finite command, for the per-joint audit
    Eigen::VectorXd q_hard_limited = q_sdk;
    last_clamp_count_ = clamp_q_to_limits(q_hard_limited);
    last_clamp_viol_ = q_preclamp - q_hard_limited;
    ++clamp_ticks_;
    for (int i = 0; i < kNumJoints; ++i) {  // per-backend-slot clamp stats (waist_roll audit)
      const double viol = std::abs(q_preclamp[i] - q_sdk[i]);
      if (viol > 1e-9) {
        ++clamp_count_[i];
        if (viol > clamp_max_viol_[i]) clamp_max_viol_[i] = viol;
      }
    }
    if (last_clamp_count_ > 0 && !clamp_warned_) {
      clamp_warned_ = true;
      if (cfg_.gate3_qdes_audit_only) {
        std::fprintf(stderr,
            "[pp qdes-audit] q_des would hit backend hard limits on %d joint(s); "
            "Gate3 continues with the unmodified command\n",
            last_clamp_count_);
      } else {
        std::fprintf(stderr,
            "[pp WARN] q_des clamped to joint limits on %d joint(s) (policy commanded "
            "out-of-range; check gains/targets/loc-mode)\n",
            last_clamp_count_);
      }
    }
    if (!cfg_.gate3_qdes_audit_only) q_sdk = std::move(q_hard_limited);

    // One-shot comprehensive FIRST-TICK debug dump (joint pos/vel, IMU/gravity,
    // full per-block obs stats, raw ONNX action stats, decoded q_des/kp/kd) for
    // bring-up + AGI staff review. Fires on the first policy tick only.
    if (!dbg_done_) {
      dbg_done_ = true;
      LogFirstTick(obs, action, q_sdk, kp_sdk, kd_sdk, st, state, time_step);
    }

    cmd.q_des = q_sdk;
    cmd.kp = kp_sdk;
    cmd.kd = kd_sdk;
    cmd.dq_des = Eigen::VectorXd::Zero(kNumJoints);
    cmd.tau_ff = Eigen::VectorXd::Zero(kNumJoints);

    // --- diagnostics: snapshot + rolling per-joint ranges (SDK order) ---
    if (state.q.size() == kNumJoints && state.dq.size() == kNumJoints) {
      const Eigen::VectorXd err = (q_sdk - state.q).cwiseAbs();
      const Eigen::VectorXd qda = state.dq.cwiseAbs();
      std::lock_guard<std::mutex> lk(diag_mu_);
      last_q_des_ = q_sdk;
      last_q_meas_ = state.q;
      last_qd_meas_ = state.dq;
      if (!ranges_init_) {
        des_lo_ = des_hi_ = q_sdk;
        meas_lo_ = meas_hi_ = state.q;
        err_peak_ = err;
        qd_peak_ = qda;
        ranges_init_ = true;
      } else {
        des_lo_ = des_lo_.cwiseMin(q_sdk);     des_hi_ = des_hi_.cwiseMax(q_sdk);
        meas_lo_ = meas_lo_.cwiseMin(state.q); meas_hi_ = meas_hi_.cwiseMax(state.q);
        err_peak_ = err_peak_.cwiseMax(err);
        qd_peak_ = qd_peak_.cwiseMax(qda);
      }
    }

    if (onnx_.has_bounded_qdes_contract()) {
      // Feed back the command that actually survived every runner override and the final hard
      // limit clamp.  The same final command seeds next tick's q_des-rate interval.
      qdes_projected_isaac_ = from_sdk_order(q_sdk, isaac_to_sdk_);
      qdes_projector_initialized_ = true;
      last_action_ = onnx_.qdes_feedback(qdes_projected_isaac_);
    } else {
      last_action_ = action;
    }
    // NECK PASSIVE, part 2: the head q_des is replaced by the bounded
    // default-in-interval target (or legacy zero), so feed back the action
    // that was actually applied. Models whose ONNX metadata declares zero head feedback
    // trained those slots as exact zero; retaining even a clipped raw neck output here creates
    // an unobserved recurrent integrator. Resolve by the ONNX-name -> SDK mapping rather than
    // assuming Isaac action columns. The metadata is operational; recipe names do not decide it.
    if (onnx_.last_action_head_is_zero()) {
      for (int i = 0; i < static_cast<int>(last_action_.size()); ++i)
        if (isaac_to_sdk_[i] == kHeadSlot0 || isaac_to_sdk_[i] == kHeadSlot1)
          last_action_[i] = 0.0;
    }
    last_time_step_ = time_step;
    return true;
  }

  // Bind to a std::function with the a3_deploy::CommandFn signature.
  std::function<bool(std::uint64_t, const robot_io::RobotState&, robot_io::RobotCommand&)>
  AsCommandFn() {
    return [this](std::uint64_t tick, const robot_io::RobotState& s,
                  robot_io::RobotCommand& c) { return ComputeCommand(tick, s, c); };
  }

  int last_time_step() const { return last_time_step_; }
  Vec3 last_proj_grav() const { return last_proj_grav_; }
  bool authoritative_mocap_required() const {
    return onnx_.uses_authoritative_mocap_pose();
  }
  bool authoritative_mocap_fresh() const {
    return !authoritative_mocap_required() || base_fresh_;
  }

  // Refresh the safety/diagnostic projected gravity independently of policy execution.
  // A full-pose-mocap ONNX contract uses authoritative mocap attitude; all
  // other modes use the IMU.
  // ComputeCommand only runs in
  // SHADOW/MOTION, so without this the status/trace gravity FREEZES at the [0,0,-1]
  // default in PASSIVE/PD_STAND -- which hides whether the robot is actually upright
  // on the ground (the whole point of the PD_STAND ground check). Call every tick in
  // every mode. Diagnostic-only: does NOT touch the published command.
  void observe_imu(const robot_io::RobotState& state) {
    if (onnx_.uses_authoritative_mocap_pose() &&
        cfg_.loc_mode == LocMode::kExternalBase) {
      PpBaseSample sample;
      base_fresh_ = base_in_ &&
          base_in_->Latest(sample, cfg_.external_base_max_age_s, true);
      if (base_fresh_) {
        last_proj_grav_ = projected_gravity_body(sample.quat);
      } else if (state.imu_quat_wxyz.size() == 4 &&
                 state.imu_quat_wxyz.norm() > 0.5) {
        // The actor is about to be removed from MOTION, so this is not an observation fallback.
        // Keep the always-on fall guard live during the existing PD-STAND exit path instead of
        // freezing it on the last mocap attitude.
        last_proj_grav_ = projected_gravity_body(state.imu_quat_wxyz);
      }
      return;
    }
    if (state.imu_quat_wxyz.size() == 4 && state.imu_quat_wxyz.norm() > 0.5)
      last_proj_grav_ = projected_gravity_body(state.imu_quat_wxyz);
  }

  // --- clamp audit (per backend slot; for the waist_roll mismatch investigation) ---
  int last_clamp_count() const { return last_clamp_count_; }       // # joints clamped last tick
  int last_safe_interval_violation_count() const {
    return last_safe_interval_violation_count_;
  }
  std::uint64_t safe_interval_audit_ticks() const {
    return safe_interval_audit_ticks_;
  }
  std::uint64_t safe_interval_count_for(int slot) const {
    return (slot >= 0 && slot < kNumJoints) ? safe_interval_count_[slot] : 0;
  }
  double safe_interval_max_excess_for(int slot) const {
    return (slot >= 0 && slot < kNumJoints) ? safe_interval_max_excess_[slot] : 0.0;
  }
  bool gate3_qdes_audit_only() const { return cfg_.gate3_qdes_audit_only; }
  std::uint64_t clamp_ticks() const { return clamp_ticks_; }       // ticks the clamp has run
  std::uint64_t clamp_count_for(int slot) const {                  // times this slot was clamped
    return (slot >= 0 && slot < kNumJoints) ? clamp_count_[slot] : 0;
  }
  double clamp_max_viol_for(int slot) const {                      // worst out-of-range amount (rad)
    return (slot >= 0 && slot < kNumJoints) ? clamp_max_viol_[slot] : 0.0;
  }
  const Eigen::VectorXd& last_clamp_viol() const {                 // signed pre-clamp - published
    return last_clamp_viol_;
  }
  int worst_clamped_slot() const {                                 // most-clamped backend slot (-1 none)
    int w = -1; std::uint64_t best = 0;
    for (int i = 0; i < kNumJoints; ++i)
      if (clamp_count_[i] > best) { best = clamp_count_[i]; w = i; }
    return w;
  }
  bool bounded_qdes_active() const { return onnx_.has_bounded_qdes_contract(); }
  std::uint64_t qdes_projector_ticks() const { return qdes_projector_ticks_; }
  int qdes_projector_active_count() const { return last_qdes_projector_active_count_; }
  int qdes_projector_rate_count() const { return last_qdes_projector_rate_count_; }
  int qdes_projector_tracking_count() const {
    return last_qdes_projector_tracking_count_;
  }
  int qdes_projector_torque_count() const { return last_qdes_projector_torque_count_; }
  int qdes_projector_infeasible_count() const {
    return last_qdes_projector_infeasible_count_;
  }
  double qdes_projector_max_normalized_error() const {
    return last_qdes_projector_max_normalized_error_;
  }
  double qdes_feasible_action_utilization_max() const {
    return last_qdes_feasible_action_utilization_max_;
  }
  double qdes_feasible_interval_width_min() const {
    return last_qdes_feasible_interval_width_min_;
  }
  double qdes_feasible_rate_utilization_max() const {
    return last_qdes_feasible_rate_utilization_max_;
  }
  int qdes_feasible_rate_bound_count() const {
    return last_qdes_feasible_rate_bound_count_;
  }
  int qdes_feasible_tracking_bound_count() const {
    return last_qdes_feasible_tracking_bound_count_;
  }
  int qdes_feasible_torque_bound_count() const {
    return last_qdes_feasible_torque_bound_count_;
  }
  int worst_projected_isaac_joint() const {
    int worst = -1;
    std::uint64_t best = 0;
    for (int i = 0; i < kNumJoints; ++i)
      if (qdes_projector_joint_count_[i] > best) {
        best = qdes_projector_joint_count_[i];
        worst = i;
      }
    return worst;
  }
  std::uint64_t qdes_projector_joint_count(int isaac_joint) const {
    return isaac_joint >= 0 && isaac_joint < kNumJoints
        ? qdes_projector_joint_count_[isaac_joint] : 0;
  }
  Vec3 last_base_pos() const { return last_base_pos_; }
  const Eigen::VectorXd& last_action() const { return last_action_; }
  const std::array<int, 31>& isaac_to_sdk() const { return isaac_to_sdk_; }
  PpOnnxPolicy& onnx() { return onnx_; }

 private:
  void ApplyFixedYHomingObservation_(PpRacketTarget& target,
                                     const Vec2& measured_base_xy,
                                     bool in_hold,
                                     std::uint64_t tick_idx) {
    if (!cfg_.fixed_y_homing_replay || !station_session_origin_set_)
      return;
    const bool localization_fresh =
        (cfg_.loc_mode == LocMode::kExternalBase && base_fresh_) ||
        (cfg_.loc_mode == LocMode::kOracle && oracle_fresh_);
    if (!localization_fresh)
      return;

    const double origin_y = station_session_origin_w_[1];
    const double error_y = origin_y - measured_base_xy[1];
    const double abs_error_y = std::fabs(error_y);

    // The physical setpoint is always the same origin, including during a
    // swing. Gain shaping is a level-0 recovery aid only.
    target.base_target_xy[1] = origin_y;
    if (!in_hold || !planner_have_hold_)
      return;

    // HITTER training couples the ready-racket point to its commanded base
    // station. Shaping base_target alone creates a contradictory observation:
    // one channel asks the feet to move while racket_target_rel_base still
    // asks them to stay. Preserve the trained reach offset and move both
    // actor-facing hold channels together. This is only a virtual recovery
    // target; the external station and every admitted strike remain locked to
    // station_session_origin_w_.
    const double hold_reach_y = target.pos_w[1] - hold_station_w_[1];
    target.pos_w[1] = origin_y + hold_reach_y;

    if (fixed_y_homing_active_ &&
        abs_error_y <= cfg_.fixed_y_homing_exit_m) {
      fixed_y_homing_active_ = false;
      ++fixed_y_homing_completion_count_;
      const double elapsed_s =
          (tick_idx >= fixed_y_homing_start_tick_
               ? tick_idx - fixed_y_homing_start_tick_
               : 0) *
          std::max(cfg_.dt, 1.0e-6);
      std::fprintf(
          stderr,
          "[pp fixed-y-homing] COMPLETE origin_y=%+.3f base_y=%+.3f "
          "residual=%+.3f m peak=%.3f m elapsed=%.2f s count=%llu\n",
          origin_y, measured_base_xy[1], error_y,
          fixed_y_homing_peak_error_m_, elapsed_s,
          static_cast<unsigned long long>(
              fixed_y_homing_completion_count_));
    }
    if (!fixed_y_homing_active_ &&
        abs_error_y >= cfg_.fixed_y_homing_enter_m) {
      fixed_y_homing_active_ = true;
      fixed_y_homing_start_tick_ = tick_idx;
      fixed_y_homing_peak_error_m_ = abs_error_y;
      ++fixed_y_homing_activation_count_;
      std::fprintf(
          stderr,
          "[pp fixed-y-homing] ACTIVATE origin_y=%+.3f base_y=%+.3f "
          "error=%+.3f m gain=%.2f count=%llu\n",
          origin_y, measured_base_xy[1], error_y,
          cfg_.fixed_y_homing_gain,
          static_cast<unsigned long long>(
              fixed_y_homing_activation_count_));
    }
    if (!fixed_y_homing_active_)
      return;

    fixed_y_homing_peak_error_m_ =
        std::max(fixed_y_homing_peak_error_m_, abs_error_y);
    const double shaped_delta_y = std::clamp(
        cfg_.fixed_y_homing_gain * error_y,
        -cfg_.fixed_y_homing_max_delta_m,
        cfg_.fixed_y_homing_max_delta_m);
    const double actor_station_y = measured_base_xy[1] + shaped_delta_y;
    const double actor_shift_y = actor_station_y - origin_y;
    target.base_target_xy[1] = actor_station_y;
    target.pos_w[1] += actor_shift_y;
  }

  void ResetFiniteLateralGait_() {
    finite_gait_planned_cycles_ = 0;
    finite_gait_duration_steps_ = 0;
    finite_gait_elapsed_steps_ = 0;
    finite_gait_velocity_y_ = 0.0;
    finite_gait_initial_delta_y_ = 0.0;
    finite_gait_settle_ticks_ = 0;
  }

  // Convert a newly latched HITTER station into exactly one finite HUGWBC gait command.
  // explicit_station_transition is false for a new shot that deliberately keeps the previous
  // station: just like training's station_y_step_class==SAME, residual pose error must not re-arm
  // another corrective step.
  void PlanFiniteLateralGait_(const Vec2& measured_base_xy, const Vec2& station_xy,
                              bool explicit_station_transition) {
    if (!onnx_.has_finite_lateral_gait()) return;
    const double delta_y = station_xy[1] - measured_base_xy[1];
    const bool move = explicit_station_transition &&
        std::fabs(delta_y) >= onnx_.gait_move_deadband();
    int cycles = 0;
    if (move) {
      cycles = static_cast<int>(std::ceil(
          std::fabs(delta_y) / std::max(onnx_.gait_step_distance(), 1.0e-6)));
      cycles = std::max(1, std::min(cycles, onnx_.gait_max_cycles()));
    }
    const double duration_s = cycles > 0
        ? static_cast<double>(cycles) / std::max(onnx_.gait_frequency_hz(), 1.0e-6)
        : 0.0;
    const int duration_steps = cycles > 0
        ? std::max(1, static_cast<int>(std::ceil(duration_s / std::max(cfg_.dt, 1.0e-6))))
        : 0;
    const double raw_velocity = duration_steps > 0
        ? delta_y / (duration_steps * std::max(cfg_.dt, 1.0e-6)) : 0.0;
    finite_gait_planned_cycles_ = cycles;
    finite_gait_duration_steps_ = duration_steps;
    finite_gait_elapsed_steps_ = 0;
    finite_gait_initial_delta_y_ = delta_y;
    if (cycles > 0) finite_gait_settle_ticks_ = 0;  // a planned STEP restarts the settle clock
    finite_gait_velocity_y_ = std::clamp(
        raw_velocity, -onnx_.gait_velocity_max(), onnx_.gait_velocity_max());
    std::fprintf(stderr,
        "[pp v15 gait] command=%s station_y=%+.3f base_y=%+.3f delta_y=%+.3f "
        "cycles=%d duration=%d ticks vy=%+.3f m/s (residual cannot re-arm)\n",
        cycles > 0 ? "STEP" : "STAND", station_xy[1], measured_base_xy[1], delta_y,
        cycles, duration_steps, finite_gait_velocity_y_);
  }

  bool FiniteLateralGaitFinished_() const {
    return !onnx_.has_finite_lateral_gait() ||
        finite_gait_elapsed_steps_ >= finite_gait_duration_steps_;
  }

  void UpdateFiniteLateralGaitObservation_(PpRacketTarget& target, bool in_hold) {
    if (!onnx_.has_finite_lateral_gait()) return;
    // POST-GAIT SETTLE clock: count consecutive ticks with the gait finished; any active
    // gait tick zeroes it (a newly planned STEP is zeroed in PlanFiniteLateralGait_).
    if (finite_gait_elapsed_steps_ < finite_gait_duration_steps_)
      finite_gait_settle_ticks_ = 0;
    else if (finite_gait_settle_ticks_ <= kFiniteGaitSettleTicks)
      ++finite_gait_settle_ticks_;
    target.desired_lateral_velocity = 0.0;
    target.gait_clock.setZero();
    target.upper_intervention = 0.0;  // HUGWBC action replacement is training-only.
    if (!in_hold) {
      target.locomotion_mode = -1.0;
      return;
    }
    const bool move = finite_gait_elapsed_steps_ < finite_gait_duration_steps_;
    target.locomotion_mode = move ? 1.0 : 0.0;
    if (!move) return;
    target.desired_lateral_velocity = finite_gait_velocity_y_;
    const double phase = std::fmod(
        finite_gait_elapsed_steps_ * std::max(cfg_.dt, 1.0e-6) *
            onnx_.gait_frequency_hz(),
        1.0);
    const double duty = std::clamp(onnx_.gait_duty_factor(), 1.0e-4, 1.0 - 1.0e-4);
    auto remap = [duty](double raw) {
      raw = std::fmod(raw, 1.0);
      if (raw < 0.0) raw += 1.0;
      return raw < duty ? raw * (0.5 / duty)
                        : 0.5 + (raw - duty) * (0.5 / (1.0 - duty));
    };
    // LEADING-FOOT RULE (training hope_commands 2026-07-23 audit): lead with the foot on
    // the side of travel — left leads for +y steps (the legacy fixed assignment), right
    // leads for -y steps. Otherwise a one-cycle rightward step always cross-steps with the
    // left foot first.
    const bool lead_left = finite_gait_velocity_y_ >= 0.0;
    const double left_phase = remap(phase + (lead_left ? 0.5 : 0.0));
    const double right_phase = remap(phase + (lead_left ? 0.0 : 0.5));
    target.gait_clock[0] = std::sin(2.0 * M_PI * left_phase);
    target.gait_clock[1] = std::sin(2.0 * M_PI * right_phase);
    ++finite_gait_elapsed_steps_;
  }

  Eigen::VectorXd ProjectBoundedQdes_(const Eigen::VectorXd& policy_request,
                                      const Eigen::VectorXd& measured_q,
                                      const Eigen::VectorXd& measured_qd) {
    if (!onnx_.has_bounded_qdes_contract()) return policy_request;
    if (policy_request.size() != kNumJoints || measured_q.size() != kNumJoints ||
        measured_qd.size() != kNumJoints)
      throw std::runtime_error("bounded q_des projector received a non-31D vector");

    const auto& safe_lo = onnx_.qdes_safe_lo();
    const auto& safe_hi = onnx_.qdes_safe_hi();
    if (!qdes_projector_initialized_) {
      qdes_projected_isaac_ = measured_q.cwiseMax(safe_lo).cwiseMin(safe_hi);
      qdes_projector_initialized_ = true;
    }

    Eigen::VectorXd projected(kNumJoints);
    Eigen::VectorXd decoded_nominal(kNumJoints);
    int active_count = 0;
    int rate_count = 0;
    int tracking_count = 0;
    int torque_count = 0;
    int infeasible_count = 0;
    int feasible_rate_bound_count = 0;
    int feasible_tracking_bound_count = 0;
    int feasible_torque_bound_count = 0;
    double max_normalized_error = 0.0;
    double max_action_utilization = 0.0;
    double min_interval_width_fraction = 1.0;
    double max_rate_utilization = 0.0;
    const double eps = 1.0e-7;
    for (int i = 0; i < kNumJoints; ++i) {
      const FeasibleQdesV3Result contract = ComputeFeasibleQdesV3({
          onnx_.has_feasible_qdes_contract() ? policy_request[i] : 0.0,
          qdes_projected_isaac_[i],
          measured_q[i],
          measured_qd[i],
          safe_lo[i],
          safe_hi[i],
          onnx_.qdes_rate_limit()[i],
          onnx_.qdes_tracking_error_limit()[i],
          onnx_.qdes_projector_kp()[i],
          onnx_.qdes_projector_kd()[i],
          onnx_.qdes_projector_effort_limit()[i],
          onnx_.qdes_torque_headroom_fraction(),
          onnx_.qdes_projector_dt_s(),
          onnx_.qdes_actual_q_guard_horizon_s(),
          onnx_.has_actual_q_guard_contract(),
      });
      const double rate_delta =
          onnx_.qdes_rate_limit()[i] * onnx_.qdes_projector_dt_s();
      const double rate_lo = contract.rate_lo;
      const double rate_hi = contract.rate_hi;
      const double tracking_lo = contract.tracking_lo;
      const double tracking_hi = contract.tracking_hi;
      const double torque_lo = contract.torque_lo;
      const double torque_hi = contract.torque_hi;
      const double safe_torque_lo = contract.safe_torque_lo;
      const double safe_torque_hi = contract.safe_torque_hi;
      const double base_lo = contract.base_lo;
      const double base_hi = contract.base_hi;
      const bool feasible = contract.full_feasible;
      const double selected_lo = contract.selected_lo;
      const double selected_hi = contract.selected_hi;
      double action_utilization = 0.0;
      if (onnx_.has_feasible_qdes_contract()) {
        decoded_nominal[i] = contract.qdes;
        projected[i] = decoded_nominal[i];
        action_utilization = std::fabs(contract.normalized_action);
      } else {
        decoded_nominal[i] = policy_request[i];
        projected[i] = std::min(std::max(decoded_nominal[i], selected_lo), selected_hi);
      }

      const int sdk = isaac_to_sdk_[i];
      const bool passive_head = sdk == kHeadSlot0 || sdk == kHeadSlot1;
      if (passive_head) {
        decoded_nominal[i] = onnx_.default_q()[i];
        projected[i] = ClampPassiveDefaultToQdesInterval(
            onnx_.default_q()[i], contract);
        continue;
      }
      max_action_utilization = std::max(max_action_utilization, action_utilization);
      const bool feasible_rate_bound = onnx_.has_feasible_qdes_contract() && feasible &&
          (rate_lo > base_lo + eps || rate_hi < base_hi - eps);
      const bool feasible_tracking_bound = onnx_.has_feasible_qdes_contract() &&
          (tracking_lo > safe_torque_lo + eps || tracking_hi < safe_torque_hi - eps);
      const bool feasible_torque_bound = onnx_.has_feasible_qdes_contract() &&
          (torque_lo > safe_lo[i] + eps || torque_hi < safe_hi[i] - eps);
      const bool rate_active = !onnx_.has_feasible_qdes_contract() &&
          (decoded_nominal[i] < rate_lo - eps || decoded_nominal[i] > rate_hi + eps);
      const bool tracking_active = !onnx_.has_feasible_qdes_contract() &&
          (decoded_nominal[i] < tracking_lo - eps || decoded_nominal[i] > tracking_hi + eps);
      const bool torque_active = !onnx_.has_feasible_qdes_contract() &&
          (decoded_nominal[i] < torque_lo - eps || decoded_nominal[i] > torque_hi + eps);
      const bool active = std::fabs(projected[i] - decoded_nominal[i]) > eps;
      active_count += active ? 1 : 0;
      rate_count += rate_active ? 1 : 0;
      tracking_count += tracking_active ? 1 : 0;
      torque_count += torque_active ? 1 : 0;
      infeasible_count += feasible ? 0 : 1;
      feasible_rate_bound_count += feasible_rate_bound ? 1 : 0;
      feasible_tracking_bound_count += feasible_tracking_bound ? 1 : 0;
      feasible_torque_bound_count += feasible_torque_bound ? 1 : 0;
      const double safe_half = std::max(0.5 * (safe_hi[i] - safe_lo[i]), 1.0e-6);
      max_normalized_error =
          std::max(max_normalized_error,
                   std::fabs(projected[i] - decoded_nominal[i]) / safe_half);
      min_interval_width_fraction = std::min(
          min_interval_width_fraction,
          (selected_hi - selected_lo) / std::max(safe_hi[i] - safe_lo[i], 1.0e-6));
      if (rate_delta > eps) {
        max_rate_utilization = std::max(
            max_rate_utilization,
            std::fabs(projected[i] - qdes_projected_isaac_[i]) / rate_delta);
      }
      if (active) ++qdes_projector_joint_count_[i];
    }

    qdes_nominal_isaac_ = decoded_nominal;
    qdes_projected_isaac_ = projected;
    last_qdes_projector_active_count_ = active_count;
    last_qdes_projector_rate_count_ = rate_count;
    last_qdes_projector_tracking_count_ = tracking_count;
    last_qdes_projector_torque_count_ = torque_count;
    last_qdes_projector_infeasible_count_ = infeasible_count;
    last_qdes_feasible_rate_bound_count_ = feasible_rate_bound_count;
    last_qdes_feasible_tracking_bound_count_ = feasible_tracking_bound_count;
    last_qdes_feasible_torque_bound_count_ = feasible_torque_bound_count;
    last_qdes_projector_max_normalized_error_ = max_normalized_error;
    last_qdes_feasible_action_utilization_max_ = max_action_utilization;
    last_qdes_feasible_interval_width_min_ = min_interval_width_fraction;
    last_qdes_feasible_rate_utilization_max_ = max_rate_utilization;
    ++qdes_projector_ticks_;
    return projected;
  }

  void set_planner_status_(const char* s) {
    std::lock_guard<std::mutex> lk(planner_mu_);
    if (planner_status_ != s) planner_status_ = s;
  }

  bool RecordPlannerLifecycle_(const char* event, const char* reason) {
    const std::string next_event = event ? event : "unknown";
    const std::string next_reason = reason ? reason : "unknown";
    if (planner_lifecycle_event_ == next_event && planner_lifecycle_reason_ == next_reason) return false;
    planner_lifecycle_event_ = next_event;
    planner_lifecycle_reason_ = next_reason;
    ++planner_lifecycle_seq_;
    return true;
  }

  void ClearPendingStation_(std::uint64_t tick_idx, const char* reason) {
    // An aborted ACTIVE station walk loses its pending shield mid-motion (the static
    // branch is only locked out while pending is active). Restart the recovery-window
    // clock so the policy hold gets a fresh hold_recover_s to settle before the static
    // handoff may fire — mirrors the 1->0 swing-completion edge. A stale clock here let
    // the +3 s force path freeze a mid-walk robot on the first post-reject tick (0711).
    const bool was_pending = planner_pending_station_active_;
    const int old_clip = planner_pending_clip_;
    const Vec2 old_station = planner_pending_station_w_;
    const double old_strike_time = planner_pending_strike_time_;
    if (was_pending) planner_hold_start_tick_ = tick_idx;
    planner_pending_station_active_ = false;
    planner_station_ready_timer_active_ = false;
    planner_station_ready_reported_ = false;
    planner_pending_strike_time_ = 0.0;
    planner_pending_supported_target_tick_ = 0;
    if (RecordPlannerLifecycle_("clear", reason)) {
      std::fprintf(stderr,
          "[pp lifecycle] seq=%llu event=clear reason=%s pending_was=%d clip=%d "
          "station=(%+.3f,%+.3f) strike=%.6f tts=%.3f\n",
          static_cast<unsigned long long>(planner_lifecycle_seq_), reason,
          was_pending ? 1 : 0, old_clip, old_station[0], old_station[1],
          old_strike_time, planner_current_tts_);
    }
  }

  void BlockOrExpirePendingStation_(std::uint64_t tick_idx, double tts, const char* reason) {
    const auto decision = pending_station_gap_decision(
        planner_pending_station_active_, tts, cfg_.pending_expire_after_strike_s);
    if (decision == PendingStationGapDecision::kExpire) {
      ClearPendingStation_(tick_idx, "shot_expired");
      return;
    }
    if (decision == PendingStationGapDecision::kHoldBlocked) {
      // Never credit READY time across a planner/localization gap. Keep walking/holding the
      // station target, but require a fresh continuous dwell after all release inputs recover.
      planner_station_ready_timer_active_ = false;
      planner_station_ready_reported_ = false;
      if (RecordPlannerLifecycle_("hold_blocked", reason)) {
        std::fprintf(stderr,
            "[pp lifecycle] seq=%llu event=hold_blocked reason=%s pending=1 clip=%d "
            "station=(%+.3f,%+.3f) strike=%.6f tts=%.3f valid_age=%.3f\n",
            static_cast<unsigned long long>(planner_lifecycle_seq_), reason,
            planner_pending_clip_, planner_pending_station_w_[0],
            planner_pending_station_w_[1], planner_pending_strike_time_, tts,
            planner_valid_age_s_);
      }
    }
  }

  void UpdateBaseMotion_(std::uint64_t tick_idx, const Vec3& pos_w, bool localized,
                         std::uint64_t sample_seq) {
    if (!localized) {
      base_motion_initialized_ = false;
      base_speed_xy_valid_ = false;
      base_velocity_xy_valid_ = false;
      base_velocity_xy_est_.setZero();
      return;
    }
    if (base_motion_initialized_ && sample_seq == base_motion_prev_seq_) {
      // Freshness horizon 5 ticks (100 ms), not 2 (40 ms): the sim relay is ~50 Hz with
      // scheduling jitter, and a 40 ms horizon flapped the speed estimate INVALID all run
      // (0711) — speed_ready never held, so station transitions could NEVER settle-engage.
      // Real dropouts (mocap loss) are 100s of ms and still invalidate; the 0.2 s
      // external_base_max_age_s gate independently protects the obs path.
      const std::uint64_t fresh_ticks = onnx_.uses_position_mocap_obs()
          ? static_cast<std::uint64_t>(std::max(
                1.0, std::ceil(onnx_.base_localization_max_age_s() /
                               std::max(cfg_.dt, 1e-6))))
          : 5;
      if (tick_idx > base_motion_last_new_tick_ + fresh_ticks) {
        base_speed_xy_valid_ = false;
        base_velocity_xy_valid_ = false;
        base_velocity_xy_est_.setZero();
      }
      return;  // repeated Latest(): never turn a held pose into an artificial zero-speed sample
    }
    const Vec2 xy(pos_w[0], pos_w[1]);
    if (base_motion_initialized_ && tick_idx > base_motion_prev_tick_) {
      const double dt = (tick_idx - base_motion_prev_tick_) * std::max(cfg_.dt, 1e-6);
      const Vec2 inst = (xy - base_motion_prev_xy_) / dt;
      // Short EMA rejects millimetre-level mocap differentiation noise without hiding a
      // still-moving base. V15 takes alpha from the ONNX/YAML contract; older policies retain
      // the historical 0.25 scalar-speed estimator.
      const double alpha = onnx_.uses_position_mocap_obs()
          ? onnx_.base_velocity_ema_alpha() : 0.25;
      // V15 training starts the filter from zero at reset and applies alpha even to the first
      // differentiated displacement (and again after a stale interval).  Older contracts keep
      // their historical first-sample=instantaneous behavior.
      base_velocity_xy_est_ = onnx_.uses_position_mocap_obs()
          ? alpha * inst + (1.0 - alpha) * base_velocity_xy_est_
          : (base_velocity_xy_valid_
              ? alpha * inst + (1.0 - alpha) * base_velocity_xy_est_ : inst);
      base_velocity_xy_valid_ = std::isfinite(base_velocity_xy_est_[0]) &&
                                std::isfinite(base_velocity_xy_est_[1]);
      base_speed_xy_est_ = base_velocity_xy_est_.norm();
      base_speed_xy_valid_ = base_velocity_xy_valid_ &&
          a3_deploy::numeric_safety::IsFinite(base_speed_xy_est_);
    } else {
      base_speed_xy_valid_ = false;
      // V15's first post-reset actor observation carries a fresh zero velocity.  Keep the
      // readiness speed invalid until a second position arrives, but seed the actor filter as a
      // valid zero so the next differentiated sample receives the same EMA alpha as training.
      base_velocity_xy_valid_ = onnx_.uses_position_mocap_obs();
      base_velocity_xy_est_.setZero();
    }
    base_motion_prev_xy_ = xy;
    base_motion_prev_tick_ = tick_idx;
    base_motion_prev_seq_ = sample_seq;
    base_motion_last_new_tick_ = tick_idx;
    base_motion_initialized_ = true;
  }

  // Live-planner engage machine (Path B). Reproduces the PROVEN Python wbc_runner._tick:
  // while a swing runs, the target is FROZEN and the existing clock/completion owns it (no
  // mid-swing abort on planner flutter); at idle, gate a fresh VALID command (timeout /
  // invalid-grace / min-tts / base-low / reachability) and, if it passes, FREEZE the target
  // and drive the EXISTING controls (set_swing_dir + set_level(1)). Uses the PREVIOUS tick's
  // localized base (1-tick lag @50 Hz is negligible) so it can run before localization.
  void PlannerEngageStep_(std::uint64_t tick_idx) {
    // Policy-native production keeps its historical telemetry-only readiness
    // semantics. The isolated stationary replay is intentionally stricter:
    // every legacy release gate is fail-closed while the learned full-body
    // controller remains active for balance.
    const bool release_gates_advisory =
        cfg_.policy_native &&
        !cfg_.fixed_station_replay &&
        !cfg_.moving_station_replay;
    if (level_.load() == 1) {  // in flight
      // 110-D STREAMING (paper Fig. 3): keep consuming same-side refinements while the swing
      // flies. Every other contract keeps the proven frozen-target behavior.
      if (onnx_.is_hitter_pure_obs() && cfg_.stream_target) StreamTargetStep_(tick_idx);
      set_planner_status_("swinging");
      return;
    }
    planner_engaged_ = false;  // level 0: idle/hold (ready-hold override uses planner_have_hold_)

    // The learned clip already contains its recovery.  The additional legacy
    // inter-swing rest is audit-only in normal policy-native field execution,
    // so a new physical shot is not rejected during this interval.
    const bool legacy_rest_active =
        rest_rearm_armed_ && tick_idx < rest_rearm_tick_;
    if (planner_rest_blocks_release(release_gates_advisory,
                                    legacy_rest_active)) {
      set_planner_status_("rest");
      return;
    }
    if (legacy_rest_active && (gate_warn_tick_++ % 25) == 0) {
      std::fprintf(
          stderr,
          "[pp planner telemetry] legacy inter-swing rest is active; "
          "policy-native release remains available for a new shot\n");
    }

    if (!racket_in_) {
      planner_valid_age_s_ = -1.0;
      planner_current_tts_ = 0.0;
      planner_current_strike_time_ = 0.0;
      ClearPendingStation_(tick_idx, "no_input");
      set_planner_status_("no_input");
      return;
    }
    const auto snap = racket_in_->Latest();
    planner_current_msg_seq_ = snap.seq;
    planner_current_flight_id_ = snap.cmd.flight_id;
    planner_current_revision_id_ = snap.cmd.revision_id;
    planner_current_stable_revision_count_ = snap.cmd.stable_revision_count;
    if (!snap.has_valid) {
      planner_valid_age_s_ = -1.0;
      planner_current_tts_ = 0.0;
      planner_current_strike_time_ = 0.0;
      BlockOrExpirePendingStation_(tick_idx, 0.0, "no_command");
      set_planner_status_("no_command");
      return;
    }
    const bool revisioned_v17 =
        onnx_.is_v17_r10_p0_gate3() ||
        onnx_.is_v17_r12_v11_qdes_tuple_hardware();
    if (planner_revision_release_blocked(
            revisioned_v17, snap.cmd.schema, snap.cmd.stable_revision_count)) {
      ClearPendingStation_(tick_idx, "planner_schema");
      if ((gate_warn_tick_++ % 25) == 0) {
        std::fprintf(
            stderr,
            "[pp input] V17 requires schema-2 planner data "
            "(schema=%d flight=%llu revision=%llu)\n",
            snap.cmd.schema,
            static_cast<unsigned long long>(snap.cmd.flight_id),
            static_cast<unsigned long long>(snap.cmd.revision_id));
      }
      set_planner_status_("planner_schema");
      return;
    }
    if (revisioned_v17 && snap.cmd.stable_revision_count < 3 &&
        (gate_warn_tick_++ % 25) == 0) {
      std::fprintf(
            stderr,
            "[pp planner telemetry] schema-2 revision stability %d/3; "
            "release remains on the ball clock "
            "(flight=%llu revision=%llu)\n",
            snap.cmd.stable_revision_count,
            static_cast<unsigned long long>(snap.cmd.flight_id),
            static_cast<unsigned long long>(snap.cmd.revision_id));
    }

    const double tts = snap.control_time_to_strike_s;
    planner_valid_age_s_ = snap.valid_age_s;
    planner_current_tts_ = tts;
    planner_current_strike_time_ = snap.cmd.strike_time;
    const bool command_stale = snap.valid_age_s > cfg_.command_timeout_s;
    if (command_stale) {
      if (planner_command_health_blocks_release(
              release_gates_advisory, command_stale)) {
        BlockOrExpirePendingStation_(tick_idx, tts, "stale");
        set_planner_status_("stale");
        return;
      }
      if ((gate_warn_tick_++ % 25) == 0) {
        std::fprintf(
            stderr,
            "[pp planner telemetry] latest valid command age %.3f s exceeds "
            "nominal %.3f s; policy-native release remains on the future "
            "strike clock\n",
            snap.valid_age_s, cfg_.command_timeout_s);
      }
    }
    const bool invalid_after_grace =
        snap.invalid_after && snap.valid_age_s > cfg_.planner_invalid_grace_s;
    if (invalid_after_grace) {
      if (planner_command_health_blocks_release(
              release_gates_advisory, invalid_after_grace)) {
        BlockOrExpirePendingStation_(tick_idx, tts, "planner_invalid");
        set_planner_status_("planner_invalid");
        return;
      }
      if ((gate_warn_tick_++ % 25) == 0) {
        std::fprintf(
            stderr,
            "[pp planner telemetry] invalid revision followed the latest valid "
            "command; policy-native release retains that valid future strike\n");
      }
    }
    // Late-command boundary. 110: PER-CLIP via engage_late_cutoff_ (windup - prefix_skip).
    // Policy-native treats this as scheduling/telemetry, while isolated replay can still
    // fail closed. The backhand windup is below the legacy 1.0 s constant, so a scalar
    // boundary would make backhand unreachable under the wait-for-tts semantics below.
    // The side is not chosen yet; compare against the looser (lowest-cutoff) clip here and
    // re-evaluate the selected clip after side selection.
    // ⚠ 2026-07-08 fix: this used windup_MAX, which put the pre-side cutoff ABOVE the
    // backhand's whole engage window, so every backhand serve died here as too_late and
    // the side-specific boundary below never ran (backhand mathematically unreachable in planner
    // mode). The pre-side cutoff must be the MIN of the per-clip cutoffs. Legacy contracts
    // keep the scalar behavior unchanged.
    if (onnx_.is_hitter_pure_obs()) {
      const double cutoff_min = onnx_.is_v17_r12_v11_qdes_tuple_hardware()
          ? std::min(engage_hard_late_cutoff_(0), engage_hard_late_cutoff_(1))
          : std::min(engage_late_cutoff_(0), engage_late_cutoff_(1));
      const double cutoff = std::min(cfg_.engage_min_tts_s, cutoff_min);
      if (planner_timing_blocks_release(
              release_gates_advisory, tts, cutoff)) {
        BlockOrExpirePendingStation_(tick_idx, tts,
                                     tts <= 0.0 ? "expired" : "too_late");
        set_planner_status_(tts <= 0.0 ? "expired" : "too_late");
        return;
      }
      if (tts < cutoff && (gate_warn_tick_++ % 25) == 0) {
        std::fprintf(
            stderr,
            "[pp planner telemetry] TTS %.3f s is below nominal pre-side "
            "cutoff %.3f s; policy-native release remains immediate and the "
            "clip clock will preserve phase continuity\n",
            tts, cutoff);
      }
    } else {
      const double cutoff = cfg_.engage_min_tts_s;
      if (planner_timing_blocks_release(
              release_gates_advisory, tts, cutoff)) {
        BlockOrExpirePendingStation_(tick_idx, tts,
                                     tts <= 0.0 ? "expired" : "too_late");
        set_planner_status_(tts <= 0.0 ? "expired" : "too_late");
        return;
      }
      if (tts < cutoff && (gate_warn_tick_++ % 25) == 0) {
        std::fprintf(
            stderr,
            "[pp planner telemetry] TTS %.3f s is below nominal cutoff %.3f s; "
            "policy-native release continues\n",
            tts, cutoff);
      }
    }

    const bool localization_stale =
        (cfg_.loc_mode == LocMode::kExternalBase && !base_fresh_) ||
        (cfg_.loc_mode == LocMode::kOracle && !oracle_fresh_);
    if (localization_stale) {
      if (!release_gates_advisory) {
        BlockOrExpirePendingStation_(tick_idx, tts, "no_base");
        set_planner_status_("no_base");
        return;
      }
      if ((gate_warn_tick_++ % 100) == 0) {
        std::fprintf(stderr,
            "[pp telemetry] localization stale; policy-native planner release continues "
            "using the last localized base pose\n");
      }
    }

    const Vec3 base_pos = last_base_pos_;
    const Vec4 base_yaw = yaw_quat(last_base_quat_w_);
    if (base_pos[2] < cfg_.base_low_z) {
      if (!release_gates_advisory) {
        ClearPendingStation_(tick_idx, "base_low");
        set_planner_status_("base_low");
        return;
      }
      if ((gate_warn_tick_++ % 100) == 0) {
        std::fprintf(stderr,
            "[pp telemetry] base z %.3f is below legacy gate %.3f; "
            "policy-native release remains armed\n",
            base_pos[2], cfg_.base_low_z);
      }
    }

    // MOTION-entry settle: no engage for the first engage_settle_s of a session (see cfg).
    if (!release_gates_advisory && tick_idx < planner_entry_tick_ +
                       static_cast<std::uint64_t>(cfg_.engage_settle_s / std::max(cfg_.dt, 1e-6))) {
      set_planner_status_("settling"); return;
    }
    // HEADING gate (see cfg.engage_yaw_max_deg): last_base_quat_w_ is yaw-aligned, so its yaw
    // is the drift from the engage heading (~ world +x). Swinging from a yawed stand is OOD.
    {
      const double yaw = std::atan2(
          2.0 * (last_base_quat_w_[0] * last_base_quat_w_[3] +
                 last_base_quat_w_[1] * last_base_quat_w_[2]),
          1.0 - 2.0 * (last_base_quat_w_[2] * last_base_quat_w_[2] +
                       last_base_quat_w_[3] * last_base_quat_w_[3]));
      const bool yaw_outside =
          std::fabs(yaw) > cfg_.engage_yaw_max_deg * M_PI / 180.0;
      if (yaw_outside) {
        if ((gate_warn_tick_++ % 100) == 0)
          std::fprintf(stderr,
              release_gates_advisory
                  ? "[pp telemetry] yaw outside nominal start: %+.0f deg (nominal max %.0f); "
                    "policy-native release continues on the ball clock\n"
                  : "[pp gate] REJECT yawed: base heading %+.0f deg off the engage heading "
                    "(max %.0f) — swings must start square; re-stand ('s', square, 'm') if "
                    "this persists\n",
              yaw * 180.0 / M_PI, cfg_.engage_yaw_max_deg);
        if (!planner_heading_blocks_release(
                release_gates_advisory, yaw_outside)) {
          // Training already observes yaw and learns the recovery behavior. Do
          // not turn this diagnostic into a second release clock in native mode.
        } else {
          // DEADLOCK RELEASE (2026-07-09 G3 forensics): a yawed reject while the STATIC
        // official stand holds the robot is unrecoverable without this — the static latch
        // only clears on level 1 (engage), the engage is exactly what this gate is
        // refusing, and the frozen fixed-pose stand can neither re-square nor balance a
        // yawed stance (measured: handoff near 10 deg, stand rotated/leaned to 22 deg,
        // every subsequent serve rejected 'yawed', stand toppled ~5 s later). Release the
        // latch back to the ACTIVE policy hold — the trained heading-recovery state
        // (hold_heading) is the only thing in the chain that can turn the robot square —
        // and restart the recovery window so the quiescence-gated handoff (near_heading
        // <= static_handoff_yaw_max_deg) re-freezes only once actually square. Chatter-
        // safe: the release fires on a discrete engage ATTEMPT (fresh valid target), not
        // a flapping quiescence condition, and re-handoff needs a full hold_recover_s.
          if (planner_static_active_) {
            planner_static_active_ = false;
            planner_hold_start_tick_ = tick_idx;
            std::fprintf(stderr,
                "[pp] yawed reject during STATIC stand -> releasing to the ACTIVE policy "
                "hold to re-square (%+.0f deg)\n", yaw * 180.0 / M_PI);
          }
          set_planner_status_("yawed"); return;
        }
      }
    }

    // The latest-value mailbox intentionally retains the newest valid command.
    // Once a physical shot has engaged, do not let that same absolute strike
    // event arm another swing after the first clip completes.
    if (planner_have_consumed_shot_ &&
        same_planner_shot(snap.cmd.strike_time, planner_consumed_strike_time_,
                          cfg_.shot_reuse_tolerance_s)) {
      ClearPendingStation_(tick_idx, "shot_consumed");
      set_planner_status_("shot_consumed");
      return;
    }

    // Racket target -> policy WORLD frame. frame_code 0 = same world as the base (planner
    // table frame == mocap world). frame_code 1 = base_link-relative -> lift to world
    // (BOTH position and velocity rotate; a translated-only velocity would mix frames
    // once the robot has turned).
    Vec3 pos_w = snap.cmd.pos_w;
    Vec3 vel_w = snap.cmd.vel_w;
    if (snap.cmd.frame_code == 1) {
      pos_w = base_pos + quat_rotate(base_yaw, snap.cmd.pos_w);
      vel_w = quat_rotate(base_yaw, snap.cmd.vel_w);
    }

    Vec3 tgt_b = quat_rotate_inverse(base_yaw, pos_w - base_pos);

    // Swing side. 110-D hitter_pure: the paper's §V-B-3 heuristic, implemented as
    // NEAREST-STATION — candidate station per side = target_xy − (plane_x, band_center_y),
    // pick the side needing the smaller step. The legacy y<0 split is WRONG for the pure
    // bands (the bh band [−0.05,0.45] crosses y=0: a bh-region ball at station-rel y ∈
    // [−0.10,0) would grab the fh clip + a ~0.6 m wrong station). Legacy contracts keep
    // the y-sign split.
    double sign;
    if (onnx_.is_hitter_pure_obs()) {
      if ((onnx_.is_rally_final_v3_recipe() || onnx_.is_rally_station_recipe()) &&
          std::fabs(snap.cmd.swing_sign) <= 0.5) {
        if ((gate_warn_tick_++ % 50) == 0)
          std::fprintf(stderr,
              "[pp gate] REJECT(110 %s) missing explicit swing_sign; planner flat "
              "schema must publish the intercept-selected forehand/backhand side\n",
              onnx_.is_rally_v17_recipe() ? "RallyV17" :
              (onnx_.is_rally_v14_recipe() ? "RallyV14" :
              (onnx_.is_rally_v13_recipe() ? "RallyV13" :
              (onnx_.is_rally_v12_recipe() ? "RallyV12" :
              (onnx_.is_rally_v11_recipe() ? "RallyV11" :
              (onnx_.is_rally_v10_recipe() ? "RallyV10" :
              (onnx_.is_rally_v9_recipe() ? "RallyV9" :
              (onnx_.is_rally_v8_recipe() ? "RallyV8" : "FinalV3"))))))));
        BlockOrExpirePendingStation_(tick_idx, tts, "missing_side");
        set_planner_status_("missing_side");
        return;
      }
      const Vec2 tgt_xy(pos_w[0], pos_w[1]);
      // RallyFinal training samples each transition from the PREVIOUS commanded station.
      // Use that same anchor for side selection instead of a temporarily off-station base;
      // this keeps the inverse geometry deterministic while recovery finishes.
      const Vec2 side_anchor = (rally_final_station_control_ && hold_station_set_)
          ? hold_station_w_ : Vec2(base_pos[0], base_pos[1]);
      const double d_fh = (tgt_xy - reach_offset_clip_[0] - side_anchor).norm();
      const double d_bh = (tgt_xy - reach_offset_clip_[1] - side_anchor).norm();
      // RallyFinal planner flats carry an explicit side selected from the predicted ball
      // intercept and clip reach geometry. Do not infer it from racket velocity: valid
      // per-clip velocity boxes may cross vy=0. Legacy recipes may still send 0/unspecified.
      sign = (rally_final_station_control_ && std::fabs(snap.cmd.swing_sign) > 0.5)
          ? snap.cmd.swing_sign : ((d_fh <= d_bh) ? 1.0 : -1.0);
    } else {
      // BASE-RELATIVE y (raw world-y is always <0 in the table frame).
      sign = swing_sign_from_target_y(tgt_b[1]);
    }
    const int eng_clip = clip_id_from_swing_sign(sign);
    Vec2 station = Vec2::Zero();
    if (onnx_.is_hitter_pure_obs())
      station = station_from_target_(Vec2(pos_w[0], pos_w[1]), eng_clip);
    if (onnx_.is_v17_r10_p0_gate3()) {
      if (!station_session_origin_set_) {
        ClearPendingStation_(tick_idx, "origin_unset");
        set_planner_status_("origin_unset");
        return;
      }
      const Vec2 relative_target =
          Vec2(pos_w[0], pos_w[1]) - station_session_origin_w_;
      constexpr double kFixedTargetToleranceM = 0.030;
      const bool x_supported =
          std::fabs(relative_target[0] - reach_offset_clip_[eng_clip][0]) <=
          kFixedTargetToleranceM;
      const bool y_supported =
          relative_target[1] >=
              hp_y_band_[eng_clip][0] - kFixedTargetToleranceM &&
          relative_target[1] <=
              hp_y_band_[eng_clip][1] + kFixedTargetToleranceM;
      if (!x_supported || !y_supported) {
        if ((gate_warn_tick_++ % 25) == 0) {
          std::fprintf(
              stderr,
              "[pp gate] V17-r10 target outside immutable-station %s "
              "support: relative=(%+.3f,%+.3f), expected x=%+.3f y=[%+.3f,%+.3f]\n",
              sign > 0 ? "FH" : "BH", relative_target[0],
              relative_target[1], reach_offset_clip_[eng_clip][0],
              hp_y_band_[eng_clip][0], hp_y_band_[eng_clip][1]);
        }
        ClearPendingStation_(tick_idx, "fixed_station_target_unsupported");
        set_planner_status_("fixed_station_target_unsupported");
        return;
      }
      // R10 never solves or updates a station from the ball.  The ball only
      // selects a supported racket target around the immutable MOTION-entry
      // anchor; XY error in the actor observation is the robot's real drift.
      station = station_session_origin_w_;
    }
    if (cfg_.fixed_station_replay) {
      if (cfg_.fixed_y_homing_replay) {
        const double dx =
            station[0] - station_session_origin_w_[0];
        const double dy =
            station[1] - station_session_origin_w_[1];
        const bool y_locked =
            std::fabs(dy) <= cfg_.fixed_station_tolerance_m;
        const bool x_recoverable =
            std::fabs(dx) <= cfg_.station_ready_x_max;
        if (!station_session_origin_set_ || !y_locked || !x_recoverable) {
          if ((gate_warn_tick_++ % 25) == 0)
            std::fprintf(
                stderr,
                "[pp fixed-y-homing] REJECT %s reason=%s "
                "origin=(%+.3f,%+.3f) derived=(%+.3f,%+.3f) "
                "dx=%+.3f m dy=%+.3f m limits=(%.3f,%.3f)\n",
                sign > 0 ? "fh" : "bh",
                !station_session_origin_set_
                    ? "origin_unset"
                    : (!y_locked ? "y_transition_required"
                                 : "x_outside_self_recovery"),
                station_session_origin_w_[0],
                station_session_origin_w_[1], station[0], station[1],
                dx, dy, cfg_.station_ready_x_max,
                cfg_.fixed_station_tolerance_m);
          ClearPendingStation_(
              tick_idx,
              !station_session_origin_set_
                  ? "origin_unset"
                  : (!y_locked ? "y_transition_required"
                               : "x_outside_self_recovery"));
          set_planner_status_("fixed_y_homing_reject");
          return;
        }
        // y is an immutable origin lock. x shares that same origin target but
        // may start within the trained READY range and self-recover through
        // the actor instead of being rejected by the 2 cm lateral lock.
        station = station_session_origin_w_;
      } else {
        const FixedStationReplayDecision decision = DecideFixedStationReplay(
            station_session_origin_set_, station_session_origin_w_[0],
            station_session_origin_w_[1], station[0], station[1],
            cfg_.fixed_station_tolerance_m);
        if (!decision.accept) {
          if ((gate_warn_tick_++ % 25) == 0)
            std::fprintf(
                stderr,
                "[pp stationary-replay] REJECT %s reason=%s "
                "anchor=(%+.3f,%+.3f) derived=(%+.3f,%+.3f) "
                "requested_step=%.3f m limit=%.3f m; ball skipped, target unchanged\n",
                sign > 0 ? "fh" : "bh", decision.reason.c_str(),
                station_session_origin_w_[0], station_session_origin_w_[1],
                station[0], station[1], decision.requested_delta_m,
                cfg_.fixed_station_tolerance_m);
          ClearPendingStation_(tick_idx, decision.reason.c_str());
          set_planner_status_("fixed_station_reject");
          return;
        }
        // The target is already inside the selected arm-reach band. Keep that
        // target untouched and feed the exact immutable station anchor to the
        // base-target observation; no sub-tolerance micro-step is commanded.
        station = Vec2(decision.command_x, decision.command_y);
      }
    }
    const double max_tts0 =
        (clip_.strike_frame(eng_clip) - clip_.seg_start(eng_clip)) * clip_.step_dt;

    bool target_release_blocked = false;
    if (cfg_.target_gate_enable) {
      bool ok;
      if (onnx_.is_hitter_pure_obs()) {
        // METADATA-driven gate against the TRAINED distribution: per-clip z band, required
        // station step, speed cap. No fixed base-relative box — the paper's robot WALKS to
        // targets the arm alone cannot cover (Fig. 4), so reachability is a station question.
        const double base_step = (station - Vec2(base_pos[0], base_pos[1])).norm();
        const Vec2 transition_anchor = planner_pending_station_active_
            ? ((cfg_.station_only && planner_station_ready_reported_)
                   ? hold_station_w_ : planner_pending_origin_station_w_)
            : (hold_station_set_ ? hold_station_w_ : Vec2(base_pos[0], base_pos[1]));
        const double command_step = (station - transition_anchor).norm();
        bool absolute_station_ok = true;
        if (rally_final_station_control_) {
          absolute_station_ok = station_session_origin_set_ &&
              station[0] - station_session_origin_w_[0] >=
                  hp_base_target_range_[0] - cfg_.gate_station_step_margin &&
              station[0] - station_session_origin_w_[0] <=
                  hp_base_target_range_[1] + cfg_.gate_station_step_margin &&
              station[1] - station_session_origin_w_[1] >=
                  hp_base_target_range_[2] - cfg_.gate_station_step_margin &&
              station[1] - station_session_origin_w_[1] <=
                  hp_base_target_range_[3] + cfg_.gate_station_step_margin;
        }
        // x-READINESS (see cfg.gate_station_x_max): x-locked models never trained an
        // x-station step — refuse to swing until the walk-back puts the base ON the plane.
        const double x_err = station[0] - base_pos[0];
        const bool x_ready =
            cfg_.gate_station_x_max <= 0.0 || std::fabs(x_err) <= cfg_.gate_station_x_max;
        if (!x_ready && !(rally_final_station_control_ && cfg_.station_ready_enable)) {
          if ((gate_warn_tick_++ % 50) == 0)
            std::fprintf(stderr,
                release_gates_advisory
                    ? "[pp telemetry] OUTSIDE(110) %s x-not-ready: base %+.2f m %s of the "
                      "station plane (legacy |dx| max %.2f); policy-native release continues\n"
                    : "[pp gate] REJECT(110) %s x-not-ready: base %+.2f m %s of the station "
                      "plane (|dx| max %.2f) — holding for the walk-back; next serve takes it\n",
                sign > 0 ? "fh" : "bh", -x_err, x_err < 0 ? "FORWARD" : "BEHIND",
                cfg_.gate_station_x_max);
          if (planner_target_blocks_release(
                  release_gates_advisory, x_ready)) {
            set_planner_status_("x_not_ready");
            return;
          }
        }
        // Per-clip trained VELOCITY support, per axis ± gate_vel_margin.  V10 samples core OR
        // planner; its bounding union is safety metadata and union-only corners are rejected.
        // Moot under --demo (the demand is replaced by the validated planner-contained center).
        const bool vel_ok = cfg_.vel_cmd_box_center || !hp_vel_box_set_ ||
                            vel_in_hp_box_(eng_clip, vel_w);
        ok = pos_w[2] >= hp_z_band_[eng_clip][0] - cfg_.gate_z_margin &&
             pos_w[2] <= hp_z_band_[eng_clip][1] + cfg_.gate_z_margin &&
             base_step <= cfg_.gate_station_step_max + cfg_.station_ready_y_max &&
             command_step <= cfg_.gate_station_step_max &&
             absolute_station_ok &&
             vel_w.norm() <= cfg_.gate_speed_max && vel_ok;
        if (!ok && (gate_warn_tick_++ % 50) == 0) {
          const auto& vb = hp_vel_box_[eng_clip];
          const char* target_label = release_gates_advisory
              ? "[pp telemetry] OUTSIDE" : "[pp gate] REJECT";
          std::fprintf(stderr,
              "%s(110) %s z_w=%.2f (band[%.2f,%.2f]±%.2f) "
              "base_to_station=%.2f cmd_step=%.2f (<=%.2f) |v|=%.2f (<=%.2f) "
              "vel=(%+.2f,%+.2f,%+.2f)%s tts=%.2f\n",
              target_label, sign > 0 ? "fh" : "bh", pos_w[2], hp_z_band_[eng_clip][0],
              hp_z_band_[eng_clip][1], cfg_.gate_z_margin, base_step, command_step,
              cfg_.gate_station_step_max,
              vel_w.norm(), cfg_.gate_speed_max, vel_w[0], vel_w[1], vel_w[2],
              vel_ok ? ""
                     : (hp_vel_box_set_
                            ? " OUT-OF-BAND"
                            : ""),
              tts);
          if (!vel_ok) {
            if (hp_vel_components_set_) {
              const auto& core = hp_vel_core_box_[eng_clip];
              const auto& planner = hp_vel_planner_box_[eng_clip];
              std::fprintf(stderr,
                  "[pp support] trained support (clip %d, ±%.2f): "
                  "core x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f] OR "
                  "planner x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]; "
                  "union-only combinations are rejected\n",
                  eng_clip, cfg_.gate_vel_margin,
                  core[0], core[1], core[2], core[3], core[4], core[5],
                  planner[0], planner[1], planner[2], planner[3], planner[4], planner[5]);
            } else {
              std::fprintf(stderr,
                  "[pp support] trained vel box (clip %d): x[%.2f,%.2f] y[%.2f,%.2f] "
                  "z[%.2f,%.2f] ±%.2f — planner demand out of the trained envelope; retune "
                  "the planner (delta_t_flight / target_land aim), or --vel-gate-margin\n",
                  eng_clip, vb[0], vb[1], vb[2], vb[3], vb[4], vb[5],
                  cfg_.gate_vel_margin);
            }
          }
        }
      } else {
        ok = tgt_b[0] >= cfg_.gate_x_lo && tgt_b[0] <= cfg_.gate_x_hi &&
             std::abs(tgt_b[1]) <= cfg_.gate_y_abs &&
             pos_w[2] >= cfg_.gate_z_lo && pos_w[2] <= cfg_.gate_z_hi &&
             vel_w.norm() <= cfg_.gate_speed_max;
        if (!ok && (gate_warn_tick_++ % 50) == 0) {
          // Throttled detail print (mirrors the Python runner's gate warn): without the
          // inputs a rejection is undebuggable at the venue.
          const char* target_label = release_gates_advisory
              ? "[pp telemetry] OUTSIDE" : "[pp gate] REJECT";
          std::fprintf(stderr,
              "%s base-rel (%+.2f,%+.2f) z_w=%.2f |v|=%.2f tts=%.2f "
              "(support x[%.2f,%.2f] |y|<=%.2f z[%.2f,%.2f] v<=%.2f)\n",
              target_label, tgt_b[0], tgt_b[1], pos_w[2], vel_w.norm(), tts,
              cfg_.gate_x_lo, cfg_.gate_x_hi, cfg_.gate_y_abs,
              cfg_.gate_z_lo, cfg_.gate_z_hi, cfg_.gate_speed_max);
        }
      }
      if (!ok && planner_target_blocks_release(
                     release_gates_advisory, ok)) {
        // Transient demand flap (one frame drifting past a box edge) must not destroy the
        // settle dwell of a still-identical pending station — clear+recreate resets the
        // readiness timer every time and starved every engage (0711 run 8: three pending
        // re-creations, dwell never reached 0.12 s, serve dead). Keep the pending when the
        // candidate station/clip is unchanged. Stale/invalid/too-late paths above retain the
        // station but block release; no-base is telemetry-only in normal policy-native mode.
        const double station_delta = (station - planner_pending_station_w_).norm();
        const bool same_pending = pending_station_can_progress_during_target_gap(
            planner_pending_station_active_, planner_pending_clip_ == eng_clip,
            station_delta);
        if (!same_pending) {
          ClearPendingStation_(tick_idx, "target_changed_out_of_support");
          set_planner_status_("target_gate");
          return;
        }
        // Do not make a transient strike-target flap consume the station deadline. Continue the
        // unchanged pending walk and READY dwell below, but fail closed at the release boundary
        // until a supported target returns. This matches training, where READY progression is
        // independent of the later strike clock/target validity.
        target_release_blocked = true;
        station = planner_pending_station_w_;
        if (RecordPlannerLifecycle_("hold_target_gap", "target_gate")) {
          std::fprintf(stderr,
              "[pp lifecycle] seq=%llu event=hold_target_gap reason=target_gate pending=1 "
              "clip=%d station=(%+.3f,%+.3f) strike=%.6f tts=%.3f station_delta=%.3f\n",
              static_cast<unsigned long long>(planner_lifecycle_seq_),
              planner_pending_clip_, planner_pending_station_w_[0],
              planner_pending_station_w_[1], planner_pending_strike_time_, tts,
              station_delta);
        }
      }
      if (!ok && release_gates_advisory && (gate_warn_tick_++ % 50) == 0) {
        std::fprintf(stderr,
            "[pp telemetry] target is outside the legacy trained-support gate; "
            "policy-native release continues with the planner command\n");
      }
    }

    // RallyFinal rhythm: use the valid prediction immediately while the clip remains clamped
    // at its windup. The policy first walks laterally to the derived station, then must be
    // position- and speed-ready for a sustained dwell before the clock can arm. No actor input
    // changes: the existing 110-D station delta and racket target carry the pending command.
    if (rally_final_station_control_ && cfg_.station_ready_enable) {
      const bool first_pending = !planner_pending_station_active_;
      const bool shot_changed = !first_pending && planner_shot_changed(
          snap.cmd.strike_time, planner_pending_strike_time_, cfg_.shot_reuse_tolerance_s);
      // A V15 station is a per-shot command, not a continuously moving servo target.  Latch
      // the first station for this physical shot; later fit refinements may still update the
      // racket target, but cannot move the feet's goal or restart a gait cycle.
      if (onnx_.has_finite_lateral_gait() && !first_pending && !shot_changed)
        station = planner_pending_station_w_;
      const bool station_changed = first_pending || shot_changed ||
          (station - planner_pending_station_w_).norm() > 0.02 || eng_clip != planner_pending_clip_;
      const Vec2 command_origin = hold_station_set_
          ? hold_station_w_ : Vec2(base_pos[0], base_pos[1]);
      if (first_pending) {
        planner_pending_origin_station_w_ = hold_station_set_
            ? hold_station_w_ : Vec2(base_pos[0], base_pos[1]);
      } else if (shot_changed) {
        // The next physical ball may request the same station. Shot identity, not station delta,
        // owns the transition boundary and resets latency/coverage from the last held station.
        planner_pending_origin_station_w_ = hold_station_set_
            ? hold_station_w_ : Vec2(base_pos[0], base_pos[1]);
      } else if (station_changed && cfg_.station_only && planner_station_ready_reported_) {
        // No swing completion exists in Gate3A to clear the pending latch. Treat the
        // previously READY station as the next transition's command origin, matching
        // production's post-engage hold_station semantics.
        planner_pending_origin_station_w_ = hold_station_w_;
      }
      if (station_changed) {
        planner_station_ready_timer_active_ = false;
        planner_station_ready_reported_ = false;
        planner_pending_start_tick_ = tick_idx;
        if (first_pending || (gate_warn_tick_++ % 25) == 0) {
          const char* release_note = release_gates_advisory
              ? "READY is telemetry; release follows the ball clock"
              : "windup clamped until ready";
          std::fprintf(stderr,
              "[pp station] pending %s station=(%+.3f,%+.3f) cmd_step=%.3f m "
              "tts=%.2f s -- %s\n",
              sign > 0 ? "fh" : "bh", station[0], station[1],
              (station - planner_pending_origin_station_w_).norm(), tts, release_note);
        }
      }
      if (onnx_.has_finite_lateral_gait() && (first_pending || shot_changed)) {
        const bool explicit_station_transition =
            std::fabs(station[1] - command_origin[1]) >= onnx_.gait_move_deadband();
        PlanFiniteLateralGait_(Vec2(base_pos[0], base_pos[1]), station,
                               explicit_station_transition);
      }
      planner_pending_station_active_ = true;
      prefirst_active_station_tracking_started_ = true;
      planner_pending_station_w_ = station;
      planner_pending_clip_ = eng_clip;
      if (a3_deploy::numeric_safety::IsFinite(snap.cmd.strike_time) &&
          snap.cmd.strike_time > 0.0) {
        planner_pending_strike_time_ = snap.cmd.strike_time;
      }
      if (!target_release_blocked) {
        planner_pending_pos_w_ = pos_w;
        planner_pending_vel_w_ = cfg_.vel_cmd_box_center
            ? cfg_.racket_vel_w_clip[eng_clip] : vel_w;
        planner_pending_supported_target_tick_ = tick_idx;
      }
      if (station_changed) RecordPlannerLifecycle_("pending", shot_changed ? "new_shot" : "new_station");
      hold_station_w_ = station;
      hold_station_set_ = true;
      if (!target_release_blocked) planner_hold_z_w_ = pos_w[2];
      set_swing_dir(sign >= 0.0 ? 1 : -1);
      // A static official stand cannot walk. Hand control back to the level-0 policy as
      // soon as a pending station exists; the static branch below explicitly stays out.
      if (planner_static_active_) {
        planner_static_active_ = false;
        planner_policy_takeover_active_ = true;
        planner_policy_takeover_start_tick_ = tick_idx;
        planner_policy_takeover_q0_ = last_q_des_.size() == kNumJoints
            ? last_q_des_ : nominal_q_sdk_;
        planner_hold_start_tick_ = tick_idx;
        std::fprintf(stderr,
            "[pp station] fresh target -> ACTIVE policy station tracking (static released)\n");
      }

      const double x_err = std::fabs(base_pos[0] - station[0]);
      const double y_err = std::fabs(base_pos[1] - station[1]);
      // Once drift-homing starts, finish the hysteresis cycle before releasing
      // another ball. Otherwise crossing the looser READY boundary can launch
      // a new swing while the base is still between enter and exit thresholds.
      const bool homing_ready =
          !cfg_.fixed_y_homing_replay || !fixed_y_homing_active_;
      const bool position_ready =
          (cfg_.station_ready_x_max <= 0.0 || x_err <= cfg_.station_ready_x_max) &&
          y_err <= cfg_.station_ready_y_max && homing_ready;
      const bool speed_ready = base_speed_xy_valid_ &&
          base_speed_xy_est_ <= cfg_.station_ready_speed_max;
      // A planner can drift by <2 cm on every tick and evade the per-tick
      // station_changed test while moving materially over the 0.12 s dwell. Latch the
      // command at dwell start and require cumulative station+side stability until READY.
      if (planner_station_ready_timer_active_ &&
          ((station - planner_station_dwell_anchor_w_).norm() > 0.02 ||
           eng_clip != planner_station_dwell_anchor_clip_)) {
        planner_station_ready_timer_active_ = false;
      }
      if (position_ready && speed_ready && !planner_policy_takeover_active_) {
        if (!planner_station_ready_timer_active_) {
          planner_station_ready_timer_active_ = true;
          planner_station_ready_since_tick_ = tick_idx;
          planner_station_dwell_anchor_w_ = station;
          planner_station_dwell_anchor_clip_ = eng_clip;
        }
      } else {
        planner_station_ready_timer_active_ = false;
      }
      const double dwell = planner_station_ready_timer_active_
          ? (tick_idx - planner_station_ready_since_tick_) * cfg_.dt : 0.0;
      const bool station_ready = planner_station_ready_timer_active_ &&
          dwell + 1e-9 >= cfg_.station_ready_hold_s;
      if (!station_ready) {
        if ((gate_warn_tick_++ % 50) == 0)
          std::fprintf(stderr,
              release_gates_advisory
                  ? "[pp station telemetry] %s: dx=%.3f dy=%.3f speed=%s%.3f m/s "
                    "dwell=%.2f/%.2f s; ball-clock release remains armed\n"
                  : "[pp station] %s: dx=%.3f dy=%.3f speed=%s%.3f m/s "
                    "dwell=%.2f/%.2f s\n",
              position_ready ? "settling" : "moving", x_err, y_err,
              base_speed_xy_valid_ ? "" : "INVALID/", base_speed_xy_est_, dwell,
              cfg_.station_ready_hold_s);
        set_planner_status_(position_ready ? "settling_station" : "moving_station");
        if (planner_station_blocks_release(
                release_gates_advisory, cfg_.station_only, station_ready))
          return;
      }
      // Gate3A stops at the exact runner readiness boundary. This is deliberately inside
      // the policy rather than emulated by a harness-side pose threshold: it therefore
      // includes the production localization freshness, EMA speed, takeover blend, and
      // sustained dwell semantics. Repeated command messages do not repeat the event.
      if (station_ready && !planner_station_ready_reported_) {
        const double latency =
            (tick_idx - planner_pending_start_tick_) * std::max(cfg_.dt, 1e-6);
        std::fprintf(stderr,
            cfg_.station_only
                ? "[pp station-only] READY %s station=(%+.3f,%+.3f) cmd_step=%.3f m "
                  "dx=%.3f dy=%.3f speed=%.3f m/s dwell=%.2f s latency=%.3f s "
                  "-- swing release inhibited\n"
                : "[pp station telemetry] READY %s station=(%+.3f,%+.3f) cmd_step=%.3f m "
                  "dx=%.3f dy=%.3f speed=%.3f m/s dwell=%.2f s latency=%.3f s\n",
            sign > 0 ? "fh" : "bh", station[0], station[1],
            (station - planner_pending_origin_station_w_).norm(), x_err, y_err,
            base_speed_xy_est_, dwell, latency);
        planner_station_ready_reported_ = true;
      }
      if (cfg_.station_only) {
        std::fprintf(stderr,
                     "[pp station-only] READY %s station=(%+.3f,%+.3f); strike release inhibited\n",
                     sign > 0 ? "fh" : "bh", station[0], station[1]);
        set_planner_status_("station_ready");
        return;
      }
      // Training always executes the finite STEP during the pre-swing hold and then leaves
      // >= 16 STAND control ticks before the swing releases.  Even --policy-native may not
      // release the strike while that one gait cycle is active or inside the post-gait
      // settle margin (kFiniteGaitSettleTicks consecutive finished ticks = 0.3 s at 50 Hz);
      // readiness pose/speed gates remain telemetry, but the command ordering is invariant.
      if (onnx_.has_finite_lateral_gait() &&
          (!FiniteLateralGaitFinished_() ||
           finite_gait_settle_ticks_ < kFiniteGaitSettleTicks)) {
        set_planner_status_("finite_gait");
        return;
      }
    }

    if (target_release_blocked) {
      const double supported_age_s = planner_pending_supported_target_tick_ > 0 &&
              tick_idx >= planner_pending_supported_target_tick_
          ? (tick_idx - planner_pending_supported_target_tick_) * std::max(cfg_.dt, 1e-6)
          : -1.0;
      const bool in_release_window = tts <= max_tts0;
      const bool use_supported_latch = in_release_window &&
          pending_target_latch_can_release(
              planner_pending_station_active_, planner_station_ready_reported_,
              planner_pending_clip_ == eng_clip, supported_age_s,
              cfg_.planner_invalid_grace_s);
      if (!use_supported_latch) {
        set_planner_status_("target_gate");
        return;
      }
      // The current fit is outside support, but the exact same READY shot had a supported
      // command only moments ago.  Use that supported command for this release tick instead of
      // accepting the OOD fit or missing the whole clip.  The per-clip timing boundary below
      // still owns the replay profile; policy-native keeps it audit-only.
      pos_w = planner_pending_pos_w_;
      vel_w = planner_pending_vel_w_;
      tgt_b = quat_rotate_inverse(base_yaw, pos_w - base_pos);
      target_release_blocked = false;
      if (RecordPlannerLifecycle_("release_latched_target", "target_gap_grace")) {
        std::fprintf(stderr,
            "[pp lifecycle] seq=%llu event=release_latched_target "
            "reason=target_gap_grace pending=1 clip=%d station=(%+.3f,%+.3f) "
            "strike=%.6f tts=%.3f supported_age=%.3f\n",
            static_cast<unsigned long long>(planner_lifecycle_seq_),
            planner_pending_clip_, planner_pending_station_w_[0],
            planner_pending_station_w_[1], planner_pending_strike_time_, tts,
            supported_age_s);
      }
    }

    // Strike-time alignment. 110: WAIT-until-tts (paper: the hit time comes from the
    // virtual-plane crossing and the strike fires when the ball arrives). The legacy clamp
    // planner_tts0_ = min(tts, max_tts0) starts the clip early and lets the strike frame
    // fire (planner_tts − max_tts0) seconds BEFORE the ball (bh: >1 s early on a slow lob
    // = multi-decimeter miss). Wait at ready until the decaying tts enters the windup
    // window, then engage with the strike frame exactly on the predicted arrival. Re-evaluate
    // the per-clip timing boundary now that the side is known.
    if (onnx_.is_hitter_pure_obs()) {
      // model_21800 is rally_v14 and retains a fixed near-static-prefix sample.
      // Offline spin replay is still unqualified, so this block makes no claim
      // about target accuracy. It does not reject a command: before the sample
      // time we keep replacing it with the latest revision, and a positive-TTS
      // command that arrives later engages immediately from a phase-continuous
      // near-static start.
      const bool late_commit =
          onnx_.is_rally_v14_recipe() ||
          onnx_.is_v17_r12_v11_qdes_tuple_hardware();
      const double hard_late_tts = late_commit
          ? engage_hard_late_cutoff_(eng_clip)
          : engage_late_cutoff_(eng_clip);
      const double cutoff = std::min(cfg_.engage_min_tts_s, hard_late_tts);
      if (planner_timing_blocks_release(
              release_gates_advisory, tts, cutoff)) {
        BlockOrExpirePendingStation_(
            tick_idx, tts, tts <= 0.0 ? "expired" : "too_late_clip");
        set_planner_status_(tts <= 0.0 ? "expired" : "too_late");
        return;
      }
      if (tts < cutoff && (gate_warn_tick_++ % 25) == 0) {
        std::fprintf(
            stderr,
            "[pp planner telemetry] TTS %.3f s is below nominal %s cutoff "
            "%.3f s; policy-native release remains immediate and the clip "
            "clock will preserve phase continuity\n",
            tts, eng_clip == 0 ? "forehand" : "backhand", cutoff);
      }
      const double commit_tts = late_commit
          ? engage_target_sample_tts_(eng_clip)
          : max_tts0;
      if (tts > commit_tts) {
        set_planner_status_(late_commit ? "tracking_latest_revision" : "waiting_tts");
        return;
      }
      // A normal command starts at its exact ball clock. If a positive command
      // arrives deeper than the qualified prefix, engage it immediately but
      // start at the deepest near-static frame instead of teleporting the actor
      // into a large dynamic pose. No stability/READY condition is consulted.
      const auto phase_start = planner_phase_continuous_start(
          late_commit, tts, hard_late_tts);
      planner_tts0_ = phase_start.clock_tts_s;
      planner_late_phase_clamped_ = phase_start.late_phase_clamped;
      planner_engage_expected_strike_lateness_s_ =
          phase_start.expected_strike_lateness_s;
    } else {
      // ENGAGE: tts0 stored CLAMPED to the clip's windup length; DRIVES the swing clock
      // (ScriptedTarget planner branch: tts = tts0 - t). Mirrors wbc_runner's
      // `"tts0": min(tts, max_tts0)`; without the transfer every strike would be late by
      // (max_tts - planner_tts).
      planner_tts0_ = std::min(tts, max_tts0);
      planner_late_phase_clamped_ = false;
    }
    planner_engage_raw_tts_ = tts;
    planner_engage_clock_tts0_ = planner_tts0_;
    planner_engage_requested_phase_s_ = std::max(0.0, max_tts0 - tts);
    planner_engage_actual_phase_s_ = std::max(0.0, max_tts0 - planner_tts0_);
    if (!onnx_.is_hitter_pure_obs()) {
      planner_engage_expected_strike_lateness_s_ =
          std::max(0.0, planner_tts0_ - tts);
    }
    planner_frozen_pos_w_ = pos_w;
    planner_frozen_vel_w_ = (onnx_.is_hitter_pure_obs() && cfg_.vel_cmd_box_center)
                                ? cfg_.racket_vel_w_clip[eng_clip]
                                : vel_w;
    planner_frozen_sign_ = sign;
    // Freeze the complete identity/timing tuple from the exact same mailbox
    // snapshot as position and velocity. These fields are audit only; actor
    // observations and the frozen-target control contract are unchanged.
    planner_frozen_command_seq_ = snap.cmd.command_seq;
    planner_frozen_flight_id_ = snap.cmd.flight_id;
    planner_frozen_revision_id_ = snap.cmd.revision_id;
    planner_frozen_strike_time_ = snap.cmd.strike_time;
    planner_frozen_raw_tts_ = tts;
    planner_hold_pos_b_engage_ = tgt_b;
    planner_hold_z_w_ = pos_w[2];
    planner_have_hold_ = true;
    planner_engaged_ = true;
    ++planner_shot_seq_;
    const bool was_pending_station = planner_pending_station_active_;
    const bool station_was_ready = planner_station_ready_reported_;
    if (a3_deploy::numeric_safety::IsFinite(snap.cmd.strike_time) &&
        snap.cmd.strike_time > 0.0) {
      planner_consumed_strike_time_ = snap.cmd.strike_time;
      planner_have_consumed_shot_ = true;
    }
    ClearPendingStation_(tick_idx, "engaged");  // hold station remains the recovery/next-transition anchor
    set_swing_dir(sign >= 0.0 ? 1 : -1);
    set_level(1);
    std::fprintf(stderr,
        "[pp engage] %s %s: tgt base-rel (%+.2f,%+.2f,%+.2f) tts=%.2fs "
        "(clock tts0=%.2fs) station=(%+.3f,%+.3f) dx=%.3f dy=%.3f "
        "speed=%s%.3f flight=%llu revision=%llu stable=%d "
        "late_phase_clamped=%d%s\n",
        sign > 0 ? "forehand" : "backhand",
        (onnx_.is_hitter_pure_obs() && cfg_.stream_target) ? "engaged (streaming)" : "locked",
        tgt_b[0], tgt_b[1], tgt_b[2], tts, planner_tts0_, station[0], station[1],
        std::fabs(base_pos[0] - station[0]), std::fabs(base_pos[1] - station[1]),
        base_speed_xy_valid_ ? "" : "INVALID/", base_speed_xy_est_,
        static_cast<unsigned long long>(snap.cmd.flight_id),
        static_cast<unsigned long long>(snap.cmd.revision_id),
        snap.cmd.stable_revision_count,
        planner_late_phase_clamped_ ? 1 : 0,
        was_pending_station
            ? (station_was_ready ? " READY" : " READY=telemetry-not-met")
            : "");
    set_planner_status_("engage");
  }

  // 110-D stream-until-contact (paper Fig. 3: the planner's prediction error converges to ~0
  // at contact and the WBC consumes the stream — there is no lock-at-engage in HITTER).
  // Refresh WHERE (pos/vel) from the latest valid command while the swing flies; the side and
  // the swing clock (WHEN) stay engage-latched (training never varies tts mid-swing). Guards:
  // fresh+valid, same side under the nearest-station heuristic (a planner re-side mid-swing
  // is ignored), locked-side band membership, speed cap, and a tts floor mirroring training's
  // midswing_resample_tts_floor so the final approach is not perturbed.
  void StreamTargetStep_(std::uint64_t tick_idx) {
    if (!racket_in_) return;
    const auto snap = racket_in_->Latest();
    planner_current_msg_seq_ = snap.seq;
    if (!snap.has_valid || snap.invalid_after) return;
    if (snap.valid_age_s > cfg_.command_timeout_s) return;
    const std::uint64_t origin = swing_clock_origin_.load();
    const double t = (tick_idx >= origin ? tick_idx - origin : 0) * cfg_.dt * swing_speed_.load();
    if (planner_tts0_ - t < cfg_.stream_tts_floor_s) return;  // freeze near the strike
    Vec3 pos_w = snap.cmd.pos_w;
    Vec3 vel_w = snap.cmd.vel_w;
    if (snap.cmd.frame_code == 1) {
      const Vec4 base_yaw = yaw_quat(last_base_quat_w_);
      pos_w = last_base_pos_ + quat_rotate(base_yaw, snap.cmd.pos_w);
      vel_w = quat_rotate(base_yaw, snap.cmd.vel_w);
    }
    const int c = clip_id_from_swing_sign(planner_frozen_sign_);
    const Vec2 tgt_xy(pos_w[0], pos_w[1]);
    const Vec2 base_xy(last_base_pos_[0], last_base_pos_[1]);
    if ((tgt_xy - reach_offset_clip_[1 - c] - base_xy).norm() <
        (tgt_xy - reach_offset_clip_[c] - base_xy).norm())
      return;  // nearest-station now prefers the OTHER side: keep the locked target
    if (pos_w[2] < hp_z_band_[c][0] - cfg_.gate_z_margin ||
        pos_w[2] > hp_z_band_[c][1] + cfg_.gate_z_margin)
      return;
    if (vel_w.norm() > cfg_.gate_speed_max) return;
    // Same trained-vel-box membership as engage: a mid-swing refinement must not drag the
    // velocity command out of the trained envelope (keep the engage-gated value instead).
    if (!cfg_.vel_cmd_box_center && hp_vel_box_set_ && !vel_in_hp_box_(c, vel_w)) return;
    planner_frozen_pos_w_ = pos_w;
    if (!cfg_.vel_cmd_box_center) planner_frozen_vel_w_ = vel_w;  // box-center mode: vel stays pinned
  }

  // Station inversion for a racket target (110-D hitter_pure). Legacy semantics: station =
  // target − per-clip (plane_x, band_CENTER_y) — every serve re-centers the ball in the band,
  // i.e. every serve is a step. RallyV8 semantics (stay_if_reachable_): the trained contract is
  // MOSTLY-STATIONARY — keep the HELD station when the target y falls inside its arm-reach band
  // [station_y + band_lo, station_y + band_hi]; otherwise step the MINIMUM lateral distance
  // (target snaps to the nearest band edge). x is always the fixed plane subtraction. The
  // clamp anchors on hold_station_w_ (the previous commanded station — the same anchor V8
  // training samples transitions from); with no held station yet it falls back to band-center.
  Vec2 station_from_target_(const Vec2& tgt_xy, int clip) const {
    Vec2 station(tgt_xy[0] - reach_offset_clip_[clip][0],
                 tgt_xy[1] - reach_offset_clip_[clip][1]);
    if (stay_if_reachable_ && hold_station_set_) {
      const double lo = hp_y_band_[clip][0], hi = hp_y_band_[clip][1];
      if (hi > lo) {
        const double rel_y = tgt_xy[1] - hold_station_w_[1];
        if (rel_y >= lo && rel_y <= hi)
          station[1] = hold_station_w_[1];      // reachable by arm extension: stay
        else if (rel_y > hi)
          station[1] = tgt_xy[1] - hi;          // minimum step toward +y (band edge)
        else
          station[1] = tgt_xy[1] - lo;          // minimum step toward −y (band edge)
      }
    }
    return station;
  }

  // 110 late-command phase boundary for one clip.  For model_21800 this is no longer an
  // admission gate: a command below the boundary still engages immediately, but its actor
  // clock starts no deeper than this near-static prefix.  The requested prefix skip is
  // clamped to [0.0, 0.45*windup]; the cap stays inside the measured ready prefix.
  double engage_late_cutoff_(int clip) const {
    const double windup =
        (clip_.strike_frame(clip) - clip_.seg_start(clip)) * clip_.step_dt;
    return planner_prefix_commit_tts(windup, cfg_.engage_prefix_skip_s);
  }

  double engage_hard_late_cutoff_(int clip) const {
    const double windup =
        (clip_.strike_frame(clip) - clip_.seg_start(clip)) * clip_.step_dt;
    return planner_prefix_hard_late_tts(windup);
  }

  // model_21800/rally_v14 keeps replacing the pending snapshot throughout the
  // whole near-static prefix and atomically freezes it at the fixed dynamic
  // boundary. A positive command first seen after that instant is still
  // accepted immediately by planner_phase_continuous_start().
  double engage_target_sample_tts_(int clip) const {
    const double windup =
        (clip_.strike_frame(clip) - clip_.seg_start(clip)) * clip_.step_dt;
    return planner_target_sample_tts(
        onnx_.is_rally_v14_recipe(), windup, cfg_.engage_prefix_skip_s);
  }

  // vel_w inside the per-clip trained hitter_pure support, per axis ± gate_vel_margin. V10
  // samples two components, so membership must be core OR planner rather than their union box.
  // Engage and mid-swing refinement both call this one helper.
  bool vel_in_hp_box_(int clip, const Vec3& v) const {
    const double m = cfg_.gate_vel_margin;
    if (hp_vel_components_set_)
      return velocity_in_component_support(
          hp_vel_core_box_[clip], hp_vel_planner_box_[clip],
          v[0], v[1], v[2], m);
    const auto& box = hp_vel_box_[clip];
    return velocity_in_box(box, v[0], v[1], v[2], m);
  }

  // One-shot first-tick diagnostic dump (stderr). action = raw Isaac-order policy
  // output; q_sdk/kp_sdk/kd_sdk = final backend-slot command; st = policy-frame
  // robot state; state = raw backend RobotState (SDK order).
  void LogFirstTick(const Eigen::VectorXd& obs, const Eigen::VectorXd& action,
                    const Eigen::VectorXd& q_sdk, const Eigen::VectorXd& kp_sdk,
                    const Eigen::VectorXd& kd_sdk, const PpRobotState& st,
                    const robot_io::RobotState& state, int time_step) const {
    auto S = [](const Eigen::VectorXd& v) {
      char b[96];
      std::snprintf(b, sizeof b, "min=%+.4f mean=%+.4f max=%+.4f |.|=%.4f",
                    v.minCoeff(), v.mean(), v.maxCoeff(), v.norm());
      return std::string(b);
    };
    std::fprintf(stderr,
        "\n===================== [pp FIRST-TICK DEBUG] =====================\n");
    std::fprintf(stderr, " loc_mode=%s  time_step=%d  swing_level=%d  obs_dim=%d act_dim=%d\n",
                 loc_mode_name(), time_step, level_.load(), (int)obs.size(), (int)action.size());
    const Vec3 g = last_proj_grav_;
    std::fprintf(stderr,
        " IMU: base_quat(wxyz)=[%+.3f %+.3f %+.3f %+.3f] proj_grav=[%+.3f %+.3f %+.3f]\n"
        "      gyro=[%+.3f %+.3f %+.3f] sec_imu=%d torso_quat=[%+.3f %+.3f %+.3f %+.3f]\n",
        st.base_quat_w[0], st.base_quat_w[1], st.base_quat_w[2], st.base_quat_w[3],
        g[0], g[1], g[2], state.imu_gyro[0], state.imu_gyro[1], state.imu_gyro[2],
        (int)state.has_secondary_imu, st.torso_quat_w[0], st.torso_quat_w[1],
        st.torso_quat_w[2], st.torso_quat_w[3]);
    if (state.q.size() == kNumJoints) std::fprintf(stderr, " STATE(SDK) q : %s\n", S(state.q).c_str());
    if (state.dq.size() == kNumJoints) std::fprintf(stderr, " STATE(SDK) qd: %s\n", S(state.dq).c_str());
    struct Blk { const char* n; int lo; int len; };
    static const Blk blks180[] = {
        {"command", 0, 62}, {"motion_anchor_pos_b", 62, 3}, {"motion_anchor_ori_b", 65, 6},
        {"base_ang_vel", 71, 3}, {"joint_pos_rel", 74, 31}, {"joint_vel", 105, 31},
        {"actions(last)", 136, 31}, {"projected_gravity", 167, 3}, {"base_target_pos_b", 170, 2},
        {"racket_target_pos_b", 172, 3}, {"racket_target_vel_w", 175, 3},
        {"time_to_strike", 178, 1}, {"swing_type", 179, 1}};
    // deploy_parity 175-D: motion_anchor_pos_b + base_target_pos_b dropped; racket_target_pos_b is
    // relative to the CURRENT racket FK (not base). Everything after motion_anchor_ori_b shifts down 3.
    static const Blk blks175[] = {
        {"command", 0, 62}, {"motion_anchor_ori_b", 62, 6}, {"base_ang_vel", 68, 3},
        {"joint_pos_rel", 71, 31}, {"joint_vel", 102, 31}, {"actions(last)", 133, 31},
        {"projected_gravity", 164, 3}, {"racket_target_pos_b(relFK)", 167, 3},
        {"racket_target_vel_w", 170, 3}, {"time_to_strike", 173, 1}, {"swing_type", 174, 1}};
    // hitter_footwork 177-D: the 175 layout + base_target_pos_b(2) station Δxy re-inserted
    // after projected_gravity; everything after it shifts up 2.
    static const Blk blks177[] = {
        {"command", 0, 62}, {"motion_anchor_ori_b", 62, 6}, {"base_ang_vel", 68, 3},
        {"joint_pos_rel", 71, 31}, {"joint_vel", 102, 31}, {"actions(last)", 133, 31},
        {"projected_gravity", 164, 3}, {"base_target_pos_b", 167, 2},
        {"racket_target_pos_b(relFK)", 169, 3}, {"racket_target_vel_w", 172, 3},
        {"time_to_strike", 175, 1}, {"swing_type", 176, 1}};
    // hitter_pure 110-D (HITTER Table-I exact): no reference stream, no swing_type;
    // world-frame deltas + e_base,x. Matches training contract `hitter_pure`.
    static const Blk blks110[] = {
        {"base_ang_vel", 0, 3}, {"joint_pos_rel", 3, 31}, {"joint_vel", 34, 31},
        {"actions(last)", 65, 31}, {"projected_gravity", 96, 3}, {"base_forward_xy", 99, 2},
        {"base_target_delta_xy(world)", 101, 2}, {"racket_target_rel_base(world)", 103, 3},
        {"racket_target_vel_w", 106, 3}, {"time_to_strike", 109, 1}};
    static const Blk blks113[] = {
        {"base_ang_vel", 0, 3}, {"joint_pos_rel", 3, 31}, {"joint_vel", 34, 31},
        {"executed_qdes_feedback", 65, 31}, {"projected_gravity", 96, 3},
        {"base_forward_xy", 99, 2}, {"base_target_delta_xy(world)", 101, 2},
        {"racket_target_rel_base(world)", 103, 3}, {"racket_target_vel_w", 106, 3},
        {"time_to_strike", 109, 1}, {"base_velocity_xy(mocap)", 110, 2},
        {"localization_age", 112, 1}};
    static const Blk blks118[] = {
        {"base_ang_vel", 0, 3}, {"joint_pos_rel", 3, 31}, {"joint_vel", 34, 31},
        {"executed_qdes_feedback", 65, 31}, {"projected_gravity", 96, 3},
        {"base_forward_xy", 99, 2}, {"base_target_delta_xy(world)", 101, 2},
        {"racket_target_rel_base(world)", 103, 3}, {"racket_target_vel_w", 106, 3},
        {"time_to_strike", 109, 1}, {"base_velocity_xy(mocap)", 110, 2},
        {"localization_age", 112, 1}, {"desired_lateral_velocity", 113, 1},
        {"gait_clock(left,right)", 114, 2}, {"locomotion_mode", 116, 1},
        {"upper_intervention(deploy=0)", 117, 1}};
    std::fprintf(stderr, " OBS blocks (%d-D):\n", (int)obs.size());
    const Blk* blks = (obs.size() == kObsDim175) ? blks175
                    : (obs.size() == kObsDim177) ? blks177
                    : (obs.size() == kObsDim118) ? blks118
                    : (obs.size() == kObsDim113) ? blks113
                    : (obs.size() == kObsDim110) ? blks110
                                                 : blks180;
    const int nblk = (obs.size() == kObsDim175) ? (int)(sizeof(blks175) / sizeof(Blk))
                   : (obs.size() == kObsDim177) ? (int)(sizeof(blks177) / sizeof(Blk))
                   : (obs.size() == kObsDim118) ? (int)(sizeof(blks118) / sizeof(Blk))
                   : (obs.size() == kObsDim113) ? (int)(sizeof(blks113) / sizeof(Blk))
                   : (obs.size() == kObsDim110) ? (int)(sizeof(blks110) / sizeof(Blk))
                                                : (int)(sizeof(blks180) / sizeof(Blk));
    for (int i = 0; i < nblk; ++i)
      std::fprintf(stderr, "   %-24s [%3d:%3d] %s\n", blks[i].n, blks[i].lo, blks[i].lo + blks[i].len,
                   S(obs.segment(blks[i].lo, blks[i].len)).c_str());
    std::fprintf(stderr, " ACTION(raw,Isaac)[31]: %s\n", S(action).c_str());
    if (onnx_.has_bounded_qdes_contract()) {
      std::fprintf(stderr, " Q_DES(nominal,Isaac) : %s\n", S(qdes_nominal_isaac_).c_str());
      std::fprintf(stderr, " Q_DES(projected,Isaac): %s\n", S(qdes_projected_isaac_).c_str());
      std::fprintf(stderr,
          " PROJECTOR: active=%d rate=%d tracking=%d torque=%d infeasible=%d "
          "max_norm_debt=%.4f\n",
          last_qdes_projector_active_count_, last_qdes_projector_rate_count_,
          last_qdes_projector_tracking_count_, last_qdes_projector_torque_count_,
          last_qdes_projector_infeasible_count_,
          last_qdes_projector_max_normalized_error_);
    }
    std::fprintf(stderr, " Q_DES(SDK)[31]       : %s\n", S(q_sdk).c_str());
    std::fprintf(stderr, " KP(SDK)[31]          : %s\n", S(kp_sdk).c_str());
    std::fprintf(stderr, " KD(SDK)[31]          : %s\n", S(kd_sdk).c_str());
    if (state.q.size() == kNumJoints) {
      const Eigen::VectorXd e = (q_sdk - state.q).cwiseAbs();
      int wi = 0;
      e.maxCoeff(&wi);
      std::fprintf(stderr, " |q_des-q_meas| max=%.4f at %s (slot %d)\n", e[wi],
                   backend_joint_order()[wi].c_str(), wi);
    }
    std::fprintf(stderr,
        " NECK passive: slots[%d,%d] q=%.2f kp=%.1f kd=%.1f (model neck output dropped)\n",
        kHeadSlot0, kHeadSlot1, q_sdk[kHeadSlot0], kp_sdk[kHeadSlot0], kd_sdk[kHeadSlot0]);
    std::fprintf(stderr,
        " SAFETY: q_des %s on %d/31 joint(s), safe_interval_excess=%d/31 | "
        "sec(torso) IMU=%d -> torso ori %s\n",
        cfg_.gate3_qdes_audit_only ? "would hard-clamp" : "clamped",
        last_clamp_count_, last_safe_interval_violation_count_,
        (int)state.has_secondary_imu,
        state.has_secondary_imu ? "from IMU" : "IDENTITY-FALLBACK(!)");
    std::fprintf(stderr,
        "=================================================================\n\n");
  }

  PpOnnxPolicy onnx_;
  PpPolicyConfig cfg_;
  std::atomic<int> level_;
  std::atomic<double> swing_speed_;
  std::atomic<int> swing_dir_;  // +1 forehand / -1 backhand (scripted; live f/b toggle)
  std::atomic<int> pending_swing_dir_{0};  // queued mid-swing dir flip (0 = none); see set_swing_dir
  bool last_tts_at_windup_ = true;  // last tick's clock sat at the windup start (safe flip point)
  int yaw_align_defer_ticks_ = 0;   // ticks spent waiting for upright+still before yaw capture
  std::atomic<bool> legs_passive_{false};   // hold legs at nominal (dyn: --auto-leg-hold flips by level)
  std::atomic<bool> waist_passive_{false};  // hold waist at nominal (dyn: --auto-leg-hold flips by level)
  std::uint64_t arm_quiet_ticks_ = 0;       // --arm-hold-nominal sustained-quiet counter (driver thread only)
  bool arm_hold_armed_ = true;              // pre-swing arm hold armed (set at MOTION entry, cleared by the 1st swing)
  double leg_clamp_rad_ = 0.0;              // clamp policy-driven leg q_des to nominal ± band (0=off)
  double leg_smooth_alpha_ = 1.0;           // EMA low-pass on released leg q_des (1=off)
  Eigen::VectorXd leg_qdes_smooth_;         // EMA state for the leg q_des low-pass (seeded to nominal)
  std::atomic<std::uint64_t> swing_clock_origin_{0};  // tick offset; reset on each level->1 entry
  std::uint64_t rest_rearm_tick_ = 0;                 // driver thread only
  std::atomic<bool> rest_rearm_armed_{false};         // cleared by any external set_level()
  // yaw-align state (see PpPolicyConfig::yaw_align / rearm_yaw_align)
  std::atomic<bool> yaw_align_pending_{true};
  Vec4 yaw0_base_inv_ = Vec4(1.0, 0.0, 0.0, 0.0);     // driver thread only
  Vec4 yaw0_torso_inv_ = Vec4(1.0, 0.0, 0.0, 0.0);    // driver thread only
  int swing_level_prev_ = 0;                          // ComputeCommand (driver thread) only
  int swing_dir_prev_ = 1;                            // detect f<->b switch -> restart swing at windup
  ClipLayout clip_;
  // 177-D hitter_footwork + 110-D hitter_pure: per-clip station geometry.
  // station_xy = racket_target_xy - reach_offset_clip_[clip]. 177: reference base->racket
  // reach at the strike frame (ONNX metadata or refs fallback). 110: (fixed_plane_x,
  // y_band_center) from the baked hitter_pure sampling boxes (≈ the same numbers by
  // construction). Zero for 175/180 models (never read there).
  Vec2 reach_offset_clip_[2] = {Vec2::Zero(), Vec2::Zero()};
  // 110-D hitter_pure only: trained per-clip target bands (engage gate + streaming gate).
  // Defaults = the legacy shared gate; overwritten from ONNX metadata in the ctor.
  Vec2 hp_y_band_[2] = {Vec2(-0.65, -0.15), Vec2(-0.05, 0.45)};
  Vec2 hp_z_band_[2] = {Vec2(0.55, 1.40), Vec2(0.55, 1.40)};
  // Per-clip trained velocity boxes {x_lo,x_hi,y_lo,y_hi,z_lo,z_hi} from the ONNX
  // hitter_pure_vel_range_per_clip metadata; gate engage + streaming when set.
  std::array<double, 6> hp_vel_box_[2] = {{0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}};
  bool hp_vel_box_set_ = false;
  std::array<double, 6> hp_vel_core_box_[2] = {
      {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}};
  std::array<double, 6> hp_vel_planner_box_[2] = {
      {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}};
  bool hp_vel_components_set_ = false;
  std::array<double, 2> hp_station_y_step_range_ = {0.0, 0.0};
  std::array<double, 4> hp_base_target_range_ = {0.0, 0.0, 0.0, 0.0};
  bool rally_final_station_control_ = false;
  // RallyV8 "stay if reachable" (2026-07-12): V8 trains a MOSTLY-STATIONARY hitter — 50% of
  // wraps command the SAME station and targets are sampled from an ARM-REACH y band around
  // it, so the deploy inversion must keep the CURRENT station when the target is in-band and
  // otherwise make the MINIMUM lateral move (nearest band edge). The legacy unconditional
  // band-CENTER subtraction turns every serve into a step, erasing the trained behavior.
  // Set from the ONNX recipe (rally_v8) at ctor; CLI --no-stay-if-reachable is the A/B escape.
  bool stay_if_reachable_ = false;
  bool station_session_origin_set_ = false;
  Vec2 station_session_origin_w_ = Vec2::Zero();
  // 177-D hold-station anchor (driver thread only): the fixed WORLD station fed to the
  // base_target obs during level-0 holds (captured at hold entry; carried from the last
  // swing's station after a completed swing). Cleared on localization dropout and by
  // rearm_yaw_align() (mode re-entry — the robot may have been carried/moved).
  Vec2 hold_station_w_ = Vec2::Zero();
  bool hold_station_set_ = false;
  bool fixed_y_homing_active_ = false;
  std::uint64_t fixed_y_homing_start_tick_ = 0;
  double fixed_y_homing_peak_error_m_ = 0.0;
  std::uint64_t fixed_y_homing_activation_count_ = 0;
  std::uint64_t fixed_y_homing_completion_count_ = 0;
  // Rewritten V15: one station transition owns one finite gait episode.  Completion is
  // terminal for that shot; measured residual error never re-arms these counters.
  int finite_gait_planned_cycles_ = 0;
  int finite_gait_duration_steps_ = 0;
  int finite_gait_elapsed_steps_ = 0;
  double finite_gait_velocity_y_ = 0.0;
  double finite_gait_initial_delta_y_ = 0.0;
  // POST-GAIT SETTLE (2026-07-23): consecutive policy ticks with the finite gait finished.
  // Training always leaves >= 16 STAND control ticks between gait completion and swing
  // release; the release gate requires kFiniteGaitSettleTicks of them (0.3 s at 50 Hz).
  static constexpr int kFiniteGaitSettleTicks = 15;
  int finite_gait_settle_ticks_ = 0;
  std::array<int, 31> isaac_to_sdk_{};
  Eigen::VectorXd nominal_q_sdk_;
  Eigen::VectorXd official_kp_sdk_;
  Eigen::VectorXd official_kd_sdk_;
  Eigen::VectorXd last_action_;
  // Armed at mode rearm / static handoff (and at construction: the very first policy tick is
  // a training reset too); consumed by the first policy tick's measured-posture seed.
  bool last_action_seed_pending_ = true;
  bool qdes_projector_initialized_ = false;
  Eigen::VectorXd qdes_nominal_isaac_ = Eigen::VectorXd::Zero(kNumJoints);
  Eigen::VectorXd qdes_projected_isaac_ = Eigen::VectorXd::Zero(kNumJoints);
  int last_qdes_projector_active_count_ = 0;
  int last_qdes_projector_rate_count_ = 0;
  int last_qdes_projector_tracking_count_ = 0;
  int last_qdes_projector_torque_count_ = 0;
  int last_qdes_projector_infeasible_count_ = 0;
  double last_qdes_projector_max_normalized_error_ = 0.0;
  double last_qdes_feasible_action_utilization_max_ = 0.0;
  double last_qdes_feasible_interval_width_min_ = 1.0;
  double last_qdes_feasible_rate_utilization_max_ = 0.0;
  int last_qdes_feasible_rate_bound_count_ = 0;
  int last_qdes_feasible_tracking_bound_count_ = 0;
  int last_qdes_feasible_torque_bound_count_ = 0;
  std::uint64_t qdes_projector_ticks_ = 0;
  std::array<std::uint64_t, kNumJoints> qdes_projector_joint_count_{};
  int last_time_step_ = -1;
  Vec3 last_proj_grav_ = Vec3(0.0, 0.0, -1.0);
  Vec3 last_base_pos_ = Vec3(0.0, 0.0, 0.95);
  Vec4 last_base_quat_w_ = Vec4(1.0, 0.0, 0.0, 0.0);  // yaw-aligned; planner gate uses last tick's
  Vec3 last_target_pos_w_ = Vec3::Zero();
  Vec3 last_target_vel_w_ = Vec3::Zero();
  bool last_racket_fk_valid_ = false;
  std::uint64_t last_racket_fk_tick_ = 0;
  Vec3 last_racket_pos_w_ = Vec3::Zero();
  Vec3 last_racket_vel_w_ = Vec3::Zero();
  Vec3 last_racket_normal_w_ = Vec3::Zero();
  bool dbg_done_ = false;
  bool sec_imu_warned_ = false;  // one-shot warn when torso IMU is absent
  bool clamp_warned_ = false;    // one-shot warn when q_des hits a joint limit
  bool safe_interval_warned_ = false;
  int last_safe_interval_violation_count_ = 0;
  std::uint64_t safe_interval_audit_ticks_ = 0;
  std::array<std::uint64_t, kNumJoints> safe_interval_count_{};
  std::array<double, kNumJoints> safe_interval_max_excess_{};
  std::array<bool, kNumJoints> actual_q_hard_audit_active_{};
  int last_clamp_count_ = 0;     // # joints clamped on the last tick
  std::uint64_t clamp_ticks_ = 0;                        // ticks the clamp ran
  std::array<std::uint64_t, kNumJoints> clamp_count_{};  // per-slot clamp hit count
  std::array<double, kNumJoints> clamp_max_viol_{};      // per-slot max out-of-range (rad)
  Eigen::VectorXd last_clamp_viol_ = Eigen::VectorXd::Zero(kNumJoints);

  // --- localization / oracle (sim-only) + obs-debug state ---
  std::shared_ptr<PpOraclePose> oracle_;       // only used when loc_mode == kOracle
  bool oracle_fresh_ = false;
  std::uint64_t oracle_warn_tick_ = 0;  // repeat the stale-oracle warning every ~2 s (100 ticks)
  double oracle_age_s_ = -1.0;
  std::uint64_t sync_miss_ = 0;

  // --- LIVE PLANNER inputs + engage state (Path B; driver-thread only unless noted) ---
  std::shared_ptr<PpRacketTargetInput> racket_in_;  // written by AimRT subscriber thread
  std::shared_ptr<PpBasePoseInput> base_in_;        // written by AimRT subscriber thread
  bool base_fresh_ = false;             // a fresh external-base sample was used this tick
  bool have_last_external_base_ = false;
  Vec3 last_external_base_pos_w_ = Vec3(0.0, 0.0, 0.95);
  Vec4 last_external_base_quat_w_ = Vec4(1.0, 0.0, 0.0, 0.0);
  std::uint64_t base_warn_tick_ = 0;    // repeat the stale-mocap warning every ~2 s
  std::uint64_t gate_warn_tick_ = 0;    // throttle the target-gate rejection detail print
  bool planner_engaged_ = false;        // a planner swing is active (frozen target in flight)
  bool planner_pending_station_active_ = false;  // Final: level-0 move/settle before clock release
  Vec2 planner_pending_station_w_ = Vec2::Zero();
  Vec2 planner_pending_origin_station_w_ = Vec2::Zero();
  int planner_pending_clip_ = 0;
  double planner_pending_strike_time_ = 0.0;
  Vec3 planner_pending_pos_w_ = Vec3::Zero();
  Vec3 planner_pending_vel_w_ = Vec3::Zero();
  std::uint64_t planner_pending_supported_target_tick_ = 0;
  bool planner_station_ready_timer_active_ = false;
  std::uint64_t planner_station_ready_since_tick_ = 0;
  Vec2 planner_station_dwell_anchor_w_ = Vec2::Zero();
  int planner_station_dwell_anchor_clip_ = -1;
  std::uint64_t planner_pending_start_tick_ = 0;
  bool planner_station_ready_reported_ = false;
  // False only during a genuine cold boot. Once a pending station releases ACTIVE policy
  // tracking, an abort must prove localization-backed base quiescence before static handoff.
  bool prefirst_active_station_tracking_started_ = false;
  bool base_motion_initialized_ = false;
  bool base_speed_xy_valid_ = false;
  bool base_velocity_xy_valid_ = false;
  Vec2 base_motion_prev_xy_ = Vec2::Zero();
  std::uint64_t base_motion_prev_tick_ = 0;
  std::uint64_t base_motion_prev_seq_ = 0;
  std::uint64_t base_motion_last_new_tick_ = 0;
  double base_speed_xy_est_ = 0.0;
  Vec2 base_velocity_xy_est_ = Vec2::Zero();
  double planner_current_strike_time_ = 0.0;
  double planner_current_tts_ = 0.0;
  double planner_engage_raw_tts_ = 0.0;
  double planner_engage_clock_tts0_ = 0.0;
  double planner_engage_requested_phase_s_ = 0.0;
  double planner_engage_actual_phase_s_ = 0.0;
  double planner_engage_expected_strike_lateness_s_ = 0.0;
  bool planner_late_phase_clamped_ = false;
  double planner_valid_age_s_ = -1.0;
  std::string planner_lifecycle_event_ = "init";
  std::string planner_lifecycle_reason_ = "init";
  std::uint64_t planner_lifecycle_seq_ = 0;
  std::uint64_t planner_shot_seq_ = 0;
  std::uint64_t planner_current_msg_seq_ = 0;
  std::uint64_t planner_current_flight_id_ = 0;
  std::uint64_t planner_current_revision_id_ = 0;
  int planner_current_stable_revision_count_ = 0;
  std::uint64_t planner_frozen_command_seq_ = 0;
  std::uint64_t planner_frozen_flight_id_ = 0;
  std::uint64_t planner_frozen_revision_id_ = 0;
  double planner_frozen_strike_time_ = 0.0;
  double planner_frozen_raw_tts_ = 0.0;
  bool planner_have_hold_ = false;      // at least one swing engaged (diagnostic)
  bool planner_have_consumed_shot_ = false;
  double planner_consumed_strike_time_ = 0.0;
  double planner_tts0_ = 0.0;           // engage-time tts, clamped to the clip windup length;
                                        // seeds the swing clock so the strike meets the ball
  Vec3 planner_frozen_pos_w_ = Vec3::Zero();
  // Hold/pre-engage target velocity: initialised in the ctor to the forehand box-center
  // vel (a ZERO vel target is outside every trained target-vel box = an obs state
  // training never saw); overwritten by each engage's frozen velocity.
  Vec3 planner_frozen_vel_w_ = Vec3::Zero();
  double planner_frozen_sign_ = 1.0;
  // base-rel target at engage (hold anchor); defaults = a centered, racket-reachable ready
  // stance so the pre-first-engage hold is safe even before any command arrives.
  Vec3 planner_hold_pos_b_engage_ = Vec3(0.40, 0.0, 0.0);
  double planner_hold_z_w_ = 0.90;
  // post-swing recovery clock + static-stand blend state (driver thread only)
  std::uint64_t planner_hold_start_tick_ = 0;
  // MOTION/SHADOW session start (engage settle clock); pending set by rearm_yaw_align.
  std::uint64_t planner_entry_tick_ = 0;
  std::atomic<bool> planner_entry_pending_{true};
  bool planner_static_active_ = false;
  bool serve_static_handoff_pending_ = false;
  std::uint64_t planner_static_start_tick_ = 0;
  // static-handoff base-settle dwell (driver thread only; see BASE-SETTLE guard)
  std::uint64_t static_settle_ticks_ = 0;
  // throttle for the deferred pre-first-engage static warning (~every 2 s)
  std::uint64_t prefirst_warn_tick_ = 0;
  Eigen::VectorXd planner_static_q0_;
  bool planner_policy_takeover_active_ = false;
  std::uint64_t planner_policy_takeover_start_tick_ = 0;
  Eigen::VectorXd planner_policy_takeover_q0_;
  mutable std::mutex planner_mu_;
  std::string planner_status_ = "init";
  mutable std::mutex obs_mu_;
  Eigen::VectorXd last_obs_;

  // --- live diagnostics (written in ComputeCommand, read by status thread) ---
  mutable std::mutex diag_mu_;
  Eigen::VectorXd last_q_des_, last_q_meas_, last_qd_meas_;
  Eigen::VectorXd des_lo_, des_hi_, meas_lo_, meas_hi_, err_peak_, qd_peak_;
  bool ranges_init_ = false;
};

}  // namespace a3_pingpong
