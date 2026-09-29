// 180/175/177/110/112/127/113/118-D observation builders. 180 ported from
// hope_ws/.../hope_wbc_runner/obs_builder.py (build_obs). Layout (total 180):
//   command(62) = ref joint_pos[31] + ref joint_vel[31]
//   motion_anchor_pos_b(3), motion_anchor_ori_b(6)
//   base_ang_vel(3), joint_pos_rel(31), joint_vel(31), last_action(31),
//   projected_gravity(3), base_target_pos_b(2), racket_target_pos_b(3),
//   racket_target_vel_w(3), time_to_strike(1), swing_type(1)
// 175 = deploy_parity (drops anchor_pos + base_target, racket target FK-relative);
// 177 = hitter_footwork (the 175 layout + base_target_pos_b(2) re-inserted after
// projected_gravity — the HITTER-style relative-Δ station footwork channel);
// 110 = hitter_pure (2026-07-07, HITTER Table-I exact: NO reference stream, NO
// swing_type, WORLD-frame target vectors + explicit base forward vector e_base,x).
// 112 = Schema27: the complete Schema26 110-D prefix plus two Planner-owned optional-reach
// permission labels. Disabled artifacts supply exact zeros; enabled artifacts carry screened
// target-tuple labels. No plant state is used to infer them.
// 127 = Schema28 append-only shadow-feedback ABI: the exact 112-D composed-action
// prefix plus the previous frozen-core actions for policy-owned waist/legs.
// 113 = V15 legacy position-mocap extension (base velocity xy + localization age).
// 118 = V15 HUGWBC locomotion extension (113 prefix + lateral velocity command,
// left/right gait clocks, STAND/STEP/swing mode and train-only intervention indicator).
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "a3_pingpong/pp_frame_math.hpp"
#include "a3_pingpong/pp_racket_fk.hpp"

namespace a3_pingpong {

constexpr int kObsDim = 180;
constexpr int kObsDim175 = 175;
constexpr int kObsDim177 = 177;
constexpr int kObsDim110 = 110;
constexpr int kObsDim112 = 112;
constexpr int kObsDim127 = 127;
constexpr int kObsDim113 = 113;
constexpr int kObsDim118 = 118;
constexpr int kNumJoints = 31;
constexpr int kCoreActionShadowDim = 15;
inline constexpr std::array<int, kCoreActionShadowDim>
    kCoreActionShadowOwnedIndices = {
        0, 1, 2, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30};
constexpr int kAnchorTrackedIdx = 7;  // torso_Link in the 14-body tracked order
constexpr int kHitterPureLastActionObsOffset = 65;
// These are indices in the resolved Isaac/action order carried by ONNX joint_names, not backend
// SDK slots.  The A3 backend keeps the head at SDK slots 3/4, but the deployed model action order
// is interleaved and places head_yaw/head_pitch at 11/16.  The 110-D observation prepends 65 values
// before its 31-D action-feedback block, hence the corresponding global observation slots 76/81.
constexpr int kHitterPureHeadYawActionIndex = 11;
constexpr int kHitterPureHeadPitchActionIndex = 16;
constexpr int kHitterPureBaseVelocityXObsIndex =
    kHitterPureLastActionObsOffset + kHitterPureHeadYawActionIndex;
constexpr int kHitterPureBaseVelocityYObsIndex =
    kHitterPureLastActionObsOffset + kHitterPureHeadPitchActionIndex;
static_assert(kHitterPureBaseVelocityXObsIndex == 76);
static_assert(kHitterPureBaseVelocityYObsIndex == 81);

inline bool hitter_pure_v4_velocity_slot_joint_order_matches(
    const std::vector<std::string>& joint_names) {
  return joint_names.size() == kNumJoints &&
      joint_names[kHitterPureHeadYawActionIndex] == "head_yaw_joint" &&
      joint_names[kHitterPureHeadPitchActionIndex] == "head_pitch_joint";
}

// Reference motion at the current time_step (the ONNX side-outputs). Only the
// anchor body (index 7) is needed from the 14-body pose arrays.
struct PpRefs {
  Eigen::VectorXd joint_pos;  // (31)
  Eigen::VectorXd joint_vel;  // (31)
  Vec3 anchor_pos_w;          // body_pos_w[7]  (torso anchor)
  Vec4 anchor_quat_w;         // body_quat_w[7]  (w,x,y,z)
  Vec3 ref_pelvis_pos_w;      // body_pos_w[0]  (reference pelvis/root; used only by
                              // the perfect-tracking localization mode for the
                              // racket/base-target obs terms - NOT part of the obs itself)
};

// Robot state (all world poses in the policy frame). torso/base world pose is
// the sim2real localisation gap on the real robot (nominal there).
struct PpRobotState {
  Vec3 base_pos_w;
  Vec4 base_quat_w;
  Vec3 torso_pos_w;
  Vec4 torso_quat_w;
  Vec3 base_ang_vel_b;        // pelvis gyro, pelvis BODY frame
  Eigen::VectorXd q;          // (31) Isaac order
  Eigen::VectorXd qd;         // (31)
  Vec2 base_velocity_xy_w = Vec2::Zero();  // position-differenced mocap velocity
  double localization_age = 1.0;           // receipt age / stale threshold, clamped [0,1]
};

// Planner target, already in the policy world frame.
struct PpRacketTarget {
  Vec3 pos_w;
  Vec3 vel_w;
  double swing_sign;          // +1 forehand / -1 backhand
  double time_to_strike;      // seconds
  Vec2 base_target_xy = Vec2::Zero();
  double desired_lateral_velocity = 0.0;
  Vec2 gait_clock = Vec2::Zero();
  double locomotion_mode = -1.0;       // +1 STEP, 0 STAND, -1 swing/recovery
  double upper_intervention = 0.0;     // training-only; deploy is always zero
  double reach_level = 0.0;            // Planner permission: 0 fixed, 1 unload, 2 microstep
  double swing_foot_sign = 0.0;        // Planner permission: -1 right, 0 none, +1 left
  double signed_external_reach = 0.0;  // Schema34: reach_level * swing_foot_sign
  double signed_replant_need = 0.0;    // Schema34: signed continuous FK-to-HOME debt
  Eigen::Vector4d footstep_task_errors = Eigen::Vector4d::Zero();  // Schema34v3 origin XY, destination XY
};

// Assemble the 180-D observation (double precision; cast to float at the ONNX
// boundary). last_action and default_q are length-31.
// use_base_yaw_for_targets: rotate the (base/racket) world targets into the
// robot's yaw-heading frame using the REAL base-IMU yaw. TRUE reproduces the
// training/sim2sim transform exactly (default, so the parity golden + sim are
// untouched). On HARDWARE the pelvis IMU yaw is NOT world-referenced (it drifts
// boot-to-boot, e.g. 130-165 deg, and disagrees with the torso IMU), so feeding
// it here rotates the scripted target away from where it should be. Pass FALSE
// there: the scripted target is already expressed in the robot's nominal heading
// (+x), so we use identity yaw. This is a no-op whenever base yaw ~ 0 (sim), and
// it does NOT affect projected_gravity (yaw-invariant) or the anchor terms.
inline Eigen::VectorXd build_obs_180(const PpRefs& refs, const PpRobotState& state,
                                     const PpRacketTarget& target,
                                     const Eigen::VectorXd& last_action,
                                     const Eigen::VectorXd& default_q,
                                     bool use_base_yaw_for_targets = true) {
  Eigen::VectorXd obs(kObsDim);
  int o = 0;

  // 1. command (62) = ref joint_pos[31] ++ ref joint_vel[31]
  obs.segment(o, kNumJoints) = refs.joint_pos; o += kNumJoints;
  obs.segment(o, kNumJoints) = refs.joint_vel; o += kNumJoints;

  // 2/3. ref anchor (torso) expressed in robot anchor (torso) frame
  Vec3 pos_b;
  Vec4 ori_q;
  subtract_frame_transforms(state.torso_pos_w, state.torso_quat_w,
                            refs.anchor_pos_w, refs.anchor_quat_w, pos_b, ori_q);
  const Mat3 R = mat_from_quat(ori_q);
  // 6D rot = first two COLUMNS of R, row-major: R00,R01,R10,R11,R20,R21
  obs.segment<3>(o) = pos_b; o += 3;
  obs[o++] = R(0, 0); obs[o++] = R(0, 1);
  obs[o++] = R(1, 0); obs[o++] = R(1, 1);
  obs[o++] = R(2, 0); obs[o++] = R(2, 1);

  // 4. base angular velocity (3), body frame
  obs.segment<3>(o) = state.base_ang_vel_b; o += 3;

  // 5/6. joint pos rel / joint vel (31 each)
  obs.segment(o, kNumJoints) = state.q - default_q; o += kNumJoints;
  obs.segment(o, kNumJoints) = state.qd; o += kNumJoints;

  // 7. last action (31)
  obs.segment(o, kNumJoints) = last_action; o += kNumJoints;

  // 8. projected gravity (3), body frame
  obs.segment<3>(o) = projected_gravity_body(state.base_quat_w); o += 3;

  // 9. base target XY in yaw-heading base frame (2)
  // yaw-heading from the real IMU (training) OR identity (hardware: the absolute
  // IMU yaw is unreferenced; the scripted target lives in the robot's +x frame).
  const Vec4 yq = use_base_yaw_for_targets ? yaw_quat(state.base_quat_w)
                                           : Vec4(1.0, 0.0, 0.0, 0.0);
  Vec3 delta_base(target.base_target_xy[0] - state.base_pos_w[0],
                  target.base_target_xy[1] - state.base_pos_w[1], 0.0);
  const Vec3 base_tgt_b = quat_rotate_inverse(yq, delta_base);
  obs[o++] = base_tgt_b[0];
  obs[o++] = base_tgt_b[1];

  // 10. racket target pos in yaw-heading base frame (3)
  obs.segment<3>(o) = quat_rotate_inverse(yq, target.pos_w - state.base_pos_w); o += 3;

  // 11. racket target velocity, world (3)
  obs.segment<3>(o) = target.vel_w; o += 3;

  // 12/13. time_to_strike (1), swing_type (1)
  obs[o++] = target.time_to_strike;
  obs[o++] = target.swing_sign;

  return obs;  // o == 180
}

// Assemble the 175-D "deploy_parity" observation. Layout (total 175):
//   command(62), motion_anchor_ori_b(6), base_ang_vel(3), joint_pos_rel(31),
//   joint_vel(31), last_action(31), projected_gravity(3), racket_target_pos_b(3),
//   racket_target_vel_w(3), time_to_strike(1), swing_type(1)
//
// Differences from build_obs_180 (everything else is byte-identical):
//   1. motion_anchor_pos_b (the 3-vec before the 6D rot) is REMOVED.
//   2. base_target_pos_b (the 2-vec block) is REMOVED.
//   3. racket_target_pos_b uses the CURRENT racket FK world position as the
//      reference point instead of the base position:
//        180: quat_rotate_inverse(yq, target.pos_w - state.base_pos_w)
//        175: quat_rotate_inverse(yq, target.pos_w - racket_pos_w)
//      where racket_pos_w = base_pos_w + R(base_quat_w) * racket_pos_pelvis(q).
//      On the real robot this cancels the (unobservable) world base position,
//      matching the deploy-honest training recipe.
//
// Signature matches build_obs_180 exactly. See build_obs_180 for the meaning of
// use_base_yaw_for_targets.
inline Eigen::VectorXd build_obs_175(const PpRefs& refs, const PpRobotState& state,
                                     const PpRacketTarget& target,
                                     const Eigen::VectorXd& last_action,
                                     const Eigen::VectorXd& default_q,
                                     bool use_base_yaw_for_targets = true) {
  Eigen::VectorXd obs(kObsDim175);
  int o = 0;

  // 1. command (62) = ref joint_pos[31] ++ ref joint_vel[31]
  obs.segment(o, kNumJoints) = refs.joint_pos; o += kNumJoints;
  obs.segment(o, kNumJoints) = refs.joint_vel; o += kNumJoints;

  // 2. motion_anchor_ori_b (6) — ref anchor (torso) orientation in robot anchor
  //    (torso) frame. NOTE: motion_anchor_pos_b (3) is intentionally NOT emitted.
  Vec3 pos_b;  // computed but dropped from the 175 layout
  Vec4 ori_q;
  subtract_frame_transforms(state.torso_pos_w, state.torso_quat_w,
                            refs.anchor_pos_w, refs.anchor_quat_w, pos_b, ori_q);
  const Mat3 R = mat_from_quat(ori_q);
  // 6D rot = first two COLUMNS of R, row-major: R00,R01,R10,R11,R20,R21
  obs[o++] = R(0, 0); obs[o++] = R(0, 1);
  obs[o++] = R(1, 0); obs[o++] = R(1, 1);
  obs[o++] = R(2, 0); obs[o++] = R(2, 1);

  // 3. base angular velocity (3), body frame
  obs.segment<3>(o) = state.base_ang_vel_b; o += 3;

  // 4/5. joint pos rel / joint vel (31 each)
  obs.segment(o, kNumJoints) = state.q - default_q; o += kNumJoints;
  obs.segment(o, kNumJoints) = state.qd; o += kNumJoints;

  // 6. last action (31)
  obs.segment(o, kNumJoints) = last_action; o += kNumJoints;

  // 7. projected gravity (3), body frame
  obs.segment<3>(o) = projected_gravity_body(state.base_quat_w); o += 3;

  // 8. racket target pos, relative to the CURRENT racket FK position, in the
  //    yaw-heading base frame (3). base_target_pos_b (2) is NOT emitted.
  const Vec4 yq = use_base_yaw_for_targets ? yaw_quat(state.base_quat_w)
                                           : Vec4(1.0, 0.0, 0.0, 0.0);
  const Vec3 racket_pos_w =
      state.base_pos_w + mat_from_quat(state.base_quat_w) * racket_pos_pelvis(state.q);
  obs.segment<3>(o) = quat_rotate_inverse(yq, target.pos_w - racket_pos_w); o += 3;

  // 9. racket target velocity, world (3)
  obs.segment<3>(o) = target.vel_w; o += 3;

  // 10/11. time_to_strike (1), swing_type (1)
  obs[o++] = target.time_to_strike;
  obs[o++] = target.swing_sign;

  return obs;  // o == 175
}

// Assemble the 177-D "hitter_footwork" observation. Layout (total 177):
//   command(62), motion_anchor_ori_b(6), base_ang_vel(3), joint_pos_rel(31),
//   joint_vel(31), last_action(31), projected_gravity(3), base_target_pos_b(2),
//   racket_target_pos_b(3), racket_target_vel_w(3), time_to_strike(1), swing_type(1)
//
// = build_obs_175 PLUS the base_target_pos_b(2) block from build_obs_180 re-inserted
// between projected_gravity and racket_target_pos_b. Training semantics
// (hope_commands.py base_target_pos_b): Δxy from the CURRENT base to the commanded
// station, rotated into the yaw-heading base frame. Callers must set
// target.base_target_xy to the WORLD station — during holds a FIXED world anchor,
// not the live base (the Δ to the anchor is the policy's balance signal; feeding
// Δ=0 through holds lets the base free-wander — 2026-07-06 finding). Δ=0 (station
// := current base xy) is ONLY the no-real-localization dropout fallback.
// Everything else is byte-identical to build_obs_175.
inline Eigen::VectorXd build_obs_177(const PpRefs& refs, const PpRobotState& state,
                                     const PpRacketTarget& target,
                                     const Eigen::VectorXd& last_action,
                                     const Eigen::VectorXd& default_q,
                                     bool use_base_yaw_for_targets = true) {
  Eigen::VectorXd obs(kObsDim177);
  int o = 0;

  // 1. command (62) = ref joint_pos[31] ++ ref joint_vel[31]
  obs.segment(o, kNumJoints) = refs.joint_pos; o += kNumJoints;
  obs.segment(o, kNumJoints) = refs.joint_vel; o += kNumJoints;

  // 2. motion_anchor_ori_b (6) — ref anchor (torso) orientation in robot anchor
  //    (torso) frame. motion_anchor_pos_b (3) is intentionally NOT emitted.
  Vec3 pos_b;  // computed but dropped from the 177 layout
  Vec4 ori_q;
  subtract_frame_transforms(state.torso_pos_w, state.torso_quat_w,
                            refs.anchor_pos_w, refs.anchor_quat_w, pos_b, ori_q);
  const Mat3 R = mat_from_quat(ori_q);
  // 6D rot = first two COLUMNS of R, row-major: R00,R01,R10,R11,R20,R21
  obs[o++] = R(0, 0); obs[o++] = R(0, 1);
  obs[o++] = R(1, 0); obs[o++] = R(1, 1);
  obs[o++] = R(2, 0); obs[o++] = R(2, 1);

  // 3. base angular velocity (3), body frame
  obs.segment<3>(o) = state.base_ang_vel_b; o += 3;

  // 4/5. joint pos rel / joint vel (31 each)
  obs.segment(o, kNumJoints) = state.q - default_q; o += kNumJoints;
  obs.segment(o, kNumJoints) = state.qd; o += kNumJoints;

  // 6. last action (31)
  obs.segment(o, kNumJoints) = last_action; o += kNumJoints;

  // 7. projected gravity (3), body frame
  obs.segment<3>(o) = projected_gravity_body(state.base_quat_w); o += 3;

  // 8. base target XY in yaw-heading base frame (2) — the footwork station channel.
  const Vec4 yq = use_base_yaw_for_targets ? yaw_quat(state.base_quat_w)
                                           : Vec4(1.0, 0.0, 0.0, 0.0);
  Vec3 delta_base(target.base_target_xy[0] - state.base_pos_w[0],
                  target.base_target_xy[1] - state.base_pos_w[1], 0.0);
  const Vec3 base_tgt_b = quat_rotate_inverse(yq, delta_base);
  obs[o++] = base_tgt_b[0];
  obs[o++] = base_tgt_b[1];

  // 9. racket target pos, relative to the CURRENT racket FK position, in the
  //    yaw-heading base frame (3) — same deploy-honest reframe as build_obs_175.
  const Vec3 racket_pos_w =
      state.base_pos_w + mat_from_quat(state.base_quat_w) * racket_pos_pelvis(state.q);
  obs.segment<3>(o) = quat_rotate_inverse(yq, target.pos_w - racket_pos_w); o += 3;

  // 10. racket target velocity, world (3)
  obs.segment<3>(o) = target.vel_w; o += 3;

  // 11/12. time_to_strike (1), swing_type (1)
  obs[o++] = target.time_to_strike;
  obs[o++] = target.swing_sign;

  return obs;  // o == 177
}

// Assemble the 110-D "hitter_pure" observation (HITTER arXiv:2508.21043 Table I exact,
// sized for the A3's 31 joints). Layout (training contract `hitter_pure`, verified by
// scripts/verify_hitter_pure.py):
//   base_ang_vel(3), joint_pos_rel(31), joint_vel(31), last_action(31),
//   projected_gravity(3), base_forward_xy(2), base_target_delta_xy(2),
//   racket_target_rel_base(3), racket_target_vel_w(3), time_to_strike(1)
//
// Differences from every other layout here:
//   * NO reference stream (command/anchor blocks) and NO swing_type — the paper keeps the
//     reference CRITIC-only and infers the swing side outside the policy (§V-B-3). PpRefs
//     is therefore NOT a parameter; the reference clock is still used by the CALLER for
//     time_to_strike and swing-side selection, it just never enters the observation.
//   * Target vectors are WORLD-frame differences (NO yaw-heading rotation):
//       base_target_delta_xy    = station_xy − base_pos_xy          (world)
//       racket_target_rel_base  = target_pos_w − base_pos_w         (world, rel BASE not FK)
//     plus the explicit base forward vector e_base,x (world xy of the base +x axis) that
//     lets the policy do the rotation itself — this is what carries the yaw-facing signal
//     once the reference-orientation term is gone. On hardware the quats are the
//     yaw-align-at-engage IMU attitudes and positions are mocap/table frame; the operator
//     faces the robot toward +x at engage (same assumption as every other layout).
//   * Callers must set target.base_target_xy to the WORLD station. Repair-v3 preserves the last
//     verified station while the actor-visible mocap pose is held during an outage; cold start
//     uses station := current base xy. Legacy callers may retain their historical delta-zero path.
inline Eigen::VectorXd build_obs_110(const PpRobotState& state,
                                     const PpRacketTarget& target,
                                     const Eigen::VectorXd& last_action,
                                     const Eigen::VectorXd& default_q) {
  Eigen::VectorXd obs(kObsDim110);
  int o = 0;

  // 1. base angular velocity (3), body frame
  obs.segment<3>(o) = state.base_ang_vel_b; o += 3;

  // 2/3. joint pos rel / joint vel (31 each)
  obs.segment(o, kNumJoints) = state.q - default_q; o += kNumJoints;
  obs.segment(o, kNumJoints) = state.qd; o += kNumJoints;

  // 4. last action (31)
  obs.segment(o, kNumJoints) = last_action; o += kNumJoints;

  // 5. projected gravity (3), body frame
  obs.segment<3>(o) = projected_gravity_body(state.base_quat_w); o += 3;

  // 6. base forward vector e_base,x (2): world xy of the base +x axis, renormalized.
  {
    const Vec3 fwd = quat_rotate(state.base_quat_w, Vec3(1.0, 0.0, 0.0));
    const double n = std::max(std::hypot(fwd[0], fwd[1]), 1e-6);
    obs[o++] = fwd[0] / n;
    obs[o++] = fwd[1] / n;
  }

  // 7. base target delta (2): station − base, WORLD xy (no rotation).
  obs[o++] = target.base_target_xy[0] - state.base_pos_w[0];
  obs[o++] = target.base_target_xy[1] - state.base_pos_w[1];

  // 8. racket target relative to the BASE (3), WORLD frame (no rotation, not FK-relative).
  obs.segment<3>(o) = target.pos_w - state.base_pos_w; o += 3;

  // 9. racket target velocity, world (3)
  obs.segment<3>(o) = target.vel_w; o += 3;

  // 10. time_to_strike (1)
  obs[o++] = target.time_to_strike;

  return obs;  // o == 110
}

// Continuous-rally-v4 keeps the exact 110-D layout and 31-D action ABI, but repurposes the two
// passive-head action-feedback slots (which are exact zeros in all earlier recipes) as filtered
// world-frame mocap base velocity. Keep this as a distinct builder: changing build_obs_110 would
// silently alter every v1-v3 and legacy artifact sharing the same tensor shape.
inline Eigen::VectorXd build_obs_110_headslots_vxy(
    const PpRobotState& state,
    const PpRacketTarget& target,
    const Eigen::VectorXd& last_action,
    const Eigen::VectorXd& default_q) {
  Eigen::VectorXd obs = build_obs_110(state, target, last_action, default_q);
  obs[kHitterPureBaseVelocityXObsIndex] = state.base_velocity_xy_w[0];
  obs[kHitterPureBaseVelocityYObsIndex] = state.base_velocity_xy_w[1];
  return obs;
}

// Schema27 is append-only: every value in [0,110) is byte-identical to Schema26. The two new
// values are carried by the training-screened Planner target tuple. This builder never examines
// foot contact, load, height, achieved pose, or any future/privileged plant state. Transport
// wiring is not trajectory/deployment qualification; PpRacketTarget defaults keep both columns
// exact zero whenever optional reach is disabled or no active flight owns them.
inline Eigen::VectorXd build_obs_112_headslots_vxy_reach(
    const PpRobotState& state,
    const PpRacketTarget& target,
    const Eigen::VectorXd& last_action,
    const Eigen::VectorXd& default_q) {
  Eigen::VectorXd obs(kObsDim112);
  obs.head(kObsDim110) =
      build_obs_110_headslots_vxy(state, target, last_action, default_q);
  obs[kObsDim110] = target.reach_level;
  obs[kObsDim110 + 1] = target.swing_foot_sign;
  return obs;
}

// Schema34 preserves the 110-D prefix but reassigns the two appended columns. Column 110 is the
// signed external Planner request (0, +/-1, +/-2); column 111 is an independent signed,
// continuous ankle-roll-link-to-HOME recovery need reconstructed from actor-visible state.
inline Eigen::VectorXd build_obs_112_headslots_vxy_signed_support(
    const PpRobotState& state,
    const PpRacketTarget& target,
    const Eigen::VectorXd& last_action,
    const Eigen::VectorXd& default_q) {
  Eigen::VectorXd obs(kObsDim112);
  obs.head(kObsDim110) =
      build_obs_110_headslots_vxy(state, target, last_action, default_q);
  obs[kObsDim110] = target.signed_external_reach;
  obs[kObsDim110 + 1] = target.signed_replant_need;
  return obs;
}

inline Eigen::VectorXd build_obs_112_headslots_step_task(
    const PpRobotState& state, const PpRacketTarget& target,
    const Eigen::VectorXd& last_action, const Eigen::VectorXd& default_q) {
  Eigen::VectorXd obs = build_obs_112_headslots_vxy_signed_support(state, target, last_action, default_q);
  obs[14] = target.footstep_task_errors[0]; obs[19] = target.footstep_task_errors[1];
  obs[45] = target.footstep_task_errors[2]; obs[50] = target.footstep_task_errors[3];
  return obs;
}

// Schema28 preserves [0,112) byte-for-byte, including the true previous
// composed policy action. The appended 15-D auxiliary ONNX output is fed back
// only to the frozen-core shadow columns on the next tick; it is not q_des,
// does not replace any body command, and contains no measured/privileged state.
inline Eigen::VectorXd build_obs_127_headslots_vxy_reach_core_shadow(
    const PpRobotState& state,
    const PpRacketTarget& target,
    const Eigen::VectorXd& last_composed_action,
    const Eigen::VectorXd& default_q,
    const Eigen::VectorXd& core_action_shadow_owned) {
  if (core_action_shadow_owned.size() != kCoreActionShadowDim) {
    throw std::invalid_argument(
        "Schema28 core action shadow feedback must contain exactly 15 values");
  }
  Eigen::VectorXd obs(kObsDim127);
  obs.head(kObsDim112) = build_obs_112_headslots_vxy_reach(
      state, target, last_composed_action, default_q);
  obs.tail(kCoreActionShadowDim) = core_action_shadow_owned;
  return obs;
}

// V15 preserves every 110-D field and ordering, then appends the two quantities needed to
// distinguish a moving/stale localized base from a stationary/fresh one.  Both are obtainable
// from the real position-only mocap stream: velocity is position-differenced and low-pass filtered;
// age is measured from local receipt time.
inline Eigen::VectorXd build_obs_113(const PpRobotState& state,
                                     const PpRacketTarget& target,
                                     const Eigen::VectorXd& executed_qdes_feedback,
                                     const Eigen::VectorXd& default_q) {
  Eigen::VectorXd obs(kObsDim113);
  obs.head(kObsDim110) =
      build_obs_110(state, target, executed_qdes_feedback, default_q);
  obs.segment<2>(kObsDim110) = state.base_velocity_xy_w;
  obs[kObsDim110 + 2] = std::clamp(state.localization_age, 0.0, 1.0);
  return obs;
}

// Rewritten V15 keeps the deploy-proven 113-D position-mocap prefix byte-identical and appends
// the HUGWBC command state.  Every appended quantity is generated by the same finite gait
// scheduler in training and deployment; none requires an additional real-robot sensor.
inline Eigen::VectorXd build_obs_118(const PpRobotState& state,
                                     const PpRacketTarget& target,
                                     const Eigen::VectorXd& executed_qdes_feedback,
                                     const Eigen::VectorXd& default_q) {
  Eigen::VectorXd obs(kObsDim118);
  obs.head(kObsDim113) =
      build_obs_113(state, target, executed_qdes_feedback, default_q);
  int o = kObsDim113;
  obs[o++] = target.desired_lateral_velocity;
  obs.segment<2>(o) = target.gait_clock; o += 2;
  obs[o++] = target.locomotion_mode;
  obs[o++] = target.upper_intervention;
  return obs;  // o == 118
}

}  // namespace a3_pingpong
