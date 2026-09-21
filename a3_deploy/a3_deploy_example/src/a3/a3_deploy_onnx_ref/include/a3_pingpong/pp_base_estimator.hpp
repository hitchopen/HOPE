// Leg-FK + IMU floating-base estimator for planted-stance ping-pong.
//
// The 180-D obs needs the pelvis world height (base_pos_w.z) for the
// racket_target_pos_b term and (via torso_pos_from_base) the motion_anchor term.
// On the real robot there is no global localisation; for a PLANTED stance we
// recover the pelvis height from leg kinematics + the pelvis IMU, assuming the
// support foot's sole is on the ground (z = 0).
//
// FK validated against the model's own reference body_pos_w: ankle matches to
// 1.5 mm, and max(estL,estR) reproduces the reference pelvis height to ~3 mm
// across the whole forehand (the support/right foot stays planted; the swing
// foot lifts a few cm, so max() picks the grounded one). Chain + offsets are
// from a3_t2d5.xml; sole offset (0.04,0,-0.072) from the foot contact geoms.
#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>

#include "a3_pingpong/pp_frame_math.hpp"

namespace a3_pingpong {

// One leg-chain link: static offset + fixed orientation (w,x,y,z), the hinge
// axis (0=X,1=Y,2=Z), and the Isaac-order joint index that drives it.
struct LegLink {
  Vec3 pos;
  Vec4 quat;
  int axis;
  int qidx;
};

inline Mat3 axis_rot(int axis, double th) {
  switch (axis) {
    case 0: return Mat3(Eigen::AngleAxisd(th, Vec3::UnitX()));
    case 1: return Mat3(Eigen::AngleAxisd(th, Vec3::UnitY()));
    default: return Mat3(Eigen::AngleAxisd(th, Vec3::UnitZ()));
  }
}

// Foot-sole point in the pelvis frame for a leg chain at joint config q (Isaac order).
inline Vec3 sole_in_pelvis(const std::array<LegLink, 6>& leg, const Vec3& sole_off,
                           const Eigen::VectorXd& q) {
  Mat3 R = Mat3::Identity();
  Vec3 t = Vec3::Zero();
  for (const auto& L : leg) {
    const double th = (q.size() > L.qidx) ? q[L.qidx] : 0.0;
    t = t + R * L.pos;
    R = R * (mat_from_quat(L.quat) * axis_rot(L.axis, th));
  }
  return t + R * sole_off;
}

namespace leg_fk_detail {
inline const std::array<LegLink, 6>& left_leg() {
  static const std::array<LegLink, 6> kLeft = {{
      {Vec3(0, 0.122983, -0.178753), Vec4(0.991445, -0.130526, 0, 0), 1, 0},    // hip_pitch
      {Vec3(0, 0.0113163, -0.042233), Vec4(0.991445, 0.130526, 0, 0), 0, 3},    // hip_roll
      {Vec3(0, 0, 0), Vec4(1, 0, 0, 0), 2, 6},                                   // hip_yaw
      {Vec3(0, 0, -0.37), Vec4(1, 0, 0, 0), 1, 9},                               // knee
      {Vec3(0, 0, -0.415), Vec4(1, 0, 0, 0), 1, 14},                            // ankle_pitch
      {Vec3(0, 0, 0), Vec4(1, 0, 0, 0), 0, 19}}};                               // ankle_roll
  return kLeft;
}
inline const std::array<LegLink, 6>& right_leg() {
  static const std::array<LegLink, 6> kRight = {{
      {Vec3(0, -0.122983, -0.178753), Vec4(0.991445, 0.130526, 0, 0), 1, 1},
      {Vec3(-0.0011, -0.011316, -0.042233), Vec4(0.991445, -0.130526, 0, 0), 0, 4},
      {Vec3(0, 0, 0), Vec4(1, 0, 0, 0), 2, 7},
      {Vec3(0, 0, -0.37), Vec4(1, 0, 0, 0), 1, 10},
      {Vec3(0, 0, -0.415), Vec4(1, 0, 0, 0), 1, 15},
      {Vec3(0, 0, 0), Vec4(1, 0, 0, 0), 0, 20}}};
  return kRight;
}
inline const Vec3& sole_offset() {
  static const Vec3 kSole(0.04, 0.0, -0.072);  // ankle_roll origin -> ground contact
  return kSole;
}
}  // namespace leg_fk_detail

// Schema33 Layer C: both sole contact points in the WORLD frame (mocap pelvis pose +
// leg FK), for the support-geometry envelope channel.  Index 0 = left, 1 = right.
inline std::array<Vec3, 2> sole_positions_w(const Eigen::VectorXd& q_isaac,
                                            const Vec3& base_pos_w,
                                            const Vec4& base_quat) {
  const Mat3 Rb = mat_from_quat(base_quat);
  return {base_pos_w + Rb * sole_in_pelvis(leg_fk_detail::left_leg(),
                                            leg_fk_detail::sole_offset(), q_isaac),
          base_pos_w + Rb * sole_in_pelvis(leg_fk_detail::right_leg(),
                                            leg_fk_detail::sole_offset(), q_isaac)};
}

// Schema34 actor column 111 uses the ankle-roll LINK origins, matching Isaac Lab's
// ``robot.data.body_pos_w[left/right_ankle_roll_Link]`` exactly.  This is deliberately
// distinct from sole_positions_w(): the latter includes the contact-point offset and remains
// the Schema33 recovery-envelope source.
inline std::array<Vec3, 2> ankle_roll_positions_w(
    const Eigen::VectorXd& q_isaac, const Vec3& base_pos_w,
    const Vec4& base_quat) {
  const Mat3 Rb = mat_from_quat(base_quat);
  const Vec3 zero_offset = Vec3::Zero();
  return {base_pos_w + Rb * sole_in_pelvis(
                               leg_fk_detail::left_leg(), zero_offset, q_isaac),
          base_pos_w + Rb * sole_in_pelvis(
                               leg_fk_detail::right_leg(), zero_offset, q_isaac)};
}

// Pelvis world height (m) from leg-FK + IMU, assuming the support foot's sole is
// on the ground. max() over feet picks the grounded one (robust to weight shift).
inline double estimate_base_height(const Eigen::VectorXd& q_isaac, const Vec4& base_quat) {
  static const std::array<LegLink, 6> kLeft = {{
      {Vec3(0, 0.122983, -0.178753), Vec4(0.991445, -0.130526, 0, 0), 1, 0},    // hip_pitch
      {Vec3(0, 0.0113163, -0.042233), Vec4(0.991445, 0.130526, 0, 0), 0, 3},    // hip_roll
      {Vec3(0, 0, 0), Vec4(1, 0, 0, 0), 2, 6},                                   // hip_yaw
      {Vec3(0, 0, -0.37), Vec4(1, 0, 0, 0), 1, 9},                               // knee
      {Vec3(0, 0, -0.415), Vec4(1, 0, 0, 0), 1, 14},                            // ankle_pitch
      {Vec3(0, 0, 0), Vec4(1, 0, 0, 0), 0, 19}}};                               // ankle_roll
  static const std::array<LegLink, 6> kRight = {{
      {Vec3(0, -0.122983, -0.178753), Vec4(0.991445, 0.130526, 0, 0), 1, 1},
      {Vec3(-0.0011, -0.011316, -0.042233), Vec4(0.991445, -0.130526, 0, 0), 0, 4},
      {Vec3(0, 0, 0), Vec4(1, 0, 0, 0), 2, 7},
      {Vec3(0, 0, -0.37), Vec4(1, 0, 0, 0), 1, 10},
      {Vec3(0, 0, -0.415), Vec4(1, 0, 0, 0), 1, 15},
      {Vec3(0, 0, 0), Vec4(1, 0, 0, 0), 0, 20}}};
  static const Vec3 kSole(0.04, 0.0, -0.072);  // ankle_roll origin -> ground contact
  const Mat3 Rb = mat_from_quat(base_quat);
  const double zl = -(Rb * sole_in_pelvis(kLeft, kSole, q_isaac)).z();
  const double zr = -(Rb * sole_in_pelvis(kRight, kSole, q_isaac)).z();
  return std::max(zl, zr);
}

}  // namespace a3_pingpong
