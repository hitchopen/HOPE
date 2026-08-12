// Racket forward kinematics for the 175-D deploy_parity observation.
//
// racket_pos_pelvis(q) returns the racket (pingpang_red_Link) position expressed
// in the PELVIS (root) frame, from the 10-revolute-joint chain
// pelvis_link -> right_wrist_yaw_Link plus the fixed pingpang mount offset.
//
// World position is then:
//   racket_pos_w = base_pos_w + mat_from_quat(base_quat_w) * racket_pos_pelvis(q)
//
// Source of truth: hope_training/.../assets/agibot_a3/urdf/model.urdf.
// Each joint transform is  Translate(origin_xyz) * Rot_rpy(origin_rpy) * Rot_axis(q_i),
// accumulated left-to-right down the chain. All origin_rpy are 0 except joints
// 4 and 5 (the two shoulder joints). q_i is read from q at the Isaac q-index.
// This is validated to < 1e-4 m against Isaac's pingpang_red_Link body pose
// (scripts/racket_fk_ref.py). Keep it in lock-step with that numpy reference.
#pragma once

#include <Eigen/Dense>
#include <cmath>

#include "a3_pingpong/pp_frame_math.hpp"

namespace a3_pingpong {

// URDF fixed-axis roll-pitch-yaw = Rz(yaw) * Ry(pitch) * Rx(roll).
inline Mat3 rot_rpy(double roll, double pitch, double yaw) {
  const double cr = std::cos(roll), sr = std::sin(roll);
  const double cp = std::cos(pitch), sp = std::sin(pitch);
  const double cy = std::cos(yaw), sy = std::sin(yaw);
  Mat3 Rx, Ry, Rz;
  Rx << 1, 0, 0, 0, cr, -sr, 0, sr, cr;
  Ry << cp, 0, sp, 0, 1, 0, -sp, 0, cp;
  Rz << cy, -sy, 0, sy, cy, 0, 0, 0, 1;
  return Rz * Ry * Rx;
}

// Right-handed rotation of angle a about a principal local axis (0=x,1=y,2=z).
inline Mat3 rot_axis(int axis, double a) {
  const double c = std::cos(a), s = std::sin(a);
  Mat3 R = Mat3::Identity();
  switch (axis) {
    case 0:  // x
      R << 1, 0, 0, 0, c, -s, 0, s, c;
      break;
    case 1:  // y
      R << c, 0, s, 0, 1, 0, -s, 0, c;
      break;
    case 2:  // z
      R << c, -s, 0, s, c, 0, 0, 0, 1;
      break;
    default:
      break;
  }
  return R;
}

struct RacketPosePelvis {
  Vec3 position = Vec3::Zero();
  Mat3 rotation = Mat3::Identity();
};

// Racket pose in the pelvis (root) frame. q is the 31-vector of joint angles
// in Isaac order (same order as PpRobotState::q). The pingpang mount fixed
// joints have identity rotation, so the final wrist-chain rotation is also the
// pingpang_red_Link rotation. Its local +Y axis is the hitting-face normal.
inline RacketPosePelvis racket_pose_pelvis(const Eigen::VectorXd& q) {
  // Chain from pelvis_link to right_wrist_yaw_Link. Columns:
  //   isaac q-index, origin xyz, origin rpy, revolute axis (0=x,1=y,2=z).
  struct Joint {
    int idx;
    double ox, oy, oz;
    double rr, rp, ry;
    int axis;
  };
  static const Joint chain[10] = {
      // #1  waist_yaw            idx 2
      {2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2},
      // #2  waist_roll           idx 5
      {5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0},
      // #3  waist_pitch          idx 8   (child = torso_Link)
      {8, -0.0199999999998296, 0.0, 0.00500000000000012, 0.0, 0.0, 0.0, 1},
      // #4  right_shoulder_pitch idx 13  (nonzero rpy: roll = -0.0872664625997163)
      {13, 0.03030000000207, -0.148224512812557, 0.283817906204843,
       -0.0872664625997163, 0.0, 0.0, 1},
      // #5  right_shoulder_roll  idx 18  (nonzero rpy: roll = +0.0872664625997163)
      {18, 0.0, -0.0589999999982601, 0.0, 0.0872664625997163, 0.0, 0.0, 0},
      // #6  right_shoulder_yaw   idx 22
      {22, 0.0, -0.0100000000003387, -0.147499999999977, 0.0, 0.0, 0.0, 2},
      // #7  right_elbow          idx 24
      {24, 0.00999999997356363, 0.0, -0.132999999990091, 0.0, 0.0, 0.0, 1},
      // #8  right_wrist_roll     idx 26
      {26, 0.129, 0.0, -0.00999999999980283, 0.0, 0.0, 0.0, 0},
      // #9  right_wrist_pitch    idx 28
      {28, 0.0600000000000003, 0.0, 0.0, 0.0, 0.0, 0.0, 1},
      // #10 right_wrist_yaw      idx 30
      {30, 0.046, 0.0, 0.0, 0.0, 0.0, 0.0, 2},
  };

  // Fixed pingpang_red mount offset relative to right_wrist_yaw_Link
  // (right_hand_pingpang_joint is identity, so this is applied at the wrist_yaw
  // frame). Matches Isaac's pingpang_red_Link body pose.
  static const Vec3 kMount(0.21021, 0.032078, 0.032036);

  Mat3 R = Mat3::Identity();
  Vec3 t = Vec3::Zero();
  for (const Joint& j : chain) {
    // T <- T * ( Translate(origin_xyz) * Rot_rpy(origin_rpy) * Rot_axis(q) )
    const Vec3 ti(j.ox, j.oy, j.oz);
    const Mat3 Ri = rot_rpy(j.rr, j.rp, j.ry) * rot_axis(j.axis, q(j.idx));
    t = t + R * ti;
    R = R * Ri;
  }
  RacketPosePelvis pose;
  pose.position = t + R * kMount;
  pose.rotation = R;
  return pose;
}

inline Vec3 racket_pos_pelvis(const Eigen::VectorXd& q) {
  return racket_pose_pelvis(q).position;
}

}  // namespace a3_pingpong
