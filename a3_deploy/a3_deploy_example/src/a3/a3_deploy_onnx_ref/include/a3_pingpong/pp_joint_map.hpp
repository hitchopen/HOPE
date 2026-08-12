// Map model_15200's 31 actions/targets (ONNX joint_names = interleaved Isaac
// articulation order) onto the backend's 31 MuJoCo slots (waist3, neck2, Larm7,
// Rarm7, Lleg6, Rleg6 -- from robot_io::MakeA3Layout31). The RobotCommand is
// positional; publishers attach names per slot, so we MUST scatter by name into
// the correct slot. A wrong map commands the wrong joints.
#pragma once

#include <array>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace a3_pingpong {

// Backend slot -> joint name, MuJoCo order (mirrors robot_io::MakeA3Layout31()).
inline const std::array<std::string, 31>& backend_joint_order() {
  static const std::array<std::string, 31> N = {
      "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
      "head_yaw_joint", "head_pitch_joint",
      "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
      "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
      "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
      "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
      "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
      "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
      "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
      "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint"};
  return N;
}

// map_out[i] = backend slot for source joint i. Returns false unless every
// source name matches a backend slot AND the result is a bijection (all 31
// slots filled exactly once).
inline bool build_src_to_sdk(const std::vector<std::string>& src,
                             std::array<int, 31>& map_out) {
  const auto& B = backend_joint_order();
  if (src.size() != 31) return false;
  std::array<int, 31> hit{}; hit.fill(0);
  for (int i = 0; i < 31; ++i) {
    int slot = -1;
    for (int s = 0; s < 31; ++s)
      if (B[s] == src[i]) { slot = s; break; }
    if (slot < 0) return false;
    map_out[i] = slot;
    hit[slot]++;
  }
  for (int s = 0; s < 31; ++s)
    if (hit[s] != 1) return false;
  return true;
}

// Scatter a source-ordered (Isaac) vector into backend (MuJoCo slot) order:
//   out[map[i]] = src[i]   (src in Isaac order -> out in SDK order)
inline Eigen::VectorXd to_sdk_order(const Eigen::VectorXd& src,
                                    const std::array<int, 31>& map) {
  Eigen::VectorXd out(31);
  for (int i = 0; i < 31; ++i) out[map[i]] = src[i];
  return out;
}

// Gather a backend-ordered (SDK/MuJoCo) vector into source (Isaac) order:
//   out[i] = sdk[map[i]]   (sdk in SDK order -> out in Isaac order)
inline Eigen::VectorXd from_sdk_order(const Eigen::VectorXd& sdk,
                                      const std::array<int, 31>& map) {
  Eigen::VectorXd out(31);
  for (int i = 0; i < 31; ++i) out[i] = sdk[map[i]];
  return out;
}

}  // namespace a3_pingpong
