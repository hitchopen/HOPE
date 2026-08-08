// Backend(SDK)-order joint POSITION limits (31 DOF), extracted verbatim from the
// A3 ping-pong MJCF (aimrt_mujoco_sim a3_pingpong.xml) and ordered to match
// robot_io::MakeA3Layout31() / pp_joint_map backend_joint_order(). Validated: the
// policy's default_joint_pos lies strictly inside every range. Used as a hardware
// SAFETY clamp on q_des before publish — in-range commands are untouched, so this
// is a no-op for in-distribution actions and only catches pathological spikes.
#pragma once

#include <Eigen/Dense>

namespace a3_pingpong {

// order: waist(yaw,roll,pitch), neck(head_yaw,head_pitch), Larm[5..11], Rarm[12..18],
//        Lleg[19..24], Rleg[25..30]  (== backend_joint_order()).
inline constexpr double kSdkJointPosLo[31] = {
    -2.61799, -0.34907, -0.48869, -1.04720, -0.43633, -2.87979, -0.08727, -2.79253,
    -0.95993, -2.79253, -1.62316, -1.62316, -2.87979, -2.61799, -2.79253, -0.95993,
    -2.79253, -1.62316, -1.62316, -2.51327, -0.52360, -2.72271, -0.12217, -0.90757,
    -0.34907, -2.51327, -1.60570, -2.72271, -0.12217, -0.90757, -0.34907};
inline constexpr double kSdkJointPosHi[31] = {
    2.61799,  0.34907,  0.41888,  1.04720,  0.26180,  2.87979,  2.61799,  2.79253,
    1.74533,  2.79253,  1.62316,  1.62316,  2.87979,  0.08727,  2.79253,  1.74533,
    2.79253,  1.62316,  1.62316,  2.93215,  1.60570,  2.72271,  2.49582,  0.52360,
    0.34907,  2.93215,  0.52360,  2.72271,  2.49582,  0.52360,  0.34907};

// Match the training-side post-physics audit exactly: a joint is a physical
// fault only when its hard-limit excess is strictly greater than the exported
// non-negative tolerance.  q_des is still constrained separately and never
// receives this tolerance.
inline bool exceeds_joint_hard_limit(double q, double lo, double hi,
                                     double tolerance) {
  return q < lo - tolerance || q > hi + tolerance;
}

enum class ActualQHardLimitDisposition {
  kInside,
  kTelemetry,
  kFault,
};

// The exported policy contract owns whether measured-q excess is a termination
// or telemetry audit.  q_des hard clamping is separate and remains unchanged.
inline ActualQHardLimitDisposition classify_actual_q_hard_limit(
    double q, double lo, double hi, double tolerance,
    bool telemetry_only) {
  if (!exceeds_joint_hard_limit(q, lo, hi, tolerance))
    return ActualQHardLimitDisposition::kInside;
  return telemetry_only ? ActualQHardLimitDisposition::kTelemetry
                        : ActualQHardLimitDisposition::kFault;
}

// Clamp a backend(SDK)-order joint command in place to the position limits.
// Returns the number of joints that were clamped (0 == all in range).
inline int clamp_q_to_limits(Eigen::VectorXd& q_sdk) {
  int n = 0;
  const int m = q_sdk.size() < 31 ? static_cast<int>(q_sdk.size()) : 31;
  for (int i = 0; i < m; ++i) {
    if (q_sdk[i] < kSdkJointPosLo[i]) {
      q_sdk[i] = kSdkJointPosLo[i];
      ++n;
    } else if (q_sdk[i] > kSdkJointPosHi[i]) {
      q_sdk[i] = kSdkJointPosHi[i];
      ++n;
    }
  }
  return n;
}

}  // namespace a3_pingpong
