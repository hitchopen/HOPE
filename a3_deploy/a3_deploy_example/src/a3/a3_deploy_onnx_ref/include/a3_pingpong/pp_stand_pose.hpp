#pragma once

#include <Eigen/Core>

#include "a3_policy_parameters.hpp"
#include "robot_io/a3_layout_extra.hpp"

namespace a3_pingpong {

// Manual PD_STAND belongs to the robot controller, not the policy's action
// coordinates. A policy may use a crouched or gravity-compensated action offset.
inline const Eigen::VectorXd& official_stand_q_sdk() {
  static const Eigen::VectorXd pose = [] {
    Eigen::VectorXd q = Eigen::VectorXd::Zero(31);
    for (int i = 0; i < robot_io::kA3PolicyDof; ++i)
      q[robot_io::kA3PolicyToSdkIdx[i]] = a3_default_angles[i];
    return q;
  }();
  return pose;
}

}  // namespace a3_pingpong
