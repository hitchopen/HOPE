// Copyright (c) 2026, AgiBot Inc. All rights reserved.
// 对应 notes/a3_backend_plan.md §PR 8
//
// BuildSafeHaltCommand: produces a 31-DOF "safe halt" command that holds the
// current measured position with kp = kd = tau_ff = 0 so that the robot goes
// compliant without breaking the command stream (motion_control_a3 still sees
// a live backend).
#pragma once

#include "robot_io/robot_io_backend.hpp"

namespace a3_deploy {

// Build a finite 31-DOF safe-halt command. If `q_current.size() == 31`, finite
// q_des entries mirror q_current and non-finite entries are zeroed; otherwise
// q_des is fully zeroed. kp / kd / tau_ff / dq_des are always zero, size 31.
void BuildSafeHaltCommand(const Eigen::VectorXd& q_current,
                          robot_io::RobotCommand& out);

}  // namespace a3_deploy
