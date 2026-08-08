// Copyright (c) 2026, AgiBot Inc. All rights reserved.
// 对应 notes/a3_backend_plan.md §PR 8 Task 8.2
//
// ExpandToBackend: glue from the 29-DOF policy output (MuJoCo/flat body-joint
// order, first 29 entries of MakeA3Layout31()) to a 31-DOF RobotCommand that
// A3AimrtBackend::SendCommand can consume. Head/neck slots [3..4] are not
// controlled by the policy; deployment holds them at target position 0 with
// fixed PD gains.
#pragma once

#include "robot_io/robot_io_backend.hpp"

#include <array>

namespace a3_deploy {

inline constexpr double kA3HeadTargetPositionRad = 0.0;
inline constexpr double kA3HeadKp = 40.0;
inline constexpr double kA3HeadKd = 2.0;

// Expand a 29-DOF policy output + per-joint Kp/Kd arrays into a 31-DOF
// RobotCommand. All five Eigen vectors of `out` are resized to 31; body
// [0..28] filled from inputs; head/neck [3..4] get fixed zero-position PD
// control; dq_des / tau_ff zeroed everywhere.
void ExpandToBackend(
    const std::array<double, 29>& q_des_29,
    const std::array<double, 29>& kps,
    const std::array<double, 29>& kds,
    robot_io::RobotCommand& out);

}  // namespace a3_deploy
