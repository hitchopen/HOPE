// Copyright (c) 2026, AgiBot Inc.
// All rights reserved.

#pragma once

#include <cstddef>

namespace aimrt_mujoco_sim::mujoco_sim_module::common::gate3_ball {

inline constexpr char kBodyName[] = "gate3_ball";
inline constexpr char kJointName[] = "gate3_ball_free_joint";
inline constexpr char kGeomName[] = "gate3_ball_collision";
inline constexpr char kRacketGeomName[] = "right_racket_collision";
inline constexpr char kTableGeomName[] = "gate3_table_collision";
inline constexpr char kNetGeomName[] = "gate3_net_collision";

inline constexpr double kTableSurfaceZ = 0.760;
inline constexpr double kBallRadius = 0.020;

inline constexpr std::size_t kShotId = 0;
inline constexpr std::size_t kActive = 1;
inline constexpr std::size_t kContactBits = 2;
inline constexpr std::size_t kRacketContactCount = 3;
inline constexpr std::size_t kTableContactCount = 4;
inline constexpr std::size_t kNetContactCount = 5;
inline constexpr std::size_t kRacketNormalForce = 6;
inline constexpr std::size_t kRequiredUserData = 7;

inline constexpr unsigned char kContactRacket = 1;
inline constexpr unsigned char kContactTable = 2;
inline constexpr unsigned char kContactNet = 4;

}  // namespace aimrt_mujoco_sim::mujoco_sim_module::common::gate3_ball
