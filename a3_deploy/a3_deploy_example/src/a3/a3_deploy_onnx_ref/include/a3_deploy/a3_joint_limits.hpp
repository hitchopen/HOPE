// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// A3 SDK-order actuator limits shared by the generic motion runner and the
// ping-pong runner. Values come at full stored precision from
// agi/URDF/a3_t2d5/urdf/model.urdf.
#pragma once

#include "robot_io/a3_layout_extra.hpp"

#include <Eigen/Dense>

#include <array>
#include <cstddef>

namespace a3_deploy {

// SDK order: waist[3], neck[2], left arm[7], right arm[7],
// left leg[6], right leg[6].
inline constexpr std::array<double, robot_io::kA3Dof> kA3SdkJointPosLo = {
    -2.6179938779914944, -0.3490658503988659, -0.4886921905584123,
    -1.0471975511965976, -0.4363323129985824, -2.8797932657906435,
    -0.08726646259971647, -2.792526803190927, -0.9599310885968813,
    -2.792526803190927, -1.6231562043547265, -1.6231562043547265,
    -2.8797932657906404, -2.6179938779914944, -2.792526803190927,
    -0.9599310885968813, -2.792526803190927, -1.6231562043547265,
    -1.6231562043547265, -2.5132741228718345, -0.5235987755982988,
    -2.722713633111154, -0.12217304763960307, -0.9075712110370514,
    -0.3490658503988659, -2.5132741228718345, -1.6057029118347832,
    -2.722713633111154, -0.12217304763960307, -0.9075712110370514,
    -0.3490658503988659};

inline constexpr std::array<double, robot_io::kA3Dof> kA3SdkJointPosHi = {
    2.6179938779914944, 0.3490658503988659, 0.4188790204786391,
    1.0471975511965976, 0.2617993877991494, 2.8797932657906435,
    2.6179938779914944, 2.792526803190927, 1.7453292519943295,
    2.792526803190927, 1.6231562043547265, 1.6231562043547265,
    2.8797932657906435, 0.08726646259971647, 2.792526803190927,
    1.7453292519943295, 2.792526803190927, 1.6231562043547265,
    1.6231562043547265, 2.9321531433504737, 1.6057029118347832,
    2.722713633111154, 2.4958208303518914, 0.5235987755982988,
    0.3490658503988659, 2.9321531433504737, 0.5235987755982988,
    2.722713633111154, 2.4958208303518914, 0.5235987755982988,
    0.3490658503988659};

inline constexpr std::array<double, robot_io::kA3Dof> kA3SdkJointEffortLimit = {
    220.0, 46.0, 118.0, 6.0, 6.0, 60.0, 60.0, 24.0, 24.0, 24.0, 6.0,
    6.0, 60.0, 60.0, 24.0, 24.0, 24.0, 6.0, 6.0, 220.0, 220.0, 220.0,
    320.0, 118.2, 54.75, 220.0, 220.0, 220.0, 320.0, 118.2, 54.75};

inline constexpr std::array<double, robot_io::kA3Dof>
    kA3SdkJointVelocityLimit = {
        12.042771838760874, 22.7, 9.24785, 12.775810124598491,
        12.775810124598491, 13.613568165555769, 13.613568165555769,
        15.707963267948966, 15.707963267948966, 15.707963267948966,
        12.775810124598491, 12.775810124598491, 13.613568165555769,
        13.613568165555769, 15.707963267948966, 15.707963267948966,
        15.707963267948966, 12.775810124598491, 12.775810124598491,
        12.042771838760874, 12.042771838760874, 12.042771838760874,
        14.660765716752367, 10.8, 19.37, 12.042771838760874,
        12.042771838760874, 12.042771838760874, 14.660765716752367,
        10.8, 19.37};

inline double A3PolicyJointPosLo(std::size_t policy_index) noexcept {
  if (policy_index >= robot_io::kA3PolicyToSdkIdx.size()) return 0.0;
  return kA3SdkJointPosLo[robot_io::kA3PolicyToSdkIdx[policy_index]];
}

inline double A3PolicyJointPosHi(std::size_t policy_index) noexcept {
  if (policy_index >= robot_io::kA3PolicyToSdkIdx.size()) return 0.0;
  return kA3SdkJointPosHi[robot_io::kA3PolicyToSdkIdx[policy_index]];
}

inline int ClampA3SdkQToPositionLimits(Eigen::VectorXd& q_sdk) noexcept {
  int clamped = 0;
  const int count =
      q_sdk.size() < robot_io::kA3Dof ? static_cast<int>(q_sdk.size())
                                     : robot_io::kA3Dof;
  for (int i = 0; i < count; ++i) {
    if (q_sdk[i] < kA3SdkJointPosLo[i]) {
      q_sdk[i] = kA3SdkJointPosLo[i];
      ++clamped;
    } else if (q_sdk[i] > kA3SdkJointPosHi[i]) {
      q_sdk[i] = kA3SdkJointPosHi[i];
      ++clamped;
    }
  }
  return clamped;
}

}  // namespace a3_deploy
