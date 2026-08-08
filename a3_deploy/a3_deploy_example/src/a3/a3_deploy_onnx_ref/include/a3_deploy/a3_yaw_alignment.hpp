// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// Shared yaw-alignment helpers for A3 teleop references.  Both /ta A3
// teleop and SMPL ZMQ teleop use these functions so entering, reconnecting,
// and source switching apply the same heading convention.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace a3_deploy::yaw_alignment {

inline constexpr double kPi = 3.141592653589793238462643383279502884;

inline double WrapPi(double x) noexcept {
  while (x > kPi) x -= 2.0 * kPi;
  while (x < -kPi) x += 2.0 * kPi;
  return x;
}

inline std::array<double, 4> NormalizeQuat(
    std::array<double, 4> q) noexcept {
  double n2 = 0.0;
  for (double v : q) n2 += v * v;
  if (n2 <= std::numeric_limits<double>::epsilon()) {
    return {1.0, 0.0, 0.0, 0.0};
  }
  const double inv = 1.0 / std::sqrt(n2);
  for (double& v : q) v *= inv;
  return q;
}

inline std::array<double, 4> QuatConj(
    const std::array<double, 4>& q) noexcept {
  return {q[0], -q[1], -q[2], -q[3]};
}

inline std::array<double, 4> QuatMul(
    const std::array<double, 4>& a,
    const std::array<double, 4>& b) noexcept {
  const double w1 = a[0], x1 = a[1], y1 = a[2], z1 = a[3];
  const double w2 = b[0], x2 = b[1], y2 = b[2], z2 = b[3];
  return {
      w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
      w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
      w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
      w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
  };
}

inline double QuatYawRad(const std::array<double, 4>& q_in) noexcept {
  const auto q = NormalizeQuat(q_in);
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  return std::atan2(2.0 * (w * z + x * y),
                    1.0 - 2.0 * (y * y + z * z));
}

inline std::array<double, 4> QuatFromYawRad(double yaw) noexcept {
  const double half = 0.5 * yaw;
  return {std::cos(half), 0.0, 0.0, std::sin(half)};
}

inline double ComputeYawOffsetRad(
    const std::array<double, 4>& robot_root_quat_wxyz,
    const std::array<double, 4>& reference_root_quat_wxyz) noexcept {
  return WrapPi(QuatYawRad(robot_root_quat_wxyz) -
                QuatYawRad(reference_root_quat_wxyz));
}

inline std::array<double, 4> ApplyYawOffset(
    double yaw_offset_rad,
    const std::array<double, 4>& reference_root_quat_wxyz) noexcept {
  return QuatMul(QuatFromYawRad(yaw_offset_rad),
                 NormalizeQuat(reference_root_quat_wxyz));
}

}  // namespace a3_deploy::yaw_alignment
