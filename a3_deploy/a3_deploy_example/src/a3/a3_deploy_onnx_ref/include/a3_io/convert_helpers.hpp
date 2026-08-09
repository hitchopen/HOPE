// Copyright (c) 2026, AgiBot Inc. All rights reserved.
// 对应 notes/a3_backend_plan.md §? / PR 5
//
// Shared pure-function helpers for the 4 joint-state subscribers. Lives in
// the header so the templated reorder helper can be used by each concrete
// ConvertMessage() without touching ROS2 types.

#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <string>

namespace a3_io {

// Reorder a single topic's `N` joints into the A3 topic's native order.
//
// Input arrays (`names_in`, `pos_in`, `vel_in`, `eff_in`) come straight out
// of a joint_msgs::msg::JointState's `joints` vector (already extracted and
// size-checked at the caller side). `expected` is the canonical A3 topic
// ordering (e.g. kA3ArmJointNames). For every slot `j` in `expected`, we
// scan `names_in` for a matching name and copy its (pos, vel, eff) into
// `out_pos[j]`, `out_vel[j]`, `out_eff[j]`.
//
// Returns true iff every expected name was located exactly once in
// `names_in`; returns false if any expected name is missing. Duplicate
// names in `names_in` are tolerated — the last matching entry wins (caller
// can choose stricter policy by inspecting the returned miss count).
template <std::size_t N>
bool ReorderByName(const std::array<std::string, N>& names_in,
                   const std::array<double, N>& pos_in,
                   const std::array<double, N>& vel_in,
                   const std::array<double, N>& eff_in,
                   const std::array<std::string, N>& expected,
                   std::array<double, N>& out_pos,
                   std::array<double, N>& out_vel,
                   std::array<double, N>& out_eff) {
  for (std::size_t j = 0; j < N; ++j) {
    bool found = false;
    for (std::size_t i = 0; i < N; ++i) {
      if (names_in[i] == expected[j]) {
        out_pos[j] = pos_in[i];
        out_vel[j] = vel_in[i];
        out_eff[j] = eff_in[i];
        found = true;
        // Do not break — "last wins" for duplicates.
      }
    }
    if (!found) return false;
  }
  return true;
}

// header::stamp (sec, nanosec) -> int64 ns.
inline std::int64_t StampToNs(std::int32_t sec, std::uint32_t nanosec) {
  return static_cast<std::int64_t>(sec) * 1'000'000'000LL +
         static_cast<std::int64_t>(nanosec);
}

inline std::int64_t NowSystemNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

template <class Sample>
void StampReceiptTime(Sample& sample) {
  sample.source_stamp_valid = sample.stamp.value > 0;
  sample.recv_stamp.value = NowSystemNs();
  if (!sample.source_stamp_valid) sample.stamp = sample.recv_stamp;
}

// XYZW (ROS2 sensor_msgs/Imu convention) -> WXYZ (a3_sync::ImuSample
// convention). Pure; no normalisation (upstream already emits unit quats).
inline std::array<double, 4> QuatXyzwToWxyz(double x, double y, double z,
                                            double w) {
  return {w, x, y, z};
}

}  // namespace a3_io
