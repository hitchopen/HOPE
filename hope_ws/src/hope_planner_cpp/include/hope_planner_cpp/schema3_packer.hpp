#pragma once

#include "hope_planner_cpp/reach_permission.hpp"
#include "hope_planner_cpp/schema2_packer.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace hope_planner_cpp {

struct Schema3Packet {
  std::array<double, 21> values{};
  WireIdentity identity{};
  bool valid = false;
};

class Schema3Packer {
 public:
  static Schema3Packet pack(
      const RacketCommand* command,
      double swing_sign,
      double strike_deadline_wall_s,
      double policy_z_offset,
      std::int64_t producer_wall_ns,
      const WireIdentity& identity,
      std::size_t estimator_sample_count,
      double estimator_span_s,
      const ReachPermission& permission) noexcept;
};

}  // namespace hope_planner_cpp
