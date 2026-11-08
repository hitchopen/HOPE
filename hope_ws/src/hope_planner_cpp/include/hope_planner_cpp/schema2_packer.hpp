#pragma once

#include "hope_planner_cpp/types.hpp"

#include <array>
#include <cstdint>
#include <stdexcept>

namespace hope_planner_cpp {

struct WireIdentity {
  std::uint64_t command_sequence = 0;
  std::uint64_t flight_id = 0;
  std::uint64_t revision_id = 0;
};

struct Schema2Packet {
  std::array<double, 19> values{};
  WireIdentity identity{};
  bool valid = false;
};

class Schema2Packer {
 public:
  explicit Schema2Packer(std::uint64_t seed = 0, std::uint64_t end = (1ULL << 52))
      : command_sequence_(seed), flight_id_(seed), identity_end_(end) {
    if (seed >= end || end > (1ULL << 52))
      throw std::invalid_argument("invalid Planner wire identity range");
  }
  WireIdentity next_identity(bool valid, std::int64_t steady_now_ns, bool new_physical_flight = false) noexcept;

  static Schema2Packet pack(
      const RacketCommand* command,
      double swing_sign,
      double strike_deadline_wall_s,
      double policy_z_offset,
      std::int64_t producer_wall_ns,
      const WireIdentity& identity,
      std::size_t estimator_sample_count,
      double estimator_span_s) noexcept;

 private:
  std::uint64_t command_sequence_ = 0;
  std::uint64_t flight_id_ = 0;
  std::uint64_t revision_id_ = 0;
  std::int64_t last_valid_steady_ns_ = 0;
  std::uint64_t identity_end_ = (1ULL << 52);
};

}  // namespace hope_planner_cpp
