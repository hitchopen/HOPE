#include "hope_planner_cpp/schema2_packer.hpp"

#include <cmath>

namespace hope_planner_cpp {

WireIdentity Schema2Packer::next_identity(
    bool valid, std::int64_t steady_now_ns) noexcept {
  ++command_sequence_;
  if (valid) {
    if (last_valid_steady_ns_ <= 0 || steady_now_ns <= last_valid_steady_ns_ ||
        steady_now_ns - last_valid_steady_ns_ > 250'000'000LL) {
      ++flight_id_;
      revision_id_ = 1;
    } else {
      ++revision_id_;
    }
    last_valid_steady_ns_ = steady_now_ns;
  }
  return WireIdentity{command_sequence_, flight_id_, revision_id_};
}

Schema2Packet Schema2Packer::pack(
    const RacketCommand* command,
    double swing_sign,
    double strike_deadline_wall_s,
    double policy_z_offset,
    std::int64_t producer_wall_ns,
    const WireIdentity& identity,
    std::size_t estimator_sample_count,
    double estimator_span_s) noexcept {
  Schema2Packet packet;
  const std::int64_t producer_sec = producer_wall_ns / 1'000'000'000LL;
  const std::int64_t producer_nsec = producer_wall_ns % 1'000'000'000LL;
  const double producer_wall_s = static_cast<double>(producer_wall_ns) * 1.0e-9;
  const double time_to_strike_s = command != nullptr
      ? strike_deadline_wall_s - producer_wall_s
      : 0.0;
  const bool valid = command != nullptr && command->valid &&
                     command->position.allFinite() && command->velocity.allFinite() &&
                     std::isfinite(swing_sign) &&
                     std::isfinite(strike_deadline_wall_s) &&
                     strike_deadline_wall_s > 0.0 &&
                     std::isfinite(time_to_strike_s) && time_to_strike_s > 0.0 &&
                     std::isfinite(policy_z_offset) && producer_wall_ns > 0 &&
                     std::isfinite(estimator_span_s) && estimator_span_s >= 0.0;
  packet.valid = valid;
  packet.identity = identity;
  packet.values = {
      2.0,
      valid ? 1.0 : 0.0,
      valid ? swing_sign : 0.0,
      valid ? command->position.x() : 0.0,
      valid ? command->position.y() : 0.0,
      valid ? command->position.z() + policy_z_offset : 0.0,
      valid ? command->velocity.x() : 0.0,
      valid ? command->velocity.y() : 0.0,
      valid ? command->velocity.z() : 0.0,
      valid ? time_to_strike_s : 0.0,
      valid ? strike_deadline_wall_s : 0.0,
      0.0,
      static_cast<double>(producer_sec),
      static_cast<double>(producer_nsec),
      static_cast<double>(identity.command_sequence),
      valid ? static_cast<double>(identity.flight_id) : 0.0,
      valid ? static_cast<double>(identity.revision_id) : 0.0,
      valid ? static_cast<double>(estimator_sample_count) : 0.0,
      valid ? estimator_span_s : 0.0,
  };
  return packet;
}

}  // namespace hope_planner_cpp
