#pragma once

#include "hope_planner_cpp/incoming_trajectory.hpp"

#include <hope_msgs/msg/ball_flight_packet.hpp>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <unordered_map>

namespace hope_planner_cpp {

constexpr std::uint16_t kBallFlightPacketSchemaVersion = 2;
constexpr const char* kBallFlightPacketHashAlgorithm = "fnv1a64-v2";

std::string flight_packet_identity_key(
    const FlightPacketMetadata& metadata);
std::string flight_packet_flight_identity_key(
    const FlightPacketMetadata& metadata);

// Hashes only immutable revision content. Retry index and transport receipt/
// publish times are intentionally excluded, so all retries share one content
// address.
std::string flight_packet_payload_hash(
    const FlightPacketMetadata& metadata,
    double net_x,
    double post_net_delay_s,
    const TrajectorySnapshot& snapshot) noexcept;

// Formal wire hash.  It operates on the serialized integer exposure stamps
// instead of converting Unix time through double seconds, so producer and
// consumer compute the same content address at current epoch magnitudes.
// Retry/receipt metadata is deliberately excluded.
std::string flight_packet_message_payload_hash(
    const hope_msgs::msg::BallFlightPacket& packet) noexcept;

bool validate_flight_snapshot(
    const TrajectorySnapshot& snapshot,
    std::string& reason) noexcept;

enum class FlightPacketDedupResult : std::uint8_t {
  kAccepted = 0,
  kDuplicate = 1,
  kIdentityConflict = 2,
};

// Bounded revision identity memory prevents retries of the same revision from
// ever producing another solve. Eviction is insertion-order only and is far
// longer than a rally.
class FlightPacketDeduplicator {
 public:
  explicit FlightPacketDeduplicator(std::size_t capacity = 256);

  FlightPacketDedupResult observe(
      const std::string& identity_key,
      const std::string& payload_hash);
  std::size_t size() const noexcept { return hashes_.size(); }

 private:
  std::size_t capacity_;
  std::deque<std::string> insertion_order_;
  std::unordered_map<std::string, std::string> hashes_;
};

}  // namespace hope_planner_cpp
