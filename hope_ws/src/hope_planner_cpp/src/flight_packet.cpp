#include "hope_planner_cpp/flight_packet.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <type_traits>

namespace hope_planner_cpp {
namespace {

class Fnv1a64 {
 public:
  void bytes(const void* data, std::size_t size) noexcept {
    const auto* value = static_cast<const std::uint8_t*>(data);
    for (std::size_t i = 0; i < size; ++i) {
      hash_ ^= value[i];
      hash_ *= 1099511628211ULL;
    }
  }

  template <typename Integer>
  void integer(Integer value) noexcept {
    using Unsigned = std::make_unsigned_t<Integer>;
    const Unsigned bits = static_cast<Unsigned>(value);
    for (std::size_t i = 0; i < sizeof(Unsigned); ++i) {
      const std::uint8_t byte = static_cast<std::uint8_t>(
          (bits >> (8U * i)) & static_cast<Unsigned>(0xff));
      bytes(&byte, 1);
    }
  }

  void floating(double value) noexcept {
    static_assert(sizeof(double) == sizeof(std::uint64_t));
    std::uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    integer(bits);
  }

  void string(const std::string& value) noexcept {
    integer<std::uint64_t>(value.size());
    bytes(value.data(), value.size());
  }

  std::string hex() const {
    std::ostringstream output;
    output << std::hex << std::setfill('0') << std::setw(16) << hash_;
    return output.str();
  }

 private:
  std::uint64_t hash_ = 14695981039346656037ULL;
};

std::int64_t seconds_to_ns(double value) noexcept {
  if (!std::isfinite(value)) return 0;
  constexpr double kLimit =
      static_cast<double>(std::numeric_limits<std::int64_t>::max()) * 1.0e-9;
  if (std::abs(value) > kLimit) return 0;
  return static_cast<std::int64_t>(std::llround(value * 1.0e9));
}

}  // namespace

std::string flight_packet_identity_key(
    const FlightPacketMetadata& metadata) {
  std::ostringstream output;
  output << metadata.session_id << '\x1f'
         << metadata.producer_instance_id << '\x1f'
         << metadata.trajectory_epoch << '\x1f'
         << metadata.flight_sequence;
  return output.str();
}

std::string flight_packet_payload_hash(
    const FlightPacketMetadata& metadata,
    double net_x,
    double post_net_delay_s,
    const TrajectorySnapshot& snapshot) noexcept {
  Fnv1a64 hash;
  hash.integer(kBallFlightPacketSchemaVersion);
  hash.string(metadata.session_id);
  hash.string(metadata.producer_instance_id);
  hash.integer(metadata.trajectory_epoch);
  hash.integer(metadata.flight_sequence);
  hash.string(metadata.frame_id);
  hash.string(snapshot.segment_boundary_reason);
  hash.floating(net_x);
  hash.floating(post_net_delay_s);
  hash.integer(seconds_to_ns(snapshot.segment_start_source_time_s));
  hash.integer(seconds_to_ns(snapshot.previous_segment_last_source_time_s));
  hash.integer(seconds_to_ns(snapshot.one_shot.net_cross_source_time_s));
  hash.integer(seconds_to_ns(snapshot.one_shot.commit_source_time_s));
  hash.integer<std::uint64_t>(snapshot.sample_count);
  for (std::size_t i = 0; i < snapshot.sample_count; ++i) {
    const BallSample& sample = snapshot.samples[i];
    hash.integer(
        sample.source_time_ns != 0
            ? sample.source_time_ns
            : seconds_to_ns(sample.source_time_s));
    hash.floating(sample.position.x());
    hash.floating(sample.position.y());
    hash.floating(sample.position.z());
    hash.integer<std::uint8_t>(sample.orientation_valid ? 1 : 0);
    if (sample.orientation_valid) {
      hash.floating(sample.orientation.w());
      hash.floating(sample.orientation.x());
      hash.floating(sample.orientation.y());
      hash.floating(sample.orientation.z());
    }
  }
  return hash.hex();
}

std::string flight_packet_message_payload_hash(
    const hope_msgs::msg::BallFlightPacket& packet) noexcept {
  Fnv1a64 hash;
  hash.integer(packet.schema_version);
  hash.string(packet.session_id);
  hash.string(packet.producer_instance_id);
  hash.integer(packet.trajectory_epoch);
  hash.integer(packet.flight_sequence);
  hash.string(packet.frame_id);
  hash.string(packet.segment_boundary_reason);
  hash.floating(packet.net_x);
  hash.floating(packet.post_net_delay_s);
  hash.integer(packet.segment_start_exposure_unix_ns);
  hash.integer(packet.previous_segment_last_exposure_unix_ns);
  hash.integer(packet.net_cross_exposure_unix_ns);
  hash.integer(packet.commit_exposure_unix_ns);
  hash.integer<std::uint64_t>(packet.samples.size());
  for (const auto& sample : packet.samples) {
    hash.integer(sample.exposure_unix_stamp_ns);
    hash.floating(sample.position.x);
    hash.floating(sample.position.y);
    hash.floating(sample.position.z);
    hash.integer<std::uint8_t>(sample.orientation_valid ? 1 : 0);
    if (sample.orientation_valid) {
      hash.floating(sample.orientation.w);
      hash.floating(sample.orientation.x);
      hash.floating(sample.orientation.y);
      hash.floating(sample.orientation.z);
    }
  }
  return hash.hex();
}

bool validate_flight_snapshot(
    const TrajectorySnapshot& snapshot,
    std::string& reason) noexcept {
  if (snapshot.sample_count == 0 ||
      snapshot.sample_count > kMaxEstimatorSamples) {
    reason = "invalid_sample_count";
    return false;
  }
  if (snapshot.trajectory_epoch == 0 || snapshot.snapshot_sequence == 0) {
    reason = "invalid_flight_identity";
    return false;
  }
  if (!snapshot.one_shot.commit_due ||
      !std::isfinite(snapshot.one_shot.net_cross_source_time_s) ||
      !std::isfinite(snapshot.one_shot.commit_source_time_s) ||
      snapshot.one_shot.commit_source_time_s + 1.0e-12 <
          snapshot.one_shot.net_cross_source_time_s) {
    reason = "invalid_commit_timestamps";
    return false;
  }
  double previous = -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < snapshot.sample_count; ++i) {
    const BallSample& sample = snapshot.samples[i];
    if (!std::isfinite(sample.source_time_s) ||
        !sample.position.allFinite() ||
        sample.source_time_s <= previous) {
      reason = "invalid_or_unordered_sample";
      return false;
    }
    if (sample.orientation_valid &&
        (!sample.orientation.coeffs().allFinite() ||
         sample.orientation.norm() < 0.5 ||
         sample.orientation.norm() > 1.5)) {
      reason = "invalid_orientation";
      return false;
    }
    previous = sample.source_time_s;
  }
  reason = "ok";
  return true;
}

FlightPacketDeduplicator::FlightPacketDeduplicator(std::size_t capacity)
    : capacity_(std::max<std::size_t>(1, capacity)) {}

FlightPacketDedupResult FlightPacketDeduplicator::observe(
    const std::string& identity_key,
    const std::string& payload_hash) {
  const auto found = hashes_.find(identity_key);
  if (found != hashes_.end()) {
    return found->second == payload_hash
        ? FlightPacketDedupResult::kDuplicate
        : FlightPacketDedupResult::kIdentityConflict;
  }
  if (hashes_.size() >= capacity_) {
    hashes_.erase(insertion_order_.front());
    insertion_order_.pop_front();
  }
  hashes_.emplace(identity_key, payload_hash);
  insertion_order_.push_back(identity_key);
  return FlightPacketDedupResult::kAccepted;
}

}  // namespace hope_planner_cpp
