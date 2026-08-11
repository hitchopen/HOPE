#include "hope_planner_cpp/flight_packet.hpp"

#include <gtest/gtest.h>

#include <string>

namespace hope_planner_cpp {
namespace {

TrajectorySnapshot example_snapshot() {
  TrajectorySnapshot snapshot;
  snapshot.trajectory_epoch = 7;
  snapshot.snapshot_sequence = 11;
  snapshot.segment_boundary_reason = "opponent_turnaround";
  snapshot.segment_start_source_time_s = 1000.0;
  snapshot.previous_segment_last_source_time_s = 999.5;
  snapshot.one_shot.commit_due = true;
  snapshot.one_shot.flight_sequence = 11;
  snapshot.one_shot.net_cross_source_time_s = 1000.10;
  snapshot.one_shot.commit_source_time_s = 1000.15;
  snapshot.sample_count = 3;
  for (std::size_t i = 0; i < snapshot.sample_count; ++i) {
    auto& sample = snapshot.samples[i];
    sample.source_time_s = 1000.0 + 0.01 * static_cast<double>(i);
    sample.position = Vec3(
        1.6 - 0.03 * static_cast<double>(i),
        -0.5 + 0.001 * static_cast<double>(i), 0.4);
    sample.orientation_valid = true;
    sample.orientation = Eigen::Quaterniond::Identity();
  }
  return snapshot;
}

FlightPacketMetadata example_metadata() {
  FlightPacketMetadata metadata;
  metadata.present = true;
  metadata.session_id = "session_a";
  metadata.producer_instance_id = "laptop_boot_1";
  metadata.trajectory_epoch = 7;
  metadata.flight_sequence = 11;
  metadata.frame_id = "world";
  return metadata;
}

TEST(FlightPacket, ContentHashIsStableAndCoversSamples) {
  auto snapshot = example_snapshot();
  const auto metadata = example_metadata();
  const std::string first =
      flight_packet_payload_hash(metadata, 1.37, 0.05, snapshot);
  const std::string second =
      flight_packet_payload_hash(metadata, 1.37, 0.05, snapshot);
  EXPECT_EQ(first, second);
  EXPECT_EQ(first.size(), 16U);

  snapshot.samples[1].position.y() += 0.001;
  EXPECT_NE(
      first, flight_packet_payload_hash(metadata, 1.37, 0.05, snapshot));
}

TEST(FlightPacket, RetryMetadataDoesNotChangeContentHash) {
  const auto snapshot = example_snapshot();
  auto first = example_metadata();
  auto retry = first;
  retry.transmit_index = 2;
  retry.transmit_count = 3;
  retry.publish_wall_unix_ns = 123;
  retry.receipt_wall_unix_ns = 456;
  EXPECT_EQ(
      flight_packet_payload_hash(first, 1.37, 0.05, snapshot),
      flight_packet_payload_hash(retry, 1.37, 0.05, snapshot));
}

TEST(FlightPacket, WireHashUsesIntegerExposureTimeAndIgnoresRetries) {
  hope_msgs::msg::BallFlightPacket packet;
  packet.schema_version = kBallFlightPacketSchemaVersion;
  packet.session_id = "session_a";
  packet.producer_instance_id = "laptop_boot_1";
  packet.trajectory_epoch = 7;
  packet.flight_sequence = 11;
  packet.frame_id = "world";
  packet.segment_boundary_reason = "opponent_turnaround";
  packet.net_x = 1.37;
  packet.post_net_delay_s = 0.05;
  packet.segment_start_exposure_unix_ns = 1'786'113'439'000'000'123LL;
  packet.net_cross_exposure_unix_ns = 1'786'113'439'100'000'123LL;
  packet.commit_exposure_unix_ns = 1'786'113'439'150'000'123LL;
  hope_msgs::msg::BallFlightSample sample;
  sample.exposure_unix_stamp_ns = 1'786'113'439'149'999'987LL;
  sample.position.x = 1.2;
  sample.position.y = -0.4;
  sample.position.z = 0.3;
  sample.orientation_valid = true;
  sample.orientation.w = 1.0;
  packet.samples.push_back(sample);

  const std::string content_hash =
      flight_packet_message_payload_hash(packet);
  packet.transmit_index = 2;
  packet.transmit_count = 3;
  packet.freeze_wall_unix_ns = 99;
  packet.publish_wall_unix_ns = 123;
  EXPECT_EQ(content_hash, flight_packet_message_payload_hash(packet));

  packet.samples[0].exposure_unix_stamp_ns += 1;
  EXPECT_NE(content_hash, flight_packet_message_payload_hash(packet));
}

TEST(FlightPacket, DedupeAcceptsExactlyOnePayloadPerIdentity) {
  FlightPacketDeduplicator dedupe(2);
  EXPECT_EQ(
      dedupe.observe("session/producer/1/1", "aaaa"),
      FlightPacketDedupResult::kAccepted);
  EXPECT_EQ(
      dedupe.observe("session/producer/1/1", "aaaa"),
      FlightPacketDedupResult::kDuplicate);
  EXPECT_EQ(
      dedupe.observe("session/producer/1/1", "bbbb"),
      FlightPacketDedupResult::kIdentityConflict);
  EXPECT_EQ(dedupe.size(), 1U);
}

TEST(FlightPacket, OneHundredRetriesStillProduceOneAcceptedFlight) {
  FlightPacketDeduplicator dedupe(256);
  const std::string identity =
      "model21800_session/laptop_boot_4f2a/12/237";
  std::size_t accepted = 0;
  std::size_t duplicates = 0;
  for (int receive_index = 0; receive_index < 100; ++receive_index) {
    const auto result = dedupe.observe(identity, "immutable_payload");
    accepted += result == FlightPacketDedupResult::kAccepted ? 1U : 0U;
    duplicates += result == FlightPacketDedupResult::kDuplicate ? 1U : 0U;
  }
  EXPECT_EQ(accepted, 1U);
  EXPECT_EQ(duplicates, 99U);
  EXPECT_EQ(dedupe.size(), 1U);
}

TEST(FlightPacket, ValidationRequiresOrderedFiniteImmutableWindow) {
  auto snapshot = example_snapshot();
  std::string reason;
  EXPECT_TRUE(validate_flight_snapshot(snapshot, reason));
  EXPECT_EQ(reason, "ok");
  snapshot.samples[2].source_time_s = snapshot.samples[1].source_time_s;
  EXPECT_FALSE(validate_flight_snapshot(snapshot, reason));
  EXPECT_EQ(reason, "invalid_or_unordered_sample");
}

}  // namespace
}  // namespace hope_planner_cpp
