#include "hope_planner_cpp/flight_transport_v2.hpp"
#include <gtest/gtest.h>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
using namespace hope_planner_cpp;
TEST(FlightTransportV2, B04_B05_ExactTimeAndAuthoritativeValidityRoundTrip) {
  for (auto ns : {std::int64_t(0), std::int64_t(1792412345678901234LL)})
    for (bool authoritative : {false, true}) {
      BallSample sample;
      sample.source_time_ns = ns;
      sample.position = Vec3(.5, -.7, .4);
      sample.clock_domain = "simulation";
      sample.orientation_valid = true;
      sample.orientation_authoritative = authoritative;
      auto encoded = encode_observation(sample);
      rclcpp::Serialization<hope_msgs::msg::BallObservationV2> serializer;
      rclcpp::SerializedMessage bytes;
      serializer.serialize_message(&encoded, &bytes);
      hope_msgs::msg::BallObservationV2 received;
      serializer.deserialize_message(&bytes, &received);
      auto decoded = decode_observation(received);
      EXPECT_EQ(decoded.source_time_ns, ns);
      EXPECT_TRUE(decoded.source_time_valid);
      EXPECT_TRUE(decoded.position_valid);
      EXPECT_EQ(decoded.position, sample.position);
      EXPECT_EQ(decoded.orientation_valid, authoritative);
      EXPECT_EQ(decoded.orientation_authoritative, authoritative);
    }
}
TEST(FlightTransportV2,
     T01_B07_PacketHashCoversValidityCheckpointAndContactLedger) {
  hope_msgs::msg::FlightPacketV2 packet;
  packet.session_id = "session";
  packet.producer_id = "producer";
  packet.shot_generation = 2;
  packet.config_id = "profile";
  BallSample sample;
  sample.clock_domain = "simulation";
  sample.frame_id = "table";
  sample.position = Vec3(.5, -.7, .4);
  packet.observations.push_back(encode_observation(sample));
  sample.source_time_ns = 3333333;
  packet.observations.push_back(encode_observation(sample));
  packet.payload_sha256 = packet_v2_hash(packet);
  std::string reason;
  ASSERT_TRUE(validate_packet_v2(packet, reason)) << reason;
  auto tampered = packet;
  tampered.observations[0].orientation_authoritative = true;
  EXPECT_FALSE(validate_packet_v2(tampered, reason));
  EXPECT_EQ(reason, "packet_hash");
  tampered = packet;
  tampered.checkpoint_last_contact_id++;
  EXPECT_FALSE(validate_packet_v2(tampered, reason));
  tampered = packet;
  tampered.observations[1].source_time_ns = 0;
  tampered.payload_sha256 = packet_v2_hash(tampered);
  EXPECT_FALSE(validate_packet_v2(tampered, reason));
}
TEST(FlightTransportV2,
     A12_T03_TargetAndEveryWaypointHaveExactArrivalEquation) {
  FlightPipeline pipeline({});
  pipeline.start(20000000000LL, -1, BoundaryEvidence::RACKET_CONTACT,
                 HistoryContinuity::kComplete);
  for (int i = 0; i < 20; ++i) {
    BallSample sample;
    sample.source_time_ns = 20000000000LL + i * 3333333LL;
    sample.position = Vec3(2.5 - .02 * i, -.7, .5 - .001 * i);
    pipeline.observe(sample);
  }
  HitPlaneSnapshot plane;
  plane.calibrated = true;
  plane.x_hit_m = .15;
  auto result = pipeline.forecast(plane, 20063333327LL);
  auto encoded = encode_prediction(result, pipeline.shot());
  EXPECT_TRUE(encoded.physical_available);
  for (const auto &p : encoded.waypoints)
    EXPECT_EQ(p.predicted_arrival_time_ns,
              p.prediction_reference_time_ns + p.arrival_dt_ns);
  if (result.target.event)
    EXPECT_EQ(encoded.predicted_arrival_time_ns,
              encoded.prediction_reference_time_ns + encoded.arrival_dt_ns);
  EXPECT_EQ(encoded.forecast_route, name(result.route));
  EXPECT_FALSE(encoded.target_historical);
}
TEST(FlightTransportV2,
     B07_T06_TruncatedSnapshotReproducesPosteriorWithoutRefittingPastSpin) {
  ProbabilisticConfig config;
  FlightPipeline original(config);
  original.start(0, 1, BoundaryEvidence::RACKET_CONTACT,
                 HistoryContinuity::kComplete);
  for (int i = 0; i < 80; ++i) {
    const double t = i / 300.;
    BallSample sample;
    sample.source_time_ns = std::llround(t * 1e9);
    sample.frame_id = "table";
    sample.clock_domain = "simulation";
    sample.position = Vec3(.2 + 3 * t, -.7, .8 + .5 * t - 4.905 * t * t);
    if (i == 10) {
      sample.producer_omega_valid = true;
      sample.producer_omega_rad_s = Vec3(0, 30, 0);
    }
    original.observe(sample);
  }
  HitPlaneSnapshot plane;
  plane.calibrated = true;
  plane.x_hit_m = .15;
  auto forecast =
      original.forecast(plane, original.shot().samples().back().source_time_ns);
  auto packet =
      encode_packet_v2(original, forecast, "session", "producer", "config");
  // Keep only a suffix, preserving the posterior, ledger and trusted cache.
  packet.observations.erase(packet.observations.begin(),
                            packet.observations.end() - 10);
  packet.payload_sha256 = packet_v2_hash(packet);
  FlightPipeline received(config);
  ASSERT_NO_THROW(restore_packet_v2(received, packet, "config"));
  EXPECT_EQ(received.shot().generation(), original.shot().generation());
  EXPECT_EQ(received.shot().spin().mode, EffectiveSpinMode::HELD);
  ASSERT_TRUE(received.shot().spin().usable());
  EXPECT_EQ(*received.shot().spin().omega, Vec3(0, 30, 0));
  EXPECT_LT(
      (received.checkpoint().belief.mean - original.checkpoint().belief.mean)
          .norm(),
      1e-12);
  auto restored = received.forecast(plane, forecast.reference_ns);
  ASSERT_EQ(restored.nominal.waypoints.size(),
            forecast.nominal.waypoints.size());
  for (std::size_t i = 0; i < restored.nominal.waypoints.size(); ++i)
    EXPECT_LT((restored.nominal.waypoints[i].state -
               forecast.nominal.waypoints[i].state)
                  .norm(),
              1e-12);
  EXPECT_THROW(restore_packet_v2(received, packet, "other_config"),
               std::invalid_argument);
}
TEST(FlightTransportV2,
     X09_CheckpointRejectsDifferentNetPhysicsEvenWithACallerSuppliedTag) {
  ProbabilisticConfig config;
  FlightPipeline original(config);
  original.start(0, 1, BoundaryEvidence::RACKET_CONTACT,
                 HistoryContinuity::kComplete);
  for (int i = 0; i < 8; ++i) {
    BallSample sample;
    sample.source_time_ns = i * 5000000LL;
    sample.frame_id = "table";
    sample.position = Vec3(.3 + .01 * i, -.7, .8 - .001 * i);
    original.observe(sample);
  }
  HitPlaneSnapshot plane;
  plane.calibrated = true;
  plane.x_hit_m = .15;
  auto forecast = original.forecast(plane, 35000000);
  auto packet =
      encode_packet_v2(original, forecast, "session", "producer", "tag");
  config.table.net_height += .01;
  FlightPipeline other(config);
  EXPECT_THROW(restore_packet_v2(other, packet, "tag"), std::invalid_argument);
}
TEST(FlightTransportV2,
     C04_T06_AuthoritativeContextProvenanceSurvivesTransfer) {
  FlightPipeline original({});
  original.start(0, -1, BoundaryEvidence::RACKET_CONTACT,
                 HistoryContinuity::kUnknown);
  for (int i = 0; i < 8; ++i) {
    BallSample s;
    s.source_time_ns = i * 5000000LL;
    s.position = Vec3(2. - .02 * i, -.7, .8 - .001 * i);
    s.frame_id = "table";
    s.clock_domain = "simulation";
    original.observe(s);
  }
  ASSERT_TRUE(original.shot().context(MatchContext::NORMAL_RALLY, 30000000,
                                      "independent_referee",
                                      original.shot().generation()));
  EXPECT_FALSE(original.shot().route_supported());
  original.shot().set_history_continuity(HistoryContinuity::kComplete);
  HitPlaneSnapshot plane;
  plane.calibrated = true;
  plane.x_hit_m = .15;
  auto prediction = original.forecast(plane, 35000000);
  auto packet =
      encode_packet_v2(original, prediction, "session", "producer", "profile");
  FlightPipeline restored({});
  ASSERT_NO_THROW(restore_packet_v2(restored, packet, "profile"));
  EXPECT_EQ(restored.shot().context_source(), "independent_referee");
  EXPECT_EQ(restored.shot().context_time_ns(), 30000000);
  EXPECT_TRUE(restored.shot().route_supported());
  EXPECT_FALSE(restored.shot().context(MatchContext::SERVE, 35000000,
                                       "other_source",
                                       restored.shot().generation()));
  const auto wire = encode_prediction(prediction, original.shot());
  EXPECT_EQ(wire.context_source, "independent_referee");
  EXPECT_EQ(wire.context_time_ns, 30000000);
}
