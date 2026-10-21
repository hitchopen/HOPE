#include "hope_planner_cpp/flight_transport_v2.hpp"
#include <gtest/gtest.h>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
using namespace hope_planner_cpp;
TEST(FlightTransportV2, RV03_RV06_PacketResumeCannotReconfirmTheSameReversal) {
  for(int rate:{150,300}) for(bool known:{false,true}) {
    ProbabilisticConfig c;ShotSettings settings;settings.game_legality_advisory=true;
    FlightPipeline original(c,settings),restored(c,settings);
    constexpr std::int64_t epoch=20000000000LL;
    original.start(epoch,-1,BoundaryEvidence::RESTART,HistoryContinuity::kUnknown);
    ModelState state;state.position=Vec3(1.5,-.7,.2525);state.velocity=Vec3(-20,0,-5);
    if(known){state.spin.omega=Vec3::Zero();state.spin.mode=EffectiveSpinMode::MEASURED;}
    EventGeometry g;HitPlaneSnapshot plane;plane.calibrated=true;plane.x_hit_m=.15;plane.frame_id="table";
    bool transferred=false;int continued=0;
    for(int i=0;i<rate/5;++i) {
      BallSample sample;sample.source_time_ns=epoch+std::llround(i*1e9/rate);
      sample.position=state.position;sample.frame_id="table";sample.clock_domain="simulation";
      sample.ball_id="wire_ball";sample.calibration_id="wire";
      sample.producer_omega_valid=known;
      if(known)sample.producer_omega_rad_s=*state.spin.omega;
      original.observe(sample);
      if(transferred) {
        restored.observe(sample);++continued;
        EXPECT_EQ(restored.shot().observed_contacts(),1);
        EXPECT_EQ(restored.shot().revision(),original.shot().revision());
        EXPECT_LT((restored.checkpoint().belief.mean-original.checkpoint().belief.mean).norm(),1e-10);
        EXPECT_EQ(restored.shot().spin().omega,original.shot().spin().omega);
      } else if(original.shot().observed_contacts()==1) {
        auto prediction=original.forecast(plane,sample.source_time_ns);
        auto packet=encode_packet_v2(original,prediction,"wire_resume","producer","config");
        rclcpp::Serialization<hope_msgs::msg::FlightPacketV2> serializer;
        rclcpp::SerializedMessage bytes;serializer.serialize_message(&packet,&bytes);
        hope_msgs::msg::FlightPacketV2 received;serializer.deserialize_message(&bytes,&received);
        restore_packet_v2(restored,received,"config");transferred=true;
      }
      auto step=advance_ball(state,1./rate,c.physics,g);
      if(state.position.x()<.15||step.stopped)break;
    }
    EXPECT_TRUE(transferred);EXPECT_GE(continued,2);
  }
}
TEST(FlightTransportV2, B04_B05_ExactTimeAndAuthoritativeValidityRoundTrip) {
  for (auto ns : {std::int64_t(0), std::int64_t(1792412345678901234LL)})
    for (bool authoritative : {false, true}) {
      BallSample sample;
      sample.source_time_ns = ns;
      sample.position = Vec3(.5, -.7, .4);
      sample.clock_domain = "simulation";
      sample.producer_omega_provenance = "qualified_fixture";
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
      EXPECT_EQ(decoded.producer_omega_provenance, "qualified_fixture");
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
  packet.physics_profile = kFlightPhysicsProfile;
  packet.model_config_id = flight_model_config_id({}, {});
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
  EXPECT_TRUE(restored.shot().export_snapshot().boundary_qualified);
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

TEST(FlightTransportV2,
     A12_B12_T03_X07_ForecastOnlyPosteriorSpinGeometryAndEventCovariance) {
  FlightPipeline pipeline({});
  pipeline.start(0, -1, BoundaryEvidence::RACKET_CONTACT,
                 HistoryContinuity::kComplete);
  for (int i = 0; i < 12; ++i) {
    BallSample s;
    s.source_time_ns = i * 5000000LL;
    s.position = Vec3(2. - .02 * i, -.7, .5 + .002 * i - .000122625 * i * i);
    s.frame_id = "table";
    s.clock_domain = "simulation";
    s.producer_omega_valid = i == 0;
    s.producer_omega_rad_s = Vec3(0, 30, 0);
    pipeline.observe(s);
  }
  HitPlaneSnapshot plane;
  plane.calibrated = true;
  plane.x_hit_m = .15;
  plane.frame_id = "table";
  plane.registration_id = "fixture";
  auto prediction = pipeline.forecast(plane, 55000000);
  auto wire = encode_prediction(prediction, pipeline.shot());
  EXPECT_EQ(wire.schema_version, 3);
  EXPECT_EQ(wire.shot_id, wire.shot_generation);
  ASSERT_TRUE(wire.current_state_available);
  EXPECT_EQ(wire.current_state.predicted_arrival_time_ns, 55000000);
  EXPECT_EQ(wire.current_state.arrival_dt_ns, 0);
  EXPECT_TRUE(wire.current_state.spin_available);
  EXPECT_FALSE(wire.current_state.spin_measured_valid);
  EXPECT_EQ(wire.current_state.spin_measurement_time_ns, 0);
  EXPECT_FALSE(wire.current_state.spin_provenance.empty());
  EXPECT_EQ(wire.plane_point.x, .15);
  EXPECT_EQ(wire.plane_normal.x, 1.);
  EXPECT_EQ(wire.plane_registration_id, "fixture");
  ASSERT_FALSE(wire.waypoints.empty());
  for (const auto &point : wire.waypoints) {
    EXPECT_EQ(point.predicted_arrival_time_ns,
              point.prediction_reference_time_ns + point.arrival_dt_ns);
    EXPECT_FALSE(point.spin_measured_valid);
    if (point.spin_available)
      EXPECT_EQ(point.spin_measurement_time_ns, 0);
  }
  for (const auto &event : wire.events) {
    EXPECT_EQ(event.event_time_ns,
              event.prediction_reference_time_ns + event.arrival_dt_ns);
    if (event.pre_spin_available)
      EXPECT_EQ(event.pre_spin_measurement_time_ns, 0);
  }
  if (prediction.target.event && !prediction.target.historical)
    EXPECT_NE(wire.target_linearization_quality, "unavailable");
}
TEST(FlightTransportV2,
     X06_T06_DuplicatePacketsAndProducerRestartCannotRestoreRetiredState) {
  FlightPipeline original({});
  original.start(0, -1, BoundaryEvidence::RACKET_CONTACT,
                 HistoryContinuity::kComplete);
  for (int i = 0; i < 8; ++i) {
    BallSample s;
    s.source_time_ns = i * 5000000LL;
    s.position = Vec3(2. - .02 * i, -.7, .5);
    s.clock_domain = "simulation";
    original.observe(s);
  }
  HitPlaneSnapshot plane;
  plane.calibrated = true;
  plane.x_hit_m = .15;
  auto prediction = original.forecast(plane, 35000000);
  auto packet =
      encode_packet_v2(original, prediction, "session", "producer", "config");
  FlightPacketAdmissionV2 receiver("session");
  std::string reason;
  EXPECT_TRUE(receiver.admit(packet, "config", reason));
  EXPECT_FALSE(receiver.admit(packet, "config", reason));
  EXPECT_EQ(reason, "duplicate_packet");
  auto changed = packet;
  changed.producer_id = "restart";
  changed.payload_sha256 = packet_v2_hash(changed);
  EXPECT_FALSE(receiver.admit(changed, "config", reason));
  EXPECT_EQ(reason, "producer_restart_requires_session_reset");
  changed = packet;
  changed.context_revision++;
  changed.payload_sha256 = packet_v2_hash(changed);
  EXPECT_TRUE(receiver.admit(changed, "config", reason));
  EXPECT_FALSE(receiver.admit(packet, "config", reason));
  EXPECT_EQ(reason, "retired_packet");
}
