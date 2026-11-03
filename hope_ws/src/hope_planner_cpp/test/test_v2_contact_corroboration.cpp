#include "hope_planner_cpp/flight_state_v2.hpp"

#include <gtest/gtest.h>

using namespace hope_planner_cpp;
namespace {
constexpr std::int64_t origin = 1800000000000000000LL;
BallSample observation(double t, double z) {
  BallSample sample;
  sample.source_time_ns = origin + std::llround(t * 1e9);
  sample.position = Vec3(1.5 - 2 * t, -.7, z);
  sample.frame_id = "table";
  sample.calibration_id = "contact-regression";
  sample.clock_domain = "simulation";
  sample.clock_mapping_id = "source-clock";
  sample.ball_id = "ball";
  return sample;
}
ShotSettings settings() {
  ShotSettings result;
  result.game_legality_advisory = true;
  return result;
}

TEST(ContactCorroboration, RejectedIsolatedHeightOutlierCannotConfirmBounce) {
  for (int rate : {150, 300, 360}) for (double vz : {-1., 1.}) {
    SCOPED_TRACE(::testing::Message() << rate << ":" << vz);
    ProbabilisticConfig config;
    config.physics.drag_k = 0;
    FlightPipeline pipeline(config, settings());
    pipeline.start(origin, -1, BoundaryEvidence::RESTART, HistoryContinuity::kUnknown);
    bool outlier_rejected = false;
    for (int i = 0; i < 35; ++i) {
      const double t = double(i) / rate;
      const double z = i == 20 ? config.physics.radius : .6 + vz*t - 4.905*t*t;
      const bool accepted = pipeline.observe(observation(t, z));
      if (i == 20) outlier_rejected = !accepted;
      EXPECT_EQ(pipeline.shot().observed_contacts(), 0) << i;
      ASSERT_TRUE(pipeline.shot().active()) << pipeline.shot().reason();
    }
    EXPECT_TRUE(outlier_rejected);
  }
}

TEST(ContactCorroboration, GenuineBounceSurvivesLargeInnovationAndOverlappingWindows) {
  for (int rate : {150, 300, 360}) {
    SCOPED_TRACE(rate);
    ProbabilisticConfig config;
    config.physics.drag_k = 0;
    // Independent analytic impact has upward speed 3 m/s. Deliberately use a
    // mismatched filter restitution so genuine post-contact evidence is gated.
    config.physics.restitution_v = .15;
    config.innovation_gate_squared = 9;
    config.reacquire_rejections = 100;
    FlightPipeline pipeline(config, settings());
    pipeline.start(origin, -1, BoundaryEvidence::RESTART, HistoryContinuity::kUnknown);
    bool rejected_after_contact = false;
    std::optional<std::uint64_t> contact_id;
    for (int i = 0; i <= int(.15 * rate); ++i) {
      const double t = double(i) / rate;
      const double dt = t - .1;
      const double z = .02 + (dt <= 0 ? -4*dt : 3*dt) - 4.905*dt*dt;
      const bool accepted = pipeline.observe(observation(t, z));
      if (dt > 0 && dt < .025 && !accepted) rejected_after_contact = true;
      ASSERT_TRUE(pipeline.shot().active()) << pipeline.shot().reason();
      EXPECT_LE(pipeline.shot().observed_contacts(), 1);
      for (const auto& event : pipeline.shot().events()) {
        if (event.kind != PhysicalEventKind::TABLE || event.status != EventStatus::CONFIRMED) continue;
        if (contact_id) EXPECT_EQ(event.id, *contact_id);
        contact_id = event.id;
      }
    }
    EXPECT_TRUE(rejected_after_contact);
    EXPECT_EQ(pipeline.shot().observed_contacts(), 1);
    EXPECT_TRUE(contact_id.has_value());
  }
}

TEST(ContactCorroboration, RestoreBeforeOutlierDoesNotInventContact) {
  ProbabilisticConfig config;
  config.physics.drag_k = 0;
  FlightPipeline pipeline(config, settings()), restored(config, settings());
  pipeline.start(origin, -1, BoundaryEvidence::RESTART, HistoryContinuity::kUnknown);
  for (int i = 0; i < 20; ++i) {
    const double t = i / 300.;
    pipeline.observe(observation(t, .6 - t - 4.905*t*t));
  }
  restored.restore(pipeline.shot().export_snapshot(), pipeline.checkpoint(), pipeline.source_origin_ns());
  for (int i = 20; i < 30; ++i) {
    const double t = i / 300.;
    const auto sample = observation(t, i == 20 ? .02 : .6 - t - 4.905*t*t);
    pipeline.observe(sample);
    restored.observe(sample);
    EXPECT_EQ(pipeline.shot().observed_contacts(), 0);
    EXPECT_EQ(restored.shot().observed_contacts(), 0);
    EXPECT_EQ(pipeline.shot().revision(), restored.shot().revision());
  }
}
}  // namespace
