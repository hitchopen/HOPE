#include "hope_planner_cpp/flight_state_v2.hpp"
#include "hope_planner_cpp/integrated_command_lifecycle.hpp"
#include "hope_planner_cpp/return_eligibility.hpp"
#include <gtest/gtest.h>
#include <set>
using namespace hope_planner_cpp;
namespace {
constexpr std::int64_t epoch = 1800000000000000000LL;
ShotSettings settings() {
  ShotSettings s;
  s.game_legality_advisory = true;
  s.automatic_rearm = true;
  return s;
}
BallSample sample(double t, const ModelState &s) {
  BallSample b;
  b.source_time_ns = epoch + std::llround(t * 1e9);
  b.source_time_s = t;
  b.position = s.position;
  b.frame_id = "table";
  b.calibration_id = "survey";
  b.ball_id = "ball";
  b.clock_domain = "simulation";
  b.clock_mapping_id = "mapped";
  return b;
}
ModelState initial(double speed = 5., int direction = -1) {
  ModelState s;
  s.position = Vec3(direction < 0 ? 2.5 : .24, -.7, .4);
  s.velocity = Vec3(direction * speed, 0, -.5);
  return s;
}
void feed(FlightPipeline &p, ModelState &s, double begin, double span, int hz,
          bool quaternion = false, double omega = 0) {
  for (int i = 0; i <= int(span * hz); ++i) {
    auto b = sample(begin + i * 1. / hz, s);
    if (quaternion) {
      b.orientation_valid = b.orientation_authoritative = true;
      b.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(
          omega * double(b.source_time_ns - epoch - std::llround(begin * 1e9)) *
              1e-9,
          Vec3::UnitZ()));
    }
    p.observe(b);
    integrate_ball(s, 1. / hz, p.config().physics);
  }
}
void reject(FlightPipeline &p, double t) {
  ModelState bad;
  bad.position = Vec3(1, -.7, -.2);
  p.observe(sample(t, bad));
  ASSERT_FALSE(p.shot().active());
  ASSERT_EQ(p.shot().reason(), "observed_under_table");
}
TEST(Reacquisition, RA01_AlreadyRejectedThenGapNeedsMultipleNewSamples) {
  for (int hz : {150, 300}) {
    FlightPipeline p({}, settings());
    auto a = initial();
    feed(p, a, 0, .08, hz);
    ASSERT_TRUE(p.shot().active());
    auto generation = p.shot().generation();
    reject(p, .1);
    auto retired = p.shot().generation();
    auto b = initial();
    p.observe(sample(1, b));
    EXPECT_FALSE(p.shot().active());
    EXPECT_EQ(p.shot().generation(), retired);
    integrate_ball(b, 1. / hz, {});
    feed(p, b, 1 + 1. / hz, .08, hz);
    ASSERT_TRUE(p.shot().active()) << p.shot().reason();
    EXPECT_EQ(p.shot().generation(), retired + 1);
    EXPECT_GT(p.shot().generation(), generation);
    EXPECT_EQ(p.shot().observed_contacts(), 0);
    EXPECT_EQ(p.shot().continuity(), HistoryContinuity::kUnknown);
    EXPECT_GE(p.shot().samples().front().source_time_ns, epoch + 1000000000LL);
    EXPECT_EQ(p.source_origin_ns(), p.shot().samples().front().source_time_ns);
  }
}
TEST(Reacquisition, RA02_QuietHeldReleaseWithoutSourceGap) {
  for (int hz : {150, 300}) {
    FlightPipeline p({}, settings());
    auto a = initial();
    feed(p, a, 0, .08, hz);
    ASSERT_TRUE(p.shot().active());
    auto old = p.shot().generation();
    ModelState held;
    held.position = Vec3(2.5, -.7, .4);
    for (int i = 0; i < int(.7 * hz); ++i)
      p.observe(sample(.09 + i * 1. / hz, held));
    ASSERT_FALSE(p.shot().active());
    auto b = initial();
    feed(p, b, .8, .08, hz);
    ASSERT_TRUE(p.shot().active()) << p.shot().reason();
    EXPECT_GT(p.shot().generation(), old);
    EXPECT_GE(p.shot().samples().front().source_time_ns, epoch + 800000000LL);
  }
}
TEST(Reacquisition,
     RA03_ContinuousRejectedMotionAndDirectionChangeStayQuarantined) {
  FlightPipeline p({}, settings());
  auto a = initial();
  feed(p, a, 0, .08, 300);
  ASSERT_TRUE(p.shot().active());
  p.invalidate("observed_net_collision");
  auto retired = p.shot().generation();
  feed(p, a, .09, .20, 300);
  EXPECT_FALSE(p.shot().active());
  a.velocity.x() *= -1;
  feed(p, a, .30, .20, 300);
  EXPECT_FALSE(p.shot().active());
  EXPECT_EQ(p.shot().generation(), retired);
  // Rearm is an explicit profile choice; service-armed input still needs its
  // explicit boundary even if automatic rearm is requested.
  FlightPipeline standalone;
  a = initial();
  feed(standalone, a, 0, .08, 300);
  ASSERT_TRUE(standalone.shot().active());
  reject(standalone, .1);
  a = initial();
  feed(standalone, a, 1, .1, 300);
  EXPECT_FALSE(standalone.shot().active());
  auto armed_settings = settings();
  armed_settings.service_armed = true;
  FlightPipeline armed({}, armed_settings);
  a = initial();
  feed(armed, a, 0, .1, 300);
  EXPECT_FALSE(armed.shot().active());
}
TEST(Reacquisition, RA04_ProvenanceAndClockFaultCannotAuthorizeNewEpoch) {
  for (std::string fault : {"frame", "calibration", "mapping", "ball", "clock",
                            "duplicate", "reorder"}) {
    FlightPipeline p({}, settings());
    auto a = initial();
    feed(p, a, 0, .08, 300);
    reject(p, .1);
    auto b = initial();
    for (int i = 0; i < 30; ++i) {
      auto s = sample(1 + i / 300., b);
      if (fault == "frame")
        s.frame_id = "wrong";
      if (fault == "calibration")
        s.calibration_id = "wrong";
      if (fault == "mapping")
        s.clock_mapping_id = "wrong";
      if (fault == "ball")
        s.ball_id = "wrong";
      if (fault == "clock")
        s.clock_mapping_valid = false;
      if (fault == "duplicate")
        s.source_time_ns = epoch + 50000000;
      if (fault == "reorder")
        s.source_time_ns = epoch + 70000000 - i;
      p.observe(s);
      integrate_ball(b, 1. / 300, {});
    }
    EXPECT_FALSE(p.shot().active()) << fault;
    b = initial();
    feed(p, b, 2, .09, 300);
    EXPECT_TRUE(p.shot().active()) << fault << ":" << p.shot().reason();
  }
}
TEST(Reacquisition, RA05_NewPosteriorMatchesFreshOwnerWithNoOldSpinOrContacts) {
  FlightPipeline p({}, settings()), fresh({}, settings());
  auto a = initial();
  feed(p, a, 0, .08, 300);
  ASSERT_TRUE(p.shot().active());
  auto latest = p.shot().samples().back();
  latest.source_time_ns += 1000000;
  latest.source_time_s += .001;
  latest.producer_omega_valid = true;
  latest.producer_omega_rad_s = Vec3(0, 150, 0);
  p.observe(latest);
  ModelState contact;
  contact.position = Vec3(1, -.7, .02);
  contact.velocity = Vec3(-5, 0, -2);
  ASSERT_TRUE(p.shot().confirm(PhysicalEventKind::TABLE, latest.source_time_ns,
                               latest.source_time_ns, contact,
                               "qualified_fixture_contact"));
  ASSERT_TRUE(p.shot().spin().usable());
  ASSERT_EQ(p.shot().observed_contacts(), 1);
  reject(p, .1);
  auto b = initial(), c = b;
  feed(p, b, 1, .09, 300);
  feed(fresh, c, 1, .09, 300);
  ASSERT_TRUE(p.shot().active());
  auto x = p.checkpoint(), y = fresh.checkpoint();
  EXPECT_TRUE(x.belief.mean.isApprox(y.belief.mean, 1e-12));
  EXPECT_TRUE(x.belief.covariance.isApprox(y.belief.covariance, 1e-12));
  EXPECT_FALSE(p.shot().spin().usable());
  EXPECT_EQ(p.shot().observed_contacts(), 0);
  EXPECT_EQ(p.source_origin_ns(), fresh.source_origin_ns());
}
TEST(Reacquisition, RA06_RetiredSnapshotCannotPublishIntoNewGeneration) {
  FlightPipeline p({}, settings());
  auto a = initial();
  feed(p, a, 0, .08, 300);
  auto token = p.shot().snapshot();
  reject(p, .1);
  auto b = initial();
  feed(p, b, 1, .08, 300);
  ASSERT_TRUE(p.shot().active());
  EXPECT_FALSE(p.accept(token, p.shot().samples().back().source_time_ns));
  auto current = p.shot().snapshot();
  EXPECT_TRUE(p.accept(current, current.source_ns));
  EXPECT_FALSE(p.accept(token, current.source_ns));
}
TEST(Reacquisition, RA07_TenRejectRearmCyclesRemainBounded) {
  FlightPipeline p({}, settings());
  std::set<std::uint64_t> generations;
  for (int k = 0; k < 10; ++k) {
    auto a = initial();
    feed(p, a, k * 1., .08, 300);
    ASSERT_TRUE(p.shot().active()) << k << ":" << p.shot().reason();
    EXPECT_TRUE(generations.insert(p.shot().generation()).second);
    reject(p, k + .1);
    ModelState held;
    held.position = Vec3(1, -.7, .4);
    for (int i = 0; i < 60; ++i) {
      auto s = sample(k + .11 + i / 300., held);
      s.position.x() += .0001 * std::sin(i);
      p.observe(s);
    }
    EXPECT_FALSE(p.shot().active());
    EXPECT_LE(p.shot().candidates().size(), p.settings().candidate_capacity);
  }
}
TEST(Reacquisition,
     RA08_BothRatesDirectionsSpeedsAndJitterHavePhysicalForecasts) {
  for (int hz : {150, 300})
    for (int direction : {-1, 1})
      for (double speed : {5., 10., 12.}) {
        FlightPipeline p({}, settings());
        auto a = initial();
        feed(p, a, 0, .08, hz);
        reject(p, .1);
        auto b = initial(speed, direction);
        double t = 1;
        bool forecasted = false;
        HitPlaneSnapshot plane;
        plane.calibrated = true;
        plane.x_hit_m = .15;
        plane.frame_id = "table";
        plane.registration_id = "survey";
        for (int i = 0; i < hz / 10; ++i) {
          auto s = sample(t, b);
          p.observe(s);
          if (p.shot().active())
            forecasted |=
                p.forecast(plane, s.source_time_ns).physical_available;
          double dt = (i % 2 ? .95 : 1.05) / hz;
          integrate_ball(b, dt, {});
          t += dt;
        }
        EXPECT_TRUE(p.shot().active())
            << hz << ":" << direction << ":" << speed;
        EXPECT_TRUE(forecasted);
      }
}
TEST(SpinReadiness, SP01_FullPositionOnlyShotNeverClaimsMeasuredZero) {
  FlightPipeline p({}, settings());
  auto a = initial();
  feed(p, a, 0, .1, 300);
  ASSERT_TRUE(p.shot().active());
  EXPECT_FALSE(p.shot().spin().usable());
  EXPECT_EQ(p.shot().spin_readiness().state, "UNAVAILABLE");
}
TEST(SpinReadiness, SP02_QuaternionOnlyCollectsBeforeMeasuredAtBothRates) {
  for (int hz : {150, 300}) {
    FlightPipeline p({}, settings());
    auto a = initial();
    bool collecting = false, measured = false;
    for (int i = 0; i < int(.10 * hz); ++i) {
      auto s = sample(i * 1. / hz, a);
      s.orientation_valid = s.orientation_authoritative = true;
      s.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(
          20 * double(s.source_time_ns - epoch) * 1e-9, Vec3::UnitZ()));
      p.observe(s);
      auto r = p.shot().spin_readiness();
      collecting |= r.state == "COLLECTING";
      if (p.shot().spin().usable()) {
        measured = true;
        EXPECT_NEAR(p.shot().spin().omega->z(), 20, 1e-8);
        EXPECT_GE(r.span_s + 1e-12, .05);
        EXPECT_LE(p.shot().spin().measurement_time_ns, s.source_time_ns);
      }
      integrate_ball(a, 1. / hz, {});
    }
    EXPECT_TRUE(collecting);
    EXPECT_TRUE(measured);
  }
}
TEST(SpinReadiness, SP03_MagnusAcquisitionActuallyUsesCausalQuaternionSpin) {
  for (int hz : {150, 300}) {
    FlightPipeline p({}, settings());
    auto a = initial(30);
    a.position.z() = 1.5;
    a.spin.omega = Vec3(0, 0, 100);
    a.spin.mode = EffectiveSpinMode::MEASURED;
    feed(p, a, 0, .085, hz, true, 100);
    ASSERT_TRUE(p.shot().active()) << p.shot().reason();
    const auto &used = p.shot().acquisition_spin();
    ASSERT_TRUE(used.usable());
    EXPECT_NEAR(used.omega->z(), 100, 1e-7);
    EXPECT_LE(used.measurement_time_ns, p.shot().acquisition_spin_cutoff_ns());
    EXPECT_GE(p.shot().acquisition_spin_cutoff_ns(), epoch + 50000000);
  }
}
TEST(SpinReadiness,
     SP04_PositionOnlyStartsPromptlyThenSameGenerationAdoptsSpin) {
  for (int hz : {150, 300}) {
    FlightPipeline p({}, settings());
    auto a = initial();
    std::uint64_t gen = 0;
    bool early = false, late = false;
    for (int i = 0; i < int(.11 * hz); ++i) {
      auto s = sample(i * 1. / hz, a);
      s.orientation_valid = s.orientation_authoritative = true;
      s.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(
          10 * double(s.source_time_ns - epoch) * 1e-9, Vec3::UnitY()));
      p.observe(s);
      if (p.shot().active()) {
        if (!gen)
          gen = p.shot().generation();
        EXPECT_EQ(p.shot().generation(), gen);
        early |= i * 1. / hz < .05 && !p.shot().spin().usable();
        late |= p.shot().spin().usable();
      }
      integrate_ball(a, 1. / hz, {});
    }
    EXPECT_TRUE(early);
    EXPECT_TRUE(late);
  }
}
TEST(SpinReadiness, SP05_HeldOrientationExcludedFromLaunchAngularWindow) {
  for (int hz : {150, 300}) {
    FlightPipeline p({}, settings());
    ModelState held;
    held.position = Vec3(2.5, -.7, .4);
    for (int i = 0; i < int(.3 * hz); ++i) {
      auto s = sample(i * 1. / hz, held);
      s.orientation_valid = s.orientation_authoritative = true;
      s.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(
          100 * double(s.source_time_ns - epoch) * 1e-9, Vec3::UnitZ()));
      p.observe(s);
    }
    auto a = initial();
    feed(p, a, .3, .10, hz, true, 20);
    ASSERT_TRUE(p.shot().active());
    ASSERT_TRUE(p.shot().spin().usable());
    EXPECT_NEAR(p.shot().spin().omega->z(), 20, 1e-7);
    EXPECT_GE(p.shot().samples().front().source_time_ns, epoch + 300000000);
  }
}
TEST(SpinReadiness, SP06_UntrustedInvalidReorderedAndSignAliases) {
  for (bool authoritative : {false, true}) {
    TrustedSpin spin;
    ModelState a;
    for (int i = 0; i < 30; ++i) {
      auto s = sample(i / 300., a);
      s.orientation_valid = true;
      s.orientation_authoritative = authoritative;
      s.orientation =
          Eigen::Quaterniond(Eigen::AngleAxisd(20 * i / 300., Vec3::UnitY()));
      if (i % 2)
        s.orientation.coeffs() *= -1;
      spin.observe(s);
      spin.observe(s);
    }
    EXPECT_EQ(spin.value().usable(), authoritative);
    if (authoritative) {
      EXPECT_NEAR(spin.value().omega->y(), 20, 1e-8);
    }
    auto invalid = sample(.2, a);
    invalid.orientation_valid = invalid.orientation_authoritative = true;
    invalid.orientation.coeffs().setZero();
    spin.observe(invalid);
    EXPECT_NE(spin.value().mode, EffectiveSpinMode::MEASURED);
  }
}
TEST(SpinReadiness, SP07_ValidZeroAndMissingRemainDistinct) {
  for (bool producer : {false, true}) {
    TrustedSpin spin;
    ModelState a;
    for (int i = 0; i < 30; ++i) {
      auto s = sample(i / 300., a);
      s.orientation_valid = s.orientation_authoritative = !producer;
      s.orientation = Eigen::Quaterniond::Identity();
      s.producer_omega_valid = producer;
      s.producer_omega_rad_s.setZero();
      spin.observe(s);
    }
    ASSERT_TRUE(spin.value().usable());
    EXPECT_EQ(spin.readiness().state, "READY_MEASURED");
    EXPECT_EQ(spin.value().omega->norm(), 0);
  }
}
TEST(SpinReadiness, SP08_DelayedBounceDropoutAndPostOnlyRecovery) {
  TrustedSpin spin;
  ModelState a;
  for (int i = 0; i <= 42; ++i) {
    double t = i / 300.;
    auto s = sample(t, a);
    s.orientation_valid = s.orientation_authoritative = true;
    s.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(
        10 * std::min(t, .12) + 54.2 * std::max(0., t - .12), Vec3::UnitY()));
    spin.observe(s);
  }
  ASSERT_TRUE(spin.contact(1, epoch + 120000000, Vec3(3, 0, -2), {},
                           epoch + 123333333));
  EXPECT_NE(spin.value().mode, EffectiveSpinMode::MEASURED);
  for (int i = 43; i < 65; ++i) {
    double t = i / 300.;
    auto s = sample(t, a);
    s.orientation_valid = s.orientation_authoritative = i >= 46;
    s.orientation = Eigen::Quaterniond(
        Eigen::AngleAxisd(1.2 + 54.2 * (t - .12), Vec3::UnitY()));
    spin.observe(s);
  }
  ASSERT_TRUE(spin.value().usable());
  EXPECT_EQ(spin.value().mode, EffectiveSpinMode::MEASURED);
  EXPECT_NEAR(spin.value().omega->y(), 54.2, 1e-7);
  EXPECT_FALSE(spin.contact(1, epoch + 120000000, Vec3(3, 0, -2), {}));
}
TEST(SpinReadiness, SP09_BoundedAttemptsAndUnsupportedUnknownSpin) {
  for (int hz : {150, 300}) {
    FlightPipeline p({}, settings());
    for (int i = 0; i < 1500; ++i) {
      ModelState s;
      s.position =
          Vec3(1 + i * 1. / hz, -.7, 1 + 20 * std::pow(i * 1. / hz, 2));
      auto b = sample(i * 1. / hz, s);
      p.observe(b);
      EXPECT_LE(p.shot().candidates().size(), p.settings().candidate_capacity);
    }
    EXPECT_FALSE(p.shot().active());
    EXPECT_FALSE(p.shot().spin().usable());
  }
}
} // namespace
