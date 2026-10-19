#include "hope_planner_cpp/flight_state_v2.hpp"
#include "hope_planner_cpp/future_contact_predictor.hpp"
#include "hope_planner_cpp/racket_target_planner.hpp"
#include <Eigen/Eigenvalues>
#include <future>
#include <gtest/gtest.h>
#include <thread>
using namespace hope_planner_cpp;
namespace {
BallSample sample(std::int64_t ns, Vec3 p = Vec3(.3, -.7, .5)) {
  BallSample s;
  s.source_time_ns = ns;
  s.source_time_s = ns * 1e-9;
  s.position = p;
  s.frame_id = "table";
  s.clock_domain = "simulation";
  s.ball_id = "ball";
  return s;
}
ModelState contact_state(double x) {
  ModelState s;
  s.position = Vec3(x, -.7, .02);
  s.velocity = Vec3(3, 0, -2);
  return s;
}
PhysicalEvent event(PhysicalEventKind kind, double t, double x) {
  PhysicalEvent e;
  e.kind = kind;
  e.dt_s = t;
  e.pre = e.post = contact_state(x);
  return e;
}
std::vector<double> base_wire(std::uint64_t sequence, std::int64_t ns,
                              double x) {
  return {2,
          1,
          double(sequence),
          double(ns / 1000000000LL),
          double(ns % 1000000000LL),
          x,
          -.7,
          0,
          0,
          0,
          0,
          1,
          1,
          47,
          7,
          9};
}
TEST(FlightV2, A01_C01_PublicStatesAndCandidateQualification) {
  EXPECT_STREQ(name(PlayState::INVALID), "INVALID");
  for (int rate : {150, 300})
    for (int direction : {-1, 1}) {
      ShotContext shot({});
      EXPECT_EQ(shot.state(), PlayState::INVALID);
      for (int i = 0; i < 20; ++i) {
        double t = double(i) / rate;
        shot.observe(
            sample(std::llround(t * 1e9),
                   Vec3(1 + direction * 3 * t, -.7, .5 + t - 4.905 * t * t)));
      }
      EXPECT_TRUE(shot.qualify_candidate());
      EXPECT_EQ(shot.state(),
                direction > 0 ? PlayState::RALLY : PlayState::OPPONENT_RALLY);
      EXPECT_EQ(shot.classification(), ClassificationStatus::PROVISIONAL);
      EXPECT_FALSE(shot.route_supported());
      ShotContext held({});
      for (int i = 0; i < 20; ++i)
        held.observe(sample(i * 10000000LL));
      EXPECT_FALSE(held.qualify_candidate());
      ShotSettings settings;
      settings.service_armed = true;
      ShotContext armed({}, settings);
      for (int i = 0; i < 20; ++i) {
        double t = double(i) / rate;
        armed.observe(sample(std::llround(t * 1e9),
                             Vec3(1 + 3 * t, -.7, .5 + t - 4.905 * t * t)));
      }
      EXPECT_FALSE(armed.qualify_candidate());
    }
}
TEST(FlightV2,
     A02_A08_C02_C03_T06_ReclassificationPreservesShotDataAndHardResetRetires) {
  for (int rate : {150, 300})
    for (int direction : {-1, 1}) {
      ShotContext shot({});
      shot.start(0, direction, BoundaryEvidence::RACKET_CONTACT,
                 HistoryContinuity::kComplete);
      auto measured = sample(0, Vec3(direction > 0 ? .2 : 2.5, -.7, .4));
      measured.producer_omega_valid = true;
      measured.producer_omega_rad_s = Vec3(0, 30, 0);
      shot.observe(measured);
      for (int i = 1; i < 5; ++i)
        shot.observe(sample(std::llround(i * 1e9 / rate), measured.position));
      const auto generation = shot.generation(), revision = shot.revision(),
                 count = shot.samples().size();
      auto old = shot.snapshot();
      EXPECT_TRUE(shot.confirm(PhysicalEventKind::TABLE, 40000000, 60000000,
                               contact_state(direction > 0 ? .5 : 2.3),
                               "independent"));
      EXPECT_EQ(shot.state(),
                direction > 0 ? PlayState::SERVE : PlayState::OPPONENT_SERVE);
      EXPECT_EQ(shot.generation(), generation);
      EXPECT_EQ(shot.samples().size(), count);
      EXPECT_EQ(shot.samples().front().source_time_ns, 0);
      EXPECT_GT(shot.revision(), revision);
      EXPECT_FALSE(shot.publishable(old, old.source_ns));
      EXPECT_EQ(shot.spin().mode, EffectiveSpinMode::CONTACT_PROPAGATED);
      const auto omega = *shot.spin().omega;
      EXPECT_TRUE(shot.confirm(PhysicalEventKind::TABLE, 40000000, 70000000,
                               contact_state(direction > 0 ? .5 : 2.3),
                               "independent"));
      EXPECT_EQ(*shot.spin().omega, omega);
      EXPECT_EQ(shot.observed_contacts(), 1);
      EXPECT_TRUE(shot.confirm(PhysicalEventKind::TABLE, 160000000, 180000000,
                               contact_state(direction > 0 ? .7 : 2.0),
                               "independent"));
      EXPECT_EQ(shot.state(), PlayState::INVALID);
      EXPECT_TRUE(shot.samples().empty());
      EXPECT_FALSE(shot.spin().usable());
      EXPECT_GT(shot.generation(), generation);
      for (int i = 0; i < 20; ++i)
        shot.observe(sample(200000000 + i * 10000000LL,
                            Vec3(1 - direction * .03 * i, -.7, .5)));
      EXPECT_FALSE(shot.qualify_candidate());
      shot.start(300000000, direction, BoundaryEvidence::RACKET_CONTACT,
                 HistoryContinuity::kComplete);
      for (const auto &s : shot.samples())
        EXPECT_GE(s.source_time_ns, 300000000);
    }
}
TEST(FlightV2, C04_C05_C06_ObservedAndPredictedRoutesAreSeparate) {
  ShotContext shot({});
  shot.start(0, 1, BoundaryEvidence::ACQUISITION, HistoryContinuity::kUnknown);
  shot.observe(sample(0));
  auto predicted = event(PhysicalEventKind::TABLE, .1, .5);
  shot.predict_events({predicted}, 0);
  EXPECT_EQ(shot.state(), PlayState::RALLY);
  EXPECT_EQ(shot.route(), PlayState::SERVE);
  EXPECT_EQ(shot.observed_contacts(), 0);
  EXPECT_FALSE(shot.route_supported());
  shot.predict_events({}, 0);
  EXPECT_EQ(shot.route(), PlayState::RALLY);
  EXPECT_EQ(shot.samples().size(), 1u);
  EXPECT_EQ(shot.events()[0].status, EventStatus::SUPERSEDED);
  EXPECT_TRUE(shot.context(MatchContext::NORMAL_RALLY, 0, "referee",
                           shot.generation()));
  shot.confirm(PhysicalEventKind::TABLE, 100000000, 120000000,
               contact_state(.5), "referee");
  EXPECT_FALSE(shot.active());
}
TEST(FlightV2, A06_C06_T02_OrderedServesInBothDirectionsAndFirstReturn) {
  for (int direction : {-1, 1}) {
    ShotContext shot({});
    shot.start(0, direction, BoundaryEvidence::RACKET_CONTACT,
               HistoryContinuity::kComplete);
    shot.observe(sample(0));
    const auto generation = shot.generation();
    shot.confirm(PhysicalEventKind::TABLE, 10000000000LL, 10020000000LL,
                 contact_state(direction > 0 ? .4 : 2.3), "camera");
    ASSERT_TRUE(shot.active());
    EXPECT_EQ(shot.events()[0].event_time_ns, 10000000000LL);
    EXPECT_EQ(shot.events()[0].confirmed_at_ns, 10020000000LL);
    auto net = contact_state(1.37);
    net.position.z() = .3;
    shot.confirm(PhysicalEventKind::NET_CROSSING, 10100000000LL, 10100000000LL,
                 net, "camera");
    shot.confirm(PhysicalEventKind::TABLE, 10200000000LL, 10220000000LL,
                 contact_state(direction > 0 ? 2.2 : .5), "camera");
    EXPECT_EQ(shot.generation(), generation);
    EXPECT_EQ(shot.state(),
              direction > 0 ? PlayState::SERVE : PlayState::OPPONENT_SERVE);
    EXPECT_EQ(shot.observed_contacts(), 2);
    shot.start(10300000000LL, -direction, BoundaryEvidence::RACKET_CONTACT,
               HistoryContinuity::kComplete);
    EXPECT_GT(shot.generation(), generation);
    EXPECT_TRUE(shot.events().empty());
    EXPECT_TRUE(shot.samples().empty());
  }
}
TEST(FlightV2, A03_A04_A05_A14_EndpointApexAndPrefixLegality) {
  for (int direction : {-1, 1}) {
    ShotContext shot({});
    shot.start(0, direction, BoundaryEvidence::RACKET_CONTACT,
               HistoryContinuity::kComplete);
    const auto net = event(PhysicalEventKind::NET_CROSSING, .1, 1.37),
               b1 = event(PhysicalEventKind::TABLE, .2, direction > 0 ? 2 : .7),
               apex =
                   event(PhysicalEventKind::APEX, .3, direction > 0 ? 2.2 : .5),
               b2 = event(PhysicalEventKind::TABLE, .4,
                          direction > 0 ? 2.4 : .3),
               target = event(direction > 0 ? PhysicalEventKind::END
                                            : PhysicalEventKind::PLANE,
                              .5, direction > 0 ? 2.74 : .15);
    auto result =
        select_route_target(shot, {net, b1, apex, b2, target}, {}, true);
    EXPECT_EQ(result.valid, direction > 0);
    if (direction > 0) {
      ASSERT_TRUE(result.event);
      EXPECT_EQ(result.event->kind, PhysicalEventKind::APEX);
    } else
      EXPECT_TRUE(result.known_forbidden);
    auto early = target;
    early.dt_s = .35;
    result = select_route_target(shot, {net, b1, early, b2}, {}, true);
    EXPECT_TRUE(result.valid);
    result = select_route_target(shot, {}, {}, true);
    EXPECT_EQ(result.status, ForecastStatus::HORIZON_EXHAUSTED);
    EXPECT_TRUE(shot.active());
  }
}
TEST(FlightV2,
     A09_C07_X03_OrdinarySamplesDoNotStarveAndUncertainViolationCanRecover) {
  ShotContext shot({});
  shot.start(0, -1, BoundaryEvidence::RACKET_CONTACT,
             HistoryContinuity::kComplete);
  shot.observe(sample(0));
  auto token = shot.snapshot();
  shot.observe(sample(1000000));
  EXPECT_TRUE(shot.publishable(token, 1000000));
  EXPECT_FALSE(shot.publishable(token, 1000000));
  auto current = shot.snapshot();
  shot.context(MatchContext::SERVE, 1000000, "referee", shot.generation());
  EXPECT_FALSE(shot.publishable(current, 1000000));
  auto result = select_route_target(
      shot, {event(PhysicalEventKind::NET_COLLISION, .1, 1.37)}, {}, true);
  EXPECT_TRUE(result.known_forbidden);
  EXPECT_TRUE(shot.active());
  auto old = shot.snapshot();
  shot.invalidate("confirmed_violation");
  EXPECT_FALSE(shot.publishable(old, 1000000));
}
TEST(FlightV2, C08_ReversalAloneNeverChangesShotAndSameDirectionContactDoes) {
  ShotContext shot({});
  shot.start(0, -1, BoundaryEvidence::RACKET_CONTACT,
             HistoryContinuity::kComplete);
  auto generation = shot.generation();
  for (int i = 0; i < 20; ++i)
    shot.observe(sample(i * 5000000LL, Vec3(.5 + .02 * i, -.7, .5)));
  EXPECT_EQ(shot.generation(), generation);
  EXPECT_EQ(shot.state(), PlayState::OPPONENT_RALLY);
  shot.start(100000000, -1, BoundaryEvidence::RACKET_CONTACT,
             HistoryContinuity::kComplete);
  EXPECT_GT(shot.generation(), generation);
  EXPECT_TRUE(shot.samples().empty());
}
TEST(FlightV2, B01_B03_B04_B07_B08_TrustedVectorDropoutAndRequalification) {
  for (int rate : {150, 300}) {
    TrustedSpin spin;
    auto s = sample(0);
    s.producer_omega_valid = true;
    s.producer_omega_rad_s = Vec3(0, 30, 0);
    spin.observe(s);
    const auto source = spin.value().measurement_time_ns;
    for (int i = 1; i < 8; ++i) {
      s = sample(std::llround(i * 1e9 / rate));
      s.producer_omega_valid = i % 2;
      s.producer_omega_rad_s = i == 1 ? Vec3(0, 0, 0) : Vec3(NAN, 2, 3);
      spin.observe(s);
      ASSERT_TRUE(spin.value().usable());
      EXPECT_EQ(*spin.value().omega, Vec3(0, 30, 0));
      EXPECT_EQ(spin.value().measurement_time_ns, source);
      EXPECT_EQ(spin.value().mode, EffectiveSpinMode::HELD);
    }
    spin.contact(3, 100000000, Vec3(2, 0, -3), {});
    const auto after = *spin.value().omega;
    EXPECT_NE(after, Vec3(0, 30, 0));
    EXPECT_FALSE(spin.contact(3, 100000000, Vec3(2, 0, -3), {}));
    EXPECT_EQ(*spin.value().omega, after);
    s = sample(200000000);
    s.orientation_valid = true;
    s.orientation_authoritative = false;
    spin.observe(s);
    EXPECT_EQ(spin.value().mode, EffectiveSpinMode::CONTACT_PROPAGATED);
    s.producer_omega_valid = true;
    s.producer_omega_rad_s = Vec3(0, 40, 0);
    s.source_time_ns++;
    s.source_time_s += 1e-9;
    spin.observe(s);
    EXPECT_EQ(spin.value().mode, EffectiveSpinMode::MEASURED);
    spin.reset();
    EXPECT_FALSE(spin.value().usable());
  }
}
TEST(FlightV2, B02_B06_PositionOnlyNeverSynthesizesSpinAndEnergyIsDissipative) {
  for (bool trusted : {false, true})
    for (int direction : {-1, 1}) {
      ModelState model;
      model.position = Vec3(.5, -.7, .02);
      model.velocity = Vec3(direction * 3., 1, -2);
      if (trusted) {
        model.spin.omega = Vec3(0, 30, 0);
        model.spin.mode = EffectiveSpinMode::HELD;
      }
      auto energy = [](const ModelState &s) {
        return .5 * s.velocity.squaredNorm() +
               (s.spin.usable()
                    ? .5 * (2. / 3) * .02 * .02 * s.spin.omega->squaredNorm()
                    : 0);
      };
      const double before = energy(model);
      apply_model_contact(model, {});
      EXPECT_LE(energy(model), before + 1e-12);
      EXPECT_EQ(model.spin.usable(), trusted);
      model.velocity.z() = -2;
      apply_model_contact(model, {});
      EXPECT_EQ(model.spin.usable(), trusted);
    }
}
TEST(FlightV2, B08_QuaternionSignsAxesAndGapCausality) {
  TrustedSpin spin;
  for (int i = 0; i < 30; ++i) {
    auto s = sample(i * 5000000LL);
    s.orientation_authoritative = true;
    s.orientation_valid = true;
    s.orientation =
        Eigen::Quaterniond(Eigen::AngleAxisd(30 * i * .005, Vec3::UnitY()));
    if (i % 2)
      s.orientation.coeffs() *= -1;
    spin.observe(s);
  }
  ASSERT_TRUE(spin.value().usable());
  EXPECT_LT((*spin.value().omega - Vec3(0, 30, 0)).norm(), 1e-8);
  auto time = spin.value().measurement_time_ns;
  auto s = sample(500000000);
  s.orientation_authoritative = true;
  s.orientation_valid = true;
  spin.observe(s);
  EXPECT_EQ(spin.value().measurement_time_ns, time);
  EXPECT_EQ(spin.value().mode, EffectiveSpinMode::HELD);
}
TEST(FlightV2, B09_A07_FiniteNetFootprintBelowTableAndOrdering) {
  BallPhysics p;
  p.drag_k = 0;
  p.gravity.setZero();
  EventGeometry g;
  g.receiving_x = .9995;
  ModelState below;
  below.position = Vec3(.5, -.7, .01);
  below.velocity = Vec3(-1, 0, -1);
  auto r = advance_ball(below, .01, p, g);
  EXPECT_TRUE(r.stopped);
  EXPECT_EQ(r.reason, "below_table_initial_state");
  EXPECT_LT(below.velocity.z(), 0);
  ModelState net;
  net.position = Vec3(1.4, -.7, .1);
  net.velocity = Vec3(-3, 0, 0);
  r = advance_ball(net, .02, p, g);
  EXPECT_TRUE(r.stopped);
  EXPECT_EQ(r.reason, "net_collision");
  net.position = Vec3(1.4, .3, .1);
  r = advance_ball(net, .02, p, g);
  EXPECT_FALSE(r.stopped);
  ModelState edge;
  edge.position = Vec3(-.01, -.7, .021);
  edge.velocity = Vec3(0, 0, -1);
  r = advance_ball(edge, .002, p, g);
  EXPECT_TRUE(r.stopped);
  EXPECT_EQ(r.reason, "edge_ambiguous");
  EXPECT_LT(edge.velocity.z(), 0);
  ModelState hit;
  hit.position = Vec3(1, -.7, .02075);
  hit.velocity = Vec3(-2, 0, -1);
  r = advance_ball(hit, .001, p, g);
  ASSERT_GE(r.events.size(), 2u);
  EXPECT_EQ(r.events.front().kind, PhysicalEventKind::PLANE);
  EXPECT_NEAR(r.events.front().dt_s, .00025, 1e-10);
  g.receiving_x = .9985;
  hit.position = Vec3(1, -.7, .02075);
  hit.velocity = Vec3(-2, 0, -1);
  r = advance_ball(hit, .001, p, g);
  ASSERT_FALSE(r.events.empty());
  EXPECT_EQ(r.events.front().kind, PhysicalEventKind::TABLE);
}
TEST(FlightV2, B10_X09_DerivativesCovarianceAndInvalidParameters) {
  ProbabilisticConfig c;
  State6 state;
  state << .5, -.7, .4, -3, .4, 1;
  EffectiveSpin spin;
  spin.omega = Vec3(0, 30, 5);
  const auto j = free_flight_jacobian(state, .02, c.physics, spin);
  for (int i = 0; i < 6; ++i) {
    State6 a = state, b = state;
    const double eps = 1e-4;
    a[i] += eps;
    b[i] -= eps;
    EXPECT_LT((j.col(i) - (free_flight(a, .02, c.physics, spin) -
                           free_flight(b, .02, c.physics, spin)) /
                              (2 * eps))
                  .norm(),
              1e-7);
  }
  StateBelief belief;
  belief.initialized = true;
  belief.mean = state;
  belief.covariance = Matrix6::Identity() * .00001;
  c.effective_spin = spin;
  const auto forecast = forecast_nominal(belief, c);
  ASSERT_TRUE(forecast.available);
  for (const auto &w : forecast.waypoints) {
    ASSERT_TRUE(w.covariance.allFinite());
    EXPECT_LT((w.covariance - w.covariance.transpose()).norm(), 1e-8);
    Eigen::SelfAdjointEigenSolver<Matrix6> eig(w.covariance);
    EXPECT_GT(eig.eigenvalues().minCoeff(), -1e-9);
  }
  EXPECT_NE(forecast.uncertainty_provenance.find("conditional"),
            std::string::npos);
  for (double gain : {.40001, -.01, double(NAN)}) {
    auto bad = c;
    bad.physics.table_tangential_gain = gain;
    EXPECT_THROW(bad.validate(), std::invalid_argument);
  }
}
TEST(FlightV2, A10_T04_EventIdentityConfirmationAndPredictedTimeIsNotEvidence) {
  ShotContext shot({});
  shot.start(0, 1, BoundaryEvidence::RACKET_CONTACT,
             HistoryContinuity::kComplete);
  auto p = event(PhysicalEventKind::TABLE, .1, .5);
  shot.predict_events({p}, 0);
  auto id = shot.events()[0].id;
  shot.observe(sample(200000000));
  EXPECT_EQ(shot.observed_contacts(), 0);
  shot.confirm(PhysicalEventKind::TABLE, 100000000, 120000000, p.pre,
               "independent");
  EXPECT_EQ(shot.events()[0].id, id);
  EXPECT_EQ(shot.observed_contacts(), 1);
}
TEST(FlightV2, T01_T03_T05_ExactEpochZeroDuplicatesOverflowAndProvenance) {
  constexpr std::int64_t epoch = 1792412345678901234LL;
  for (auto base : {std::int64_t(0), epoch})
    for (int rate : {150, 300}) {
      ShotContext shot({});
      shot.start(base, 1, BoundaryEvidence::RACKET_CONTACT,
                 HistoryContinuity::kComplete);
      std::vector<std::int64_t> stamps;
      for (int i = 0; i < 100; ++i) {
        auto ns = base + std::llround(i * 1e9 / rate) + (i % 3) * 100;
        stamps.push_back(ns);
        shot.observe(
            sample(ns, Vec3(.3 + .001 * i, -.7,
                            3. - 4.905 * std::pow(double(i) / rate, 2))));
      }
      for (const auto &s : shot.samples())
        EXPECT_NE(std::find(stamps.begin(), stamps.end(), s.source_time_ns),
                  stamps.end());
      auto last = shot.samples().back();
      EXPECT_FALSE(shot.observe(last));
      last.source_time_ns--;
      EXPECT_FALSE(shot.observe(last));
    }
  std::int64_t arrival;
  EXPECT_TRUE(checked_arrival(20000000000LL, 300000000, arrival));
  EXPECT_EQ(arrival, 20300000000LL);
  EXPECT_FALSE(checked_arrival(INT64_MAX - 5, 10, arrival));
  EXPECT_FALSE(duration_ns(NAN));
  ShotContext shot({});
  shot.start(0, 1, BoundaryEvidence::RACKET_CONTACT,
             HistoryContinuity::kComplete);
  shot.observe(sample(0));
  auto altered = sample(1000000);
  altered.calibration_id = "new";
  EXPECT_FALSE(shot.observe(altered));
  EXPECT_FALSE(shot.active());
}
TEST(FlightV2, A11_X01_X02_PlanePoliciesAndFreshness) {
  for (auto policy : {PlanePolicy::ON_REFRESH, PlanePolicy::FOLLOW_POSE}) {
    HitPlaneSettings settings;
    settings.policy = policy;
    settings.min_samples = 2;
    HitPlaneManager manager(settings, "table", "registration", 9);
    auto source = 1000000000LL, receipt = 1000000000LL;
    ASSERT_TRUE(
        manager.observe(base_wire(1, source, -.58), receipt, source, true));
    ASSERT_TRUE(manager.observe(base_wire(2, source + 10000000, -.58),
                                receipt + 10000000, source + 10000000, true));
    ASSERT_TRUE(manager.freeze(receipt + 10000000, source + 10000000, 1, true));
    auto first = manager.for_solve(receipt + 10000000, source + 10000000);
    EXPECT_NEAR(first.x_hit_m, 0, 1e-12);
    ASSERT_TRUE(manager.observe(base_wire(3, source + 20000000, -.48),
                                receipt + 20000000, source + 20000000, true));
    auto moved = manager.for_solve(receipt + 20000000, source + 20000000);
    EXPECT_NEAR(moved.x_hit_m, policy == PlanePolicy::FOLLOW_POSE ? .1 : 0,
                1e-12);
    auto stale = manager.for_solve(receipt + 500000000, source + 500000000);
    EXPECT_EQ(stale.calibrated, policy == PlanePolicy::ON_REFRESH);
  }
}
TEST(FlightV2, X04_LatestWorkIsBoundedAndNeverWaitsForNextSampleToPublish) {
  LatestWork<int> work;
  for (int i = 0; i < 1000; ++i)
    work.push(i);
  EXPECT_EQ(work.superseded(), 999u);
  EXPECT_EQ(*work.take(), 999);
  auto waiting = std::async(std::launch::async, [&] { return work.take(); });
  work.close();
  EXPECT_FALSE(waiting.get());
  EXPECT_FALSE(work.push(2));
}
TEST(FlightV2, X09_OutgoingSurfaceToCenterExactlyOnceAndNoIncomingSpinLeak) {
  StrikeTarget strike;
  strike.valid = true;
  strike.ball_position = Vec3(.15, -.7, .3);
  strike.ball_velocity = Vec3(-3, 0, 1);
  const Vec3 target(2.055, -.7625, 0);
  auto command = RacketTargetPlanner({}, {}, {}).plan(strike, target, .5);
  ASSERT_TRUE(command.valid) << command.reason;
  EXPECT_TRUE(command.outgoing_route_valid);
  EXPECT_FALSE(command.outgoing_spin.usable());
  EXPECT_EQ(command.target_land, target);
  ModelState model;
  model.position = strike.ball_position;
  model.velocity = command.outgoing_ball_velocity;
  EventGeometry geometry;
  bool found = false;
  for (double t = 0; t < command.landing_duration_s + .002; t += .00025) {
    auto step = advance_ball(model, .00025, {}, geometry);
    for (const auto &e : step.events)
      if (e.kind == PhysicalEventKind::TABLE) {
        EXPECT_NEAR(e.pre.position.z(), .02, 1e-12);
        EXPECT_LT((e.pre.position.head<2>() - target.head<2>()).norm(), .001);
        EXPECT_NEAR(t + e.dt_s, command.landing_duration_s, .0005);
        found = true;
        break;
      }
    if (found)
      break;
  }
  EXPECT_TRUE(found);
}
} // namespace
TEST(
    FlightV2Support,
    X03_ConditionalCovarianceNeverProvesViolationButCompleteConservativeBoundCan) {
  ShotContext shot({});
  shot.start(0, -1, BoundaryEvidence::RACKET_CONTACT,
             HistoryContinuity::kComplete);
  PredictedViolationBound bound;
  bound.forbidden_latest_s = .1;
  bound.target_earliest_s = .3;
  bound.net_clearance_upper_m = -.02;
  bound.finite_net_geometry = true;
  EXPECT_FALSE(shot.reject_supported_prediction(
      PhysicalEventKind::NET_COLLISION, bound));
  EXPECT_TRUE(shot.active());
  bound.all_relevant_uncertainty_bounded = true;
  bound.forbidden_latest_s = .2999;
  EXPECT_FALSE(shot.reject_supported_prediction(
      PhysicalEventKind::NET_COLLISION, bound));
  bound.forbidden_latest_s = .1;
  EXPECT_TRUE(shot.reject_supported_prediction(PhysicalEventKind::NET_COLLISION,
                                               bound));
  EXPECT_FALSE(shot.active());
  EXPECT_TRUE(shot.samples().empty());
}
TEST(FlightV2Motion,
     A07_HeldMotionAndApexUsePhysicalEvidenceRatherThanLowVerticalSpeed) {
  for (bool apex : {false, true}) {
    ShotContext shot({});
    shot.start(0, 1, BoundaryEvidence::RACKET_CONTACT,
               HistoryContinuity::kComplete);
    for (int i = 0; i < 60 && shot.active(); ++i) {
      double t = i / 300.;
      shot.observe(sample(
          std::llround(t * 1e9),
          Vec3(.2 + 3 * t, -.7, apex ? .6 + .5 * t - 4.905 * t * t : .6)));
    }
    EXPECT_EQ(shot.active(), apex);
  }
}
TEST(FlightV2Load,
     X04_PacedSlowSolvesPublishMonotoneFreshResultsWithOnePendingSlot) {
  for (int rate : {150, 300}) {
    LatestWork<std::pair<int, std::chrono::steady_clock::time_point>> work;
    std::vector<int> published;
    std::vector<double> ages;
    std::thread worker([&] {
      while (auto job = work.take()) {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(job->first % 16 == 0 ? 16 : 1));
        const auto age = std::chrono::duration<double>(
                             std::chrono::steady_clock::now() - job->second)
                             .count();
        if (age <= .030) {
          published.push_back(job->first);
          ages.push_back(age);
        }
      }
    });
    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < 60; ++i) {
      std::this_thread::sleep_until(
          start + std::chrono::nanoseconds(std::llround(i * 1e9 / rate)));
      work.push({i, std::chrono::steady_clock::now()});
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    work.close();
    worker.join();
    ASSERT_GT(published.size(), 35u);
    EXPECT_TRUE(std::is_sorted(published.begin(), published.end()));
    EXPECT_EQ(std::adjacent_find(published.begin(), published.end()),
              published.end());
    EXPECT_GT(work.superseded(), 0u);
    for (double age : ages)
      EXPECT_LE(age, .030);
  }
}
TEST(FlightV2Numerics,
     A03_A11_B10_AnalyticApexAndBothTargetDirectionsMeetEventBudget) {
  BallPhysics physics;
  physics.drag_k = 0;
  physics.magnus_k = 0;
  ModelState model;
  model.position = Vec3(.5, -.7, .4);
  model.velocity = Vec3(2, 0, .981);
  EventGeometry geometry;
  geometry.record_end = false;
  const auto apex_step = advance_ball(model, .1, physics, geometry);
  auto apex = std::find_if(
      apex_step.events.begin(), apex_step.events.end(),
      [](const auto &event) { return event.kind == PhysicalEventKind::APEX; });
  ASSERT_NE(apex, apex_step.events.end());
  EXPECT_NEAR(apex->dt_s, .1, .0005);
  EXPECT_NEAR(apex->pre.position.z(), .44905, .001);
  EXPECT_NEAR(apex->pre.position.x(), .7, .001);
  physics.gravity.setZero();
  for (int direction : {-1, 1}) {
    model.position = Vec3(direction > 0 ? 2.7395 : 1., -.7, .1);
    model.velocity = Vec3(direction * 2., 0, 0);
    geometry.record_end = true;
    geometry.receiving_x = .9995;
    const auto step = advance_ball(model, .001, physics, geometry);
    auto target = std::find_if(
        step.events.begin(), step.events.end(), [&](const auto &event) {
          return event.kind == (direction > 0 ? PhysicalEventKind::END
                                              : PhysicalEventKind::PLANE);
        });
    ASSERT_NE(target, step.events.end());
    EXPECT_NEAR(target->dt_s, .00025, 1e-10);
    EXPECT_NEAR(target->pre.position.x(), direction > 0 ? 2.74 : .9995, 1e-10);
  }
}
TEST(FlightV2History,
     C04_C07_MatchTypeDoesNotCertifyMissingHistoryAndGapFencesOldSolve) {
  ProbabilisticConfig config;
  config.gap_warning_s = .010;
  ShotContext shot(config);
  shot.start(0, -1, BoundaryEvidence::ACQUISITION, HistoryContinuity::kUnknown);
  shot.observe(sample(0));
  ASSERT_TRUE(shot.context(MatchContext::NORMAL_RALLY, 0, "referee",
                           shot.generation()));
  EXPECT_FALSE(shot.route_supported());
  const auto generation = shot.generation(), size = shot.samples().size();
  shot.set_history_continuity(HistoryContinuity::kComplete);
  EXPECT_TRUE(shot.route_supported());
  EXPECT_EQ(shot.generation(), generation);
  EXPECT_EQ(shot.samples().size(), size);
  auto token = shot.snapshot();
  shot.observe(sample(20000000, Vec3(.24, -.7, .5)));
  EXPECT_EQ(shot.continuity(), HistoryContinuity::kIncomplete);
  EXPECT_FALSE(shot.publishable(token, 20000000));
  EXPECT_FALSE(shot.route_supported());
  const auto revision = shot.revision();
  ASSERT_TRUE(shot.context(MatchContext::NORMAL_RALLY, 20000000, "referee",
                           shot.generation()));
  EXPECT_EQ(shot.revision(), revision);
}

TEST(FlightV2History,
     C04_ReceiverAcquisitionAcceptsVerifiedPrehistoryWithoutReset) {
  for (int direction : {-1, 1}) {
    ShotContext shot({});
    shot.start(1000000000, direction, BoundaryEvidence::ACQUISITION,
               HistoryContinuity::kUnknown);
    auto s = sample(1000000000, Vec3(direction > 0 ? 2.1 : .6, -.7, .15));
    s.producer_omega_valid = true;
    s.producer_omega_rad_s = Vec3(0, 30, 0);
    ASSERT_TRUE(shot.observe(s));
    const auto generation = shot.generation();
    ASSERT_TRUE(shot.confirm(PhysicalEventKind::TABLE, 1100000000, 1120000000,
                             contact_state(direction > 0 ? 2.3 : .4),
                             "camera"));
    ASSERT_TRUE(shot.active());
    EXPECT_FALSE(shot.route_supported());
    const auto omega = *shot.spin().omega;
    const auto contact_id = shot.spin().last_contact_id;
    ASSERT_TRUE(
        shot.context(MatchContext::SERVE, 1120000000, "referee", generation));
    ASSERT_TRUE(shot.active());
    ASSERT_TRUE(shot.confirm(PhysicalEventKind::TABLE, 700000000, 1130000000,
                             contact_state(direction > 0 ? .4 : 2.3),
                             "independent_history"));
    auto net = contact_state(1.37);
    net.position.z() = .3;
    ASSERT_TRUE(shot.confirm(PhysicalEventKind::NET_CROSSING, 800000000,
                             1130000000, net, "independent_history"));
    EXPECT_EQ(shot.generation(), generation);
    EXPECT_EQ(shot.samples().size(), 1u);
    EXPECT_EQ(shot.samples().front().source_time_ns, 1000000000);
    EXPECT_EQ(shot.spin().measurement_time_ns, 1000000000);
    EXPECT_EQ(*shot.spin().omega, omega);
    EXPECT_EQ(shot.spin().last_contact_id, contact_id);
    EXPECT_FALSE(shot.events().front().pre.spin.usable());
    shot.set_history_continuity(HistoryContinuity::kComplete);
    ASSERT_TRUE(shot.active());
    EXPECT_TRUE(shot.route_supported());
    EXPECT_EQ(shot.state(),
              direction > 0 ? PlayState::SERVE : PlayState::OPPONENT_SERVE);
    ShotContext restored({});
    restored.restore_snapshot(shot.export_snapshot());
    EXPECT_FALSE(restored.export_snapshot().boundary_qualified);
    EXPECT_EQ(restored.observed_contacts(), 2);
    shot.start(1200000000, direction, BoundaryEvidence::RACKET_CONTACT,
               HistoryContinuity::kComplete);
    EXPECT_FALSE(shot.confirm(PhysicalEventKind::TABLE, 1100000000, 1210000000,
                              contact_state(.4), "prior_shot"));
  }
}
TEST(FlightV2History,
     B03_B05_C04_ContactIdsAreIdentityAndLateImpulsesAreAmbiguous) {
  TrustedSpin spin;
  auto s = sample(1000000000);
  s.producer_omega_valid = true;
  s.producer_omega_rad_s = Vec3(0, 30, 0);
  spin.observe(s);
  ASSERT_TRUE(spin.contact(7, 700000000, Vec3(3, 0, -2), {}));
  EXPECT_EQ(spin.value().last_contact_id, 0u);
  ASSERT_TRUE(spin.contact(2, 1100000000, Vec3(3, 0, -2), {}));
  const auto omega = *spin.value().omega;
  EXPECT_FALSE(spin.contact(2, 1100000000, Vec3(3, 0, -2), {}));
  EXPECT_FALSE(spin.contact(8, 1050000000, Vec3(3, 0, -2), {}));
  EXPECT_EQ(*spin.value().omega, omega);
  s = sample(1200000000);
  s.producer_omega_valid = true;
  s.producer_omega_rad_s = Vec3(0, 35, 0);
  spin.observe(s);
  EXPECT_TRUE(spin.contact(8, 1050000000, Vec3(3, 0, -2), {}));
  EXPECT_EQ(*spin.value().omega, Vec3(0, 35, 0));

  ShotContext shot({});
  shot.start(0, 1, BoundaryEvidence::RACKET_CONTACT,
             HistoryContinuity::kComplete);
  s = sample(1000000000);
  s.producer_omega_valid = true;
  s.producer_omega_rad_s = Vec3(0, 30, 0);
  shot.observe(s);
  ASSERT_TRUE(shot.confirm(PhysicalEventKind::TABLE, 1100000000, 1120000000,
                           contact_state(.4), "camera"));
  EXPECT_FALSE(shot.confirm(PhysicalEventKind::TABLE, 1050000000, 1130000000,
                            contact_state(.8), "late_camera"));
  EXPECT_TRUE(shot.active());
  EXPECT_FALSE(shot.route_supported());
  EXPECT_EQ(shot.events().back().status, EventStatus::AMBIGUOUS);
}

TEST(FlightV2History,
     B05_T01_AcquisitionReplaysSpinCausallyAndNewShotClearsIt) {
  FlightPipeline acquired({}), causal({});
  causal.start(0, 1, BoundaryEvidence::RACKET_CONTACT,
               HistoryContinuity::kUnknown);
  for (int i = 0; i < 6; ++i) {
    const double t = i * .005;
    auto s =
        sample(i * 5000000LL, Vec3(.3 + 3 * t, -.7, .5 + t - 4.905 * t * t));
    if (i == 5) {
      s.producer_omega_valid = true;
      s.producer_omega_rad_s = Vec3(0, 300, 0);
    }
    acquired.observe(s);
    causal.observe(s);
  }
  ASSERT_TRUE(acquired.shot().active());
  EXPECT_LT(
      (acquired.checkpoint().belief.mean - causal.checkpoint().belief.mean)
          .norm(),
      1e-14);
  EXPECT_LT((acquired.checkpoint().belief.covariance -
             causal.checkpoint().belief.covariance)
                .norm(),
            1e-14);
  EXPECT_EQ(acquired.checkpoint().belief.effective_spin.measurement_time_ns,
            25000000);
  acquired.start(30000000, 1, BoundaryEvidence::RACKET_CONTACT,
                 HistoryContinuity::kComplete);
  acquired.observe(sample(30000000));
  EXPECT_FALSE(acquired.checkpoint().belief.effective_spin.usable());
}
