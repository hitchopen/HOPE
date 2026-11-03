#include "hope_planner_cpp/v2_strike_adapter.hpp"
#include "hope_planner_cpp/racket_target_planner.hpp"
#include "hope_planner_cpp/schema2_packer.hpp"
#include "hope_planner_cpp/v2_managed_revocation.hpp"
#include "a3_pingpong/pp_planner_input.hpp"
#include <gtest/gtest.h>

using namespace hope_planner_cpp;
namespace {
constexpr std::int64_t epoch = 1800000000000000000LL;
ForecastPhysicsEvent event(const char* kind, double dt, double x) {
  ForecastPhysicsEvent e;
  e.kind = kind; e.dt_s = dt; e.pre << x, -.7, .3, -3, 0, 1;
  e.post = e.pre; e.covariance.setZero();
  e.quality = LinearizationQuality::Valid; e.time_std_s = .0001;
  return e;
}
struct Fixture {
  FlightPrediction forecast;
  ShotSnapshot shot;
  IncomingPlane plane;
  Fixture() {
    PlaneSelection selected;
    selected.epoch = 7; selected.flight = 42; selected.valid = true;
    selected.reference_x = -.43; selected.plane_x = .15;
    plane = incoming_plane(selected, "table", "fixture", false);
    forecast.plane = plane.snapshot;
    forecast.physical_available = forecast.belief.initialized = true;
    forecast.belief.mean << 1, -.7, .4, -3, 0, -1;
    forecast.token.generation = shot.generation = 5;
    forecast.reference_ns = forecast.token.source_ns = epoch;
    forecast.expires_ns = epoch + 30000000;
    shot.direction = -1; shot.boundary_ns = epoch - 100000000;
    BallSample source;
    source.source_time_ns = epoch; source.frame_id = "table";
    source.calibration_id = "fixture"; source.clock_mapping_id = "source";
    source.ball_id = "ball";
    shot.observations.push_back(source);
    forecast.nominal.events = {event("table_bounce", .1, .6), event("receiving_plane", .3, .15)};
  }
  V2StrikeCandidate candidate(std::int64_t now = epoch) {
    return evaluate_v2_strike(forecast, shot, {}, plane, now);
  }
};
TEST(V2StrikeAdapter, PhysicalSupportAndRouteTargetAreDistinct) {
  Fixture f;
  ASSERT_FALSE(f.forecast.target.valid);
  const auto candidate = f.candidate();
  ASSERT_TRUE(candidate.command_eligible);
  EXPECT_TRUE(candidate.physical_state_available);
  EXPECT_TRUE(candidate.strike_contact_supported);
  EXPECT_EQ(candidate.status, IncomingStatus::Supported);
  const auto strike = v2_strike_target(candidate, epoch);
  ASSERT_TRUE(strike.valid);
  EXPECT_EQ(strike.ball_position, Vec3(.15, -.7, .3));
  EXPECT_EQ(strike.ball_velocity, Vec3(-3, 0, 1));
  EXPECT_EQ(strike.strike_source_time_s, double(epoch + 300000000) * 1e-9);
}
TEST(V2StrikeAdapter, UnknownUnsupportedExpiredAndForbiddenAreExplicit) {
  Fixture f;
  f.forecast.belief.initialized = false;
  EXPECT_EQ(f.candidate().status, IncomingStatus::Acquiring);
  f = Fixture{}; f.forecast.nominal.events.clear();
  EXPECT_EQ(f.candidate().status, IncomingStatus::Unavailable);
  EXPECT_FALSE(v2_strike_target(f.candidate(), epoch).valid);
  for (const char* kind : {"ground", "net_collision", "table_bounce"}) {
    f = Fixture{};
    f.forecast.nominal.events.insert(f.forecast.nominal.events.begin()+1, event(kind, .2, .3));
    EXPECT_EQ(f.candidate().status, IncomingStatus::Ineligible) << kind;
    EXPECT_FALSE(v2_strike_target(f.candidate(), epoch).valid);
  }
  f = Fixture{}; f.forecast.nominal.events.front().time_std_s = .2;
  EXPECT_EQ(f.candidate().status, IncomingStatus::Unavailable);
  f = Fixture{};
  EXPECT_EQ(f.candidate(epoch + 30000001).status, IncomingStatus::Expired);
  auto candidate = f.candidate();
  EXPECT_FALSE(v2_strike_target(candidate, epoch + 30000001).valid);
  candidate.status = IncomingStatus::Revoked;
  EXPECT_FALSE(v2_strike_target(candidate, epoch).valid);
  f = Fixture{}; f.forecast.nominal.events.push_back(event("table_bounce", .4, .1));
  EXPECT_TRUE(f.candidate().command_eligible);  // Later second bounce is irrelevant.
}
TEST(V2StrikeAdapter, TentativePlaneHasPhysicalSupportWithoutCommandAuthority) {
  Fixture f; f.plane.tentative = true;
  const auto candidate = f.candidate();
  EXPECT_TRUE(candidate.strike_contact_supported);
  EXPECT_FALSE(candidate.command_eligible);
  EXPECT_FALSE(v2_strike_target(candidate, epoch).valid);
  f.plane.tentative = false; f.plane.snapshot.generation++;
  EXPECT_FALSE(f.candidate().command_eligible);
}
TEST(V2StrikeAdapter, AbsentBounceWrongDirectionAndAmbiguousOrderingNeverAuthorize) {
  Fixture f;
  f.forecast.nominal.events.erase(f.forecast.nominal.events.begin());
  EXPECT_EQ(f.candidate().reason, "PLANE_BEFORE_FIRST_BOUNCE");
  EXPECT_FALSE(v2_strike_target(f.candidate(), epoch).valid);
  f = Fixture{}; f.shot.direction = 1;
  EXPECT_FALSE(f.candidate().command_eligible);
  f = Fixture{};
  f.forecast.nominal.events.insert(f.forecast.nominal.events.begin(),
                                 event("edge_ambiguous", .05, .9));
  EXPECT_EQ(f.candidate().status, IncomingStatus::Unavailable);
  EXPECT_FALSE(v2_strike_target(f.candidate(), epoch).valid);
  f = Fixture{}; f.plane.selection.flight = 0;
  const auto preview = f.candidate();
  EXPECT_TRUE(preview.strike_contact_supported);
  EXPECT_FALSE(preview.command_eligible);
  EXPECT_EQ(preview.reason, "WAITING_FLIGHT_COMMIT");
}
TEST(V2StrikeAdapter, ConfirmedTerminalAndMismatchedCutoffCannotAuthorize) {
  for (auto kind : {PhysicalEventKind::NET_COLLISION, PhysicalEventKind::GROUND}) {
    Fixture f;
    ShotEvent collision;
    collision.kind = kind; collision.status = EventStatus::CONFIRMED;
    collision.generation = f.shot.generation;
    collision.event_time_ns = collision.confirmed_at_ns = epoch - 1000000;
    f.shot.events.push_back(collision);
    EXPECT_EQ(f.candidate().reason, "CONFIRMED_TERMINAL_CONTACT");
    EXPECT_FALSE(v2_strike_target(f.candidate(), epoch).valid);
  }
  Fixture f; f.shot.observations.back().source_time_ns++;
  EXPECT_EQ(f.candidate().reason, "SOURCE_PREFIX_MISMATCH");
  EXPECT_FALSE(f.candidate().command_eligible);
}
TEST(V2StrikeAdapter, IdenticalStrikePreservesDownstreamPlannerAndActualSchema45Receiver) {
  for (double y : {-.3, -.7, -1.1}) for (double speed : {3., 10., 12.})
    for (PlaneMode mode : {PlaneMode::Fixed, PlaneMode::PerBall, PlaneMode::StablePerBall}) {
      Fixture f;
      f.plane.selection.mode = mode;
      f.forecast.nominal.events.back().pre[1] = y;
      f.forecast.nominal.events.back().pre[3] = -speed;
      const auto converted = v2_strike_target(f.candidate(), epoch);
      StrikeTarget reference;
      reference.valid = true; reference.ball_position = Vec3(.15, y, .3);
      reference.ball_velocity = Vec3(-speed, 0, 1); reference.predicted_bounces = 1;
      reference.strike_source_time_s = double(epoch + 300000000) * 1e-9;
      RacketTargetPlanner planner({}, {}, {});
      const auto before = planner.plan(reference, Vec3(2.055, -.7625, .02), .7);
      const auto after = planner.plan(converted, Vec3(2.055, -.7625, .02), .7);
      ASSERT_TRUE(before.valid); ASSERT_TRUE(after.valid);
      EXPECT_EQ(before.position, after.position); EXPECT_EQ(before.velocity, after.velocity);
      EXPECT_EQ(before.normal, after.normal);
      EXPECT_EQ(before.outgoing_ball_velocity, after.outgoing_ball_velocity);
      const WireIdentity id{1, 42, 1};
      const auto expected = Schema2Packer::pack(&before, y > -.7 ? 1 : -1,
          reference.strike_source_time_s, .76, epoch, id, 20, .1);
      const auto actual = Schema2Packer::pack(&after, y > -.7 ? 1 : -1,
          converted.strike_source_time_s, .76, epoch, id, 20, .1);
      ASSERT_TRUE(actual.valid); EXPECT_EQ(actual.values, expected.values);
      std::vector<double> wire(actual.values.begin(), actual.values.end());
      wire[0] = mode == PlaneMode::Fixed ? 4 : 5;
      wire.push_back(7);
      if (mode != PlaneMode::Fixed) {
        wire.push_back(-.43); wire.push_back(static_cast<int>(mode));
      }
      a3_pingpong::PpRacketTargetInput receiver(4);
      receiver.SetFromFlat(wire);
      const auto received = receiver.Latest();
      ASSERT_TRUE(received.has_valid);
      EXPECT_EQ(received.cmd.flight_id, 42u); EXPECT_EQ(received.cmd.home_epoch, 7u);
      EXPECT_EQ(received.cmd.schema, mode == PlaneMode::Fixed ? 4 : 5);
      EXPECT_EQ(received.cmd.pos_w, after.position + Vec3(0, 0, .76));
      EXPECT_EQ(received.cmd.vel_w, after.velocity);
      a3_pingpong::PpRacketTargetInput opted_in(4,true);
      opted_in.SetExpectedHomeEpoch(7); opted_in.SetFromFlat(wire);
      auto revoked=f.candidate(); revoked.status=IncomingStatus::Revoked;
      revoked.command_sequence=1; revoked.command_flight_id=42;
      revoked.command_revision_id=1; revoked.revision=17;
      const auto invalid=managed_revocation(revoked,{2,42,1},epoch);
      ASSERT_TRUE(invalid);
      EXPECT_EQ((*invalid)[0],wire[0]); EXPECT_EQ((*invalid)[1],0.);
      opted_in.SetFromFlat(*invalid);
      EXPECT_TRUE(opted_in.Latest().flight_revoked);
      EXPECT_FALSE(opted_in.GuardEngagement(received.cmd).allowed);
    }
}
}  // namespace
