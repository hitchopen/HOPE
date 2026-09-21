#include "hope_planner_cpp/schema2_packer.hpp"
#include "hope_planner_cpp/schema3_packer.hpp"
#include "hope_planner_cpp/session_home_reference.hpp"
#include "hope_planner_cpp/spsc_ring.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <tuple>

namespace hope_planner_cpp {
namespace {

TEST(SessionHomeReference, IgnoresProcessStartupPoseAndFreezesFirstSolvePose) {
  SessionHomeReference home;
  // set_base_snapshot() no longer calls this method, so a startup origin cannot mutate state.
  EXPECT_FALSE(home.set);
  const auto unresolved = home.resolve_xy_on_solve(
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN());
  EXPECT_TRUE(std::isnan(unresolved.first));
  EXPECT_TRUE(std::isnan(unresolved.second));
  EXPECT_FALSE(home.set);
  const auto resolved = home.resolve_xy_on_solve(-0.50, -0.7625);
  EXPECT_DOUBLE_EQ(resolved.first, -0.50);
  EXPECT_DOUBLE_EQ(resolved.second, -0.7625);
  EXPECT_TRUE(home.set);
  EXPECT_DOUBLE_EQ(home.x, -0.50);
  EXPECT_DOUBLE_EQ(home.y, -0.7625);
  EXPECT_EQ(home.resolve_xy_on_solve(0.25, -0.7000), resolved);
}

TEST(SessionHomeReference, ExplicitBoundaryFreezeOwnsTargetTupleFrame) {
  SessionHomeReference home;
  EXPECT_FALSE(home.freeze_xy_at_session_boundary(
      std::numeric_limits<double>::quiet_NaN(), -0.7625));
  EXPECT_FALSE(home.set);

  ASSERT_TRUE(home.freeze_xy_at_session_boundary(-0.501641, -0.762290));
  constexpr double kFixedPlaneOffset = 0.58;
  const double x_hit = home.x + kFixedPlaneOffset;
  EXPECT_DOUBLE_EQ(x_hit - home.x, kFixedPlaneOffset);

  // The first solve sees a later drifting base, but it cannot replace the
  // explicitly frozen playing-session HOME.
  EXPECT_EQ(
      home.resolve_xy_on_solve(-0.491185, -0.793111),
      std::make_pair(-0.501641, -0.762290));
}

TEST(SessionHomeReference, Gate3PhysicalLanesSelectForehandAndBackhand) {
  SessionHomeReference home;
  // Gate3 may publish an origin pose before reset/placement; no solve means no mutation.
  EXPECT_FALSE(home.set);
  const auto home_xy = home.resolve_xy_on_solve(-0.50, -0.7625);
  const double home_y = home_xy.second;
  const double fh = select_swing_sign_with_hysteresis(
      home_y - 0.295, home_y, -0.25, 0.0, 0.0);
  const double bh = select_swing_sign_with_hysteresis(
      home_y + 0.105, home_y, -0.25, 0.0, fh);
  EXPECT_DOUBLE_EQ(fh, 1.0);
  EXPECT_DOUBLE_EQ(bh, -1.0);
  EXPECT_EQ(home.resolve_xy_on_solve(-0.25, -0.6500), home_xy);
}

TEST(SessionHomeReference, MissingSolveTimeBaseFailsClosed) {
  EXPECT_TRUE(std::isnan(select_swing_sign_with_hysteresis(
      -1.2025, std::numeric_limits<double>::quiet_NaN(),
      -0.25, 0.04, 0.0)));
}

TEST(Schema2, PreservesTheNineteenDoubleModel21800Contract) {
  Schema2Packer packer;
  const auto identity = packer.next_identity(true, 10'000'000'000LL);
  RacketCommand command;
  command.position = Vec3(0.15, -0.4, 0.2);
  command.velocity = Vec3(1.0, 2.0, 3.0);
  command.valid = true;
  constexpr double deadline = 1'785'870'000.7234569;
  const auto packet = Schema2Packer::pack(
      &command, -1.0, deadline, 0.76, 1'785'870'000'123'456'789LL,
      identity, 37, 0.1);
  ASSERT_TRUE(packet.valid);
  EXPECT_EQ(packet.values.size(), 19U);
  EXPECT_DOUBLE_EQ(packet.values[0], 2.0);
  EXPECT_DOUBLE_EQ(packet.values[1], 1.0);
  EXPECT_DOUBLE_EQ(packet.values[2], -1.0);
  EXPECT_DOUBLE_EQ(packet.values[3], 0.15);
  EXPECT_DOUBLE_EQ(packet.values[4], -0.4);
  EXPECT_DOUBLE_EQ(packet.values[5], 0.96);
  EXPECT_DOUBLE_EQ(packet.values[6], 1.0);
  EXPECT_DOUBLE_EQ(packet.values[7], 2.0);
  EXPECT_DOUBLE_EQ(packet.values[8], 3.0);
  EXPECT_NEAR(packet.values[9], 0.6, 1.0e-7);
  EXPECT_DOUBLE_EQ(packet.values[10], deadline);
  EXPECT_DOUBLE_EQ(packet.values[11], 0.0);
  EXPECT_DOUBLE_EQ(packet.values[12], 1'785'870'000.0);
  EXPECT_DOUBLE_EQ(packet.values[13], 123'456'789.0);
  EXPECT_DOUBLE_EQ(packet.values[14], 1.0);
  EXPECT_DOUBLE_EQ(packet.values[15], 1.0);
  EXPECT_DOUBLE_EQ(packet.values[16], 1.0);
  EXPECT_DOUBLE_EQ(packet.values[17], 37.0);
  EXPECT_DOUBLE_EQ(packet.values[18], 0.1);
}

TEST(Schema2, AdvancesRevisionWithinAFlightAndStartsANewFlightAfterAGap) {
  Schema2Packer packer;
  const auto first = packer.next_identity(true, 1'000'000'000LL);
  const auto second = packer.next_identity(true, 1'100'000'000LL);
  const auto invalid = packer.next_identity(false, 1'200'000'000LL);
  const auto next_flight = packer.next_identity(true, 1'400'000'001LL);

  EXPECT_EQ(first.command_sequence, 1U);
  EXPECT_EQ(first.flight_id, 1U);
  EXPECT_EQ(first.revision_id, 1U);
  EXPECT_EQ(second.command_sequence, 2U);
  EXPECT_EQ(second.flight_id, first.flight_id);
  EXPECT_EQ(second.revision_id, 2U);
  EXPECT_EQ(invalid.command_sequence, 3U);
  EXPECT_EQ(invalid.flight_id, second.flight_id);
  EXPECT_EQ(invalid.revision_id, second.revision_id);
  EXPECT_EQ(next_flight.command_sequence, 4U);
  EXPECT_EQ(next_flight.flight_id, 2U);
  EXPECT_EQ(next_flight.revision_id, 1U);
}

TEST(Schema2, InvalidPacketsZeroAllControlFieldsButKeepIdentityAndProducerTime) {
  Schema2Packer packer;
  const auto identity = packer.next_identity(false, 1);
  const auto packet = Schema2Packer::pack(
      nullptr, 1.0, 0.0, 0.76, 2'000'000'001LL,
      identity, 20, 0.1);
  EXPECT_FALSE(packet.valid);
  EXPECT_DOUBLE_EQ(packet.values[1], 0.0);
  for (int index = 2; index <= 11; ++index) {
    EXPECT_DOUBLE_EQ(packet.values[static_cast<std::size_t>(index)], 0.0);
  }
  EXPECT_DOUBLE_EQ(packet.values[12], 2.0);
  EXPECT_DOUBLE_EQ(packet.values[13], 1.0);
  EXPECT_DOUBLE_EQ(packet.values[14], 1.0);
  EXPECT_DOUBLE_EQ(packet.values[15], 0.0);
  EXPECT_DOUBLE_EQ(packet.values[16], 0.0);
  EXPECT_DOUBLE_EQ(packet.values[17], 0.0);
  EXPECT_DOUBLE_EQ(packet.values[18], 0.0);
}

TEST(Schema2, DeadlineDirectlyAccountsForAllAgeBeforePlannerPublish) {
  Schema2Packer packer;
  const auto identity = packer.next_identity(true, 1);
  RacketCommand command;
  command.position = Vec3(0.15, -0.4, 0.2);
  command.velocity = Vec3(1.0, 2.0, 3.0);
  command.valid = true;

  // The exposure-derived crossing deadline is fixed at 10.700 s. Publishing
  // at 10.200 s must put 0.500 s on the wire; no receipt-time reconstruction
  // or fixed network-latency constant participates.
  const auto packet = Schema2Packer::pack(
      &command, 1.0, 10.700, 0.76, 10'200'000'000LL,
      identity, 37, 0.1);
  ASSERT_TRUE(packet.valid);
  EXPECT_NEAR(packet.values[9], 0.500, 1.0e-12);
  EXPECT_NEAR(packet.values[10], 10.700, 1.0e-12);
  EXPECT_NEAR(
      packet.values[10] -
          (packet.values[12] + packet.values[13] * 1.0e-9),
      packet.values[9], 1.0e-12);
}

TEST(ReachPermission, MirrorsTheScreenedTrainingBankAtExactBoundaries) {
  ReachPermissionConfig config;
  config.enabled = true;
  config.contract = kScreenedReachPermissionContract;
  ASSERT_NO_THROW(validate_reach_permission_config(config));

  EXPECT_DOUBLE_EQ(
      classify_reach_permission(config, 0.725001, 1.0, 1.0, 0.60)
          .reach_level,
      0.0);
  const auto fh_level1 =
      classify_reach_permission(config, 0.725, 1.0, 1.0, 0.60);
  EXPECT_DOUBLE_EQ(fh_level1.reach_level, 1.0);
  EXPECT_DOUBLE_EQ(fh_level1.swing_foot_sign, 1.0);
  const auto fh_level2 =
      classify_reach_permission(config, 0.700, 1.0, 1.0, 0.60);
  EXPECT_DOUBLE_EQ(fh_level2.reach_level, 2.0);
  EXPECT_DOUBLE_EQ(fh_level2.swing_foot_sign, -1.0);

  const auto bh_level1 =
      classify_reach_permission(config, 1.090, 1.0, -1.0, 0.60);
  EXPECT_DOUBLE_EQ(bh_level1.reach_level, 1.0);
  EXPECT_DOUBLE_EQ(bh_level1.swing_foot_sign, -1.0);
  const auto bh_level2 =
      classify_reach_permission(config, 1.120, 1.0, -1.0, 0.60);
  EXPECT_DOUBLE_EQ(bh_level2.reach_level, 2.0);
  EXPECT_DOUBLE_EQ(bh_level2.swing_foot_sign, 1.0);
  const auto bh_right_corner =
      classify_reach_permission(config, 0.880, 1.0, -1.0, 0.60);
  EXPECT_DOUBLE_EQ(bh_right_corner.reach_level, 2.0);
  EXPECT_DOUBLE_EQ(bh_right_corner.swing_foot_sign, -1.0);

  // The sign names the actual moving foot. Level 1 unloads the target-opposite
  // foot; Level 2 moves the target-side foot. FH/BH only selects thresholds.
  for (const auto [swing_sign, level1_offset, level2_offset] :
       {std::tuple{-1.0, 0.10, 0.13}, std::tuple{1.0, 0.28, 0.31}}) {
    for (const double target_sign : {-1.0, 1.0}) {
      // The production FH side is the exact negative side of the immutable
      // -0.25 split; its positive mirror is deliberately OOD, not a second FH
      // permission. BH legitimately spans both signed lateral directions.
      if (swing_sign > 0.0 && target_sign > 0.0) continue;
      const auto level1 = classify_reach_permission(
          config, -0.7625 + target_sign * level1_offset, -0.7625,
          swing_sign, 0.60);
      const auto level2 = classify_reach_permission(
          config, -0.7625 + target_sign * level2_offset, -0.7625,
          swing_sign, 0.60);
      EXPECT_DOUBLE_EQ(level1.reach_level, 1.0);
      EXPECT_DOUBLE_EQ(level1.swing_foot_sign, -target_sign);
      EXPECT_DOUBLE_EQ(level2.reach_level, 2.0);
      EXPECT_DOUBLE_EQ(level2.swing_foot_sign, target_sign);
    }
  }
}

TEST(ReachPermission, DisabledAndMalformedInputsFailClosedToExactZero) {
  ReachPermissionConfig disabled;
  ASSERT_NO_THROW(validate_reach_permission_config(disabled));
  const auto zero = classify_reach_permission(
      disabled, -1.0, 0.0, 1.0, 5.0);
  EXPECT_DOUBLE_EQ(zero.reach_level, 0.0);
  EXPECT_DOUBLE_EQ(zero.swing_foot_sign, 0.0);

  ReachPermissionConfig enabled;
  enabled.enabled = true;
  enabled.contract = kScreenedReachPermissionContract;
  const auto short_tts = classify_reach_permission(
      enabled, -0.30, 0.0, 1.0, 0.599999);
  EXPECT_DOUBLE_EQ(short_tts.reach_level, 0.0);
  const auto missing_home = classify_reach_permission(
      enabled, -0.30, std::numeric_limits<double>::quiet_NaN(), 1.0, 0.60);
  EXPECT_DOUBLE_EQ(missing_home.reach_level, 0.0);
  EXPECT_DOUBLE_EQ(missing_home.swing_foot_sign, 0.0);
}

TEST(ReachPermission, ConfigurationDriftFailsAtStartup) {
  ReachPermissionConfig disabled_wrong_contract;
  disabled_wrong_contract.contract = kScreenedReachPermissionContract;
  EXPECT_THROW(
      validate_reach_permission_config(disabled_wrong_contract),
      std::invalid_argument);

  ReachPermissionConfig enabled_wrong_contract;
  enabled_wrong_contract.enabled = true;
  EXPECT_THROW(
      validate_reach_permission_config(enabled_wrong_contract),
      std::invalid_argument);

  ReachPermissionConfig threshold_drift;
  threshold_drift.enabled = true;
  threshold_drift.contract = kScreenedReachPermissionContract;
  threshold_drift.fh_level1_abs_y_m = 0.274;
  EXPECT_THROW(
      validate_reach_permission_config(threshold_drift),
      std::invalid_argument);
}

TEST(ReachPermission, CompleteTargetPrefilterUsesImmutableHomeXY) {
  ReachPermissionConfig config;
  config.enabled = true;
  config.contract = kScreenedReachPermissionContract;
  const Vec3 velocity(1.8, 0.2, 1.0);
  const Vec3 normal(0.8, 0.0, 0.6);
  EXPECT_TRUE(target_tuple_inside_axis_prefilter(
      config, Vec3(0.08, -1.0575, 1.10), velocity, normal,
      -0.50, -0.7625, 1.0));
  EXPECT_TRUE(target_tuple_inside_axis_prefilter(
      config, Vec3(0.78, 0.705, 1.10), velocity, normal,
      0.20, 1.0, 1.0));
  EXPECT_TRUE(target_tuple_inside_axis_prefilter(
      config, Vec3(0.7815, 0.705, 1.10), velocity, normal,
      0.20, 1.0, 1.0));
  EXPECT_FALSE(target_tuple_inside_axis_prefilter(
      config, Vec3(0.7816, 0.705, 1.10), velocity, normal,
      0.20, 1.0, 1.0));
  EXPECT_FALSE(target_tuple_inside_axis_prefilter(
      config, Vec3(0.08, 0.705, 1.10), velocity, normal,
      0.20, 1.0, 1.0));
  EXPECT_FALSE(target_tuple_inside_axis_prefilter(
      config, Vec3(0.78, 0.705, 1.27), velocity, normal,
      0.20, 1.0, 1.0));
}

TEST(ReachPermission, RevisionCannotCrossGeometryCellOrMovingFoot) {
  EXPECT_TRUE(revision_preserves_reach_geometry_cell(
      ReachPermission{1.0, 1.0}, ReachPermission{1.0, 1.0}));
  EXPECT_FALSE(revision_preserves_reach_geometry_cell(
      ReachPermission{1.0, 1.0}, ReachPermission{2.0, -1.0}));
  EXPECT_FALSE(revision_preserves_reach_geometry_cell(
      ReachPermission{1.0, 1.0}, ReachPermission{1.0, -1.0}));
  EXPECT_FALSE(revision_preserves_reach_geometry_cell(
      ReachPermission{1.0, 1.0}, ReachPermission{0.0, 0.0, false}));
}

TEST(Schema3, PreservesSchema2FieldsAndAppendsReachPermission) {
  Schema2Packer packer;
  const auto identity = packer.next_identity(true, 10'000'000'000LL);
  RacketCommand command;
  command.position = Vec3(0.15, -0.4, 0.2);
  command.velocity = Vec3(1.0, 2.0, 3.0);
  command.valid = true;
  constexpr double deadline = 1'785'870'000.7234569;
  constexpr std::int64_t producer_ns = 1'785'870'000'123'456'789LL;
  const auto schema2 = Schema2Packer::pack(
      &command, -1.0, deadline, 0.76, producer_ns, identity, 37, 0.1);
  const auto schema3 = Schema3Packer::pack(
      &command, -1.0, deadline, 0.76, producer_ns, identity, 37, 0.1,
      ReachPermission{2.0, 1.0});
  ASSERT_TRUE(schema3.valid);
  ASSERT_EQ(schema2.values.size(), 19U);
  ASSERT_EQ(schema3.values.size(), 21U);
  EXPECT_DOUBLE_EQ(schema3.values[0], 3.0);
  for (std::size_t index = 1; index < schema2.values.size(); ++index) {
    EXPECT_DOUBLE_EQ(schema3.values[index], schema2.values[index]);
  }
  EXPECT_DOUBLE_EQ(schema3.values[19], 2.0);
  EXPECT_DOUBLE_EQ(schema3.values[20], 1.0);
}

TEST(Schema3, InvalidPacketAlwaysZerosPermissionFields) {
  Schema2Packer packer;
  const auto identity = packer.next_identity(false, 1);
  const auto packet = Schema3Packer::pack(
      nullptr, 1.0, 0.0, 0.76, 2'000'000'001LL, identity, 20, 0.1,
      ReachPermission{2.0, -1.0});
  EXPECT_FALSE(packet.valid);
  EXPECT_DOUBLE_EQ(packet.values[0], 3.0);
  EXPECT_DOUBLE_EQ(packet.values[1], 0.0);
  EXPECT_DOUBLE_EQ(packet.values[19], 0.0);
  EXPECT_DOUBLE_EQ(packet.values[20], 0.0);
}

TEST(Schema3, DisabledValidProducerCarriesExactZeroPermissionTuple) {
  ReachPermissionConfig disabled;
  const auto permission = classify_reach_permission(
      disabled, -0.35, 0.0, 1.0, 0.80);
  Schema2Packer packer;
  const auto identity = packer.next_identity(true, 1);
  RacketCommand command;
  command.position = Vec3(0.15, -0.35, 0.2);
  command.velocity = Vec3(1.0, 2.0, 3.0);
  command.valid = true;
  const auto packet = Schema3Packer::pack(
      &command, 1.0, 10.8, 0.76, 10'000'000'000LL, identity, 20, 0.1,
      permission);
  ASSERT_TRUE(packet.valid);
  EXPECT_DOUBLE_EQ(packet.values[19], 0.0);
  EXPECT_DOUBLE_EQ(packet.values[20], 0.0);
}

TEST(SpscRing, BoundedSingleProducerSingleConsumerSemantics) {
  SpscRing<int, 4> ring;
  EXPECT_TRUE(ring.try_push(1));
  EXPECT_TRUE(ring.try_push(2));
  EXPECT_TRUE(ring.try_push(3));
  EXPECT_FALSE(ring.try_push(4));
  int value = 0;
  EXPECT_TRUE(ring.try_pop(value));
  EXPECT_EQ(value, 1);
  EXPECT_TRUE(ring.try_push(4));
  EXPECT_TRUE(ring.try_pop(value));
  EXPECT_EQ(value, 2);
  EXPECT_TRUE(ring.try_pop(value));
  EXPECT_EQ(value, 3);
  EXPECT_TRUE(ring.try_pop(value));
  EXPECT_EQ(value, 4);
  EXPECT_FALSE(ring.try_pop(value));
}

}  // namespace
}  // namespace hope_planner_cpp
