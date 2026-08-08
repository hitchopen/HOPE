#include "hope_planner_cpp/schema2_packer.hpp"
#include "hope_planner_cpp/spsc_ring.hpp"

#include <gtest/gtest.h>

namespace hope_planner_cpp {
namespace {

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
