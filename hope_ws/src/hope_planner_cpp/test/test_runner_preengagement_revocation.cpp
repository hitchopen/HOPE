#include "a3_pingpong/pp_planner_input.hpp"
#include <gtest/gtest.h>
#include <future>

namespace {
std::vector<double> packet(bool valid, unsigned sequence, unsigned flight=1,
                           unsigned revision=1, unsigned home=7, int schema=4) {
  const auto now = a3_pingpong::PpNowWallSec();
  std::vector<double> wire{double(schema), valid ? 1. : 0., 1, .08, -.7, 1.05, 2,0,1,
      valid ? .5 : 0., valid ? now+.5 : 0., 0, std::floor(now),
      std::floor((now-std::floor(now))*1e9), double(sequence), double(flight),
      double(revision), 30, .1, double(home)};
  if (schema==5) { wire.push_back(-.5); wire.push_back(1); }
  return wire;
}
TEST(RunnerRevocation, LegacyRetainsValidButOptInVetoesMatchingFlight) {
  for (int schema : {4,5}) for (bool enabled : {false,true}) {
    a3_pingpong::PpRacketTargetInput receiver(4, enabled);
    receiver.SetExpectedHomeEpoch(7);
    receiver.SetFromFlat(packet(true,1,1,1,7,schema));
    receiver.SetFromFlat(packet(false,2,1,2,7,schema));
    const auto state=receiver.Latest();
    ASSERT_TRUE(state.has_valid);
    EXPECT_TRUE(state.invalid_after);
    EXPECT_EQ(state.flight_revoked,enabled);
    auto guard=receiver.GuardEngagement(state.cmd);
    EXPECT_EQ(guard.allowed,!enabled);
  }
}
TEST(RunnerRevocation, LateValidCannotRestoreAndUnrelatedFlightRemainsAvailable) {
  for (int schema : {4,5}) {
    a3_pingpong::PpRacketTargetInput receiver(4,true); receiver.SetExpectedHomeEpoch(7);
    receiver.SetFromFlat(packet(false,2,1,2,7,schema));
    receiver.SetFromFlat(packet(true,1,1,1,7,schema));
    receiver.SetFromFlat(packet(true,3,1,3,7,schema));
    EXPECT_FALSE(receiver.Latest().has_valid);
    receiver.SetFromFlat(packet(true,4,2,1,7,schema));
    receiver.SetFromFlat(packet(false,5,1,4,7,schema));
    const auto latest=receiver.Latest();
    ASSERT_TRUE(latest.has_valid);
    EXPECT_EQ(latest.cmd.flight_id,2u);
    EXPECT_FALSE(latest.flight_revoked); EXPECT_FALSE(latest.invalid_after);
  }
}
TEST(RunnerRevocation, RejectsStaleRevisionAndCrossSessionWithoutConsumingSequence) {
  a3_pingpong::PpRacketTargetInput receiver(4,true); receiver.SetExpectedHomeEpoch(7);
  receiver.SetFromFlat(packet(true,1));
  receiver.SetFromFlat(packet(false,2,1,1));
  receiver.SetFromFlat(packet(false,999,1,2,8));
  receiver.SetFromFlat(packet(true,1000,2,1,8));
  EXPECT_EQ(receiver.Latest().seq,1u);
  EXPECT_FALSE(receiver.Latest().flight_revoked);
  receiver.SetFromFlat(packet(false,2,1,2));
  EXPECT_TRUE(receiver.Latest().flight_revoked);
  receiver.SetExpectedHomeEpoch(8);
  receiver.SetFromFlat(packet(true,1,1,1,8));
  receiver.SetFromFlat(packet(false,999,1,3,7));
  EXPECT_FALSE(receiver.Latest().flight_revoked);
  EXPECT_EQ(receiver.Latest().seq,1u);
}
TEST(RunnerRevocation, BoundedLedgerOverflowRequiresNewAuthoritativeHome) {
  a3_pingpong::PpRacketTargetInput receiver(4,true); receiver.SetExpectedHomeEpoch(7);
  receiver.SetFromFlat(packet(true,1,1000));
  for (unsigned i=1;i<=257;++i) receiver.SetFromFlat(packet(false,i+1,i,1));
  EXPECT_TRUE(receiver.Latest().flight_revoked);
  receiver.SetFromFlat(packet(true,300,1001));
  EXPECT_EQ(receiver.Latest().cmd.flight_id,1000u);
  receiver.SetExpectedHomeEpoch(8);
  receiver.SetFromFlat(packet(true,1,1,1,8));
  EXPECT_FALSE(receiver.Latest().flight_revoked);
}
TEST(RunnerRevocation, ReorderedUnrelatedStatusCannotHideCurrentFlightVeto) {
  for (int schema : {4,5}) {
    a3_pingpong::PpRacketTargetInput receiver(4,true); receiver.SetExpectedHomeEpoch(7);
    receiver.SetFromFlat(packet(true,1,1,1,7,schema));
    receiver.SetFromFlat(packet(false,3,2,2,7,schema));
    EXPECT_FALSE(receiver.Latest().flight_revoked);
    receiver.SetFromFlat(packet(false,2,1,2,7,schema));
    EXPECT_TRUE(receiver.Latest().flight_revoked);
    EXPECT_EQ(receiver.Latest().seq,3u);
    EXPECT_FALSE(receiver.GuardEngagement(receiver.Latest().cmd).allowed);
  }
}
TEST(RunnerRevocation, RevocationAndFinalEngagementHaveAnAtomicOrdering) {
  a3_pingpong::PpRacketTargetInput receiver(4,true); receiver.SetExpectedHomeEpoch(7);
  receiver.SetFromFlat(packet(true,1));
  const auto command=receiver.Latest().cmd;
  auto guard=receiver.GuardEngagement(command);
  ASSERT_TRUE(guard.allowed);
  std::promise<void> entering;
  auto invalidation=std::async(std::launch::async,[&] {
    entering.set_value(); receiver.SetFromFlat(packet(false,2,1,2));
  });
  entering.get_future().wait();
  EXPECT_EQ(invalidation.wait_for(std::chrono::milliseconds(10)), std::future_status::timeout);
  guard.lock.unlock();
  invalidation.get();
  EXPECT_TRUE(receiver.Latest().flight_revoked);
  EXPECT_FALSE(receiver.GuardEngagement(command).allowed);
}
}  // namespace
