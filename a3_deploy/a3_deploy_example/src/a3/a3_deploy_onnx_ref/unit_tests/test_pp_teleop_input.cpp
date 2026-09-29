#include "a3_pingpong/pp_teleop_input.hpp"
#include "a3_pingpong/pp_runner_control.hpp"
#include <gtest/gtest.h>

TEST(PpTeleopInput, FreshEnableEdgeAndTimeout) {
  a3_pingpong::PpTeleopInput input(42);
  std::vector<double> v = {1, 42, 7, 1, 100, 1, 0, 0, 0, 0, 1};
  ASSERT_TRUE(input.Receive(v, 100, 10));
  EXPECT_TRUE(input.Ready(100, 10));
  EXPECT_TRUE(input.Sample(true, 100, 10).isZero());
  v[3] = 2;
  v[6] = 1;
  ASSERT_TRUE(input.Receive(v, 100, 10.01));
  input.Sample(true, 100, 10.01);
  EXPECT_TRUE(input.armed());
  v[3] = 3;
  v[7] = .2;
  ASSERT_TRUE(input.Receive(v, 100, 10.02));
  EXPECT_DOUBLE_EQ(input.Sample(true, 100, 10.02)[0], .2);
  EXPECT_TRUE(input.Sample(true, 100.3, 10.3).isZero());
  EXPECT_FALSE(input.armed());
  v[3] = 4;
  v[4] = 100.31;
  ASSERT_TRUE(input.Receive(v, 100.31, 10.31));
  EXPECT_TRUE(input.Sample(true, 100.31, 10.31).isZero());
}

TEST(PpTeleopInput, RejectsWrongBootReplayDelayedAndSourceTakeover) {
  a3_pingpong::PpTeleopInput input(42);
  std::vector<double> v = {1, 42, 7, 1, 100, 1, 0, 0, 0, 0, 1};
  EXPECT_TRUE(input.Receive(v, 100, 10));
  EXPECT_FALSE(input.Receive(v, 100, 10.01));
  v[3] = 2;
  v[1] = 43;
  EXPECT_FALSE(input.Receive(v, 100, 10.02));
  v[1] = 42;
  EXPECT_FALSE(input.Receive(v, 101, 11));
  v[2] = 8;
  EXPECT_FALSE(input.Receive(v, 100, 10.03));
  v[4] = 101;
  EXPECT_TRUE(input.Receive(v, 101, 11));
  v[3] = 3;
  v[7] = .2;
  EXPECT_FALSE(input.Receive(v, 101, 11.01)); // disabled must carry zero
}

TEST(PpTeleopInput, EntryDoesNotReplayAlreadyHeldEnable) {
  a3_pingpong::PpTeleopInput input(42);
  std::vector<double> v = {1, 42, 7, 1, 100, 1, 0, 0, 0, 0, 1};
  ASSERT_TRUE(input.Receive(v, 100, 10));
  input.Sample(true, 100, 10);
  v[3] = 2;
  v[6] = 1;
  ASSERT_TRUE(input.Receive(v, 100, 10.01));
  input.Sample(false, 100, 10.01);
  input.Sample(true, 100, 10.02);
  EXPECT_FALSE(input.armed());
}

TEST(PpRunnerTeleop, AdmissionStopAndEmergencyPreserveOwnership) {
  using namespace a3_pingpong;
  PpRunnerControl c(RunnerMode::kPdStand, 42, "test");
  c.EnqueueLocalAction(RunnerAction::kEnterTeleop);
  EXPECT_EQ(c.ProcessPending(false, false)[0].result, RunnerActionResult::kRejectedTeleop);
  c.EnqueueLocalAction(RunnerAction::kEnterTeleop);
  EXPECT_EQ(c.ProcessPending(false, false, false, -1, false, -1, false, true, true)[0].result,
            RunnerActionResult::kApplied);
  EXPECT_EQ(c.mode(), RunnerMode::kTeleop);
  c.EnqueueLocalAction(RunnerAction::kEnterPdStand);
  EXPECT_TRUE(c.ProcessPending(false, false)[0].request_teleop_stop);
  EXPECT_EQ(c.mode(), RunnerMode::kTeleop);
  EXPECT_TRUE(c.TeleopStopRequested());
  c.EnqueueLocalAction(RunnerAction::kEmergencyPassive);
  c.ProcessPending(false, false);
  EXPECT_EQ(c.mode(), RunnerMode::kPassive);
}

TEST(PpTeleopInput, RejectsFutureMalformedAndOutOfRangePackets) {
  a3_pingpong::PpTeleopInput input(42);
  const std::vector<double> valid = {1, 42, 7, 1, 100, 1, 0, 0, 0, 0, 1};
  auto packet = valid;
  packet[4] = 100.051;
  EXPECT_FALSE(input.Receive(packet, 100, 10));
  for (const auto index : {5, 6, 10}) {
    packet = valid;
    packet[index] = .5;
    EXPECT_FALSE(input.Receive(packet, 100, 10));
  }
  for (const auto index : {7, 8, 9}) {
    packet = valid;
    packet[6] = 1;
    packet[index] = 2;
    EXPECT_FALSE(input.Receive(packet, 100, 10));
  }
  packet = valid;
  packet[3] = 1.5;
  EXPECT_FALSE(input.Receive(packet, 100, 10));
  packet = valid;
  packet.pop_back();
  EXPECT_FALSE(input.Receive(packet, 100, 10));
}

TEST(PpTeleopInput, DisconnectRequiresFreshNeutralReleaseBeforeMovement) {
  a3_pingpong::PpTeleopInput input(42);
  std::vector<double> packet = {1, 42, 7, 1, 100, 1, 0, 0, 0, 0, 1};
  ASSERT_TRUE(input.Receive(packet, 100, 10));
  input.Sample(true, 100, 10);
  packet[3] = 2;
  packet[6] = 1;
  ASSERT_TRUE(input.Receive(packet, 100, 10.01));
  input.Sample(true, 100, 10.01);
  ASSERT_TRUE(input.armed());
  packet[3] = 3;
  packet[5] = 0;
  packet[6] = 0;
  ASSERT_TRUE(input.Receive(packet, 100, 10.02));
  EXPECT_TRUE(input.Sample(true, 100, 10.02).isZero());
  packet[3] = 4;
  packet[5] = 1;
  packet[6] = 1;
  ASSERT_TRUE(input.Receive(packet, 100, 10.03));
  input.Sample(true, 100, 10.03);
  EXPECT_FALSE(input.armed());
  packet[3] = 5;
  packet[6] = 0;
  ASSERT_TRUE(input.Receive(packet, 100, 10.04));
  input.Sample(true, 100, 10.04);
  packet[3] = 6;
  packet[6] = 1;
  ASSERT_TRUE(input.Receive(packet, 100, 10.05));
  input.Sample(true, 100, 10.05);
  EXPECT_TRUE(input.armed());
}
