#include "a3_pingpong/pp_operator_heartbeat.hpp"
#include <gtest/gtest.h>
namespace {
using namespace a3_pingpong;
TEST(PpOperatorHeartbeat, IndependentRolesAndNoReplayOrDelayedRenewal) {
  PpOperatorHeartbeat heartbeat(42, "test");
  auto challenge = heartbeat.Issue(10.);
  auto ui = challenge; ui.push_back(1.);
  auto computer = challenge; computer.push_back(2.);
  EXPECT_EQ(heartbeat.Fresh(10.001), (std::array<bool,2>{false,false}));
  EXPECT_TRUE(heartbeat.Echo(ui, 10.001));
  EXPECT_EQ(heartbeat.Fresh(10.002), (std::array<bool,2>{true,false}));
  EXPECT_TRUE(heartbeat.Echo(computer, 10.090));
  EXPECT_FALSE(heartbeat.Echo(ui, 10.099));
  EXPECT_EQ(heartbeat.Fresh(10.101), (std::array<bool,2>{true,true}));
  EXPECT_EQ(heartbeat.Fresh(11.001), (std::array<bool,2>{false,false}));
  for (int field : {1,2,3}) {
    auto wrong = ui; wrong[field] += 1;
    EXPECT_FALSE(heartbeat.Echo(wrong, 10.099));
  }
  auto wrong = ui; wrong[3] = NAN;
  EXPECT_FALSE(heartbeat.Echo(wrong, 10.099));
}
TEST(PpOperatorHeartbeat, BothRolesExpireOnlyAfterOneSecond) {
  PpOperatorHeartbeat heartbeat(42, "test");
  auto ui = heartbeat.Issue(0.); auto computer = ui;
  ui.push_back(1.); computer.push_back(2.);
  ASSERT_TRUE(heartbeat.Echo(ui, .080));
  ASSERT_TRUE(heartbeat.Echo(computer, .080));
  EXPECT_EQ(heartbeat.Fresh(.100), (std::array<bool,2>{true,true}));
  EXPECT_EQ(heartbeat.Fresh(.101), (std::array<bool,2>{true,true}));
  EXPECT_EQ(heartbeat.Fresh(1.), (std::array<bool,2>{true,true}));
  EXPECT_EQ(heartbeat.Fresh(1.001), (std::array<bool,2>{false,false}));
}
TEST(PpOperatorHeartbeat, DelayedEchoesUseFullDeadlineButCannotRenewPastIssueTime) {
  PpOperatorHeartbeat heartbeat(42, "test");
  auto ui = heartbeat.Issue(10.); auto computer = ui;
  ui.push_back(1.); computer.push_back(2.);
  // At 50 Hz the challenge history must cover the roles' whole one-second window.
  for (int i = 1; i <= 45; ++i) heartbeat.Issue(10. + i*.020);
  EXPECT_TRUE(heartbeat.Echo(ui, 10.950));
  EXPECT_TRUE(heartbeat.Echo(computer, 10.950));
  EXPECT_FALSE(heartbeat.Echo(ui, 10.960));
  EXPECT_FALSE(heartbeat.Echo(computer, 10.960));
  EXPECT_EQ(heartbeat.Fresh(10.999), (std::array<bool,2>{true,true}));
  EXPECT_EQ(heartbeat.Fresh(11.001), (std::array<bool,2>{false,false}));
  auto expired = heartbeat.Issue(12.); expired.push_back(1.);
  EXPECT_FALSE(heartbeat.Echo(expired, 13.001));
  expired.back() = 2.;
  EXPECT_FALSE(heartbeat.Echo(expired, 13.001));
}
TEST(PpOperatorHeartbeat, LiveComputerCannotHideFrozenUiAfterItsOwnDeadline) {
  PpOperatorHeartbeat heartbeat(42, "test");
  PpRunnerControl control(RunnerMode::kMotion, 42, "test");
  control.RequireOperatorHeartbeat(true);
  int stops = 0;
  for (int i = 0; i < 150; ++i) {
    const double now = 10. + i*.020;
    auto ui = heartbeat.Issue(now); auto computer = ui;
    ui.push_back(1.); computer.push_back(2.);
    if (i < 25) ASSERT_TRUE(heartbeat.Echo(ui, now+.005));
    ASSERT_TRUE(heartbeat.Echo(computer, now+.005));
    const auto fresh = heartbeat.Fresh(now+.010);
    if (control.ObserveOperatorHeartbeat(fresh[0], fresh[1])) ++stops;
    if (i < 74) EXPECT_EQ(control.mode(), RunnerMode::kMotion);
    if (i > 75) EXPECT_EQ(control.mode(), RunnerMode::kPdStand);
  }
  EXPECT_EQ(stops, 1);
  EXPECT_EQ(control.EncodeMocapSafety()[13], 36.);
}
TEST(PpOperatorHeartbeat, FiftyHzRoundTripsAndFrozenComputerCannotBeHiddenByLiveUi) {
  PpOperatorHeartbeat heartbeat(42, "test");
  for (int i = 0; i < 150; ++i) {
    const double now = 10. + i*.02;
    auto ui = heartbeat.Issue(now); auto computer = ui;
    ui.push_back(1.); computer.push_back(2.);
    EXPECT_TRUE(heartbeat.Echo(ui, now+.005));
    if (i < 50) EXPECT_TRUE(heartbeat.Echo(computer, now+.005));
    const auto live = heartbeat.Fresh(now+.010);
    EXPECT_TRUE(live[0]);
    if (i < 99) EXPECT_TRUE(live[1]);
    if (i >= 99) EXPECT_FALSE(live[1]);
  }
}
TEST(PpOperatorHeartbeat, EitherLinkLossStopsNormalPlayAndRequiresExplicitReentry) {
  for (bool lose_ui : {false, true}) {
    PpRunnerControl control(RunnerMode::kMotion, 1, "test");
    control.RequireOperatorHeartbeat(true);
    control.ObserveOperatorHeartbeat(true, true);
    EXPECT_TRUE(control.ObserveOperatorHeartbeat(!lose_ui, lose_ui));
    EXPECT_EQ(control.mode(), RunnerMode::kPdStand);
    EXPECT_FALSE(control.SetRuntimeMode(RunnerMode::kMotion));
    control.EnqueueLocalAction(RunnerAction::kEnterMotion);
    EXPECT_EQ(control.ProcessPending(false, false)[0].reason,
        lose_ui ? RunnerActionReason::kOperatorUiUnavailable : RunnerActionReason::kOperatorComputerUnavailable);
    control.ObserveOperatorHeartbeat(true, true);
    EXPECT_EQ(control.mode(), RunnerMode::kPdStand);
    EXPECT_TRUE(control.SetRuntimeMode(RunnerMode::kMotion));
  }
}
TEST(PpOperatorHeartbeat, ServeUsesAbortRecoveryAndExemptModesRemainAvailable) {
  PpRunnerControl serve(RunnerMode::kServe, 1, "test");
  serve.RequireOperatorHeartbeat(true);
  EXPECT_TRUE(serve.ObserveOperatorHeartbeat(false, true));
  EXPECT_TRUE(serve.PlayStopRequested());
  serve.ObserveOperatorHeartbeat(true, true);
  EXPECT_TRUE(serve.CompleteServe());
  EXPECT_EQ(serve.mode(), RunnerMode::kPdStand);
  for (auto mode : {RunnerMode::kPassive, RunnerMode::kPdStand, RunnerMode::kTeleop}) {
    PpRunnerControl control(mode, 1, "test");
    control.RequireOperatorHeartbeat(true);
    EXPECT_FALSE(control.ObserveOperatorHeartbeat(false, false));
    EXPECT_EQ(control.mode(), mode);
  }
  PpRunnerControl offline(RunnerMode::kServe, 1, "test", 16, true);
  EXPECT_FALSE(offline.ObserveOperatorHeartbeat(false, false));
  EXPECT_FALSE(offline.PlayStopRequested());
}
}  // namespace
