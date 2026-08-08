// Copyright (c) 2026, AgiBot Inc. All rights reserved.
// 对应 notes/a3_backend_plan.md §PR 8
//
// Unit tests for PR 8: ExpandToBackend, BuildSafeHaltCommand, A3Watchdog
// (pure-function Part A) + A3PolicyDriver end-to-end with A3AimrtBackend in
// ENABLE_A3_AIMRT_BACKEND=OFF (sync-only) mode (Part B).
#include <gtest/gtest.h>

#include "a3_deploy/a3_policy_driver.hpp"
#include "a3_deploy/a3_watchdog.hpp"
#include "a3_deploy/expand_to_backend.hpp"
#include "a3_deploy/safe_halt.hpp"
#include "a3_policy_parameters.hpp"
#include "robot_io/a3_aimrt_backend.hpp"
#include "robot_io/a3_layout_extra.hpp"
#include "robot_io/robot_io_backend.hpp"

#include <atomic>
#include <chrono>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

using a3_deploy::A3PolicyDriver;
using a3_deploy::A3PolicyDriverOptions;
using a3_deploy::A3Watchdog;
using a3_deploy::BuildSafeHaltCommand;
using a3_deploy::ExpandToBackend;
using a3_deploy::PolicyFn;
using a3_deploy::WatchdogConfig;
using a3_deploy::WatchdogVerdict;

using robot_io::A3AimrtBackend;
using robot_io::RobotCommand;
using robot_io::RobotState;

namespace {

std::int64_t NowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

std::array<double, 29> MakeLinearArray(double base, double step) {
  std::array<double, 29> a{};
  for (int i = 0; i < 29; ++i) a[i] = base + step * i;
  return a;
}

RobotState MakeState(bool complete,
                     bool aligned,
                     std::int64_t timestamp_ns = 0,
                     std::int64_t ready_ns = 0) {
  if (timestamp_ns <= 0) timestamp_ns = NowNs();
  if (ready_ns <= 0) ready_ns = NowNs();
  RobotState state;
  state.timestamp_ns = timestamp_ns;
  state.state_data_ready_ns = ready_ns;
  state.state_sync_ready_ns = ready_ns;
  state.tick = 1;
  state.sync_complete = complete;
  state.sync_aligned = aligned;
  state.sync_skew_ns = aligned ? 0 : 20'000'000;
  state.q = Eigen::VectorXd::Zero(robot_io::kA3Dof);
  state.dq = Eigen::VectorXd::Zero(robot_io::kA3Dof);
  state.tau_est = Eigen::VectorXd::Zero(robot_io::kA3Dof);
  return state;
}

template <typename Predicate>
bool WaitUntil(Predicate pred, std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return pred();
}

class CapturingBackend final : public robot_io::RobotIOBackend {
 public:
  bool Init(const std::string&) override { return true; }
  bool Start() override { return true; }
  void Stop() override {}

  const robot_io::JointLayout& GetLayout() const override {
    return robot_io::MakeA3Layout31();
  }

  void RegisterStateCallback(StateCallback cb) override {
    cb_ = std::move(cb);
  }

  bool SendCommand(const RobotCommand& cmd) override {
    std::lock_guard<std::mutex> lk(mu_);
    commands_.push_back(cmd);
    return true;
  }

  std::string Name() const override { return "capturing"; }
  double StateRateHz() const override { return 200.0; }

  void Emit(const RobotState& state) {
    if (cb_) cb_(state);
  }

  std::size_t CommandCount() const {
    std::lock_guard<std::mutex> lk(mu_);
    return commands_.size();
  }

  RobotCommand LastCommand() const {
    std::lock_guard<std::mutex> lk(mu_);
    return commands_.empty() ? RobotCommand{} : commands_.back();
  }

 private:
  StateCallback cb_{};
  mutable std::mutex mu_;
  std::vector<RobotCommand> commands_;
};

}  // namespace

// =============================================================================
// Part A — pure functions
// =============================================================================

TEST(ExpandToBackend, ScattersBodyToNonNeckSlotsAndControlsHead) {
  // Post 2026-04 realignment (notes/a3_dof_orderings.md): the 29-DOF
  // policy view scatters into the 29 non-neck slots of the 31-DOF SDK
  // layout via kA3PolicyToSdkIdx[]. Head/neck slots [3..4] get fixed PD.
  auto q   = MakeLinearArray(0.1, 0.01);
  auto kps = MakeLinearArray(10.0, 1.0);
  auto kds = MakeLinearArray(1.0, 0.1);
  RobotCommand cmd;
  ExpandToBackend(q, kps, kds, cmd);

  ASSERT_EQ(cmd.q_des.size(), 31);
  ASSERT_EQ(cmd.dq_des.size(), 31);
  ASSERT_EQ(cmd.tau_ff.size(), 31);
  ASSERT_EQ(cmd.kp.size(), 31);
  ASSERT_EQ(cmd.kd.size(), 31);

  // Non-neck slots: scattered via kA3PolicyToSdkIdx.
  for (int i = 0; i < robot_io::kA3PolicyDof; ++i) {
    const int sdk = robot_io::kA3PolicyToSdkIdx[i];
    EXPECT_DOUBLE_EQ(cmd.q_des[sdk], q[i])
        << "policy i=" << i << " sdk=" << sdk;
    EXPECT_DOUBLE_EQ(cmd.kp[sdk],    kps[i]);
    EXPECT_DOUBLE_EQ(cmd.kd[sdk],    kds[i]);
    EXPECT_DOUBLE_EQ(cmd.dq_des[sdk], 0.0);
    EXPECT_DOUBLE_EQ(cmd.tau_ff[sdk], 0.0);
  }
  // Head/neck slots [3..4] — fixed zero-position PD command.
  for (int i = robot_io::kA3NeckStart;
       i < robot_io::kA3NeckStart + robot_io::kA3NeckCount; ++i) {
    EXPECT_DOUBLE_EQ(cmd.q_des[i], a3_deploy::kA3HeadTargetPositionRad);
    EXPECT_DOUBLE_EQ(cmd.dq_des[i], 0.0);
    EXPECT_DOUBLE_EQ(cmd.tau_ff[i], 0.0);
    EXPECT_DOUBLE_EQ(cmd.kp[i], a3_deploy::kA3HeadKp);
    EXPECT_DOUBLE_EQ(cmd.kd[i], a3_deploy::kA3HeadKd);
  }
}

TEST(ExpandToBackend, ReusableOutputIsOverwritten) {
  auto q   = MakeLinearArray(0.0, 0.0);     // all zeros
  auto kps = MakeLinearArray(5.0, 0.0);     // all 5.0
  auto kds = MakeLinearArray(0.5, 0.0);     // all 0.5
  RobotCommand cmd;
  // Pre-fill with garbage so we know it gets overwritten.
  cmd.q_des  = Eigen::VectorXd::Constant(31, 999.0);
  cmd.dq_des = Eigen::VectorXd::Constant(31, 999.0);
  cmd.tau_ff = Eigen::VectorXd::Constant(31, 999.0);
  cmd.kp     = Eigen::VectorXd::Constant(31, 999.0);
  cmd.kd     = Eigen::VectorXd::Constant(31, 999.0);

  ExpandToBackend(q, kps, kds, cmd);

  // All 31 slots should be overwritten: non-neck slots get (0, 5.0, 0.5);
  // head/neck slots get the fixed zero-position PD command.
  for (int i = 0; i < 31; ++i) {
    EXPECT_DOUBLE_EQ(cmd.q_des[i],  0.0);
    EXPECT_DOUBLE_EQ(cmd.dq_des[i], 0.0);
    EXPECT_DOUBLE_EQ(cmd.tau_ff[i], 0.0);
  }
  for (int i = 0; i < robot_io::kA3PolicyDof; ++i) {
    const int sdk = robot_io::kA3PolicyToSdkIdx[i];
    EXPECT_DOUBLE_EQ(cmd.kp[sdk], 5.0);
    EXPECT_DOUBLE_EQ(cmd.kd[sdk], 0.5);
  }
  for (int i = robot_io::kA3NeckStart;
       i < robot_io::kA3NeckStart + robot_io::kA3NeckCount; ++i) {
    EXPECT_DOUBLE_EQ(cmd.kp[i], a3_deploy::kA3HeadKp);
    EXPECT_DOUBLE_EQ(cmd.kd[i], a3_deploy::kA3HeadKd);
  }
}

TEST(BuildSafeHalt, Copies31DofQAndZeroesEverythingElse) {
  Eigen::VectorXd q = Eigen::VectorXd::Zero(31);
  for (int i = 0; i < 31; ++i) q[i] = 0.5 * i;

  RobotCommand cmd;
  BuildSafeHaltCommand(q, cmd);

  ASSERT_EQ(cmd.q_des.size(), 31);
  ASSERT_EQ(cmd.dq_des.size(), 31);
  ASSERT_EQ(cmd.tau_ff.size(), 31);
  ASSERT_EQ(cmd.kp.size(), 31);
  ASSERT_EQ(cmd.kd.size(), 31);

  for (int i = 0; i < 31; ++i) {
    EXPECT_DOUBLE_EQ(cmd.q_des[i], q[i]);
    EXPECT_DOUBLE_EQ(cmd.dq_des[i], 0.0);
    EXPECT_DOUBLE_EQ(cmd.tau_ff[i], 0.0);
    EXPECT_DOUBLE_EQ(cmd.kp[i], 0.0);
    EXPECT_DOUBLE_EQ(cmd.kd[i], 0.0);
  }
}

TEST(BuildSafeHalt, WrongSizeInputYieldsZeros) {
  Eigen::VectorXd q = Eigen::VectorXd::Constant(29, 3.0);  // not 31
  RobotCommand cmd;
  BuildSafeHaltCommand(q, cmd);
  ASSERT_EQ(cmd.q_des.size(), 31);
  for (int i = 0; i < 31; ++i) {
    EXPECT_DOUBLE_EQ(cmd.q_des[i], 0.0);
    EXPECT_DOUBLE_EQ(cmd.kp[i], 0.0);
    EXPECT_DOUBLE_EQ(cmd.kd[i], 0.0);
  }
}

// ---------------------------------------------------------------------------
TEST(A3Watchdog, FreshAlignedFrameIsOk) {
  A3Watchdog wd{WatchdogConfig{}};
  const std::int64_t now = 1'000'000'000;
  const std::int64_t ts  = now - 10'000'000;  // 10ms old
  EXPECT_EQ(wd.Check(now, ts, /*aligned=*/true), WatchdogVerdict::Ok);
  EXPECT_EQ(wd.CurrentUnalignedStreak(), 0);
  EXPECT_EQ(wd.StaleCount(), 0u);
  EXPECT_EQ(wd.UnalignedCount(), 0u);
}

TEST(A3Watchdog, StaleFrameDetected) {
  WatchdogConfig cfg{};
  cfg.max_frame_age_ns = 50'000'000;  // 50ms
  A3Watchdog wd(cfg);
  const std::int64_t now = 1'000'000'000;
  const std::int64_t ts  = now - 100'000'000;  // 100ms old
  EXPECT_EQ(wd.Check(now, ts, true), WatchdogVerdict::StaleFrame);
  EXPECT_EQ(wd.StaleCount(), 1u);
  // A stale but aligned=true frame still clears streak.
  EXPECT_EQ(wd.CurrentUnalignedStreak(), 0);
}

TEST(A3Watchdog, ChronicUnalignedAfterThreshold) {
  WatchdogConfig cfg{};
  cfg.max_consecutive_unaligned = 5;
  A3Watchdog wd(cfg);
  const std::int64_t now = 1'000'000'000;
  const std::int64_t ts  = now;  // fresh

  // Ticks 1..4 -> Ok, streak 1..4.
  for (int i = 1; i <= 4; ++i) {
    EXPECT_EQ(wd.Check(now, ts, /*aligned=*/false), WatchdogVerdict::Ok)
        << "tick " << i;
    EXPECT_EQ(wd.CurrentUnalignedStreak(), i);
  }
  // Tick 5 -> ChronicUnaligned.
  EXPECT_EQ(wd.Check(now, ts, false), WatchdogVerdict::ChronicUnaligned);
  EXPECT_EQ(wd.UnalignedCount(), 1u);
  EXPECT_EQ(wd.CurrentUnalignedStreak(), 5);
}

TEST(A3Watchdog, AlignedFrameResetsStreak) {
  A3Watchdog wd{WatchdogConfig{}};
  const std::int64_t now = 1'000'000'000;
  const std::int64_t ts  = now;
  for (int i = 0; i < 3; ++i) wd.Check(now, ts, false);
  EXPECT_EQ(wd.CurrentUnalignedStreak(), 3);
  EXPECT_EQ(wd.Check(now, ts, /*aligned=*/true), WatchdogVerdict::Ok);
  EXPECT_EQ(wd.CurrentUnalignedStreak(), 0);
}

TEST(A3Watchdog, ResetClearsStreak) {
  A3Watchdog wd{WatchdogConfig{}};
  const std::int64_t now = 1'000'000'000;
  const std::int64_t ts  = now;
  for (int i = 0; i < 4; ++i) wd.Check(now, ts, false);
  EXPECT_EQ(wd.CurrentUnalignedStreak(), 4);
  wd.Reset();
  EXPECT_EQ(wd.CurrentUnalignedStreak(), 0);
}

TEST(A3Watchdog, StaleDoesNotResetStreakSoBothCanCooccur) {
  WatchdogConfig cfg{};
  cfg.max_consecutive_unaligned = 3;
  cfg.max_frame_age_ns = 10'000'000;  // 10ms
  A3Watchdog wd(cfg);
  const std::int64_t now = 1'000'000'000;
  const std::int64_t fresh_ts = now - 1'000'000;   // 1ms
  const std::int64_t stale_ts = now - 100'000'000; // 100ms

  // Build up streak w/ fresh+unaligned.
  wd.Check(now, fresh_ts, false);
  wd.Check(now, fresh_ts, false);
  EXPECT_EQ(wd.CurrentUnalignedStreak(), 2);
  // Stale+unaligned next tick: StaleFrame wins the verdict, streak still
  // increments to 3 (does NOT reset).
  EXPECT_EQ(wd.Check(now, stale_ts, false), WatchdogVerdict::StaleFrame);
  EXPECT_EQ(wd.CurrentUnalignedStreak(), 3);
}

TEST(A3PolicyDriver, RunsPolicyThroughTransientUnalignedFrames) {
  CapturingBackend backend;
  ASSERT_TRUE(backend.Init(""));

  std::atomic<int> policy_calls{0};
  auto counting_policy = [&](std::uint64_t /*tick_idx*/,
                             const RobotState& /*state*/,
                             std::array<double, 29>& out) {
    policy_calls.fetch_add(1, std::memory_order_relaxed);
    for (int i = 0; i < 29; ++i) out[i] = 0.2 + 0.01 * i;
  };

  A3PolicyDriverOptions opt;
  opt.policy_hz = 100.0;
  opt.watchdog.max_frame_age_ns = 500'000'000;  // keep this test about alignment
  opt.watchdog.max_consecutive_unaligned = 1000;

  A3PolicyDriver driver(backend, counting_policy, opt);
  ASSERT_TRUE(driver.StartDriver());

  RobotState state;
  state.timestamp_ns = NowNs();
  state.tick = 1;
  state.sync_complete = true;
  state.sync_aligned = false;
  state.sync_skew_ns = 20'000'000;
  state.q = Eigen::VectorXd::Zero(robot_io::kA3Dof);
  state.dq = Eigen::VectorXd::Zero(robot_io::kA3Dof);
  state.tau_est = Eigen::VectorXd::Zero(robot_io::kA3Dof);
  backend.Emit(state);

  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  driver.StopDriver();

  EXPECT_GT(policy_calls.load(std::memory_order_relaxed), 0);
  EXPECT_GT(driver.PolicyTickCount(), 0u);
  EXPECT_EQ(driver.SafeHaltCount(), 0u);
  EXPECT_GT(driver.Watchdog().CurrentUnalignedStreak(), 0);
  ASSERT_GT(backend.CommandCount(), 0u);

  const auto cmd = backend.LastCommand();
  ASSERT_EQ(cmd.kp.size(), robot_io::kA3Dof);
  bool saw_nonzero_kp = false;
  for (int i = 0; i < robot_io::kA3Dof; ++i) {
    if (cmd.kp[i] != 0.0) {
      saw_nonzero_kp = true;
      break;
    }
  }
  EXPECT_TRUE(saw_nonzero_kp)
      << "transient unaligned frames should still publish policy PD commands";
}

TEST(A3PolicyDriver, SafeHaltsWhenFrameIsIncomplete) {
  CapturingBackend backend;
  ASSERT_TRUE(backend.Init(""));

  std::atomic<int> policy_calls{0};
  auto counting_policy = [&](std::uint64_t /*tick_idx*/,
                             const RobotState& /*state*/,
                             std::array<double, 29>& out) {
    policy_calls.fetch_add(1, std::memory_order_relaxed);
    out.fill(0.2);
  };

  A3PolicyDriverOptions opt;
  opt.policy_hz = 100.0;
  opt.watchdog.max_frame_age_ns = 500'000'000;
  opt.watchdog.max_consecutive_unaligned = 1000;

  A3PolicyDriver driver(backend, counting_policy, opt);
  ASSERT_TRUE(driver.StartDriver());

  RobotState state;
  state.timestamp_ns = NowNs();
  state.tick = 1;
  state.sync_complete = false;
  state.sync_aligned = false;
  state.q = Eigen::VectorXd::Zero(robot_io::kA3Dof);
  state.dq = Eigen::VectorXd::Zero(robot_io::kA3Dof);
  state.tau_est = Eigen::VectorXd::Zero(robot_io::kA3Dof);
  backend.Emit(state);

  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  driver.StopDriver();

  EXPECT_EQ(policy_calls.load(std::memory_order_relaxed), 0);
  EXPECT_EQ(driver.PolicyTickCount(), 0u);
  EXPECT_GT(driver.SafeHaltCount(), 0u);
  ASSERT_GT(backend.CommandCount(), 0u);

  const auto cmd = backend.LastCommand();
  ASSERT_EQ(cmd.kp.size(), robot_io::kA3Dof);
  EXPECT_TRUE(cmd.kp.isZero(0.0));
  EXPECT_TRUE(cmd.kd.isZero(0.0));
}

TEST(A3PolicyDriver, TriggerOnStateRunsPolicyFromCompleteStateEvent) {
  CapturingBackend backend;
  ASSERT_TRUE(backend.Init(""));

  std::atomic<int> policy_calls{0};
  auto counting_policy = [&](std::uint64_t /*tick_idx*/,
                             const RobotState& /*state*/,
                             std::array<double, 29>& out) {
    policy_calls.fetch_add(1, std::memory_order_relaxed);
    out.fill(0.2);
  };

  A3PolicyDriverOptions opt;
  opt.policy_hz = 50.0;
  opt.trigger_on_state = true;
  opt.trigger_min_period_ns = 20'000'000;
  opt.watchdog.max_frame_age_ns = 500'000'000;
  opt.watchdog.max_consecutive_unaligned = 1000;

  A3PolicyDriver driver(backend, counting_policy, opt);
  ASSERT_TRUE(driver.StartDriver());

  EXPECT_FALSE(WaitUntil([&] { return backend.CommandCount() > 0; },
                         std::chrono::milliseconds(30)))
      << "event-triggered mode should not run the periodic timer";

  backend.Emit(MakeState(/*complete=*/true, /*aligned=*/true));

  EXPECT_TRUE(WaitUntil([&] { return backend.CommandCount() >= 1; },
                        std::chrono::milliseconds(100)));
  driver.StopDriver();

  EXPECT_EQ(policy_calls.load(std::memory_order_relaxed), 1);
  EXPECT_EQ(driver.PolicyTickCount(), 1u);
  EXPECT_EQ(driver.SafeHaltCount(), 0u);
}

TEST(A3PolicyDriver, TriggerOnStateThrottlesHighRateEvents) {
  CapturingBackend backend;
  ASSERT_TRUE(backend.Init(""));

  std::atomic<int> policy_calls{0};
  auto counting_policy = [&](std::uint64_t /*tick_idx*/,
                             const RobotState& /*state*/,
                             std::array<double, 29>& out) {
    policy_calls.fetch_add(1, std::memory_order_relaxed);
    out.fill(0.2);
  };

  A3PolicyDriverOptions opt;
  opt.policy_hz = 50.0;
  opt.trigger_on_state = true;
  opt.trigger_min_period_ns = 20'000'000;
  opt.watchdog.max_frame_age_ns = 500'000'000;
  opt.watchdog.max_consecutive_unaligned = 1000;

  A3PolicyDriver driver(backend, counting_policy, opt);
  ASSERT_TRUE(driver.StartDriver());

  const auto base_ns = NowNs();
  backend.Emit(MakeState(/*complete=*/true, /*aligned=*/true, base_ns));
  ASSERT_TRUE(WaitUntil([&] { return backend.CommandCount() >= 1; },
                        std::chrono::milliseconds(100)));

  for (int i = 1; i <= 10; ++i) {
    backend.Emit(MakeState(/*complete=*/true, /*aligned=*/true,
                           base_ns + i * 1'000'000));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  EXPECT_EQ(policy_calls.load(std::memory_order_relaxed), 1);
  EXPECT_EQ(backend.CommandCount(), 1u);

  backend.Emit(MakeState(/*complete=*/true, /*aligned=*/true,
                         base_ns + 25'000'000));
  EXPECT_TRUE(WaitUntil([&] { return backend.CommandCount() >= 2; },
                        std::chrono::milliseconds(100)));
  driver.StopDriver();

  EXPECT_EQ(policy_calls.load(std::memory_order_relaxed), 2);
  EXPECT_EQ(driver.PolicyTickCount(), 2u);
  EXPECT_EQ(driver.SafeHaltCount(), 0u);
}

TEST(A3PolicyDriver, TriggerOnStateIgnoresIncompleteStateEvents) {
  CapturingBackend backend;
  ASSERT_TRUE(backend.Init(""));

  std::atomic<int> policy_calls{0};
  auto counting_policy = [&](std::uint64_t /*tick_idx*/,
                             const RobotState& /*state*/,
                             std::array<double, 29>& out) {
    policy_calls.fetch_add(1, std::memory_order_relaxed);
    out.fill(0.2);
  };

  A3PolicyDriverOptions opt;
  opt.policy_hz = 50.0;
  opt.trigger_on_state = true;
  opt.trigger_min_period_ns = 20'000'000;
  opt.watchdog.max_frame_age_ns = 500'000'000;
  opt.watchdog.max_consecutive_unaligned = 1000;

  A3PolicyDriver driver(backend, counting_policy, opt);
  ASSERT_TRUE(driver.StartDriver());

  backend.Emit(MakeState(/*complete=*/false, /*aligned=*/false));
  EXPECT_FALSE(WaitUntil([&] { return backend.CommandCount() > 0; },
                         std::chrono::milliseconds(50)));
  driver.StopDriver();

  EXPECT_EQ(policy_calls.load(std::memory_order_relaxed), 0);
  EXPECT_EQ(driver.PolicyTickCount(), 0u);
  EXPECT_EQ(driver.SafeHaltCount(), 0u);
  EXPECT_EQ(backend.CommandCount(), 0u);
}

TEST(A3PolicyDriver, CommandFnCanSkipPublishing) {
  CapturingBackend backend;
  ASSERT_TRUE(backend.Init(""));

  std::atomic<int> command_calls{0};
  auto skip_command = [&](std::uint64_t /*tick_idx*/,
                          const RobotState& /*state*/,
                          RobotCommand& /*cmd*/) {
    command_calls.fetch_add(1, std::memory_order_relaxed);
    return false;
  };

  A3PolicyDriverOptions opt;
  opt.policy_hz = 100.0;
  opt.watchdog.max_frame_age_ns = 500'000'000;
  opt.watchdog.max_consecutive_unaligned = 1000;

  A3PolicyDriver driver(backend, a3_deploy::CommandFn(skip_command), opt);
  ASSERT_TRUE(driver.StartDriver());

  RobotState state;
  state.timestamp_ns = NowNs();
  state.tick = 1;
  state.sync_complete = true;
  state.sync_aligned = true;
  state.q = Eigen::VectorXd::Zero(robot_io::kA3Dof);
  state.dq = Eigen::VectorXd::Zero(robot_io::kA3Dof);
  state.tau_est = Eigen::VectorXd::Zero(robot_io::kA3Dof);
  backend.Emit(state);

  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  driver.StopDriver();

  EXPECT_GT(command_calls.load(std::memory_order_relaxed), 0);
  EXPECT_EQ(backend.CommandCount(), 0u);
  EXPECT_EQ(driver.PolicyTickCount(), 0u);
  EXPECT_EQ(driver.SafeHaltCount(), 0u);
}

TEST(A3PolicyDriver, CommandExceptionLatchesSafeHaltUntilRestart) {
  CapturingBackend backend;
  ASSERT_TRUE(backend.Init(""));

  std::atomic<int> command_calls{0};
  auto throwing_command = [&](std::uint64_t /*tick_idx*/,
                              const RobotState& /*state*/,
                              RobotCommand& /*cmd*/) -> bool {
    command_calls.fetch_add(1, std::memory_order_relaxed);
    throw std::runtime_error("synthetic inference fault");
  };

  A3PolicyDriverOptions opt;
  opt.policy_hz = 200.0;
  opt.watchdog.max_frame_age_ns = 500'000'000;
  opt.watchdog.max_consecutive_unaligned = 1000;

  A3PolicyDriver driver(backend, a3_deploy::CommandFn(throwing_command), opt);
  ASSERT_TRUE(driver.StartDriver());
  backend.Emit(MakeState(true, true));

  ASSERT_TRUE(WaitUntil(
      [&]() { return driver.CommandFaultLatched() && driver.SafeHaltCount() >= 2; },
      std::chrono::milliseconds(150)));
  driver.StopDriver();

  EXPECT_EQ(command_calls.load(std::memory_order_relaxed), 1)
      << "a latched command fault must never re-enter inference";
  EXPECT_EQ(driver.PolicyTickCount(), 0u);
  EXPECT_GE(backend.CommandCount(), 2u);
  const RobotCommand halt = backend.LastCommand();
  ASSERT_EQ(halt.q_des.size(), robot_io::kA3Dof);
  EXPECT_TRUE(halt.q_des.allFinite());
  EXPECT_TRUE(halt.kp.isZero(0.0));
  EXPECT_TRUE(halt.kd.isZero(0.0));
}

TEST(A3PolicyDriver, NonFiniteCommandLatchesSafeHaltUntilRestart) {
  CapturingBackend backend;
  ASSERT_TRUE(backend.Init(""));

  std::atomic<int> command_calls{0};
  auto nonfinite_command = [&](std::uint64_t /*tick_idx*/,
                               const RobotState& state,
                               RobotCommand& cmd) {
    command_calls.fetch_add(1, std::memory_order_relaxed);
    BuildSafeHaltCommand(state.q, cmd);
    cmd.q_des[0] = std::numeric_limits<double>::quiet_NaN();
    return true;
  };

  A3PolicyDriverOptions opt;
  opt.policy_hz = 200.0;
  opt.watchdog.max_frame_age_ns = 500'000'000;
  opt.watchdog.max_consecutive_unaligned = 1000;

  A3PolicyDriver driver(backend, a3_deploy::CommandFn(nonfinite_command), opt);
  ASSERT_TRUE(driver.StartDriver());
  backend.Emit(MakeState(true, true));

  ASSERT_TRUE(WaitUntil(
      [&]() { return driver.CommandFaultLatched() && driver.SafeHaltCount() >= 2; },
      std::chrono::milliseconds(150)));
  driver.StopDriver();

  EXPECT_EQ(command_calls.load(std::memory_order_relaxed), 1)
      << "a non-finite command must permanently suppress the command callback";
  EXPECT_EQ(driver.PolicyTickCount(), 0u);
  EXPECT_GE(backend.CommandCount(), 2u);
  const RobotCommand halt = backend.LastCommand();
  ASSERT_EQ(halt.q_des.size(), robot_io::kA3Dof);
  EXPECT_TRUE(halt.q_des.allFinite());
  EXPECT_TRUE(halt.kp.isZero(0.0));
  EXPECT_TRUE(halt.kd.isZero(0.0));
}

TEST(A3PolicyDriver, SuppressesSafeHaltBeforeFirstCommandWhenConfigured) {
  CapturingBackend backend;
  ASSERT_TRUE(backend.Init(""));

  std::atomic<bool> allow_command{false};
  auto gated_command = [&](std::uint64_t /*tick_idx*/,
                           const RobotState& state,
                           RobotCommand& cmd) {
    if (!allow_command.load(std::memory_order_acquire)) return false;
    BuildSafeHaltCommand(state.q, cmd);
    return true;
  };

  A3PolicyDriverOptions opt;
  opt.policy_hz = 100.0;
  opt.watchdog.max_frame_age_ns = 100'000'000;
  opt.watchdog.max_consecutive_unaligned = 1000;
  opt.send_safe_halt_before_first_command = false;

  A3PolicyDriver driver(backend, a3_deploy::CommandFn(gated_command), opt);
  ASSERT_TRUE(driver.StartDriver());

  RobotState state;
  state.timestamp_ns = NowNs();
  state.tick = 1;
  state.sync_complete = false;
  state.sync_aligned = false;
  state.q = Eigen::VectorXd::Zero(robot_io::kA3Dof);
  state.dq = Eigen::VectorXd::Zero(robot_io::kA3Dof);
  state.tau_est = Eigen::VectorXd::Zero(robot_io::kA3Dof);
  backend.Emit(state);

  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  EXPECT_EQ(backend.CommandCount(), 0u);
  EXPECT_EQ(driver.SafeHaltCount(), 0u);

  allow_command.store(true, std::memory_order_release);
  state.timestamp_ns = NowNs();
  state.tick = 2;
  state.sync_complete = true;
  state.sync_aligned = true;
  backend.Emit(state);

  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  ASSERT_GT(backend.CommandCount(), 0u);
  EXPECT_GT(driver.PolicyTickCount(), 0u);

  std::this_thread::sleep_for(std::chrono::milliseconds(180));
  driver.StopDriver();

  EXPECT_GT(driver.SafeHaltCount(), 0u)
      << "after one valid command, watchdog safe-halt should be active again";
  EXPECT_GT(backend.CommandCount(), 1u);
}

// =============================================================================
// Part B — A3PolicyDriver integration with A3AimrtBackend (OFF, sync-only)
// =============================================================================

#ifndef ENABLE_A3_AIMRT_BACKEND

namespace {

// Stub policy: write q_des = 0.1 + 0.01*i for every body joint.
void StubLinearPolicy(std::uint64_t /*tick*/,
                      const RobotState& /*state*/,
                      std::array<double, 29>& out) {
  for (int i = 0; i < 29; ++i) out[i] = 0.1 + 0.01 * i;
}

// Feed fresh samples at ~1kHz.
void FeedSamples(A3AimrtBackend& b, std::atomic<bool>& stop) {
  while (!stop.load()) {
    const auto t = NowNs();
    a3_sync::WaistSample w{}; w.stamp = {t};
    a3_sync::LegSample   l{}; l.stamp = {t};
    a3_sync::ArmSample   a{}; a.stamp = {t};
    a3_sync::NeckSample  n{}; n.stamp = {t};
    a3_sync::ImuSample   p{}; p.stamp = {t};
    a3_sync::ImuSample   to{}; to.stamp = {t};
    b.InjectWaistSample_ForTest(w);
    b.InjectLegSample_ForTest(l);
    b.InjectArmSample_ForTest(a);
    b.InjectNeckSample_ForTest(n);
    b.InjectPelvisImu_ForTest(p);
    b.InjectTorsoImu_ForTest(to);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

}  // namespace

TEST(A3PolicyDriver, RunsPolicyAndSendsExpanded31DofCommand) {
  A3AimrtBackend backend;
  ASSERT_TRUE(backend.Init("sync_hz=200,align_delay_ms=1,max_skew_ms=10"));

  // Capture SendCommand payloads. test_capture_fn fires once per topic per
  // SendCommand, so 4 entries per call — collect the first per unique stamp.
  std::mutex mu;
  std::vector<RobotCommand> captured;
  backend.SetTestCaptureFn_ForTest(
      [&](const std::string& topic, std::int64_t, std::uint32_t,
          const RobotCommand& cmd) {
        if (topic != "waist") return;  // de-dup to one capture per SendCommand
        std::lock_guard<std::mutex> lk(mu);
        captured.push_back(cmd);
      });

  A3PolicyDriverOptions opt;
  opt.policy_hz = 50.0;
  opt.watchdog.max_frame_age_ns = 100'000'000;  // 100ms tolerance

  A3PolicyDriver driver(backend, StubLinearPolicy, opt);

  ASSERT_TRUE(backend.Start());
  ASSERT_TRUE(driver.StartDriver());

  std::atomic<bool> stop{false};
  std::thread feeder([&]() { FeedSamples(backend, stop); });

  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  stop.store(true);
  feeder.join();
  driver.StopDriver();
  backend.Stop();

  std::lock_guard<std::mutex> lk(mu);
  ASSERT_GT(captured.size(), 0u) << "driver should have called SendCommand";
  // With 300ms @ 50Hz we expect ~15 ticks; allow slack for startup.
  EXPECT_GT(driver.PolicyTickCount(), 5u);

  // Inspect the last captured command. Post 2026-04 MuJoCo realignment
  // (notes/a3_dof_orderings.md), 29-DOF policy output is scattered into
  // the 29 non-neck SDK slots via kA3PolicyToSdkIdx; head/neck [3..4] is
  // controlled by fixed zero-position PD command.
  // kp/kd at non-neck SDK slots should equal a3_kps/a3_kds[policy_i].
  const auto& cmd = captured.back();
  ASSERT_EQ(cmd.q_des.size(), 31);
  for (int i = 0; i < robot_io::kA3PolicyDof; ++i) {
    const int sdk = robot_io::kA3PolicyToSdkIdx[i];
    EXPECT_NEAR(cmd.q_des[sdk], 0.1 + 0.01 * i, 1e-12)
        << "policy i=" << i << " sdk=" << sdk;
    EXPECT_DOUBLE_EQ(cmd.kp[sdk], a3_kps[i]);
    EXPECT_DOUBLE_EQ(cmd.kd[sdk], a3_kds[i]);
  }
  for (int i = robot_io::kA3NeckStart;
       i < robot_io::kA3NeckStart + robot_io::kA3NeckCount; ++i) {
    EXPECT_DOUBLE_EQ(cmd.q_des[i], a3_deploy::kA3HeadTargetPositionRad);
    EXPECT_DOUBLE_EQ(cmd.kp[i], a3_deploy::kA3HeadKp);
    EXPECT_DOUBLE_EQ(cmd.kd[i], a3_deploy::kA3HeadKd);
  }
}

TEST(A3PolicyDriver, SafeHaltTriggersWhenFramesGoStale) {
  A3AimrtBackend backend;
  ASSERT_TRUE(backend.Init("sync_hz=200,align_delay_ms=1,max_skew_ms=10"));

  std::mutex mu;
  std::vector<RobotCommand> captured;
  backend.SetTestCaptureFn_ForTest(
      [&](const std::string& topic, std::int64_t, std::uint32_t,
          const RobotCommand& cmd) {
        if (topic != "waist") return;
        std::lock_guard<std::mutex> lk(mu);
        captured.push_back(cmd);
      });

  A3PolicyDriverOptions opt;
  opt.policy_hz = 50.0;
  opt.watchdog.max_frame_age_ns = 20'000'000;  // 20ms

  A3PolicyDriver driver(backend, StubLinearPolicy, opt);

  ASSERT_TRUE(backend.Start());
  ASSERT_TRUE(driver.StartDriver());

  // Brief feed so driver caches at least one state, then stop the feeder
  // AND the backend (sync_loop) so no new state callbacks refresh the
  // cached timestamp. The driver's RT thread keeps ticking and the cached
  // state ages past max_frame_age_ns.
  std::atomic<bool> stop{false};
  std::thread feeder([&]() { FeedSamples(backend, stop); });
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  stop.store(true);
  feeder.join();
  backend.Stop();

  // Wait for the cached frame to age out and watchdog to trip.
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  driver.StopDriver();

  EXPECT_GT(driver.SafeHaltCount(), 0u)
      << "expected watchdog to trip and emit safe-halt commands";
  EXPECT_GT(driver.Watchdog().StaleCount(), 0u);

  // At least one of the captured commands should be a safe-halt signature
  // (kp/kd all zero across the full 31-DOF vector — the stub policy would
  // have produced non-zero kp via a3_kps).
  std::lock_guard<std::mutex> lk(mu);
  bool saw_halt = false;
  for (const auto& c : captured) {
    if (c.kp.size() == 31 && c.kp.isZero(0.0) && c.kd.isZero(0.0)) {
      saw_halt = true;
      break;
    }
  }
  EXPECT_TRUE(saw_halt) << "expected at least one safe-halt command capture";
}

// ---------------------------------------------------------------------------
// PR 9.4: PolicyFn now receives (tick_idx, RobotState, q_des_out). Verify
// tick_idx is monotonic-nondecreasing across successive Ok ticks and does
// NOT advance during safe-halt ticks (so the downstream TokenizerReplay stays
// in sync with the RL policy's logical tick counter).
TEST(A3PolicyDriver, TickIdxMonotonicAndStableAcrossSafeHalt) {
  A3AimrtBackend backend;
  ASSERT_TRUE(backend.Init("sync_hz=200,align_delay_ms=1,max_skew_ms=10"));

  std::mutex mu;
  std::vector<std::uint64_t> observed_ticks;
  auto counting_policy = [&](std::uint64_t tick_idx,
                             const RobotState& /*state*/,
                             std::array<double, 29>& out) {
    out.fill(0.0);
    std::lock_guard<std::mutex> lk(mu);
    observed_ticks.push_back(tick_idx);
  };

  A3PolicyDriverOptions opt;
  opt.policy_hz = 50.0;
  opt.watchdog.max_frame_age_ns = 20'000'000;  // 20ms

  A3PolicyDriver driver(backend, counting_policy, opt);

  ASSERT_TRUE(backend.Start());
  ASSERT_TRUE(driver.StartDriver());

  // Phase 1: feed samples → several Ok ticks.
  std::atomic<bool> stop{false};
  std::thread feeder([&]() { FeedSamples(backend, stop); });
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  stop.store(true);
  feeder.join();
  backend.Stop();

  // Phase 2: sleep while cached state ages out → safe-halt ticks.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  driver.StopDriver();

  std::lock_guard<std::mutex> lk(mu);
  ASSERT_GT(observed_ticks.size(), 0u) << "policy should have been called";

  // Tick idxs must be non-decreasing and each distinct tick must be the
  // previous + 1 (since tick_counter post-increments on every Ok call).
  for (std::size_t i = 0; i < observed_ticks.size(); ++i) {
    EXPECT_EQ(observed_ticks[i], i) << "observed_ticks[" << i << "]";
  }

  // Safe-halt ticks must have fired (Phase 2 stop of sync_loop).
  EXPECT_GT(driver.SafeHaltCount(), 0u);

  // PolicyTickCount must equal the last observed tick index + 1.
  EXPECT_EQ(driver.PolicyTickCount(), observed_ticks.size());
  EXPECT_EQ(driver.TickIndex(), observed_ticks.size());
}

#endif  // !ENABLE_A3_AIMRT_BACKEND
