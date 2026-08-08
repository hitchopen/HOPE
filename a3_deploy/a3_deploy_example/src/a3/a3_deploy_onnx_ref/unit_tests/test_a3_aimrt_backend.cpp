// Copyright (c) 2026, AgiBot Inc. All rights reserved.
// 对应 notes/a3_backend_plan.md §PR 7 / PR 7a
//
// Unit tests for A3AimrtBackend. These tests never require AimRT: they drive
// the pipeline via the Inject*_ForTest + SetTestCaptureFn_ForTest hooks.
// When ENABLE_A3_AIMRT_BACKEND=OFF, Start() skips all AimRT code entirely and
// the tests exercise the pure sync_loop-only path.

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

#include "robot_io/a3_aimrt_backend.hpp"
#include "robot_io/a3_layout_extra.hpp"
#include "robot_io/robot_io_backend.hpp"

using robot_io::A3AimrtBackend;
using robot_io::CreateBackend;
using robot_io::RobotCommand;
using robot_io::RobotState;

namespace {

std::int64_t NowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

RobotCommand ZeroCmd31() {
  RobotCommand cmd;
  cmd.q_des  = Eigen::VectorXd::Zero(robot_io::kA3Dof);
  cmd.dq_des = Eigen::VectorXd::Zero(robot_io::kA3Dof);
  cmd.tau_ff = Eigen::VectorXd::Zero(robot_io::kA3Dof);
  cmd.kp     = Eigen::VectorXd::Zero(robot_io::kA3Dof);
  cmd.kd     = Eigen::VectorXd::Zero(robot_io::kA3Dof);
  return cmd;
}

}  // namespace

TEST(A3AimrtBackend, LayoutIs31AndNameIsA3) {
  A3AimrtBackend b;
  EXPECT_EQ(b.GetLayout().dof(), robot_io::kA3Dof);
  EXPECT_EQ(b.GetLayout().dof(), 31);
  EXPECT_EQ(b.Name(), "a3");
}

TEST(A3AimrtBackend, InitSucceedsWithoutCfgPathWhenAimrtOff) {
#ifndef ENABLE_A3_AIMRT_BACKEND
  A3AimrtBackend b;
  EXPECT_TRUE(b.Init(""));
#else
  GTEST_SKIP() << "ENABLE_A3_AIMRT_BACKEND=ON requires cfg_file_path";
#endif
}

TEST(A3AimrtBackend, InitParsesSyncKnobs) {
  A3AimrtBackend b;
#ifdef ENABLE_A3_AIMRT_BACKEND
  ASSERT_TRUE(b.Init(
      "cfg_file_path=a3_deploy_example/src/a3/a3_deploy_onnx_ref/config/"
      "a3_aimrt_config.yaml,"
      "sync_hz=100,align_delay_ms=1,max_skew_ms=5"));
#else
  ASSERT_TRUE(b.Init("sync_hz=100,align_delay_ms=1,max_skew_ms=5"));
#endif
  EXPECT_DOUBLE_EQ(b.StateRateHz(), 100.0);
}

TEST(A3AimrtBackend, InitAcceptsLatestFrameSyncMode) {
  A3AimrtBackend b;
#ifdef ENABLE_A3_AIMRT_BACKEND
  ASSERT_TRUE(b.Init(
      "cfg_file_path=a3_deploy_example/src/a3/a3_deploy_onnx_ref/config/"
      "a3_aimrt_config.yaml,"
      "sync_mode=latest_frame,sync_hz=100"));
#else
  ASSERT_TRUE(b.Init("sync_mode=latest_frame,sync_hz=100"));
#endif
  EXPECT_DOUBLE_EQ(b.StateRateHz(), 100.0);
}

TEST(A3AimrtBackend, InitRejectsBadKnob) {
  A3AimrtBackend b;
  EXPECT_FALSE(b.Init("sync_hz=not-a-number"));

  A3AimrtBackend b2;
  EXPECT_FALSE(b2.Init("max_sample_age_ms=not-a-number"));
}

TEST(A3AimrtBackend, StartWithoutAimrtSucceeds) {
#ifndef ENABLE_A3_AIMRT_BACKEND
  A3AimrtBackend b;
  ASSERT_TRUE(b.Init("sync_hz=200"));
  EXPECT_TRUE(b.Start());
  // Idempotent
  EXPECT_TRUE(b.Start());
  b.Stop();
  // Stop idempotent
  b.Stop();
#else
  GTEST_SKIP() << "covered by workstation-side PR 7b verification";
#endif
}

TEST(A3AimrtBackend, StartBeforeInitFails) {
  A3AimrtBackend b;
  EXPECT_FALSE(b.Start());
}

TEST(A3AimrtBackend, SendCommandSizeCheck) {
#ifdef ENABLE_A3_AIMRT_BACKEND
  GTEST_SKIP() << "requires real AimRT to call SendCommand after Start()";
#else
  A3AimrtBackend b;
  ASSERT_TRUE(b.Init(""));
  ASSERT_TRUE(b.Start());

  RobotCommand bad = ZeroCmd31();
  bad.q_des = Eigen::VectorXd::Zero(29);
  EXPECT_FALSE(b.SendCommand(bad));

  RobotCommand good = ZeroCmd31();
  EXPECT_TRUE(b.SendCommand(good));
  b.Stop();
#endif
}

TEST(A3AimrtBackend, SendCommandInvokesTestCaptureForAllFourTopics) {
#ifdef ENABLE_A3_AIMRT_BACKEND
  GTEST_SKIP();
#else
  A3AimrtBackend b;
  ASSERT_TRUE(b.Init(""));

  std::mutex mu;
  std::vector<std::string> topics;
  std::vector<std::uint32_t> seqs;
  b.SetTestCaptureFn_ForTest(
      [&](const std::string& t, std::int64_t, std::uint32_t seq,
          const RobotCommand&) {
        std::lock_guard<std::mutex> lk(mu);
        topics.push_back(t);
        seqs.push_back(seq);
      });

  ASSERT_TRUE(b.Start());
  ASSERT_TRUE(b.SendCommand(ZeroCmd31()));
  ASSERT_TRUE(b.SendCommand(ZeroCmd31()));

  b.Stop();

  std::lock_guard<std::mutex> lk(mu);
  ASSERT_EQ(topics.size(), 8u);  // 4 topics * 2 SendCommand calls
  std::set<std::string> unique(topics.begin(), topics.end());
  EXPECT_EQ(unique.size(), 4u);
  EXPECT_TRUE(unique.count("waist"));
  EXPECT_TRUE(unique.count("leg"));
  EXPECT_TRUE(unique.count("arm"));
  EXPECT_TRUE(unique.count("neck"));
  // seq monotonically increasing across SendCommand calls (same seq for the
  // 4 topics within a single call).
  EXPECT_EQ(seqs[0], seqs[3]);
  EXPECT_EQ(seqs[4], seqs[7]);
  EXPECT_LT(seqs[0], seqs[4]);
#endif
}

TEST(A3AimrtBackend, EndToEndSampleInjectionTriggersStateCallback) {
#ifdef ENABLE_A3_AIMRT_BACKEND
  GTEST_SKIP();
#else
  A3AimrtBackend b;
  // align_delay = 1ms, max_skew = 10ms so sync_loop accepts loose stamps.
  ASSERT_TRUE(b.Init("sync_hz=200,align_delay_ms=1,max_skew_ms=10"));

  std::atomic<int> cb_count{0};
  std::atomic<std::int64_t> last_tick{0};
  b.RegisterStateCallback([&](const RobotState& s) {
    cb_count.fetch_add(1);
    last_tick.store(s.tick);
  });
  ASSERT_TRUE(b.Start());

  // Feed samples in a background thread so that at each tick there is a
  // bracket (pre + post) for every channel. ~1 kHz feeder over 200 ms at
  // 200 Hz sync rate should yield a bunch of callbacks.
  std::atomic<bool> stop{false};
  std::thread feeder([&] {
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
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  stop.store(true);
  feeder.join();
  b.Stop();

  EXPECT_GT(cb_count.load(), 0);
#endif
}

TEST(BackendFactory, CreateA3ReturnsValidBackend) {
  auto b = CreateBackend("a3");
  ASSERT_NE(b, nullptr);
  EXPECT_EQ(b->Name(), "a3");
  EXPECT_EQ(b->GetLayout().dof(), 31);
}

TEST(BackendFactory, CreateA3StillWorks) {
  auto b = CreateBackend("a3");
  ASSERT_NE(b, nullptr);
  EXPECT_EQ(b->Name(), "a3");
}

TEST(BackendFactory, UnknownReturnsNull) {
  EXPECT_EQ(CreateBackend("does-not-exist"), nullptr);
}
