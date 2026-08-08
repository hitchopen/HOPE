// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// Unit tests for the A3 sync machinery (notes/a3_backend_plan.md §8 / PR 3):
// - Lerp / LerpArrays / SlerpQuatWxyz pure functions
// - AssembleState (31-DOF layout ordering, IMU routing, secondary flag)
// - RingBuffer basic write/read and wrap-around behaviour
// - A3SyncLoop end-to-end: feed synthetic samples, expect aligned callbacks

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <mutex>
#include <thread>
#include <vector>

#include "a3_sync/a3_ring_buffer.hpp"
#include "a3_sync/a3_sync_loop.hpp"
#include "a3_sync/a3_sync_types.hpp"
#include "robot_io/a3_layout_extra.hpp"

using a3_sync::ArmSample;
using a3_sync::A3SyncLoop;
using a3_sync::ImuSample;
using a3_sync::LegSample;
using a3_sync::NeckSample;
using a3_sync::RingBuffer;
using a3_sync::TimestampNs;
using a3_sync::WaistSample;

// ---------------- Pure function tests ---------------------------------------

TEST(LerpScalar, Endpoints) {
  EXPECT_DOUBLE_EQ(a3_sync::Lerp(2.0, 8.0, 0.0), 2.0);
  EXPECT_DOUBLE_EQ(a3_sync::Lerp(2.0, 8.0, 1.0), 8.0);
  EXPECT_DOUBLE_EQ(a3_sync::Lerp(2.0, 8.0, 0.5), 5.0);
}

TEST(LerpArrays, ElementWise) {
  std::array<double, 3> a{0.0, 1.0, 2.0};
  std::array<double, 3> b{10.0, 11.0, 12.0};
  std::array<double, 3> o{};
  a3_sync::LerpArrays(a, b, 0.25, o);
  EXPECT_DOUBLE_EQ(o[0], 2.5);
  EXPECT_DOUBLE_EQ(o[1], 3.5);
  EXPECT_DOUBLE_EQ(o[2], 4.5);
}

static double QuatNorm(const std::array<double, 4>& q) {
  return std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
}

static std::int64_t SystemNowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

static WaistSample MakeWaistSample(std::int64_t stamp_ns,
                                   std::int64_t recv_ns,
                                   double marker = 0.0) {
  WaistSample s{};
  s.stamp = TimestampNs{stamp_ns};
  s.recv_stamp = TimestampNs{recv_ns};
  s.source_stamp_valid = true;
  s.pos[0] = marker;
  return s;
}

static LegSample MakeLegSample(std::int64_t stamp_ns,
                               std::int64_t recv_ns) {
  LegSample s{};
  s.stamp = TimestampNs{stamp_ns};
  s.recv_stamp = TimestampNs{recv_ns};
  s.source_stamp_valid = true;
  return s;
}

static ArmSample MakeArmSample(std::int64_t stamp_ns,
                               std::int64_t recv_ns) {
  ArmSample s{};
  s.stamp = TimestampNs{stamp_ns};
  s.recv_stamp = TimestampNs{recv_ns};
  s.source_stamp_valid = true;
  return s;
}

static NeckSample MakeNeckSample(std::int64_t stamp_ns,
                                 std::int64_t recv_ns) {
  NeckSample s{};
  s.stamp = TimestampNs{stamp_ns};
  s.recv_stamp = TimestampNs{recv_ns};
  s.source_stamp_valid = true;
  return s;
}

static ImuSample MakeImuSample(std::int64_t stamp_ns,
                               std::int64_t recv_ns,
                               double marker = 0.0) {
  ImuSample s{};
  s.stamp = TimestampNs{stamp_ns};
  s.recv_stamp = TimestampNs{recv_ns};
  s.source_stamp_valid = true;
  s.gyro[0] = marker;
  return s;
}

static void InjectJointGroup(A3SyncLoop& loop, std::int64_t stamp_ns,
                             std::int64_t recv_ns,
                             double marker = 0.0) {
  loop.OnWaistState(MakeWaistSample(stamp_ns, recv_ns, marker));
  loop.OnLegState(MakeLegSample(stamp_ns, recv_ns));
  loop.OnArmState(MakeArmSample(stamp_ns, recv_ns));
  loop.OnNeckState(MakeNeckSample(stamp_ns, recv_ns));
}

static void InjectImuGroup(A3SyncLoop& loop, std::int64_t stamp_ns,
                           std::int64_t recv_ns,
                           double marker = 0.0) {
  loop.OnPelvisImu(MakeImuSample(stamp_ns, recv_ns, marker));
  loop.OnTorsoImu(MakeImuSample(stamp_ns, recv_ns, marker));
}

static a3_sync::SyncConfig MinSkewPairTestConfig() {
  a3_sync::SyncConfig cfg;
  cfg.sync_hz = 100.0;
  cfg.sync_mode = a3_sync::SyncMode::MinSkewPair;
  cfg.align_delay_ns = 0;
  cfg.max_sample_age_ns = 1'000'000'000;
  cfg.max_group_internal_skew_ns = 250'000;
  cfg.max_group_pair_skew_ns = 1'000'000;
  cfg.group_pair_search_depth = 4;
  cfg.max_backtrack = 64;
  return cfg;
}

static a3_sync::SyncConfig LatestFrameTestConfig() {
  a3_sync::SyncConfig cfg;
  cfg.sync_mode = a3_sync::SyncMode::LatestFrame;
  cfg.max_skew_ns = 3'000'000;
  cfg.max_sample_age_ns = 1'000'000'000;
  return cfg;
}

TEST(SlerpQuatWxyz, EndpointsAndMidpoint) {
  // q0 = identity, q1 = 90° about Z in WXYZ.
  const double s = std::sin(M_PI / 4.0);
  const double c = std::cos(M_PI / 4.0);
  std::array<double, 4> q0{1.0, 0.0, 0.0, 0.0};
  std::array<double, 4> q1{c,   0.0, 0.0, s};

  auto r0 = a3_sync::SlerpQuatWxyz(q0, q1, 0.0);
  EXPECT_NEAR(r0[0], 1.0, 1e-9);
  EXPECT_NEAR(QuatNorm(r0), 1.0, 1e-9);

  auto r1 = a3_sync::SlerpQuatWxyz(q0, q1, 1.0);
  EXPECT_NEAR(r1[0], c, 1e-9);
  EXPECT_NEAR(r1[3], s, 1e-9);
  EXPECT_NEAR(QuatNorm(r1), 1.0, 1e-9);

  auto rm = a3_sync::SlerpQuatWxyz(q0, q1, 0.5);
  // halfway is 45° about Z: w=cos(22.5°), z=sin(22.5°)
  EXPECT_NEAR(rm[0], std::cos(M_PI / 8.0), 1e-9);
  EXPECT_NEAR(rm[3], std::sin(M_PI / 8.0), 1e-9);
  EXPECT_NEAR(QuatNorm(rm), 1.0, 1e-9);
}

TEST(SlerpQuatWxyz, ShortestArcAntipodal) {
  std::array<double, 4> q0{1.0, 0.0, 0.0, 0.0};
  std::array<double, 4> q1{-1.0, 0.0, 0.0, 0.0};  // antipodal same rotation
  auto rm = a3_sync::SlerpQuatWxyz(q0, q1, 0.5);
  EXPECT_NEAR(QuatNorm(rm), 1.0, 1e-9);
  // Should not blow up; result should represent zero rotation.
  EXPECT_NEAR(std::fabs(rm[0]), 1.0, 1e-9);
}

// ---------------- AssembleState tests ---------------------------------------

TEST(AssembleState, Layout31DOFOrdering) {
  WaistSample w{}; LegSample l{}; ArmSample a{}; NeckSample n{};
  ImuSample p{}; ImuSample t{};

  for (int i = 0; i < 12; ++i) l.pos[i] = 100.0 + i;
  for (int i = 0; i < 3;  ++i) w.pos[i] = 200.0 + i;
  for (int i = 0; i < 14; ++i) a.pos[i] = 300.0 + i;
  for (int i = 0; i < 2;  ++i) n.pos[i] = 400.0 + i;

  p.quat_wxyz = {1.0, 0.0, 0.0, 0.0};
  p.gyro      = {0.1, 0.2, 0.3};
  p.acc       = {0.0, 0.0, 9.81};

  t.quat_wxyz = {std::cos(M_PI/4), 0.0, 0.0, std::sin(M_PI/4)};
  t.gyro      = {1.0, 2.0, 3.0};
  t.acc       = {4.0, 5.0, 6.0};

  auto s = a3_sync::AssembleState(1234, 7, w, true, l, true, a, true,
                                  n, true, p, true, t, true);
  EXPECT_EQ(s.timestamp_ns, 1234);
  EXPECT_EQ(s.tick, 7);
  EXPECT_EQ(s.q.size(), robot_io::kA3Dof);

  // Post-realignment (notes/a3_dof_orderings.md): MuJoCo real order
  //   [0..2]   waist  ← waist_sample[0..2]
  //   [3..4]   neck   ← neck_sample[0..1]
  //   [5..11]  L_arm  ← arm_sample[0..6]
  //   [12..18] R_arm  ← arm_sample[7..13]
  //   [19..24] L_leg  ← leg_sample[0..5]
  //   [25..30] R_leg  ← leg_sample[6..11]
  for (int i = 0; i < 3;  ++i) EXPECT_DOUBLE_EQ(s.q(i),        200.0 + i);  // waist
  for (int i = 0; i < 2;  ++i) EXPECT_DOUBLE_EQ(s.q(3 + i),    400.0 + i);  // neck
  for (int i = 0; i < 7;  ++i) EXPECT_DOUBLE_EQ(s.q(5 + i),    300.0 + i);  // L_arm
  for (int i = 0; i < 7;  ++i) EXPECT_DOUBLE_EQ(s.q(12 + i),   300.0 + (7 + i));  // R_arm
  for (int i = 0; i < 6;  ++i) EXPECT_DOUBLE_EQ(s.q(19 + i),   100.0 + i);  // L_leg
  for (int i = 0; i < 6;  ++i) EXPECT_DOUBLE_EQ(s.q(25 + i),   100.0 + (6 + i));  // R_leg

  // pelvis -> imu_*
  EXPECT_DOUBLE_EQ(s.imu_quat_wxyz(0), 1.0);
  EXPECT_DOUBLE_EQ(s.imu_gyro(0), 0.1);
  EXPECT_DOUBLE_EQ(s.imu_accel(2), 9.81);

  // torso -> sec_imu_*
  EXPECT_TRUE(s.has_secondary_imu);
  EXPECT_DOUBLE_EQ(s.sec_imu_gyro(2), 3.0);
  EXPECT_DOUBLE_EQ(s.sec_imu_accel(0), 4.0);
}

TEST(AssembleState, SecondaryImuFlagOff) {
  WaistSample w{}; LegSample l{}; ArmSample a{}; NeckSample n{};
  ImuSample p{}; ImuSample t{};
  auto s = a3_sync::AssembleState(0, 0, w, true, l, true, a, true,
                                  n, true, p, true, t, false);
  EXPECT_FALSE(s.has_secondary_imu);
}

// ---------------- Ring buffer tests -----------------------------------------

TEST(RingBufferBasic, WriteReadAndWrap) {
  RingBuffer<int> rb(4);
  EXPECT_EQ(rb.capacity(), 4u);
  EXPECT_EQ(rb.latest_index(), 0u);

  auto i1 = rb.write([](int& x) { x = 10; });
  auto i2 = rb.write([](int& x) { x = 20; });
  auto i3 = rb.write([](int& x) { x = 30; });
  EXPECT_EQ(i1, 1u);
  EXPECT_EQ(i2, 2u);
  EXPECT_EQ(i3, 3u);

  int v = 0;
  EXPECT_TRUE(rb.read_at(i3, v)); EXPECT_EQ(v, 30);
  EXPECT_TRUE(rb.read_at(i2, v)); EXPECT_EQ(v, 20);

  // Past-capacity writes should invalidate old indices.
  for (int k = 0; k < 10; ++k) rb.write([&](int& x) { x = 1000 + k; });
  EXPECT_EQ(rb.latest_index(), 13u);
  // index 1 must be gone (capacity only 4).
  EXPECT_FALSE(rb.read_at(1, v));
  // latest index must be readable.
  EXPECT_TRUE(rb.read_at(13, v));
  EXPECT_EQ(v, 1009);
}

TEST(RingBufferBasic, ReadAtZeroIsEmpty) {
  RingBuffer<int> rb(2);
  int v = 42;
  EXPECT_FALSE(rb.read_at(0, v));  // no write yet (and idx 0 is sentinel)
}

// ---------------- A3SyncLoop end-to-end -------------------------------------

TEST(A3SyncLoop, LatestFrameEmitsCompleteLatestStateWithoutThreadedSync) {
  a3_sync::A3SyncLoop::Options opt;
  opt.sync = LatestFrameTestConfig();
  A3SyncLoop loop(std::move(opt));

  std::mutex mu;
  std::condition_variable cv;
  robot_io::RobotState observed;
  int callbacks = 0;
  loop.RegisterStateCallback([&](const robot_io::RobotState& s) {
    std::lock_guard<std::mutex> lk(mu);
    observed = s;
    ++callbacks;
    cv.notify_all();
  });

  constexpr std::int64_t kBase = 5'000'000'000;
  const auto recv = SystemNowNs();
  loop.Start();
  loop.OnWaistState(MakeWaistSample(kBase + 0, recv + 10, 0.0));
  loop.OnLegState(MakeLegSample(kBase + 1'000'000, recv + 20));
  loop.OnArmState(MakeArmSample(kBase + 2'000'000, recv + 30));
  loop.OnNeckState(MakeNeckSample(kBase + 500'000, recv + 40));
  loop.OnPelvisImu(MakeImuSample(kBase + 1'500'000, recv + 50, 1.5));
  {
    std::unique_lock<std::mutex> lk(mu);
    EXPECT_FALSE(cv.wait_for(lk, std::chrono::milliseconds(20),
                             [&] { return callbacks > 0; }));
  }

  loop.OnTorsoImu(MakeImuSample(kBase + 2'500'000, recv + 60, 2.5));
  {
    std::unique_lock<std::mutex> lk(mu);
    EXPECT_TRUE(cv.wait_for(lk, std::chrono::milliseconds(100),
                            [&] { return callbacks > 0; }));
  }
  loop.Stop();

  ASSERT_EQ(callbacks, 1);
  EXPECT_TRUE(observed.sync_complete);
  EXPECT_TRUE(observed.sync_aligned);
  EXPECT_EQ(observed.timestamp_ns, kBase);
  EXPECT_EQ(observed.sync_skew_ns, 2'500'000);
  EXPECT_EQ(observed.state_data_ready_ns, recv + 60);
  EXPECT_GE(observed.state_sync_ready_ns, observed.state_data_ready_ns);
  EXPECT_DOUBLE_EQ(observed.q(0), 0.0);
  EXPECT_DOUBLE_EQ(observed.imu_gyro(0), 1.5);

  const auto snap = loop.Statistics();
  EXPECT_EQ(snap.tick_total, 1u);
  EXPECT_EQ(snap.frame_complete_total, 1u);
  EXPECT_EQ(snap.frame_aligned_total, 1u);
}

TEST(A3SyncLoop, LatestFrameDoesNotEmitWhenAChannelIsStale) {
  a3_sync::A3SyncLoop::Options opt;
  opt.sync = LatestFrameTestConfig();
  opt.sync.max_sample_age_ns = 1'000'000;
  A3SyncLoop loop(std::move(opt));

  std::atomic<int> callbacks{0};
  loop.RegisterStateCallback([&](const robot_io::RobotState&) {
    callbacks.fetch_add(1);
  });

  constexpr std::int64_t kBase = 6'000'000'000;
  const auto now = SystemNowNs();
  const auto stale_recv = now - 10'000'000;
  loop.Start();
  loop.OnWaistState(MakeWaistSample(kBase, stale_recv));
  loop.OnLegState(MakeLegSample(kBase, now));
  loop.OnArmState(MakeArmSample(kBase, now));
  loop.OnNeckState(MakeNeckSample(kBase, now));
  loop.OnPelvisImu(MakeImuSample(kBase, now));
  loop.OnTorsoImu(MakeImuSample(kBase, now));
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  loop.Stop();

  EXPECT_EQ(callbacks.load(), 0);
  EXPECT_EQ(loop.Statistics().tick_total, 0u);
}

TEST(A3SyncLoop, MinSkewPairChoosesClosestHistoricalGroups) {
  auto cfg = MinSkewPairTestConfig();

  a3_sync::A3SyncLoop::Options opt;
  opt.sync = cfg;
  A3SyncLoop loop(std::move(opt));

  std::mutex mu;
  std::condition_variable cv;
  robot_io::RobotState observed;
  bool got = false;
  loop.RegisterStateCallback([&](const robot_io::RobotState& s) {
    if (!s.sync_complete) return;
    std::lock_guard<std::mutex> lk(mu);
    if (!got) {
      observed = s;
      got = true;
      cv.notify_all();
    }
  });

  constexpr std::int64_t kBase = 1'000'000'000;
  const auto recv = SystemNowNs();
  InjectJointGroup(loop, kBase + 0, recv, 0.0);
  InjectJointGroup(loop, kBase + 2'000'000, recv, 2.0);
  InjectImuGroup(loop, kBase + 800'000, recv, 0.8);

  loop.Start();
  {
    std::unique_lock<std::mutex> lk(mu);
    EXPECT_TRUE(cv.wait_for(lk, std::chrono::milliseconds(100),
                            [&] { return got; }));
  }
  loop.Stop();

  ASSERT_TRUE(got);
  EXPECT_TRUE(observed.sync_complete);
  EXPECT_TRUE(observed.sync_aligned);
  EXPECT_EQ(observed.timestamp_ns, kBase + 800'000);
  EXPECT_EQ(observed.sync_skew_ns, 800'000);
  EXPECT_DOUBLE_EQ(observed.q(0), 0.0);
  EXPECT_DOUBLE_EQ(observed.imu_gyro(0), 0.8);
}

TEST(A3SyncLoop, MinSkewPairWaitsForFixedTickInsteadOfSampleEvent) {
  auto cfg = MinSkewPairTestConfig();
  cfg.sync_hz = 20.0;
  cfg.sync_ready_after_input_ns = 0;
  constexpr std::int64_t kPeriodNs = 50'000'000;
  constexpr std::int64_t kDesiredFirstWakeNs = 30'000'000;
  cfg.phase_ns = (SystemNowNs() + kDesiredFirstWakeNs) % kPeriodNs;

  a3_sync::A3SyncLoop::Options opt;
  opt.sync = cfg;
  A3SyncLoop loop(std::move(opt));

  std::mutex mu;
  std::condition_variable cv;
  bool got = false;
  std::int64_t first_ready_ns = 0;
  loop.RegisterStateCallback([&](const robot_io::RobotState& s) {
    if (!s.sync_complete) return;
    std::lock_guard<std::mutex> lk(mu);
    if (!got) {
      got = true;
      first_ready_ns = s.state_sync_ready_ns;
      cv.notify_all();
    }
  });

  const auto recv = SystemNowNs();
  InjectJointGroup(loop, recv, recv, 0.0);
  InjectImuGroup(loop, recv + 100'000, recv, 0.1);

  const auto start_ns = SystemNowNs();
  loop.Start();
  {
    std::unique_lock<std::mutex> lk(mu);
    EXPECT_FALSE(cv.wait_for(lk, std::chrono::milliseconds(10),
                             [&] { return got; }));
    EXPECT_TRUE(cv.wait_for(lk, std::chrono::milliseconds(100),
                            [&] { return got; }));
  }
  loop.Stop();

  ASSERT_TRUE(got);
  EXPECT_GE(first_ready_ns - start_ns, 15'000'000)
      << "min_skew_pair should release on the fixed sync grid, not "
         "immediately on pre-existing samples";
}

TEST(A3SyncLoop, MinSkewPairTieBreaksToNewestPair) {
  auto cfg = MinSkewPairTestConfig();

  a3_sync::A3SyncLoop::Options opt;
  opt.sync = cfg;
  A3SyncLoop loop(std::move(opt));

  std::mutex mu;
  std::condition_variable cv;
  robot_io::RobotState observed;
  bool got = false;
  loop.RegisterStateCallback([&](const robot_io::RobotState& s) {
    if (!s.sync_complete) return;
    std::lock_guard<std::mutex> lk(mu);
    if (!got) {
      observed = s;
      got = true;
      cv.notify_all();
    }
  });

  constexpr std::int64_t kBase = 2'000'000'000;
  const auto recv = SystemNowNs();
  InjectJointGroup(loop, kBase + 0, recv, 0.0);
  InjectJointGroup(loop, kBase + 2'000'000, recv, 2.0);
  InjectImuGroup(loop, kBase + 1'000'000, recv, 1.0);
  InjectImuGroup(loop, kBase + 3'000'000, recv, 3.0);

  loop.Start();
  {
    std::unique_lock<std::mutex> lk(mu);
    EXPECT_TRUE(cv.wait_for(lk, std::chrono::milliseconds(100),
                            [&] { return got; }));
  }
  loop.Stop();

  ASSERT_TRUE(got);
  EXPECT_TRUE(observed.sync_complete);
  EXPECT_TRUE(observed.sync_aligned);
  EXPECT_EQ(observed.timestamp_ns, kBase + 3'000'000);
  EXPECT_EQ(observed.sync_skew_ns, 1'000'000);
  EXPECT_DOUBLE_EQ(observed.q(0), 2.0);
  EXPECT_DOUBLE_EQ(observed.imu_gyro(0), 3.0);
}

TEST(A3SyncLoop, MinSkewPairPrefersFreshAlignedPairOverOlderPerfectSkew) {
  auto cfg = MinSkewPairTestConfig();

  a3_sync::A3SyncLoop::Options opt;
  opt.sync = cfg;
  A3SyncLoop loop(std::move(opt));

  std::mutex mu;
  std::condition_variable cv;
  robot_io::RobotState observed;
  bool got = false;
  loop.RegisterStateCallback([&](const robot_io::RobotState& s) {
    if (!s.sync_complete) return;
    std::lock_guard<std::mutex> lk(mu);
    if (!got) {
      observed = s;
      got = true;
      cv.notify_all();
    }
  });

  constexpr std::int64_t kBase = 2'500'000'000;
  const auto recv = SystemNowNs();
  InjectJointGroup(loop, kBase + 0, recv, 0.0);
  InjectImuGroup(loop, kBase + 0, recv, 0.0);
  InjectJointGroup(loop, kBase + 2'000'000, recv + 1'000'000, 2.0);
  InjectImuGroup(loop, kBase + 2'500'000, recv + 1'000'000, 2.5);

  loop.Start();
  {
    std::unique_lock<std::mutex> lk(mu);
    EXPECT_TRUE(cv.wait_for(lk, std::chrono::milliseconds(100),
                            [&] { return got; }));
  }
  loop.Stop();

  ASSERT_TRUE(got);
  EXPECT_TRUE(observed.sync_complete);
  EXPECT_TRUE(observed.sync_aligned);
  EXPECT_EQ(observed.timestamp_ns, kBase + 2'500'000);
  EXPECT_EQ(observed.sync_skew_ns, 500'000);
  EXPECT_DOUBLE_EQ(observed.q(0), 2.0);
  EXPECT_DOUBLE_EQ(observed.imu_gyro(0), 2.5);
}

TEST(A3SyncLoop, MinSkewPairEstimatesAutoPhaseFromLatestAlignedPair) {
  auto cfg = MinSkewPairTestConfig();
  cfg.sync_hz = 100.0;
  cfg.sync_ready_after_input_ns = 200'000;
  cfg.phase_ns = 0;

  a3_sync::A3SyncLoop::Options opt;
  opt.sync = cfg;
  A3SyncLoop loop(std::move(opt));

  constexpr std::int64_t kPeriodNs = 10'000'000;
  constexpr std::int64_t kBase = 10'000'000'000;
  const auto now_ns = SystemNowNs();
  auto recv_base_ns = (now_ns / kPeriodNs) * kPeriodNs;
  auto kLatestRecvNs = recv_base_ns + 1'200'000;
  if (kLatestRecvNs > now_ns) kLatestRecvNs -= kPeriodNs;
  InjectJointGroup(loop, kBase, kLatestRecvNs - 50'000, 1.0);
  InjectImuGroup(loop, kBase + 100'000, kLatestRecvNs, 1.1);

  std::int64_t phase_ns = 0;
  std::int64_t latest_recv_ns = 0;
  std::int64_t pair_skew_ns = 0;
  ASSERT_TRUE(loop.EstimateAutoPhaseNs(500'000, phase_ns, latest_recv_ns,
                                       pair_skew_ns));

  EXPECT_EQ(latest_recv_ns, kLatestRecvNs);
  EXPECT_EQ(pair_skew_ns, 100'000);
  EXPECT_EQ(phase_ns, 1'500'000);
}

TEST(A3SyncLoop, MinSkewPairEmitsUnalignedWhenPairSkewExceedsThreshold) {
  auto cfg = MinSkewPairTestConfig();
  cfg.max_group_pair_skew_ns = 1'000'000;

  a3_sync::A3SyncLoop::Options opt;
  opt.sync = cfg;
  A3SyncLoop loop(std::move(opt));

  std::mutex mu;
  std::condition_variable cv;
  robot_io::RobotState observed;
  bool got = false;
  loop.RegisterStateCallback([&](const robot_io::RobotState& s) {
    if (!s.sync_complete) return;
    std::lock_guard<std::mutex> lk(mu);
    if (!got) {
      observed = s;
      got = true;
      cv.notify_all();
    }
  });

  constexpr std::int64_t kBase = 3'000'000'000;
  const auto recv = SystemNowNs();
  InjectJointGroup(loop, kBase + 0, recv, 0.0);
  InjectImuGroup(loop, kBase + 2'000'000, recv, 2.0);

  loop.Start();
  {
    std::unique_lock<std::mutex> lk(mu);
    EXPECT_TRUE(cv.wait_for(lk, std::chrono::milliseconds(100),
                            [&] { return got; }));
  }
  loop.Stop();

  ASSERT_TRUE(got);
  EXPECT_TRUE(observed.sync_complete);
  EXPECT_FALSE(observed.sync_aligned);
  EXPECT_EQ(observed.timestamp_ns, kBase + 2'000'000);
  EXPECT_EQ(observed.sync_skew_ns, 2'000'000);
}

TEST(A3SyncLoop, MinSkewPairRejectsGroupsWithLargeInternalSkew) {
  auto cfg = MinSkewPairTestConfig();
  cfg.max_group_internal_skew_ns = 250'000;

  a3_sync::A3SyncLoop::Options opt;
  opt.sync = cfg;
  A3SyncLoop loop(std::move(opt));

  std::mutex mu;
  std::condition_variable cv;
  robot_io::RobotState observed;
  int callbacks = 0;
  loop.RegisterStateCallback([&](const robot_io::RobotState& s) {
    std::lock_guard<std::mutex> lk(mu);
    observed = s;
    ++callbacks;
    cv.notify_all();
  });

  constexpr std::int64_t kBase = 4'000'000'000;
  const auto recv = SystemNowNs();
  loop.Start();

  loop.OnWaistState(MakeWaistSample(kBase, recv));
  loop.OnLegState(MakeLegSample(kBase + 500'000, recv));
  loop.OnArmState(MakeArmSample(kBase, recv));
  loop.OnNeckState(MakeNeckSample(kBase, recv));
  InjectImuGroup(loop, kBase, recv);

  {
    std::unique_lock<std::mutex> lk(mu);
    EXPECT_FALSE(cv.wait_for(lk, std::chrono::milliseconds(30),
                             [&] { return callbacks > 0; }));
  }

  InjectJointGroup(loop, kBase + 2'000'000, recv, 2.0);
  InjectImuGroup(loop, kBase + 2'500'000, recv, 2.5);
  {
    std::unique_lock<std::mutex> lk(mu);
    EXPECT_TRUE(cv.wait_for(lk, std::chrono::milliseconds(100),
                            [&] { return callbacks > 0; }));
  }
  loop.Stop();

  EXPECT_EQ(callbacks, 1);
  EXPECT_TRUE(observed.sync_complete);
  EXPECT_TRUE(observed.sync_aligned);
  EXPECT_EQ(observed.timestamp_ns, kBase + 2'500'000);
  EXPECT_EQ(observed.sync_skew_ns, 500'000);
}

TEST(A3SyncLoop, ProducesAlignedFrames) {
  a3_sync::SyncConfig cfg;
  cfg.sync_hz          = 200.0;
  cfg.align_delay_ns   = 1'000'000;    // 1 ms
  cfg.max_skew_ns      = 5'000'000;    // 5 ms
  cfg.max_backtrack    = 200;

  a3_sync::A3SyncLoop::Options opt;
  opt.sync = cfg;

  A3SyncLoop loop(std::move(opt));

  std::atomic<int> frames{0};
  std::atomic<int> aligned_frames{0};
  std::atomic<std::int64_t> last_tick{0};

  loop.RegisterStateCallback([&](const robot_io::RobotState& s) {
    frames.fetch_add(1);
    last_tick.store(s.timestamp_ns);
    // tick increments monotonically
    if (s.tick > 0) {
      // Consider "aligned" if skew ≤ max_skew_ns — we re-derive by requiring
      // a complete sample for all channels (which AssembleState always fills,
      // but stats will tell us complete/aligned). For this test we just count
      // invocations; alignment asserted below via Statistics().
    }
    (void)aligned_frames;
  });

  // Feeder thread — pump synthetic samples into all 6 rings at ~1 kHz.
  std::atomic<bool> stop{false};
  std::thread feeder([&] {
    while (!stop.load()) {
      const auto t = SystemNowNs();
      InjectJointGroup(loop, t, t);
      InjectImuGroup(loop, t, t);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  loop.Start();
  EXPECT_TRUE(loop.Running());
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  loop.Stop();
  EXPECT_FALSE(loop.Running());
  stop.store(true);
  feeder.join();

  // At 200 Hz over ~200 ms we expect roughly 40 frames. Allow a wide band.
  EXPECT_GE(frames.load(), 20);

  auto snap = loop.Statistics();
  EXPECT_GE(snap.tick_total, static_cast<std::uint64_t>(frames.load()));
  // Most frames should be aligned (skew between 1 kHz feeders tiny).
  EXPECT_GT(snap.frame_aligned_total * 2, snap.tick_total)
      << "expected > 50% aligned, got " << snap.frame_aligned_total << " / "
      << snap.tick_total;
  EXPECT_LE(snap.last_skew_ns, static_cast<std::int64_t>(5'000'000));
}

TEST(A3SyncLoop, HeldSamplesExpireWhenTooOld) {
  a3_sync::SyncConfig cfg;
  // HeaderInterp emits every control tick, including incomplete frames after
  // held samples expire. MinSkewPair intentionally emits only complete pairs,
  // so it cannot exercise the stale-frame accounting asserted below.
  cfg.sync_mode          = a3_sync::SyncMode::HeaderInterp;
  cfg.sync_hz            = 100.0;
  cfg.align_delay_ns     = 0;
  cfg.max_skew_ns        = 5'000'000;     // 5 ms
  cfg.max_sample_age_ns  = 40'000'000;    // 40 ms
  cfg.max_backtrack      = 200;

  a3_sync::A3SyncLoop::Options opt;
  opt.sync = cfg;

  A3SyncLoop loop(std::move(opt));

  std::atomic<int> complete_frames{0};
  std::atomic<int> incomplete_frames{0};

  loop.RegisterStateCallback([&](const robot_io::RobotState& s) {
    if (s.sync_complete) {
      complete_frames.fetch_add(1);
    } else {
      incomplete_frames.fetch_add(1);
    }
  });

  loop.Start();
  EXPECT_TRUE(loop.Running());

  const auto t = SystemNowNs();
  InjectJointGroup(loop, t, t);
  InjectImuGroup(loop, t, t);

  std::this_thread::sleep_for(std::chrono::milliseconds(160));
  loop.Stop();

  EXPECT_GT(complete_frames.load(), 0);
  EXPECT_GT(incomplete_frames.load(), 0);

  const auto snap = loop.Statistics();
  EXPECT_GT(snap.stale_waist, 0u);
  EXPECT_GT(snap.stale_leg, 0u);
  EXPECT_GT(snap.stale_arm, 0u);
  EXPECT_GT(snap.stale_neck, 0u);
  EXPECT_GT(snap.stale_pelvis_imu, 0u);
  EXPECT_GT(snap.stale_torso_imu, 0u);
}

TEST(A3SyncLoop, FreshnessUsesReceiveStampWhenHeaderClockDiffers) {
  a3_sync::SyncConfig cfg;
  cfg.sync_hz            = 100.0;
  cfg.align_delay_ns     = 0;
  cfg.max_skew_ns        = 5'000'000;     // 5 ms
  cfg.max_sample_age_ns  = 80'000'000;    // 80 ms
  cfg.max_backtrack      = 200;

  a3_sync::A3SyncLoop::Options opt;
  opt.sync = cfg;

  A3SyncLoop loop(std::move(opt));

  std::atomic<int> complete_frames{0};
  loop.RegisterStateCallback([&](const robot_io::RobotState& s) {
    if (s.sync_complete && s.sync_aligned) complete_frames.fetch_add(1);
  });

  loop.Start();

  const auto recv = SystemNowNs();
  constexpr std::int64_t kForeignClockStamp = 1'000'000'000;
  InjectJointGroup(loop, kForeignClockStamp, recv);
  InjectImuGroup(loop, kForeignClockStamp, recv);

  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  loop.Stop();

  EXPECT_GT(complete_frames.load(), 0);
}

TEST(A3SyncLoop, StartStopIdempotent) {
  A3SyncLoop::Options opt;
  opt.sync.sync_hz = 100.0;
  A3SyncLoop loop(opt);
  loop.Start();
  loop.Start();  // second call should be a no-op
  EXPECT_TRUE(loop.Running());
  loop.Stop();
  loop.Stop();  // idempotent
  EXPECT_FALSE(loop.Running());
}
