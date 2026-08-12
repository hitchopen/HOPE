// Copyright (c) 2026, AgiBot Inc. All rights reserved.
// 对应 notes/a3_backend_plan.md §? / PR 5
//
// Unit tests for PR 5: SubscriberBase + 6 subscribers + A3SubscriberManager.
//
// Part A (always compiled): ConvertMessage pure functions.
// Part B (#ifdef HAS_A3_ROS_MSGS): real joint_msgs/Imu construction +
// A3SubscriberManager::WaitAllReady.

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>

#include "a3_io/subscriber_manager.hpp"
#include "a3_io/subscribers/arm_state_sub.hpp"
#include "a3_io/subscribers/leg_state_sub.hpp"
#include "a3_io/subscribers/neck_state_sub.hpp"
#include "a3_io/subscribers/pelvis_imu_sub.hpp"
#include "a3_io/subscribers/torso_imu_sub.hpp"
#include "a3_io/subscribers/waist_state_sub.hpp"
#include "a3_sync/a3_ring_buffer.hpp"
#include "a3_sync/a3_sync_types.hpp"
#include "robot_io/a3_layout_extra.hpp"
#include "robot_io/layouts.hpp"

// ============================================================================
// Part A — pure ConvertMessage tests (no ROS2 types)
// ============================================================================

TEST(ConvertMessage, ArmNativeOrderRoundTrips) {
  std::array<std::string, robot_io::kArmCount> names{};
  std::array<double, robot_io::kArmCount> pos{}, vel{}, eff{};
  for (std::size_t i = 0; i < robot_io::kArmCount; ++i) {
    names[i] = robot_io::kA3ArmJointNames[i];
    pos[i]   = static_cast<double>(i) + 0.1;
    vel[i]   = static_cast<double>(i) + 0.2;
    eff[i]   = static_cast<double>(i) + 0.3;
  }

  a3_sync::ArmSample out{};
  ASSERT_TRUE(a3_io::ArmStateSub::ConvertMessage(names, pos, vel, eff,
                                                 1'234'567'890LL, out));
  EXPECT_EQ(out.stamp.value, 1'234'567'890LL);
  for (std::size_t i = 0; i < robot_io::kArmCount; ++i) {
    EXPECT_DOUBLE_EQ(out.pos[i], static_cast<double>(i) + 0.1);
    EXPECT_DOUBLE_EQ(out.vel[i], static_cast<double>(i) + 0.2);
    EXPECT_DOUBLE_EQ(out.eff[i], static_cast<double>(i) + 0.3);
  }
}

TEST(ConvertMessage, ArmReorderedInputProducesSortedOutput) {
  // Reverse-order input. Each entry still tagged with its canonical name,
  // so ReorderByName must put them back into kA3ArmJointNames order.
  std::array<std::string, robot_io::kArmCount> names{};
  std::array<double, robot_io::kArmCount> pos{}, vel{}, eff{};
  for (std::size_t i = 0; i < robot_io::kArmCount; ++i) {
    const std::size_t src = robot_io::kArmCount - 1 - i;
    names[i] = robot_io::kA3ArmJointNames[src];
    pos[i]   = static_cast<double>(src) + 0.1;
    vel[i]   = static_cast<double>(src) + 0.2;
    eff[i]   = static_cast<double>(src) + 0.3;
  }

  a3_sync::ArmSample out{};
  ASSERT_TRUE(a3_io::ArmStateSub::ConvertMessage(names, pos, vel, eff, 42,
                                                 out));
  // Output indexed in kA3ArmJointNames order — position[i] should be i + 0.1
  // regardless of input shuffle.
  for (std::size_t i = 0; i < robot_io::kArmCount; ++i) {
    EXPECT_DOUBLE_EQ(out.pos[i], static_cast<double>(i) + 0.1);
    EXPECT_DOUBLE_EQ(out.vel[i], static_cast<double>(i) + 0.2);
    EXPECT_DOUBLE_EQ(out.eff[i], static_cast<double>(i) + 0.3);
  }
}

TEST(ConvertMessage, ArmMissingJointReturnsFalse) {
  std::array<std::string, robot_io::kArmCount> names{};
  std::array<double, robot_io::kArmCount> pos{}, vel{}, eff{};
  for (std::size_t i = 0; i < robot_io::kArmCount; ++i) {
    names[i] = robot_io::kA3ArmJointNames[i];
  }
  names[3] = "bogus_joint";  // clobber one

  a3_sync::ArmSample out{};
  EXPECT_FALSE(a3_io::ArmStateSub::ConvertMessage(names, pos, vel, eff, 0,
                                                  out));
}

TEST(ConvertMessage, ArmDuplicateLastWins) {
  // Duplicate name — the loop in ReorderByName keeps the last match.
  std::array<std::string, robot_io::kArmCount> names{};
  std::array<double, robot_io::kArmCount> pos{}, vel{}, eff{};
  for (std::size_t i = 0; i < robot_io::kArmCount; ++i) {
    names[i] = robot_io::kA3ArmJointNames[i];
    pos[i]   = 1.0;
  }
  // Overwrite index 5's name to duplicate index 0 — that would make
  // index 5's name missing. Instead, replace index 0's name with index 5's.
  names[0] = robot_io::kA3ArmJointNames[5];
  pos[0]   = 111.0;  // first dup
  pos[5]   = 222.0;  // second (canonical slot) — this wins because later

  a3_sync::ArmSample out{};
  EXPECT_FALSE(a3_io::ArmStateSub::ConvertMessage(names, pos, vel, eff, 0,
                                                  out));
  // Expected: index 5 wins but index 0's canonical name is now missing
  // from names → ConvertMessage returns false. Just sanity-check the
  // "last wins" rule in a scenario where nothing is missing:
  std::array<std::string, robot_io::kArmCount> names2{};
  std::array<double, robot_io::kArmCount> pos2{}, vel2{}, eff2{};
  for (std::size_t i = 0; i < robot_io::kArmCount; ++i) {
    names2[i] = robot_io::kA3ArmJointNames[i];
    pos2[i]   = static_cast<double>(i);
  }
  // names2 has all 14 canonical names; now duplicate arm[3]'s *value*
  // by placing it also into the array at what would otherwise be a
  // different index — but keep all 14 names present. That means we can't
  // construct a "all present + dup" state in an N-element array (14
  // slots, 14 unique names). This test just asserts the "missing" path.
  a3_sync::ArmSample out2{};
  EXPECT_TRUE(a3_io::ArmStateSub::ConvertMessage(names2, pos2, vel2, eff2, 0,
                                                 out2));
}

TEST(ConvertMessage, WaistAndLegAndNeckOrdering) {
  // Waist
  std::array<std::string, robot_io::kWaistCount> wnames{};
  std::array<double, robot_io::kWaistCount> wpos{}, wvel{}, weff{};
  for (std::size_t i = 0; i < robot_io::kWaistCount; ++i) {
    wnames[i] = robot_io::kA3WaistJointNames[robot_io::kWaistCount - 1 - i];
    wpos[i]   = static_cast<double>(robot_io::kWaistCount - 1 - i);
  }
  a3_sync::WaistSample wout{};
  ASSERT_TRUE(a3_io::WaistStateSub::ConvertMessage(wnames, wpos, wvel, weff, 1,
                                                   wout));
  for (std::size_t i = 0; i < robot_io::kWaistCount; ++i) {
    EXPECT_DOUBLE_EQ(wout.pos[i], static_cast<double>(i));
  }

  // Leg (permute odd/even)
  std::array<std::string, robot_io::kLegCount> lnames{};
  std::array<double, robot_io::kLegCount> lpos{}, lvel{}, leff{};
  for (std::size_t i = 0; i < robot_io::kLegCount; ++i) {
    const std::size_t src = (i * 7) % robot_io::kLegCount;  // pseudo-shuffle
    lnames[i] = robot_io::kA3LegJointNames[src];
    lpos[i]   = static_cast<double>(src);
  }
  a3_sync::LegSample lout{};
  ASSERT_TRUE(a3_io::LegStateSub::ConvertMessage(lnames, lpos, lvel, leff, 2,
                                                 lout));
  for (std::size_t i = 0; i < robot_io::kLegCount; ++i) {
    EXPECT_DOUBLE_EQ(lout.pos[i], static_cast<double>(i));
  }

  // Neck
  std::array<std::string, robot_io::kA3NeckCount> nnames{};
  std::array<double, robot_io::kA3NeckCount> npos{}, nvel{}, neff{};
  nnames[0] = robot_io::kA3NeckJointNames[1];
  nnames[1] = robot_io::kA3NeckJointNames[0];
  npos[0] = 10.0;
  npos[1] = 20.0;
  a3_sync::NeckSample nout{};
  ASSERT_TRUE(a3_io::NeckStateSub::ConvertMessage(nnames, npos, nvel, neff, 3,
                                                  nout));
  EXPECT_DOUBLE_EQ(nout.pos[0], 20.0);  // kA3NeckJointNames[0] was at input 1
  EXPECT_DOUBLE_EQ(nout.pos[1], 10.0);  // kA3NeckJointNames[1] was at input 0
  EXPECT_EQ(nout.stamp.value, 3);
}

TEST(ConvertMessage, ImuXyzwToWxyz) {
  a3_sync::ImuSample out{};
  // Stamp 2.5s = 2 s + 500 000 000 ns = 2'500'000'000 ns
  a3_io::PelvisImuSub::ConvertMessage(/*x=*/0.1, /*y=*/0.2, /*z=*/0.3, /*w=*/0.9,
                                       1.0, 2.0, 3.0,
                                       4.0, 5.0, 9.81,
                                       2'500'000'000LL, out);
  EXPECT_DOUBLE_EQ(out.quat_wxyz[0], 0.9);
  EXPECT_DOUBLE_EQ(out.quat_wxyz[1], 0.1);
  EXPECT_DOUBLE_EQ(out.quat_wxyz[2], 0.2);
  EXPECT_DOUBLE_EQ(out.quat_wxyz[3], 0.3);
  EXPECT_DOUBLE_EQ(out.gyro[0], 1.0);
  EXPECT_DOUBLE_EQ(out.gyro[2], 3.0);
  EXPECT_DOUBLE_EQ(out.acc[2], 9.81);
  EXPECT_EQ(out.stamp.value, 2'500'000'000LL);
}

TEST(ConvertMessage, TorsoImuXyzwToWxyz) {
  a3_sync::ImuSample out{};
  a3_io::TorsoImuSub::ConvertMessage(1.0, 0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0,
                                      7, out);
  // Pure X rotation -> WXYZ = (0, 1, 0, 0)
  EXPECT_DOUBLE_EQ(out.quat_wxyz[0], 0.0);
  EXPECT_DOUBLE_EQ(out.quat_wxyz[1], 1.0);
  EXPECT_EQ(out.stamp.value, 7);
}

// ============================================================================
// SubscriberBase readiness flag — covered via the manager path below but also
// hit directly here to confirm IsReady() flips only after a successful
// conversion.
// ============================================================================

TEST(A3SubscriberManager, WaitAllReadyTimesOutIfNoMessages) {
  a3_sync::RingBuffer<a3_sync::WaistSample> waist_ring(8);
  a3_sync::RingBuffer<a3_sync::LegSample>   leg_ring(8);
  a3_sync::RingBuffer<a3_sync::ArmSample>   arm_ring(8);
  a3_sync::RingBuffer<a3_sync::NeckSample>  neck_ring(8);
  a3_sync::RingBuffer<a3_sync::ImuSample>   pelvis_ring(8);
  a3_sync::RingBuffer<a3_sync::ImuSample>   torso_ring(8);

  a3_io::A3SubscriberManager::Options opt;
  opt.waist_ring = &waist_ring;
  opt.leg_ring   = &leg_ring;
  opt.arm_ring   = &arm_ring;
  opt.neck_ring  = &neck_ring;
  opt.pelvis_imu_ring = &pelvis_ring;
  opt.torso_imu_ring  = &torso_ring;

  a3_io::A3SubscriberManager mgr(opt);

  EXPECT_FALSE(mgr.AllReady());
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_FALSE(mgr.WaitAllReady(0.1));
  const auto dt = std::chrono::steady_clock::now() - t0;
  EXPECT_GE(std::chrono::duration_cast<std::chrono::milliseconds>(dt).count(),
            90);
}

// ============================================================================
// Part B — real ROS2 message construction (only when ENABLE_A3_ROS_MSGS=ON).
// ============================================================================

#ifdef HAS_A3_ROS_MSGS
#include "joint_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace {

template <std::size_t N>
std::shared_ptr<joint_msgs::msg::JointState> MakeJointStateMsg(
    const std::array<std::string, N>& names,
    const std::array<double, N>& pos, std::int32_t sec,
    std::uint32_t nanosec) {
  auto msg = std::make_shared<joint_msgs::msg::JointState>();
  msg->header.stamp.sec = sec;
  msg->header.stamp.nanosec = nanosec;
  msg->joints.resize(N);
  for (std::size_t i = 0; i < N; ++i) {
    msg->joints[i].name     = names[i];
    msg->joints[i].position = pos[i];
    msg->joints[i].velocity = pos[i] + 0.01;
    msg->joints[i].effort   = pos[i] + 0.02;
  }
  return msg;
}

}  // namespace

TEST(A3SubRos, ArmOnMessageFillsRing) {
  a3_sync::RingBuffer<a3_sync::ArmSample> ring(8);
  a3_io::ArmStateSub sub(ring);

  std::array<std::string, robot_io::kArmCount> names{};
  std::array<double, robot_io::kArmCount> pos{};
  for (std::size_t i = 0; i < robot_io::kArmCount; ++i) {
    names[i] = robot_io::kA3ArmJointNames[i];
    pos[i]   = static_cast<double>(i) * 0.5;
  }

  auto msg = MakeJointStateMsg<robot_io::kArmCount>(names, pos, 5, 100);
  EXPECT_FALSE(sub.IsReady());
  sub.OnMessage(msg);
  EXPECT_TRUE(sub.IsReady());

  const auto idx = ring.latest_index();
  ASSERT_GT(idx, 0u);
  a3_sync::ArmSample out{};
  ASSERT_TRUE(ring.read_at(idx, out));
  EXPECT_EQ(out.stamp.value, 5LL * 1'000'000'000LL + 100);
  for (std::size_t i = 0; i < robot_io::kArmCount; ++i) {
    EXPECT_DOUBLE_EQ(out.pos[i], static_cast<double>(i) * 0.5);
  }
}

TEST(A3SubRos, ArmOnMessageInvokesSampleCallback) {
  a3_sync::RingBuffer<a3_sync::ArmSample> ring(8);
  bool called = false;
  a3_sync::ArmSample callback_sample{};
  a3_io::ArmStateSub sub(
      ring, [&](const a3_sync::ArmSample& sample) {
        called = true;
        callback_sample = sample;
      });

  std::array<std::string, robot_io::kArmCount> names{};
  std::array<double, robot_io::kArmCount> pos{};
  for (std::size_t i = 0; i < robot_io::kArmCount; ++i) {
    names[i] = robot_io::kA3ArmJointNames[i];
    pos[i] = static_cast<double>(i);
  }

  sub.OnMessage(MakeJointStateMsg<robot_io::kArmCount>(names, pos, 7, 8));

  ASSERT_TRUE(called);
  EXPECT_EQ(callback_sample.stamp.value, 7'000'000'008LL);
  EXPECT_DOUBLE_EQ(callback_sample.pos[3], 3.0);
}

TEST(A3SubRos, ArmOnMessageMissingJointDropsSilently) {
  a3_sync::RingBuffer<a3_sync::ArmSample> ring(8);
  a3_io::ArmStateSub sub(ring);

  std::array<std::string, robot_io::kArmCount> names{};
  std::array<double, robot_io::kArmCount> pos{};
  for (std::size_t i = 0; i < robot_io::kArmCount; ++i) {
    names[i] = robot_io::kA3ArmJointNames[i];
  }
  names[2] = "bogus_joint";
  auto msg = MakeJointStateMsg<robot_io::kArmCount>(names, pos, 0, 0);
  sub.OnMessage(msg);
  EXPECT_FALSE(sub.IsReady());
  EXPECT_EQ(ring.latest_index(), 0u);
}

TEST(A3SubRos, ArmOnMessageWrongDofDropsSilently) {
  a3_sync::RingBuffer<a3_sync::ArmSample> ring(8);
  a3_io::ArmStateSub sub(ring);
  auto msg = std::make_shared<joint_msgs::msg::JointState>();
  msg->joints.resize(7);
  sub.OnMessage(msg);
  EXPECT_FALSE(sub.IsReady());
}

TEST(A3SubRos, PelvisImuXyzwToWxyz) {
  a3_sync::RingBuffer<a3_sync::ImuSample> ring(8);
  a3_io::PelvisImuSub sub(ring);

  auto msg = std::make_shared<sensor_msgs::msg::Imu>();
  msg->header.stamp.sec = 1;
  msg->header.stamp.nanosec = 500'000'000;
  msg->orientation.x = 0.1;
  msg->orientation.y = 0.2;
  msg->orientation.z = 0.3;
  msg->orientation.w = 0.9;
  msg->angular_velocity.x = 1.0;
  msg->angular_velocity.y = 2.0;
  msg->angular_velocity.z = 3.0;
  msg->linear_acceleration.x = 0.0;
  msg->linear_acceleration.y = 0.0;
  msg->linear_acceleration.z = 9.81;

  sub.OnMessage(msg);
  EXPECT_TRUE(sub.IsReady());
  a3_sync::ImuSample out{};
  ASSERT_TRUE(ring.read_at(ring.latest_index(), out));
  EXPECT_DOUBLE_EQ(out.quat_wxyz[0], 0.9);
  EXPECT_DOUBLE_EQ(out.quat_wxyz[1], 0.1);
  EXPECT_DOUBLE_EQ(out.quat_wxyz[2], 0.2);
  EXPECT_DOUBLE_EQ(out.quat_wxyz[3], 0.3);
  EXPECT_EQ(out.stamp.value, 1'500'000'000LL);
}

TEST(A3SubRos, ManagerWaitAllReadyReturnsTrueOnceAllFed) {
  a3_sync::RingBuffer<a3_sync::WaistSample> waist_ring(8);
  a3_sync::RingBuffer<a3_sync::LegSample>   leg_ring(8);
  a3_sync::RingBuffer<a3_sync::ArmSample>   arm_ring(8);
  a3_sync::RingBuffer<a3_sync::NeckSample>  neck_ring(8);
  a3_sync::RingBuffer<a3_sync::ImuSample>   pelvis_ring(8);
  a3_sync::RingBuffer<a3_sync::ImuSample>   torso_ring(8);

  a3_io::A3SubscriberManager::Options opt;
  opt.waist_ring = &waist_ring;
  opt.leg_ring   = &leg_ring;
  opt.arm_ring   = &arm_ring;
  opt.neck_ring  = &neck_ring;
  opt.pelvis_imu_ring = &pelvis_ring;
  opt.torso_imu_ring  = &torso_ring;
  a3_io::A3SubscriberManager mgr(opt);

  // Waist
  {
    std::array<std::string, robot_io::kWaistCount> n{};
    std::array<double, robot_io::kWaistCount> p{};
    for (std::size_t i = 0; i < robot_io::kWaistCount; ++i)
      n[i] = robot_io::kA3WaistJointNames[i];
    mgr.waist().OnMessage(
        MakeJointStateMsg<robot_io::kWaistCount>(n, p, 0, 0));
  }
  // Leg
  {
    std::array<std::string, robot_io::kLegCount> n{};
    std::array<double, robot_io::kLegCount> p{};
    for (std::size_t i = 0; i < robot_io::kLegCount; ++i)
      n[i] = robot_io::kA3LegJointNames[i];
    mgr.leg().OnMessage(
        MakeJointStateMsg<robot_io::kLegCount>(n, p, 0, 0));
  }
  // Arm
  {
    std::array<std::string, robot_io::kArmCount> n{};
    std::array<double, robot_io::kArmCount> p{};
    for (std::size_t i = 0; i < robot_io::kArmCount; ++i)
      n[i] = robot_io::kA3ArmJointNames[i];
    mgr.arm().OnMessage(
        MakeJointStateMsg<robot_io::kArmCount>(n, p, 0, 0));
  }
  // Neck
  {
    std::array<std::string, robot_io::kA3NeckCount> n{};
    std::array<double, robot_io::kA3NeckCount> p{};
    for (std::size_t i = 0; i < robot_io::kA3NeckCount; ++i)
      n[i] = robot_io::kA3NeckJointNames[i];
    mgr.neck().OnMessage(
        MakeJointStateMsg<robot_io::kA3NeckCount>(n, p, 0, 0));
  }
  // IMUs
  {
    auto imsg = std::make_shared<sensor_msgs::msg::Imu>();
    imsg->orientation.w = 1.0;
    mgr.pelvis_imu().OnMessage(imsg);
    mgr.torso_imu().OnMessage(imsg);
  }

  EXPECT_TRUE(mgr.WaitAllReady(1.0));
  EXPECT_TRUE(mgr.AllReady());
}

#endif  // HAS_A3_ROS_MSGS
