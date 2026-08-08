#include "a3_deploy/a3_yaw_alignment.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#define private public
#include "a3_deploy/a3_zmq_smpl_source.hpp"
#undef private

#include "motion_data_reader.hpp"

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

std::array<double, 4> YawQuat(double yaw) {
  return a3_deploy::yaw_alignment::QuatFromYawRad(yaw);
}

std::shared_ptr<MotionSequence> MakeSmplMotion(double root_yaw_rad) {
  auto motion = std::make_shared<MotionSequence>();
  motion->ReserveCapacity(
      /*max_frames=*/10,
      /*joints=*/29,
      /*bodies=*/1,
      /*body_quaternions=*/1,
      /*smpl_joints=*/24,
      /*smpl_poses=*/21);
  motion->timesteps = 10;
  const auto root = YawQuat(root_yaw_rad);
  for (int f = 0; f < motion->timesteps; ++f) {
    motion->BodyQuaternions(f)[0] = root;
    for (int j = 0; j < motion->GetNumJoints(); ++j) {
      motion->JointPositions(f)[j] = 0.0;
      motion->JointVelocities(f)[j] = 0.0;
    }
    for (int j = 0; j < motion->GetNumSmplJoints(); ++j) {
      motion->SmplJoints(f)[j] = {0.0, 0.0, 0.0};
    }
  }
  return motion;
}

}  // namespace

TEST(A3YawAlignment, ComputesWrappedYawOffset) {
  const auto robot = YawQuat(-170.0 * kPi / 180.0);
  const auto reference = YawQuat(170.0 * kPi / 180.0);

  const double offset =
      a3_deploy::yaw_alignment::ComputeYawOffsetRad(robot, reference);

  EXPECT_NEAR(offset, 20.0 * kPi / 180.0, 1e-9);
}

TEST(A3YawAlignment, ApplyYawOffsetAlignsReferenceYaw) {
  const auto robot = YawQuat(100.0 * kPi / 180.0);
  const auto reference = YawQuat(30.0 * kPi / 180.0);
  const double offset =
      a3_deploy::yaw_alignment::ComputeYawOffsetRad(robot, reference);

  const auto aligned =
      a3_deploy::yaw_alignment::ApplyYawOffset(offset, reference);

  EXPECT_NEAR(a3_deploy::yaw_alignment::QuatYawRad(aligned),
              a3_deploy::yaw_alignment::QuatYawRad(robot), 1e-9);
}

TEST(A3ZmqSmplSource, YawOffsetChangesRootOrientationTokenizer) {
  a3_deploy::A3ZmqSmplSource source;
  source.motion_ = MakeSmplMotion(kPi / 2.0);
  source.playback_frame_ = 0;

  std::array<float, a3_deploy::kA3SmplTokenizerTotalFloats> no_offset{};
  ASSERT_TRUE(source.BuildTokenizerSlice(
      YawQuat(0.0),
      /*reference_yaw_offset_rad=*/0.0,
      no_offset,
      /*advance_playback=*/false));

  std::array<float, a3_deploy::kA3SmplTokenizerTotalFloats> aligned{};
  const double offset = source.ComputeLatestYawOffsetRad(YawQuat(0.0));
  ASSERT_TRUE(source.BuildTokenizerSlice(
      YawQuat(0.0),
      offset,
      aligned,
      /*advance_playback=*/false));

  const std::size_t root_ori_offset = a3_deploy::kA3SmplJointsFloats;
  EXPECT_NEAR(no_offset[root_ori_offset + 0], 0.0f, 1e-5f);
  EXPECT_NEAR(no_offset[root_ori_offset + 1], -1.0f, 1e-5f);
  EXPECT_NEAR(no_offset[root_ori_offset + 2], 1.0f, 1e-5f);
  EXPECT_NEAR(no_offset[root_ori_offset + 3], 0.0f, 1e-5f);

  EXPECT_NEAR(aligned[root_ori_offset + 0], 1.0f, 1e-5f);
  EXPECT_NEAR(aligned[root_ori_offset + 1], 0.0f, 1e-5f);
  EXPECT_NEAR(aligned[root_ori_offset + 2], 0.0f, 1e-5f);
  EXPECT_NEAR(aligned[root_ori_offset + 3], 1.0f, 1e-5f);
}
