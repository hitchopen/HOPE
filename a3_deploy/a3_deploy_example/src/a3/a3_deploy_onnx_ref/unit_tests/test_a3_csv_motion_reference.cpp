// Copyright (c) 2026, AgiBot Inc. All rights reserved.
#include <gtest/gtest.h>

#include "a3_deploy/a3_csv_motion_reference.hpp"
#include "a3_policy_parameters.hpp"
#include "robot_io/a3_layout_extra.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>
#include <cstdlib>
#include <unistd.h>

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

constexpr std::array<const char*, robot_io::kA3Dof> kJointNames = {{
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "head_yaw_joint",
    "head_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
}};

double SyntheticMujocoRad(int row, int mj29) {
  return static_cast<double>(row) + 0.01 * static_cast<double>(mj29);
}

std::array<double, 4> QuatFromYawDeg(double yaw_deg) {
  const double half = 0.5 * yaw_deg * kDegToRad;
  return {std::cos(half), 0.0, 0.0, std::sin(half)};
}

class A3CsvMotionReference : public ::testing::Test {
 protected:
  void SetUp() override {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("a3_csv_ref_test_" + std::to_string(::getpid()) + "_" +
                std::to_string(
                    ::testing::UnitTest::GetInstance()->random_seed()));
    std::filesystem::create_directories(tmp_dir_);
    csv_path_ = tmp_dir_ / "synthetic_a3.csv";
  }

  void TearDown() override {
    std::error_code ec;
    std::filesystem::remove_all(tmp_dir_, ec);
  }

  void WriteCsv(int rows, bool yaw_90 = false, bool dof_suffix = false) {
    std::vector<double> yaws(static_cast<std::size_t>(rows), 0.0);
    if (yaw_90 && rows > 0) yaws[0] = 90.0;
    WriteCsvWithYaw(yaws, dof_suffix);
  }

  void WriteCsvWithYaw(const std::vector<double>& yaws_deg,
                       bool dof_suffix = false) {
    std::ofstream f(csv_path_);
    ASSERT_TRUE(f.is_open());
    f << "Frame,root_translateX,root_translateY,root_translateZ,"
         "root_rotateX,root_rotateY,root_rotateZ";
    for (const char* name : kJointNames) {
      f << "," << name;
      if (dof_suffix) f << "_dof";
    }
    f << "\n";
    f << std::fixed << std::setprecision(9);

    const int rows = static_cast<int>(yaws_deg.size());
    for (int r = 0; r < rows; ++r) {
      const double yaw = yaws_deg[static_cast<std::size_t>(r)];
      f << r << "," << 100.0 + r << "," << 200.0 << "," << 300.0
        << ",0,0," << yaw;

      for (int sdk = 0; sdk < robot_io::kA3Dof; ++sdk) {
        double rad = 99.0;  // neck filler; policy view skips SDK [3,4].
        for (int mj = 0; mj < robot_io::kA3PolicyDof; ++mj) {
          if (robot_io::kA3PolicyToSdkIdx[mj] == sdk) {
            rad = SyntheticMujocoRad(r, mj);
            break;
          }
        }
        f << "," << rad * kRadToDeg;
      }
      f << "\n";
    }
    ASSERT_TRUE(f.good());
  }

  std::filesystem::path tmp_dir_;
  std::filesystem::path csv_path_;
};

void ExpectYaw6dAt(const std::array<float, a3_deploy::kA3TokenizerFloatsPerTick>& tok,
                   int future_idx,
                   double yaw_deg,
                   double tol = 1e-6) {
  const double yaw = yaw_deg * kDegToRad;
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const std::size_t o = 580u + static_cast<std::size_t>(future_idx) * 6u;
  EXPECT_NEAR(tok[o + 0], c, tol);
  EXPECT_NEAR(tok[o + 1], -s, tol);
  EXPECT_NEAR(tok[o + 2], s, tol);
  EXPECT_NEAR(tok[o + 3], c, tol);
  EXPECT_NEAR(tok[o + 4], 0.0, tol);
  EXPECT_NEAR(tok[o + 5], 0.0, tol);
}

}  // namespace

TEST_F(A3CsvMotionReference, LoadsCsvAndBuildsCommandInIsaacLabOrder) {
  WriteCsv(4);

  a3_deploy::A3CsvMotionReference ref;
  a3_deploy::A3CsvMotionReferenceOptions opt;
  opt.source_fps = 50.0;
  opt.target_fps = 50.0;
  opt.future_frame_skip = 1;
  opt.on_end = a3_deploy::OnEndPolicy::kHoldLast;
  ASSERT_TRUE(ref.Load(csv_path_.string(), opt));

  // Matches sim2sim's np.arange(0, duration, 1/target_fps): final source frame
  // is excluded.
  ASSERT_EQ(ref.NumTicks(), 3u);
  EXPECT_TRUE(ref.Meta().has_initial_state);
  EXPECT_NEAR(ref.Meta().init_root_pos[0], 1.0, 1e-12);
  EXPECT_NEAR(ref.Meta().init_root_lin_vel[0], 0.5, 1e-12);

  std::array<float, a3_deploy::kA3TokenizerFloatsPerTick> tok{};
  ASSERT_TRUE(ref.BuildTokenizerSlice(0, {1.0, 0.0, 0.0, 0.0}, tok));

  for (int il = 0; il < 29; ++il) {
    const int mj = a3_mujoco_to_isaaclab[il];
    EXPECT_NEAR(tok[il], SyntheticMujocoRad(0, mj), 1e-5);
    EXPECT_NEAR(tok[29 + il], SyntheticMujocoRad(1, mj), 1e-5);
    EXPECT_NEAR(tok[58 + il], SyntheticMujocoRad(2, mj), 1e-5);
    EXPECT_NEAR(tok[9 * 29 + il], SyntheticMujocoRad(2, mj), 1e-5);

    EXPECT_NEAR(tok[290 + il], 50.0, 2e-4);
    EXPECT_NEAR(tok[290 + 9 * 29 + il], 50.0, 2e-4);
  }

  // Identity robot root and identity reference root -> first two rotation
  // matrix columns flattened row-major: [1,0, 0,1, 0,0].
  EXPECT_NEAR(tok[580], 1.0, 1e-6);
  EXPECT_NEAR(tok[581], 0.0, 1e-6);
  EXPECT_NEAR(tok[582], 0.0, 1e-6);
  EXPECT_NEAR(tok[583], 1.0, 1e-6);
  EXPECT_NEAR(tok[584], 0.0, 1e-6);
  EXPECT_NEAR(tok[585], 0.0, 1e-6);
}

TEST_F(A3CsvMotionReference, LoadsCsvWithDofSuffixJointHeaders) {
  WriteCsv(4, /*yaw_90=*/false, /*dof_suffix=*/true);

  a3_deploy::A3CsvMotionReference ref;
  a3_deploy::A3CsvMotionReferenceOptions opt;
  opt.source_fps = 50.0;
  opt.target_fps = 50.0;
  opt.future_frame_skip = 1;
  opt.on_end = a3_deploy::OnEndPolicy::kHoldLast;
  ASSERT_TRUE(ref.Load(csv_path_.string(), opt));

  std::array<float, a3_deploy::kA3TokenizerFloatsPerTick> tok{};
  ASSERT_TRUE(ref.BuildTokenizerSlice(0, {1.0, 0.0, 0.0, 0.0}, tok));

  for (int il = 0; il < 29; ++il) {
    const int mj = a3_mujoco_to_isaaclab[il];
    EXPECT_NEAR(tok[il], SyntheticMujocoRad(0, mj), 1e-5);
    EXPECT_NEAR(tok[29 + il], SyntheticMujocoRad(1, mj), 1e-5);
  }
}

TEST_F(A3CsvMotionReference, RootYawUsesScipyLowercaseXyzConvention) {
  WriteCsv(4, /*yaw_90=*/true);

  a3_deploy::A3CsvMotionReference ref;
  a3_deploy::A3CsvMotionReferenceOptions opt;
  opt.source_fps = 50.0;
  opt.target_fps = 50.0;
  opt.future_frame_skip = 1;
  ASSERT_TRUE(ref.Load(csv_path_.string(), opt));

  std::array<float, a3_deploy::kA3TokenizerFloatsPerTick> tok{};
  ASSERT_TRUE(ref.BuildTokenizerSlice(0, {1.0, 0.0, 0.0, 0.0}, tok));

  // Rz(+90deg), mat[:, :2].reshape(6) in NumPy row-major order.
  EXPECT_NEAR(tok[580], 0.0, 1e-6);
  EXPECT_NEAR(tok[581], -1.0, 1e-6);
  EXPECT_NEAR(tok[582], 1.0, 1e-6);
  EXPECT_NEAR(tok[583], 0.0, 1e-6);
  EXPECT_NEAR(tok[584], 0.0, 1e-6);
  EXPECT_NEAR(tok[585], 0.0, 1e-6);
}

TEST_F(A3CsvMotionReference, YawCompensationLocksStartAndKeepsClipYaw) {
  WriteCsvWithYaw({0.0, 30.0, 60.0, 90.0});

  a3_deploy::A3CsvMotionReference ref;
  a3_deploy::A3CsvMotionReferenceOptions opt;
  opt.source_fps = 50.0;
  opt.target_fps = 50.0;
  opt.future_frame_skip = 1;
  opt.on_end = a3_deploy::OnEndPolicy::kHoldLast;
  ASSERT_TRUE(ref.Load(csv_path_.string(), opt));
  ASSERT_EQ(ref.NumTicks(), 3u);

  const auto robot_yaw_90 = QuatFromYawDeg(90.0);
  const double yaw_offset_rad = ref.ComputeYawOffsetRad(robot_yaw_90);
  EXPECT_NEAR(yaw_offset_rad, 90.0 * kDegToRad, 1e-12);

  std::array<float, a3_deploy::kA3TokenizerFloatsPerTick> tok{};
  ASSERT_TRUE(ref.BuildTokenizerSlice(0, robot_yaw_90, yaw_offset_rad, tok));

  // Frame 0 is yaw-aligned to the robot, so robot-relative orientation is I.
  ExpectYaw6dAt(tok, 0, 0.0);
  // Later frames keep the clip's own yaw delta instead of being flattened.
  ExpectYaw6dAt(tok, 1, 30.0);
  ExpectYaw6dAt(tok, 2, 60.0);

  ASSERT_TRUE(ref.BuildTokenizerSlice(999, robot_yaw_90, yaw_offset_rad, tok));
  // hold_last clamps to the final generated tick while using the same offset.
  ExpectYaw6dAt(tok, 0, 60.0);
}

TEST_F(A3CsvMotionReference, HeldTokenizerRepeatsFrameAndZerosFutureVelocity) {
  WriteCsvWithYaw({0.0, 30.0, 60.0, 90.0});

  a3_deploy::A3CsvMotionReference ref;
  a3_deploy::A3CsvMotionReferenceOptions opt;
  opt.source_fps = 50.0;
  opt.target_fps = 50.0;
  opt.future_frame_skip = 1;
  opt.on_end = a3_deploy::OnEndPolicy::kHoldLast;
  ASSERT_TRUE(ref.Load(csv_path_.string(), opt));

  const auto robot_yaw_90 = QuatFromYawDeg(90.0);
  const double yaw_offset_rad = ref.ComputeYawOffsetRad(robot_yaw_90);

  std::array<float, a3_deploy::kA3TokenizerFloatsPerTick> tok{};
  ASSERT_TRUE(ref.BuildHeldTokenizerSlice(1, robot_yaw_90, yaw_offset_rad, tok));

  for (int future = 0; future < 10; ++future) {
    for (int il = 0; il < 29; ++il) {
      const int mj = a3_mujoco_to_isaaclab[il];
      EXPECT_NEAR(tok[future * 29 + il], SyntheticMujocoRad(1, mj), 1e-5);
      EXPECT_NEAR(tok[290 + future * 29 + il], 0.0, 1e-6);
    }
    ExpectYaw6dAt(tok, future, 30.0);
  }
}

TEST_F(A3CsvMotionReference, CsvFrameStrideMatchesConverterDownsample) {
  WriteCsv(13);

  a3_deploy::A3CsvMotionReference ref;
  a3_deploy::A3CsvMotionReferenceOptions opt;
  opt.source_fps = 30.0;
  opt.target_fps = 30.0;
  opt.csv_frame_stride = 4;
  opt.future_frame_skip = 1;
  ASSERT_TRUE(ref.Load(csv_path_.string(), opt));

  // Kept raw rows are 0, 4, 8, 12. The final source frame is excluded by the
  // canonical np.arange-style timeline, so target ticks are 0, 4, 8.
  ASSERT_EQ(ref.NumTicks(), 3u);

  std::array<float, a3_deploy::kA3TokenizerFloatsPerTick> tok{};
  ASSERT_TRUE(ref.BuildTokenizerSlice(0, {1.0, 0.0, 0.0, 0.0}, tok));

  for (int il = 0; il < 29; ++il) {
    const int mj = a3_mujoco_to_isaaclab[il];
    EXPECT_NEAR(tok[il], SyntheticMujocoRad(0, mj), 1e-5);
    EXPECT_NEAR(tok[29 + il], SyntheticMujocoRad(4, mj), 1e-5);
    EXPECT_NEAR(tok[58 + il], SyntheticMujocoRad(8, mj), 1e-5);
  }
}

TEST_F(A3CsvMotionReference, StopPolicyReturnsFalseBeyondEnd) {
  WriteCsv(4);

  a3_deploy::A3CsvMotionReference ref;
  a3_deploy::A3CsvMotionReferenceOptions opt;
  opt.source_fps = 50.0;
  opt.target_fps = 50.0;
  opt.future_frame_skip = 1;
  opt.on_end = a3_deploy::OnEndPolicy::kStop;
  ASSERT_TRUE(ref.Load(csv_path_.string(), opt));

  std::array<float, a3_deploy::kA3TokenizerFloatsPerTick> tok{};
  EXPECT_TRUE(ref.BuildTokenizerSlice(2, {1.0, 0.0, 0.0, 0.0}, tok));
  EXPECT_FALSE(ref.BuildTokenizerSlice(3, {1.0, 0.0, 0.0, 0.0}, tok));
}

TEST(A3CsvMotionReferenceRealData, LoadsRepoA3CsvWhenAvailable) {
  const char* env_path = std::getenv("A3_REAL_CSV_PATH");
  const std::string path =
      env_path ? std::string(env_path)
               : std::string("a3_data/agibot_a3/neutral_walk_ff_360_R_002__A535.csv");
  if (!std::filesystem::exists(path)) {
    GTEST_SKIP() << "A3 real CSV not found: " << path;
  }

  a3_deploy::A3CsvMotionReference ref;
  a3_deploy::A3CsvMotionReferenceOptions opt;
  opt.source_fps = 30.0;
  opt.target_fps = 50.0;
  opt.csv_frame_stride = 4;
  opt.future_frame_skip = 5;
  opt.on_end = a3_deploy::OnEndPolicy::kHoldLast;
  ASSERT_TRUE(ref.Load(path, opt));
  EXPECT_GT(ref.NumTicks(), 100u);
  EXPECT_TRUE(ref.Meta().has_initial_state);

  std::array<float, a3_deploy::kA3TokenizerFloatsPerTick> tok{};
  ASSERT_TRUE(ref.BuildTokenizerSlice(0, ref.Meta().init_root_quat_wxyz, tok));
  for (float v : tok) EXPECT_TRUE(std::isfinite(v));
}
