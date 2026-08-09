// Copyright (c) 2026, AgiBot Inc. All rights reserved.
#include <gtest/gtest.h>

#include "a3_deploy/a3_motion_library.hpp"
#include "robot_io/a3_layout_extra.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <array>
#include <string>
#include <unistd.h>

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
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

void WriteCsv(const std::filesystem::path& path, int rows) {
  std::ofstream f(path);
  ASSERT_TRUE(f.is_open());
  f << "Frame,root_translateX,root_translateY,root_translateZ,"
       "root_rotateX,root_rotateY,root_rotateZ";
  for (const char* name : kJointNames) f << "," << name;
  f << "\n";
  f << std::fixed << std::setprecision(9);

  for (int r = 0; r < rows; ++r) {
    f << r << "," << r << ",0,0,0,0,0";
    for (int sdk = 0; sdk < robot_io::kA3Dof; ++sdk) {
      double rad = 0.0;
      for (int mj = 0; mj < robot_io::kA3PolicyDof; ++mj) {
        if (robot_io::kA3PolicyToSdkIdx[mj] == sdk) {
          rad = static_cast<double>(r) + 0.01 * static_cast<double>(mj);
          break;
        }
      }
      f << "," << rad * kRadToDeg;
    }
    f << "\n";
  }
  ASSERT_TRUE(f.good());
}

class A3MotionLibraryTest : public ::testing::Test {
 protected:
  void SetUp() override {
    tmp_dir_ = std::filesystem::temp_directory_path() /
               ("a3_motion_library_test_" + std::to_string(::getpid()) +
                "_" +
                std::to_string(
                    ::testing::UnitTest::GetInstance()->random_seed()));
    std::filesystem::create_directories(tmp_dir_);
  }

  void TearDown() override {
    std::error_code ec;
    std::filesystem::remove_all(tmp_dir_, ec);
  }

  std::filesystem::path tmp_dir_;
};

}  // namespace

TEST_F(A3MotionLibraryTest, LoadsMotionDirInFilenameOrder) {
  WriteCsv(tmp_dir_ / "b_motion.csv", 4);
  WriteCsv(tmp_dir_ / "a_motion.csv", 4);

  a3_deploy::A3MotionLibraryOptions options;
  options.motion_dir = tmp_dir_.string();
  options.initial_index = 1;
  options.reference_options.source_fps = 50.0;
  options.reference_options.target_fps = 50.0;
  options.reference_options.future_frame_skip = 1;

  a3_deploy::A3MotionLibrary library;
  ASSERT_TRUE(library.Load(options));
  ASSERT_EQ(library.Size(), 2u);
  EXPECT_EQ(library.InitialIndex(), 1u);
  EXPECT_EQ(library.Clip(0).name, "a_motion");
  EXPECT_EQ(library.Clip(1).name, "b_motion");
}

TEST_F(A3MotionLibraryTest, FallsBackToSingleCsvPath) {
  const auto csv_path = tmp_dir_ / "single.csv";
  WriteCsv(csv_path, 4);

  a3_deploy::A3MotionLibraryOptions options;
  options.csv_path = csv_path.string();
  options.reference_options.source_fps = 50.0;
  options.reference_options.target_fps = 50.0;
  options.reference_options.future_frame_skip = 1;

  a3_deploy::A3MotionLibrary library;
  ASSERT_TRUE(library.Load(options));
  ASSERT_EQ(library.Size(), 1u);
  EXPECT_EQ(library.Clip(0).name, "single");
}

TEST_F(A3MotionLibraryTest, AppendsExtraMotionDirsAfterPrimaryDir) {
  const auto primary_dir = tmp_dir_ / "primary";
  const auto extra_dir = tmp_dir_ / "extra";
  std::filesystem::create_directories(primary_dir);
  std::filesystem::create_directories(extra_dir);
  WriteCsv(primary_dir / "base_b.csv", 4);
  WriteCsv(primary_dir / "base_a.csv", 4);
  WriteCsv(extra_dir / "remote_b.csv", 4);
  WriteCsv(extra_dir / "remote_a.csv", 4);

  a3_deploy::A3MotionLibraryOptions options;
  options.motion_dir = primary_dir.string();
  options.extra_motion_dirs.push_back(extra_dir.string());
  options.reference_options.source_fps = 50.0;
  options.reference_options.target_fps = 50.0;
  options.reference_options.future_frame_skip = 1;

  a3_deploy::A3MotionLibrary library;
  ASSERT_TRUE(library.Load(options));
  ASSERT_EQ(library.Size(), 4u);
  EXPECT_EQ(library.Clip(0).name, "base_a");
  EXPECT_EQ(library.Clip(1).name, "base_b");
  EXPECT_EQ(library.Clip(2).name, "remote_a");
  EXPECT_EQ(library.Clip(3).name, "remote_b");
}

TEST(A3MotionLibraryIndex, WrapsForwardAndBackward) {
  EXPECT_EQ(a3_deploy::WrapMotionIndex(0, -1, 3), 2u);
  EXPECT_EQ(a3_deploy::WrapMotionIndex(2, 1, 3), 0u);
  EXPECT_EQ(a3_deploy::WrapMotionIndex(1, 3, 3), 1u);
  EXPECT_EQ(a3_deploy::WrapMotionIndex(0, -4, 3), 2u);
  EXPECT_EQ(a3_deploy::WrapMotionIndex(9, 1, 0), 0u);
}
