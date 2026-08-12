// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// A3CsvMotionReference: runtime reference-motion provider for A3 deployment.
//
// This reads the same flat A3 CSV format consumed by
// convert_soma_csv_to_motion_lib.py and generates the 640-float tokenizer
// prefix online, using the current robot root orientation for
// motion_anchor_ori_b_mf_nonflat.
#pragma once

#include "a3_deploy/a3_tokenizer_replay.hpp"

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace a3_deploy {

struct A3CsvMotionReferenceOptions {
  double source_fps = 30.0;
  double target_fps = 50.0;
  int csv_frame_stride = 1;
  int future_frame_skip = 5;
  OnEndPolicy on_end = OnEndPolicy::kHoldLast;
};

class A3CsvMotionReference {
 public:
  A3CsvMotionReference() = default;

  bool Load(const std::string& csv_path,
            const A3CsvMotionReferenceOptions& options);

  // Builds:
  //   [0..579]   command_multi_future_nonflat
  //              q_il[10,29] flattened, then dq_il[10,29] flattened
  //   [580..639] motion_anchor_ori_b_mf_nonflat
  //              relative root orientation 6D for each future frame
  //
  // Returns false only when no data is loaded or on_end==kStop and tick_idx is
  // beyond the clip. Future frames are clamped/wrapped according to on_end.
  bool BuildTokenizerSlice(
      std::size_t tick_idx,
      const std::array<double, 4>& robot_root_quat_wxyz,
      std::array<float, kA3TokenizerFloatsPerTick>& out) const noexcept;

  // Same as above, but rotates the world-frame reference root orientation by
  // `reference_yaw_offset_rad` before computing the robot-relative anchor:
  //   rel = inv(robot_root_quat) * (yaw_offset * ref_root_quat)
  //
  // This is used by real deployment to lock the clip's first-frame yaw to the
  // robot's current yaw when entering POLICY. The offset is expected to remain
  // fixed for the whole policy clip.
  bool BuildTokenizerSlice(
      std::size_t tick_idx,
      const std::array<double, 4>& robot_root_quat_wxyz,
      double reference_yaw_offset_rad,
      std::array<float, kA3TokenizerFloatsPerTick>& out) const noexcept;

  // Builds a stable "paused" tokenizer slice by repeating the resolved frame
  // for all 10 future joint-position slots and writing zero future joint
  // velocities. The anchor orientation also repeats the resolved frame. This
  // keeps POLICY mode on the normal ONNX path while holding a motion frame
  // during pause, motion switching, and terminal hold.
  bool BuildHeldTokenizerSlice(
      std::size_t tick_idx,
      const std::array<double, 4>& robot_root_quat_wxyz,
      double reference_yaw_offset_rad,
      std::array<float, kA3TokenizerFloatsPerTick>& out) const noexcept;

  // Computes yaw(robot_root_quat) - yaw(reference frame 0), wrapped to [-pi,pi].
  // Returns 0 when no data is loaded.
  double ComputeYawOffsetRad(
      const std::array<double, 4>& robot_root_quat_wxyz) const noexcept;

  std::size_t NumTicks() const noexcept { return meta_.num_ticks; }
  double TargetFps() const noexcept { return options_.target_fps; }
  const A3TokenizerReplayMeta& Meta() const noexcept { return meta_; }

 private:
  struct Frame {
    std::array<double, 3> root_pos{};
    std::array<double, 4> root_quat_wxyz{1.0, 0.0, 0.0, 0.0};
    std::array<float, 29> dof_mujoco{};
  };

  bool ResolveTick(std::size_t tick_idx, std::size_t* out_idx) const noexcept;
  std::size_t ResolveFutureTick(std::size_t base_idx,
                                std::size_t future_offset) const noexcept;

  std::vector<Frame> frames_;
  std::vector<std::array<float, 29>> dof_isaaclab_;
  std::vector<std::array<float, 29>> dof_vel_isaaclab_;
  A3TokenizerReplayMeta meta_{};
  A3CsvMotionReferenceOptions options_{};
};

}  // namespace a3_deploy
