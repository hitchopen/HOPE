// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// Runtime teleop reference provider for A3 deploy.
//
// The current A3 ONNX export consumes obs_dict[1570], whose first 640 floats
// are the same tokenizer/reference prefix produced by A3CsvMotionReference:
//   q_ref_isaaclab[10,29], dq_ref_isaaclab[10,29], rel_root_ori_6d[10].
// This component turns /ta/whole_body_command frames into that 640-float prefix
// with a configurable delay window, and provides a default standing prefix for
// TELEOP startup before any upstream data has arrived.
#pragma once

#include "a3_deploy/a3_tokenizer_replay.hpp"

#include <array>
#include <cstdint>
#include <cstddef>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace aimdk::protocol {
class TaWholeBodyCommandChannel;
}

namespace a3_deploy {

enum class A3TeleopTokenizerStatus {
  kNoData,
  kBuffering,
  kRunning,
};

struct A3TeleopFrame {
  std::int64_t stamp_ns = 0;
  std::array<double, 29> q_mujoco{};
  std::array<double, 29> dq_mujoco{};
  std::array<double, 4> pelvis_quat_wxyz{1.0, 0.0, 0.0, 0.0};
};

struct A3TeleopReferenceOptions {
  std::int64_t delay_ns = 900'000'000;  // 0.9s
  double policy_hz = 50.0;
  int future_frame_skip = 5;
  std::size_t max_frames = 512;
};

// Builds the fallback standing tokenizer prefix used while TELEOP has no
// usable whole-body command window. This still feeds ONNX; it does not bypass
// the policy with a direct PD command.
void BuildDefaultStandTokenizerSlice(
    const std::array<double, 4>& robot_root_quat_wxyz,
    std::array<float, kA3TokenizerFloatsPerTick>& out) noexcept;

// Safety envelope for TELEOP startup/no-data fallback. ONNX is still evaluated
// with the default standing tokenizer, but the decoded command is blended back
// toward the nominal standing pose before it is sent to the robot.
void ApplyStandFallbackCommandFilter(
    double policy_blend,
    double max_delta_rad,
    std::array<double, 29>& q_des_mujoco) noexcept;

class A3TeleopReferenceBuffer {
 public:
  explicit A3TeleopReferenceBuffer(A3TeleopReferenceOptions options = {});

  void Configure(const A3TeleopReferenceOptions& options);
  void Reset();
  bool ResetToLatestFrame();
  void PushFrame(const A3TeleopFrame& frame);

  bool BuildTokenizerSlice(
      std::int64_t now_ns,
      const std::array<double, 4>& robot_root_quat_wxyz,
      double reference_yaw_offset_rad,
      std::array<float, kA3TokenizerFloatsPerTick>& out,
      A3TeleopTokenizerStatus* status = nullptr) noexcept;

  double ComputeYawOffsetRad(
      std::int64_t now_ns,
      const std::array<double, 4>& robot_root_quat_wxyz) const noexcept;
  double ComputeLatestYawOffsetRad(
      const std::array<double, 4>& robot_root_quat_wxyz) const noexcept;

  bool HasAnyFrame() const noexcept;
  bool HasReadyWindow(std::int64_t now_ns) const noexcept;
  std::size_t FrameCount() const noexcept;
  std::int64_t LatestStampNs() const noexcept;
  std::int64_t DelayNs() const noexcept;

 private:
  bool SampleAtLocked(std::int64_t stamp_ns, A3TeleopFrame& out) const noexcept;
  std::int64_t FutureStepNs() const noexcept;

  mutable std::mutex mu_;
  A3TeleopReferenceOptions options_{};
  std::vector<A3TeleopFrame> frames_;
  bool running_started_ = false;
};

#ifdef HAS_A3_TA_PROTO
bool ConvertTaWholeBodyCommand(
    const aimdk::protocol::TaWholeBodyCommandChannel& msg,
    std::int64_t fallback_stamp_ns,
    A3TeleopFrame& out,
    std::string* error = nullptr);
#endif

}  // namespace a3_deploy
