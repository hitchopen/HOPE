// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// Thin A3 adapter for the A3 ZMQ packed SMPL stream.
#pragma once

#include "a3_deploy/a3_encoder_obs_builder.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

class MotionSequence;
class StreamedMotionMerger;
class ZMQPackedMessageSubscriber;

namespace a3_deploy {

enum class A3SmplJointOrder {
  kIsaacLab,
  kMujocoPolicy,
};

struct A3ZmqSmplSourceOptions {
  bool enabled = false;
  std::string host{"localhost"};
  int port = 5556;
  std::string topic{"pose"};
  bool conflate = true;
  bool verbose = false;
  A3SmplJointOrder joint_order = A3SmplJointOrder::kIsaacLab;
};

class A3ZmqSmplSource {
 public:
  A3ZmqSmplSource();
  ~A3ZmqSmplSource();

  bool Start(const A3ZmqSmplSourceOptions& options);
  void Stop();
  void ResetPlayback();
  void ResetPlaybackToLatestWindow();

  bool BuildTokenizerSlice(
      const std::array<double, 4>& robot_root_quat_wxyz,
      double reference_yaw_offset_rad,
      std::array<float, kA3SmplTokenizerTotalFloats>& out,
      bool advance_playback) noexcept;

  double ComputeLatestYawOffsetRad(
      const std::array<double, 4>& robot_root_quat_wxyz) const noexcept;
  bool HasAnyFrame() const noexcept;
  bool HasReadyWindow() const noexcept;
  bool Enabled() const noexcept { return options_.enabled; }
  std::int64_t LatestUpdateMonotonicNs() const noexcept;
  std::size_t FrameCount() const noexcept;
  std::string LastError() const;

 private:
  A3ZmqSmplSourceOptions options_{};
  std::unique_ptr<ZMQPackedMessageSubscriber> subscriber_;
  std::unique_ptr<StreamedMotionMerger> merger_;

  mutable std::mutex mu_;
  std::shared_ptr<MotionSequence> motion_;
  int playback_frame_ = 0;
  std::int64_t latest_update_monotonic_ns_ = 0;
  std::string last_error_;
};

A3SmplJointOrder ParseA3SmplJointOrder(const std::string& value);

}  // namespace a3_deploy
