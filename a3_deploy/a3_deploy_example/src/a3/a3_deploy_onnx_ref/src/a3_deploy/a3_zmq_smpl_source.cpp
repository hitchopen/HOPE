// Copyright (c) 2026, AgiBot Inc. All rights reserved.

#include "a3_deploy/a3_zmq_smpl_source.hpp"

#include "a3_deploy/numeric_safety.hpp"
#include "a3_deploy/a3_yaw_alignment.hpp"
#include "a3_policy_parameters.hpp"
#include "input_interface/packed_motion_decoder.hpp"
#include "input_interface/streamed_motion_merger.hpp"
#include "input_interface/zmq_packed_message_subscriber.hpp"
#include "motion_data_reader.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <ctime>
#include <iostream>
#include <limits>

namespace a3_deploy {
namespace {

constexpr int kSmplFutureFrames = 10;
constexpr int kSmplJointCount = 24;
constexpr int kA3JointCount = 29;

std::int64_t NowMonotonicNsNoThrow() noexcept {
  timespec ts{};
  if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return 0;
  return static_cast<std::int64_t>(ts.tv_sec) * 1'000'000'000LL +
         static_cast<std::int64_t>(ts.tv_nsec);
}

std::array<double, 4> NormalizeQuat(std::array<double, 4> q) noexcept {
  double n2 = 0.0;
  for (double v : q) n2 += v * v;
  if (n2 <= std::numeric_limits<double>::epsilon()) {
    return {1.0, 0.0, 0.0, 0.0};
  }
  const double inv = 1.0 / std::sqrt(n2);
  for (double& v : q) v *= inv;
  return q;
}

std::array<double, 4> QuatConj(const std::array<double, 4>& q) noexcept {
  return {q[0], -q[1], -q[2], -q[3]};
}

std::array<double, 4> QuatMul(const std::array<double, 4>& a,
                              const std::array<double, 4>& b) noexcept {
  const double w1 = a[0], x1 = a[1], y1 = a[2], z1 = a[3];
  const double w2 = b[0], x2 = b[1], y2 = b[2], z2 = b[3];
  return {
      w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
      w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
      w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
      w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
  };
}

void QuatToMatrix(const std::array<double, 4>& q_in,
                  double m[3][3]) noexcept {
  const auto q = NormalizeQuat(q_in);
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  m[0][0] = 1.0 - 2.0 * (y * y + z * z);
  m[0][1] = 2.0 * (x * y - z * w);
  m[0][2] = 2.0 * (x * z + y * w);
  m[1][0] = 2.0 * (x * y + z * w);
  m[1][1] = 1.0 - 2.0 * (x * x + z * z);
  m[1][2] = 2.0 * (y * z - x * w);
  m[2][0] = 2.0 * (x * z - y * w);
  m[2][1] = 2.0 * (y * z + x * w);
  m[2][2] = 1.0 - 2.0 * (x * x + y * y);
}

void WriteOrientation6d(const std::array<double, 4>& rel_quat,
                        float* dst) noexcept {
  double m[3][3]{};
  QuatToMatrix(rel_quat, m);
  dst[0] = static_cast<float>(m[0][0]);
  dst[1] = static_cast<float>(m[0][1]);
  dst[2] = static_cast<float>(m[1][0]);
  dst[3] = static_cast<float>(m[1][1]);
  dst[4] = static_cast<float>(m[2][0]);
  dst[5] = static_cast<float>(m[2][1]);
}

bool IsFiniteArray(const std::array<double, 4>& v) noexcept {
  for (double x : v) {
    if (!numeric_safety::IsFinite(x)) return false;
  }
  return true;
}

}  // namespace

A3ZmqSmplSource::A3ZmqSmplSource()
    : merger_(std::make_unique<StreamedMotionMerger>()) {}

A3ZmqSmplSource::~A3ZmqSmplSource() { Stop(); }

bool A3ZmqSmplSource::Start(const A3ZmqSmplSourceOptions& options) {
  Stop();
  options_ = options;
  if (!options_.enabled) return true;

  merger_ = std::make_unique<StreamedMotionMerger>();
  subscriber_ = std::make_unique<ZMQPackedMessageSubscriber>(
      options_.host, options_.port, options_.topic,
      /*timeout_ms=*/100, options_.verbose, options_.conflate,
      /*rcv_hwm=*/options_.conflate ? 1 : 3);

  subscriber_->SetOnDecodedMessage(
      [this](const std::string& topic,
             const ZMQPackedMessageSubscriber::DecodedHeader& header,
             const std::vector<ZMQPackedMessageSubscriber::BufferView>& buffers) {
        input_interface::PackedMotionDecodeResult decoded;
        if (!input_interface::DecodePackedMotionMessage(header, buffers,
                                                        decoded)) {
          std::lock_guard<std::mutex> lock(mu_);
          last_error_ = decoded.error;
          return;
        }

        auto& data = decoded.data;
        if (data.num_smpl_joints < kSmplJointCount) {
          std::lock_guard<std::mutex> lock(mu_);
          last_error_ = "SMPL ZMQ message has fewer than 24 joints";
          return;
        }
        if (data.num_quat_bodies < 1) {
          std::lock_guard<std::mutex> lock(mu_);
          last_error_ = "SMPL ZMQ message has no body_quat root";
          return;
        }

        std::lock_guard<std::mutex> lock(mu_);
        const int playback = std::max(0, playback_frame_);
        const auto merge = merger_->MergeIncomingData(data, playback);
        if (!merge.motion) {
          last_error_ = "StreamedMotionMerger rejected " + topic;
          return;
        }
        motion_ = merge.motion;
        if (merge.did_catchup_reset) {
          playback_frame_ = 0;
        } else {
          playback_frame_ =
              std::max(0, playback_frame_ - merge.frame_offset_adjustment);
        }
        latest_update_monotonic_ns_ = NowMonotonicNsNoThrow();
        last_error_.clear();
      });

  subscriber_->Start();
  std::cout << "✓ SMPL ZMQ source enabled: " << options_.host << ":"
            << options_.port << " topic=" << options_.topic
            << " joint_order="
            << (options_.joint_order == A3SmplJointOrder::kIsaacLab
                    ? "isaaclab"
                    : "mujoco")
            << "\n";
  return true;
}

void A3ZmqSmplSource::Stop() {
  if (subscriber_) {
    subscriber_->Stop();
    subscriber_.reset();
  }
}

void A3ZmqSmplSource::ResetPlayback() {
  std::lock_guard<std::mutex> lock(mu_);
  playback_frame_ = 0;
}

void A3ZmqSmplSource::ResetPlaybackToLatestWindow() {
  std::lock_guard<std::mutex> lock(mu_);
  if (!motion_ || motion_->timesteps <= 0) {
    playback_frame_ = 0;
    return;
  }
  playback_frame_ = std::max(0, motion_->timesteps - kSmplFutureFrames);
}

bool A3ZmqSmplSource::BuildTokenizerSlice(
    const std::array<double, 4>& robot_root_quat_wxyz,
    double reference_yaw_offset_rad,
    std::array<float, kA3SmplTokenizerTotalFloats>& out,
    bool advance_playback) noexcept {
  out.fill(0.0f);
  std::lock_guard<std::mutex> lock(mu_);
  if (!motion_ || motion_->timesteps <= 0) return false;
  if (motion_->GetNumSmplJoints() < kSmplJointCount ||
      motion_->GetNumBodyQuaternions() < 1 ||
      motion_->GetNumJoints() < kA3JointCount) {
    return false;
  }

  const int base = std::clamp(playback_frame_, 0, motion_->timesteps - 1);
  const auto inv_robot =
      yaw_alignment::QuatConj(yaw_alignment::NormalizeQuat(robot_root_quat_wxyz));
  std::size_t cursor = 0;

  for (int f = 0; f < kSmplFutureFrames; ++f) {
    const int idx = std::min(base + f, motion_->timesteps - 1);
    const auto* joints = motion_->SmplJoints(idx);
    for (int j = 0; j < kSmplJointCount; ++j) {
      out[cursor++] = static_cast<float>(joints[j][0]);
      out[cursor++] = static_cast<float>(joints[j][1]);
      out[cursor++] = static_cast<float>(joints[j][2]);
    }
  }

  for (int f = 0; f < kSmplFutureFrames; ++f) {
    const int idx = std::min(base + f, motion_->timesteps - 1);
    const auto* quats = motion_->BodyQuaternions(idx);
    std::array<double, 4> root = quats[0];
    if (!IsFiniteArray(root)) root = {1.0, 0.0, 0.0, 0.0};
    const auto yaw_aligned_root =
        yaw_alignment::ApplyYawOffset(reference_yaw_offset_rad, root);
    const auto rel = yaw_alignment::QuatMul(inv_robot, yaw_aligned_root);
    WriteOrientation6d(rel, out.data() + cursor);
    cursor += 6;
  }

  for (int f = 0; f < kSmplFutureFrames; ++f) {
    const int idx = std::min(base + f, motion_->timesteps - 1);
    const double* q = motion_->JointPositions(idx);
    for (int il = 23; il <= 28; ++il) {
      const int src =
          (options_.joint_order == A3SmplJointOrder::kMujocoPolicy)
              ? a3_mujoco_to_isaaclab[static_cast<std::size_t>(il)]
              : il;
      out[cursor++] = static_cast<float>(q[src]);
    }
  }

  if (advance_playback && playback_frame_ + 1 < motion_->timesteps) {
    ++playback_frame_;
  }
  return true;
}

double A3ZmqSmplSource::ComputeLatestYawOffsetRad(
    const std::array<double, 4>& robot_root_quat_wxyz) const noexcept {
  std::lock_guard<std::mutex> lock(mu_);
  if (!motion_ || motion_->timesteps <= 0 ||
      motion_->GetNumBodyQuaternions() < 1) {
    return 0.0;
  }
  std::array<double, 4> root =
      motion_->BodyQuaternions(motion_->timesteps - 1)[0];
  if (!IsFiniteArray(root)) root = {1.0, 0.0, 0.0, 0.0};
  return yaw_alignment::ComputeYawOffsetRad(robot_root_quat_wxyz, root);
}

bool A3ZmqSmplSource::HasAnyFrame() const noexcept {
  std::lock_guard<std::mutex> lock(mu_);
  return motion_ && motion_->timesteps > 0 &&
         motion_->GetNumBodyQuaternions() >= 1;
}

bool A3ZmqSmplSource::HasReadyWindow() const noexcept {
  std::lock_guard<std::mutex> lock(mu_);
  return motion_ && motion_->timesteps >= kSmplFutureFrames &&
         motion_->GetNumSmplJoints() >= kSmplJointCount &&
         motion_->GetNumBodyQuaternions() >= 1 &&
         motion_->GetNumJoints() >= kA3JointCount;
}

std::int64_t A3ZmqSmplSource::LatestUpdateMonotonicNs() const noexcept {
  std::lock_guard<std::mutex> lock(mu_);
  return latest_update_monotonic_ns_;
}

std::size_t A3ZmqSmplSource::FrameCount() const noexcept {
  std::lock_guard<std::mutex> lock(mu_);
  return motion_ ? static_cast<std::size_t>(std::max(0, motion_->timesteps))
                 : 0ULL;
}

std::string A3ZmqSmplSource::LastError() const {
  std::lock_guard<std::mutex> lock(mu_);
  return last_error_;
}

A3SmplJointOrder ParseA3SmplJointOrder(const std::string& value) {
  std::string v = value;
  std::transform(v.begin(), v.end(), v.begin(),
                 [](unsigned char c) {
                   return static_cast<char>(std::tolower(c));
                 });
  if (v == "mujoco" || v == "mujoco_policy" || v == "policy") {
    return A3SmplJointOrder::kMujocoPolicy;
  }
  return A3SmplJointOrder::kIsaacLab;
}

}  // namespace a3_deploy
