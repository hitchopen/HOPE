// Copyright (c) 2026, AgiBot Inc. All rights reserved.
#include "a3_deploy/a3_teleop_reference.hpp"

#include "a3_deploy/numeric_safety.hpp"
#include "a3_deploy/a3_obs_builder.hpp"
#include "a3_deploy/a3_yaw_alignment.hpp"
#include "a3_policy_parameters.hpp"

#ifdef HAS_A3_TA_PROTO
#include "aimdk/protocol/ta/ta_channel.pb.h"
#endif

#include <algorithm>
#include <cmath>
#include <limits>

namespace a3_deploy {

namespace {

constexpr std::size_t kFutureFrames = 10;
constexpr std::size_t kPolicyDof = 29;
constexpr std::size_t kCommandDofFloats = kFutureFrames * kPolicyDof;  // 290

std::array<double, 4> NormalizeQuat(std::array<double, 4> q) noexcept {
  double n2 = 0.0;
  for (double v : q) n2 += v * v;
  if (n2 <= std::numeric_limits<double>::epsilon()) return {1.0, 0.0, 0.0, 0.0};
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

double QuatDot(const std::array<double, 4>& a,
               const std::array<double, 4>& b) noexcept {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
}

std::array<double, 4> QuatSlerp(const std::array<double, 4>& qa,
                                const std::array<double, 4>& qb,
                                double alpha) noexcept {
  std::array<double, 4> a = NormalizeQuat(qa);
  std::array<double, 4> b = NormalizeQuat(qb);
  double dot = QuatDot(a, b);
  if (dot < 0.0) {
    dot = -dot;
    for (double& v : b) v = -v;
  }

  if (dot > 0.9995) {
    std::array<double, 4> out{};
    for (std::size_t i = 0; i < 4; ++i) {
      out[i] = (1.0 - alpha) * a[i] + alpha * b[i];
    }
    return NormalizeQuat(out);
  }

  dot = std::clamp(dot, -1.0, 1.0);
  const double theta0 = std::acos(dot);
  const double theta = theta0 * alpha;
  const double sin_theta = std::sin(theta);
  const double sin_theta0 = std::sin(theta0);
  const double s0 = std::cos(theta) - dot * sin_theta / sin_theta0;
  const double s1 = sin_theta / sin_theta0;
  std::array<double, 4> out{};
  for (std::size_t i = 0; i < 4; ++i) out[i] = s0 * a[i] + s1 * b[i];
  return NormalizeQuat(out);
}

void QuatToMatrix(const std::array<double, 4>& q_in, double m[3][3]) noexcept {
  const auto q = NormalizeQuat(q_in);
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  const double xx = x * x, yy = y * y, zz = z * z;
  const double xy = x * y, xz = x * z, yz = y * z;
  const double wx = w * x, wy = w * y, wz = w * z;
  m[0][0] = 1.0 - 2.0 * (yy + zz);
  m[0][1] = 2.0 * (xy - wz);
  m[0][2] = 2.0 * (xz + wy);
  m[1][0] = 2.0 * (xy + wz);
  m[1][1] = 1.0 - 2.0 * (xx + zz);
  m[1][2] = 2.0 * (yz - wx);
  m[2][0] = 2.0 * (xz - wy);
  m[2][1] = 2.0 * (yz + wx);
  m[2][2] = 1.0 - 2.0 * (xx + yy);
}

void WriteOrientation6d(const std::array<double, 4>& rel_quat,
                        std::size_t frame,
                        std::array<float, kA3TokenizerFloatsPerTick>& out) noexcept {
  double m[3][3];
  QuatToMatrix(rel_quat, m);
  const std::size_t o = kA3CommandMultiFutureFloats + frame * 6;
  out[o + 0] = static_cast<float>(m[0][0]);
  out[o + 1] = static_cast<float>(m[0][1]);
  out[o + 2] = static_cast<float>(m[1][0]);
  out[o + 3] = static_cast<float>(m[1][1]);
  out[o + 4] = static_cast<float>(m[2][0]);
  out[o + 5] = static_cast<float>(m[2][1]);
}

bool IsFiniteArray(const std::array<double, 29>& values) noexcept {
  for (double v : values) {
    if (!numeric_safety::IsFinite(v)) return false;
  }
  return true;
}

bool IsFiniteQuat(const std::array<double, 4>& q) noexcept {
  for (double v : q) {
    if (!numeric_safety::IsFinite(v)) return false;
  }
  return true;
}

}  // namespace

void BuildDefaultStandTokenizerSlice(
    const std::array<double, 4>& /*robot_root_quat_wxyz*/,
    std::array<float, kA3TokenizerFloatsPerTick>& out) noexcept {
  out.fill(0.0f);

  for (std::size_t k = 0; k < kFutureFrames; ++k) {
    for (std::size_t i_il = 0; i_il < kPolicyDof; ++i_il) {
      out[k * kPolicyDof + i_il] =
          static_cast<float>(a3_default_angles[a3_mujoco_to_isaaclab[i_il]]);
      out[kCommandDofFloats + k * kPolicyDof + i_il] = 0.0f;
    }
    WriteOrientation6d({1.0, 0.0, 0.0, 0.0}, k, out);
  }
}

void ApplyStandFallbackCommandFilter(
    double policy_blend,
    double max_delta_rad,
    std::array<double, 29>& q_des_mujoco) noexcept {
  const double blend = std::clamp(policy_blend, 0.0, 1.0);
  const double limit =
      numeric_safety::IsFinite(max_delta_rad) && max_delta_rad >= 0.0
          ? max_delta_rad
          : std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < kPolicyDof; ++i) {
    double delta = (q_des_mujoco[i] - a3_default_angles[i]) * blend;
    delta = std::clamp(delta, -limit, limit);
    q_des_mujoco[i] = a3_default_angles[i] + delta;
  }
}

A3TeleopReferenceBuffer::A3TeleopReferenceBuffer(
    A3TeleopReferenceOptions options)
    : options_(options) {
  if (options_.max_frames == 0) options_.max_frames = 1;
  frames_.reserve(options_.max_frames);
}

void A3TeleopReferenceBuffer::Configure(
    const A3TeleopReferenceOptions& options) {
  std::lock_guard<std::mutex> lk(mu_);
  options_ = options;
  if (options_.max_frames == 0) options_.max_frames = 1;
  frames_.clear();
  frames_.reserve(options_.max_frames);
  running_started_ = false;
}

void A3TeleopReferenceBuffer::Reset() {
  std::lock_guard<std::mutex> lk(mu_);
  frames_.clear();
  running_started_ = false;
}

bool A3TeleopReferenceBuffer::ResetToLatestFrame() {
  std::lock_guard<std::mutex> lk(mu_);
  if (frames_.empty()) return false;
  const A3TeleopFrame latest = frames_.back();
  frames_.clear();
  frames_.push_back(latest);
  running_started_ = false;
  return true;
}

void A3TeleopReferenceBuffer::PushFrame(const A3TeleopFrame& frame) {
  if (frame.stamp_ns <= 0 || !IsFiniteArray(frame.q_mujoco) ||
      !IsFiniteArray(frame.dq_mujoco) || !IsFiniteQuat(frame.pelvis_quat_wxyz)) {
    return;
  }

  A3TeleopFrame normalized = frame;
  normalized.pelvis_quat_wxyz = NormalizeQuat(normalized.pelvis_quat_wxyz);

  std::lock_guard<std::mutex> lk(mu_);
  if (!frames_.empty() && normalized.stamp_ns < frames_.back().stamp_ns) {
    auto it = std::lower_bound(
        frames_.begin(), frames_.end(), normalized.stamp_ns,
        [](const A3TeleopFrame& f, std::int64_t stamp) {
          return f.stamp_ns < stamp;
        });
    if (it != frames_.end() && it->stamp_ns == normalized.stamp_ns) {
      *it = normalized;
    } else {
      frames_.insert(it, normalized);
    }
  } else if (!frames_.empty() && normalized.stamp_ns == frames_.back().stamp_ns) {
    frames_.back() = normalized;
  } else {
    frames_.push_back(normalized);
  }

  while (frames_.size() > options_.max_frames) {
    frames_.erase(frames_.begin());
  }
}

bool A3TeleopReferenceBuffer::BuildTokenizerSlice(
    std::int64_t now_ns,
    const std::array<double, 4>& robot_root_quat_wxyz,
    double reference_yaw_offset_rad,
    std::array<float, kA3TokenizerFloatsPerTick>& out,
    A3TeleopTokenizerStatus* status) noexcept {
  out.fill(0.0f);
  if (status) *status = A3TeleopTokenizerStatus::kNoData;

  std::lock_guard<std::mutex> lk(mu_);
  if (frames_.empty()) return false;
  if (frames_.size() < 2) {
    if (status) *status = A3TeleopTokenizerStatus::kBuffering;
    return false;
  }

  const std::int64_t base_ns = now_ns - options_.delay_ns;
  if (base_ns < frames_.front().stamp_ns) {
    if (status) *status = A3TeleopTokenizerStatus::kBuffering;
    return false;
  }
  if (base_ns > frames_.back().stamp_ns && !running_started_) {
    if (status) *status = A3TeleopTokenizerStatus::kBuffering;
    return false;
  }

  const auto inv_robot_quat =
      yaw_alignment::QuatConj(yaw_alignment::NormalizeQuat(robot_root_quat_wxyz));
  const std::int64_t future_step_ns = FutureStepNs();
  // Future slots beyond the freshest received command are intentionally
  // clamped by SampleAtLocked(). With delay_ms=900 and 10 frames at 100 ms
  // spacing, the last slot nominally lands on "now"; requiring a teleop
  // callback timestamp at or after the current policy tick makes startup race
  // asynchronous subscriber delivery forever.

  for (std::size_t k = 0; k < kFutureFrames; ++k) {
    A3TeleopFrame sample{};
    const std::int64_t sample_ns =
        base_ns + static_cast<std::int64_t>(k) * future_step_ns;
    if (!SampleAtLocked(sample_ns, sample)) {
      if (status) *status = A3TeleopTokenizerStatus::kBuffering;
      return false;
    }

    for (std::size_t i_il = 0; i_il < kPolicyDof; ++i_il) {
      const std::size_t i_mj = a3_mujoco_to_isaaclab[i_il];
      out[k * kPolicyDof + i_il] =
          static_cast<float>(sample.q_mujoco[i_mj]);
      out[kCommandDofFloats + k * kPolicyDof + i_il] =
          static_cast<float>(sample.dq_mujoco[i_mj]);
    }

    const auto yaw_aligned_ref = yaw_alignment::ApplyYawOffset(
        reference_yaw_offset_rad, sample.pelvis_quat_wxyz);
    const auto rel = yaw_alignment::QuatMul(inv_robot_quat, yaw_aligned_ref);
    WriteOrientation6d(rel, k, out);
  }

  running_started_ = true;
  if (status) *status = A3TeleopTokenizerStatus::kRunning;
  return true;
}

double A3TeleopReferenceBuffer::ComputeYawOffsetRad(
    std::int64_t now_ns,
    const std::array<double, 4>& robot_root_quat_wxyz) const noexcept {
  std::lock_guard<std::mutex> lk(mu_);
  if (frames_.empty()) return 0.0;

  A3TeleopFrame sample{};
  const std::int64_t base_ns = now_ns - options_.delay_ns;
  if (!SampleAtLocked(base_ns, sample)) {
    sample = frames_.front();
  }
  return yaw_alignment::ComputeYawOffsetRad(robot_root_quat_wxyz,
                                            sample.pelvis_quat_wxyz);
}

double A3TeleopReferenceBuffer::ComputeLatestYawOffsetRad(
    const std::array<double, 4>& robot_root_quat_wxyz) const noexcept {
  std::lock_guard<std::mutex> lk(mu_);
  if (frames_.empty()) return 0.0;
  return yaw_alignment::ComputeYawOffsetRad(robot_root_quat_wxyz,
                                            frames_.back().pelvis_quat_wxyz);
}

bool A3TeleopReferenceBuffer::HasAnyFrame() const noexcept {
  std::lock_guard<std::mutex> lk(mu_);
  return !frames_.empty();
}

bool A3TeleopReferenceBuffer::HasReadyWindow(std::int64_t now_ns) const noexcept {
  std::lock_guard<std::mutex> lk(mu_);
  if (frames_.size() < 2) return false;
  const std::int64_t base_ns = now_ns - options_.delay_ns;
  if (base_ns < frames_.front().stamp_ns) return false;
  return base_ns <= frames_.back().stamp_ns;
}

std::size_t A3TeleopReferenceBuffer::FrameCount() const noexcept {
  std::lock_guard<std::mutex> lk(mu_);
  return frames_.size();
}

std::int64_t A3TeleopReferenceBuffer::LatestStampNs() const noexcept {
  std::lock_guard<std::mutex> lk(mu_);
  return frames_.empty() ? 0 : frames_.back().stamp_ns;
}

std::int64_t A3TeleopReferenceBuffer::DelayNs() const noexcept {
  std::lock_guard<std::mutex> lk(mu_);
  return options_.delay_ns;
}

bool A3TeleopReferenceBuffer::SampleAtLocked(
    std::int64_t stamp_ns,
    A3TeleopFrame& out) const noexcept {
  if (frames_.empty()) return false;
  if (stamp_ns <= frames_.front().stamp_ns) {
    out = frames_.front();
    return true;
  }
  if (stamp_ns >= frames_.back().stamp_ns) {
    out = frames_.back();
    return true;
  }

  auto hi = std::lower_bound(
      frames_.begin(), frames_.end(), stamp_ns,
      [](const A3TeleopFrame& f, std::int64_t stamp) {
        return f.stamp_ns < stamp;
      });
  if (hi == frames_.end()) {
    out = frames_.back();
    return true;
  }
  if (hi == frames_.begin()) {
    out = frames_.front();
    return true;
  }
  if (hi->stamp_ns == stamp_ns) {
    out = *hi;
    return true;
  }

  const auto lo = hi - 1;
  const double denom = static_cast<double>(hi->stamp_ns - lo->stamp_ns);
  const double alpha =
      denom > 0.0 ? static_cast<double>(stamp_ns - lo->stamp_ns) / denom : 0.0;
  out.stamp_ns = stamp_ns;
  for (std::size_t i = 0; i < kPolicyDof; ++i) {
    out.q_mujoco[i] =
        (1.0 - alpha) * lo->q_mujoco[i] + alpha * hi->q_mujoco[i];
    out.dq_mujoco[i] =
        (1.0 - alpha) * lo->dq_mujoco[i] + alpha * hi->dq_mujoco[i];
  }
  out.pelvis_quat_wxyz =
      QuatSlerp(lo->pelvis_quat_wxyz, hi->pelvis_quat_wxyz, alpha);
  return true;
}

std::int64_t A3TeleopReferenceBuffer::FutureStepNs() const noexcept {
  const double hz = options_.policy_hz > 0.0 ? options_.policy_hz : 50.0;
  const int skip = options_.future_frame_skip > 0 ? options_.future_frame_skip : 5;
  return static_cast<std::int64_t>(
      std::llround(1'000'000'000.0 * static_cast<double>(skip) / hz));
}

#ifdef HAS_A3_TA_PROTO
bool ConvertTaWholeBodyCommand(
    const aimdk::protocol::TaWholeBodyCommandChannel& msg,
    std::int64_t fallback_stamp_ns,
    A3TeleopFrame& out,
    std::string* error) {
  const auto& data = msg.data();
  const auto& pelvis_quat = data.pelvis_pose().quat_wxyz();
  const auto& leg = data.leg_command().angles_rad();
  const auto& waist = data.waist_command().angles_rad();
  const auto& arm = data.arm_command().angles_rad();
  const auto& vel = data.joint_velocities().velocities_rad_s();

  auto fail = [error](const char* msg_text) {
    if (error) *error = msg_text;
    return false;
  };

  if (pelvis_quat.size() != 4) return fail("pelvis_pose.quat_wxyz must have size 4");
  if (leg.size() != 12) return fail("leg_command.angles_rad must have size 12");
  if (waist.size() != 3) return fail("waist_command.angles_rad must have size 3");
  if (arm.size() != 14) return fail("arm_command.angles_rad must have size 14");
  const auto& ts = msg.header().timestamp();
  std::int64_t stamp_ns = fallback_stamp_ns;
  if (ts.seconds() != 0 || ts.nanos() != 0) {
    stamp_ns = ts.seconds() * 1'000'000'000LL + ts.nanos();
  } else if (ts.ms_since_epoch() != 0) {
    stamp_ns = ts.ms_since_epoch() * 1'000'000LL;
  }
  if (stamp_ns <= 0) return fail("timestamp must be positive");

  A3TeleopFrame frame{};
  frame.stamp_ns = stamp_ns;
  frame.pelvis_quat_wxyz = {
      pelvis_quat.Get(0), pelvis_quat.Get(1),
      pelvis_quat.Get(2), pelvis_quat.Get(3)};

  // Policy-view order: waist[0..2], left arm[3..9], right arm[10..16],
  // left leg[17..22], right leg[23..28].
  for (int i = 0; i < 3; ++i) frame.q_mujoco[i] = waist.Get(i);
  for (int i = 0; i < 14; ++i) frame.q_mujoco[3 + i] = arm.Get(i);
  for (int i = 0; i < 12; ++i) frame.q_mujoco[17 + i] = leg.Get(i);

  frame.dq_mujoco.fill(0.0);
  if (vel.size() == 30) {
    // Protocol layout: leg(12) + waist(3) + head(1) + arm(14).
    for (int i = 0; i < 3; ++i) frame.dq_mujoco[i] = vel.Get(12 + i);
    for (int i = 0; i < 14; ++i) frame.dq_mujoco[3 + i] = vel.Get(16 + i);
    for (int i = 0; i < 12; ++i) frame.dq_mujoco[17 + i] = vel.Get(i);
  } else if (vel.size() == 31) {
    // Some A3 teleop recordings use the URDF body layout:
    // leg(12) + waist(3) + head(2) + arm(14).
    for (int i = 0; i < 3; ++i) frame.dq_mujoco[i] = vel.Get(12 + i);
    for (int i = 0; i < 14; ++i) frame.dq_mujoco[3 + i] = vel.Get(17 + i);
    for (int i = 0; i < 12; ++i) frame.dq_mujoco[17 + i] = vel.Get(i);
  } else if (vel.size() == 29) {
    // Already in policy-view order: waist(3) + arm(14) + leg(12).
    for (int i = 0; i < 29; ++i) frame.dq_mujoco[i] = vel.Get(i);
  }

  if (!IsFiniteArray(frame.q_mujoco) || !IsFiniteArray(frame.dq_mujoco) ||
      !IsFiniteQuat(frame.pelvis_quat_wxyz)) {
    return fail("whole body command contains non-finite values");
  }
  out = frame;
  return true;
}
#endif

}  // namespace a3_deploy
