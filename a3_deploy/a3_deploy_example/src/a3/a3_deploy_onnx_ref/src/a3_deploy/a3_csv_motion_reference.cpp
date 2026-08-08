// Copyright (c) 2026, AgiBot Inc. All rights reserved.
#include "a3_deploy/a3_csv_motion_reference.hpp"

#include "a3_deploy/numeric_safety.hpp"
#include "a3_deploy/a3_obs_builder.hpp"
#include "a3_policy_parameters.hpp"
#include "robot_io/a3_layout_extra.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <unordered_map>

namespace a3_deploy {

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kCsvCentimetersToMeters = 0.01;
constexpr std::size_t kFutureFrames = 10;
constexpr std::size_t kPolicyDof = 29;
constexpr std::size_t kCommandDofFloats = kFutureFrames * kPolicyDof;  // 290

constexpr std::array<const char*, robot_io::kA3Dof> kA3CsvJointNames = {{
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

std::string Trim(std::string s) {
  while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) {
    s.erase(s.begin());
  }
  while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) {
    s.pop_back();
  }
  if (s.size() >= 2 && s.front() == '"' && s.back() == '"') {
    s = s.substr(1, s.size() - 2);
  }
  return s;
}

std::vector<std::string> SplitCsvLine(const std::string& line) {
  std::vector<std::string> fields;
  std::string cur;
  std::stringstream ss(line);
  while (std::getline(ss, cur, ',')) fields.push_back(Trim(cur));
  if (!line.empty() && line.back() == ',') fields.emplace_back();
  return fields;
}

bool ParseDouble(const std::string& s, double* out) {
  if (!out) return false;
  try {
    std::size_t parsed = 0;
    const double value = std::stod(s, &parsed);
    if (parsed != s.size() || !numeric_safety::IsFinite(value)) return false;
    *out = value;
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

std::array<double, 4> NormalizeQuat(std::array<double, 4> q) {
  double n2 = 0.0;
  for (double v : q) n2 += v * v;
  if (n2 <= std::numeric_limits<double>::epsilon()) return {1.0, 0.0, 0.0, 0.0};
  const double inv = 1.0 / std::sqrt(n2);
  for (double& v : q) v *= inv;
  return q;
}

std::array<double, 4> QuatMul(const std::array<double, 4>& a,
                              const std::array<double, 4>& b) {
  const double w1 = a[0], x1 = a[1], y1 = a[2], z1 = a[3];
  const double w2 = b[0], x2 = b[1], y2 = b[2], z2 = b[3];
  return {
      w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
      w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
      w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
      w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
  };
}

std::array<double, 4> QuatConj(const std::array<double, 4>& q) {
  return {q[0], -q[1], -q[2], -q[3]};
}

double WrapPi(double x) {
  while (x > kPi) x -= 2.0 * kPi;
  while (x < -kPi) x += 2.0 * kPi;
  return x;
}

double QuatYawRad(const std::array<double, 4>& q_in) {
  const auto q = NormalizeQuat(q_in);
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  return std::atan2(2.0 * (w * z + x * y),
                    1.0 - 2.0 * (y * y + z * z));
}

std::array<double, 4> QuatFromYawRad(double yaw) {
  const double half = 0.5 * yaw;
  return {std::cos(half), 0.0, 0.0, std::sin(half)};
}

double QuatDot(const std::array<double, 4>& a, const std::array<double, 4>& b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
}

std::array<double, 4> QuatSlerp(const std::array<double, 4>& qa,
                                const std::array<double, 4>& qb,
                                double alpha) {
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

// Matches scipy.spatial.transform.Rotation.from_euler("xyz", degrees=True):
// extrinsic fixed-axis X, then Y, then Z; quaternion product qz * qy * qx.
std::array<double, 4> QuatFromEulerXyzDegrees(double x_deg,
                                              double y_deg,
                                              double z_deg) {
  const double hx = 0.5 * x_deg * kDegToRad;
  const double hy = 0.5 * y_deg * kDegToRad;
  const double hz = 0.5 * z_deg * kDegToRad;
  const std::array<double, 4> qx{std::cos(hx), std::sin(hx), 0.0, 0.0};
  const std::array<double, 4> qy{std::cos(hy), 0.0, std::sin(hy), 0.0};
  const std::array<double, 4> qz{std::cos(hz), 0.0, 0.0, std::sin(hz)};
  return NormalizeQuat(QuatMul(QuatMul(qz, qy), qx));
}

std::array<double, 3> QuatToRotvec(const std::array<double, 4>& q_in) {
  std::array<double, 4> q = NormalizeQuat(q_in);
  if (q[0] < 0.0) {
    for (double& v : q) v = -v;
  }
  const double v_norm = std::sqrt(q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (v_norm < 1e-12) return {0.0, 0.0, 0.0};
  double angle = 2.0 * std::atan2(v_norm, q[0]);
  if (angle > kPi) angle -= 2.0 * kPi;
  const double scale = angle / v_norm;
  return {q[1] * scale, q[2] * scale, q[3] * scale};
}

void QuatToMatrix(const std::array<double, 4>& q_in, double m[3][3]) {
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

std::string BasenameNoExt(const std::string& path) {
  const std::filesystem::path p(path);
  return p.stem().string();
}

}  // namespace

bool A3CsvMotionReference::Load(
    const std::string& csv_path,
    const A3CsvMotionReferenceOptions& options) {
  options_ = options;
  frames_.clear();
  dof_isaaclab_.clear();
  dof_vel_isaaclab_.clear();
  meta_ = {};

  if (options_.source_fps <= 0.0 || options_.target_fps <= 0.0) {
    std::cerr << "A3CsvMotionReference: source_fps/target_fps must be > 0\n";
    return false;
  }
  if (options_.future_frame_skip <= 0) {
    std::cerr << "A3CsvMotionReference: future_frame_skip must be > 0\n";
    return false;
  }
  if (options_.csv_frame_stride <= 0) {
    std::cerr << "A3CsvMotionReference: csv_frame_stride must be > 0\n";
    return false;
  }

  std::ifstream f(csv_path);
  if (!f.is_open()) {
    std::cerr << "A3CsvMotionReference: failed to open CSV: " << csv_path
              << "\n";
    return false;
  }

  std::string line;
  if (!std::getline(f, line)) {
    std::cerr << "A3CsvMotionReference: empty CSV: " << csv_path << "\n";
    return false;
  }
  if (!line.empty() && line.back() == '\r') line.pop_back();
  const auto header = SplitCsvLine(line);
  std::unordered_map<std::string, std::size_t> col;
  for (std::size_t i = 0; i < header.size(); ++i) col[header[i]] = i;

  const std::array<const char*, 6> root_cols = {{
      "root_translateX", "root_translateY", "root_translateZ",
      "root_rotateX", "root_rotateY", "root_rotateZ",
  }};
  for (const char* name : root_cols) {
    if (col.find(name) == col.end()) {
      std::cerr << "A3CsvMotionReference: missing column: " << name << "\n";
      return false;
    }
  }

  std::array<std::string, robot_io::kA3Dof> joint_cols;
  for (std::size_t i = 0; i < kA3CsvJointNames.size(); ++i) {
    const std::string name = kA3CsvJointNames[i];
    if (col.find(name) != col.end()) {
      joint_cols[i] = name;
      continue;
    }
    const std::string dof_name = name + "_dof";
    if (col.find(dof_name) != col.end()) {
      joint_cols[i] = dof_name;
      continue;
    }
    std::cerr << "A3CsvMotionReference: missing A3 joint column: " << name
              << "\n";
    return false;
  }

  std::vector<Frame> src;
  std::size_t row_num = 1;
  std::size_t raw_rows = 0;
  while (std::getline(f, line)) {
    ++row_num;
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (Trim(line).empty()) continue;
    const std::size_t raw_idx = raw_rows++;
    if ((raw_idx % static_cast<std::size_t>(options_.csv_frame_stride)) != 0) {
      continue;
    }
    const auto fields = SplitCsvLine(line);
    if (fields.size() < header.size()) {
      std::cerr << "A3CsvMotionReference: row " << row_num
                << " has too few columns (" << fields.size() << " < "
                << header.size() << ")\n";
      return false;
    }

    auto get = [&](const std::string& name, double* out) -> bool {
      const auto it = col.find(name);
      return it != col.end() && it->second < fields.size() &&
             ParseDouble(fields[it->second], out);
    };

    double tx = 0.0, ty = 0.0, tz = 0.0;
    double rx = 0.0, ry = 0.0, rz = 0.0;
    if (!get("root_translateX", &tx) || !get("root_translateY", &ty) ||
        !get("root_translateZ", &tz) || !get("root_rotateX", &rx) ||
        !get("root_rotateY", &ry) || !get("root_rotateZ", &rz)) {
      std::cerr << "A3CsvMotionReference: failed to parse root row "
                << row_num << "\n";
      return false;
    }

    Frame frame;
    frame.root_pos = {tx * kCsvCentimetersToMeters,
                      ty * kCsvCentimetersToMeters,
                      tz * kCsvCentimetersToMeters};
    frame.root_quat_wxyz = QuatFromEulerXyzDegrees(rx, ry, rz);

    for (std::size_t mj29 = 0; mj29 < kPolicyDof; ++mj29) {
      const int sdk31 = robot_io::kA3PolicyToSdkIdx[mj29];
      double deg = 0.0;
      if (!get(joint_cols[sdk31], &deg)) {
        std::cerr << "A3CsvMotionReference: failed to parse joint "
                  << kA3CsvJointNames[sdk31] << " at row " << row_num << "\n";
        return false;
      }
      frame.dof_mujoco[mj29] = static_cast<float>(deg * kDegToRad);
    }
    src.push_back(frame);
  }

  if (src.empty()) {
    std::cerr << "A3CsvMotionReference: CSV has no data rows: " << csv_path
              << "\n";
    return false;
  }

  const double source_dt = 1.0 / options_.source_fps;
  const double target_dt = 1.0 / options_.target_fps;
  const double duration =
      (src.size() <= 1) ? 0.0 : (static_cast<double>(src.size() - 1) * source_dt);
  std::vector<double> target_times;
  if (src.size() == 1) {
    target_times.push_back(0.0);
  } else {
    for (double t = 0.0; t < duration - 1e-12; t += target_dt) {
      target_times.push_back(t);
    }
    if (target_times.empty()) target_times.push_back(0.0);
  }

  frames_.reserve(target_times.size());
  for (double t : target_times) {
    const double src_pos = t * options_.source_fps;
    std::size_t i0 = static_cast<std::size_t>(std::floor(src_pos));
    if (i0 >= src.size() - 1) {
      i0 = src.size() - 1;
      frames_.push_back(src[i0]);
      continue;
    }
    const std::size_t i1 = i0 + 1;
    const double alpha = std::clamp(src_pos - static_cast<double>(i0), 0.0, 1.0);

    Frame out;
    for (std::size_t i = 0; i < 3; ++i) {
      out.root_pos[i] =
          (1.0 - alpha) * src[i0].root_pos[i] + alpha * src[i1].root_pos[i];
    }
    out.root_quat_wxyz =
        QuatSlerp(src[i0].root_quat_wxyz, src[i1].root_quat_wxyz, alpha);
    for (std::size_t j = 0; j < kPolicyDof; ++j) {
      out.dof_mujoco[j] = static_cast<float>(
          (1.0 - alpha) * src[i0].dof_mujoco[j] +
          alpha * src[i1].dof_mujoco[j]);
    }
    frames_.push_back(out);
  }

  dof_isaaclab_.resize(frames_.size());
  dof_vel_isaaclab_.resize(frames_.size());
  for (std::size_t t = 0; t < frames_.size(); ++t) {
    for (std::size_t i_il = 0; i_il < kPolicyDof; ++i_il) {
      dof_isaaclab_[t][i_il] =
          frames_[t].dof_mujoco[a3_mujoco_to_isaaclab[i_il]];
    }
  }

  for (auto& v : dof_vel_isaaclab_) v.fill(0.0f);
  if (frames_.size() >= 2) {
    for (std::size_t t = 0; t + 1 < frames_.size(); ++t) {
      for (std::size_t j = 0; j < kPolicyDof; ++j) {
        dof_vel_isaaclab_[t][j] =
            static_cast<float>((dof_isaaclab_[t + 1][j] - dof_isaaclab_[t][j]) *
                               options_.target_fps);
      }
    }
    dof_vel_isaaclab_.back() = dof_vel_isaaclab_[frames_.size() - 2];
  }

  meta_.num_ticks = frames_.size();
  meta_.dt_ns =
      static_cast<std::int64_t>(std::llround(1'000'000'000.0 / options_.target_fps));
  meta_.pkl_source = csv_path;
  meta_.clip_name = BasenameNoExt(csv_path);
  meta_.has_initial_state = true;
  meta_.init_root_pos = frames_.front().root_pos;
  meta_.init_root_quat_wxyz = frames_.front().root_quat_wxyz;
  meta_.init_joint_pos_mujoco_29.fill(0.0);
  meta_.init_joint_vel_mujoco_29.fill(0.0);
  for (std::size_t j = 0; j < kPolicyDof; ++j) {
    meta_.init_joint_pos_mujoco_29[j] = frames_.front().dof_mujoco[j];
    if (frames_.size() >= 2) {
      meta_.init_joint_vel_mujoco_29[j] =
          (frames_[1].dof_mujoco[j] - frames_[0].dof_mujoco[j]) *
          options_.target_fps;
    }
  }
  if (frames_.size() >= 2) {
    for (std::size_t i = 0; i < 3; ++i) {
      meta_.init_root_lin_vel[i] =
          (frames_[1].root_pos[i] - frames_[0].root_pos[i]) * options_.target_fps;
    }
    const auto q_delta =
        QuatMul(QuatConj(frames_[0].root_quat_wxyz), frames_[1].root_quat_wxyz);
    const auto rotvec = QuatToRotvec(q_delta);
    for (std::size_t i = 0; i < 3; ++i) {
      meta_.init_root_ang_vel[i] = rotvec[i] * options_.target_fps;
    }
  }

  std::cout << "A3CsvMotionReference loaded: " << csv_path
            << " raw_rows=" << raw_rows
            << " src_rows=" << src.size()
            << " csv_frame_stride=" << options_.csv_frame_stride
            << " ticks=" << frames_.size()
            << " source_fps=" << options_.source_fps
            << " target_fps=" << options_.target_fps << "\n";
  return true;
}

bool A3CsvMotionReference::ResolveTick(std::size_t tick_idx,
                                       std::size_t* out_idx) const noexcept {
  if (frames_.empty() || out_idx == nullptr) return false;
  if (tick_idx < frames_.size()) {
    *out_idx = tick_idx;
    return true;
  }
  switch (options_.on_end) {
    case OnEndPolicy::kHoldLast:
      *out_idx = frames_.size() - 1;
      return true;
    case OnEndPolicy::kWrap:
      *out_idx = tick_idx % frames_.size();
      return true;
    case OnEndPolicy::kStop:
      return false;
  }
  return false;
}

std::size_t A3CsvMotionReference::ResolveFutureTick(
    std::size_t base_idx,
    std::size_t future_offset) const noexcept {
  if (frames_.empty()) return 0;
  const std::size_t idx = base_idx + future_offset;
  if (idx < frames_.size()) return idx;
  if (options_.on_end == OnEndPolicy::kWrap) return idx % frames_.size();
  return frames_.size() - 1;
}

bool A3CsvMotionReference::BuildTokenizerSlice(
    std::size_t tick_idx,
    const std::array<double, 4>& robot_root_quat_wxyz,
    std::array<float, kA3TokenizerFloatsPerTick>& out) const noexcept {
  return BuildTokenizerSlice(tick_idx, robot_root_quat_wxyz, 0.0, out);
}

bool A3CsvMotionReference::BuildTokenizerSlice(
    std::size_t tick_idx,
    const std::array<double, 4>& robot_root_quat_wxyz,
    double reference_yaw_offset_rad,
    std::array<float, kA3TokenizerFloatsPerTick>& out) const noexcept {
  out.fill(0.0f);
  std::size_t base_idx = 0;
  if (!ResolveTick(tick_idx, &base_idx)) return false;

  const auto inv_robot_quat = QuatConj(NormalizeQuat(robot_root_quat_wxyz));
  const auto yaw_offset_quat = QuatFromYawRad(reference_yaw_offset_rad);

  for (std::size_t k = 0; k < kFutureFrames; ++k) {
    const std::size_t idx = ResolveFutureTick(
        base_idx, k * static_cast<std::size_t>(options_.future_frame_skip));

    for (std::size_t j = 0; j < kPolicyDof; ++j) {
      out[k * kPolicyDof + j] = dof_isaaclab_[idx][j];
      out[kCommandDofFloats + k * kPolicyDof + j] = dof_vel_isaaclab_[idx][j];
    }

    const auto yaw_aligned_ref =
        QuatMul(yaw_offset_quat, frames_[idx].root_quat_wxyz);
    const auto rel = QuatMul(inv_robot_quat, yaw_aligned_ref);
    double m[3][3];
    QuatToMatrix(rel, m);
    const std::size_t o = kA3CommandMultiFutureFloats + k * 6;
    out[o + 0] = static_cast<float>(m[0][0]);
    out[o + 1] = static_cast<float>(m[0][1]);
    out[o + 2] = static_cast<float>(m[1][0]);
    out[o + 3] = static_cast<float>(m[1][1]);
    out[o + 4] = static_cast<float>(m[2][0]);
    out[o + 5] = static_cast<float>(m[2][1]);
  }
  return true;
}

bool A3CsvMotionReference::BuildHeldTokenizerSlice(
    std::size_t tick_idx,
    const std::array<double, 4>& robot_root_quat_wxyz,
    double reference_yaw_offset_rad,
    std::array<float, kA3TokenizerFloatsPerTick>& out) const noexcept {
  out.fill(0.0f);
  std::size_t idx = 0;
  if (!ResolveTick(tick_idx, &idx)) return false;

  const auto inv_robot_quat = QuatConj(NormalizeQuat(robot_root_quat_wxyz));
  const auto yaw_offset_quat = QuatFromYawRad(reference_yaw_offset_rad);
  const auto yaw_aligned_ref =
      QuatMul(yaw_offset_quat, frames_[idx].root_quat_wxyz);
  const auto rel = QuatMul(inv_robot_quat, yaw_aligned_ref);
  double m[3][3];
  QuatToMatrix(rel, m);

  for (std::size_t k = 0; k < kFutureFrames; ++k) {
    for (std::size_t j = 0; j < kPolicyDof; ++j) {
      out[k * kPolicyDof + j] = dof_isaaclab_[idx][j];
      out[kCommandDofFloats + k * kPolicyDof + j] = 0.0f;
    }

    const std::size_t o = kA3CommandMultiFutureFloats + k * 6;
    out[o + 0] = static_cast<float>(m[0][0]);
    out[o + 1] = static_cast<float>(m[0][1]);
    out[o + 2] = static_cast<float>(m[1][0]);
    out[o + 3] = static_cast<float>(m[1][1]);
    out[o + 4] = static_cast<float>(m[2][0]);
    out[o + 5] = static_cast<float>(m[2][1]);
  }
  return true;
}

double A3CsvMotionReference::ComputeYawOffsetRad(
    const std::array<double, 4>& robot_root_quat_wxyz) const noexcept {
  if (frames_.empty()) return 0.0;
  return WrapPi(QuatYawRad(robot_root_quat_wxyz) -
                QuatYawRad(frames_.front().root_quat_wxyz));
}

}  // namespace a3_deploy
