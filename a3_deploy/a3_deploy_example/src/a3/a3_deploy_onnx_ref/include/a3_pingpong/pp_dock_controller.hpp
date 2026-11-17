#pragma once
// Header-only serve-pose dock. No HAL, ROS, or clock reads. The Runner and the
// MuJoCo probe both call Update; only the velocity source differs from Teleop.
#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>

namespace a3_pingpong {

enum class DockPhase : int {
  kIdle = 0,
  kEntry = 1,
  kAlign = 2,
  kSettle = 3,
  kVerify = 4,
  kDone = 5,
  kFailed = 6,
};

enum class DockResult : int {
  kNone = 0,
  kRunning = 1,
  kDone = 2,
  kFailed = 3,
};

enum class DockReason : int {
  kNone = 0,
  kTimeout = 1,
  kMocapStale = 2,
  kMocapInvalid = 3,
  kWrongFrame = 4,
  kStartTooFar = 5,
  kAttemptsExhausted = 6,
  kStopped = 7,
};

struct DockConfig {
  double target_x{-0.50};
  double target_y{-0.7625};
  double target_yaw{0.0};
  double position_tolerance_m{0.03};
  double yaw_tolerance_rad{0.02617993877991494};
  double capture_position_m{0.03};
  double capture_yaw_rad{0.02617993877991494};
  double reopen_position_m{0.05};
  double reopen_yaw_rad{0.05235987755982988};
  double gain_xy{0.5};
  double gain_yaw{0.8};
  double forward_m_s{0.25};
  double backward_m_s{0.15};
  double lateral_m_s{0.12};
  double yaw_rad_s{0.35};
  double min_linear_m_s{0.06};
  double min_yaw_rad_s{0.08};
  double yaw_priority_rad{0.35};
  double approach_distance_m{1.0e9};
  double timeout_s{45.0};
  double mocap_max_age_s{0.15};
  double settle_hold_s{0.30};
  int max_attempts{3};
  double max_start_distance_m{3.0};
};

struct DockPose {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double age_s{0.0};
  bool valid{false};
  bool frame_ok{false};
};

struct DockOutput {
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  DockPhase phase{DockPhase::kIdle};
  int attempt{0};
  double error_x{0.0};
  double error_y{0.0};
  double error_yaw{0.0};
  double distance{0.0};
  double elapsed_s{0.0};
  DockResult result{DockResult::kNone};
  DockReason reason{DockReason::kNone};
};

inline double DockWrap(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline double DockYawWxyz(double w, double x, double y, double z) {
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

inline void DockValidate(const DockConfig& cfg) {
  if (!(cfg.capture_position_m > 0.0 && cfg.capture_position_m <= cfg.position_tolerance_m &&
        cfg.position_tolerance_m < cfg.reopen_position_m && cfg.capture_yaw_rad > 0.0 &&
        cfg.capture_yaw_rad <= cfg.yaw_tolerance_rad && cfg.yaw_tolerance_rad < cfg.reopen_yaw_rad &&
        cfg.gain_xy > 0.0 && cfg.gain_yaw > 0.0 && cfg.forward_m_s > 0.0 && cfg.backward_m_s > 0.0 &&
        cfg.lateral_m_s > 0.0 && cfg.yaw_rad_s > 0.0 && cfg.min_linear_m_s > 0.0 &&
        cfg.min_linear_m_s <= cfg.forward_m_s && cfg.min_linear_m_s <= cfg.lateral_m_s &&
        cfg.min_yaw_rad_s > 0.0 && cfg.min_yaw_rad_s <= cfg.yaw_rad_s && cfg.yaw_priority_rad > 0.0 &&
        cfg.approach_distance_m >= 0.0 && cfg.timeout_s > 0.0 && cfg.mocap_max_age_s > 0.0 &&
        cfg.settle_hold_s >= 0.0 &&
        cfg.max_attempts >= 1 && cfg.max_start_distance_m > 0.0 && std::isfinite(cfg.target_x) &&
        std::isfinite(cfg.target_y) && std::isfinite(cfg.target_yaw))) {
    throw std::runtime_error("dock.yaml has an invalid limit, gain, or tolerance");
  }
}

inline double DockRequire(const YAML::Node& node, const char* key) {
  if (!node || !node[key] || !node[key].IsScalar()) {
    throw std::runtime_error(std::string("dock.yaml missing ") + key);
  }
  const double value = node[key].as<double>();
  if (!std::isfinite(value)) throw std::runtime_error(std::string("dock.yaml non-finite ") + key);
  return value;
}

inline DockConfig LoadDockConfig(const std::string& path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception& error) {
    throw std::runtime_error(std::string("dock.yaml unreadable: ") + error.what());
  }
  DockConfig cfg;
  cfg.target_x = DockRequire(root["target"], "x");
  cfg.target_y = DockRequire(root["target"], "y");
  cfg.target_yaw = DockRequire(root["target"], "yaw");
  cfg.position_tolerance_m = DockRequire(root["tolerance"], "position_m");
  cfg.yaw_tolerance_rad = DockRequire(root["tolerance"], "yaw_rad");
  cfg.capture_position_m = DockRequire(root["capture"], "position_m");
  cfg.capture_yaw_rad = DockRequire(root["capture"], "yaw_rad");
  cfg.reopen_position_m = DockRequire(root["reopen"], "position_m");
  cfg.reopen_yaw_rad = DockRequire(root["reopen"], "yaw_rad");
  cfg.gain_xy = DockRequire(root["gains"], "xy");
  cfg.gain_yaw = DockRequire(root["gains"], "yaw");
  cfg.forward_m_s = DockRequire(root["limits"], "forward_m_s");
  cfg.backward_m_s = DockRequire(root["limits"], "backward_m_s");
  cfg.lateral_m_s = DockRequire(root["limits"], "lateral_m_s");
  cfg.yaw_rad_s = DockRequire(root["limits"], "yaw_rad_s");
  cfg.min_linear_m_s = DockRequire(root["limits"], "min_linear_m_s");
  cfg.min_yaw_rad_s = DockRequire(root["limits"], "min_yaw_rad_s");
  cfg.yaw_priority_rad = DockRequire(root, "yaw_priority_rad");
  cfg.approach_distance_m = DockRequire(root, "approach_distance_m");
  cfg.timeout_s = DockRequire(root["timing"], "timeout_s");
  cfg.mocap_max_age_s = DockRequire(root["timing"], "mocap_max_age_s");
  cfg.settle_hold_s = DockRequire(root["timing"], "settle_hold_s");
  cfg.max_start_distance_m = DockRequire(root["timing"], "max_start_distance_m");
  if (!root["timing"] || !root["timing"]["max_attempts"] || !root["timing"]["max_attempts"].IsScalar()) {
    throw std::runtime_error("dock.yaml missing max_attempts");
  }
  cfg.max_attempts = root["timing"]["max_attempts"].as<int>();
  DockValidate(cfg);
  return cfg;
}

// Body-frame velocity toward the configured pelvis pose. +x is forward, +y is
// left, and positive yaw rate turns left. Yaw takes the short way around.
class PpDockController {
 public:
  explicit PpDockController(DockConfig config) : cfg_(std::move(config)) { DockValidate(cfg_); }

  const DockConfig& config() const { return cfg_; }

  void Begin(double now) {
    begin_s_ = now;
    settle_since_s_ = -1.0;
    attempt_ = 0;
    start_checked_ = false;
    phase_ = DockPhase::kAlign;
    result_ = DockResult::kRunning;
    reason_ = DockReason::kNone;
    error_x_ = error_y_ = error_yaw_ = distance_ = 0.0;
  }

  void RequestStop() {
    if (result_ == DockResult::kDone || result_ == DockResult::kFailed) return;
    Fail_(DockReason::kStopped);
  }

  DockOutput Update(double now, const std::optional<DockPose>& pose, bool entry_active, bool settled,
                    bool quiet) {
    DockOutput out;
    out.phase = phase_;
    out.attempt = attempt_;
    out.result = result_;
    out.reason = reason_;
    out.error_x = error_x_;
    out.error_y = error_y_;
    out.error_yaw = error_yaw_;
    out.distance = distance_;
    out.elapsed_s = phase_ == DockPhase::kIdle ? 0.0 : now - begin_s_;
    if (phase_ == DockPhase::kIdle || result_ == DockResult::kDone || result_ == DockResult::kFailed) {
      return out;
    }
    if (out.elapsed_s > cfg_.timeout_s) return FailOut_(DockReason::kTimeout, out);

    if (!pose.has_value() || !pose->valid) return FailOut_(DockReason::kMocapInvalid, out);
    if (!pose->frame_ok) return FailOut_(DockReason::kWrongFrame, out);
    if (!(pose->age_s >= 0.0) || pose->age_s > cfg_.mocap_max_age_s) {
      return FailOut_(DockReason::kMocapStale, out);
    }

    error_x_ = cfg_.target_x - pose->x;
    error_y_ = cfg_.target_y - pose->y;
    error_yaw_ = DockWrap(cfg_.target_yaw - pose->yaw);
    distance_ = std::hypot(error_x_, error_y_);
    out.error_x = error_x_;
    out.error_y = error_y_;
    out.error_yaw = error_yaw_;
    out.distance = distance_;
    if (!start_checked_) {
      start_checked_ = true;
      if (distance_ > cfg_.max_start_distance_m) return FailOut_(DockReason::kStartTooFar, out);
    }

    if (entry_active) {
      phase_ = DockPhase::kEntry;
      settle_since_s_ = -1.0;
      return Publish_(out, Eigen::Vector3d::Zero());
    }
    if (phase_ == DockPhase::kEntry) phase_ = DockPhase::kAlign;

    const bool captured = distance_ <= cfg_.capture_position_m && std::abs(error_yaw_) <= cfg_.capture_yaw_rad;
    const bool wide = distance_ <= cfg_.reopen_position_m && std::abs(error_yaw_) <= cfg_.reopen_yaw_rad;
    const Eigen::Vector3d command = Command_();

    if (phase_ == DockPhase::kAlign) {
      if (captured) {
        phase_ = DockPhase::kSettle;
        settle_since_s_ = -1.0;
        return Publish_(out, Eigen::Vector3d::Zero());
      }
      return Publish_(out, command);
    }
    if (phase_ == DockPhase::kSettle) {
      if (!wide) {
        phase_ = DockPhase::kAlign;
        settle_since_s_ = -1.0;
        return Publish_(out, command);
      }
      if (settled && quiet) {
        if (settle_since_s_ < 0.0) settle_since_s_ = now;
        if (now - settle_since_s_ >= cfg_.settle_hold_s) phase_ = DockPhase::kVerify;
      } else {
        settle_since_s_ = -1.0;
      }
      if (phase_ == DockPhase::kVerify) return Verify_(out);
      return Publish_(out, Eigen::Vector3d::Zero());
    }
    if (phase_ == DockPhase::kVerify) return Verify_(out);
    return Publish_(out, Eigen::Vector3d::Zero());
  }

 private:
  DockOutput Verify_(DockOutput& out) {
    const bool acceptable = distance_ <= cfg_.position_tolerance_m && std::abs(error_yaw_) <= cfg_.yaw_tolerance_rad;
    if (acceptable) {
      phase_ = DockPhase::kDone;
      result_ = DockResult::kDone;
      reason_ = DockReason::kNone;
      return Publish_(out, Eigen::Vector3d::Zero());
    }
    ++attempt_;
    out.attempt = attempt_;
    if (attempt_ >= cfg_.max_attempts) return FailOut_(DockReason::kAttemptsExhausted, out);
    phase_ = DockPhase::kAlign;
    settle_since_s_ = -1.0;
    return Publish_(out, Command_());
  }

  Eigen::Vector3d Command_() const {
    const double current_yaw = cfg_.target_yaw - error_yaw_;
    const double c = std::cos(current_yaw);
    const double s = std::sin(current_yaw);
    // Body +x is (cos yaw, sin yaw). While the final heading is still far off
    // and the pelvis is outside the approach radius, steer toward the target
    // point. The done check keeps using the final-yaw error.
    double command_yaw = error_yaw_;
    if (distance_ > cfg_.approach_distance_m && std::abs(error_yaw_) > cfg_.yaw_priority_rad) {
      command_yaw = DockWrap(std::atan2(error_y_, error_x_) - current_yaw);
    }
    const double ex_b = c * error_x_ + s * error_y_;
    const double ey_b = -s * error_x_ + c * error_y_;
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    const bool yaw_first = std::abs(command_yaw) > cfg_.yaw_priority_rad;
    if (!yaw_first) {
      velocity.x() = std::clamp(cfg_.gain_xy * ex_b, -cfg_.backward_m_s, cfg_.forward_m_s);
      velocity.y() = std::clamp(cfg_.gain_xy * ey_b, -cfg_.lateral_m_s, cfg_.lateral_m_s);
      const double linear = std::hypot(velocity.x(), velocity.y());
      const double linear_error = std::hypot(ex_b, ey_b);
      if (linear_error > cfg_.capture_position_m && linear < cfg_.min_linear_m_s) {
        const double scale = linear > 1e-9 ? cfg_.min_linear_m_s / linear : cfg_.min_linear_m_s / linear_error;
        const double sx = linear > 1e-9 ? velocity.x() : ex_b;
        const double sy = linear > 1e-9 ? velocity.y() : ey_b;
        velocity.x() = std::clamp(sx * scale, -cfg_.backward_m_s, cfg_.forward_m_s);
        velocity.y() = std::clamp(sy * scale, -cfg_.lateral_m_s, cfg_.lateral_m_s);
      }
    }
    velocity.z() = std::clamp(cfg_.gain_yaw * command_yaw, -cfg_.yaw_rad_s, cfg_.yaw_rad_s);
    if (std::abs(command_yaw) > cfg_.capture_yaw_rad && std::abs(velocity.z()) < cfg_.min_yaw_rad_s) {
      velocity.z() = std::copysign(cfg_.min_yaw_rad_s, command_yaw == 0.0 ? 1.0 : command_yaw);
    }
    return velocity;
  }

  void Fail_(DockReason reason) {
    phase_ = DockPhase::kFailed;
    result_ = DockResult::kFailed;
    reason_ = reason;
  }

  DockOutput Publish_(DockOutput& out, const Eigen::Vector3d& velocity) const {
    out.velocity = velocity;
    out.phase = phase_;
    out.attempt = attempt_;
    out.result = result_;
    out.reason = reason_;
    return out;
  }

  DockOutput FailOut_(DockReason reason, DockOutput& out) {
    Fail_(reason);
    return Publish_(out, Eigen::Vector3d::Zero());
  }

  DockConfig cfg_;
  DockPhase phase_{DockPhase::kIdle};
  DockResult result_{DockResult::kNone};
  DockReason reason_{DockReason::kNone};
  double begin_s_{0.0};
  double settle_since_s_{-1.0};
  double error_x_{0.0};
  double error_y_{0.0};
  double error_yaw_{0.0};
  double distance_{0.0};
  int attempt_{0};
  bool start_checked_{false};
};

}  // namespace a3_pingpong
