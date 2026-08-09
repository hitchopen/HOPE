#pragma once

#include <Eigen/Dense>

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

#include "a3_pingpong/pp_joint_map.hpp"

namespace a3_pingpong {

// Final published-command monitor. It runs after every policy override, gain
// selection and entry blend, so it observes the command the backend will
// actually receive. A fault throws; A3PolicyDriver latches safe-halt.
class PpCommandSafetyMonitor {
 public:
  static constexpr int kJointCount = 31;

  void Reset() {
    initialized_ = false;
    last_q_des_.resize(0);
  }

  void Seed(const Eigen::VectorXd& q_des) {
    if (q_des.size() != kJointCount || !q_des.allFinite()) {
      Reset();
      return;
    }
    last_q_des_ = q_des;
    initialized_ = true;
  }

  void ValidateAndAdvance(const Eigen::VectorXd& q_des,
                          const Eigen::VectorXd& q_measured,
                          const Eigen::VectorXd& dq_measured) {
    if (q_des.size() != kJointCount ||
        q_measured.size() != kJointCount ||
        dq_measured.size() != kJointCount) {
      throw std::runtime_error(
          "V17 COMMAND SAFETY FAULT: expected 31-D q_des/q/dq");
    }
    if (!q_des.allFinite() || !q_measured.allFinite() ||
        !dq_measured.allFinite()) {
      throw std::runtime_error(
          "V17 COMMAND SAFETY FAULT: non-finite q_des/q/dq");
    }
    if (!initialized_) Seed(q_measured);

    const auto& names = backend_joint_order();
    for (int i = 0; i < kJointCount; ++i) {
      const Limits limits = LimitsFor_(names[static_cast<std::size_t>(i)]);
      const double step = std::fabs(q_des[i] - last_q_des_[i]);
      if (step > limits.max_step_rad + 1.0e-12) {
        throw std::runtime_error(
            "V17 COMMAND SAFETY FAULT: q_des step " +
            std::to_string(step) + " rad exceeds " +
            std::to_string(limits.max_step_rad) + " at " + names[i]);
      }
      const double tracking = std::fabs(q_des[i] - q_measured[i]);
      if (tracking > limits.max_tracking_error_rad + 1.0e-12) {
        throw std::runtime_error(
            "V17 COMMAND SAFETY FAULT: |q_des-q| " +
            std::to_string(tracking) + " rad exceeds " +
            std::to_string(limits.max_tracking_error_rad) + " at " + names[i]);
      }
      const double speed = std::fabs(dq_measured[i]);
      if (speed > limits.max_abs_velocity_rps + 1.0e-12) {
        throw std::runtime_error(
            "V17 COMMAND SAFETY FAULT: |dq| " + std::to_string(speed) +
            " rad/s exceeds " +
            std::to_string(limits.max_abs_velocity_rps) + " at " + names[i]);
      }
    }
    last_q_des_ = q_des;
  }

 private:
  struct Limits {
    double max_step_rad;
    double max_tracking_error_rad;
    double max_abs_velocity_rps;
  };

  static bool Contains_(const std::string& value, const char* token) {
    return value.find(token) != std::string::npos;
  }

  static Limits LimitsFor_(const std::string& name) {
    if (Contains_(name, "waist"))
      return {0.07, 0.45, 5.0};
    if (Contains_(name, "head"))
      return {0.10, 0.35, 5.0};
    if (Contains_(name, "knee"))
      return {0.10, 0.50, 7.0};
    if (Contains_(name, "hip") || Contains_(name, "ankle"))
      return {0.08, 0.50, 7.0};
    if (Contains_(name, "shoulder"))
      return {0.16, 0.80, 12.0};
    if (Contains_(name, "elbow"))
      return {0.18, 0.80, 12.0};
    if (Contains_(name, "wrist"))
      return {0.20, 0.80, 15.0};
    return {0.16, 0.80, 12.0};
  }

  bool initialized_ = false;
  Eigen::VectorXd last_q_des_;
};

}  // namespace a3_pingpong
