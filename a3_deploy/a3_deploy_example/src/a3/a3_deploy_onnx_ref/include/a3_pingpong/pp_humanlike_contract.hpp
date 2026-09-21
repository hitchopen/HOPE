#pragma once

#include "robot_io/robot_io_backend.hpp"
#include <Eigen/Geometry>
#include <array>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace a3_pingpong {

// Current a3_t2d5 HumanLike: deliberately separate from the legacy 15-action
// hybrid policy. Layout and processing recovered against the vendor binary.
inline constexpr std::array<int, 25> kHumanLike25ToSdk = {19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 0,
                                                          1,  2,  5,  6,  7,  8,  9,  12, 13, 14, 15, 16};

struct HumanLike25Parameters {
    Eigen::VectorXd default_q, action_scale, pos_scale, vel_scale, lower, upper, kp, kd, torque;
    Eigen::Vector3d gyro_scale {.25, .25, .25}, gravity_scale {1, 1, 1}, command_scale {1, 1, 1};
};

inline std::array<double, 89> HumanLike25Frame(const robot_io::RobotState& s, const HumanLike25Parameters& p,
                                               const Eigen::Vector3d& velocity, const std::array<double, 25>& previous,
                                               const std::array<double, 5>& gait) {
  if (s.q.size() != 31 || s.dq.size() != 31 || !s.q.allFinite() || !s.dq.allFinite() || !s.imu_gyro.allFinite() ||
      !s.imu_quat_wxyz.allFinite() || std::abs(s.imu_quat_wxyz.norm() - 1) > .02)
    throw std::invalid_argument("invalid HumanLike robot state");
  const auto& w = s.imu_quat_wxyz;
  const Eigen::Vector3d gravity = Eigen::Quaterniond(w[0], w[1], w[2], w[3]).conjugate() * (-Eigen::Vector3d::UnitZ());
  std::array<double, 89> f {};
  for (int i = 0; i < 3; ++i) {
    f[i] = s.imu_gyro[i] * p.gyro_scale[i];
    f[3 + i] = gravity[i] * p.gravity_scale[i];
    f[6 + i] = velocity[i] * p.command_scale[i];
  }
  for (int i = 0; i < 25; ++i) {
    const int j = kHumanLike25ToSdk[i];
    f[9 + i] = (s.q[j] - p.default_q[j]) * p.pos_scale[j];
    f[34 + i] = s.dq[j] * p.vel_scale[j];
    f[59 + i] = previous[i];
  }
  std::copy(gait.begin(), gait.end(), f.begin() + 84);
  return f;
}

inline Eigen::Vector3d HumanLikeAnkleInPelvis(const robot_io::RobotState& state, bool left) {
  const int i = left ? 19 : 25;
  Eigen::Vector3d p(0, left ? .12298 : -.122983, left ? -.17875 : -.178753);
  Eigen::Matrix3d r = Eigen::Quaterniond(.991445, left ? -.130526 : .130526, 0, 0).normalized().toRotationMatrix();
  r *= Eigen::AngleAxisd(state.q[i], Eigen::Vector3d::UnitY()).toRotationMatrix();
  p += r * Eigen::Vector3d(left ? 0 : -.00109999, left ? .0113163 : -.0113163, -.042233);
  r *= Eigen::Quaterniond(.991445, left ? .130526 : -.130526, 0, 0).normalized().toRotationMatrix();
  r *= Eigen::AngleAxisd(state.q[i + 1], Eigen::Vector3d::UnitX()).toRotationMatrix();
  r *= Eigen::AngleAxisd(state.q[i + 2], Eigen::Vector3d::UnitZ()).toRotationMatrix();
  p += r * Eigen::Vector3d(0, 0, -.37);
  r *= Eigen::AngleAxisd(state.q[i + 3], Eigen::Vector3d::UnitY()).toRotationMatrix();
  p += r * Eigen::Vector3d(0, left ? .0015 : -.0015, -.415);
  return p; // vendor end_frame is ankle_roll_Link, not the sole site
}

struct HumanLikeGaitParameters {
    double dt = .02, initial_frequency = 1, base = .8, vx_coeff = .1;
    double lateral_floor = .9, lateral_vx = .08, lateral_vy = .1;
    double low_floor = .9, fade_lo = .3, fade_hi = .4;
    double backward_floor = .9, backward_vx = -.05;
    double turning_floor = .9, turning_yaw = .25, turning_vx = .08, turning_vy = .08;
    double recovery_floor = .8, min_frequency = .4, max_frequency = 1.8, offset = .5, height = .15;
    double roll_limit = .08, pitch_limit = .08, pitch_offset = 0, foot_dx = .05, foot_y_min = .2, foot_y_max = .3;
    int zero_count = 15;
    bool finish_step_on_stop = false;
};

class HumanLikeGait {
  public:
    explicit HumanLikeGait(HumanLikeGaitParameters p = {})
      : p_(p) {
      Reset();
    }

    void Reset() {
      phase_ = 0;
      zero_count_ = p_.zero_count + 1;
      settled_ = true;
      features_ = {p_.initial_frequency, p_.offset, p_.height, 0, 1};
    }

    bool settled() const { return settled_; }

    const std::array<double, 5>& features() const { return features_; }

    const std::array<double, 5>& Update(const Eigen::Vector3d& target, const Eigen::Vector4d& quat_wxyz,
                                        const Eigen::Vector3d& left, const Eigen::Vector3d& right) {
      const double vx = std::abs(target[0]), vy = std::abs(target[1]);
      double f = p_.base + p_.vx_coeff * vx;
      const double speed = target.head<2>().norm();
      if (speed > .05)
        f += std::max(0., p_.low_floor - f) * std::clamp((p_.fade_hi - speed) / (p_.fade_hi - p_.fade_lo), 0., 1.);
      if (vy > p_.lateral_vy && vx < p_.lateral_vx) f = p_.lateral_floor;
      if (std::abs(target[2]) > p_.turning_yaw && vx < p_.turning_vx && vy < p_.turning_vy) f = p_.turning_floor;
      if (target[0] < p_.backward_vx) f = std::max(f, p_.backward_floor);
      if (target.norm() < .05 && !settled_) f = std::max(f, p_.recovery_floor);
      f = std::clamp(f, p_.min_frequency, p_.max_frequency);
      bool stop = false;
      if (target.norm() < .05) {
        ++zero_count_;
        const bool window = phase_ >= .95 || phase_ < .05 || (phase_ >= .45 && phase_ < .55);
        const double w = quat_wxyz[0], x = quat_wxyz[1], y = quat_wxyz[2], z = quat_wxyz[3];
        const double roll = std::atan2(2 * (w * x + y * z), w * w - x * x - y * y + z * z);
        const double pitch = std::asin(std::clamp(2 * (w * y - x * z), -.99999, .99999));
        const double dy = std::abs(left.y() - right.y());
        const bool feet_aligned = std::abs(left.x() - right.x()) < p_.foot_dx &&
                                  dy >= p_.foot_y_min && dy <= p_.foot_y_max;
        // Runner stop protocol: finish this half-step, then let the zero-phase
        // policy settle the feet. Requiring settled feet BEFORE parking phase
        // can keep the actor stepping forever. Runner independently waits for
        // measured quiet before changing controller or increasing stand gains.
        stop = zero_count_ > p_.zero_count && window &&
               (feet_aligned || p_.finish_step_on_stop) &&
               std::abs(roll) < p_.roll_limit &&
               std::abs(pitch + p_.pitch_offset) < p_.pitch_limit;
      } else zero_count_ = 0;
      if (stop) {
        phase_ = phase_ < .25 || phase_ >= .75 ? 0 : .5;
        settled_ = true;
      } else if (settled_) {
        phase_ = p_.offset;
        settled_ = false;
      } else phase_ = std::fmod(phase_ + p_.dt * f, 1.);
      features_ = {f, p_.offset, p_.height, std::sin(2 * 3.141592653589793 * phase_),
                   std::cos(2 * 3.141592653589793 * phase_)};
      return features_;
    }
  private:
    HumanLikeGaitParameters p_;
    double phase_ = 0;
    int zero_count_ = 0;
    bool settled_ = true;
    std::array<double, 5> features_ {};
};

inline robot_io::RobotCommand HumanLike25Command(const robot_io::RobotState& s, const HumanLike25Parameters& p,
                                                 const std::array<double, 25>& action,
                                                 const robot_io::RobotCommand& head_source) {
  robot_io::RobotCommand c;
  c.q_des = p.default_q;
  c.dq_des = Eigen::VectorXd::Zero(31);
  c.tau_ff = c.dq_des;
  c.kp = p.kp;
  c.kd = p.kd;
  for (int i = 0; i < 25; ++i) c.q_des[kHumanLike25ToSdk[i]] += action[i] * p.action_scale[kHumanLike25ToSdk[i]];
  for (int j = 0; j < 31; ++j) {
    if (j == 3 || j == 4) {
      c.q_des[j] = head_source.q_des[j];
      c.dq_des[j] = head_source.dq_des[j];
      c.tau_ff[j] = head_source.tau_ff[j];
      c.kp[j] = head_source.kp[j];
      c.kd[j] = head_source.kd[j];
      continue;
    }
    c.q_des[j] = std::clamp(c.q_des[j], p.lower[j], p.upper[j]);
    c.q_des[j] = std::clamp(c.q_des[j], s.q[j] + (p.kd[j] * s.dq[j] - p.torque[j]) / p.kp[j],
                            s.q[j] + (p.kd[j] * s.dq[j] + p.torque[j]) / p.kp[j]);
  }
  return c;
}
} // namespace a3_pingpong
