#pragma once

#include "robot_io/robot_io_backend.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace a3_pingpong {

// Driver-thread-only mode handoff. Blend the live destination controller
// with the source pose and a decaying source-velocity term. At t=0 all five
// command fields equal the last delivered command; at t=T they equal the
// destination. BeginImpedance instead cross-fades the two complete PD torque
// laws, using stiffness/damping-weighted setpoints. Neither mode smooths
// individual policy actions after entry.
class PpCommandTransition {
  public:
    static bool Valid(const robot_io::RobotCommand& c) noexcept {
      const auto n = c.q_des.size();
      return n > 0 && c.dq_des.size() == n && c.tau_ff.size() == n && c.kp.size() == n && c.kd.size() == n &&
             c.q_des.allFinite() && c.dq_des.allFinite() && c.tau_ff.allFinite() && c.kp.allFinite() &&
             c.kd.allFinite() && (c.kp.array() >= 0).all() && (c.kd.array() >= 0).all();
    }

    void Begin(const robot_io::RobotCommand& source, const robot_io::RobotCommand& target, double duration_s,
               double source_velocity_decay_s = -1) {
      if (!Valid(source) || !Valid(target) || source.q_des.size() != target.q_des.size() ||
          !std::isfinite(duration_s) || duration_s <= 0 || !std::isfinite(source_velocity_decay_s) ||
          (source_velocity_decay_s != -1 && source_velocity_decay_s <= 0))
        throw std::invalid_argument("invalid mode transition command/duration");
      source_ = source;
      duration_s_ = duration_s;
      velocity_decay_s_ = source_velocity_decay_s < 0 ? duration_s : std::min(duration_s, source_velocity_decay_s);
      active_ = true;
      blend_impedance_ = false;
    }

    // Cross-fade the two PD controllers, rather than multiplying independently
    // blended gains and positions (which amplifies a low-gain policy's request
    // with the standing controller's high gain during entry).
    void BeginImpedance(const robot_io::RobotCommand& source,
                        const robot_io::RobotCommand& target, double duration_s) {
      Begin(source, target, duration_s);
      blend_impedance_ = true;
    }

    void Reset() noexcept { active_ = false; }

    bool active() const noexcept { return active_; }

    void Apply(double elapsed_s, robot_io::RobotCommand& target) {
      if (!active_) return;
      if (!std::isfinite(elapsed_s) || elapsed_s < 0 || !Valid(target) || target.q_des.size() != source_.q_des.size())
        throw std::invalid_argument("invalid mode transition sample");
      const double u = std::clamp(elapsed_s / duration_s_, 0.0, 1.0);
      if (u >= 1) {
        active_ = false;
        return;
      }
      if (u == 0) {
        target = source_;
        return;
      }
      const double u2 = u * u, u3 = u2 * u, u4 = u3 * u, u5 = u4 * u;
      const double h = 1 - 10 * u3 + 15 * u4 - 6 * u5;
      if (blend_impedance_) {
        for (Eigen::Index i = 0; i < target.q_des.size(); ++i) {
          const double kp = h * source_.kp[i] + (1 - h) * target.kp[i];
          const double kd = h * source_.kd[i] + (1 - h) * target.kd[i];
          target.q_des[i] = kp > 0
              ? (h * source_.kp[i] * source_.q_des[i] +
                 (1 - h) * target.kp[i] * target.q_des[i]) / kp
              : h * source_.q_des[i] + (1 - h) * target.q_des[i];
          target.dq_des[i] = kd > 0
              ? (h * source_.kd[i] * source_.dq_des[i] +
                 (1 - h) * target.kd[i] * target.dq_des[i]) / kd
              : h * source_.dq_des[i] + (1 - h) * target.dq_des[i];
          target.kp[i] = kp;
          target.kd[i] = kd;
          target.tau_ff[i] = h * source_.tau_ff[i] + (1 - h) * target.tau_ff[i];
        }
        return;
      }
      const double dh = (-30 * u2 + 60 * u3 - 30 * u4) / duration_s_;
      // A long pose transition must not extrapolate entry velocity for its
      // whole duration. Decay that velocity on its own smooth time scale.
      const double uv = std::clamp(elapsed_s / velocity_decay_s_, 0.0, 1.0);
      const double uv2 = uv * uv, uv3 = uv2 * uv, uv4 = uv3 * uv, uv5 = uv4 * uv;
      const double v = velocity_decay_s_ * (uv - 6 * uv3 + 8 * uv4 - 3 * uv5);
      const double dv = 1 - 18 * uv2 + 32 * uv3 - 15 * uv4;
      // The weight derivative is necessary even when both controllers publish
      // zero dq. A dynamic policy target is attenuated during the handoff too.
      target.dq_des = (1 - h) * target.dq_des + dh * (source_.q_des - target.q_des) + dv * source_.dq_des;
      target.q_des = h * source_.q_des + (1 - h) * target.q_des + v * source_.dq_des;
      target.kp = h * source_.kp + (1 - h) * target.kp;
      target.kd = h * source_.kd + (1 - h) * target.kd;
      target.tau_ff = h * source_.tau_ff + (1 - h) * target.tau_ff;
    }
  private:
    robot_io::RobotCommand source_;
    double duration_s_ {0};
    double velocity_decay_s_ {0};
    bool active_ {false};
    bool blend_impedance_ {false};
};

} // namespace a3_pingpong
