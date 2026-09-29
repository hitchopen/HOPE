#pragma once
#include <Eigen/Core>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <vector>

namespace a3_pingpong {
// Separate latest-sample stream; never enters the discrete action queue.
// Wire: [1, runner_boot, source_session, sequence, original_wall_s,
//        connected, enabled, body_vx, body_vy, yaw_rate, sticks_neutral].
// Relays must not restamp.
class PpTeleopInput {
  public:
    explicit PpTeleopInput(std::uint64_t boot)
      : boot_(boot) {}

    bool Receive(const std::vector<double>& v, double wall, double steady) {
      if (v.size() != 11 || v[0] != 1) return false;
      for (double x : v)
        if (!std::isfinite(x)) return false;
      const auto integer = [](double x) { return x >= 1 && x < 4503599627370496. && std::floor(x) == x; };
      if (!integer(v[1]) || !integer(v[2]) || !integer(v[3]) || v[1] != static_cast<double>(boot_) ||
          (v[5] != 0 && v[5] != 1) || (v[6] != 0 && v[6] != 1) || (v[10] != 0 && v[10] != 1) || wall - v[4] > .2 ||
          wall - v[4] < -.05 || v[7] < -.5 || v[7] > .8 || std::abs(v[8]) > .3 || std::abs(v[9]) > 1)
        return false;
      const bool neutral = std::abs(v[7]) + std::abs(v[8]) + std::abs(v[9]) < 1e-6;
      if ((v[5] == 0 || v[6] == 0) && !neutral) return false;
      std::lock_guard<std::mutex> lock(mutex_);
      if (session_ != static_cast<std::uint64_t>(v[2])) {
        if ((session_ != 0 && steady - receipt_ < .2) || v[6] != 0 || !neutral || v[10] != 1) return false;
        session_ = static_cast<std::uint64_t>(v[2]);
        sequence_ = 0;
        armed_ = false;
        released_ = false;
      }
      if (v[3] <= static_cast<double>(sequence_)) return false;
      sequence_ = static_cast<std::uint64_t>(v[3]);
      receipt_ = steady;
      source_wall_ = v[4];
      connected_ = v[5] == 1;
      enabled_ = v[6] == 1;
      neutral_ = v[10] == 1;
      velocity_ = Eigen::Vector3d(v[7], v[8], v[9]);
      return true;
    }

    bool Ready(double wall, double steady) const {
      std::lock_guard<std::mutex> lock(mutex_);
      return Fresh_(wall, steady) && connected_ && !enabled_ && neutral_ && velocity_.norm() < 1e-6;
    }

    void Disarm() {
      std::lock_guard<std::mutex> lock(mutex_);
      armed_ = false;
      released_ = false;
    }

    Eigen::Vector3d Sample(bool gate_open, double wall, double steady) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!gate_open || !Fresh_(wall, steady) || !connected_) {
        armed_ = false;
        released_ = false;
        return Eigen::Vector3d::Zero();
      }
      if (!enabled_) {
        armed_ = false;
        released_ = neutral_;
        return Eigen::Vector3d::Zero();
      }
      if (!armed_ && released_) {
        armed_ = neutral_ && velocity_.norm() < 1e-6;
        released_ = false;
      }
      return armed_ ? velocity_ : Eigen::Vector3d::Zero();
    }

    double Age(double steady) const {
      std::lock_guard<std::mutex> lock(mutex_);
      return sequence_ ? std::max(0., steady - receipt_) : 1e9;
    }

    bool armed() const {
      std::lock_guard<std::mutex> lock(mutex_);
      return armed_;
    }
  private:
    bool Fresh_(double wall, double steady) const {
      return sequence_ && steady >= receipt_ && steady - receipt_ <= .2 && wall - source_wall_ <= .2 &&
             wall - source_wall_ >= -.05;
    }

    std::uint64_t boot_, session_ = 0, sequence_ = 0;
    double receipt_ = 0, source_wall_ = 0;
    bool connected_ = false, enabled_ = false, armed_ = false, released_ = false, neutral_ = false;
    Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
    mutable std::mutex mutex_;
};
} // namespace a3_pingpong
