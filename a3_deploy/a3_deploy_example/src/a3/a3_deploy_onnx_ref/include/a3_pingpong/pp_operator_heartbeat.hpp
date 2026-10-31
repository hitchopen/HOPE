#pragma once

#include <array>
#include <mutex>
#include <vector>
#include "a3_pingpong/pp_runner_control.hpp"

namespace a3_pingpong {
// Read-only challenge/echo, never a motor command. Freshness belongs to the
// challenge's MDU monotonic issue time, not echo receipt or a remote wall clock.
class PpOperatorHeartbeat {
 public:
  PpOperatorHeartbeat(std::uint64_t boot, std::string_view session)
      : boot_(boot), session_(RunnerSessionFingerprint(session)) {}

  std::vector<double> Issue(double now) {
    if (!std::isfinite(now)) return {};
    std::uint64_t sequence;
    {
      std::lock_guard<std::mutex> lock(mu_);
      sequence = ++sequence_;
      issued_[sequence % issued_.size()] = {sequence, now};
    }
    return {1., static_cast<double>(boot_), static_cast<double>(session_),
            static_cast<double>(sequence)};
  }

  bool Echo(const std::vector<double>& wire, double now) {
    if (wire.size() != 5 || wire[0] != 1. || !std::isfinite(now)) return false;
    std::uint64_t boot = 0, session = 0, sequence = 0, role = 0;
    if (!IsExactFloatInteger(wire[1], 1, kRunnerMaxExactFloatInteger, &boot) ||
        !IsExactFloatInteger(wire[2], 1, kRunnerMaxExactFloatInteger, &session) ||
        !IsExactFloatInteger(wire[3], 1, kRunnerMaxExactFloatInteger, &sequence) ||
        !IsExactFloatInteger(wire[4], 1, 2, &role) || boot != boot_ || session != session_)
      return false;
    std::lock_guard<std::mutex> lock(mu_);
    const auto& challenge = issued_[sequence % issued_.size()];
    const double age = now - challenge.when;
    auto& seen = echoes_[role - 1];
    if (challenge.sequence != sequence || !std::isfinite(age) || age < 0. ||
        age > kDeadlineS[role - 1] ||
        sequence <= seen.sequence) return false;
    seen = challenge;  // NEVER renew to 'now'; delayed/buffered echoes cannot buy extra time.
    return true;
  }

  std::array<bool, 2> Fresh(double now) const {
    std::lock_guard<std::mutex> lock(mu_);
    std::array<bool, 2> result{};
    for (std::size_t i = 0; i < result.size(); ++i) {
      const double age = now - echoes_[i].when;
      result[i] = echoes_[i].sequence != 0 && std::isfinite(age) && age >= 0. &&
                  age <= kDeadlineS[i];
    }
    return result;
  }
 private:
  struct Receipt { std::uint64_t sequence = 0; double when = -1.; };
  // Console/UI has two seconds; the operator-computer link has one second.
  // Both deadlines are measured from challenge issue time,
  // never echo receipt time. A late echo cannot renew the original deadline.
  static constexpr std::array<double, 2> kDeadlineS{2.0, 1.0};
  const std::uint64_t boot_, session_;
  mutable std::mutex mu_;
  std::uint64_t sequence_ = 0;
  // Cover the full 2 s UI window at 50 Hz, with scheduling headroom.
  std::array<Receipt, 128> issued_{};
  std::array<Receipt, 2> echoes_{};  // role 1: Console; role 2: configured operator relay
};
}  // namespace a3_pingpong
