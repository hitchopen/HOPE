#pragma once

#include <cmath>
#include <stdexcept>

namespace motion_capture_tracking
{
namespace detail
{

// Selects at most one source frame per output period without changing the
// frame's acquisition timestamp. A zero rate disables limiting.
class OutputRateLimiter
{
public:
  explicit OutputRateLimiter(double output_rate_hz)
  : unlimited_(output_rate_hz == 0.0), period_seconds_(0.0)
  {
    if (!std::isfinite(output_rate_hz) || output_rate_hz < 0.0) {
      throw std::invalid_argument("output rate must be finite and non-negative");
    }
    if (!unlimited_) {
      period_seconds_ = 1.0 / output_rate_hz;
    }
  }

  bool shouldPublish(double monotonic_now_seconds)
  {
    if (!std::isfinite(monotonic_now_seconds)) {
      throw std::invalid_argument("monotonic time must be finite");
    }
    if (unlimited_) {
      return true;
    }

    // Publish the first valid frame immediately. A backwards jump should not
    // happen for steady_clock, but resetting here makes the limiter fail open
    // rather than suppressing output indefinitely if a synthetic/test clock
    // is reset.
    if (!initialized_ || monotonic_now_seconds < last_now_seconds_) {
      initialized_ = true;
      last_now_seconds_ = monotonic_now_seconds;
      next_output_seconds_ = monotonic_now_seconds + period_seconds_;
      return true;
    }
    last_now_seconds_ = monotonic_now_seconds;

    const double tolerance_seconds = period_seconds_ * 1e-9;
    if (monotonic_now_seconds + tolerance_seconds < next_output_seconds_) {
      return false;
    }

    // Keep the original phase while skipping missed periods. This prevents a
    // delayed callback from causing either a catch-up burst or long-term rate
    // drift. Only the freshest source frame at/after this deadline is emitted.
    double elapsed_periods =
      std::floor((monotonic_now_seconds - next_output_seconds_) /
      period_seconds_) + 1.0;
    // The tolerance above may admit a value a few ULPs before the exact
    // deadline. It still consumes that deadline, so always advance at least
    // one period.
    if (elapsed_periods < 1.0) {
      elapsed_periods = 1.0;
    }
    next_output_seconds_ += elapsed_periods * period_seconds_;
    if (!std::isfinite(next_output_seconds_) ||
      next_output_seconds_ <= monotonic_now_seconds)
    {
      next_output_seconds_ = monotonic_now_seconds + period_seconds_;
    }
    return true;
  }

private:
  bool unlimited_;
  double period_seconds_;
  bool initialized_{false};
  double last_now_seconds_{0.0};
  double next_output_seconds_{0.0};
};

}  // namespace detail
}  // namespace motion_capture_tracking
