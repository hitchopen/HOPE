#ifndef VRPN_MOCAP__OUTPUT_RATE_LIMITER_H_
#define VRPN_MOCAP__OUTPUT_RATE_LIMITER_H_

#include <cmath>
#include <stdexcept>

namespace vrpn_mocap
{
namespace detail
{

// Selects at most one accepted source report per output period. A zero rate
// disables limiting. Callers retain the selected report's source timestamp.
class OutputRateLimiter
{
public:
  explicit OutputRateLimiter(double output_rate_hz)
  : unlimited_(output_rate_hz == 0.0), period_seconds_(0.0)
  {
    if (!std::isfinite(output_rate_hz) || output_rate_hz < 0.0)
    {
      throw std::invalid_argument("output rate must be finite and non-negative");
    }
    if (!unlimited_)
    {
      period_seconds_ = 1.0 / output_rate_hz;
    }
  }

  bool ShouldPublish(double monotonic_now_seconds)
  {
    if (!std::isfinite(monotonic_now_seconds))
    {
      throw std::invalid_argument("monotonic time must be finite");
    }
    if (unlimited_)
    {
      return true;
    }

    if (!initialized_ || monotonic_now_seconds < last_now_seconds_)
    {
      initialized_ = true;
      last_now_seconds_ = monotonic_now_seconds;
      next_output_seconds_ = monotonic_now_seconds + period_seconds_;
      return true;
    }
    last_now_seconds_ = monotonic_now_seconds;

    const double tolerance_seconds = period_seconds_ * 1e-9;
    if (monotonic_now_seconds + tolerance_seconds < next_output_seconds_)
    {
      return false;
    }

    // Preserve the original phase while skipping missed periods. A delayed
    // callback never creates a catch-up publication burst.
    double elapsed_periods =
      std::floor((monotonic_now_seconds - next_output_seconds_) /
      period_seconds_) + 1.0;
    if (elapsed_periods < 1.0)
    {
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
}  // namespace vrpn_mocap

#endif  // VRPN_MOCAP__OUTPUT_RATE_LIMITER_H_
