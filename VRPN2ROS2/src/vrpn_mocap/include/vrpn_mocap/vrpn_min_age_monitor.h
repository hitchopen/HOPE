#ifndef VRPN_MOCAP__VRPN_MIN_AGE_MONITOR_H_
#define VRPN_MOCAP__VRPN_MIN_AGE_MONITOR_H_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <stdexcept>

namespace vrpn_mocap
{
namespace detail
{

struct VrpnMinAgeObservation
{
  bool ready{false};
  bool shift_exceeded{false};
  bool expected_error_exceeded{false};
  int64_t current_min_age_ns{0};
  int64_t reference_min_age_ns{0};
  int64_t shift_ns{0};
  int64_t expected_error_ns{0};

  bool acceptable() const
  {
    return !shift_exceeded && !expected_error_exceeded;
  }
};

// Tracks min(receipt_system_time - source_stamp) over a monotonic-time
// sliding window. The minimum is a one-way-delay-plus-clock-offset proxy: it
// detects changes, but it cannot separate delay from clock offset. An optional
// commissioned expected value is needed to detect an offset already present
// when this process starts.
class VrpnMinAgeMonitor
{
public:
  VrpnMinAgeMonitor(
      int64_t window_ns, std::size_t warmup_samples, int64_t max_shift_ns,
      bool has_expected_min_age, int64_t expected_min_age_ns,
      int64_t max_expected_error_ns)
      : window_ns_(window_ns),
        warmup_samples_(warmup_samples),
        max_shift_ns_(max_shift_ns),
        has_expected_min_age_(has_expected_min_age),
        expected_min_age_ns_(expected_min_age_ns),
        max_expected_error_ns_(max_expected_error_ns)
  {
    if (window_ns_ <= 0 || warmup_samples_ == 0 || max_shift_ns_ < 0 ||
        max_expected_error_ns_ < 0)
    {
      throw std::invalid_argument("invalid VRPN minimum-age monitor configuration");
    }
  }

  VrpnMinAgeObservation Observe(int64_t monotonic_now_ns, int64_t age_ns)
  {
    if (last_monotonic_ns_valid_ && monotonic_now_ns < last_monotonic_ns_)
    {
      throw std::invalid_argument("VRPN minimum-age monitor time moved backwards");
    }
    last_monotonic_ns_ = monotonic_now_ns;
    last_monotonic_ns_valid_ = true;

    const Sample sample{next_sequence_++, monotonic_now_ns, age_ns};
    samples_.push_back(sample);
    while (!minimum_candidates_.empty() &&
           minimum_candidates_.back().age_ns >= sample.age_ns)
    {
      minimum_candidates_.pop_back();
    }
    minimum_candidates_.push_back(sample);

    const int64_t oldest_allowed_ns = monotonic_now_ns - window_ns_;
    while (!samples_.empty() && samples_.front().monotonic_ns < oldest_allowed_ns)
    {
      const uint64_t expired_sequence = samples_.front().sequence;
      samples_.pop_front();
      if (!minimum_candidates_.empty() &&
          minimum_candidates_.front().sequence == expired_sequence)
      {
        minimum_candidates_.pop_front();
      }
    }

    VrpnMinAgeObservation result;
    result.current_min_age_ns = minimum_candidates_.front().age_ns;
    const bool window_observed =
        !samples_.empty() &&
        monotonic_now_ns - samples_.front().monotonic_ns >= window_ns_;
    if (!reference_valid_ && samples_.size() >= warmup_samples_ && window_observed)
    {
      reference_min_age_ns_ = result.current_min_age_ns;
      reference_valid_ = true;
    }

    result.ready = reference_valid_;
    if (reference_valid_)
    {
      result.reference_min_age_ns = reference_min_age_ns_;
      result.shift_ns = SaturatingSubtract(
          result.current_min_age_ns, reference_min_age_ns_);
      result.shift_exceeded =
          result.shift_ns > max_shift_ns_ || result.shift_ns < -max_shift_ns_;
    }

    if (has_expected_min_age_)
    {
      result.expected_error_ns = SaturatingSubtract(
          result.current_min_age_ns, expected_min_age_ns_);
      result.expected_error_exceeded =
          result.expected_error_ns > max_expected_error_ns_ ||
          result.expected_error_ns < -max_expected_error_ns_;
    }
    return result;
  }

private:
  static int64_t SaturatingSubtract(int64_t left, int64_t right)
  {
    if (right > 0 && left < std::numeric_limits<int64_t>::min() + right)
    {
      return std::numeric_limits<int64_t>::min();
    }
    if (right < 0 && left > std::numeric_limits<int64_t>::max() + right)
    {
      return std::numeric_limits<int64_t>::max();
    }
    return left - right;
  }

  struct Sample
  {
    uint64_t sequence;
    int64_t monotonic_ns;
    int64_t age_ns;
  };

  const int64_t window_ns_;
  const std::size_t warmup_samples_;
  const int64_t max_shift_ns_;
  const bool has_expected_min_age_;
  const int64_t expected_min_age_ns_;
  const int64_t max_expected_error_ns_;

  uint64_t next_sequence_{0};
  bool last_monotonic_ns_valid_{false};
  int64_t last_monotonic_ns_{0};
  bool reference_valid_{false};
  int64_t reference_min_age_ns_{0};
  std::deque<Sample> samples_;
  std::deque<Sample> minimum_candidates_;
};

}  // namespace detail
}  // namespace vrpn_mocap

#endif  // VRPN_MOCAP__VRPN_MIN_AGE_MONITOR_H_
