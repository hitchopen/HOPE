#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace libmotioncapture {
namespace detail {

// One Cristian clock-synchronization exchange.  server_receive_seconds is
// Motive's high-resolution tick at NAT_ECHOREQUEST reception divided by the
// HighResClockFrequency advertised in NAT_SERVERINFO.  The local values are
// measured with the adapter host's monotonic clock.  NatNet exposes the server
// receive tick but no server transmit tick, so request/response asymmetry can
// bias offset() by up to RTT/2.  The mapping carries RTT/2 as uncertainty.
struct NatNetClockSample {
  double local_send_seconds;
  double local_receive_seconds;
  double server_receive_seconds;

  double rtt() const
  {
    return local_receive_seconds - local_send_seconds;
  }

  double offset() const
  {
    return (local_send_seconds + local_receive_seconds) * 0.5 -
           server_receive_seconds;
  }
};

// Maps Motive QPC ticks into the adapter host's monotonic clock.  Wall-clock
// conversion is intentionally left to the ROS layer, which combines the
// resulting age with RCL_SYSTEM_TIME/CLOCK_REALTIME after Chrony qualification.
class NatNetClockMapping {
public:
  NatNetClockMapping(
    std::size_t minimum_initial_samples,
    double accepted_rtt_slop_seconds,
    double drift_bound,
    double update_alpha,
    std::size_t rtt_regime_recovery_rejections)
    : minimum_initial_samples_(minimum_initial_samples)
    , accepted_rtt_slop_seconds_(accepted_rtt_slop_seconds)
    , drift_bound_(drift_bound)
    , update_alpha_(update_alpha)
    , rtt_regime_recovery_rejections_(
        std::max<std::size_t>(1, rtt_regime_recovery_rejections))
    , valid_(false)
    , offset_seconds_(0.0)
    , minimum_rtt_seconds_(std::numeric_limits<double>::infinity())
    , uncertainty_seconds_(std::numeric_limits<double>::infinity())
    , last_sync_local_seconds_(0.0)
    , higher_rtt_rejections_(0)
    , higher_rtt_candidate_{0.0, 0.0, 0.0}
    , last_update_rebased_rtt_floor_(false)
  {
  }

  bool initialize(const std::vector<NatNetClockSample>& samples)
  {
    std::vector<NatNetClockSample> valid_samples;
    valid_samples.reserve(samples.size());
    for (const auto& sample : samples) {
      if (std::isfinite(sample.local_send_seconds) &&
          std::isfinite(sample.local_receive_seconds) &&
          std::isfinite(sample.server_receive_seconds) &&
          sample.rtt() >= 0.0) {
        valid_samples.push_back(sample);
      }
    }
    if (valid_samples.size() < minimum_initial_samples_) {
      return false;
    }

    const auto best = std::min_element(
      valid_samples.begin(), valid_samples.end(),
      [](const NatNetClockSample& lhs, const NatNetClockSample& rhs) {
        return lhs.rtt() < rhs.rtt();
      });
    offset_seconds_ = best->offset();
    minimum_rtt_seconds_ = best->rtt();
    uncertainty_seconds_ = best->rtt() * 0.5;
    last_sync_local_seconds_ = best->local_receive_seconds;
    resetHigherRttCandidate();
    last_update_rebased_rtt_floor_ = false;
    valid_ = true;
    return true;
  }

  bool update(const NatNetClockSample& sample)
  {
    last_update_rebased_rtt_floor_ = false;
    if (!valid_ || !std::isfinite(sample.local_send_seconds) ||
        !std::isfinite(sample.local_receive_seconds) ||
        !std::isfinite(sample.server_receive_seconds) || sample.rtt() < 0.0) {
      resetHigherRttCandidate();
      return false;
    }

    const double rtt = sample.rtt();
    if (rtt <= minimum_rtt_seconds_ + accepted_rtt_slop_seconds_) {
      minimum_rtt_seconds_ = std::min(minimum_rtt_seconds_, rtt);
      resetHigherRttCandidate();
      acceptSample(sample);
      return true;
    }

    // A minimum RTT is normally a useful fixed lower bound, but a permanent
    // route/link change can establish a new, higher propagation floor.  Do
    // not let one congested echo move the floor.  After a complete run of
    // higher-RTT responses, rebase to the best (lowest RTT) sample in that new
    // regime.  Its RTT/2 and full pre-filter offset correction remain in the
    // uncertainty, so the ROS publication gate still fails closed if the new
    // path is too noisy or asymmetric.
    if (higher_rtt_rejections_ == 0 || rtt < higher_rtt_candidate_.rtt()) {
      higher_rtt_candidate_ = sample;
    }
    ++higher_rtt_rejections_;
    if (higher_rtt_rejections_ < rtt_regime_recovery_rejections_) {
      return false;
    }

    const NatNetClockSample recovery_sample = higher_rtt_candidate_;
    minimum_rtt_seconds_ = recovery_sample.rtt();
    resetHigherRttCandidate();
    acceptSample(recovery_sample);
    last_update_rebased_rtt_floor_ = true;
    return true;
  }

  bool lastUpdateRebasedRttFloor() const
  {
    return last_update_rebased_rtt_floor_;
  }

  double timestampAgeSeconds(
    uint64_t host_timestamp_ticks,
    uint64_t host_clock_frequency,
    double local_now_seconds) const
  {
    if (!valid_ || host_timestamp_ticks == 0 || host_clock_frequency == 0 ||
        !std::isfinite(local_now_seconds)) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const double capture_local_seconds =
      host_timestamp_ticks / static_cast<double>(host_clock_frequency) +
      offset_seconds_;
    return local_now_seconds - capture_local_seconds;
  }

  double uncertaintySeconds(double local_now_seconds) const
  {
    if (!valid_ || !std::isfinite(local_now_seconds)) {
      return std::numeric_limits<double>::infinity();
    }
    const double stale_seconds =
      std::max(0.0, local_now_seconds - last_sync_local_seconds_);
    return uncertainty_seconds_ + stale_seconds * drift_bound_;
  }

  bool valid() const { return valid_; }
  double minimumRttSeconds() const { return minimum_rtt_seconds_; }
  double baseUncertaintySeconds() const { return uncertainty_seconds_; }

private:
  void acceptSample(const NatNetClockSample& sample)
  {
    const double rtt = sample.rtt();
    const double correction = sample.offset() - offset_seconds_;
    offset_seconds_ += update_alpha_ * correction;
    // RTT/2 bounds the midpoint assumption.  The observed offset correction
    // is added until subsequent good samples show the mapping has settled.
    uncertainty_seconds_ = rtt * 0.5 + std::abs(correction);
    last_sync_local_seconds_ = sample.local_receive_seconds;
  }

  void resetHigherRttCandidate()
  {
    higher_rtt_rejections_ = 0;
    higher_rtt_candidate_ = {0.0, 0.0, 0.0};
  }

  std::size_t minimum_initial_samples_;
  double accepted_rtt_slop_seconds_;
  double drift_bound_;
  double update_alpha_;
  std::size_t rtt_regime_recovery_rejections_;
  bool valid_;
  double offset_seconds_;
  double minimum_rtt_seconds_;
  double uncertainty_seconds_;
  double last_sync_local_seconds_;
  std::size_t higher_rtt_rejections_;
  NatNetClockSample higher_rtt_candidate_;
  bool last_update_rebased_rtt_floor_;
};

}  // namespace detail
}  // namespace libmotioncapture
