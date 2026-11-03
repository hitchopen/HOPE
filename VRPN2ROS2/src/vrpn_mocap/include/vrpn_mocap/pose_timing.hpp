#ifndef VRPN_MOCAP__POSE_TIMING_HPP_
#define VRPN_MOCAP__POSE_TIMING_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>

namespace vrpn_mocap
{
// Raw server time is UNQUALIFIED. Differences below include clock offset and
// processing/transport delay; they must never be used as an exposure-age gate.
class PoseTiming
{
public:
  void Receive(std::int64_t sec, std::int64_t usec,
               std::int64_t receipt_ros_ns, std::int64_t steady_ns)
  {
    if (have_receipt_) {
      const double gap = (steady_ns - last_steady_) * 1e-9;
      max_receive_gap_ = std::max(max_receive_gap_, gap);
      receive_span_ += gap;
      ++receive_intervals_;
    }
    have_receipt_ = true;
    last_steady_ = steady_ns;
    receipt_ros_ = receipt_ros_ns;
    raw_sec_ = sec;
    raw_usec_ = usec;
    ++total_;
    ++count_;
    const bool valid = sec > 0 && sec <= 2147483647 && usec >= 0 && usec < 1000000;
    if (valid) {
      const std::int64_t source_us = sec * 1000000 + usec;
      delta_ms_ = (receipt_ros_ns / 1000 - source_us) * 1e-3;
      min_delta_ms_ = std::min(min_delta_ms_, delta_ms_);
      max_delta_ms_ = std::max(max_delta_ms_, delta_ms_);
      if (have_source_) {
        const double gap = (source_us - last_source_us_) * 1e-6;
        if (gap > 0) {
          max_source_gap_ = std::max(max_source_gap_, gap);
          source_span_ += gap;
          ++source_intervals_;
        } else {
          ++nonincreasing_;
        }
      }
      last_source_us_ = source_us;
      have_source_ = true;
    } else {
      ++invalid_;
      have_source_ = false;
      delta_ms_ = std::numeric_limits<double>::quiet_NaN();
    }
  }

  void Published(std::int64_t steady_ns)
  {
    max_publish_ms_ = std::max(max_publish_ms_, (steady_ns - last_steady_) * 1e-6);
  }

  std::array<double, 23> Snapshot(int sensor)
  {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    std::array<double, 23> result{{
      1, static_cast<double>(sensor), static_cast<double>(total_), static_cast<double>(count_),
      receive_span_, receive_span_ > 0 ? receive_intervals_ / receive_span_ : 0,
      static_cast<double>(raw_sec_), static_cast<double>(raw_usec_),
      static_cast<double>(receipt_ros_ / 1000000000), static_cast<double>(receipt_ros_ % 1000000000),
      last_steady_ * 1e-9, delta_ms_, count_ > invalid_ ? min_delta_ms_ : nan,
      count_ > invalid_ ? max_delta_ms_ : nan, max_receive_gap_ * 1000, max_source_gap_ * 1000,
      static_cast<double>(nonincreasing_), static_cast<double>(invalid_),
      static_cast<double>(source_intervals_), source_span_ > 0 ? source_intervals_ / source_span_ : 0,
      max_publish_ms_, 0, static_cast<double>(receive_intervals_)}};
    count_ = receive_intervals_ = source_intervals_ = nonincreasing_ = invalid_ = 0;
    receive_span_ = source_span_ = max_receive_gap_ = max_source_gap_ = max_publish_ms_ = 0;
    min_delta_ms_ = std::numeric_limits<double>::infinity();
    max_delta_ms_ = -std::numeric_limits<double>::infinity();
    return result;
  }

private:
  bool have_receipt_ = false, have_source_ = false;
  std::int64_t last_steady_ = 0, receipt_ros_ = 0, raw_sec_ = 0, raw_usec_ = 0, last_source_us_ = 0;
  std::uint64_t total_ = 0, count_ = 0, receive_intervals_ = 0, source_intervals_ = 0;
  std::uint64_t nonincreasing_ = 0, invalid_ = 0;
  double receive_span_ = 0, source_span_ = 0, max_receive_gap_ = 0, max_source_gap_ = 0;
  double delta_ms_ = 0, max_publish_ms_ = 0;
  double min_delta_ms_ = std::numeric_limits<double>::infinity();
  double max_delta_ms_ = -std::numeric_limits<double>::infinity();
};
}  // namespace vrpn_mocap
#endif
