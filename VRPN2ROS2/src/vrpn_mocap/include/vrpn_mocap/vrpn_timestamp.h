#ifndef VRPN_MOCAP__VRPN_TIMESTAMP_H_
#define VRPN_MOCAP__VRPN_TIMESTAMP_H_

#include <cstdint>
#include <limits>

namespace vrpn_mocap
{
namespace detail
{

enum class VrpnTimestampStatus
{
  kOk,
  kNegativeSeconds,
  kMicrosecondsOutOfRange,
  kNanosecondsOverflow,
  kTooOld,
  kTooFarInFuture,
};

struct VrpnTimestampValidation
{
  VrpnTimestampStatus status{VrpnTimestampStatus::kNegativeSeconds};
  int64_t stamp_ns{0};
  // Positive means the sample is in the past; negative means it is in the
  // future relative to the adapter host's NTP-disciplined system clock.
  int64_t age_ns{0};

  bool ok() const { return status == VrpnTimestampStatus::kOk; }
};

inline const char * VrpnTimestampStatusName(VrpnTimestampStatus status)
{
  switch (status)
  {
    case VrpnTimestampStatus::kOk:
      return "ok";
    case VrpnTimestampStatus::kNegativeSeconds:
      return "negative_seconds";
    case VrpnTimestampStatus::kMicrosecondsOutOfRange:
      return "microseconds_out_of_range";
    case VrpnTimestampStatus::kNanosecondsOverflow:
      return "nanoseconds_overflow";
    case VrpnTimestampStatus::kTooOld:
      return "too_old";
    case VrpnTimestampStatus::kTooFarInFuture:
      return "too_far_in_future";
  }
  return "unknown";
}

inline VrpnTimestampValidation ValidateVrpnTimestamp(
    int64_t seconds, int64_t microseconds, int64_t local_system_now_ns,
    int64_t max_age_ns, int64_t max_future_skew_ns)
{
  VrpnTimestampValidation result;
  if (seconds < 0)
  {
    result.status = VrpnTimestampStatus::kNegativeSeconds;
    return result;
  }
  if (microseconds < 0 || microseconds >= 1000000)
  {
    result.status = VrpnTimestampStatus::kMicrosecondsOutOfRange;
    return result;
  }

  constexpr int64_t kNanosecondsPerSecond = 1000000000ll;
  constexpr int64_t kNanosecondsPerMicrosecond = 1000ll;
  const int64_t fractional_ns = microseconds * kNanosecondsPerMicrosecond;
  if (seconds >
      (std::numeric_limits<int64_t>::max() - fractional_ns) /
          kNanosecondsPerSecond)
  {
    result.status = VrpnTimestampStatus::kNanosecondsOverflow;
    return result;
  }

  result.stamp_ns = seconds * kNanosecondsPerSecond + fractional_ns;
  if (result.stamp_ns <= local_system_now_ns)
  {
    result.age_ns = local_system_now_ns - result.stamp_ns;
    result.status = result.age_ns <= max_age_ns
                        ? VrpnTimestampStatus::kOk
                        : VrpnTimestampStatus::kTooOld;
    return result;
  }

  const int64_t future_ns = result.stamp_ns - local_system_now_ns;
  result.age_ns = -future_ns;
  result.status = future_ns <= max_future_skew_ns
                      ? VrpnTimestampStatus::kOk
                      : VrpnTimestampStatus::kTooFarInFuture;
  return result;
}

}  // namespace detail
}  // namespace vrpn_mocap

#endif  // VRPN_MOCAP__VRPN_TIMESTAMP_H_
