#include "vrpn_mocap/vrpn_timestamp.h"

#include <cassert>
#include <cstdint>
#include <limits>

int main()
{
  using vrpn_mocap::detail::ValidateVrpnTimestamp;
  using vrpn_mocap::detail::VrpnTimestampStatus;

  constexpr int64_t now_ns = 1700000000100000000ll;
  constexpr int64_t max_age_ns = 100000000ll;
  constexpr int64_t max_future_ns = 5000000ll;

  const auto valid = ValidateVrpnTimestamp(
      1700000000ll, 50000ll, now_ns, max_age_ns, max_future_ns);
  assert(valid.ok());
  assert(valid.stamp_ns == 1700000000050000000ll);
  assert(valid.age_ns == 50000000ll);

  const auto small_future = ValidateVrpnTimestamp(
      1700000000ll, 102000ll, now_ns, max_age_ns, max_future_ns);
  assert(small_future.ok());
  assert(small_future.age_ns == -2000000ll);

  const auto too_old = ValidateVrpnTimestamp(
      1699999999ll, 900000ll, now_ns, max_age_ns, max_future_ns);
  assert(too_old.status == VrpnTimestampStatus::kTooOld);

  const auto too_future = ValidateVrpnTimestamp(
      1700000000ll, 106000ll, now_ns, max_age_ns, max_future_ns);
  assert(too_future.status == VrpnTimestampStatus::kTooFarInFuture);

  assert(ValidateVrpnTimestamp(
             -1, 0, now_ns, max_age_ns, max_future_ns)
             .status == VrpnTimestampStatus::kNegativeSeconds);
  assert(ValidateVrpnTimestamp(
             1700000000ll, 1000000ll, now_ns, max_age_ns, max_future_ns)
             .status == VrpnTimestampStatus::kMicrosecondsOutOfRange);
  assert(ValidateVrpnTimestamp(
             std::numeric_limits<int64_t>::max(), 0, now_ns,
             max_age_ns, max_future_ns)
             .status == VrpnTimestampStatus::kNanosecondsOverflow);

  return 0;
}
