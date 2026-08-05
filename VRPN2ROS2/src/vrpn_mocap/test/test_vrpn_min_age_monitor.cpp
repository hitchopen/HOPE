#include "vrpn_mocap/vrpn_min_age_monitor.h"

#include <cassert>
#include <cstdint>
#include <limits>

namespace
{
constexpr int64_t kMillisecond = 1000000ll;
}

int main()
{
  using vrpn_mocap::detail::VrpnMinAgeMonitor;

  // Establish a 10 ms baseline after the window and sample warmup are both
  // populated. Waiting for a full window avoids learning a startup scheduling
  // spike as the permanent reference.
  VrpnMinAgeMonitor monitor(
      5 * kMillisecond, 3, 5 * kMillisecond, false, 0, 0);
  assert(!monitor.Observe(0, 12 * kMillisecond).ready);
  assert(!monitor.Observe(1 * kMillisecond, 10 * kMillisecond).ready);
  assert(!monitor.Observe(2 * kMillisecond, 11 * kMillisecond).ready);
  auto observation = monitor.Observe(5 * kMillisecond, 20 * kMillisecond);
  assert(observation.ready);
  assert(observation.reference_min_age_ns == 10 * kMillisecond);
  assert(!observation.shift_exceeded);

  // Higher-latency samples do not change the proxy while the old minimum is
  // still inside the sliding window.
  observation = monitor.Observe(6 * kMillisecond, 20 * kMillisecond);
  assert(observation.current_min_age_ns == 10 * kMillisecond);
  assert(!observation.shift_exceeded);

  // Once the old minimum leaves the window, a sustained +10 ms regime shift
  // is detected.
  observation = monitor.Observe(8 * kMillisecond, 21 * kMillisecond);
  assert(observation.current_min_age_ns == 20 * kMillisecond);
  assert(observation.shift_ns == 10 * kMillisecond);
  assert(observation.shift_exceeded);

  // Returning to the original regime clears the alarm automatically.
  observation = monitor.Observe(9 * kMillisecond, 10 * kMillisecond);
  assert(!observation.shift_exceeded);

  // A lower minimum (server clock moves ahead or latency falls) is detected
  // immediately rather than waiting for the window to expire.
  observation = monitor.Observe(10 * kMillisecond, 3 * kMillisecond);
  assert(observation.shift_ns == -7 * kMillisecond);
  assert(observation.shift_exceeded);

  // A commissioned expected minimum detects a static offset present from
  // startup; the self-reference alone fundamentally cannot do that.
  VrpnMinAgeMonitor commissioned(
      5 * kMillisecond, 2, 5 * kMillisecond, true,
      10 * kMillisecond, 3 * kMillisecond);
  observation = commissioned.Observe(0, 40 * kMillisecond);
  assert(!observation.ready);
  assert(observation.expected_error_exceeded);
  observation = commissioned.Observe(1 * kMillisecond, 39 * kMillisecond);
  assert(!observation.ready);
  assert(observation.expected_error_exceeded);
  assert(!observation.acceptable());

  VrpnMinAgeMonitor commissioned_good(
      5 * kMillisecond, 2, 5 * kMillisecond, true,
      10 * kMillisecond, 3 * kMillisecond);
  commissioned_good.Observe(0, 12 * kMillisecond);
  observation = commissioned_good.Observe(1 * kMillisecond, 10 * kMillisecond);
  assert(!observation.ready);
  assert(!observation.expected_error_exceeded);
  assert(observation.acceptable());

  // Disabled absolute validation may expose the monitor to extreme but
  // structurally representable ages. Difference reporting must saturate, not
  // invoke signed-integer overflow.
  VrpnMinAgeMonitor extreme(
      1, 1, 1, true, std::numeric_limits<int64_t>::min(), 1);
  extreme.Observe(0, std::numeric_limits<int64_t>::min());
  observation = extreme.Observe(2, std::numeric_limits<int64_t>::max());
  assert(observation.expected_error_ns == std::numeric_limits<int64_t>::max());
  assert(observation.expected_error_exceeded);

  return 0;
}
