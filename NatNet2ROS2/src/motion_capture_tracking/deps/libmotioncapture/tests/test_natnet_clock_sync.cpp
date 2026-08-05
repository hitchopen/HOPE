#include "libmotioncapture/natnet_clock_sync.h"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <vector>

namespace {

bool near(double lhs, double rhs, double tolerance = 1e-9)
{
  return std::abs(lhs - rhs) <= tolerance;
}

}  // namespace

int main()
{
  using libmotioncapture::detail::NatNetClockMapping;
  using libmotioncapture::detail::NatNetClockSample;

  NatNetClockMapping mapping(
    5,        // minimum initial samples
    0.00025,  // accepted RTT slop
    100e-6,   // drift uncertainty per stale second
    0.2,      // runtime correction gain
    10);      // higher-RTT samples before regime recovery

  // Every sample represents the same +100.001 s Motive->adapter offset.
  // initialize() must choose the 2 ms minimum-RTT observation.
  const std::vector<NatNetClockSample> initial{
    {109.999, 110.003, 10.000},
    {120.000, 120.002, 20.000},
    {130.000, 130.004, 30.001},
    {140.000, 140.006, 40.002},
    {150.000, 150.008, 50.003},
  };
  assert(mapping.initialize(initial));
  assert(mapping.valid());
  assert(near(mapping.minimumRttSeconds(), 0.002));
  assert(near(mapping.baseUncertaintySeconds(), 0.001));

  constexpr uint64_t frequency = 1000000000ULL;
  constexpr uint64_t capture_ticks = 25000000000ULL;
  assert(near(
    mapping.timestampAgeSeconds(capture_ticks, frequency, 125.021),
    0.020));

  // A low-RTT runtime sample reports a 1 ms offset correction.  The 0.2 gain
  // applies 0.2 ms and the uncertainty temporarily includes that correction.
  assert(mapping.update({130.001, 130.003, 30.000}));
  assert(near(mapping.baseUncertaintySeconds(), 0.002));
  assert(near(
    mapping.timestampAgeSeconds(capture_ticks, frequency, 125.021),
    0.0198));

  // A congested 10 ms exchange is rejected and cannot jump the mapping.
  assert(!mapping.update({140.000, 140.010, 40.000}));
  assert(near(
    mapping.timestampAgeSeconds(capture_ticks, frequency, 125.021),
    0.0198));

  // Mapping uncertainty grows when echo refresh stops.
  assert(near(mapping.uncertaintySeconds(140.003), 0.003));

  NatNetClockMapping insufficient(5, 0.00025, 100e-6, 0.2, 10);
  assert(!insufficient.initialize({initial[0], initial[1]}));
  assert(!insufficient.valid());
  assert(std::isnan(
    insufficient.timestampAgeSeconds(capture_ticks, frequency, 125.021)));

  // A permanent RTT floor change must recover without a process restart.
  // Use a short three-response threshold in this unit test. All higher-RTT
  // samples retain the original +100.001 s offset, so the recovery itself
  // does not introduce an artificial correction.
  NatNetClockMapping recovering(5, 0.00025, 100e-6, 0.2, 3);
  assert(recovering.initialize(initial));
  assert(!recovering.update({159.999, 160.003, 60.000}));
  assert(!recovering.update({169.999, 170.003, 70.000}));
  assert(!recovering.lastUpdateRebasedRttFloor());
  assert(recovering.update({179.999, 180.003, 80.000}));
  assert(recovering.lastUpdateRebasedRttFloor());
  assert(near(recovering.minimumRttSeconds(), 0.004));
  assert(near(recovering.baseUncertaintySeconds(), 0.002));

  // The new floor remains usable, and a later return to the original faster
  // path immediately lowers it again.
  assert(recovering.update({189.999, 190.003, 90.000}));
  assert(!recovering.lastUpdateRebasedRttFloor());
  assert(recovering.update({200.000, 200.002, 100.000}));
  assert(near(recovering.minimumRttSeconds(), 0.002));

  // A good low-RTT sample interrupts and resets a partial recovery streak.
  NatNetClockMapping interrupted(5, 0.00025, 100e-6, 0.2, 3);
  assert(interrupted.initialize(initial));
  assert(!interrupted.update({159.999, 160.003, 60.000}));
  assert(!interrupted.update({169.999, 170.003, 70.000}));
  assert(interrupted.update({180.000, 180.002, 80.000}));
  assert(!interrupted.update({189.999, 190.003, 90.000}));
  assert(!interrupted.update({199.999, 200.003, 100.000}));
  assert(!interrupted.lastUpdateRebasedRttFloor());
  assert(near(interrupted.minimumRttSeconds(), 0.002));

  // Exercise the production ten-echo threshold explicitly: nine persistent
  // high-RTT replies cannot move the floor; the tenth can.
  NatNetClockMapping production_threshold(5, 0.00025, 100e-6, 0.2, 10);
  assert(production_threshold.initialize(initial));
  for (int index = 0; index < 9; ++index) {
    const double server_time = 110.0 + index * 10.0;
    assert(!production_threshold.update(
      {server_time + 99.999, server_time + 100.003, server_time}));
  }
  assert(!production_threshold.lastUpdateRebasedRttFloor());
  assert(production_threshold.update({299.999, 300.003, 200.000}));
  assert(production_threshold.lastUpdateRebasedRttFloor());
  assert(near(production_threshold.minimumRttSeconds(), 0.004));

  // Rebaselining cannot hide a simultaneous offset jump/asymmetry: the full
  // correction remains in uncertainty even though the offset filter applies
  // only its configured alpha.
  NatNetClockMapping biased_recovery(5, 0.00025, 100e-6, 0.2, 1);
  assert(biased_recovery.initialize(initial));
  assert(biased_recovery.update({160.004, 160.008, 60.000}));
  assert(biased_recovery.lastUpdateRebasedRttFloor());
  assert(near(biased_recovery.baseUncertaintySeconds(), 0.007));

  return 0;
}
