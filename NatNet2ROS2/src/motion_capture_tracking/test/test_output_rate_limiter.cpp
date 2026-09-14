#include "motion_capture_tracking/output_rate_limiter.h"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

using motion_capture_tracking::detail::OutputRateLimiter;

namespace
{

void testUnlimitedRatePublishesEveryFrame()
{
  OutputRateLimiter limiter(0.0);
  assert(limiter.shouldPublish(0.0));
  assert(limiter.shouldPublish(0.001));
  assert(limiter.shouldPublish(0.001));
}

void testExactIntegerDownsampling()
{
  OutputRateLimiter limiter(120.0);
  size_t published = 0;
  for (size_t i = 0; i < 360; ++i) {
    if (limiter.shouldPublish(static_cast<double>(i) / 360.0)) {
      ++published;
    }
  }
  assert(published == 120);
}

void testNonIntegerDownsamplingKeepsAverageRate()
{
  OutputRateLimiter limiter(120.0);
  size_t published = 0;
  for (size_t i = 0; i < 3000; ++i) {
    if (limiter.shouldPublish(static_cast<double>(i) / 300.0)) {
      ++published;
    }
  }
  assert(published >= 1199);
  assert(published <= 1201);
}

void testCompetitionProfileDownsamples300To200Hz()
{
  OutputRateLimiter limiter(200.0);
  size_t published = 0;
  for (size_t i = 0; i < 3000; ++i) {
    if (limiter.shouldPublish(static_cast<double>(i) / 300.0)) {
      ++published;
    }
  }
  assert(published >= 1999);
  assert(published <= 2001);
}

void testNoCatchUpBurstAfterDelay()
{
  OutputRateLimiter limiter(100.0);
  assert(limiter.shouldPublish(1.0));
  assert(!limiter.shouldPublish(1.001));
  assert(limiter.shouldPublish(2.0));
  assert(!limiter.shouldPublish(2.0));
  assert(!limiter.shouldPublish(2.001));
  assert(limiter.shouldPublish(2.01));
}

void testBackwardsClockResetPublishesImmediately()
{
  OutputRateLimiter limiter(120.0);
  assert(limiter.shouldPublish(10.0));
  assert(!limiter.shouldPublish(10.001));
  assert(limiter.shouldPublish(9.0));
  assert(!limiter.shouldPublish(9.001));
}

void testInvalidInputsAreRejected()
{
  bool threw = false;
  try {
    OutputRateLimiter limiter(-1.0);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  assert(threw);

  threw = false;
  try {
    OutputRateLimiter limiter(std::numeric_limits<double>::infinity());
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  assert(threw);

  OutputRateLimiter limiter(120.0);
  threw = false;
  try {
    limiter.shouldPublish(std::numeric_limits<double>::quiet_NaN());
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  assert(threw);
}

}  // namespace

int main()
{
  testUnlimitedRatePublishesEveryFrame();
  testExactIntegerDownsampling();
  testNonIntegerDownsamplingKeepsAverageRate();
  testCompetitionProfileDownsamples300To200Hz();
  testNoCatchUpBurstAfterDelay();
  testBackwardsClockResetPublishesImmediately();
  testInvalidInputsAreRejected();
  return 0;
}
