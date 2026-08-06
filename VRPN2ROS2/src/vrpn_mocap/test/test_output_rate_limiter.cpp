#include "vrpn_mocap/output_rate_limiter.h"

#include <cassert>
#include <cstddef>
#include <limits>
#include <stdexcept>

using vrpn_mocap::detail::OutputRateLimiter;

namespace
{

void TestUnlimitedRatePublishesEveryReport()
{
  OutputRateLimiter limiter(0.0);
  assert(limiter.ShouldPublish(0.0));
  assert(limiter.ShouldPublish(0.001));
  assert(limiter.ShouldPublish(0.001));
}

void TestDefaultRateFromPollingRate()
{
  OutputRateLimiter limiter(200.0);
  std::size_t published = 0;
  for (std::size_t i = 0; i < 500; ++i)
  {
    if (limiter.ShouldPublish(static_cast<double>(i) / 500.0))
    {
      ++published;
    }
  }
  assert(published == 200);
}

void TestNonIntegerSourceRatio()
{
  OutputRateLimiter limiter(200.0);
  std::size_t published = 0;
  for (std::size_t i = 0; i < 3500; ++i)
  {
    if (limiter.ShouldPublish(static_cast<double>(i) / 350.0))
    {
      ++published;
    }
  }
  assert(published >= 1999);
  assert(published <= 2001);
}

void TestNoCatchUpBurstAfterDelay()
{
  OutputRateLimiter limiter(200.0);
  assert(limiter.ShouldPublish(1.0));
  assert(!limiter.ShouldPublish(1.001));
  assert(limiter.ShouldPublish(2.0));
  assert(!limiter.ShouldPublish(2.0));
  assert(!limiter.ShouldPublish(2.001));
  assert(limiter.ShouldPublish(2.005));
}

void TestBackwardsClockResetPublishesImmediately()
{
  OutputRateLimiter limiter(200.0);
  assert(limiter.ShouldPublish(10.0));
  assert(!limiter.ShouldPublish(10.001));
  assert(limiter.ShouldPublish(9.0));
  assert(!limiter.ShouldPublish(9.001));
}

void TestInvalidInputsAreRejected()
{
  bool threw = false;
  try
  {
    OutputRateLimiter limiter(-1.0);
  }
  catch (const std::invalid_argument &)
  {
    threw = true;
  }
  assert(threw);

  threw = false;
  try
  {
    OutputRateLimiter limiter(std::numeric_limits<double>::infinity());
  }
  catch (const std::invalid_argument &)
  {
    threw = true;
  }
  assert(threw);

  OutputRateLimiter limiter(200.0);
  threw = false;
  try
  {
    limiter.ShouldPublish(std::numeric_limits<double>::quiet_NaN());
  }
  catch (const std::invalid_argument &)
  {
    threw = true;
  }
  assert(threw);
}

}  // namespace

int main()
{
  TestUnlimitedRatePublishesEveryReport();
  TestDefaultRateFromPollingRate();
  TestNonIntegerSourceRatio();
  TestNoCatchUpBurstAfterDelay();
  TestBackwardsClockResetPublishesImmediately();
  TestInvalidInputsAreRejected();
  return 0;
}
