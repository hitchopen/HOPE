#pragma once
#include <cstdint>

namespace a3_pingpong {
// Driver-thread only. Modes start immediately, then keep a full native period.
class PpModeCadence {
  public:
    bool Due(std::uint64_t tick, int mode, unsigned period) {
      if (!valid_ || mode != mode_ || tick - last_tick_ >= period) {
        valid_ = true;
        mode_ = mode;
        last_tick_ = tick;
        return true;
      }
      return false;
    }

    // Monotonic time cadence: callback overruns must not slow the CSV clock.
    // Emit at most one frame per call; a full missed period rebases the clock
    // rather than skipping motion frames or bursting through them.
    bool DueTime(std::int64_t now_ns, int mode, std::int64_t period_ns) {
      if (!time_valid_ || mode != time_mode_ || period_ns != period_ns_) {
        time_valid_ = true;
        time_mode_ = mode;
        period_ns_ = period_ns;
        next_ns_ = now_ns + period_ns;
        return true;
      }
      if (now_ns < next_ns_) return false;
      if (now_ns - next_ns_ >= period_ns) next_ns_ = now_ns + period_ns;
      else next_ns_ += period_ns;
      return true;
    }

    void Reset() { valid_ = false; time_valid_ = false; }
  private:
    bool valid_ = false;
    int mode_ = 0;
    std::uint64_t last_tick_ = 0;
    bool time_valid_ = false;
    int time_mode_ = 0;
    std::int64_t next_ns_ = 0, period_ns_ = 0;
};
} // namespace a3_pingpong
