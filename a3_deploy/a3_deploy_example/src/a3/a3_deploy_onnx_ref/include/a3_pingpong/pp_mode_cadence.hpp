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

    void Reset() { valid_ = false; }
  private:
    bool valid_ = false;
    int mode_ = 0;
    std::uint64_t last_tick_ = 0;
};
} // namespace a3_pingpong
