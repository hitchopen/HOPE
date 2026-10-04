#pragma once
#include <cstdint>

namespace a3_pingpong {
// Poll on EVERY driver callback, including Stand/Passive. A button request
// must never choose the 100 Hz phase. This clock belongs to the driver thread
// and survives mode changes and repeated serves.
class PpServeCadence {
 public:
  static constexpr std::int64_t kPeriodNs = 10'000'000;
  bool Poll(std::int64_t scheduled_ns) {
    if (!initialized_) {
      initialized_ = true;
      next_ns_ = scheduled_ns;
    }
    if (scheduled_ns < next_ns_) return false;
    // HAL telemetry must never pause, cancel, or reset CSV playback. If the
    // next row is due before a receipt arrives, keep the existing CSV clock.
    if (pending_publication_ns_) {
      pending_publication_ns_ = 0;
      selection_missed_ = true;
    }
    const auto overdue = scheduled_ns - next_ns_;
    last_ns_ = scheduled_ns;
    next_ns_ += (overdue / kPeriodNs + 1) * kPeriodNs;
    // A real missed frame stretches time; never skip a CSV row or catch up
    // with a short interval. Retain the original phase even after a stall.
    if (overdue >= kPeriodNs && next_ns_ < scheduled_ns + kPeriodNs)
      next_ns_ += kPeriodNs;
    return true;
  }

  void NotePublication(std::int64_t publication_ns) {
    if (!initialized_) return;
    // Normal synchronous publication takes tens of us. Rebasing by that
    // duration rounds up to another 2 ms driver tick and changes every serve.
    // Only an abnormal >1 ms dispatch delays the next frame, in whole periods.
    if (publication_ns <= last_ns_ + 1'000'000) return;
    const auto earliest = publication_ns + kPeriodNs;
    if (next_ns_ < earliest)
      next_ns_ += ((earliest - next_ns_ + kPeriodNs - 1) / kPeriodNs) * kPeriodNs;
  }

  // Observe transport latency only. HAL selection precedes device execution
  // and ball detachment; it is not an epoch for retiming the CSV.
  void TrackHalSelection(std::int64_t publication_ns) {
    pending_publication_ns_ = publication_ns;
  }
  bool tracking_hal() const { return pending_publication_ns_ != 0; }
  enum class Receipt { kWaiting, kSelected };
  Receipt ObserveHalSelection(std::int64_t now_ns, std::int64_t edge_ns) {
    if (!pending_publication_ns_) return Receipt::kWaiting;
    if (edge_ns >= pending_publication_ns_ && edge_ns <= now_ns) {
      pending_publication_ns_ = 0;
      return Receipt::kSelected;
    }
    return Receipt::kWaiting;
  }
  bool ConsumeMissedSelection() {
    const bool missed = selection_missed_;
    selection_missed_ = false;
    return missed;
  }
  void CancelHalTracking(std::int64_t /*now_ns*/) {
    pending_publication_ns_ = 0;
  }

 private:
  bool initialized_{false};
  std::int64_t last_ns_{0}, next_ns_{0};
  std::int64_t pending_publication_ns_{0};
  bool selection_missed_{false};
};

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
    // Keep a complete frame period after the synchronous release publication,
    // including a delayed native publish. Never catch up by compressing swing.
    void RebaseAfter(std::int64_t publication_ns, int mode, std::int64_t period_ns) {
      time_valid_ = true;
      time_mode_ = mode;
      period_ns_ = period_ns;
      next_ns_ = publication_ns + period_ns;
    }
  private:
    bool valid_ = false;
    int mode_ = 0;
    std::uint64_t last_tick_ = 0;
    bool time_valid_ = false;
    int time_mode_ = 0;
    std::int64_t next_ns_ = 0, period_ns_ = 0;
};
} // namespace a3_pingpong
