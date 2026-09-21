// Explicit offline/reference ABI for hitter_execution_history_502_v1.
// Caller supplies action-order joints and the final delivered command. This
// helper does not admit a new policy, generate commands or access hardware.
#pragma once
#include <array>
#include <algorithm>
#include <cstdint>

namespace a3_pingpong {
class ExecutionHistory502 {
 public:
  using Joints = std::array<double, 31>;
  using Prefix = std::array<double, 112>;
  using Observation = std::array<double, 502>;
  void Reset() { pending_ = true; }
  Observation Build(const Prefix& prefix, const Joints& raw_action,
                    const Joints& q_des_sent, const Joints& q_actual,
                    std::int64_t tick) {
    std::array<double, 99> frame{};
    std::copy(q_actual.begin(), q_actual.end(), frame.begin());
    std::copy(prefix.begin() + 34, prefix.begin() + 65, frame.begin() + 31);
    std::copy(prefix.begin() + 96, prefix.begin() + 99, frame.begin() + 62);
    std::copy(prefix.begin(), prefix.begin() + 3, frame.begin() + 65);
    std::copy(q_des_sent.begin(), q_des_sent.end(), frame.begin() + 68);
    if (pending_) {
      frames_.fill(frame);
      pending_ = false;
    } else if (tick != last_tick_) {
      for (int i = 3; i > 0; --i) frames_[i] = frames_[i - 1];
      frames_[0] = frame;
    }
    last_tick_ = tick;
    Observation output{};
    std::copy(prefix.begin(), prefix.end(), output.begin());
    std::copy(raw_action.begin(), raw_action.end(), output.begin() + 112);
    std::copy(q_des_sent.begin(), q_des_sent.end(), output.begin() + 143);
    for (int i = 0; i < 31; ++i) output[174 + i] = q_des_sent[i] - q_actual[i];
    for (int i = 1; i < 4; ++i)
      std::copy(frames_[i].begin(), frames_[i].end(), output.begin() + 205 + 99 * (i - 1));
    return output;
  }
 private:
  bool pending_ = true;
  std::int64_t last_tick_ = -1;
  std::array<std::array<double, 99>, 4> frames_{};
};

// Separate candidate ABI. The 502-D interpretation above is unchanged.
class CompactExecutionHistory324 {
 public:
  using Joints = std::array<double, 31>;
  using Prefix = std::array<double, 112>;
  using Observation = std::array<double, 324>;
  void Reset() { pending_ = true; }
  Observation Build(const Prefix& prefix, const Joints& q_des_sent,
                    const Joints& q_actual, std::int64_t tick) {
    std::array<double, 70> frame{};
    std::copy(prefix.begin() + 34, prefix.begin() + 65, frame.begin());
    for (int i = 0; i < 31; ++i) frame[31 + i] = q_des_sent[i] - q_actual[i];
    std::copy(prefix.begin() + 96, prefix.begin() + 99, frame.begin() + 62);
    std::copy(prefix.begin(), prefix.begin() + 3, frame.begin() + 65);
    frame[68] = prefix[76]; frame[69] = prefix[81];
    if (pending_) {
      frames_.fill(frame);
      pending_ = false;
    } else if (tick != last_tick_) {
      for (int i = 3; i > 0; --i) frames_[i] = frames_[i - 1];
      frames_[0] = frame;
    }
    last_tick_ = tick;
    Observation output{};
    std::copy(prefix.begin(), prefix.begin() + 65, output.begin());
    std::copy(q_des_sent.begin(), q_des_sent.end(), output.begin() + 65);
    std::copy(prefix.begin() + 96, prefix.end(), output.begin() + 96);
    output[112] = prefix[76]; output[113] = prefix[81];
    for (int i = 1; i < 4; ++i)
      std::copy(frames_[i].begin(), frames_[i].end(), output.begin() + 114 + 70 * (i - 1));
    return output;
  }
 private:
  bool pending_ = true;
  std::int64_t last_tick_ = -1;
  std::array<std::array<double, 70>, 4> frames_{};
};
}  // namespace a3_pingpong
