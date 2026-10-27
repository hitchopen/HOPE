#pragma once

namespace a3_pingpong {

// Driver-thread-only edge reporting. A rejected terminal transition must not
// become per-tick stderr I/O; a later accepted transition is a distinct event.
class ServeTerminalRecoveryEvents {
 public:
  void Reset() noexcept { state_ = result_ = -1; }
  bool Observe(int terminal_state, bool transitioned) noexcept {
    const int result = transitioned ? 1 : 0;
    if (state_ == terminal_state && result_ == result) return false;
    state_ = terminal_state;
    result_ = result;
    return true;
  }
 private:
  int state_{-1};
  int result_{-1};
};
}  // namespace a3_pingpong
