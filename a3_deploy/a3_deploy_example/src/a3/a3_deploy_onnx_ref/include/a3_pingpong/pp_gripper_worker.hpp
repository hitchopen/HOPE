#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace a3_pingpong {

inline constexpr char kPpGripperProtocolVersion[] = "A3P_GRIPPER_V3";

enum class PpGripperCommand : std::uint8_t {
  kStatus = 0,
  kOpen,
  kGrab,
  kRelease,
};

const char* PpGripperCommandName(PpGripperCommand command) noexcept;

struct PpGripperReceipt {
  std::uint64_t request_id{0};
  bool ok{false};
  PpGripperCommand command{PpGripperCommand::kStatus};
  std::string state;
  std::uint64_t first_publish_monotonic_ns{0};
  std::uint64_t published_before_ack{0};
  std::uint64_t planned_publish_count{0};
  std::uint64_t matched_subscribers{0};
  bool physical_confirmation{false};
  bool state_observable{false};
  std::string detail;
};

struct PpGripperWorkerSnapshot {
  bool running{false};
  bool request_in_flight{false};
  bool faulted{false};
  bool stopped{false};
  std::uint64_t submitted_generation{0};
  std::uint64_t acknowledged_generation{0};
  PpGripperCommand acknowledged_command{PpGripperCommand::kStatus};
  std::string bridge_state{"UNKNOWN"};
  std::uint64_t first_publish_monotonic_ns{0};
  std::string fault_reason;
};

struct PpGripperWorkerTransport {
  using Exchange = std::function<bool(
      PpGripperCommand command, std::chrono::milliseconds timeout,
      PpGripperReceipt& receipt, std::string& error)>;
  Exchange exchange;
  std::function<void()> close;
};

// Bounded background worker. The Runner thread only submits an
// allowlisted edge and polls a snapshot; all Unix-socket waits happen here.
class PpGripperWorker final {
 public:
  explicit PpGripperWorker(
      PpGripperWorkerTransport transport,
      std::chrono::milliseconds exchange_timeout =
          std::chrono::milliseconds(1000));
  ~PpGripperWorker();

  PpGripperWorker(const PpGripperWorker&) = delete;
  PpGripperWorker& operator=(const PpGripperWorker&) = delete;

  // Validates the owner-only Unix socket, peer UID and an initial STATUS
  // response before returning a started worker.
  static std::unique_ptr<PpGripperWorker> Connect(
      const std::string& socket_path, std::string& error);

  bool Start(std::string& error);
  bool Submit(PpGripperCommand command, std::uint64_t generation,
              std::string& error);
  PpGripperWorkerSnapshot Snapshot() const;
  void Stop() noexcept;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace a3_pingpong
