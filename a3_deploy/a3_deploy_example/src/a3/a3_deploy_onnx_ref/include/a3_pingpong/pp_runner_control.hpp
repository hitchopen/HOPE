// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// Narrow operator-control contract for the local ping-pong Runner.  This is
// deliberately independent of ROS/AimRT so the state machine can be tested
// without a robot runtime.  Transport callbacks may enqueue a fixed action,
// but only the Runner action worker applies mode/role changes.
#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

namespace a3_pingpong {

constexpr double kRunnerControlSchemaVersion = 1.0;
constexpr std::size_t kRunnerControlRequestSize = 4;
constexpr std::size_t kRunnerStateSize = 19;
constexpr std::uint64_t kRunnerMaxExactFloatInteger = (1ULL << 52);

enum class RunnerMode : int {
  kPassive = 0,
  kPdStand = 1,
  kShadow = 2,
  kMotion = 3,
  kReferencePlayback = 4,
  kServe = 5,
};

inline const char* RunnerModeName(RunnerMode mode) {
  switch (mode) {
    case RunnerMode::kPassive: return "PASSIVE";
    case RunnerMode::kPdStand: return "PD_STAND";
    case RunnerMode::kShadow: return "SHADOW(no-publish)";
    case RunnerMode::kMotion: return "MOTION";
    case RunnerMode::kReferencePlayback: return "REFERENCE_PLAYBACK";
    case RunnerMode::kServe: return "SERVE";
  }
  return "UNKNOWN";
}

enum class LocalRole : int {
  kUnassigned = 0,
  kServer = 1,
  kReceiver = 2,
};

inline const char* LocalRoleName(LocalRole role) {
  switch (role) {
    case LocalRole::kUnassigned: return "UNASSIGNED";
    case LocalRole::kServer: return "SERVER";
    case LocalRole::kReceiver: return "RECEIVER";
  }
  return "UNKNOWN";
}

// These codes are a frozen wire contract.  Foxglove exposes actions 1..5 and
// 7..8.  SHADOW is included so the keyboard can share the same transition
// logic; code 6 is intentionally not exposed as a service or accepted on the
// remote flat wire.
enum class RunnerAction : int {
  kNone = 0,
  kSetServer = 1,
  kSetReceiver = 2,
  kEnterPdStand = 3,
  kEnterMotion = 4,
  kEmergencyPassive = 5,
  kEnterShadow = 6,
  kReadyToServe = 7,
  kServe = 8,
};

inline const char* RunnerActionName(RunnerAction action) {
  switch (action) {
    case RunnerAction::kNone: return "NONE";
    case RunnerAction::kSetServer: return "SET_SERVER";
    case RunnerAction::kSetReceiver: return "SET_RECEIVER";
    case RunnerAction::kEnterPdStand: return "ENTER_PD_STAND";
    case RunnerAction::kEnterMotion: return "ENTER_MOTION";
    case RunnerAction::kEmergencyPassive: return "EMERGENCY_PASSIVE";
    case RunnerAction::kEnterShadow: return "ENTER_SHADOW";
    case RunnerAction::kReadyToServe: return "READY_TO_SERVE";
    case RunnerAction::kServe: return "SERVE";
  }
  return "UNKNOWN";
}

enum class RunnerActionResult : int {
  kNone = 0,
  kApplied = 1,
  kAlreadySet = 2,
  kAcceptedPending = 3,
  kRejectedWrongMode = 4,
  kRejectedRunnerFault = 5,
  kRejectedServeActive = 6,
  kInvalidRequest = 7,
  kQueueFull = 8,
  kRejectedServeUnavailable = 9,
  kRejectedServeNotReady = 10,
  kRejectedGainScale = 11,
};

inline const char* RunnerActionResultName(RunnerActionResult result) {
  switch (result) {
    case RunnerActionResult::kNone: return "NONE";
    case RunnerActionResult::kApplied: return "APPLIED";
    case RunnerActionResult::kAlreadySet: return "ALREADY_SET";
    case RunnerActionResult::kAcceptedPending: return "ACCEPTED_PENDING";
    case RunnerActionResult::kRejectedWrongMode: return "REJECTED_WRONG_MODE";
    case RunnerActionResult::kRejectedRunnerFault: return "REJECTED_RUNNER_FAULT";
    case RunnerActionResult::kRejectedServeActive: return "REJECTED_SERVE_ACTIVE";
    case RunnerActionResult::kInvalidRequest: return "INVALID_REQUEST";
    case RunnerActionResult::kQueueFull: return "QUEUE_FULL";
    case RunnerActionResult::kRejectedServeUnavailable:
      return "REJECTED_SERVE_UNAVAILABLE";
    case RunnerActionResult::kRejectedServeNotReady:
      return "REJECTED_SERVE_NOT_READY";
    case RunnerActionResult::kRejectedGainScale:
      return "REJECTED_GAIN_SCALE";
  }
  return "UNKNOWN";
}

enum class RunnerActionReason : int {
  kNone = 0,
  kRoleChanged = 1,
  kRoleUnchanged = 2,
  kModeChanged = 3,
  kModeUnchanged = 4,
  kServeAbortRequested = 5,
  kRoleChangeRequiresPassiveOrStand = 6,
  kRunnerCommandFaultLatched = 7,
  kServeOwnsCommand = 8,
  kMalformedRequest = 9,
  kActionQueueFull = 10,
  kServeStartRequested = 11,
  kBallOnPalmConfirmRequested = 12,
  kServeControllerUnavailable = 13,
  kServeAwaitBallRequired = 14,
  kServeGainScalesMustBeOne = 15,
  kServeFaultLatched = 16,
};

inline const char* RunnerActionReasonName(RunnerActionReason reason) {
  switch (reason) {
    case RunnerActionReason::kNone: return "NONE";
    case RunnerActionReason::kRoleChanged: return "ROLE_CHANGED";
    case RunnerActionReason::kRoleUnchanged: return "ROLE_UNCHANGED";
    case RunnerActionReason::kModeChanged: return "MODE_CHANGED";
    case RunnerActionReason::kModeUnchanged: return "MODE_UNCHANGED";
    case RunnerActionReason::kServeAbortRequested: return "SERVE_ABORT_REQUESTED";
    case RunnerActionReason::kRoleChangeRequiresPassiveOrStand:
      return "ROLE_CHANGE_REQUIRES_PASSIVE_OR_PD_STAND";
    case RunnerActionReason::kRunnerCommandFaultLatched:
      return "RUNNER_COMMAND_FAULT_LATCHED";
    case RunnerActionReason::kServeOwnsCommand: return "SERVE_OWNS_COMMAND";
    case RunnerActionReason::kMalformedRequest: return "MALFORMED_REQUEST";
    case RunnerActionReason::kActionQueueFull: return "ACTION_QUEUE_FULL";
    case RunnerActionReason::kServeStartRequested:
      return "SERVE_START_REQUESTED";
    case RunnerActionReason::kBallOnPalmConfirmRequested:
      return "BALL_ON_PALM_CONFIRM_REQUESTED";
    case RunnerActionReason::kServeControllerUnavailable:
      return "SERVE_CONTROLLER_UNAVAILABLE";
    case RunnerActionReason::kServeAwaitBallRequired:
      return "SERVE_AWAIT_BALL_REQUIRED";
    case RunnerActionReason::kServeGainScalesMustBeOne:
      return "SERVE_GAIN_SCALES_MUST_BE_ONE";
    case RunnerActionReason::kServeFaultLatched:
      return "SERVE_FAULT_LATCHED";
  }
  return "UNKNOWN";
}

struct RunnerActionRequest {
  std::uint64_t request_id{0};
  RunnerAction action{RunnerAction::kNone};
  bool remote{false};
};

struct RunnerActionDecision {
  RunnerActionRequest request{};
  RunnerActionResult result{RunnerActionResult::kNone};
  RunnerActionReason reason{RunnerActionReason::kNone};
  bool hold_reference{false};
  bool request_serve_abort{false};
  bool request_serve_start{false};
  bool request_serve_confirm{false};
};

inline bool IsRemoteRunnerAction(RunnerAction action) noexcept {
  switch (action) {
    case RunnerAction::kSetServer:
    case RunnerAction::kSetReceiver:
    case RunnerAction::kEnterPdStand:
    case RunnerAction::kEnterMotion:
    case RunnerAction::kEmergencyPassive:
    case RunnerAction::kReadyToServe:
    case RunnerAction::kServe:
      return true;
    case RunnerAction::kNone:
    case RunnerAction::kEnterShadow:
      return false;
  }
  return false;
}

inline bool IsExactFloatInteger(double value, std::uint64_t minimum,
                                std::uint64_t maximum,
                                std::uint64_t* decoded) {
  if (!std::isfinite(value) || value < static_cast<double>(minimum) ||
      value > static_cast<double>(maximum)) {
    return false;
  }
  const auto integer = static_cast<std::uint64_t>(value);
  if (static_cast<double>(integer) != value) return false;
  if (decoded != nullptr) *decoded = integer;
  return true;
}

inline std::uint64_t RunnerSessionFingerprint(std::string_view session_id) {
  // FNV-1a folded into Float64's exactly representable integer range.  The
  // observer uses the same fingerprint only to associate the flat status with
  // its human-readable session id; this is not a security primitive.
  std::uint64_t value = 1469598103934665603ULL;
  for (const unsigned char character : session_id) {
    value ^= character;
    value *= 1099511628211ULL;
  }
  value &= (kRunnerMaxExactFloatInteger - 1);
  return value == 0 ? 1 : value;
}

class PpRunnerControl {
 public:
  explicit PpRunnerControl(RunnerMode initial_mode, std::uint64_t boot_id,
                           std::string_view session_id,
                           std::size_t queue_capacity = 16)
      : mode_(initial_mode),
        boot_id_(NormalizeExactId_(boot_id)),
        session_fingerprint_(RunnerSessionFingerprint(session_id)),
        queue_capacity_(queue_capacity == 0 ? 1 : queue_capacity) {}

  RunnerMode mode() const noexcept {
    return mode_.load(std::memory_order_acquire);
  }
  LocalRole local_role() const noexcept {
    return local_role_.load(std::memory_order_acquire);
  }
  std::uint64_t role_epoch() const noexcept {
    return role_epoch_.load(std::memory_order_acquire);
  }
  std::uint64_t state_sequence() const noexcept {
    return state_sequence_.load(std::memory_order_acquire);
  }

  bool RoleChangeAllowed(bool command_fault_latched) const noexcept {
    const RunnerMode current = mode();
    return !command_fault_latched &&
           (current == RunnerMode::kPassive ||
            current == RunnerMode::kPdStand);
  }

  void SetRuntimeMode(RunnerMode next) noexcept {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (mode_.exchange(next, std::memory_order_acq_rel) != next) Touch_();
  }

  // Wire request: [schema=1, request_id, action_code, reserved=0].
  bool EnqueueFlatRequest(const std::vector<double>& values) {
    std::uint64_t request_id = 0;
    std::uint64_t action_code = 0;
    const bool id_decoded =
        values.size() >= 2 &&
        IsExactFloatInteger(values[1], 1, kRunnerMaxExactFloatInteger,
                            &request_id);
    const bool action_decoded =
        values.size() >= 3 &&
        IsExactFloatInteger(values[2], 1,
                            static_cast<std::uint64_t>(RunnerAction::kServe),
                            &action_code);
    const bool valid =
        values.size() == kRunnerControlRequestSize &&
        values[0] == kRunnerControlSchemaVersion && id_decoded &&
        action_decoded &&
        IsRemoteRunnerAction(static_cast<RunnerAction>(action_code)) &&
        values[3] == 0.0;
    if (!valid) {
      RecordResult_(RunnerActionRequest{request_id, RunnerAction::kNone, true},
                    RunnerActionResult::kInvalidRequest,
                    RunnerActionReason::kMalformedRequest);
      return false;
    }
    return Enqueue({request_id, static_cast<RunnerAction>(action_code), true});
  }

  bool EnqueueLocalAction(RunnerAction action) {
    std::uint64_t id = local_request_id_.fetch_add(1, std::memory_order_relaxed);
    id &= (kRunnerMaxExactFloatInteger - 1);
    if (id == 0) id = 1;
    return Enqueue({id, action, false});
  }

  bool Enqueue(RunnerActionRequest request) {
    if (request.request_id == 0 ||
        request.request_id > kRunnerMaxExactFloatInteger ||
        request.action == RunnerAction::kNone ||
        static_cast<int>(request.action) >
            static_cast<int>(RunnerAction::kServe) ||
        (request.remote && !IsRemoteRunnerAction(request.action))) {
      RecordResult_(request, RunnerActionResult::kInvalidRequest,
                    RunnerActionReason::kMalformedRequest);
      return false;
    }
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (queue_.size() >= queue_capacity_) {
      if (request.action == RunnerAction::kEmergencyPassive) {
        // The local zero-gain escape must not be crowded out by stale normal
        // actions.  Drop the newest queued request, record it, then enqueue p.
        const RunnerActionRequest dropped = queue_.back();
        queue_.pop_back();
        RecordResult_(dropped, RunnerActionResult::kQueueFull,
                      RunnerActionReason::kActionQueueFull);
      } else {
        RecordResult_(request, RunnerActionResult::kQueueFull,
                      RunnerActionReason::kActionQueueFull);
        return false;
      }
    }
    if (request.action == RunnerAction::kEmergencyPassive) {
      queue_.push_front(request);
    } else {
      queue_.push_back(request);
    }
    return true;
  }

  std::vector<RunnerActionDecision> ProcessPending(
      bool command_fault_latched, bool serve_active,
      bool serve_capability = false, int serve_state = -1,
      bool serve_gain_scales_nominal = false) {
    std::deque<RunnerActionRequest> pending;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      pending.swap(queue_);
    }
    std::vector<RunnerActionDecision> decisions;
    decisions.reserve(pending.size());
    for (const auto& request : pending) {
      decisions.push_back(
          Apply_(request, command_fault_latched, serve_active,
                 serve_capability, serve_state, serve_gain_scales_nominal));
    }
    return decisions;
  }

  void ObserveExternalState(bool command_publishing, bool policy_native,
                            bool command_fault_latched, bool serve_capability,
                            int serve_state) noexcept {
    std::lock_guard<std::mutex> lock(state_mutex_);
    bool changed = false;
    changed |= ExchangeChanged_(command_publishing_, command_publishing);
    changed |= ExchangeChanged_(policy_native_, policy_native);
    changed |= ExchangeChanged_(command_fault_latched_, command_fault_latched);
    changed |= ExchangeChanged_(serve_capability_, serve_capability);
    changed |= ExchangeChanged_(serve_state_, serve_state);
    if (changed) Touch_();
  }

  // Frozen state wire (19 doubles): schema, boot id, state seq, mode,
  // command publishing, policy native, fault, local role, role epoch,
  // role-change allowed, role last result/reason, serve capability/state,
  // last action id/action/result/reason, session fingerprint.
  std::vector<double> EncodeState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const bool fault = command_fault_latched_.load(std::memory_order_acquire);
    return {
        kRunnerControlSchemaVersion,
        static_cast<double>(boot_id_),
        static_cast<double>(state_sequence()),
        static_cast<double>(static_cast<int>(mode())),
        command_publishing_.load(std::memory_order_acquire) ? 1.0 : 0.0,
        policy_native_.load(std::memory_order_acquire) ? 1.0 : 0.0,
        fault ? 1.0 : 0.0,
        static_cast<double>(static_cast<int>(local_role())),
        static_cast<double>(role_epoch()),
        RoleChangeAllowed(fault) ? 1.0 : 0.0,
        static_cast<double>(static_cast<int>(
            role_last_result_.load(std::memory_order_acquire))),
        static_cast<double>(static_cast<int>(
            role_last_reason_.load(std::memory_order_acquire))),
        serve_capability_.load(std::memory_order_acquire) ? 1.0 : 0.0,
        static_cast<double>(serve_state_.load(std::memory_order_acquire)),
        static_cast<double>(last_action_id_.load(std::memory_order_acquire)),
        static_cast<double>(static_cast<int>(
            last_action_.load(std::memory_order_acquire))),
        static_cast<double>(static_cast<int>(
            last_action_result_.load(std::memory_order_acquire))),
        static_cast<double>(static_cast<int>(
            last_action_reason_.load(std::memory_order_acquire))),
        static_cast<double>(session_fingerprint_),
    };
  }

 private:
  static std::uint64_t NormalizeExactId_(std::uint64_t value) noexcept {
    value &= (kRunnerMaxExactFloatInteger - 1);
    return value == 0 ? 1 : value;
  }

  template <typename T>
  static bool ExchangeChanged_(std::atomic<T>& target, T next) noexcept {
    return target.exchange(next, std::memory_order_acq_rel) != next;
  }

  void Touch_() noexcept {
    state_sequence_.fetch_add(1, std::memory_order_acq_rel);
  }

  void RecordResult_(const RunnerActionRequest& request,
                     RunnerActionResult result,
                     RunnerActionReason reason) noexcept {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_action_id_.store(request.request_id, std::memory_order_release);
    last_action_.store(request.action, std::memory_order_release);
    last_action_result_.store(result, std::memory_order_release);
    last_action_reason_.store(reason, std::memory_order_release);
    if (request.action == RunnerAction::kSetServer ||
        request.action == RunnerAction::kSetReceiver) {
      role_last_result_.store(result, std::memory_order_release);
      role_last_reason_.store(reason, std::memory_order_release);
    }
    Touch_();
  }

  RunnerActionDecision Apply_(const RunnerActionRequest& request,
                              bool command_fault_latched,
                              bool serve_active, bool serve_capability,
                              int serve_state,
                              bool serve_gain_scales_nominal) {
    RunnerActionDecision decision{};
    decision.request = request;
    const RunnerMode current_mode = mode();
    switch (request.action) {
      case RunnerAction::kSetServer:
      case RunnerAction::kSetReceiver: {
        if (command_fault_latched) {
          decision.result = RunnerActionResult::kRejectedRunnerFault;
          decision.reason = RunnerActionReason::kRunnerCommandFaultLatched;
          break;
        }
        if (serve_active) {
          decision.result = RunnerActionResult::kRejectedWrongMode;
          decision.reason = RunnerActionReason::kServeOwnsCommand;
          break;
        }
        if (current_mode != RunnerMode::kPassive &&
            current_mode != RunnerMode::kPdStand) {
          decision.result = RunnerActionResult::kRejectedWrongMode;
          decision.reason =
              RunnerActionReason::kRoleChangeRequiresPassiveOrStand;
          break;
        }
        const LocalRole requested = request.action == RunnerAction::kSetServer
                                        ? LocalRole::kServer
                                        : LocalRole::kReceiver;
        if (local_role() == requested) {
          decision.result = RunnerActionResult::kAlreadySet;
          decision.reason = RunnerActionReason::kRoleUnchanged;
          break;
        }
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          local_role_.store(requested, std::memory_order_release);
          role_epoch_.fetch_add(1, std::memory_order_acq_rel);
          Touch_();
        }
        decision.result = RunnerActionResult::kApplied;
        decision.reason = RunnerActionReason::kRoleChanged;
        break;
      }
      case RunnerAction::kEnterPdStand:
        if (current_mode == RunnerMode::kServe && serve_active) {
          decision.result = RunnerActionResult::kAcceptedPending;
          decision.reason = RunnerActionReason::kServeAbortRequested;
          decision.request_serve_abort = true;
        } else if (current_mode == RunnerMode::kPdStand) {
          decision.result = RunnerActionResult::kAlreadySet;
          decision.reason = RunnerActionReason::kModeUnchanged;
        } else {
          SetRuntimeMode(RunnerMode::kPdStand);
          decision.result = RunnerActionResult::kApplied;
          decision.reason = RunnerActionReason::kModeChanged;
          decision.hold_reference = true;
        }
        break;
      case RunnerAction::kEnterMotion:
        if (current_mode == RunnerMode::kServe && serve_active) {
          decision.result = RunnerActionResult::kRejectedServeActive;
          decision.reason = RunnerActionReason::kServeOwnsCommand;
        } else if (current_mode == RunnerMode::kMotion) {
          decision.result = RunnerActionResult::kAlreadySet;
          decision.reason = RunnerActionReason::kModeUnchanged;
        } else {
          SetRuntimeMode(RunnerMode::kMotion);
          decision.result = RunnerActionResult::kApplied;
          decision.reason = RunnerActionReason::kModeChanged;
        }
        break;
      case RunnerAction::kEmergencyPassive:
        if (current_mode == RunnerMode::kPassive) {
          decision.result = RunnerActionResult::kAlreadySet;
          decision.reason = RunnerActionReason::kModeUnchanged;
        } else {
          SetRuntimeMode(RunnerMode::kPassive);
          decision.result = RunnerActionResult::kApplied;
          decision.reason = RunnerActionReason::kModeChanged;
        }
        decision.hold_reference = true;
        break;
      case RunnerAction::kEnterShadow:
        if (current_mode == RunnerMode::kServe && serve_active) {
          decision.result = RunnerActionResult::kRejectedServeActive;
          decision.reason = RunnerActionReason::kServeOwnsCommand;
        } else if (current_mode == RunnerMode::kShadow) {
          decision.result = RunnerActionResult::kAlreadySet;
          decision.reason = RunnerActionReason::kModeUnchanged;
        } else {
          SetRuntimeMode(RunnerMode::kShadow);
          decision.result = RunnerActionResult::kApplied;
          decision.reason = RunnerActionReason::kModeChanged;
        }
        break;
      case RunnerAction::kReadyToServe:
        if (command_fault_latched) {
          decision.result = RunnerActionResult::kRejectedRunnerFault;
          decision.reason = RunnerActionReason::kRunnerCommandFaultLatched;
        } else if (!serve_capability) {
          decision.result = RunnerActionResult::kRejectedServeUnavailable;
          decision.reason = RunnerActionReason::kServeControllerUnavailable;
        } else if (serve_state == 8) {
          decision.result = RunnerActionResult::kRejectedServeNotReady;
          decision.reason = RunnerActionReason::kServeFaultLatched;
        } else if (serve_active) {
          if (serve_state == 3) {
            decision.result = RunnerActionResult::kAlreadySet;
            decision.reason = RunnerActionReason::kModeUnchanged;
          } else {
            decision.result = RunnerActionResult::kRejectedServeActive;
            decision.reason = RunnerActionReason::kServeOwnsCommand;
          }
        } else if (!serve_gain_scales_nominal) {
          decision.result = RunnerActionResult::kRejectedGainScale;
          decision.reason = RunnerActionReason::kServeGainScalesMustBeOne;
        } else {
          SetRuntimeMode(RunnerMode::kServe);
          decision.result = RunnerActionResult::kApplied;
          decision.reason = RunnerActionReason::kServeStartRequested;
          decision.request_serve_start = true;
        }
        break;
      case RunnerAction::kServe:
        if (command_fault_latched) {
          decision.result = RunnerActionResult::kRejectedRunnerFault;
          decision.reason = RunnerActionReason::kRunnerCommandFaultLatched;
        } else if (!serve_capability) {
          decision.result = RunnerActionResult::kRejectedServeUnavailable;
          decision.reason = RunnerActionReason::kServeControllerUnavailable;
        } else if (serve_state == 8) {
          decision.result = RunnerActionResult::kRejectedServeNotReady;
          decision.reason = RunnerActionReason::kServeFaultLatched;
        } else if (current_mode != RunnerMode::kServe || serve_state != 3) {
          decision.result = RunnerActionResult::kRejectedServeNotReady;
          decision.reason = RunnerActionReason::kServeAwaitBallRequired;
        } else {
          decision.result = RunnerActionResult::kAcceptedPending;
          decision.reason = RunnerActionReason::kBallOnPalmConfirmRequested;
          decision.request_serve_confirm = true;
        }
        break;
      case RunnerAction::kNone:
        decision.result = RunnerActionResult::kInvalidRequest;
        decision.reason = RunnerActionReason::kMalformedRequest;
        break;
    }
    RecordResult_(request, decision.result, decision.reason);
    return decision;
  }

  std::atomic<RunnerMode> mode_;
  const std::uint64_t boot_id_;
  const std::uint64_t session_fingerprint_;
  const std::size_t queue_capacity_;
  mutable std::mutex state_mutex_;
  mutable std::mutex queue_mutex_;
  std::deque<RunnerActionRequest> queue_;
  std::atomic<std::uint64_t> local_request_id_{1};

  std::atomic<LocalRole> local_role_{LocalRole::kUnassigned};
  std::atomic<std::uint64_t> role_epoch_{0};
  std::atomic<std::uint64_t> state_sequence_{1};
  std::atomic<bool> command_publishing_{false};
  std::atomic<bool> policy_native_{false};
  std::atomic<bool> command_fault_latched_{false};
  std::atomic<bool> serve_capability_{false};
  std::atomic<int> serve_state_{-1};
  std::atomic<RunnerActionResult> role_last_result_{RunnerActionResult::kNone};
  std::atomic<RunnerActionReason> role_last_reason_{RunnerActionReason::kNone};
  std::atomic<std::uint64_t> last_action_id_{0};
  std::atomic<RunnerAction> last_action_{RunnerAction::kNone};
  std::atomic<RunnerActionResult> last_action_result_{RunnerActionResult::kNone};
  std::atomic<RunnerActionReason> last_action_reason_{RunnerActionReason::kNone};
};

}  // namespace a3_pingpong
