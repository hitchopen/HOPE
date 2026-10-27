// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// Narrow, role-aware operator contract for the local ping-pong Runner.
// Transport callbacks may enqueue only one fixed action code; the Runner
// worker remains the sole authority for role/mode/serve admission.
#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string_view>
#include <vector>

namespace a3_pingpong {

constexpr double kRunnerControlSchemaVersion = 2.0;
constexpr std::size_t kRunnerControlRequestSize = 4;
constexpr std::size_t kRunnerStateSize = 21;
constexpr std::uint64_t kRunnerMaxExactFloatInteger = (1ULL << 52);

enum class RunnerMode : int {
  kPassive = 0,
  kPdStand = 1,
  kShadow = 2,
  kMotion = 3,
  kReferencePlayback = 4,
  kServe = 5,
  kTeleop = 6,
};

inline const char* RunnerModeName(RunnerMode mode) {
  switch (mode) {
    case RunnerMode::kPassive: return "PASSIVE";
    case RunnerMode::kPdStand: return "PD_STAND";
    case RunnerMode::kShadow: return "SHADOW(no-publish)";
    case RunnerMode::kMotion: return "MOTION";
    case RunnerMode::kReferencePlayback: return "REFERENCE_PLAYBACK";
    case RunnerMode::kServe: return "SERVE";
    case RunnerMode::kTeleop: return "TELEOP";
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

// Frozen schema-2 action codes.  Code 6 remains keyboard-only SHADOW.
enum class RunnerAction : int {
  kNone = 0,
  kSetServer = 1,
  kSetReceiver = 2,
  kEnterPdStand = 3,
  kEnterMotion = 4,
  kEmergencyPassive = 5,
  kEnterShadow = 6,
  kPrepareServe = 7,
  kConfirmBallLoaded = 8,
  kReadyToServe = 9,
  kOpenGripper = 10,
  kConfirmLoadingZoneClear = 11,
  kConfirmGripSecure = 12,
  kEnterTeleop = 13,
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
    case RunnerAction::kPrepareServe: return "PREPARE_SERVE";
    case RunnerAction::kConfirmBallLoaded:
      return "CONFIRM_BALL_LOADED";
    case RunnerAction::kReadyToServe: return "READY_TO_SERVE";
    case RunnerAction::kOpenGripper: return "OPEN_GRIPPER";
    case RunnerAction::kConfirmLoadingZoneClear:
      return "CONFIRM_LOADING_ZONE_CLEAR";
    case RunnerAction::kConfirmGripSecure:
      return "CONFIRM_GRIP_SECURE";
    case RunnerAction::kEnterTeleop: return "ENTER_TELEOP";
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
  kRejectedWrongRole = 12,
  kRejectedCleanupRequired = 13,
  kRejectedGripperState = 14,
  kRejectedTeleop = 15,
};

inline const char* RunnerActionResultName(RunnerActionResult result) {
  switch (result) {
    case RunnerActionResult::kNone: return "NONE";
    case RunnerActionResult::kApplied: return "APPLIED";
    case RunnerActionResult::kAlreadySet: return "ALREADY_SET";
    case RunnerActionResult::kAcceptedPending: return "ACCEPTED_PENDING";
    case RunnerActionResult::kRejectedWrongMode:
      return "REJECTED_WRONG_MODE";
    case RunnerActionResult::kRejectedRunnerFault:
      return "REJECTED_RUNNER_FAULT";
    case RunnerActionResult::kRejectedServeActive:
      return "REJECTED_SERVE_ACTIVE";
    case RunnerActionResult::kInvalidRequest: return "INVALID_REQUEST";
    case RunnerActionResult::kQueueFull: return "QUEUE_FULL";
    case RunnerActionResult::kRejectedServeUnavailable:
      return "REJECTED_SERVE_UNAVAILABLE";
    case RunnerActionResult::kRejectedServeNotReady:
      return "REJECTED_SERVE_NOT_READY";
    case RunnerActionResult::kRejectedGainScale:
      return "REJECTED_GAIN_SCALE";
    case RunnerActionResult::kRejectedWrongRole:
      return "REJECTED_WRONG_ROLE";
    case RunnerActionResult::kRejectedCleanupRequired:
      return "REJECTED_CLEANUP_REQUIRED";
    case RunnerActionResult::kRejectedGripperState:
      return "REJECTED_GRIPPER_STATE";
    case RunnerActionResult::kRejectedTeleop: return "REJECTED_TELEOP";
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
  kServePrepareRequested = 11,
  kBallLoadedConfirmRequested = 12,
  kServeControllerUnavailable = 13,
  kServePhaseMismatch = 14,
  kServeGainScalesMustBeOne = 15,
  kServeFaultLatched = 16,
  kServerRoleRequired = 17,
  kReceiverRoleRequired = 18,
  kGripperCleanupRequired = 19,
  kReadyToServePlayRequested = 20,
  kGripperOpenRequested = 21,
  kLoadingZoneClearRequested = 22,
  kPdStandRequired = 23,
  kGripperMustBeGrabbed = 24,
  kGripSecureConfirmRequested = 25,
  // Appended without changing the frozen action/state schema.  Action 8 is
  // retained for old clients but PREPARE_SERVE now owns an immediate GRAB.
  kBallLoadMergedIntoPrepare = 26,
  kLoadingZoneClearRemoved = 27,
  kPureServeOnly = 28,
  kTeleopUnavailable = 29,
  kTeleopInputNotReady = 30,
  kTeleopStopRequested = 31,
  kServeRecoveryPending = 34,
  kMocapUnavailable = 35,
  kOperatorUiUnavailable = 36,
  kOperatorComputerUnavailable = 37,
};

inline const char* RunnerActionReasonName(RunnerActionReason reason) {
  switch (reason) {
    case RunnerActionReason::kNone: return "NONE";
    case RunnerActionReason::kRoleChanged: return "ROLE_CHANGED";
    case RunnerActionReason::kRoleUnchanged: return "ROLE_UNCHANGED";
    case RunnerActionReason::kModeChanged: return "MODE_CHANGED";
    case RunnerActionReason::kModeUnchanged: return "MODE_UNCHANGED";
    case RunnerActionReason::kServeAbortRequested:
      return "SERVE_ABORT_REQUESTED";
    case RunnerActionReason::kRoleChangeRequiresPassiveOrStand:
      return "ROLE_CHANGE_REQUIRES_PASSIVE_OR_PD_STAND";
    case RunnerActionReason::kRunnerCommandFaultLatched:
      return "RUNNER_COMMAND_FAULT_LATCHED";
    case RunnerActionReason::kServeOwnsCommand:
      return "SERVE_OWNS_COMMAND";
    case RunnerActionReason::kMalformedRequest:
      return "MALFORMED_REQUEST";
    case RunnerActionReason::kActionQueueFull:
      return "ACTION_QUEUE_FULL";
    case RunnerActionReason::kServePrepareRequested:
      return "SERVE_PREPARE_REQUESTED";
    case RunnerActionReason::kBallLoadedConfirmRequested:
      return "BALL_LOADED_CONFIRM_REQUESTED";
    case RunnerActionReason::kServeControllerUnavailable:
      return "SERVE_CONTROLLER_UNAVAILABLE";
    case RunnerActionReason::kServePhaseMismatch:
      return "SERVE_PHASE_MISMATCH";
    case RunnerActionReason::kServeGainScalesMustBeOne:
      return "SERVE_GAIN_SCALES_MUST_BE_ONE";
    case RunnerActionReason::kServeFaultLatched:
      return "SERVE_FAULT_LATCHED";
    case RunnerActionReason::kServerRoleRequired:
      return "SERVER_ROLE_REQUIRED";
    case RunnerActionReason::kReceiverRoleRequired:
      return "RECEIVER_ROLE_REQUIRED";
    case RunnerActionReason::kGripperCleanupRequired:
      return "GRIPPER_CLEANUP_REQUIRED";
    case RunnerActionReason::kReadyToServePlayRequested:
      return "READY_TO_SERVE_PLAY_REQUESTED";
    case RunnerActionReason::kGripperOpenRequested:
      return "GRIPPER_OPEN_REQUESTED";
    case RunnerActionReason::kLoadingZoneClearRequested:
      return "LOADING_ZONE_CLEAR_REQUESTED";
    case RunnerActionReason::kPdStandRequired:
      return "PD_STAND_REQUIRED";
    case RunnerActionReason::kGripperMustBeGrabbed:
      return "GRIPPER_MUST_BE_GRABBED";
    case RunnerActionReason::kGripSecureConfirmRequested:
      return "GRIP_SECURE_CONFIRM_REQUESTED";
    case RunnerActionReason::kBallLoadMergedIntoPrepare:
      return "BALL_LOAD_MERGED_INTO_PREPARE";
    case RunnerActionReason::kPureServeOnly:
      return "PURE_SERVE_ONLY";
    case RunnerActionReason::kLoadingZoneClearRemoved:
      return "LOADING_ZONE_CLEAR_REMOVED";
    case RunnerActionReason::kTeleopUnavailable: return "TELEOP_UNAVAILABLE";
    case RunnerActionReason::kTeleopInputNotReady: return "TELEOP_INPUT_NOT_READY";
    case RunnerActionReason::kTeleopStopRequested: return "TELEOP_STOP_REQUESTED";
    case RunnerActionReason::kServeRecoveryPending: return "SERVE_RECOVERY_PENDING";
    case RunnerActionReason::kMocapUnavailable: return "MOCAP_UNAVAILABLE";
    case RunnerActionReason::kOperatorUiUnavailable: return "OPERATOR_UI_UNAVAILABLE";
    case RunnerActionReason::kOperatorComputerUnavailable: return "OPERATOR_COMPUTER_UNAVAILABLE";
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
  bool request_prepare_serve{false};
  bool request_confirm_ball_loaded{false};
  bool request_confirm_grip_secure{false};
  bool request_ready_to_serve{false};
  bool request_open_gripper{false};
  bool request_teleop_stop{false};
};

inline bool IsRemoteRunnerAction(RunnerAction action) noexcept {
  switch (action) {
    case RunnerAction::kSetServer:
    case RunnerAction::kSetReceiver:
    case RunnerAction::kEnterPdStand:
    case RunnerAction::kEnterMotion:
    case RunnerAction::kEmergencyPassive:
    case RunnerAction::kPrepareServe:
    case RunnerAction::kConfirmBallLoaded:
    case RunnerAction::kReadyToServe:
    case RunnerAction::kOpenGripper:
    case RunnerAction::kConfirmLoadingZoneClear:
    case RunnerAction::kConfirmGripSecure:
    case RunnerAction::kEnterTeleop:
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
                           std::size_t queue_capacity = 16, bool serve_only = false)
      : mode_(initial_mode),
        boot_id_(NormalizeExactId_(boot_id)),
        session_fingerprint_(RunnerSessionFingerprint(session_id)),
        queue_capacity_(queue_capacity == 0 ? 1 : queue_capacity),
        serve_only_(serve_only) {}

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

  bool serve_only() const noexcept { return serve_only_; }
  void RequireMocap(bool required) noexcept {
    std::lock_guard<std::mutex> lock(state_mutex_);
    mocap_required_.store(required);
    mocap_fresh_.store(!required);
    mocap_within_deadline_ = !required;
  }
  bool MocapStopRequested() const noexcept { return mocap_stop_requested_.load(); }
  bool PlayStopRequested() const noexcept {
    return mocap_stop_requested_.load() || operator_stop_requested_.load();
  }
  void RequireOperatorHeartbeat(bool required) noexcept {
    std::lock_guard<std::mutex> lock(state_mutex_);
    operator_required_ = required;
    operator_ui_fresh_ = operator_computer_fresh_ = !required;
  }
  bool ObserveOperatorHeartbeat(bool ui_fresh, bool computer_fresh) noexcept {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (operator_ui_fresh_ != ui_fresh || operator_computer_fresh_ != computer_fresh) Touch_();
    operator_ui_fresh_ = ui_fresh;
    operator_computer_fresh_ = computer_fresh;
    const auto current = mode();
    if (current != RunnerMode::kServe) operator_stop_requested_.store(false);
    if (!operator_required_ || (ui_fresh && computer_fresh)) return false;
    if (current != RunnerMode::kMotion && current != RunnerMode::kServe) return false;
    if (current == RunnerMode::kServe && operator_stop_requested_.exchange(true)) return false;
    if (current == RunnerMode::kMotion) mode_.store(RunnerMode::kPdStand, std::memory_order_release);
    operator_interrupted_ = ui_fresh ? RunnerActionReason::kOperatorComputerUnavailable
                                    : RunnerActionReason::kOperatorUiUnavailable;
    ++operator_stop_count_;
    Touch_();
    return true;
  }
  // Called on the command thread with monotonic mailbox freshness. SERVE uses
  // its authored abort/recovery path; MOTION enters the existing Stand blend.
  bool ObserveMocap(bool within_deadline, bool held = false, double age_s = -1.) noexcept {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const bool fresh = within_deadline && !held;
    const bool deadline_changed = mocap_within_deadline_ != within_deadline;
    mocap_within_deadline_ = within_deadline;
    mocap_held_ = within_deadline && held;
    mocap_age_ms_ = std::isfinite(age_s) && age_s >= 0. ? age_s * 1000. : -1.;
    if (mocap_fresh_.exchange(fresh) != fresh || deadline_changed) Touch_();
    if (!mocap_required_.load()) return false;
    const auto current = mode();
    if (current != RunnerMode::kServe) mocap_stop_requested_.store(false);
    if (within_deadline) return false;
    if (current == RunnerMode::kMotion) {
      mode_.store(RunnerMode::kPdStand, std::memory_order_release);
      mocap_interrupted_ = true;
      ++mocap_stop_count_;
      Touch_();
      return true;
    }
    if (current == RunnerMode::kServe && !mocap_stop_requested_.exchange(true)) {
      mocap_interrupted_ = true;
      ++mocap_stop_count_;
      Touch_();
      return true;
    }
    return false;
  }
  bool TeleopStopRequested() const noexcept { return teleop_stop_requested_.load(); }
  // Consumed by the command thread, after observing SERVE. Keep this separate
  // from mode edges: COMPLETE -> MOTION -> SERVE can occur between callbacks.
  bool ConsumeServePrepare() noexcept { return serve_prepare_pending_.exchange(false); }

  bool CompleteServe(bool kernel_mode = false) noexcept {
    // Kernel serving finishes in Stand after the controller lowers the arms.
    // Normal play retains automatic receive-policy entry without a telemetry
    // permit. A late completion must not undo an explicit emergency mode.
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (mode() != RunnerMode::kServe) return false;
    const auto reason = PlayAdmissionReasonLocked_();
    // The completion edge can precede the monitor's next observation. If it
    // cannot resume normal play, expose that safety event, not a silent Stand.
    // Do not count a deliberate offline/serve-only completion as a link fault.
    if (!serve_only_ && !kernel_mode) {
      if (reason == RunnerActionReason::kMocapUnavailable && !mocap_interrupted_) {
        mocap_interrupted_ = true;
        ++mocap_stop_count_;
      } else if ((reason == RunnerActionReason::kOperatorUiUnavailable ||
                  reason == RunnerActionReason::kOperatorComputerUnavailable) &&
                 operator_interrupted_ == RunnerActionReason::kNone) {
        operator_interrupted_ = reason;
        ++operator_stop_count_;
      }
    }
    mode_.store((serve_only_ || kernel_mode || PlayStopRequested() ||
                 reason != RunnerActionReason::kNone)
                    ? RunnerMode::kPdStand : RunnerMode::kMotion,
                std::memory_order_release);
    Touch_();
    return true;
  }

  // Command-thread recovery after ComputeCommand validates the current sample.
  // Keep the authored command this tick; normal mode blending enters Stand on
  // the next tick. A concurrent emergency PASSIVE request remains authoritative.
  bool RecoverTerminalServeToStand(int serve_state, bool current_sample_valid) noexcept {
    if (!current_sample_valid ||
        (serve_state != kServeFault && serve_state != kServeAborted)) return false;
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (mode() != RunnerMode::kServe) return false;
    mode_.store(RunnerMode::kPdStand, std::memory_order_release);
    Touch_();
    return true;
  }

  bool SetRuntimeMode(RunnerMode next, bool prepare_serve = false,
                      RunnerActionReason* refused = nullptr) noexcept {
    // Also covers local keyboard/reference-playback/warmup paths.
    if (serve_only_ && (next == RunnerMode::kMotion ||
                        next == RunnerMode::kShadow ||
                        next == RunnerMode::kReferencePlayback)) {
      if (refused) *refused = RunnerActionReason::kPureServeOnly;
      return false;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (next == RunnerMode::kMotion || next == RunnerMode::kServe) {
      const auto reason = PlayAdmissionReasonLocked_();
      if (reason != RunnerActionReason::kNone) {
        if (refused) *refused = reason;
        return false;
      }
    }
    // Publish the prepare edge only after admission, before exposing SERVE to
    // the command thread. A refused request must not leave a pending prepare.
    if (prepare_serve && next == RunnerMode::kServe) serve_prepare_pending_.store(true);
    if (next == RunnerMode::kMotion || next == RunnerMode::kServe) {
      mocap_interrupted_ = false;
      operator_interrupted_ = RunnerActionReason::kNone;
    }
    if (next == RunnerMode::kTeleop && mode() != next)
      teleop_stop_requested_.store(false);
    if (mode_.exchange(next, std::memory_order_acq_rel) != next) Touch_();
    return true;
  }

  // Wire request: [schema=2, request_id, action_code, reserved=0].
  bool EnqueueFlatRequest(const std::vector<double>& values) {
    std::uint64_t request_id = 0;
    std::uint64_t action_code = 0;
    const bool id_decoded =
        values.size() >= 2 &&
        IsExactFloatInteger(values[1], 1, kRunnerMaxExactFloatInteger,
                            &request_id);
    const bool action_decoded =
        values.size() >= 3 &&
        IsExactFloatInteger(
            values[2], 1,
            static_cast<std::uint64_t>(
                RunnerAction::kEnterTeleop),
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
    return Enqueue(
        {request_id, static_cast<RunnerAction>(action_code), true});
  }

  bool EnqueueLocalAction(RunnerAction action) {
    std::uint64_t id =
        local_request_id_.fetch_add(1, std::memory_order_relaxed);
    id &= (kRunnerMaxExactFloatInteger - 1);
    if (id == 0) id = 1;
    return Enqueue({id, action, false});
  }

  bool Enqueue(RunnerActionRequest request) {
    if (request.request_id == 0 ||
        request.request_id > kRunnerMaxExactFloatInteger ||
        request.action == RunnerAction::kNone ||
        static_cast<int>(request.action) >
            static_cast<int>(
                RunnerAction::kEnterTeleop) ||
        (request.remote && !IsRemoteRunnerAction(request.action))) {
      RecordResult_(request, RunnerActionResult::kInvalidRequest,
                    RunnerActionReason::kMalformedRequest);
      return false;
    }
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (queue_.size() >= queue_capacity_) {
      if (request.action == RunnerAction::kEmergencyPassive) {
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
    if (request.action == RunnerAction::kEmergencyPassive)
      queue_.push_front(request);
    else
      queue_.push_back(request);
    return true;
  }

  std::vector<RunnerActionDecision> ProcessPending(
      bool command_fault_latched, bool serve_active,
      bool serve_capability = false, int serve_state = -1,
      bool serve_gain_scales_nominal = false,
      int gripper_state = -1, bool cleanup_required = false,
      bool teleop_capable = false, bool teleop_input_ready = false) {
    std::deque<RunnerActionRequest> pending;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      pending.swap(queue_);
    }
    std::vector<RunnerActionDecision> decisions;
    decisions.reserve(pending.size());
    for (const RunnerActionRequest& request : pending) {
      decisions.push_back(Apply_(
          request, command_fault_latched, serve_active,
          serve_capability, serve_state, serve_gain_scales_nominal,
          gripper_state, cleanup_required, teleop_capable, teleop_input_ready));
    }
    return decisions;
  }

  void ObserveExternalState(bool command_publishing, bool policy_native,
                            bool command_fault_latched,
                            bool serve_capability, int serve_state,
                            int gripper_state = -1,
                            bool cleanup_required = false) noexcept {
    std::lock_guard<std::mutex> lock(state_mutex_);
    bool changed = false;
    changed |= ExchangeChanged_(command_publishing_, command_publishing);
    changed |= ExchangeChanged_(policy_native_, policy_native);
    changed |=
        ExchangeChanged_(command_fault_latched_, command_fault_latched);
    changed |= ExchangeChanged_(serve_capability_, serve_capability);
    changed |= ExchangeChanged_(serve_state_, serve_state);
    changed |= ExchangeChanged_(gripper_state_, gripper_state);
    changed |=
        ExchangeChanged_(serve_cleanup_required_, cleanup_required);
    if (changed) Touch_();
  }

  // Schema-2 state (21 doubles): schema, boot, seq, mode, publishing,
  // policy-native, fault, local role, role epoch, role-change allowed,
  // role result/reason, serve capability/state, gripper state,
  // cleanup-required, last action id/action/result/reason, session fingerprint.
  std::vector<double> EncodeState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const bool fault =
        command_fault_latched_.load(std::memory_order_acquire);
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
        static_cast<double>(
            serve_state_.load(std::memory_order_acquire)),
        static_cast<double>(
            gripper_state_.load(std::memory_order_acquire)),
        serve_cleanup_required_.load(std::memory_order_acquire) ? 1.0
                                                                : 0.0,
        static_cast<double>(
            last_action_id_.load(std::memory_order_acquire)),
        static_cast<double>(static_cast<int>(
            last_action_.load(std::memory_order_acquire))),
        static_cast<double>(static_cast<int>(
            last_action_result_.load(std::memory_order_acquire))),
        static_cast<double>(static_cast<int>(
            last_action_reason_.load(std::memory_order_acquire))),
        static_cast<double>(session_fingerprint_),
    };
  }

  // Separate read-only telemetry preserves the frozen 21-value schema-2 state.
  // Prefix: schema, boot, session, seq, mocap required/fresh/stops/interrupted.
  // Tail: held, real age ms, operator required/UI/computer, stop reason/count.
  std::vector<double> EncodeMocapSafety() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return {1., static_cast<double>(boot_id_), static_cast<double>(session_fingerprint_),
            static_cast<double>(state_sequence()), mocap_required_.load() ? 1. : 0.,
            mocap_fresh_.load() ? 1. : 0., static_cast<double>(mocap_stop_count_),
            mocap_interrupted_ ? 1. : 0., mocap_held_ ? 1. : 0., mocap_age_ms_,
            operator_required_ ? 1. : 0., operator_ui_fresh_ ? 1. : 0.,
            operator_computer_fresh_ ? 1. : 0., static_cast<double>(operator_interrupted_),
            static_cast<double>(operator_stop_count_)};
  }

 private:
  RunnerActionReason PlayAdmissionReasonLocked_() const noexcept {
    // HELD is a model-input annotation, not the 100 ms loss criterion. Preserve
    // real-fresh telemetry while admitting mode entry/completion until expiry.
    if (mocap_required_.load() && (!mocap_within_deadline_ || mocap_stop_requested_.load()))
      return RunnerActionReason::kMocapUnavailable;
    if (operator_required_ && operator_stop_requested_.load() &&
        operator_interrupted_ != RunnerActionReason::kNone) return operator_interrupted_;
    if (operator_required_ && !operator_ui_fresh_)
      return RunnerActionReason::kOperatorUiUnavailable;
    if (operator_required_ && !operator_computer_fresh_)
      return RunnerActionReason::kOperatorComputerUnavailable;
    return RunnerActionReason::kNone;
  }
  static constexpr int kServeIdle = 0;
  static constexpr int kServeWaitBallLoad = 3;
  static constexpr int kServeWaitReady = 5;
  static constexpr int kServeComplete = 12;
  static constexpr int kServeAborted = 15;
  static constexpr int kServeFault = 16;
  static constexpr int kServeWaitGripSecure = 18;
  static constexpr int kGripperGrabbed = 4;

  static std::uint64_t NormalizeExactId_(
      std::uint64_t value) noexcept {
    value &= (kRunnerMaxExactFloatInteger - 1);
    return value == 0 ? 1 : value;
  }

  template <typename T>
  static bool ExchangeChanged_(std::atomic<T>& target,
                               T next) noexcept {
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

  RunnerActionDecision Apply_(
      const RunnerActionRequest& request, bool command_fault_latched,
      bool serve_active, bool serve_capability, int serve_state,
      bool serve_gain_scales_nominal, int gripper_state,
      bool cleanup_required, bool teleop_capable, bool teleop_input_ready) {
    (void)gripper_state;
    (void)cleanup_required;
    // Retain the argument and wire result codes for compatibility with older
    // clients, but serve025 owns official Kp/Kd and ignores policy gain-scale
    // flags. Gain scale is therefore no longer an admission gate.
    (void)serve_gain_scales_nominal;
    RunnerActionDecision decision;
    decision.request = request;
    const RunnerMode current_mode = mode();

    auto reject_fault = [&]() {
      decision.result = RunnerActionResult::kRejectedRunnerFault;
      decision.reason = RunnerActionReason::kRunnerCommandFaultLatched;
    };
    auto reject_role = [&](RunnerActionReason reason) {
      decision.result = RunnerActionResult::kRejectedWrongRole;
      decision.reason = reason;
    };
    auto reject_unavailable = [&]() {
      decision.result = RunnerActionResult::kRejectedServeUnavailable;
      decision.reason = RunnerActionReason::kServeControllerUnavailable;
    };
    auto require_server = [&]() {
      if (local_role() == LocalRole::kServer) return true;
      reject_role(RunnerActionReason::kServerRoleRequired);
      return false;
    };

    RunnerActionReason play_reason;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      play_reason = PlayAdmissionReasonLocked_();
    }
    if (play_reason != RunnerActionReason::kNone &&
        (request.action == RunnerAction::kEnterMotion ||
         request.action == RunnerAction::kPrepareServe ||
         request.action == RunnerAction::kReadyToServe)) {
      decision.result = RunnerActionResult::kRejectedWrongMode;
      decision.reason = play_reason;
      RecordResult_(request, decision.result, decision.reason);
      return decision;
    }

    if (serve_only_ && (request.action == RunnerAction::kSetReceiver ||
                        request.action == RunnerAction::kEnterMotion ||
                        request.action == RunnerAction::kEnterShadow)) {
      decision.result = RunnerActionResult::kRejectedWrongMode;
      decision.reason = RunnerActionReason::kPureServeOnly;
      RecordResult_(request, decision.result, decision.reason);
      return decision;
    }
    switch (request.action) {
      case RunnerAction::kSetServer:
      case RunnerAction::kSetReceiver: {
        if (command_fault_latched) {
          reject_fault();
        } else if (serve_active) {
          decision.result = RunnerActionResult::kRejectedWrongMode;
          decision.reason = RunnerActionReason::kServeOwnsCommand;
        } else if (current_mode != RunnerMode::kPassive &&
                   current_mode != RunnerMode::kPdStand) {
          decision.result = RunnerActionResult::kRejectedWrongMode;
          decision.reason =
              RunnerActionReason::kRoleChangeRequiresPassiveOrStand;
        } else {
          const LocalRole requested =
              request.action == RunnerAction::kSetServer
                  ? LocalRole::kServer
                  : LocalRole::kReceiver;
          if (local_role() == requested) {
            decision.result = RunnerActionResult::kAlreadySet;
            decision.reason = RunnerActionReason::kRoleUnchanged;
          } else {
            {
              std::lock_guard<std::mutex> lock(state_mutex_);
              local_role_.store(requested, std::memory_order_release);
              role_epoch_.fetch_add(1, std::memory_order_acq_rel);
              Touch_();
            }
            decision.result = RunnerActionResult::kApplied;
            decision.reason = RunnerActionReason::kRoleChanged;
          }
        }
        break;
      }

      case RunnerAction::kEnterPdStand:
        if (current_mode == RunnerMode::kServe &&
            (serve_state == kServeFault || serve_state == kServeAborted)) {
          // The action worker cannot certify the current control sample.
          // Keep Stand available as a recovery action; the command thread
          // retains the existing validated recovery path.
          decision.result = RunnerActionResult::kAcceptedPending;
          decision.reason = RunnerActionReason::kServeRecoveryPending;
        } else if (current_mode == RunnerMode::kTeleop) {
          teleop_stop_requested_.store(true);
          decision.result = RunnerActionResult::kAcceptedPending;
          decision.reason = RunnerActionReason::kTeleopStopRequested;
          decision.request_teleop_stop = true;
        } else if (current_mode == RunnerMode::kServe && serve_active) {
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

      case RunnerAction::kEnterTeleop:
        if (command_fault_latched) {
          reject_fault();
        } else if (!teleop_capable) {
          decision.result = RunnerActionResult::kRejectedTeleop;
          decision.reason = RunnerActionReason::kTeleopUnavailable;
        } else if (current_mode == RunnerMode::kTeleop) {
          decision.result = RunnerActionResult::kAlreadySet;
          decision.reason = RunnerActionReason::kModeUnchanged;
        } else if (current_mode != RunnerMode::kPdStand || serve_active) {
          decision.result = RunnerActionResult::kRejectedWrongMode;
          decision.reason = RunnerActionReason::kPdStandRequired;
        } else if (!teleop_input_ready) {
          decision.result = RunnerActionResult::kRejectedTeleop;
          decision.reason = RunnerActionReason::kTeleopInputNotReady;
        } else {
          SetRuntimeMode(RunnerMode::kTeleop);
          decision.result = RunnerActionResult::kApplied;
          decision.reason = RunnerActionReason::kModeChanged;
        }
        break;

      case RunnerAction::kEnterMotion:
        if (command_fault_latched) {
          reject_fault();
        } else if (serve_active) {
          decision.result = RunnerActionResult::kRejectedServeActive;
          decision.reason = RunnerActionReason::kServeOwnsCommand;
        } else if (local_role() != LocalRole::kReceiver) {
          reject_role(RunnerActionReason::kReceiverRoleRequired);
        } else if (current_mode == RunnerMode::kMotion) {
          decision.result = RunnerActionResult::kAlreadySet;
          decision.reason = RunnerActionReason::kModeUnchanged;
        } else if (current_mode != RunnerMode::kPdStand) {
          decision.result = RunnerActionResult::kRejectedWrongMode;
          decision.reason = RunnerActionReason::kPdStandRequired;
        } else {
          if (SetRuntimeMode(RunnerMode::kMotion, false, &decision.reason)) {
            decision.result = RunnerActionResult::kApplied;
            decision.reason = RunnerActionReason::kModeChanged;
          } else {
            decision.result = RunnerActionResult::kRejectedWrongMode;
          }
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
        if (current_mode == RunnerMode::kTeleop) {
          decision.result = RunnerActionResult::kRejectedWrongMode;
          decision.reason = RunnerActionReason::kPdStandRequired;
        } else if (serve_active) {
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

      case RunnerAction::kPrepareServe:
        if (command_fault_latched) {
          reject_fault();
        } else if (!serve_capability) {
          reject_unavailable();
        } else if (!require_server()) {
        } else if (serve_state == kServeFault) {
          decision.result =
              RunnerActionResult::kRejectedServeNotReady;
          decision.reason = RunnerActionReason::kServeFaultLatched;
        } else if (serve_active) {
          decision.result = RunnerActionResult::kRejectedServeActive;
          decision.reason = RunnerActionReason::kServeOwnsCommand;
        } else if (current_mode != RunnerMode::kPdStand &&
                   current_mode != RunnerMode::kMotion) {
          decision.result = RunnerActionResult::kRejectedWrongMode;
          decision.reason = RunnerActionReason::kPdStandRequired;
        } else if (serve_state != kServeIdle &&
                   serve_state != kServeComplete &&
                   serve_state != kServeAborted) {
          decision.result =
              RunnerActionResult::kRejectedServeNotReady;
          decision.reason = RunnerActionReason::kServePhaseMismatch;
        } else {
          if (SetRuntimeMode(RunnerMode::kServe, true, &decision.reason)) {
            decision.result = RunnerActionResult::kApplied;
            decision.reason = RunnerActionReason::kServePrepareRequested;
            decision.request_prepare_serve = true;
          } else {
            decision.result = RunnerActionResult::kRejectedWrongMode;
          }
        }
        break;

      case RunnerAction::kConfirmBallLoaded:
        if (command_fault_latched) {
          reject_fault();
        } else if (!serve_capability) {
          reject_unavailable();
        } else if (!require_server()) {
        } else if (current_mode == RunnerMode::kServe &&
                   serve_state == kServeWaitReady) {
          decision.result = RunnerActionResult::kAlreadySet;
          decision.reason =
              RunnerActionReason::kBallLoadMergedIntoPrepare;
        } else if (current_mode != RunnerMode::kServe ||
                   serve_state != kServeWaitBallLoad) {
          decision.result =
              RunnerActionResult::kRejectedServeNotReady;
          decision.reason = RunnerActionReason::kServePhaseMismatch;
        } else {
          decision.result = RunnerActionResult::kAcceptedPending;
          decision.reason =
              RunnerActionReason::kBallLoadedConfirmRequested;
          decision.request_confirm_ball_loaded = true;
        }
        break;

      case RunnerAction::kConfirmGripSecure:
        if (command_fault_latched) {
          reject_fault();
        } else if (!serve_capability) {
          reject_unavailable();
        } else if (!require_server()) {
        } else if (current_mode != RunnerMode::kServe ||
                   (serve_state != kServeWaitGripSecure &&
                    serve_state != kServeWaitReady)) {
          decision.result =
              RunnerActionResult::kRejectedServeNotReady;
          decision.reason = RunnerActionReason::kServePhaseMismatch;
        } else {
          decision.result = RunnerActionResult::kAcceptedPending;
          decision.reason =
              RunnerActionReason::kGripSecureConfirmRequested;
          decision.request_confirm_grip_secure = true;
        }
        break;

      case RunnerAction::kReadyToServe:
        if (command_fault_latched) {
          reject_fault();
        } else if (!serve_capability) {
          reject_unavailable();
        } else if (!require_server()) {
        } else if (current_mode != RunnerMode::kServe ||
                   serve_state != kServeWaitReady) {
          decision.result =
              RunnerActionResult::kRejectedServeNotReady;
          decision.reason = RunnerActionReason::kServePhaseMismatch;
        } else {
          decision.result = RunnerActionResult::kAcceptedPending;
          decision.reason =
              RunnerActionReason::kReadyToServePlayRequested;
          decision.request_ready_to_serve = true;
        }
        break;

      case RunnerAction::kOpenGripper:
        if (!serve_capability) {
          reject_unavailable();
        } else if (!require_server()) {
        } else if (current_mode != RunnerMode::kPdStand || serve_active) {
          decision.result = RunnerActionResult::kRejectedWrongMode;
          decision.reason = RunnerActionReason::kPdStandRequired;
        } else {
          decision.result = RunnerActionResult::kAcceptedPending;
          decision.reason = RunnerActionReason::kGripperOpenRequested;
          decision.request_open_gripper = true;
        }
        break;

      case RunnerAction::kConfirmLoadingZoneClear:
        // Frozen action code 11 is retained for old wire clients only.  The
        // current serve flow never enters ABORT_WAIT_HUMAN_CLEAR and exposes
        // no loading-zone-clear operation.
        decision.result = RunnerActionResult::kAlreadySet;
        decision.reason = RunnerActionReason::kLoadingZoneClearRemoved;
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
  std::atomic<bool> mocap_required_{false};
  std::atomic<bool> mocap_fresh_{true};
  bool mocap_within_deadline_ = true;  // state_mutex_; distinct from non-held telemetry
  std::atomic<bool> mocap_stop_requested_{false};
  std::atomic<bool> operator_stop_requested_{false};
  bool operator_required_ = false, operator_ui_fresh_ = true, operator_computer_fresh_ = true;
  RunnerActionReason operator_interrupted_ = RunnerActionReason::kNone;
  std::uint64_t operator_stop_count_ = 0;
  bool mocap_held_ = false;
  double mocap_age_ms_ = -1.;
  // Guarded by state_mutex_; not operator action results.
  bool mocap_interrupted_{false};
  std::uint64_t mocap_stop_count_{0};
  std::atomic<bool> teleop_stop_requested_{false};
  std::atomic<bool> serve_prepare_pending_{false};
  const std::uint64_t boot_id_;
  const std::uint64_t session_fingerprint_;
  const std::size_t queue_capacity_;
  const bool serve_only_;
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
  std::atomic<int> gripper_state_{-1};
  std::atomic<bool> serve_cleanup_required_{false};
  std::atomic<RunnerActionResult> role_last_result_{
      RunnerActionResult::kNone};
  std::atomic<RunnerActionReason> role_last_reason_{
      RunnerActionReason::kNone};
  std::atomic<std::uint64_t> last_action_id_{0};
  std::atomic<RunnerAction> last_action_{RunnerAction::kNone};
  std::atomic<RunnerActionResult> last_action_result_{
      RunnerActionResult::kNone};
  std::atomic<RunnerActionReason> last_action_reason_{
      RunnerActionReason::kNone};
};

}  // namespace a3_pingpong
