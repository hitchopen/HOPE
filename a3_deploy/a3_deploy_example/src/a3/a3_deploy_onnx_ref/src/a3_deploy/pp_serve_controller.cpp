#include "a3_pingpong/pp_serve_controller.hpp"

#include "a3_deploy/a3_joint_limits.hpp"
#include "a3_deploy/numeric_safety.hpp"
#include "a3_pingpong/pp_frame_math.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <utility>

namespace a3_pingpong {
namespace {

constexpr int kDof = robot_io::kA3Dof;
constexpr double kStandEndpointToleranceRad = 1.0e-6;
constexpr double kTrackingWarningRad = 0.35;
constexpr double kPrepareStandSeconds = 1.0;
constexpr int kWaistYawSdk = 0;
constexpr int kWaistRollSdk = 1;
constexpr int kWaistPitchSdk = 2;
constexpr int kLeftHipPitchSdk = 19;
constexpr int kLeftKneePitchSdk = 22;
constexpr int kLeftAnklePitchSdk = 23;
constexpr int kRightHipPitchSdk = 25;
constexpr int kRightKneePitchSdk = 28;
constexpr int kRightAnklePitchSdk = 29;

bool AllFinite(const Eigen::VectorXd& values) {
  for (Eigen::Index index = 0; index < values.size(); ++index) {
    if (!a3_deploy::numeric_safety::IsFinite(values[index])) return false;
  }
  return true;
}

robot_io::RobotCommand ZeroGainHold(const Eigen::VectorXd& q) {
  robot_io::RobotCommand command;
  command.q_des = q;
  command.dq_des = Eigen::VectorXd::Zero(kDof);
  command.tau_ff = Eigen::VectorXd::Zero(kDof);
  command.kp = Eigen::VectorXd::Zero(kDof);
  command.kd = Eigen::VectorXd::Zero(kDof);
  return command;
}

}  // namespace

const char* ServeControllerStateName(ServeControllerState state) noexcept {
  switch (state) {
    case ServeControllerState::kIdle: return "IDLE";
    case ServeControllerState::kPreparingStand: return "PREPARING_STAND";
    case ServeControllerState::kTransitionToLoad: return "TRANSITION_TO_LOAD";
    case ServeControllerState::kWaitBallLoad: return "WAIT_BALL_LOAD";
    case ServeControllerState::kGripClosing: return "GRIP_CLOSING";
    case ServeControllerState::kWaitReadyToServe: return "WAIT_READY_TO_SERVE";
    case ServeControllerState::kPlayingPreRelease: return "PLAYING_PRE_RELEASE";
    case ServeControllerState::kReleasePending: return "RELEASE_PENDING";
    case ServeControllerState::kStrike: return "STRIKE";
    case ServeControllerState::kFollowThrough: return "FOLLOW_THROUGH";
    case ServeControllerState::kRecovery: return "RECOVERY";
    case ServeControllerState::kHandoffReady: return "HANDOFF_READY";
    case ServeControllerState::kComplete: return "COMPLETE";
    case ServeControllerState::kAbortWaitHumanClear:
      return "ABORT_WAIT_HUMAN_CLEAR";
    case ServeControllerState::kAbortReturn: return "ABORT_RETURN";
    case ServeControllerState::kAborted: return "ABORTED";
    case ServeControllerState::kFault: return "FAULT";
    case ServeControllerState::kCleanupOpening: return "CLEANUP_OPENING";
    case ServeControllerState::kWaitGripSecure: return "WAIT_GRIP_SECURE";
  }
  return "UNKNOWN";
}

const char* ServeGripperStateName(ServeGripperState state) noexcept {
  switch (state) {
    case ServeGripperState::kUnavailable: return "UNAVAILABLE";
    case ServeGripperState::kUnknown: return "UNKNOWN";
    case ServeGripperState::kOpening: return "OPENING";
    case ServeGripperState::kOpen: return "OPEN";
    case ServeGripperState::kClosing: return "CLOSING";
    case ServeGripperState::kGrabbed: return "GRABBED";
    case ServeGripperState::kReleasing: return "RELEASING";
    case ServeGripperState::kReleased: return "RELEASED";
    case ServeGripperState::kFault: return "FAULT";
  }
  return "UNKNOWN";
}

PpServeController::PpServeController(
    PpServe025FullbodyTimeline timeline,
    const Eigen::VectorXd& official_stand_q_sdk,
    const Eigen::VectorXd& official_stand_kp_sdk,
    const Eigen::VectorXd& official_stand_kd_sdk,
    std::unique_ptr<PpGripperWorker> gripper)
    : timeline_(std::move(timeline)),
      official_stand_q_sdk_(official_stand_q_sdk),
      official_stand_kp_sdk_(official_stand_kp_sdk),
      official_stand_kd_sdk_(official_stand_kd_sdk),
      gripper_(std::move(gripper)) {
  if (timeline_.empty() || timeline_.size() != kServe025FullbodyFrames) {
    throw std::runtime_error(
        "serve025 controller requires the complete 468-frame full31 timeline");
  }
  if (official_stand_q_sdk_.size() != kDof ||
      official_stand_kp_sdk_.size() != kDof ||
      official_stand_kd_sdk_.size() != kDof ||
      !AllFinite(official_stand_q_sdk_) ||
      !AllFinite(official_stand_kp_sdk_) ||
      !AllFinite(official_stand_kd_sdk_)) {
    throw std::runtime_error(
        "serve025 controller requires finite 31-D official PD_STAND vectors");
  }
  for (int sdk = 0; sdk < kDof; ++sdk) {
    if (official_stand_kp_sdk_[sdk] <= 0.0 ||
        official_stand_kd_sdk_[sdk] <= 0.0) {
      throw std::runtime_error(
          "serve025 controller requires positive PD gains for all SDK31 joints");
    }
    if (std::abs(official_stand_q_sdk_[sdk] -
                 timeline_.complete().q_sdk[sdk]) >
        kStandEndpointToleranceRad) {
      throw std::runtime_error(
          "model stand pose does not match serve025 CSV COMPLETE frame");
    }
  }
  FillPdCommand_(official_stand_q_sdk_, Eigen::VectorXd::Zero(kDof),
                 last_command_);
  if (!gripper_ || !gripper_->Snapshot().running) {
    gripper_state_.store(ServeGripperState::kUnavailable,
                         std::memory_order_release);
  }
}

PpServeController::~PpServeController() {
  if (gripper_) gripper_->Stop();
}

void PpServeController::SetPolicyHandoffFrame(std::size_t frame) {
  if (frame <= kServe025ContactFrame || frame > kServe025CompleteFrame || active())
    throw std::invalid_argument("serve policy handoff must follow contact and be configured while idle");
  policy_handoff_frame_ = frame;
}

void PpServeController::Start() {
  start_requested_.store(true, std::memory_order_release);
}

void PpServeController::SetPolicyReturnSeconds(double seconds) {
  if (!std::isfinite(seconds) || seconds <= 0 || active())
    throw std::invalid_argument("serve return duration must be positive and configured while idle");
  policy_return_seconds_ = seconds;
}

void PpServeController::SetReleaseLeadFrames(std::size_t frames) {
  if (frames > 3 || active())
    throw std::invalid_argument("serve release lead must be 0..3 frames and configured while idle");
  release_lead_frames_ = frames;
}

void PpServeController::ConfirmBallLoaded() {
  // Retained for schema compatibility. The new flow has no Load Ball step.
}

void PpServeController::ConfirmGripSecure() {
  // Gripper receipts remain telemetry and never gate the command clock.
}

void PpServeController::TriggerReadyToServe() {
  ready_to_serve_requested_.store(true, std::memory_order_release);
}

void PpServeController::SetNativeReleasePublisher(NativeReleasePublisher publisher) {
  if (state() != ServeControllerState::kIdle || !publisher)
    throw std::invalid_argument("native release publisher must be configured while idle");
  native_release_publisher_ = std::move(publisher);
}

void PpServeController::RequestAbort() {
  abort_requested_.store(true, std::memory_order_release);
}

void PpServeController::EmergencyStop() {
  std::lock_guard<std::mutex> lock(async_mutex_);
  emergency_stop_requested_.store(true, std::memory_order_release);
  prepare_grab_pending_ = false;
  fault_reason_ = "emergency passive selected during serve025";
  state_.store(ServeControllerState::kFault, std::memory_order_release);
}

bool PpServeController::RequestOpenGripper(std::string& error) {
  std::lock_guard<std::mutex> lock(async_mutex_);
  if (active()) {
    error = "manual OPEN is unavailable while serve025 owns q_des";
    return false;
  }
  if (!SubmitGripper_(PpGripperCommand::kOpen,
                      ServeGripperState::kOpening, SteadyNowNs_())) {
    error = gripper_fault_reason_.empty()
                ? "gripper command transport is unavailable"
                : gripper_fault_reason_;
    return false;
  }
  cleanup_required_.store(false, std::memory_order_release);
  error.clear();
  return true;
}

void PpServeController::PollAsync() {
  std::lock_guard<std::mutex> lock(async_mutex_);
  PollGripper_();
}

void PpServeController::SetEntryCommand(
    const robot_io::RobotCommand& command) {
  std::lock_guard<std::mutex> lock(async_mutex_);
  if (!PpCommandTransition::Valid(command) || command.q_des.size() != kDof)
    throw std::invalid_argument("invalid Serve entry command");
  entry_command_ = command;
  entry_command_valid_ = true;
}

std::uint64_t PpServeController::SteadyNowNs_() noexcept {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

bool PpServeController::ValidateState_(
    const robot_io::RobotState& state) {
  if (state.q.size() != kDof || state.dq.size() != kDof) {
    TripFault_("robot state DOF mismatch");
    return false;
  }
  if (!AllFinite(state.q) || !AllFinite(state.dq) ||
      !state.imu_quat_wxyz.allFinite() || !state.imu_gyro.allFinite() ||
      state.imu_quat_wxyz.norm() < 1.0e-6) {
    TripFault_("robot state contains non-finite joint/IMU data");
    return false;
  }
  if (!state.sync_complete) {
    TripFault_("robot state is incomplete");
    return false;
  }
  for (int sdk = 0; sdk < kDof; ++sdk) {
    if (state.q[sdk] <
            a3_deploy::kA3SdkJointPosLo[static_cast<std::size_t>(sdk)] -
                0.03 ||
        state.q[sdk] >
            a3_deploy::kA3SdkJointPosHi[static_cast<std::size_t>(sdk)] +
                0.03) {
      TripFault_("actual_q violates an A3 hard joint limit");
      return false;
    }
  }
  return true;
}

void PpServeController::ResetAction_() {
  entry_transition_.Reset();
  replay_entry_transition_.Reset();
  replay_entry_tick_ = kServeReplaySettleTicks;
  prepare_balance_rad_ = 0.0;
  prepare_balance_velocity_ = 0.0;
  policy_return_transition_.Reset();
  policy_return_tick_ = 0;
  policy_return_quiet_ticks_ = 0;
  transition_tick_ = 0;
  timeline_tick_ = 0;
  source_frame_ = 0;
  phase_ = "PD_STAND_TO_CSV_READY";
  prepare_grab_pending_ = true;
  prepare_open_submitted_ = false;
  release_request_submitted_ = false;
  native_release_copies_left_ = 0;
  release_clock_rebase_ns_.store(0, std::memory_order_release);
  release_acknowledged_ = false;
  release_dispatch_monotonic_ns_ = 0;
  release_first_publish_monotonic_ns_ = 0;
  fault_reason_.clear();
  gripper_fault_reason_.clear();
  cleanup_required_.store(false, std::memory_order_release);
  ready_to_serve_requested_.store(false, std::memory_order_release);
  abort_requested_.store(false, std::memory_order_release);
  ankle_pitch_offset_rad_ = 0.0;
  hip_pitch_offset_rad_ = 0.0;
  knee_pitch_offset_rad_ = 0.0;
  waist_roll_offset_rad_ = 0.0;
  waist_pitch_offset_rad_ = 0.0;
  sagittal_support_error_rad_ = 0.0;
  max_waist_error_rad_ = 0.0;
}

void PpServeController::PollGripper_() {
  if (!gripper_) {
    gripper_state_.store(ServeGripperState::kUnavailable,
                         std::memory_order_release);
    return;
  }
  const PpGripperWorkerSnapshot snapshot = gripper_->Snapshot();
  if (snapshot.faulted) {
    gripper_fault_reason_ = snapshot.fault_reason;
    gripper_state_.store(ServeGripperState::kFault,
                         std::memory_order_release);
    return;
  }
  if (!snapshot.running) {
    gripper_fault_reason_ = "gripper worker is not running";
    gripper_state_.store(ServeGripperState::kUnavailable,
                         std::memory_order_release);
    prepare_grab_pending_ = false;
    return;
  }
  if (snapshot.acknowledged_generation == 0 ||
      snapshot.acknowledged_generation <= acknowledged_gripper_generation_) {
    return;
  }
  acknowledged_gripper_generation_ = snapshot.acknowledged_generation;
  switch (snapshot.acknowledged_command) {
    case PpGripperCommand::kOpen:
      gripper_state_.store(ServeGripperState::kOpen,
                           std::memory_order_release);
      cleanup_required_.store(false, std::memory_order_release);
      break;
    case PpGripperCommand::kGrab:
      gripper_state_.store(ServeGripperState::kGrabbed,
                           std::memory_order_release);
      cleanup_required_.store(true, std::memory_order_release);
      break;
    case PpGripperCommand::kRelease:
      gripper_state_.store(ServeGripperState::kReleased,
                           std::memory_order_release);
      release_acknowledged_ = true;
      release_first_publish_monotonic_ns_ =
          snapshot.first_publish_monotonic_ns;
      cleanup_required_.store(false, std::memory_order_release);
      break;
    case PpGripperCommand::kStatus:
      break;
  }
}

bool PpServeController::SubmitGripper_(
    PpGripperCommand command, ServeGripperState pending_state,
    std::uint64_t now_ns) {
  if (command == PpGripperCommand::kRelease && native_release_publisher_) {
    release_dispatch_monotonic_ns_ = now_ns;
    std::uint64_t published_ns = 0;
    if (!native_release_publisher_(published_ns) || published_ns == 0) {
      gripper_fault_reason_ = "native release publication failed";
      gripper_state_.store(ServeGripperState::kFault, std::memory_order_release);
      return false;
    }
    release_first_publish_monotonic_ns_ = published_ns;
    release_acknowledged_ = true;  // Publication only, never physical release.
    gripper_state_.store(ServeGripperState::kReleased, std::memory_order_release);
    cleanup_required_.store(false, std::memory_order_release);
    native_release_copies_left_ = 4;
    native_release_next_frame_ = release_frame() + 10;
    release_clock_rebase_ns_.store(published_ns, std::memory_order_release);
    return true;
  }
  if (!gripper_) {
    gripper_state_.store(ServeGripperState::kUnavailable,
                         std::memory_order_release);
    gripper_fault_reason_ = "gripper command transport is unavailable";
    return false;
  }
  std::string error;
  const std::uint64_t generation = gripper_generation_ + 1;
  if (!gripper_->Submit(command, generation, error)) {
    gripper_fault_reason_ = error;
    return false;
  }
  gripper_generation_ = generation;
  gripper_state_.store(pending_state, std::memory_order_release);
  if (command == PpGripperCommand::kRelease) {
    release_dispatch_monotonic_ns_ = now_ns;
  }
  return true;
}

void PpServeController::AdvancePrepareGripper_(std::uint64_t now_ns) {
  if (!prepare_grab_pending_) return;
  PollGripper_();
  const ServeGripperState current = gripper_state();
  switch (current) {
    case ServeGripperState::kUnknown:
    case ServeGripperState::kReleased:
      // Reproduce the c89e5ab hardware-proven OPEN -> GRAB command sequence.
      // It runs asynchronously during the transition to CSV frame 0 and never
      // pauses or gates the body command clock.
      prepare_open_submitted_ = SubmitGripper_(
          PpGripperCommand::kOpen, ServeGripperState::kOpening, now_ns);
      if (!prepare_open_submitted_) prepare_grab_pending_ = false;
      return;
    case ServeGripperState::kOpen:
      (void)SubmitGripper_(PpGripperCommand::kGrab,
                           ServeGripperState::kClosing, now_ns);
      return;
    case ServeGripperState::kGrabbed:
      prepare_grab_pending_ = false;
      cleanup_required_.store(true, std::memory_order_release);
      return;
    case ServeGripperState::kOpening:
    case ServeGripperState::kClosing:
    case ServeGripperState::kReleasing:
      return;
    case ServeGripperState::kUnavailable:
      prepare_grab_pending_ = false;
      return;
    case ServeGripperState::kFault:
      // A new PREPARE_SERVE is an independent best-effort actuator edge. Retry
      // OPEN once so a stale RELEASE/transport receipt fault cannot suppress
      // the next Raise hand sequence. A failure in this new attempt remains
      // non-gating and is not retried every control tick.
      if (!prepare_open_submitted_) {
        prepare_open_submitted_ = SubmitGripper_(
            PpGripperCommand::kOpen, ServeGripperState::kOpening, now_ns);
      } else {
        prepare_grab_pending_ = false;
      }
      if (!prepare_open_submitted_) prepare_grab_pending_ = false;
      return;
  }
}

void PpServeController::ProcessRequests_(
    const robot_io::RobotState& /*state*/, std::uint64_t now_ns) {
  if (emergency_stop_requested_.exchange(false,
                                          std::memory_order_acq_rel)) {
    TripFault_("emergency passive selected during serve025");
    return;
  }
  if (start_requested_.exchange(false, std::memory_order_acq_rel)) {
    const ServeControllerState current = state();
    if (current == ServeControllerState::kIdle ||
        current == ServeControllerState::kComplete ||
        current == ServeControllerState::kAborted) {
      ResetAction_();
      robot_io::RobotCommand stand;
      FillPdCommand_(official_stand_q_sdk_, Eigen::VectorXd::Zero(kDof), stand);
      const bool recover_stand = entry_command_valid_ &&
          ((entry_command_.q_des - stand.q_des).cwiseAbs().maxCoeff() > 1e-6 ||
           entry_command_.dq_des.cwiseAbs().maxCoeff() > 1e-6 ||
           entry_command_.tau_ff.cwiseAbs().maxCoeff() > 1e-6 ||
           (entry_command_.kp - stand.kp).cwiseAbs().maxCoeff() > 1e-6 ||
           (entry_command_.kd - stand.kd).cwiseAbs().maxCoeff() > 1e-6);
      // The receive actor's balancing pose/gains are not a static support pose
      // to hold during a long arm raise. Recover official stand first, within
      // this same PREPARE request, then run the original loading transition.
      if (recover_stand)
        entry_transition_.Begin(entry_command_, stand, kPrepareStandSeconds, .25);
      state_.store(recover_stand ? ServeControllerState::kPreparingStand
                                : ServeControllerState::kTransitionToLoad,
                   std::memory_order_release);
      AdvancePrepareGripper_(now_ns);
    }
  }
  if (abort_requested_.exchange(false, std::memory_order_acq_rel) &&
      active()) {
    prepare_grab_pending_ = false;
    phase_ = "ABORTED";
    state_.store(ServeControllerState::kAborted,
                 std::memory_order_release);
    return;
  }
  AdvancePrepareGripper_(now_ns);
  if (state() == ServeControllerState::kWaitReadyToServe &&
      ready_to_serve_requested_.exchange(false,
                                          std::memory_order_acq_rel)) {
    timeline_tick_ = 0;
    source_frame_ = 0;
    swing_balance_rad_ = replay_support_rad_;
    swing_balance_velocity_ = 0.;
    robot_io::RobotCommand target;
    FillPdCommand_(timeline_.ready().q_sdk, timeline_.ready().qd_sdk, target);
    for (const int sdk : {kLeftAnklePitchSdk, kRightAnklePitchSdk})
      target.q_des[sdk] += replay_support_rad_;
    replay_entry_transition_.Begin(last_command_, target, .5, .25);
    replay_entry_tick_ = 0;
    const auto sequence = playback_.sequence + 1;
    playback_ = {};
    playback_.sequence = sequence;
    playback_.start_support_rad = swing_balance_rad_;
    playback_.start_support_velocity = swing_balance_velocity_;
    phase_ = timeline_.ready().phase;
    state_.store(ServeControllerState::kPlayingPreRelease,
                 std::memory_order_release);
  } else if (state() != ServeControllerState::kWaitReadyToServe) {
    ready_to_serve_requested_.store(false, std::memory_order_release);
  }
}

double PpServeController::Smooth5_(double value) noexcept {
  const double x = std::clamp(value, 0.0, 1.0);
  return x * x * x * (10.0 + x * (-15.0 + 6.0 * x));
}

double PpServeController::Smooth5Derivative_(double value) noexcept {
  const double x = std::clamp(value, 0.0, 1.0);
  return 30.0 * x * x * (1.0 - x) * (1.0 - x);
}

void PpServeController::FillPdCommand_(
    const Eigen::VectorXd& q_des, const Eigen::VectorXd& dq_des,
    robot_io::RobotCommand& command) const {
  command.q_des = q_des;
  command.dq_des = dq_des;
  command.tau_ff = Eigen::VectorXd::Zero(kDof);
  command.kp = official_stand_kp_sdk_;
  command.kd = official_stand_kd_sdk_;
}

void PpServeController::SetTransitionCommand_(
    std::size_t tick, const robot_io::RobotState& state,
    robot_io::RobotCommand& command) {
  constexpr double kDurationS =
      static_cast<double>(kServe025FullbodyTransitionTicks - 1) /
      kServe025FullbodyRunnerHz;
  const double alpha =
      static_cast<double>(tick) /
      static_cast<double>(kServe025FullbodyTransitionTicks - 1);
  const double position_alpha = Smooth5_(alpha);
  const double velocity_alpha = Smooth5Derivative_(alpha) / kDurationS;
  const Eigen::VectorXd source_q =
      entry_command_valid_ ? entry_command_.q_des : official_stand_q_sdk_;
  const Eigen::VectorXd delta = timeline_.ready().q_sdk - source_q;
  if (entry_command_valid_) {
    FillPdCommand_(timeline_.ready().q_sdk, Eigen::VectorXd::Zero(kDof), command);
    // Preserve the current command exactly, then settle its residual velocity
    // in 250 ms while the loading pose/gains still blend over the full 5 s.
    // Using the full 5 s for both can overshoot when Ready's blend is interrupted.
    if (tick == 0) entry_transition_.Begin(entry_command_, command, kDurationS, .25);
    entry_transition_.Apply(alpha * kDurationS, command);
  } else {
    FillPdCommand_(source_q + position_alpha * delta,
                   velocity_alpha * delta, command);
  }
  // Fold the elbows before the shoulders finish rising. Synchronous joint
  // interpolation passes through an extended-arm pose with a larger forward
  // moment. Each segment has zero endpoint velocity/acceleration, and the
  // original entry-command velocity/gains/feedforward blend stays intact.
  for (const int sdk : {5, 6, 8, 12, 13, 15}) {
    const bool elbow = sdk == 8 || sdk == 15;
    const double start = elbow ? 0.0 : .15;
    const double span = elbow ? .65 : .85;
    const double u = std::clamp((alpha - start) / span, 0.0, 1.0);
    command.q_des[sdk] += (Smooth5_(u) - position_alpha) * delta[sdk];
    command.dq_des[sdk] +=
        (Smooth5Derivative_(u) / (span * kDurationS) - velocity_alpha) * delta[sdk];
  }
  // Transient symmetric ankle support during the arm lift. This C2 bump
  // vanishes, including its first two derivatives, at both ends; it never
  // modifies CSV READY or any played frame. Peak offset is 0.015 rad.
  const double u = std::clamp(alpha, 0.0, 1.0);
  const double v = 1.0 - u;
  const double support_q = .015 * 64.0 * u*u*u * v*v*v;
  const double support_dq = .015 * 192.0 * u*u * v*v * (1.0 - 2.0*u) / kDurationS;
  for (const int sdk : {kLeftAnklePitchSdk, kRightAnklePitchSdk}) {
    command.q_des[sdk] += support_q;
    command.dq_des[sdk] += support_dq;
  }
  // Keep support continuous into READY; dropping the correction after the
  // lift left the physical robot tipping backward while the arms were held.
  const double elapsed = alpha * kDurationS;
  ApplyPitchSupport_(state, command, Smooth5_(elapsed / .25),
                     Smooth5Derivative_(elapsed / .25) / .25);
  UpdateSupportTelemetry_(state, command);
  last_command_ = command;
  phase_ = "PD_STAND_TO_CSV_READY";
}

void PpServeController::ApplyPitchSupport_(
    const robot_io::RobotState& state, robot_io::RobotCommand& command,
    double envelope, double envelope_velocity) {
  const auto gravity = projected_gravity_body(state.imu_quat_wxyz.normalized());
  const double pitch = std::asin(std::clamp(gravity[0], -1.0, 1.0));
  const double target = std::clamp(.9 * pitch + .2 * state.imu_gyro[1], -.08, .08);
  const double delta = std::clamp(target - prepare_balance_rad_, -.0012, .0012);
  prepare_balance_rad_ += delta;
  prepare_balance_velocity_ = delta * kServe025FullbodyRunnerHz;
  for (const int sdk : {kLeftAnklePitchSdk, kRightAnklePitchSdk}) {
    command.q_des[sdk] += envelope * prepare_balance_rad_;
    command.dq_des[sdk] += envelope_velocity * prepare_balance_rad_ +
        envelope * delta * kServe025FullbodyRunnerHz;
  }
}

void PpServeController::SetTimelineCommand_(
    std::size_t frame, const robot_io::RobotState& state,
    robot_io::RobotCommand& command) {
  const Serve025FullbodyFrame& value = timeline_.At(frame);
  FillPdCommand_(value.q_sdk, value.qd_sdk, command);
  // Only sagittal ankle support is corrected. Arm path, CSV clock, release,
  // leg width and gains stay unchanged. The full-command return captures this
  // correction at the early policy handoff. Full-length playback fades it
  // during the final second so COMPLETE still matches official Stand exactly.
  const double remaining = static_cast<double>(kServe025CompleteFrame - frame) /
      kServe025FullbodyRunnerHz;
  if (this->state() == ServeControllerState::kWaitReadyToServe ||
      this->state() == ServeControllerState::kWaitBallLoad ||
      this->state() == ServeControllerState::kGripClosing ||
      this->state() == ServeControllerState::kWaitGripSecure) {
    ApplyPitchSupport_(state, command, 1., 0.);
  } else {
    // Hold the settled support offset during the short swing. Reacting to the
    // intentional swing pitch changes the release-to-racket interception.
    // Preserve support velocity on frame zero and settle it in 250 ms, using
    // the same source-velocity basis as the full-command handoff.
    const double u = std::clamp(static_cast<double>(frame) /
        (kServe025FullbodyRunnerHz * .25), 0., 1.);
    const double v = .25 * (u - 6*u*u*u + 8*u*u*u*u - 3*u*u*u*u*u);
    const double dv = 1 - 18*u*u + 32*u*u*u - 15*u*u*u*u;
    double offset = swing_balance_rad_ + v * swing_balance_velocity_;
    double velocity = dv * swing_balance_velocity_;
    // Restore feedback after contact while the arms recover, smoothly joining
    // the frozen strike support. A constant strike offset is unsuitable once
    // the upper-body mass returns to Stand.
    // Keep the same strike and complete follow-through in both Kernel and
    // early policy-handoff modes. Resume support when CSV recovery begins;
    // enabling full replay must not reintroduce IMU feedback mid-swing.
    const std::size_t kResumeFrame = policy_handoff_frame_ < kServe025CompleteFrame
        ? policy_handoff_frame_ + 1 : kServe025RecoveryFrame;
    if (frame >= kResumeFrame) {
      const auto gravity = projected_gravity_body(state.imu_quat_wxyz.normalized());
      const double pitch = std::asin(std::clamp(gravity[0], -1., 1.));
      const double target = std::clamp(.9 * pitch + .2 * state.imu_gyro[1], -.08, .08);
      const double delta = std::clamp(target - prepare_balance_rad_, -.0012, .0012);
      prepare_balance_rad_ += delta;
      const double t = static_cast<double>(frame - kResumeFrame) /
          (kServe025FullbodyRunnerHz * .25);
      offset = swing_balance_rad_ + Smooth5_(t) * (prepare_balance_rad_ - swing_balance_rad_);
      velocity = Smooth5Derivative_(t) / .25 * (prepare_balance_rad_ - swing_balance_rad_) +
          Smooth5_(t) * delta * kServe025FullbodyRunnerHz;
    }
    for (const int sdk : {kLeftAnklePitchSdk, kRightAnklePitchSdk}) {
      command.q_des[sdk] += Smooth5_(remaining) * offset;
      command.dq_des[sdk] += Smooth5_(remaining) * velocity -
          Smooth5Derivative_(remaining) * offset;
    }
  }
  UpdateSupportTelemetry_(state, command);
  last_command_ = command;
  phase_ = value.phase;
}

void PpServeController::UpdateSupportTelemetry_(
    const robot_io::RobotState& state,
    const robot_io::RobotCommand& desired) {
  const double desired_left =
      desired.q_des[kLeftHipPitchSdk] + desired.q_des[kLeftKneePitchSdk] +
      desired.q_des[kLeftAnklePitchSdk];
  const double actual_left =
      state.q[kLeftHipPitchSdk] + state.q[kLeftKneePitchSdk] +
      state.q[kLeftAnklePitchSdk];
  const double desired_right =
      desired.q_des[kRightHipPitchSdk] + desired.q_des[kRightKneePitchSdk] +
      desired.q_des[kRightAnklePitchSdk];
  const double actual_right =
      state.q[kRightHipPitchSdk] + state.q[kRightKneePitchSdk] +
      state.q[kRightAnklePitchSdk];
  sagittal_support_error_rad_ =
      0.5 * ((desired_left - actual_left) +
             (desired_right - actual_right));
  max_waist_error_rad_ = std::max({
      std::abs(desired.q_des[kWaistYawSdk] -
               state.q[kWaistYawSdk]),
      std::abs(desired.q_des[kWaistRollSdk] -
               state.q[kWaistRollSdk]),
      std::abs(desired.q_des[kWaistPitchSdk] -
               state.q[kWaistPitchSdk]),
  });
  hip_pitch_offset_rad_ = 0.5 * (
      official_stand_q_sdk_[kLeftHipPitchSdk] -
          desired.q_des[kLeftHipPitchSdk] +
      official_stand_q_sdk_[kRightHipPitchSdk] -
          desired.q_des[kRightHipPitchSdk]);
  knee_pitch_offset_rad_ = 0.5 * (
      official_stand_q_sdk_[kLeftKneePitchSdk] -
          desired.q_des[kLeftKneePitchSdk] +
      official_stand_q_sdk_[kRightKneePitchSdk] -
          desired.q_des[kRightKneePitchSdk]);
  ankle_pitch_offset_rad_ = 0.5 * (
      official_stand_q_sdk_[kLeftAnklePitchSdk] -
          desired.q_des[kLeftAnklePitchSdk] +
      official_stand_q_sdk_[kRightAnklePitchSdk] -
          desired.q_des[kRightAnklePitchSdk]);
  waist_roll_offset_rad_ =
      official_stand_q_sdk_[kWaistRollSdk] - desired.q_des[kWaistRollSdk];
  waist_pitch_offset_rad_ =
      official_stand_q_sdk_[kWaistPitchSdk] -
      desired.q_des[kWaistPitchSdk];
}

void PpServeController::SetLastCommand_(
    const robot_io::RobotState& state, robot_io::RobotCommand& command) {
  if (last_command_.q_des.size() == kDof) {
    command = last_command_;
  } else {
    command = ZeroGainHold(state.q);
  }
}

void PpServeController::TripFault_(const std::string& reason) {
  prepare_grab_pending_ = false;
  fault_reason_ = reason;
  phase_ = "FAULT";
  state_.store(ServeControllerState::kFault, std::memory_order_release);
}

bool PpServeController::ComputeCommand(
    std::uint64_t /*tick_idx*/, const robot_io::RobotState& state,
    robot_io::RobotCommand& command) {
  std::lock_guard<std::mutex> lock(async_mutex_);
  if (!ValidateState_(state)) return false;
  const std::uint64_t now_ns = SteadyNowNs_();
  PollGripper_();
  ProcessRequests_(state, now_ns);

  switch (this->state()) {
    case ServeControllerState::kPreparingStand: {
      FillPdCommand_(official_stand_q_sdk_, Eigen::VectorXd::Zero(kDof), command);
      entry_transition_.Apply(static_cast<double>(transition_tick_) / kServe025FullbodyRunnerHz, command);
      UpdateSupportTelemetry_(state, command);
      last_command_ = command;
      phase_ = "READY_TO_STAND_SUPPORT";
      ++transition_tick_;
      // Finish this bounded command transition without an extra operator gate
      // or an indefinite measured-quiet wait. The loading lift damps body pitch.
      if (!entry_transition_.active()) {
        entry_command_ = command;
        transition_tick_ = 0;
        state_.store(ServeControllerState::kTransitionToLoad, std::memory_order_release);
      }
      break;
    }
    case ServeControllerState::kTransitionToLoad: {
      source_frame_ = 0;
      SetTransitionCommand_(transition_tick_, state, command);
      ++transition_tick_;
      if (transition_tick_ >= kServe025FullbodyTransitionTicks) {
        transition_tick_ = kServe025FullbodyTransitionTicks - 1;
        // Capture at the deterministic end of preparation, before Play is
        // available. The first operator's waiting time must not set support
        // for this entire session. READY feedback and the smooth pre-play
        // transition remain active so holding the ball stays supported.
        if (!replay_support_valid_) {
          replay_support_rad_ = prepare_balance_rad_;
          replay_support_valid_ = true;
        }
        state_.store(ServeControllerState::kWaitReadyToServe,
                     std::memory_order_release);
      }
      break;
    }

    case ServeControllerState::kWaitReadyToServe:
    case ServeControllerState::kWaitBallLoad:
    case ServeControllerState::kGripClosing:
    case ServeControllerState::kWaitGripSecure:
      source_frame_ = 0;
      SetTimelineCommand_(0, state, command);
      break;

    case ServeControllerState::kPlayingPreRelease:
    case ServeControllerState::kReleasePending:
    case ServeControllerState::kStrike:
    case ServeControllerState::kFollowThrough:
    case ServeControllerState::kRecovery: {
      if (replay_entry_tick_ < kServeReplaySettleTicks) {
        FillPdCommand_(timeline_.ready().q_sdk, timeline_.ready().qd_sdk, command);
        for (const int sdk : {kLeftAnklePitchSdk, kRightAnklePitchSdk})
          command.q_des[sdk] += replay_support_rad_;
        replay_entry_transition_.Apply(
            static_cast<double>(replay_entry_tick_++) / kServe025FullbodyRunnerHz, command);
        prepare_balance_rad_ = replay_support_rad_;
        prepare_balance_velocity_ = 0.;
        phase_ = "PRE_PLAY_SETTLE";
        last_command_ = command;
        UpdateSupportTelemetry_(state, command);
        break;
      }
      // Direct playback: timeline_tick_ is the original CSV frame index.
      // Stop at the configured post-contact handoff frame (legacy pure Serve
      // keeps frame 467). Played frames remain original and sequential; an
      // early cutoff returns to measured stand before the actor is engaged.
      const std::size_t frame = timeline_tick_;
      source_frame_ = frame;
      if (frame >= release_frame() &&
          !release_request_submitted_) {
        release_request_submitted_ = SubmitGripper_(
            PpGripperCommand::kRelease,
            ServeGripperState::kReleasing, now_ns);
        if (native_release_publisher_ && !release_request_submitted_) {
          // Do not swing after a failed release dispatch. Use the same smooth
          // supported return as the normal handoff, not a sudden Stand jump.
          robot_io::RobotCommand stand;
          FillPdCommand_(official_stand_q_sdk_, Eigen::VectorXd::Zero(kDof), stand);
          policy_return_transition_.Begin(last_command_, stand, policy_return_seconds_, .15);
          policy_return_tick_ = 0;
          policy_return_quiet_ticks_ = 0;
          command = last_command_;
          playback_.finished = true;
          phase_ = "RELEASE_FAILED_RETURN";
          state_.store(ServeControllerState::kHandoffReady, std::memory_order_release);
          break;
        }
      }
      if (native_release_publisher_ && native_release_copies_left_ &&
          frame >= native_release_next_frame_) {
        std::uint64_t repeated_ns = 0;
        if (!native_release_publisher_(repeated_ns))
          gripper_fault_reason_ = "native release redundant publication failed";
        --native_release_copies_left_;
        native_release_next_frame_ += 10;
      }
      SetTimelineCommand_(frame, state, command);
      const auto evaluated_ns = native_release_publisher_ ? SteadyNowNs_() : now_ns;
      playback_.ObserveClock(frame, evaluated_ns);
      const int sample_index = frame == 0 ? 0 :
          (frame == release_frame() ? 1 : (frame == kServe025ContactFrame ? 2 : -1));
      if (sample_index >= 0) {
        auto& sample = playback_.samples[sample_index];
        sample.frame = frame;
        sample.evaluation_ns = evaluated_ns;
        sample.state_timestamp_ns = state.timestamp_ns;
        std::copy_n(state.q.data(), 31, sample.q.begin());
        std::copy_n(state.dq.data(), 31, sample.dq.begin());
        std::copy_n(command.q_des.data(), 31, sample.q_des.begin());
        std::copy_n(command.dq_des.data(), 31, sample.dq_des.begin());
        std::copy_n(state.imu_quat_wxyz.data(), 4, sample.imu_quat.begin());
        std::copy_n(state.imu_gyro.data(), 3, sample.gyro.begin());
      }
      if (frame == release_frame()) playback_.release_frame_ns = evaluated_ns;
      if (frame == kServe025ContactFrame) playback_.contact_reference_frame_ns = evaluated_ns;
      const auto& csv = timeline_.At(frame);
      playback_.max_csv_q_delta_rad = std::max(playback_.max_csv_q_delta_rad,
          (command.q_des - csv.q_sdk).cwiseAbs().maxCoeff());
      playback_.max_csv_dq_delta_rad_s = std::max(playback_.max_csv_dq_delta_rad_s,
          (command.dq_des - csv.qd_sdk).cwiseAbs().maxCoeff());
      playback_.max_arm_csv_q_delta_rad = std::max(playback_.max_arm_csv_q_delta_rad,
          (command.q_des.segment(5, 14) - csv.q_sdk.segment(5, 14)).cwiseAbs().maxCoeff());
      playback_.max_arm_csv_dq_delta_rad_s = std::max(playback_.max_arm_csv_dq_delta_rad_s,
          (command.dq_des.segment(5, 14) - csv.qd_sdk.segment(5, 14)).cwiseAbs().maxCoeff());
      playback_.max_tracking_error_rad = std::max(playback_.max_tracking_error_rad,
          (command.q_des - state.q).cwiseAbs().maxCoeff());
      playback_.max_arm_tracking_error_rad = std::max(playback_.max_arm_tracking_error_rad,
          (command.q_des.segment(5, 14) - state.q.segment(5, 14)).cwiseAbs().maxCoeff());
      if (frame == policy_handoff_frame_) {
        playback_.finished = true;
        if (frame < kServe025CompleteFrame) {
          // The post-contact pose is far outside the receive actor's WAIT
          // posture (including its executed-action feedback). A live actor
          // crossfade is continuous but drives the legs while the arms are
          // still raised. Return under official stand gains first, then use
          // exactly the normal Stand -> policy entry/reset path.
          robot_io::RobotCommand stand;
          FillPdCommand_(official_stand_q_sdk_, Eigen::VectorXd::Zero(kDof), stand);
          policy_return_transition_.Begin(command, stand, policy_return_seconds_, .15);
          policy_return_tick_ = 0;
          policy_return_quiet_ticks_ = 0;
          phase_ = "POST_CONTACT_TO_STAND";
          state_.store(ServeControllerState::kHandoffReady, std::memory_order_release);
        } else {
          phase_ = "COMPLETE";
          state_.store(ServeControllerState::kComplete, std::memory_order_release);
        }
      } else if (frame < release_frame()) {
        state_.store(ServeControllerState::kPlayingPreRelease,
                     std::memory_order_release);
      } else if (frame < kServe025StrikeFrame) {
        state_.store(ServeControllerState::kReleasePending,
                     std::memory_order_release);
      } else if (frame <= kServe025ContactFrame) {
        state_.store(ServeControllerState::kStrike,
                     std::memory_order_release);
      } else if (frame < kServe025RecoveryFrame) {
        state_.store(ServeControllerState::kFollowThrough,
                     std::memory_order_release);
      } else {
        state_.store(ServeControllerState::kRecovery,
                     std::memory_order_release);
      }
      ++timeline_tick_;
      break;
    }

    case ServeControllerState::kHandoffReady: {
      FillPdCommand_(official_stand_q_sdk_, Eigen::VectorXd::Zero(kDof), command);
      const double elapsed = static_cast<double>(policy_return_tick_++) / kServe025FullbodyRunnerHz;
      policy_return_transition_.Apply(elapsed, command);
      if (policy_return_seconds_ > 1.) {
        // A slow arm return needs balance throughout the changing upper-body
        // load. Preserve the source command at t=0 and exact Stand at t=T.
        const double remaining = policy_return_seconds_ - elapsed;
        const double envelope = Smooth5_(elapsed / .25) * Smooth5_(remaining / .75);
        const double derivative = Smooth5Derivative_(elapsed / .25) / .25 * Smooth5_(remaining / .75) -
            Smooth5_(elapsed / .25) * Smooth5Derivative_(remaining / .75) / .75;
        ApplyPitchSupport_(state, command, envelope, derivative);
      }
      last_command_ = command;
      UpdateSupportTelemetry_(state, command);
      const auto gravity = projected_gravity_body(state.imu_quat_wxyz.normalized());
      // Completion follows the measured robot, not just the trajectory clock.
      // No manual acknowledgement: proceed as soon as the return has settled.
      const bool quiet = !policy_return_transition_.active() &&
          (state.q - official_stand_q_sdk_).cwiseAbs().maxCoeff() <= .10 &&
          state.dq.cwiseAbs().maxCoeff() <= .3 &&
          gravity[2] <= -std::cos(.06) && state.imu_gyro.norm() <= .10;
      policy_return_quiet_ticks_ = quiet ? policy_return_quiet_ticks_ + 1 : 0;
      phase_ = policy_return_transition_.active() ? "POST_CONTACT_TO_STAND" : "WAIT_STAND_SETTLE";
      if (policy_return_quiet_ticks_ >= 20) {
        phase_ = "COMPLETE";
        std::fprintf(stderr,
            "[serve handoff] measured stand settled: return_s=%.3f q_error=%.4f "
            "joint_speed=%.4f tilt=%.4f gyro=%.4f -> serve complete\n",
            static_cast<double>(policy_return_tick_ - 1) / kServe025FullbodyRunnerHz,
            (state.q - official_stand_q_sdk_).cwiseAbs().maxCoeff(),
            state.dq.cwiseAbs().maxCoeff(),
            std::acos(std::clamp(-gravity[2], -1.0, 1.0)), state.imu_gyro.norm());
        state_.store(ServeControllerState::kComplete, std::memory_order_release);
      }
      break;
    }
    case ServeControllerState::kIdle:
    case ServeControllerState::kComplete:
    case ServeControllerState::kAbortWaitHumanClear:
    case ServeControllerState::kAbortReturn:
    case ServeControllerState::kAborted:
    case ServeControllerState::kFault:
    case ServeControllerState::kCleanupOpening:
      SetLastCommand_(state, command);
      break;
  }

  UpdateDiag_(state, command);
  return true;
}

bool PpServeController::active() const noexcept {
  switch (state()) {
    case ServeControllerState::kPreparingStand:
    case ServeControllerState::kTransitionToLoad:
    case ServeControllerState::kWaitReadyToServe:
    case ServeControllerState::kPlayingPreRelease:
    case ServeControllerState::kReleasePending:
    case ServeControllerState::kStrike:
    case ServeControllerState::kFollowThrough:
    case ServeControllerState::kRecovery:
    case ServeControllerState::kHandoffReady:
      return true;
    default:
      return false;
  }
}

std::string PpServeController::fault_reason() const {
  std::lock_guard<std::mutex> lock(async_mutex_);
  return fault_reason_;
}

std::string PpServeController::PhaseName_() const { return phase_; }

void PpServeController::UpdateDiag_(
    const robot_io::RobotState& state,
    const robot_io::RobotCommand& command) {
  ServeControllerDiag value;
  value.valid = true;
  value.state = this->state();
  value.gripper_state = gripper_state();
  value.phase = PhaseName_();
  value.frame = source_frame_;
  value.playback = playback_;
  value.transition_tick = transition_tick_;
  value.cleanup_required = cleanup_required();
  value.release_acknowledged = release_acknowledged_;
  value.release_dispatch_monotonic_ns = release_dispatch_monotonic_ns_;
  value.release_first_publish_monotonic_ns =
      release_first_publish_monotonic_ns_;
  if (command.q_des.size() == kDof) {
    value.max_q_error_rad =
        (command.q_des - state.q).cwiseAbs().maxCoeff();
    value.tracking_warning = value.max_q_error_rad > kTrackingWarningRad;
  }
  value.max_joint_speed_rad_s = state.dq.cwiseAbs().maxCoeff();
  const Eigen::Vector4d quaternion = state.imu_quat_wxyz.normalized();
  const double gravity_x = projected_gravity_body(quaternion)[0];
  const double gravity_z =
      2.0 * (quaternion[1] * quaternion[1] +
             quaternion[2] * quaternion[2]) -
      1.0;
  value.projected_gravity_x = gravity_x;
  value.ankle_pitch_offset_rad = ankle_pitch_offset_rad_;
  value.hip_pitch_offset_rad = hip_pitch_offset_rad_;
  value.knee_pitch_offset_rad = knee_pitch_offset_rad_;
  value.waist_roll_offset_rad = waist_roll_offset_rad_;
  value.waist_pitch_offset_rad = waist_pitch_offset_rad_;
  value.sagittal_support_error_rad = sagittal_support_error_rad_;
  value.max_waist_error_rad = max_waist_error_rad_;
  value.tilt_rad = std::acos(std::clamp(-gravity_z, -1.0, 1.0));
  value.yaw_rate_rad_s = std::abs(state.imu_gyro[2]);
  value.local_ready = value.max_q_error_rad <= 0.08 &&
                      value.max_joint_speed_rad_s <= 0.15 &&
                      value.tilt_rad <= 0.10 &&
                      value.yaw_rate_rad_s <= 0.20;
  value.ready_ticks = value.local_ready ? 1 : 0;
  if (value.phase == "PRE_PLAY_SETTLE") {
    value.local_ready = false;
    value.ready_ticks = 0;
  }
  if (value.state == ServeControllerState::kHandoffReady ||
      value.state == ServeControllerState::kPreparingStand) {
    value.ready_ticks = policy_return_quiet_ticks_;
    value.local_ready = policy_return_quiet_ticks_ > 0;
  }
  value.gripper_fault_reason = gripper_fault_reason_;
  value.fault_reason = fault_reason_;
  std::lock_guard<std::mutex> lock(diag_mutex_);
  diag_ = std::move(value);
}

ServeControllerDiag PpServeController::TakeDiag() {
  std::lock_guard<std::mutex> lock(diag_mutex_);
  return diag_;
}

}  // namespace a3_pingpong
