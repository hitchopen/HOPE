#pragma once

#include "a3_pingpong/pp_command_transition.hpp"

#include "a3_pingpong/pp_gripper_worker.hpp"
#include "a3_pingpong/pp_serve025_fullbody_timeline.hpp"
#include "robot_io/robot_io_backend.hpp"

#include <atomic>
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Dense>

namespace a3_pingpong {

// A selected CSV may request a symmetric wider Stand, but cannot redefine
// the model's other joints. Check every SDK joint before changing the stance.
inline double ServeStandLateralOffset(const Eigen::VectorXd& nominal,
                                     const Eigen::VectorXd& complete) {
  if (nominal.size() != 31 || complete.size() != 31 ||
      !nominal.allFinite() || !complete.allFinite())
    throw std::invalid_argument("serve Stand requires finite SDK31 poses");
  const double raw = complete[20] - nominal[20];
  if (raw < -1e-6 || raw > .06 + 1e-6)
    throw std::invalid_argument("serve Stand lateral offset outside [0, 0.06] rad");
  const double offset = std::clamp(raw, 0.0, .06);
  Eigen::VectorXd expected = nominal;
  expected[20] += offset;
  expected[24] -= offset;
  expected[26] -= offset;
  expected[30] += offset;
  if ((complete - expected).cwiseAbs().maxCoeff() > 1e-6)
    throw std::invalid_argument("CSV COMPLETE is not the model Stand or its symmetric lateral stance");
  return offset;
}

// Kernel serving returns to Stand and has no receive-policy handoff.
// Pure Serve likewise owns the complete CSV, including follow-through/recovery.
inline constexpr std::size_t DefaultServeHandoffFrame(bool kernel_mode,
                                                     bool serve_only) noexcept {
  return (kernel_mode || serve_only) ? kServe025CompleteFrame : 110;
}

// The single Runner/publisher is 100 Hz while serving so every original
// selected SDK31 CSV FRAME maps one-to-one to one command tick. Outside SERVE,
// the learned receive policy contract remains 50 Hz and its command is held for
// the intervening 10 ms callback. Arms follow CSV q_des/dq_des exactly, without
// resampling or velocity scaling. Existing bounded ankle pitch support does
// modify SDK joints 23/29; playback diagnostics report that delta explicitly.
inline constexpr double kServePolicyHz = kServe025FullbodyRunnerHz;
inline constexpr std::size_t kServe025ActionTicks100Hz =
    kServe025FullbodyCommandTicks;
// Automatic pre-play normalization: 0.5 s command blend + 0.5 s quiet hold.
// The CSV clock and release frame do not advance during these command ticks.
inline constexpr std::size_t kServeReplaySettleTicks = 101;

enum class ServeControllerState : int {
  kIdle = 0,
  kPreparingStand = 1,
  kTransitionToLoad = 2,
  kWaitBallLoad = 3,
  kGripClosing = 4,
  kWaitReadyToServe = 5,
  kPlayingPreRelease = 6,
  kReleasePending = 7,
  kStrike = 8,
  kFollowThrough = 9,
  kRecovery = 10,
  kHandoffReady = 11,
  kComplete = 12,
  kAbortWaitHumanClear = 13,
  kAbortReturn = 14,
  kAborted = 15,
  kFault = 16,
  kCleanupOpening = 17,
  kWaitGripSecure = 18,
};

const char* ServeControllerStateName(ServeControllerState state) noexcept;

enum class ServeGripperState : int {
  kUnavailable = -1,
  kUnknown = 0,
  kOpening = 1,
  kOpen = 2,
  kClosing = 3,
  kGrabbed = 4,
  kReleasing = 5,
  kReleased = 6,
  kFault = 7,
};

const char* ServeGripperStateName(ServeGripperState state) noexcept;

struct ServeReplaySample {
  std::size_t frame{0};
  std::uint64_t evaluation_ns{0};
  std::int64_t state_timestamp_ns{0};
  std::array<double, 31> q{}, dq{}, q_des{}, dq_des{};
  std::array<double, 4> imu_quat{};
  std::array<double, 3> gyro{};
};

// Fixed-size per-action evidence, accumulated on every CSV evaluation and
// printed by the status thread after playback. No I/O on the control thread.
// These are software evaluation times, not actuator delivery or ball contact.
struct ServePlaybackAudit {
  std::uint64_t sequence{0}, first_ns{0}, last_ns{0};
  std::uint64_t min_interval_ns{0}, max_interval_ns{0}, max_phase_error_ns{0};
  std::uint64_t release_frame_ns{0}, contact_reference_frame_ns{0};
  std::size_t frames{0};
  bool finished{false};
  double start_support_rad{0}, start_support_velocity{0};
  double max_csv_q_delta_rad{0}, max_csv_dq_delta_rad_s{0};
  double max_arm_csv_q_delta_rad{0}, max_arm_csv_dq_delta_rad_s{0};
  double max_tracking_error_rad{0}, max_arm_tracking_error_rad{0};
  // Frame zero, release-command frame, CSV contact-reference frame.
  // Recorded on the command thread; emitted only after playback by status.
  std::array<ServeReplaySample, 3> samples{};

  void ObserveClock(std::size_t frame, std::uint64_t now_ns) {
    if (frames == 0) first_ns = now_ns;
    else {
      const auto interval = now_ns - last_ns;
      min_interval_ns = frames == 1 ? interval : std::min(min_interval_ns, interval);
      max_interval_ns = std::max(max_interval_ns, interval);
    }
    const auto expected = first_ns + frame * 10'000'000ULL;
    const std::uint64_t error = now_ns >= expected ? now_ns - expected : expected - now_ns;
    max_phase_error_ns = std::max(max_phase_error_ns, error);
    last_ns = now_ns;
    ++frames;
  }
};

struct ServeControllerDiag {
  bool valid{false};
  ServeControllerState state{ServeControllerState::kIdle};
  ServeGripperState gripper_state{ServeGripperState::kUnavailable};
  std::string phase{"IDLE"};
  std::size_t frame{0};
  std::size_t transition_tick{0};
  int ready_ticks{0};
  bool local_ready{false};
  bool cleanup_required{false};
  bool abort_after_strike{false};
  bool release_acknowledged{false};
  std::uint64_t release_dispatch_monotonic_ns{0};
  std::uint64_t release_first_publish_monotonic_ns{0};
  double max_q_error_rad{0.0};
  bool tracking_warning{false};
  double max_joint_speed_rad_s{0.0};
  double tilt_rad{0.0};
  double yaw_rate_rad_s{0.0};
  double projected_gravity_x{0.0};
  double ankle_pitch_offset_rad{0.0};
  double hip_pitch_offset_rad{0.0};
  double knee_pitch_offset_rad{0.0};
  double waist_roll_offset_rad{0.0};
  double waist_pitch_offset_rad{0.0};
  double sagittal_support_error_rad{0.0};
  double max_waist_error_rad{0.0};
  std::string gripper_fault_reason;
  std::string fault_reason;
  ServePlaybackAudit playback;
};

// Same-runner direct player for the named SDK31 serve025 timeline. PREPARE_SERVE
// makes a smooth, controller-owned official-stand -> CSV-frame-0 transition.
// READY_TO_SERVE consumes full31 q from the CSV with bounded ankle support and returns the same
// publisher to MOTION after the CSV COMPLETE frame, or after a configured
// early post-contact return to measured, settled official stand. CSV dq is direct with no
// second runtime multiplier, preserving the selected CSV arm command exactly.
// No q_des slot is filled or overwritten from stand pose. The CSV
// has no gain columns, so the existing unscaled official
// PD_STAND Kp/Kd vectors remain the controller-owned gain source; tau_ff is
// zero.
class PpServeController final {
 public:
  PpServeController(PpServe025FullbodyTimeline timeline,
                    const Eigen::VectorXd& official_stand_q_sdk,
                    const Eigen::VectorXd& official_stand_kp_sdk,
                    const Eigen::VectorXd& official_stand_kd_sdk,
                    std::unique_ptr<PpGripperWorker> gripper);
  ~PpServeController();

  PpServeController(const PpServeController&) = delete;
  PpServeController& operator=(const PpServeController&) = delete;

  void SetPolicyHandoffFrame(std::size_t frame);
  void SetPolicyReturnSeconds(double seconds);
  // Optional hand delay compensation only; all body CSV frames stay exact.
  void SetReleaseLeadFrames(std::size_t frames);
  using NativeReleasePublisher = std::function<bool(std::uint64_t&)>;
  void SetNativeReleasePublisher(NativeReleasePublisher publisher);
  std::uint64_t ConsumeReleaseClockRebase() noexcept {
    return release_clock_rebase_ns_.exchange(0, std::memory_order_acq_rel);
  }
  std::size_t release_frame() const noexcept { return kServe025ReleaseFrame - release_lead_frames_; }
  std::size_t handoff_frame() const noexcept { return policy_handoff_frame_; }
  void Start();
  void ConfirmBallLoaded();
  void ConfirmGripSecure();
  void TriggerReadyToServe();
  void RequestAbort();
  void EmergencyStop();
  bool RequestOpenGripper(std::string& error);
  void PollAsync();

  // Called by the sole driver before the first SERVE tick. Keeps an early
  // Stand -> Serve request continuous even while Stand is still settling.
  void SetEntryCommand(const robot_io::RobotCommand& command);

  bool ComputeCommand(std::uint64_t tick_idx,
                      const robot_io::RobotState& state,
                      robot_io::RobotCommand& command);

  ServeControllerState state() const noexcept {
    return state_.load(std::memory_order_acquire);
  }
  ServeGripperState gripper_state() const noexcept {
    return gripper_state_.load(std::memory_order_acquire);
  }
  bool cleanup_required() const noexcept {
    return cleanup_required_.load(std::memory_order_acquire);
  }
  bool available() const noexcept { return !timeline_.empty(); }
  bool active() const noexcept;
  std::string fault_reason() const;
  std::size_t frame_count() const noexcept {
    return kServe025FullbodyFrames;
  }
  ServeControllerDiag TakeDiag();

 private:
  static std::uint64_t SteadyNowNs_() noexcept;
  bool ValidateState_(const robot_io::RobotState& state);
  void ResetAction_();
  void ProcessRequests_(const robot_io::RobotState& state,
                        std::uint64_t now_ns);
  void PollGripper_();
  void AdvancePrepareGripper_(std::uint64_t now_ns);
  bool SubmitGripper_(PpGripperCommand command,
                      ServeGripperState pending_state,
                      std::uint64_t now_ns);
  static double Smooth5_(double value) noexcept;
  static double Smooth5Derivative_(double value) noexcept;
  void SetTransitionCommand_(std::size_t tick,
                             const robot_io::RobotState& state,
                             robot_io::RobotCommand& command);
  void ApplyPitchSupport_(const robot_io::RobotState& state,
                          robot_io::RobotCommand& command,
                          double envelope, double envelope_velocity);
  void SetTimelineCommand_(std::size_t frame,
                           const robot_io::RobotState& state,
                           robot_io::RobotCommand& command);
  void FillPdCommand_(const Eigen::VectorXd& q_des,
                      const Eigen::VectorXd& dq_des,
                      robot_io::RobotCommand& command) const;
  void UpdateSupportTelemetry_(const robot_io::RobotState& state,
                               const robot_io::RobotCommand& desired);
  void SetLastCommand_(const robot_io::RobotState& state,
                       robot_io::RobotCommand& command);
  void TripFault_(const std::string& reason);
  void UpdateDiag_(const robot_io::RobotState& state,
                   const robot_io::RobotCommand& command);
  std::string PhaseName_() const;

  std::size_t policy_handoff_frame_ = kServe025CompleteFrame;
  double policy_return_seconds_ = 1.0;
  std::size_t release_lead_frames_ = 0;
  PpServe025FullbodyTimeline timeline_;
  Eigen::VectorXd official_stand_q_sdk_;
  Eigen::VectorXd official_stand_kp_sdk_;
  Eigen::VectorXd official_stand_kd_sdk_;
  std::unique_ptr<PpGripperWorker> gripper_;
  robot_io::RobotCommand last_command_;
  robot_io::RobotCommand entry_command_;
  PpCommandTransition entry_transition_;
  double prepare_balance_rad_{0.0};
  double prepare_balance_velocity_{0.0};
  double swing_balance_rad_{0.0};
  double swing_balance_velocity_{0.0};
  // Pin the bounded standing-support reference for this controller lifetime.
  // ResetAction must not let a new button/IMU phase redefine the next replay.
  bool replay_support_valid_{false};
  double replay_support_rad_{0.0};
  PpCommandTransition replay_entry_transition_;
  std::size_t replay_entry_tick_{kServeReplaySettleTicks};
  ServePlaybackAudit playback_;
  PpCommandTransition policy_return_transition_;
  std::size_t policy_return_tick_{0};
  int policy_return_quiet_ticks_{0};
  bool entry_command_valid_{false};

  double ankle_pitch_offset_rad_{0.0};
  double hip_pitch_offset_rad_{0.0};
  double knee_pitch_offset_rad_{0.0};
  double waist_roll_offset_rad_{0.0};
  double waist_pitch_offset_rad_{0.0};
  double sagittal_support_error_rad_{0.0};
  double max_waist_error_rad_{0.0};

  std::atomic<ServeControllerState> state_{ServeControllerState::kIdle};
  std::atomic<ServeGripperState> gripper_state_{
      ServeGripperState::kUnknown};
  std::atomic<bool> cleanup_required_{false};
  std::atomic<bool> start_requested_{false};
  std::atomic<bool> ready_to_serve_requested_{false};
  std::atomic<bool> abort_requested_{false};
  std::atomic<bool> emergency_stop_requested_{false};

  std::size_t transition_tick_{0};
  std::size_t timeline_tick_{0};
  std::size_t source_frame_{0};
  bool prepare_grab_pending_{false};
  bool prepare_open_submitted_{false};
  bool release_request_submitted_{false};
  NativeReleasePublisher native_release_publisher_;
  unsigned native_release_copies_left_{0};
  std::size_t native_release_next_frame_{0};
  std::atomic<std::uint64_t> release_clock_rebase_ns_{0};
  bool release_acknowledged_{false};
  std::uint64_t gripper_generation_{0};
  std::uint64_t acknowledged_gripper_generation_{0};
  std::uint64_t release_dispatch_monotonic_ns_{0};
  std::uint64_t release_first_publish_monotonic_ns_{0};
  std::string phase_{"IDLE"};
  std::string fault_reason_;
  std::string gripper_fault_reason_;

  mutable std::mutex async_mutex_;
  mutable std::mutex diag_mutex_;
  ServeControllerDiag diag_{};
};

}  // namespace a3_pingpong
