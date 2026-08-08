#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

#include <Eigen/Dense>

#include "a3_deploy/numeric_safety.hpp"
#include "a3_pingpong/pp_joint_limits.hpp"
#include "a3_pingpong/pp_planner_input.hpp"
#include "a3_pingpong/pp_serve_clip.hpp"
#include "a3_policy_parameters.hpp"
#include "robot_io/a3_layout_extra.hpp"
#include "robot_io/robot_io_backend.hpp"

namespace a3_pingpong {

constexpr double kServeHeadKp = 40.0;
constexpr double kServeHeadKd = 2.0;
constexpr double kServeBallRelXMinM = 0.20;
constexpr double kServeBallRelXMaxM = 0.80;
constexpr double kServeBallRelYMinM = -0.20;
constexpr double kServeBallRelYMaxM = 0.55;
constexpr double kServeBallRelZMinM = -0.25;
constexpr double kServeBallRelZMaxM = 0.50;
constexpr double kServeBallVxMinMps = -0.10;
constexpr double kServeBallVxMaxMps = 0.35;
constexpr double kServeBallAbsVyMaxMps = 0.25;
constexpr double kServeBallVzMinMps = 0.80;
constexpr double kServeBallVzMaxMps = 2.30;
constexpr int kServeBallMinEstimatorSamples = 6;
// These gains are part of the dynamically qualified clip contract.  They are
// intentionally not exposed as operator tuning knobs.

enum class ServeControllerState : int {
  kIdle = 0,
  kPreflightReady = 1,
  kPlaying = 2,
  kAwaitBall = 3,
  kAbortReturn = 4,
  kHandoffReady = 5,
  kComplete = 6,
  kAborted = 7,
  kFault = 8,
};

inline const char* ServeControllerStateName(ServeControllerState state) {
  switch (state) {
    case ServeControllerState::kIdle: return "IDLE";
    case ServeControllerState::kPreflightReady: return "PREFLIGHT_READY";
    case ServeControllerState::kPlaying: return "PLAYING";
    case ServeControllerState::kAwaitBall: return "AWAIT_BALL_ON_PALM";
    case ServeControllerState::kAbortReturn: return "ABORT_RETURN";
    case ServeControllerState::kHandoffReady: return "HANDOFF_READY";
    case ServeControllerState::kComplete: return "COMPLETE";
    case ServeControllerState::kAborted: return "ABORTED";
    case ServeControllerState::kFault: return "FAULT";
  }
  return "UNKNOWN";
}

struct ServeControllerConfig {
  int approach_ticks = 60;         // 1.2 s measured-pose -> default quintic
  int preflight_dwell_ticks = 25;  // 0.5 s at 50 Hz
  int handoff_dwell_ticks = 25;    // 0.5 s at 50 Hz
  int handoff_gain_transition_ticks =
      kServeHandoffGainTransitionTicks;  // serve PD -> V17 static PD
  int abort_return_ticks = 60;     // 1.2 s quintic return
  double ready_q_error_rad = 0.08;
  double ready_joint_speed_rad_s = 0.35;
  double ready_tilt_rad = 0.10;
  double ready_heading_error_rad = kServeGate3HeadingToleranceRad;
  double ready_yaw_rate_rad_s = 0.20;
  bool require_external_base = false;
  double base_max_age_s = 0.20;
  double ball_max_age_s = 0.050;
  double ready_base_position_error_m =
      kServeGate3StationXyToleranceM;
  double ready_base_xy_speed_mps = 0.20;
  double max_tracking_error_rad = 0.35;
};

struct ServeControllerDiag {
  bool valid = false;
  ServeControllerState state = ServeControllerState::kIdle;
  ServePhase phase = ServePhase::kApproachReady;
  std::size_t frame = 0;
  int ready_ticks = 0;
  bool local_ready = false;
  bool abort_after_commit = false;
  bool handoff_request = false;
  double max_q_error = 0.0;
  double max_joint_speed = 0.0;
  double tilt_rad = 0.0;
  double heading_error_rad = 0.0;
  double yaw_rate_rad_s = 0.0;
  bool base_valid = false;
  double base_position_error_m = 0.0;
  double base_xy_speed_mps = 0.0;
  bool branch_selected = false;
  bool toss_only_abort = false;
  std::string selected_branch;
  double ball_age_s = 0.0;
  int ball_estimator_samples = 0;
  double ball_vx_mps = 0.0;
  Vec3 ball_position_rel_base = Vec3::Zero();
  std::string branch_reason;
  std::string fault_reason;
};

// Deterministic, palm-only serving controller.  It is the sole owner of all 31
// q_des slots while active; V17 is not run or partially overlaid.
class PpServeController {
 public:
  PpServeController(PpServeClip fixed_clip,
                    const Eigen::VectorXd& default_q_sdk,
                    ServeControllerConfig config = {},
                    std::shared_ptr<PpBasePoseInput> base_input = nullptr,
                    std::shared_ptr<PpBallStateInput> ball_input = nullptr)
      : clip_(std::move(fixed_clip)),
        default_q_sdk_(default_q_sdk),
        config_(config),
        base_input_(std::move(base_input)),
        ball_input_(std::move(ball_input)) {
    if (default_q_sdk_.size() != kServeDof || !AllFinite_(default_q_sdk_)) {
      throw std::runtime_error("serve controller default_q must be finite 31-D");
    }
    if (config_.approach_ticks < 2 ||
        config_.preflight_dwell_ticks < 1 ||
        config_.handoff_dwell_ticks < 1 ||
        config_.handoff_gain_transition_ticks < 2 ||
        config_.abort_return_ticks < 2 ||
        config_.ready_q_error_rad <= 0.0 ||
        config_.ready_joint_speed_rad_s <= 0.0 ||
        config_.ready_tilt_rad <= 0.0 ||
        config_.ready_heading_error_rad <= 0.0 ||
        config_.ready_yaw_rate_rad_s <= 0.0 ||
        config_.base_max_age_s <= 0.0 ||
        config_.ball_max_age_s <= 0.0 ||
        config_.ready_base_position_error_m <= 0.0 ||
        config_.ready_base_position_error_m >
            kServeGate3StationXyToleranceM ||
        config_.ready_heading_error_rad >
            kServeGate3HeadingToleranceRad ||
        config_.ready_base_xy_speed_mps <= 0.0 ||
        base_input_ == nullptr || ball_input_ == nullptr ||
        config_.max_tracking_error_rad <= config_.ready_q_error_rad) {
      throw std::runtime_error("invalid serve controller configuration");
    }
    BuildGains_();
    last_q_des_ = default_q_sdk_;
    abort_start_q_ = default_q_sdk_;
  }

 public:
  // Thread-safe operator requests.  State transitions occur on the 50 Hz
  // driver thread inside ComputeCommand.
  void Start() {
    abort_requested_.store(false, std::memory_order_release);
    start_requested_.store(true, std::memory_order_release);
  }
  void RequestAbort() {
    abort_requested_.store(true, std::memory_order_release);
  }
  void ConfirmBallOnPalm() {
    ball_confirm_requested_.store(true, std::memory_order_release);
  }

  bool ComputeCommand(std::uint64_t /*tick_idx*/,
                      const robot_io::RobotState& state,
                      robot_io::RobotCommand& command) {
    if (!ValidateState_(state)) return false;

    if (start_requested_.exchange(false, std::memory_order_acq_rel)) {
      const ServeControllerState current =
          state_.load(std::memory_order_acquire);
      if (current == ServeControllerState::kIdle ||
          current == ServeControllerState::kComplete ||
          current == ServeControllerState::kAborted) {
        state_ = ServeControllerState::kPreflightReady;
        frame_ = 0;
        ready_ticks_ = 0;
        handoff_gain_tick_ = 0;
        abort_after_commit_ = false;
        successful_play_ = false;
        handoff_request_.store(false, std::memory_order_release);
        ball_confirm_requested_.store(false, std::memory_order_release);
        fault_reason_.clear();
        last_q_des_ = state.q;
        approach_start_q_ = state.q;
        approach_tick_ = 0;
        initial_heading_rad_ = YawFromQuat_(state.imu_quat_wxyz);
        base_anchor_set_ = false;
        base_velocity_valid_ = false;
        base_prev_seq_ = 0;
        base_prev_stamp_wall_s_ = -1.0;
        branch_selected_ = false;
        toss_only_abort_ = false;
        selected_branch_.clear();
        branch_reason_.clear();
        branch_ball_age_s_ = 0.0;
        branch_ball_estimator_samples_ = 0;
        branch_ball_vx_mps_ = 0.0;
        branch_ball_position_rel_base_.setZero();
        ball_seq_at_confirm_ = 0;
      }
    }
    if (ball_confirm_requested_.exchange(false,
                                         std::memory_order_acq_rel) &&
        state_.load(std::memory_order_acquire) ==
            ServeControllerState::kAwaitBall) {
      PpBallSample ball;
      if (ball_input_->Latest(ball, config_.ball_max_age_s)) {
        ball_seq_at_confirm_ = ball.seq;
      }
      state_ = ServeControllerState::kPlaying;
    }

    const ReadyValues ready = Ready_(state);
    const bool abort = abort_requested_.exchange(false,
                                                  std::memory_order_acq_rel);
    if (abort) HandleAbortRequest_();

    switch (state_) {
      case ServeControllerState::kIdle:
      case ServeControllerState::kComplete:
      case ServeControllerState::kAborted:
        FillCommand_(default_q_sdk_, command);
        break;
      case ServeControllerState::kPreflightReady:
        if (approach_tick_ < config_.approach_ticks) {
          const double u = static_cast<double>(approach_tick_) /
                           static_cast<double>(config_.approach_ticks - 1);
          const double blend = Smooth5_(u);
          FillCommand_(
              (1.0 - blend) * approach_start_q_ +
                  blend * default_q_sdk_,
              command);
          ++approach_tick_;
          ready_ticks_ = 0;
        } else {
          FillCommand_(default_q_sdk_, command);
          ready_ticks_ = ready.pass ? ready_ticks_ + 1 : 0;
          if (ready_ticks_ >= config_.preflight_dwell_ticks) {
            state_ = ServeControllerState::kPlaying;
            frame_ = 0;
            ready_ticks_ = 0;
          }
        }
        break;
      case ServeControllerState::kPlaying:
        Play_(state, command);
        break;
      case ServeControllerState::kAwaitBall:
        FillCommand_(
            clip_.frame(
                static_cast<std::size_t>(clip_.events().toss_commit - 1))
                .q_sdk,
            command);
        break;
      case ServeControllerState::kAbortReturn:
        AbortReturn_(command);
        break;
      case ServeControllerState::kHandoffReady:
        FillHandoffTransitionCommand_(default_q_sdk_, command);
        ready_ticks_ = ready.pass ? ready_ticks_ + 1 : 0;
        if (handoff_gain_tick_ >=
                config_.handoff_gain_transition_ticks &&
            ready_ticks_ >= config_.handoff_dwell_ticks) {
          if (successful_play_) {
            state_ = ServeControllerState::kComplete;
            handoff_request_.store(true, std::memory_order_release);
          } else {
            state_ = ServeControllerState::kAborted;
          }
        }
        break;
      case ServeControllerState::kFault:
        FillCommand_(default_q_sdk_, command);
        break;
    }

    const double tracking_error =
        (command.q_des - state.q).cwiseAbs().maxCoeff();
    if (tracking_error > config_.max_tracking_error_rad &&
        state_ != ServeControllerState::kPreflightReady &&
        state_ != ServeControllerState::kFault) {
      TripFault_("tracking_error_exceeded");
      return false;
    }
    last_q_des_ = command.q_des;
    UpdateDiag_(ready, tracking_error);
    return true;
  }

  bool ConsumeHandoffRequest() {
    return handoff_request_.exchange(false, std::memory_order_acq_rel);
  }
  ServeControllerState state() const {
    return state_.load(std::memory_order_acquire);
  }
  bool active() const {
    const ServeControllerState current = state();
    return current != ServeControllerState::kIdle &&
           current != ServeControllerState::kComplete &&
           current != ServeControllerState::kAborted &&
           current != ServeControllerState::kFault;
  }
  const PpServeClip& clip() const { return clip_; }

  ServeControllerDiag TakeDiag() {
    std::lock_guard<std::mutex> lock(diag_mutex_);
    ServeControllerDiag out = diag_;
    diag_.valid = false;
    return out;
  }

 private:
  struct ReadyValues {
    bool pass = false;
    double q_error = 0.0;
    double joint_speed = 0.0;
    double tilt = 0.0;
    double heading_error = 0.0;
    double yaw_rate = 0.0;
    bool base_valid = false;
    double base_position_error = 0.0;
    double base_xy_speed = 0.0;
  };

  static bool AllFinite_(const Eigen::VectorXd& values) {
    for (int i = 0; i < values.size(); ++i) {
      if (!a3_deploy::numeric_safety::IsFinite(values[i])) return false;
    }
    return true;
  }

  static double Smooth5_(double value) {
    const double x = std::clamp(value, 0.0, 1.0);
    return x * x * x * (10.0 + x * (-15.0 + 6.0 * x));
  }

  static double WrapAngle_(double value) {
    return std::atan2(std::sin(value), std::cos(value));
  }

  static double YawFromQuat_(const Eigen::Vector4d& quat_wxyz) {
    const Eigen::Vector4d q = quat_wxyz.normalized();
    return std::atan2(
        2.0 * (q[0] * q[3] + q[1] * q[2]),
        1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
  }

  bool ValidateState_(const robot_io::RobotState& state) {
    if (state.q.size() != kServeDof || state.dq.size() != kServeDof ||
        !AllFinite_(state.q) || !AllFinite_(state.dq) ||
        !state.imu_quat_wxyz.allFinite() || !state.imu_gyro.allFinite() ||
        state.imu_quat_wxyz.norm() < 1.0e-6) {
      TripFault_("invalid_state");
      return false;
    }
    if (!state.sync_complete || !state.sync_aligned) {
      TripFault_("unaligned_state");
      return false;
    }
    for (int sdk = 0; sdk < kServeDof; ++sdk) {
      if (exceeds_joint_hard_limit(
              state.q[sdk], kSdkJointPosLo[sdk], kSdkJointPosHi[sdk], 0.03)) {
        TripFault_("actual_q_hard_limit");
        return false;
      }
    }
    return true;
  }

  ReadyValues Ready_(const robot_io::RobotState& state) {
    ReadyValues out;
    out.q_error = (state.q - default_q_sdk_).cwiseAbs().maxCoeff();
    out.joint_speed = state.dq.cwiseAbs().maxCoeff();
    const Eigen::Vector4d q = state.imu_quat_wxyz.normalized();
    const double gravity_z =
        2.0 * (q[1] * q[1] + q[2] * q[2]) - 1.0;
    out.tilt = std::acos(std::clamp(-gravity_z, -1.0, 1.0));
    out.heading_error = std::abs(
        WrapAngle_(YawFromQuat_(state.imu_quat_wxyz) -
                   initial_heading_rad_));
    out.yaw_rate = std::abs(state.imu_gyro[2]);
    out.base_valid = !config_.require_external_base;
    if (config_.require_external_base) {
      PpBaseSample sample;
      if (base_input_->Latest(sample, config_.base_max_age_s)) {
        const Eigen::Vector2d xy(sample.pos[0], sample.pos[1]);
        const Eigen::Vector2d station(
            kServeGate3StationXM, kServeGate3StationYM);
        const double world_heading_error = std::abs(
            WrapAngle_(YawFromQuat_(sample.quat) -
                       kServeGate3HeadingRad));
        // Keep both guarantees: mocap/world yaw must be the Gate3 +X
        // station heading, and the IMU must not drift away from its
        // engagement heading during playback.  A position-only relay uses
        // the planner's documented identity/square-start fallback.
        out.heading_error =
            std::max(out.heading_error, world_heading_error);
        if (!base_anchor_set_) {
          base_anchor_xy_ = station;
          base_anchor_set_ = true;
          base_prev_xy_ = xy;
          base_prev_seq_ = sample.seq;
          base_prev_stamp_wall_s_ = sample.stamp_wall_s;
        } else if (sample.seq != base_prev_seq_) {
          const double elapsed =
              sample.stamp_wall_s - base_prev_stamp_wall_s_;
          if (elapsed > 1.0e-6) {
            base_xy_speed_mps_ =
                (xy - base_prev_xy_).norm() / elapsed;
            base_velocity_valid_ = std::isfinite(base_xy_speed_mps_);
          } else {
            base_velocity_valid_ = false;
          }
          base_prev_xy_ = xy;
          base_prev_seq_ = sample.seq;
          base_prev_stamp_wall_s_ = sample.stamp_wall_s;
        }
        out.base_position_error = (xy - station).norm();
        out.base_xy_speed = base_xy_speed_mps_;
        out.base_valid = base_anchor_set_ && base_velocity_valid_;
      }
    }
    out.pass =
        out.q_error <= config_.ready_q_error_rad &&
        out.joint_speed <= config_.ready_joint_speed_rad_s &&
        out.tilt <= config_.ready_tilt_rad &&
        out.heading_error <= config_.ready_heading_error_rad &&
        out.yaw_rate <= config_.ready_yaw_rate_rad_s &&
        out.base_valid &&
        out.base_position_error <=
            config_.ready_base_position_error_m &&
        out.base_xy_speed <= config_.ready_base_xy_speed_mps;
    return out;
  }

  void BuildGains_() {
    kp_sdk_ = Eigen::VectorXd::Zero(kServeDof);
    kd_sdk_ = Eigen::VectorXd::Zero(kServeDof);
    static_kp_sdk_ = Eigen::VectorXd::Zero(kServeDof);
    static_kd_sdk_ = Eigen::VectorXd::Zero(kServeDof);
    for (int policy = 0; policy < robot_io::kA3PolicyDof; ++policy) {
      const int sdk = robot_io::kA3PolicyToSdkIdx[policy];
      const bool waist = policy < 3;
      const bool arm = policy >= 3 && policy <= 16;
      const bool leg = policy >= 17;
      if (waist || leg) {
        kp_sdk_[sdk] = a3_pd_stand_kps[policy];
        kd_sdk_[sdk] = a3_pd_stand_kds[policy];
      } else if (arm) {
        kp_sdk_[sdk] = a3_kps[policy] * kServeArmKpScale;
        kd_sdk_[sdk] = a3_kds[policy] * kServeArmKdScale;
        if (policy >= 3 && policy <= 6) {
          kp_sdk_[sdk] *= kServeLeftProximalArmKpBoost;
          kd_sdk_[sdk] *= kServeLeftProximalArmKdBoost;
        } else if (policy >= 10 && policy <= 16) {
          kp_sdk_[sdk] *= kServeRightArmKpBoost;
          kd_sdk_[sdk] *= kServeRightArmKdBoost;
        }
      }
      static_kp_sdk_[sdk] = a3_pd_stand_kps[policy];
      static_kd_sdk_[sdk] = a3_pd_stand_kds[policy];
    }
    kp_sdk_[3] = kServeHeadKp;
    kp_sdk_[4] = kServeHeadKp;
    kd_sdk_[3] = kServeHeadKd;
    kd_sdk_[4] = kServeHeadKd;
    static_kp_sdk_[3] = kServeHeadKp;
    static_kp_sdk_[4] = kServeHeadKp;
    static_kd_sdk_[3] = kServeHeadKd;
    static_kd_sdk_[4] = kServeHeadKd;
  }

  void FillCommand_(const Eigen::VectorXd& q_des,
                    robot_io::RobotCommand& command) const {
    FillCommand_(q_des, Eigen::VectorXd::Zero(kServeDof), command);
  }

  void FillCommand_(const Eigen::VectorXd& q_des,
                    const Eigen::VectorXd& dq_des,
                    robot_io::RobotCommand& command) const {
    command.q_des = q_des;
    command.dq_des = dq_des;
    command.tau_ff = Eigen::VectorXd::Zero(kServeDof);
    command.kp = kp_sdk_;
    command.kd = kd_sdk_;
  }

  void FillHandoffTransitionCommand_(
      const Eigen::VectorXd& q_des,
      robot_io::RobotCommand& command) {
    const double u = static_cast<double>(handoff_gain_tick_) /
                     static_cast<double>(
                         config_.handoff_gain_transition_ticks - 1);
    const double blend = Smooth5_(u);
    command.q_des = q_des;
    command.dq_des = Eigen::VectorXd::Zero(kServeDof);
    command.tau_ff = Eigen::VectorXd::Zero(kServeDof);
    command.kp =
        (1.0 - blend) * kp_sdk_ + blend * static_kp_sdk_;
    command.kd =
        (1.0 - blend) * kd_sdk_ + blend * static_kd_sdk_;
    if (handoff_gain_tick_ <
        config_.handoff_gain_transition_ticks) {
      ++handoff_gain_tick_;
    }
  }

  const PpServeClip& ActiveClip_() const {
    return clip_;
  }

  bool SelectBranch_() {
    PpBallSample ball;
    if (!ball_input_->Latest(ball, config_.ball_max_age_s)) {
      branch_reason_ = "ball_estimate_missing_or_stale";
      return false;
    }
    branch_ball_age_s_ = ball.age_s;
    branch_ball_estimator_samples_ = ball.estimator_samples;
    branch_ball_vx_mps_ = ball.vel_w[0];
    if (ball.frame_code != 0) {
      branch_reason_ = "ball_frame_not_world";
      return false;
    }
    if (ball.estimator_samples < kServeBallMinEstimatorSamples) {
      branch_reason_ = "ball_estimator_sample_count_too_small";
      return false;
    }
    if (ball.seq <= ball_seq_at_confirm_) {
      branch_reason_ = "no_post_confirm_ball_update";
      return false;
    }

    PpBaseSample base;
    if (!base_input_->Latest(base, config_.base_max_age_s)) {
      branch_reason_ = "base_pose_missing_or_stale_at_branch";
      return false;
    }
    branch_ball_position_rel_base_ = ball.pos_w - base.pos;
    const Vec3& p = branch_ball_position_rel_base_;
    const Vec3& v = ball.vel_w;
    const bool position_plausible =
        p[0] >= kServeBallRelXMinM && p[0] <= kServeBallRelXMaxM &&
        p[1] >= kServeBallRelYMinM && p[1] <= kServeBallRelYMaxM &&
        p[2] >= kServeBallRelZMinM && p[2] <= kServeBallRelZMaxM;
    const bool velocity_plausible =
        v[0] >= kServeBallVxMinMps &&
        v[0] <= kServeBallVxMaxMps &&
        std::abs(v[1]) <= kServeBallAbsVyMaxMps &&
        v[2] >= kServeBallVzMinMps &&
        v[2] <= kServeBallVzMaxMps;
    if (!position_plausible || !velocity_plausible) {
      branch_reason_ = "ball_estimate_out_of_prevalidated_envelope";
      return false;
    }
    selected_branch_ = "fixed";
    branch_reason_ = "fresh_ball_envelope_fixed_clip";
    branch_selected_ = true;
    return true;
  }

  Eigen::VectorXd TossOnlyQ_(std::size_t requested_frame) const {
    const std::size_t frame =
        std::min(requested_frame, clip_.size() - 1);
    Eigen::VectorXd q = clip_.frame(frame).q_sdk;
    const int hold_frame = clip_.right_safe_hold_frame();
    if (static_cast<int>(frame) < hold_frame) return q;

    const Eigen::VectorXd& hold_q =
        clip_.frame(static_cast<std::size_t>(hold_frame)).q_sdk;
    const int recovery_start = clip_.events().recovery_start;
    const int handoff_begin = clip_.events().handoff_begin;
    double blend = 0.0;
    if (static_cast<int>(frame) >= handoff_begin) {
      blend = 1.0;
    } else if (static_cast<int>(frame) >= recovery_start) {
      const int transition_intervals =
          std::max(handoff_begin - recovery_start - 1, 1);
      blend = Smooth5_(
          static_cast<double>(static_cast<int>(frame) - recovery_start) /
          static_cast<double>(transition_intervals));
    }
    for (int sdk = 12; sdk <= 18; ++sdk) {
      q[sdk] = (1.0 - blend) * hold_q[sdk] +
               blend * default_q_sdk_[sdk];
    }
    return q;
  }

  void FillTossOnlyFrame_(std::size_t frame,
                          robot_io::RobotCommand& command) const {
    const Eigen::VectorXd q_des = TossOnlyQ_(frame);
    Eigen::VectorXd dq_des = Eigen::VectorXd::Zero(kServeDof);
    const int frame_int = static_cast<int>(frame);
    if (frame > 0 && frame + 1 < clip_.size() &&
        frame_int != clip_.events().toss_commit - 1 &&
        frame_int < clip_.events().handoff_begin) {
      dq_des =
          (TossOnlyQ_(frame + 1) - TossOnlyQ_(frame - 1)) /
          (2.0 * kServeDt);
      if (frame_int >= kServeLeftWristDqCapStartFrame) {
        for (const int source_joint : {27, 29}) {
          const int sdk_joint = clip_.src_to_sdk()[source_joint];
          dq_des[sdk_joint] = std::clamp(
              dq_des[sdk_joint],
              -kServeWristDqReferenceCapRadS,
              kServeWristDqReferenceCapRadS);
        }
      }
    }
    FillCommand_(q_des, dq_des, command);
  }

  void EnterTossOnlyAbort_(const std::string& reason) {
    toss_only_abort_ = true;
    abort_after_commit_ = true;
    successful_play_ = false;
    selected_branch_ = "toss_only_safe_return";
    if (!reason.empty()) branch_reason_ = reason;
  }

  void Play_(const robot_io::RobotState& /*state*/,
             robot_io::RobotCommand& command) {
    if (frame_ >= clip_.size()) {
      successful_play_ =
          !abort_after_commit_ && !toss_only_abort_;
      state_ = ServeControllerState::kHandoffReady;
      ready_ticks_ = 0;
      handoff_gain_tick_ = 0;
      FillCommand_(default_q_sdk_, command);
      return;
    }
    if (frame_ == static_cast<std::size_t>(kServeBranchSelectionFrame) &&
        !branch_selected_) {
      if (!SelectBranch_()) {
        // Ball release has already happened, but the right arm is still on
        // the exact shared high/separated hold.  A whole-body reverse here
        // would disturb the free ball and contradict the MuJoCo-qualified
        // fail-closed path.  Finish the left-hand toss, keep the racket held,
        // then use the clip's recovery interval and never hand off to V17.
        EnterTossOnlyAbort_(branch_reason_);
      }
    }
    if (toss_only_abort_) {
      FillTossOnlyFrame_(frame_, command);
    } else {
      const PpServeClip& active_clip =
          branch_selected_ ? ActiveClip_() : clip_;
      const ServeClipFrame& sample = active_clip.frame(frame_);
      FillCommand_(sample.q_sdk, sample.dq_sdk, command);
    }
    ++frame_;
    if (static_cast<int>(frame_) == clip_.events().toss_commit) {
      // The robot has already reached its right-arm pre-swing pose and the
      // left palm is still stationary.  Pause only now, one frame before the
      // toss, so a ball placed on the rigid palm does not have to survive the
      // long pre-swing dwell without fingers or a gripper.
      state_ = ServeControllerState::kAwaitBall;
      return;
    }
    if (frame_ >= clip_.size()) {
      successful_play_ =
          !abort_after_commit_ && !toss_only_abort_;
      state_ = ServeControllerState::kHandoffReady;
      ready_ticks_ = 0;
      handoff_gain_tick_ = 0;
    }
  }

  void HandleAbortRequest_() {
    if (state_ == ServeControllerState::kPlaying) {
      const int current = static_cast<int>(
          std::min(frame_, clip_.size() - 1));
      if (branch_selected_ &&
          current >= kServeBranchSelectionFrame) {
        // A high-speed right arm must finish the prevalidated follow-through
        // and recovery; never freeze or reverse it in place.
        abort_after_commit_ = true;
        successful_play_ = false;
      } else if (current >= clip_.events().toss_commit) {
        EnterTossOnlyAbort_("operator_abort_after_toss_commit");
      } else {
        abort_start_q_ = last_q_des_;
        abort_tick_ = 0;
        state_ = ServeControllerState::kAbortReturn;
        successful_play_ = false;
      }
    } else if (state_ == ServeControllerState::kAwaitBall) {
      abort_start_q_ = last_q_des_;
      abort_tick_ = 0;
      state_ = ServeControllerState::kAbortReturn;
      successful_play_ = false;
    } else if (state_ == ServeControllerState::kPreflightReady) {
      // Even before the toss, do not replace a partially completed approach
      // with an immediate default command.  Return from the last published
      // q_des through the same zero-endpoint quintic used by other safe aborts.
      abort_start_q_ = last_q_des_;
      abort_tick_ = 0;
      state_ = ServeControllerState::kAbortReturn;
      successful_play_ = false;
    }
  }

  void AbortReturn_(robot_io::RobotCommand& command) {
    const double alpha = Smooth5_(
        static_cast<double>(abort_tick_ + 1) /
        static_cast<double>(config_.abort_return_ticks));
    FillCommand_((1.0 - alpha) * abort_start_q_ +
                     alpha * default_q_sdk_,
                 command);
    ++abort_tick_;
    if (abort_tick_ >= config_.abort_return_ticks) {
      state_ = ServeControllerState::kHandoffReady;
      ready_ticks_ = 0;
      handoff_gain_tick_ = 0;
      successful_play_ = false;
    }
  }

  void TripFault_(const std::string& reason) {
    state_ = ServeControllerState::kFault;
    fault_reason_ = reason;
    handoff_request_.store(false, std::memory_order_release);
  }

  void UpdateDiag_(const ReadyValues& ready, double tracking_error) {
    ServeControllerDiag value;
    value.valid = true;
    value.state = state_;
    if (state_ == ServeControllerState::kPlaying && frame_ < clip_.size()) {
      value.phase =
          (branch_selected_ ? ActiveClip_() : clip_).frame(frame_).phase;
    } else if (state_ == ServeControllerState::kAwaitBall) {
      value.phase = ServePhase::kPreSwing;
    } else if (state_ == ServeControllerState::kHandoffReady ||
               state_ == ServeControllerState::kComplete) {
      value.phase = ServePhase::kHandoffReady;
    }
    value.frame = frame_;
    value.ready_ticks = ready_ticks_;
    value.local_ready = ready.pass;
    value.abort_after_commit = abort_after_commit_;
    value.handoff_request =
        handoff_request_.load(std::memory_order_acquire);
    value.max_q_error = ready.q_error;
    value.max_joint_speed = ready.joint_speed;
    value.tilt_rad = ready.tilt;
    value.heading_error_rad = ready.heading_error;
    value.yaw_rate_rad_s = ready.yaw_rate;
    value.base_valid = ready.base_valid;
    value.base_position_error_m = ready.base_position_error;
    value.base_xy_speed_mps = ready.base_xy_speed;
    value.branch_selected = branch_selected_;
    value.toss_only_abort = toss_only_abort_;
    value.selected_branch = selected_branch_;
    value.ball_age_s = branch_ball_age_s_;
    value.ball_estimator_samples = branch_ball_estimator_samples_;
    value.ball_vx_mps = branch_ball_vx_mps_;
    value.ball_position_rel_base = branch_ball_position_rel_base_;
    value.branch_reason = branch_reason_;
    value.fault_reason = fault_reason_;
    if (tracking_error > value.max_q_error) {
      value.max_q_error = tracking_error;
    }
    std::lock_guard<std::mutex> lock(diag_mutex_);
    diag_ = std::move(value);
  }

  PpServeClip clip_;
  Eigen::VectorXd default_q_sdk_;
  ServeControllerConfig config_;
  Eigen::VectorXd kp_sdk_;
  Eigen::VectorXd kd_sdk_;
  Eigen::VectorXd static_kp_sdk_;
  Eigen::VectorXd static_kd_sdk_;
  Eigen::VectorXd last_q_des_;
  Eigen::VectorXd approach_start_q_;
  Eigen::VectorXd abort_start_q_;
  std::shared_ptr<PpBasePoseInput> base_input_;
  std::shared_ptr<PpBallStateInput> ball_input_;

  std::atomic<ServeControllerState> state_{ServeControllerState::kIdle};
  std::size_t frame_ = 0;
  int approach_tick_ = 0;
  int ready_ticks_ = 0;
  int handoff_gain_tick_ = 0;
  int abort_tick_ = 0;
  double initial_heading_rad_ = 0.0;
  bool base_anchor_set_ = false;
  bool base_velocity_valid_ = false;
  Eigen::Vector2d base_anchor_xy_ = Eigen::Vector2d::Zero();
  Eigen::Vector2d base_prev_xy_ = Eigen::Vector2d::Zero();
  std::uint64_t base_prev_seq_ = 0;
  double base_prev_stamp_wall_s_ = -1.0;
  double base_xy_speed_mps_ = 0.0;
  bool abort_after_commit_ = false;
  bool successful_play_ = false;
  bool branch_selected_ = false;
  bool toss_only_abort_ = false;
  std::string selected_branch_;
  std::string branch_reason_;
  double branch_ball_age_s_ = 0.0;
  int branch_ball_estimator_samples_ = 0;
  double branch_ball_vx_mps_ = 0.0;
  Vec3 branch_ball_position_rel_base_ = Vec3::Zero();
  std::uint64_t ball_seq_at_confirm_ = 0;
  std::string fault_reason_;

  std::atomic<bool> start_requested_{false};
  std::atomic<bool> ball_confirm_requested_{false};
  std::atomic<bool> abort_requested_{false};
  std::atomic<bool> handoff_request_{false};
  mutable std::mutex diag_mutex_;
  ServeControllerDiag diag_{};
};

}  // namespace a3_pingpong
