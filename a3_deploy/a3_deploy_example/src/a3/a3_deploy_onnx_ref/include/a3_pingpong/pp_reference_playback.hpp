#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>

#include <Eigen/Dense>

#include "a3_deploy/numeric_safety.hpp"
#include "a3_pingpong/pp_joint_map.hpp"
#include "a3_policy_parameters.hpp"
#include "robot_io/a3_layout_extra.hpp"
#include "robot_io/robot_io_backend.hpp"

namespace a3_pingpong {

constexpr int kRefDof = 31;
constexpr int kRefHeadSlot0 = 3;
constexpr int kRefHeadSlot1 = 4;
constexpr double kRefHeadPosRad = 0.0;
constexpr double kRefHeadKp = 40.0;
constexpr double kRefHeadKd = 2.0;
constexpr int kRefLegSlotStart = 19;
constexpr int kRefLegSlotCount = 12;
constexpr double kRefPi = 3.14159265358979323846;

enum class RefPlaybackGroup : int {
  kNeckHeadHold = 0,
  kWaist = 1,
  kRightShoulder = 2,
  kRightElbowWrist = 3,
  kRightArm = 4,
  kWaistRightArm = 5,
  kLegsHold = 6,
  kUpperBody = 7,
};

inline const char* RefPlaybackGroupName(RefPlaybackGroup g) {
  switch (g) {
    case RefPlaybackGroup::kNeckHeadHold: return "neck_head_hold";
    case RefPlaybackGroup::kWaist: return "waist";
    case RefPlaybackGroup::kRightShoulder: return "right_shoulder";
    case RefPlaybackGroup::kRightElbowWrist: return "right_elbow_wrist";
    case RefPlaybackGroup::kRightArm: return "right_arm";
    case RefPlaybackGroup::kWaistRightArm: return "waist_right_arm";
    case RefPlaybackGroup::kLegsHold: return "legs_hold";
    case RefPlaybackGroup::kUpperBody: return "upper_body";
  }
  return "unknown";
}

inline RefPlaybackGroup RefPlaybackGroupFromInt(int v) {
  switch (v) {
    case 0: return RefPlaybackGroup::kNeckHeadHold;
    case 1: return RefPlaybackGroup::kWaist;
    case 2: return RefPlaybackGroup::kRightShoulder;
    case 3: return RefPlaybackGroup::kRightElbowWrist;
    case 4: return RefPlaybackGroup::kRightArm;
    case 5: return RefPlaybackGroup::kWaistRightArm;
    case 6: return RefPlaybackGroup::kLegsHold;
    case 7: return RefPlaybackGroup::kUpperBody;
    default: return RefPlaybackGroup::kNeckHeadHold;
  }
}

struct RefPlaybackConfig {
  double dt = 0.02;
  double amplitude_rad = 0.05;
  double frequency_hz = 0.10;
  double gain_scale = 0.25;
  double max_abs_err_rad = 0.30;
  double stale_ms = 250.0;
  bool legs_passive = false;
};

struct RefPlaybackDiagSnapshot {
  bool valid = false;
  bool moving = false;
  bool faulted = false;
  std::string fault_reason;
  RefPlaybackGroup group = RefPlaybackGroup::kNeckHeadHold;
  std::uint64_t tick = 0;
  double time_s = 0.0;
  double max_abs_err = 0.0;
  int active_count = 0;
  std::array<int, kRefDof> active_slots{};
  Eigen::VectorXd q_des;
  Eigen::VectorXd q_meas;
  Eigen::VectorXd kp;
  Eigen::VectorXd kd;
};

class PpReferencePlayback {
 public:
  PpReferencePlayback(const std::array<int, kRefDof>& src_to_sdk,
                      RefPlaybackConfig cfg)
      : src_to_sdk_(src_to_sdk), cfg_(cfg) {
    cfg_.amplitude_rad = Clamp(cfg_.amplitude_rad, 0.0, 0.10);
    cfg_.frequency_hz = Clamp(cfg_.frequency_hz, 0.0, 0.20);
    cfg_.gain_scale = Clamp(cfg_.gain_scale, 0.0, 1.0);
    cfg_.max_abs_err_rad = std::max(0.0, cfg_.max_abs_err_rad);

    sdk_to_src_.fill(-1);
    for (int src = 0; src < kRefDof; ++src) sdk_to_src_[src_to_sdk_[src]] = src;

    nominal_q_sdk_ = Eigen::VectorXd::Zero(kRefDof);
    kp_sdk_ = Eigen::VectorXd::Zero(kRefDof);
    kd_sdk_ = Eigen::VectorXd::Zero(kRefDof);
    for (int i = 0; i < robot_io::kA3PolicyDof; ++i) {
      const int sdk = robot_io::kA3PolicyToSdkIdx[i];
      nominal_q_sdk_[sdk] = a3_default_angles[i];
      kp_sdk_[sdk] = a3_kps[i] * cfg_.gain_scale;
      kd_sdk_[sdk] = a3_kds[i] * cfg_.gain_scale;
    }
    for (int s : {kRefHeadSlot0, kRefHeadSlot1}) {
      nominal_q_sdk_[s] = kRefHeadPosRad;
      kp_sdk_[s] = kRefHeadKp * cfg_.gain_scale;
      kd_sdk_[s] = kRefHeadKd * cfg_.gain_scale;
    }
    hold_q_sdk_ = nominal_q_sdk_;
    last_q_des_ = last_q_meas_ = kp_last_ = kd_last_ = Eigen::VectorXd::Zero(kRefDof);
  }

  void SetGroup(RefPlaybackGroup g) {
    group_.store(static_cast<int>(g), std::memory_order_release);
    relatch_.store(true, std::memory_order_release);
  }

  RefPlaybackGroup group() const {
    return RefPlaybackGroupFromInt(group_.load(std::memory_order_acquire));
  }

  void Start() {
    faulted_.store(false, std::memory_order_release);
    {
      std::lock_guard<std::mutex> lk(fault_mu_);
      fault_reason_.clear();
    }
    relatch_.store(true, std::memory_order_release);
    moving_.store(true, std::memory_order_release);
  }

  void Hold(const std::string& reason = "operator_hold") {
    moving_.store(false, std::memory_order_release);
    relatch_.store(true, std::memory_order_release);
    if (!reason.empty()) {
      std::lock_guard<std::mutex> lk(fault_mu_);
      fault_reason_ = reason;
    }
  }

  void ClearFault() {
    faulted_.store(false, std::memory_order_release);
    {
      std::lock_guard<std::mutex> lk(fault_mu_);
      fault_reason_.clear();
    }
    relatch_.store(true, std::memory_order_release);
  }

  bool moving() const { return moving_.load(std::memory_order_acquire); }
  bool faulted() const { return faulted_.load(std::memory_order_acquire); }

  std::string fault_reason() const {
    std::lock_guard<std::mutex> lk(fault_mu_);
    return fault_reason_;
  }

  bool ComputeCommand(std::uint64_t tick_idx, const robot_io::RobotState& state,
                      robot_io::RobotCommand& cmd) {
    const std::string state_error = ValidateState(state);
    if (!state_error.empty()) {
      TripFault(state_error);
      BuildHoldCommand(state, cmd);
      UpdateDiag(tick_idx, state, cmd);
      return true;
    }

    if (relatch_.exchange(false, std::memory_order_acq_rel) || !hold_valid_) {
      hold_q_sdk_ = state.q;
      hold_valid_ = true;
    }

    if (faulted_.load(std::memory_order_acquire) ||
        !moving_.load(std::memory_order_acquire)) {
      BuildHoldCommand(state, cmd);
      UpdateDiag(tick_idx, state, cmd);
      return true;
    }

    const RefPlaybackGroup g = group();
    Eigen::VectorXd q_src = from_sdk_order(ApplyPassiveHolds(hold_q_sdk_), src_to_sdk_);
    const double t = tick_idx * cfg_.dt;
    ApplyGroupReference(g, t, q_src);

    Eigen::VectorXd q_sdk = to_sdk_order(q_src, src_to_sdk_);
    q_sdk = ApplyPassiveHolds(q_sdk);
    FillCommand(q_sdk, cmd);

    if (!CommandFinite(cmd)) {
      TripFault("nan_or_inf_command");
      BuildHoldCommand(state, cmd);
    } else {
      const double max_err = MaxAbs(cmd.q_des - state.q);
      if (cfg_.max_abs_err_rad > 0.0 && max_err > cfg_.max_abs_err_rad) {
        TripFault("tracking_error_exceeded");
        hold_q_sdk_ = state.q;
        BuildHoldCommand(state, cmd);
      }
    }

    UpdateDiag(tick_idx, state, cmd);
    return true;
  }

  RefPlaybackDiagSnapshot TakeDiag() {
    std::lock_guard<std::mutex> lk(diag_mu_);
    RefPlaybackDiagSnapshot out = diag_;
    diag_.valid = false;
    return out;
  }

  const RefPlaybackConfig& config() const { return cfg_; }

 private:
  static double Clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
  }

  static std::int64_t NowSystemNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
  }

  static bool AllFinite(const Eigen::VectorXd& v) {
    if (v.size() == 0) return false;
    for (int i = 0; i < v.size(); ++i)
      if (!a3_deploy::numeric_safety::IsFinite(v[i])) return false;
    return true;
  }

  static double MaxAbs(const Eigen::VectorXd& v) {
    double m = 0.0;
    for (int i = 0; i < v.size(); ++i) m = std::max(m, std::abs(v[i]));
    return m;
  }

  std::string ValidateState(const robot_io::RobotState& state) const {
    if (state.q.size() != kRefDof) return "bad_q_size";
    if (state.dq.size() != kRefDof) return "bad_dq_size";
    if (!AllFinite(state.q) || !AllFinite(state.dq)) return "nan_or_inf_state";
    if (cfg_.stale_ms > 0.0) {
      if (state.timestamp_ns <= 0) return "stale_state_no_timestamp";
      const double age_ms =
          static_cast<double>(NowSystemNs() - state.timestamp_ns) / 1.0e6;
      if (!a3_deploy::numeric_safety::IsFinite(age_ms) || age_ms > cfg_.stale_ms) {
        return "stale_state_age";
      }
    }
    return {};
  }

  void TripFault(const std::string& reason) {
    moving_.store(false, std::memory_order_release);
    faulted_.store(true, std::memory_order_release);
    std::lock_guard<std::mutex> lk(fault_mu_);
    fault_reason_ = reason;
  }

  Eigen::VectorXd ApplyPassiveHolds(const Eigen::VectorXd& q_in) const {
    Eigen::VectorXd q = q_in;
    q[kRefHeadSlot0] = kRefHeadPosRad;
    q[kRefHeadSlot1] = kRefHeadPosRad;
    if (cfg_.legs_passive) {
      for (int s = kRefLegSlotStart; s < kRefLegSlotStart + kRefLegSlotCount; ++s)
        q[s] = nominal_q_sdk_[s];
    }
    return q;
  }

  void FillCommand(const Eigen::VectorXd& q_sdk, robot_io::RobotCommand& cmd) const {
    cmd.q_des = q_sdk;
    cmd.dq_des = Eigen::VectorXd::Zero(kRefDof);
    cmd.tau_ff = Eigen::VectorXd::Zero(kRefDof);
    cmd.kp = kp_sdk_;
    cmd.kd = kd_sdk_;
  }

  void BuildHoldCommand(const robot_io::RobotState& state,
                        robot_io::RobotCommand& cmd) {
    Eigen::VectorXd hold = hold_valid_ ? hold_q_sdk_ : nominal_q_sdk_;
    if (state.q.size() == kRefDof && AllFinite(state.q)) {
      hold = state.q;
      hold_q_sdk_ = state.q;
      hold_valid_ = true;
    }
    FillCommand(ApplyPassiveHolds(hold), cmd);
  }

  bool CommandFinite(const robot_io::RobotCommand& cmd) const {
    return cmd.q_des.size() == kRefDof && cmd.dq_des.size() == kRefDof &&
           cmd.tau_ff.size() == kRefDof && cmd.kp.size() == kRefDof &&
           cmd.kd.size() == kRefDof && AllFinite(cmd.q_des) &&
           AllFinite(cmd.dq_des) && AllFinite(cmd.tau_ff) &&
           AllFinite(cmd.kp) && AllFinite(cmd.kd);
  }

  void AddSdkDelta(Eigen::VectorXd& q_src, int sdk_slot, double delta) const {
    const int src = sdk_to_src_[sdk_slot];
    if (src >= 0 && src < q_src.size()) q_src[src] += delta;
  }

  void ApplyGroupReference(RefPlaybackGroup g, double t, Eigen::VectorXd& q_src) const {
    const double a = cfg_.amplitude_rad;
    const double w = 2.0 * kRefPi * cfg_.frequency_hz;
    auto s = [&](double phase = 0.0) { return a * std::sin(w * t + phase); };

    auto waist = [&]() {
      AddSdkDelta(q_src, 0,  1.00 * s(0.00));
      AddSdkDelta(q_src, 1,  0.50 * s(0.50));
      AddSdkDelta(q_src, 2, -0.50 * s(1.00));
    };
    auto right_shoulder = [&]() {
      AddSdkDelta(q_src, 12,  1.00 * s(0.00));
      AddSdkDelta(q_src, 13,  0.50 * s(0.60));
      AddSdkDelta(q_src, 14, -0.50 * s(1.20));
    };
    auto right_elbow_wrist = [&]() {
      AddSdkDelta(q_src, 15,  1.00 * s(0.00));
      AddSdkDelta(q_src, 16,  0.50 * s(0.70));
      AddSdkDelta(q_src, 17, -0.50 * s(1.40));
      AddSdkDelta(q_src, 18,  0.25 * s(2.10));
    };
    auto left_arm_small = [&]() {
      AddSdkDelta(q_src, 5,  0.50 * s(0.30));
      AddSdkDelta(q_src, 6, -0.25 * s(0.90));
      AddSdkDelta(q_src, 7,  0.25 * s(1.50));
      AddSdkDelta(q_src, 8,  0.50 * s(0.00));
      AddSdkDelta(q_src, 9,  0.25 * s(0.70));
      AddSdkDelta(q_src, 10, -0.25 * s(1.40));
      AddSdkDelta(q_src, 11,  0.10 * s(2.10));
    };

    switch (g) {
      case RefPlaybackGroup::kNeckHeadHold:
      case RefPlaybackGroup::kLegsHold:
        break;
      case RefPlaybackGroup::kWaist:
        waist();
        break;
      case RefPlaybackGroup::kRightShoulder:
        right_shoulder();
        break;
      case RefPlaybackGroup::kRightElbowWrist:
        right_elbow_wrist();
        break;
      case RefPlaybackGroup::kRightArm:
        right_shoulder();
        right_elbow_wrist();
        break;
      case RefPlaybackGroup::kWaistRightArm:
        waist();
        right_shoulder();
        right_elbow_wrist();
        break;
      case RefPlaybackGroup::kUpperBody:
        waist();
        right_shoulder();
        right_elbow_wrist();
        left_arm_small();
        break;
    }
  }

  static int ActiveSlots(RefPlaybackGroup g, std::array<int, kRefDof>& out) {
    auto add = [&](int s, int& n) { out[n++] = s; };
    int n = 0;
    switch (g) {
      case RefPlaybackGroup::kNeckHeadHold:
        add(3, n); add(4, n);
        break;
      case RefPlaybackGroup::kWaist:
        add(0, n); add(1, n); add(2, n);
        break;
      case RefPlaybackGroup::kRightShoulder:
        add(12, n); add(13, n); add(14, n);
        break;
      case RefPlaybackGroup::kRightElbowWrist:
        add(15, n); add(16, n); add(17, n); add(18, n);
        break;
      case RefPlaybackGroup::kRightArm:
        for (int s = 12; s <= 18; ++s) add(s, n);
        break;
      case RefPlaybackGroup::kWaistRightArm:
        for (int s = 0; s <= 2; ++s) add(s, n);
        for (int s = 12; s <= 18; ++s) add(s, n);
        break;
      case RefPlaybackGroup::kLegsHold:
        for (int s = 19; s <= 30; ++s) add(s, n);
        break;
      case RefPlaybackGroup::kUpperBody:
        for (int s = 0; s <= 18; ++s)
          if (s != 3 && s != 4) add(s, n);
        break;
    }
    return n;
  }

  void UpdateDiag(std::uint64_t tick_idx, const robot_io::RobotState& state,
                  const robot_io::RobotCommand& cmd) {
    if (cmd.q_des.size() != kRefDof || state.q.size() != kRefDof) return;
    const RefPlaybackGroup g = group();
    std::array<int, kRefDof> slots{};
    const int n = ActiveSlots(g, slots);
    double max_err = 0.0;
    for (int i = 0; i < n; ++i) {
      const int s = slots[i];
      max_err = std::max(max_err, std::abs(cmd.q_des[s] - state.q[s]));
    }
    std::lock_guard<std::mutex> lk(diag_mu_);
    diag_.valid = true;
    diag_.moving = moving_.load(std::memory_order_acquire);
    diag_.faulted = faulted_.load(std::memory_order_acquire);
    {
      std::lock_guard<std::mutex> fk(fault_mu_);
      diag_.fault_reason = fault_reason_;
    }
    diag_.group = g;
    diag_.tick = tick_idx;
    diag_.time_s = tick_idx * cfg_.dt;
    diag_.max_abs_err = max_err;
    diag_.active_count = n;
    diag_.active_slots = slots;
    diag_.q_des = cmd.q_des;
    diag_.q_meas = state.q;
    diag_.kp = cmd.kp;
    diag_.kd = cmd.kd;
    last_q_des_ = cmd.q_des;
    last_q_meas_ = state.q;
    kp_last_ = cmd.kp;
    kd_last_ = cmd.kd;
  }

  std::array<int, kRefDof> src_to_sdk_{};
  std::array<int, kRefDof> sdk_to_src_{};
  RefPlaybackConfig cfg_;
  Eigen::VectorXd nominal_q_sdk_;
  Eigen::VectorXd kp_sdk_;
  Eigen::VectorXd kd_sdk_;
  Eigen::VectorXd hold_q_sdk_;
  bool hold_valid_ = false;

  std::atomic<int> group_{static_cast<int>(RefPlaybackGroup::kNeckHeadHold)};
  std::atomic<bool> moving_{false};
  std::atomic<bool> relatch_{true};
  std::atomic<bool> faulted_{false};
  mutable std::mutex fault_mu_;
  std::string fault_reason_;

  mutable std::mutex diag_mu_;
  RefPlaybackDiagSnapshot diag_;
  Eigen::VectorXd last_q_des_;
  Eigen::VectorXd last_q_meas_;
  Eigen::VectorXd kp_last_;
  Eigen::VectorXd kd_last_;
};

}  // namespace a3_pingpong
