// Minimal entry point for running model_15200 (ping-pong, 180-obs/31-act) on the
// A3 via AGI's native runner. Reuses robot_io::A3AimrtBackend (iceoryx/ros2 sync)
// + a3_deploy::A3PolicyDriver (watchdog + safe-halt). With --serve, the
// serve025 full31 action and the policy path share one 100 Hz publisher; the
// 50 Hz learned policy command is held on the intervening callback.
// only the front-end is ours (a3_pingpong::PpPolicy CommandFn). AGI's original
// a3_deploy_onnx_ref + main.cpp are untouched (separate CMake target).
//
// Staged modes (keyboard, or --start MODE): PASSIVE (limp) -> PD_STAND (hold
// nominal) -> SHADOW (compute, no publish) -> MOTION (publish; --gain-scale for
// low-gain first try; 0/1 = swing level). Neck passive by default. Scripted
// racket targets only (no live planner).
//
// Usage:
//   a3_deploy_onnx_ref_pingpong --runtime-cfg PATH [--policy-dir PATH]
//       [--aimrt-cfg PATH]
//       [--start passive|pd_stand|shadow|motion] [--level 0|1]
//       [--gain-scale F] [--stand-kp K --stand-kd D]
#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <termios.h>
#include <unistd.h>

#include <yaml-cpp/yaml.h>

#include "a3_deploy/a3_policy_driver.hpp"
#include "a3_pingpong/pp_command_safety.hpp"
#include "a3_pingpong/pp_hybrid_lower_policy.hpp"
#include "a3_pingpong/pp_humanlike_policy.hpp"
#include "a3_pingpong/pp_teleop_entry.hpp"
#include "a3_pingpong/pp_teleop_input.hpp"
#include "a3_pingpong/pp_mode_cadence.hpp"
#include "a3_io/serve_hal_clock.hpp"
#ifdef HAS_A3_TA_PROTO
#include "a3_io/serve_release_message.hpp"
#endif
#include "a3_pingpong/pp_policy.hpp"
#include "a3_pingpong/pp_reference_playback.hpp"
#include "a3_pingpong/pp_runner_control.hpp"
#include "a3_pingpong/pp_serve_controller.hpp"
#include "robot_io/a3_aimrt_backend.hpp"

namespace {
namespace fs = std::filesystem;

#ifdef PP_V17_R1_STATIONARY_MUJOCO_REPLAY_BINARY
constexpr bool kV17R1StationaryMujocoReplayBinary = true;
#else
constexpr bool kV17R1StationaryMujocoReplayBinary = false;
#endif
#ifdef PP_V17_R10_P0_GATE3_BINARY
constexpr bool kV17R10P0Gate3Binary = true;
#else
constexpr bool kV17R10P0Gate3Binary = false;
#endif
#ifdef PP_SCHEMA24_TRAINING_SCREEN_GATE3_BINARY
constexpr bool kSchema24TrainingScreenGate3Binary = true;
#else
constexpr bool kSchema24TrainingScreenGate3Binary = false;
#endif
#if defined(__x86_64__) || defined(_M_X64)
constexpr bool kGate3QdesAuditOnlySupported = true;
#else
constexpr bool kGate3QdesAuditOnlySupported = false;
#endif

std::atomic<bool> g_stop{false};
void OnSig(int) { g_stop.store(true); }

using Mode = a3_pingpong::RunnerMode;
const char* ModeName(Mode mode) { return a3_pingpong::RunnerModeName(mode); }

std::uint64_t NewRunnerBootId() {
  const auto wall_now = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count());
  const auto steady_now = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
  const auto mixed = wall_now ^ (steady_now << 1) ^
                     (static_cast<std::uint64_t>(getpid()) *
                      0x9e3779b97f4a7c15ULL);
  const auto folded =
      mixed & (a3_pingpong::kRunnerMaxExactFloatInteger - 1);
  return folded == 0 ? 1 : folded;
}

std::string Flag(int argc, char** argv, const char* name, const std::string& def) {
  for (int i = 1; i < argc - 1; ++i)
    if (std::string(argv[i]) == name) return argv[i + 1];
  return def;
}
bool Has(int argc, char** argv, const char* name) {
  for (int i = 1; i < argc; ++i)
    if (std::string(argv[i]) == name) return true;
  return false;
}
std::string Resolve(const std::string& p, const fs::path& base) {
  fs::path fp(p);
  if (fp.is_absolute() || fs::exists(fp)) return fp.lexically_normal().string();

  std::error_code ec;
  fs::path cursor = fs::weakly_canonical(base, ec);
  if (ec) cursor = fs::absolute(base, ec);
  if (ec) return (base / fp).lexically_normal().string();
  if (!fs::is_directory(cursor, ec)) cursor = cursor.parent_path();

  while (!cursor.empty()) {
    const fs::path candidate = (cursor / fp).lexically_normal();
    if (fs::exists(candidate)) return candidate.string();
    const fs::path parent = cursor.parent_path();
    if (parent == cursor) break;
    cursor = parent;
  }

  return (base / fp).lexically_normal().string();
}

std::string BuildBackendCfg(const YAML::Node& backend, const std::string& aimrt_override,
                            const fs::path& cfgdir, bool no_publish) {
  std::ostringstream ss;
  bool first = true;
  auto add = [&](const std::string& k, const std::string& v) {
    if (!first) ss << ',';
    ss << k << '=' << v;
    first = false;
  };
  std::string aimrt = aimrt_override.empty() ? backend["aimrt_cfg_path"].as<std::string>()
                                             : aimrt_override;
  add("cfg_file_path", Resolve(aimrt, cfgdir));
  if (!backend["sync_mode"]) add("sync_mode", "min_skew_pair");
  if (!backend["sync_hz"]) add("sync_hz", "100");
  for (auto it : backend) {
    const std::string k = it.first.as<std::string>();
    if (k == "aimrt_cfg_path") continue;
    if (it.second.IsScalar()) add(k, it.second.as<std::string>());
  }
  if (no_publish) add("publish_enabled", "false");
  return ss.str();
}

Mode ParseStartMode(const std::string& s, Mode def) {
  if (s == "passive") return Mode::kPassive;
  if (s == "pd_stand") return Mode::kPdStand;
  if (s == "shadow") return Mode::kShadow;
  if (s == "motion") return Mode::kMotion;
  if (s == "serve") return Mode::kServe;
  return def;
}

std::string DefaultServeAsset(const fs::path& cfgdir,
                              const std::string& filename) {
  const fs::path packaged = cfgdir.parent_path() / "motions" / filename;
  if (fs::exists(packaged)) return packaged.lexically_normal().string();
  // Source-tree fallback for host validation. Resolve() walks ancestors of an
  // arbitrary runtime cfg (including artifacts/...) until it reaches the
  // repository root; packaged deployments always take the branch above.
  return Resolve(
      "a3_deploy/a3_deploy_example/assets/a3_runtime/serve/motions/" + filename,
      cfgdir);
}

a3_pingpong::RefPlaybackGroup ParseRefGroup(const std::string& s) {
  if (s == "neck" || s == "head" || s == "neck_head_hold") {
    return a3_pingpong::RefPlaybackGroup::kNeckHeadHold;
  }
  if (s == "waist") return a3_pingpong::RefPlaybackGroup::kWaist;
  if (s == "right_shoulder") return a3_pingpong::RefPlaybackGroup::kRightShoulder;
  if (s == "right_elbow_wrist") return a3_pingpong::RefPlaybackGroup::kRightElbowWrist;
  if (s == "right_arm") return a3_pingpong::RefPlaybackGroup::kRightArm;
  if (s == "waist_right_arm") return a3_pingpong::RefPlaybackGroup::kWaistRightArm;
  if (s == "legs" || s == "legs_hold") return a3_pingpong::RefPlaybackGroup::kLegsHold;
  if (s == "upper_body") return a3_pingpong::RefPlaybackGroup::kUpperBody;
  return a3_pingpong::RefPlaybackGroupFromInt(std::stoi(s));
}

// Per-joint tracking/amplitude block over the last status window (SHADOW/MOTION).
// cmd_range = how far the policy COMMANDS the joint to move; meas_range = how far
// it ACTUALLY moves; trk% = meas/cmd (low => the joint can't follow the command).
void PrintDiagBlock(const a3_pingpong::PpPolicy::DiagSnapshot& d, bool legs_passive) {
  if (!d.valid) return;
  const auto& nm = a3_pingpong::backend_joint_order();
  auto row = [&](int i) {
    const double cr = d.des_range[i], mr = d.meas_range[i];
    std::printf("   %-26s des=%+0.3f q=%+0.3f err=%+0.3f | cmdR=%.3f measR=%.3f trk=%3.0f%% qdpk=%.2f\n",
                nm[i].c_str(), d.q_des[i], d.q_meas[i], d.q_des[i] - d.q_meas[i],
                cr, mr, cr > 1e-3 ? 100.0 * mr / cr : 0.0, d.qd_peak[i]);
  };
  auto group = [&](const char* title, int lo, int hi) {
    std::printf("  -- %s --\n", title);
    for (int i = lo; i <= hi; ++i) row(i);
  };
  // worst tracker among a range (max cmd_range with low trk) for a compact summary
  auto summary = [&](const char* title, int lo, int hi) {
    int wi = lo; double worst = -1;
    for (int i = lo; i <= hi; ++i) {
      const double cr = d.des_range[i];
      const double miss = cr - d.meas_range[i];  // unfollowed command
      if (cr > 0.02 && miss > worst) { worst = miss; wi = i; }
    }
    std::printf("  -- %s -- worst: %s cmdR=%.3f measR=%.3f errpk=%.3f\n", title,
                nm[wi].c_str(), d.des_range[wi], d.meas_range[wi], d.err_peak[wi]);
  };
  std::printf(" [diag] (rad, last window)   des/q/err | cmdR=commanded-range measR=measured-range trk=follow%%\n");
  group("WAIST", 0, 2);
  group("RIGHT ARM (forehand)", 12, 18);
  group("LEFT ARM", 5, 11);
  if (legs_passive) summary("LEGS (held nominal)", 19, 30);
  else group("LEGS (policy-driven)", 19, 30);  // per-joint hip/knee/ankle des/q/err for knee-sink diag
  summary("NECK (passive)", 3, 4);
}

// Obs-debug block: obs vector stats + the localization-dependent slices, so you
// can confirm at a glance that motion_anchor_pos_b ~ 0 in perfect-tracking mode.
// Index map (see pp_obs_builder.hpp build_obs_180):
//   command [0..61] | anchor_pos_b [62..64] | anchor_ori_b [65..70] |
//   base_ang_vel [71..73] | joint_pos_rel [74..104] | joint_vel [105..135] |
//   last_action [136..166] | proj_grav [167..169] | base_target_pos_b [170..171] |
//   racket_target_pos_b [172..174] | racket_target_vel_w [175..177] |
//   time_to_strike [178] | swing_type [179]
void PrintObsDebugBlock(const a3_pingpong::PpPolicy::ObsDebug& d,
                        const Eigen::VectorXd& action,
                        const std::string& planner_status = "") {
  if (!d.valid) return;
  const auto& o = d.obs;
  const long n = (long)o.size();
  const double omin = o.minCoeff(), omax = o.maxCoeff(), omean = o.mean();
  std::printf(" [obs] loc=%s oracle(en=%d fresh=%d age=%.3fs) sync_miss=%llu | dim=%ld "
              "obs[min/mean/max]=[%.3f %.3f %.3f]\n",
              d.oracle_enabled ? "oracle" : "non-oracle",
              d.oracle_enabled ? 1 : 0, d.oracle_fresh ? 1 : 0, d.oracle_age_s,
              (unsigned long long)d.sync_miss, n, omin, omean, omax);
  char src[64];
  if (planner_status.empty()) std::snprintf(src, sizeof src, "SCRIPTED target -- no live planner");
  else std::snprintf(src, sizeof src, "PLANNER: %s", planner_status.c_str());
  // Per-layout goal-block slices. 2026-07-07 fix: this used to classify 175-vs-'everything
  // else = 180' and indexed o[170..179] on a 177-D (and would on a 110-D) obs — OUT OF BOUNDS
  // (Eigen UB in release). Every supported layout now has its own offsets; unknown dims print
  // only the stats line above.
  if (n == a3_pingpong::kObsDim110 || n == a3_pingpong::kObsDim112 ||
      n == a3_pingpong::kObsDim113 || n == a3_pingpong::kObsDim118) {
    // hitter_pure: [96:99] grav, [99:101] e_base,x, [101:103] Δstation(world),
    // [103:106] racket rel base(world), [106:109] vel_w, [109] tts. No swing_type.
    std::printf("   e_base_x=[%+.3f %+.3f]  base_target_dxy=[%+.4f %+.4f]  "
                "racket_rel_base=[%+.4f %+.4f %+.4f]\n",
                o[99], o[100], o[101], o[102], o[103], o[104], o[105]);
    std::printf("   racket_target_vel_w=[%+.3f %+.3f %+.3f]  tts=%.3f  [%s]\n",
                o[106], o[107], o[108], o[109], src);
    if (n == a3_pingpong::kObsDim112)
      std::printf("   reach_level(planner)=%.0f swing_foot_sign(planner)=%+.0f\n",
                  o[110], o[111]);
    if (n == a3_pingpong::kObsDim113 || n == a3_pingpong::kObsDim118)
      std::printf("   base_velocity_xy=[%+.4f %+.4f] localization_age=%.3f\n",
                  o[110], o[111], o[112]);
    if (n == a3_pingpong::kObsDim118)
      std::printf("   gait_vy=%+.3f clocks=[%+.3f %+.3f] mode=%+.0f "
                  "upper_intervention=%.0f\n",
                  o[113], o[114], o[115], o[116], o[117]);
  } else if (n == a3_pingpong::kObsDim175 || n == a3_pingpong::kObsDim177 ||
             n == a3_pingpong::kObsDim) {
    const bool dp175 = (n == a3_pingpong::kObsDim175);
    const bool hp177 = (n == a3_pingpong::kObsDim177);
    if (!dp175 && !hp177) {  // 180 full: anchor_pos + base_target blocks exist
      const double anchor_pos_norm = o.segment<3>(62).norm();
      std::printf("   motion_anchor_pos_b=[%+.4f %+.4f %+.4f] |.|=%.4f  base_target_pos_b=[%+.4f %+.4f]\n",
                  o[62], o[63], o[64], anchor_pos_norm, o[170], o[171]);
    }
    if (hp177)
      std::printf("   base_target_pos_b=[%+.4f %+.4f]\n", o[167], o[168]);
    // racket_target_pos_b start: 180 -> 172 (rel base); 175 -> 167 (rel FK); 177 -> 169 (rel FK).
    const int rp = dp175 ? 167 : hp177 ? 169 : 172;
    std::printf("   racket_target_pos_b=[%+.4f %+.4f %+.4f]  racket_target_vel_w=[%+.3f %+.3f %+.3f]  "
                "tts=%.3f swing=%+.0f(%s)  [%s]\n",
                o[rp], o[rp + 1], o[rp + 2], o[rp + 3], o[rp + 4], o[rp + 5], o[rp + 6], o[rp + 7],
                o[rp + 7] >= 0 ? "FOREHAND" : "BACKHAND", src);
  }
  if (action.size() == a3_pingpong::kNumJoints)
    std::printf("   action[min/mean/max]=[%+.3f %+.3f %+.3f] |a|=%.3f\n",
                action.minCoeff(), action.mean(), action.maxCoeff(), action.norm());
}

// Machine-readable cumulative clamp evidence. The ordinary status line only says how many
// joints were clamped on that sampled tick; this identifies every offending backend joint and
// preserves its hit count and worst discarded request. It is diagnostic only: the safety clamp
// and all Gate thresholds remain unchanged.
void PrintClampAudit(const char* phase, const a3_pingpong::PpPolicy& policy) {
  const auto ticks = policy.clamp_ticks();
  if (ticks == 0) return;
  const auto& names = a3_pingpong::backend_joint_order();
  int active = 0;
  for (int slot = 0; slot < a3_pingpong::kNumJoints; ++slot)
    if (policy.clamp_count_for(slot) > 0) ++active;
  std::printf("[clamp-audit] phase=%s ticks=%llu active=%d", phase,
              static_cast<unsigned long long>(ticks), active);
  for (int slot = 0; slot < a3_pingpong::kNumJoints; ++slot) {
    const auto hits = policy.clamp_count_for(slot);
    if (hits == 0) continue;
    std::printf(" %s=%llu/%llu/%.6f", names[slot].c_str(),
                static_cast<unsigned long long>(hits),
                static_cast<unsigned long long>(ticks), policy.clamp_max_viol_for(slot));
  }
  std::printf("\n");
}

void PrintQdesSlewAudit(const char* phase, const a3_pingpong::PpPolicy& policy) {
  if (!policy.v12_slew_active() || policy.qdes_slew_ticks() == 0) return;
  std::printf(
      "[qdes-slew] phase=%s ticks=%llu saturated_ticks=%llu last_scale=%.3f "
      "last_saturated=%d last_saturated_legs=%d last_max_clip=%.4f "
      "session_max_clip=%.4f envelope_level=%d envelope_mode=%s\n",
      phase, static_cast<unsigned long long>(policy.qdes_slew_ticks()),
      static_cast<unsigned long long>(policy.qdes_slew_saturated_ticks()),
      policy.qdes_slew_scale(), policy.qdes_slew_saturated_count(),
      policy.qdes_slew_saturated_leg_count(), policy.qdes_slew_max_clip_rad(),
      policy.qdes_slew_max_clip_rad_session(), policy.envelope_level(),
      policy.envelope_mode_name());
}

void PrintQdesProjectorAudit(const char* phase, const a3_pingpong::PpPolicy& policy) {
  if (!policy.bounded_qdes_active() || policy.qdes_projector_ticks() == 0) return;
  const int isaac = policy.worst_projected_isaac_joint();
  const int sdk = isaac >= 0 ? policy.isaac_to_sdk()[isaac] : -1;
  const std::string worst =
      sdk >= 0 ? a3_pingpong::backend_joint_order()[sdk] : std::string("none");
  const auto hits = isaac >= 0 ? policy.qdes_projector_joint_count(isaac) : 0;
  std::printf(
      "[qdes-projector] phase=%s ticks=%llu last_active=%d rate=%d tracking=%d "
      "torque=%d infeasible=%d max_norm_debt=%.6f action_util_max=%.6f "
      "interval_width_min=%.6f rate_util_max=%.6f feasible_bounds=%d/%d/%d "
      "worst=%s hits=%llu\n",
      phase, static_cast<unsigned long long>(policy.qdes_projector_ticks()),
      policy.qdes_projector_active_count(), policy.qdes_projector_rate_count(),
      policy.qdes_projector_tracking_count(), policy.qdes_projector_torque_count(),
      policy.qdes_projector_infeasible_count(),
      policy.qdes_projector_max_normalized_error(),
      policy.qdes_feasible_action_utilization_max(),
      policy.qdes_feasible_interval_width_min(),
      policy.qdes_feasible_rate_utilization_max(),
      policy.qdes_feasible_rate_bound_count(),
      policy.qdes_feasible_tracking_bound_count(),
      policy.qdes_feasible_torque_bound_count(), worst.c_str(),
      static_cast<unsigned long long>(hits));
}

void PrintRefDiagBlock(const a3_pingpong::RefPlaybackDiagSnapshot& d) {
  if (!d.valid) return;
  const auto& nm = a3_pingpong::backend_joint_order();
  std::printf(" [ref] group=%s moving=%d fault=%d reason=%s tick=%llu time=%.3f max_abs_err=%.4f\n",
              a3_pingpong::RefPlaybackGroupName(d.group), d.moving ? 1 : 0,
              d.faulted ? 1 : 0, d.fault_reason.empty() ? "-" : d.fault_reason.c_str(),
              (unsigned long long)d.tick, d.time_s, d.max_abs_err);
  for (int i = 0; i < d.active_count; ++i) {
    const int s = d.active_slots[i];
    const double qd = d.q_des.size() == a3_pingpong::kRefDof ? d.q_des[s] : 0.0;
    const double qm = d.q_meas.size() == a3_pingpong::kRefDof ? d.q_meas[s] : 0.0;
    const double kp = d.kp.size() == a3_pingpong::kRefDof ? d.kp[s] : 0.0;
    const double kd = d.kd.size() == a3_pingpong::kRefDof ? d.kd[s] : 0.0;
    std::printf("   %-26s sdk=%02d q_des=%+0.4f q_meas=%+0.4f err=%+0.4f "
                "kp=%0.2f kd=%0.2f group=%s tick=%llu time=%.3f\n",
                nm[s].c_str(), s, qd, qm, qd - qm, kp, kd,
                a3_pingpong::RefPlaybackGroupName(d.group),
                (unsigned long long)d.tick, d.time_s);
  }
}
}  // namespace

int main(int argc, char** argv) {
  setvbuf(stdout, nullptr, _IOLBF, 0);  // line-buffer so status survives kill
  // Wire-format preflight only: no backend initialization or actuator access.
  if (Has(argc, argv, "--inspect-serve-release-wire")) {
#ifdef HAS_A3_TA_PROTO
    aimdk::protocol::HandCommandChannel message;
    a3_io::FillServeReleaseMessage(message, 7, 1'250'000'000);
    for (unsigned char byte : message.SerializeAsString()) std::printf("%02x", byte);
    std::printf("\n");
    return 0;
#else
    std::fprintf(stderr, "native release protobuf support unavailable\n");
    return 2;
#endif
  }
  // Uses the exact live ONNX/deploy loader, before any transport or actuator
  // initialization. Gate3 must not implement a second compatibility validator.
  if (Has(argc, argv, "--inspect-policy")) {
    try {
      const fs::path directory = Flag(argc, argv, "--policy-dir", "");
      if (directory.empty()) throw std::runtime_error("--inspect-policy requires --policy-dir");
      a3_pingpong::PpOnnxPolicy policy(
          (directory / "exported/policy.onnx").string(),
          a3_pingpong::PpOnnxLoadProfile::kProductionStrict,
          (directory / "params/deploy.yaml").string());
      const auto deploy = a3_pingpong::PpDeployConfig::Load(
          (directory / "params/deploy.yaml").string(),
          (directory / "exported/policy.onnx").string());
      YAML::Emitter out;
      out << YAML::BeginMap << YAML::Key << "inspection_version" << YAML::Value << 1
          << YAML::Key << "policy_abi" << YAML::Value << policy.policy_abi()
          << YAML::Key << "observation_dim" << YAML::Value << policy.obs_dim()
          << YAML::Key << "action_dim" << YAML::Value << 31
          << YAML::Key << "step_dt" << YAML::Value << deploy.step_dt
          << YAML::Key << "runtime_contract" << YAML::Value << policy.runtime_contract()
          << YAML::Key << "training_recipe" << YAML::Value << policy.training_recipe()
          << YAML::Key << "training_recipe_version" << YAML::Value << policy.training_recipe_version()
          << YAML::EndMap;
      std::cout << out.c_str() << '\n';
      return 0;
    } catch (const std::exception& e) {
      std::cerr << "POLICY_INSPECTION_FAILED: " << e.what() << '\n';
      return 2;
    }
  }
  // Validation-only entry exits before backend/gripper initialization.
  if (Has(argc, argv, "--validate-serve-timeline")) {
    a3_pingpong::PpServe025FullbodyTimeline timeline;
    std::string error;
    if (!timeline.LoadCsv(Flag(argc, argv, "--validate-serve-timeline", ""), error)) {
      std::cerr << error << "\n";
      return 2;
    }
    if (Has(argc, argv, "--validate-serve-stand")) {
      try {
        const fs::path directory = Flag(argc, argv, "--policy-dir", "");
        if (directory.empty()) throw std::runtime_error("--validate-serve-stand requires --policy-dir");
        a3_pingpong::PpOnnxPolicy policy(
            (directory / "exported/policy.onnx").string(),
            a3_pingpong::PpOnnxLoadProfile::kProductionStrict,
            (directory / "params/deploy.yaml").string());
        std::array<int, 31> mapping{};
        if (!a3_pingpong::build_src_to_sdk(policy.joint_names(), mapping))
          throw std::runtime_error("serve Stand joint mapping invalid");
        const double offset = a3_pingpong::ServeStandLateralOffset(
            a3_pingpong::to_sdk_order(policy.default_q(), mapping), timeline.complete().q_sdk);
        std::cout << "SERVE_STAND_VALID lateral_offset_rad=" << offset << '\n';
      } catch (const std::exception& e) {
        std::cerr << "SERVE_STAND_INVALID: " << e.what() << '\n';
        return 2;
      }
    }
    std::cout << "SERVE_TIMELINE_VALID frames=468 hz=100\n";
    return 0;
  }
  const std::string cfg_path = Flag(argc, argv, "--runtime-cfg", "");
  if (cfg_path.empty()) {
    std::cerr << "usage: " << argv[0]
              << " --runtime-cfg PATH [--aimrt-cfg PATH]"
                 " [--policy-dir PATH] [--teleop-policy-dir PATH]"
                 " [--start passive|pd_stand|shadow|motion]"
                 " [--level 0|1]\n"
                 "       [--backhand] [--legs-passive] [--waist-passive] [--auto-leg-hold]"
                 " [--arm-hold-nominal [--arm-hold-blend S]] [--hold-recover S]"
                 " [--gain-scale F] [--swing-speed F] [--stand-kp K --stand-kd D]\n"
                 "       [--reference-playback|--mode reference-playback]"
                 " [--no-publish|--dry-run] [--warmup-sec S]\n"
                 "       [--planner] (LIVE planner: racket <- /racket/command_flat, base <- /a3/base_pose_flat over ros2;"
                 " [--engage-min-tts S] [--cmd-timeout S] [--invalid-grace S] [--vel-gate-margin M]"
                 " [--pending-expire-after-strike S]"
                 " [--demo] [--policy-native]"
                 " [--ready-x-max M] [--ready-y-max M] [--ready-speed-max MPS]"
                 " [--ready-dwell S] [--station-takeover-blend S]"
                 " [--station-step-margin M] [--no-station-ready] [--station-only])\n"
                 "       [--loc-mode fabricated|perfect_tracking|oracle|external_base]"
                 " [--perfect-tracking] [--oracle-pelvis] [--no-imu-yaw]\n"
                 "       [--oracle-shm PATH] [--oracle-max-age S]"
                 " [--trace-csv PATH] [--obs-csv PATH] [--no-data-csv] [--session-id ID] [--shadow-frozen-clock]\n"
                 "       [--leg-gain-scale F] [--ankle-gain-scale F] [--motion-blend-sec S]"
                 " [--policy-entry-blend-sec S]"
                 " [--squat-guard-rad R] [--tilt-guard G] [--leg-clamp-rad R]"
                 " [--leg-stand-gains] [--leg-smooth-alpha A]\n"
                 "       [--hybrid-lower --hybrid-lower-model PATH]"
                 " [--hybrid-lower-owner legs|roll_legs|roll_pitch_legs|waist_legs]"
                 " [--hybrid-lower-backend ort_cpu]"
                 " [--hybrid-lower-vx MPS --hybrid-lower-vy MPS"
                 " --hybrid-lower-yaw RADPS] [--hybrid-lower-walk-phase]"
                 " [--hybrid-lower-station-hold]\n"
                 "       [--serve [--serve-gripper-socket ABS_PATH]]"
                 " [--serve-timeline FULL31_CSV] [--kernel-mode] [--serve-only]"
                 "\n"
                 "       [--stationary-v17-r1-replay] (x86 MuJoCo-only "
                 "non-certifying binary)"
                 " [--allow-trained-lateral-recovery]"
                 " [--allow-fixed-y-homing]"
                 "\n"
                 "       [--v17-r10-gate3] (x86 MuJoCo/Gate3-only R10 P0)"
                 " [--schema24-training-screen-gate3] (x86 MuJoCo/Gate3-only; "
                 "hardware forbidden)"
                 " [--gate3-qdes-audit-only] (x86 MuJoCo Gate3 telemetry; no "
                 "q_des fail-fast/clamp)"
                 " [--schema3-sim-optional-reach] (x86 MuJoCo Gate3-only "
                 "Schema31/32 L1/L2 training-candidate lane; legacy alias "
                 "--schema31-sim-optional-reach)"
                 " [--gate3-force-every-flight] (x86 MuJoCo Gate3-only; "
                 "bypass rapid-preempt temporal admission gates)"
                 "\n";
    return 2;
  }
  const fs::path cfgdir = fs::path(cfg_path).parent_path();
  YAML::Node cfg = YAML::LoadFile(cfg_path);
  const int background_cpu = cfg["background_cpu"] ? cfg["background_cpu"].as<int>() : -1;
  const int driver_cpu = cfg["driver_cpu"] ? cfg["driver_cpu"].as<int>() : -1;
  if (background_cpu < -1 || background_cpu >= 1024 || driver_cpu < -1 || driver_cpu >= 1024)
    throw std::invalid_argument("background_cpu/driver_cpu must be -1 or valid CPU indices");
  if (!a3_rt::PinCurrentThreadToCpu(background_cpu))
    throw std::runtime_error("cannot pin Runner background thread");
  std::printf("[scheduling] background_cpu=%d driver_cpu=%d\n", background_cpu, driver_cpu);

  const std::string run_mode = Flag(argc, argv, "--mode", "");
  const bool reference_playback_selected =
      Has(argc, argv, "--reference-playback") || run_mode == "reference-playback";
  const bool no_publish = Has(argc, argv, "--no-publish") || Has(argc, argv, "--dry-run");
  const bool stationary_v17_r1_replay =
      Has(argc, argv, "--stationary-v17-r1-replay");
  const bool v17_r10_gate3 = Has(argc, argv, "--v17-r10-gate3");
  const bool schema24_training_screen_gate3 =
      Has(argc, argv, "--schema24-training-screen-gate3");
  const bool moving_station_replay =
      Has(argc, argv, "--allow-trained-lateral-recovery");
  const bool fixed_y_homing_replay =
      Has(argc, argv, "--allow-fixed-y-homing");
  if (moving_station_replay && !stationary_v17_r1_replay) {
    std::cerr << "--allow-trained-lateral-recovery requires the isolated "
                 "--stationary-v17-r1-replay profile\n";
    return 2;
  }
  if (fixed_y_homing_replay && !stationary_v17_r1_replay) {
    std::cerr << "--allow-fixed-y-homing requires the isolated "
                 "--stationary-v17-r1-replay profile\n";
    return 2;
  }
  if (fixed_y_homing_replay && moving_station_replay) {
    std::cerr << "--allow-fixed-y-homing keeps an immutable station and "
                 "cannot be combined with --allow-trained-lateral-recovery\n";
    return 2;
  }
  if (stationary_v17_r1_replay !=
      kV17R1StationaryMujocoReplayBinary) {
    std::cerr
        << (stationary_v17_r1_replay
                ? "--stationary-v17-r1-replay is unavailable in the "
                  "production runner; use the x86 MuJoCo-only replay binary\n"
                : "the x86 V17-r1 replay binary requires the explicit "
                  "--stationary-v17-r1-replay flag\n");
    return 2;
  }
  if (v17_r10_gate3 != kV17R10P0Gate3Binary) {
    std::cerr
        << (v17_r10_gate3
                ? "--v17-r10-gate3 is unavailable in the production/aarch64 binary\n"
                : "this x86 R10 Gate3 binary requires --v17-r10-gate3\n");
    return 2;
  }
  if (schema24_training_screen_gate3 !=
      kSchema24TrainingScreenGate3Binary) {
    std::cerr
        << (schema24_training_screen_gate3
                ? "--schema24-training-screen-gate3 is unavailable in the "
                  "production/aarch64 binary\n"
                : "this x86 Schema24 Gate3-screen binary requires "
                  "--schema24-training-screen-gate3\n");
    return 2;
  }
  const int isolated_profile_count =
      static_cast<int>(stationary_v17_r1_replay) +
      static_cast<int>(v17_r10_gate3) +
      static_cast<int>(schema24_training_screen_gate3);
  if (isolated_profile_count > 1) {
    std::cerr << "isolated replay/Gate3 profiles are mutually exclusive\n";
    return 2;
  }

  // LIVE PLANNER mode (Path B): racket target from /racket/command_flat + mocap base pose
  // from /a3/base_pose_flat, both over the AimRT ros2 backend; body-drive stays iceoryx.
  const bool kernel_mode = Has(argc, argv, "--kernel-mode");
  const bool serve_only = Has(argc, argv, "--serve-only") && !kernel_mode;
  // Keep the deployed native policy/transport contract for Stand gains. Pure
  // Serve never computes policy commands or admits MOTION, and does not wait
  // for a Planner/base producer.
  const bool planner_mode = Has(argc, argv, "--planner") || serve_only || kernel_mode;
  const bool policy_native = Has(argc, argv, "--policy-native");
  const bool gate3_qdes_audit_only =
      Has(argc, argv, "--gate3-qdes-audit-only");
  const bool schema31_sim_optional_reach =
      Has(argc, argv, "--schema3-sim-optional-reach") ||
      Has(argc, argv, "--schema31-sim-optional-reach");
  const bool gate3_force_every_flight =
      Has(argc, argv, "--gate3-force-every-flight");
  const bool demo_mode = Has(argc, argv, "--demo");
  const bool legacy_vel_box_center = Has(argc, argv, "--vel-box-center");
  if (demo_mode && !planner_mode) {
    std::cerr << "--demo is a live-planner velocity mode and requires --planner\n";
    return 2;
  }
  if (policy_native && !planner_mode) {
    std::cerr << "--policy-native requires --planner\n";
    return 2;
  }
  if (gate3_qdes_audit_only &&
      (!kGate3QdesAuditOnlySupported || !planner_mode || !policy_native)) {
    std::cerr << "--gate3-qdes-audit-only is restricted to the x86 MuJoCo "
                 "Gate3 planner + policy-native path\n";
    return 2;
  }
  if (schema31_sim_optional_reach &&
      (!kGate3QdesAuditOnlySupported || !gate3_qdes_audit_only ||
       !planner_mode || !policy_native)) {
    std::cerr << "--schema3-sim-optional-reach is restricted to the explicit "
                 "x86 MuJoCo Gate3 planner + policy-native + q_des-audit lane\n";
    return 2;
  }
  if (gate3_force_every_flight &&
      (!kGate3QdesAuditOnlySupported || !gate3_qdes_audit_only ||
       !planner_mode || !policy_native)) {
    std::cerr << "--gate3-force-every-flight is restricted to the explicit "
                 "x86 MuJoCo Gate3 planner + policy-native + q_des-audit lane\n";
    return 2;
  }
  if (stationary_v17_r1_replay) {
    if (!planner_mode || !policy_native) {
      std::cerr << "stationary V17-r1 replay requires --planner "
                   "--policy-native\n";
      return 2;
    }
    constexpr const char* kForbiddenReplayFlags[] = {
        "--reference-playback", "--demo", "--vel-box-center",
        "--station-only", "--no-station-ready", "--no-stay-if-reachable",
        "--stream-target", "--legs-passive", "--waist-passive",
        "--auto-leg-hold", "--leg-clamp-rad", "--leg-smooth-alpha",
        "--leg-stand-gains", "--no-yaw-align", "--no-fall-guard",
        "--swing-speed", "--gain-scale", "--leg-gain-scale",
        "--ankle-gain-scale", "--ready-x-max", "--ready-y-max",
        "--ready-speed-max", "--ready-dwell", "--gate-x-max",
        "--gate-station-step-max", "--station-step-margin",
    };
    for (const char* flag : kForbiddenReplayFlags) {
      if (Has(argc, argv, flag)) {
        std::cerr << "stationary V17-r1 replay fixes its runtime contract; "
                     "override is forbidden: "
                  << flag << "\n";
        return 2;
      }
    }
  }
  if (v17_r10_gate3 &&
      (!planner_mode || !policy_native || demo_mode ||
       Has(argc, argv, "--stream-target") || Has(argc, argv, "--station-only"))) {
    std::cerr << "V17-r10 Gate3 requires --planner --policy-native, planned "
                 "velocity, strike enabled, and no --stream-target\n";
    return 2;
  }
  if (schema24_training_screen_gate3) {
    if (!planner_mode || !policy_native || !gate3_qdes_audit_only ||
        demo_mode) {
      std::cerr << "Schema24 Gate3 screen requires --planner --policy-native "
                   "--gate3-qdes-audit-only and planned velocity\n";
      return 2;
    }
    constexpr const char* kForbiddenSchema24Gate3Flags[] = {
        "--reference-playback", "--demo", "--vel-box-center",
        "--station-only", "--no-station-ready", "--no-stay-if-reachable",
        "--stream-target", "--legs-passive", "--waist-passive",
        "--auto-leg-hold", "--leg-clamp-rad", "--leg-smooth-alpha",
        "--leg-stand-gains", "--no-yaw-align", "--no-fall-guard",
        "--swing-speed", "--gain-scale", "--leg-gain-scale",
        "--ankle-gain-scale", "--ready-x-max", "--ready-y-max",
        "--ready-speed-max", "--ready-dwell", "--gate-x-max",
        "--gate-station-step-max", "--station-step-margin",
    };
    for (const char* flag : kForbiddenSchema24Gate3Flags) {
      if (Has(argc, argv, flag)) {
        std::cerr << "Schema24 Gate3 screen fixes its policy behavior; "
                     "override is forbidden: "
                  << flag << "\n";
        return 2;
      }
    }
  }
  if (legacy_vel_box_center) {
    std::cerr << "[pingpong] WARN: --vel-box-center is deprecated; use --demo\n";
  }

  const std::string aimrt_override =
      Resolve(Flag(argc, argv, "--aimrt-cfg", ""), cfgdir);
  std::string aimrt_override_arg =
      Has(argc, argv, "--aimrt-cfg") ? aimrt_override : std::string{};
  // In planner mode, default the AimRT transport cfg to the dual-plugin (iceoryx body-drive
  // + ros2 planner inputs) unless the operator passed an explicit --aimrt-cfg.
  if (planner_mode && aimrt_override_arg.empty())
    aimrt_override_arg = Resolve("a3_aimrt_config.pingpong_ros2body.yaml", cfgdir);

  // Unitree-style policy ownership: one policy directory always contains
  // params/deploy.yaml + exported/policy.onnx. Runtime YAML owns only the
  // backend/localization settings. Legacy onnx.model_path remains readable so
  // historical packages do not break, but formal HitterPingPong uses this path.
  std::string policy_dir_raw = Flag(argc, argv, "--policy-dir", "");
  if (policy_dir_raw.empty() && cfg["policy_dir"]) {
    policy_dir_raw = cfg["policy_dir"].as<std::string>();
  }
  std::string policy_dir;
  std::string deploy_cfg_path;
  std::string model_path;
  double deploy_step_dt = 0.0;
  if (!policy_dir_raw.empty()) {
    policy_dir = Resolve(policy_dir_raw, cfgdir);
    deploy_cfg_path =
        (fs::path(policy_dir) / "params" / "deploy.yaml").string();
    model_path =
        (fs::path(policy_dir) / "exported" / "policy.onnx").string();
    if (!fs::is_directory(policy_dir) ||
        !fs::is_regular_file(deploy_cfg_path) ||
        !fs::is_regular_file(model_path)) {
      std::cerr << "policy_dir must contain params/deploy.yaml and "
                   "exported/policy.onnx: " << policy_dir << "\n";
      return 2;
    }
    const auto deploy =
        a3_pingpong::PpDeployConfig::Load(deploy_cfg_path, model_path);
    deploy_step_dt = deploy.step_dt;
  } else {
    if (!cfg["onnx"] || !cfg["onnx"]["model_path"]) {
      std::cerr << "runtime config requires policy_dir (preferred) or legacy "
                   "onnx.model_path\n";
      return 2;
    }
    model_path =
        Resolve(cfg["onnx"]["model_path"].as<std::string>(), cfgdir);
  }
  const double configured_policy_hz =
      cfg["policy_driver"] && cfg["policy_driver"]["policy_hz"]
          ? cfg["policy_driver"]["policy_hz"].as<double>()
          : 0.0;
  const double policy_hz = deploy_step_dt > 0.0
                               ? 1.0 / deploy_step_dt
                               : (configured_policy_hz > 0.0
                                      ? configured_policy_hz
                                      : 50.0);
  if (deploy_step_dt > 0.0 && configured_policy_hz > 0.0 &&
      std::fabs(configured_policy_hz - policy_hz) > 1.0e-9) {
    std::cerr << "policy_driver.policy_hz disagrees with "
                 "policy_dir/params/deploy.yaml step_dt\n";
    return 2;
  }
  if (!(std::isfinite(policy_hz) && policy_hz > 0.0)) {
    std::cerr << "policy_driver.policy_hz must be finite and positive\n";
    return 2;
  }
  const double policy_dt = 1.0 / policy_hz;
  const int level = std::stoi(Flag(argc, argv, "--level", "1"));
  std::atomic<double> gain_scale{std::stod(Flag(argc, argv, "--gain-scale", "1.0"))};
  const double stand_kp = std::stod(Flag(argc, argv, "--stand-kp", "60"));
  const double stand_kd = std::stod(Flag(argc, argv, "--stand-kd", "4"));
  // The official a3_pd_stand gains (knee ~2000) are tuned for the robot bearing
  // its weight ON THE GROUND. On a HOIST they snap/buzz and swing the body, so
  // they are OFF by default; opt in only for free-standing on the ground
  // (Step 2). The hoisted demo uses the gentle flat PD that ran clean before.
  const bool official_stand = Has(argc, argv, "--official-stand");
  // Stretch the swing in real time (<1.0 = slower) so hardware actuators can
  // track it. Native (1.0) under-shoots and strains loudly on the real robot.
  const double swing_speed = std::stod(Flag(argc, argv, "--swing-speed", "1.0"));

  // ---- localization mode (A/B/C) + sim-only oracle config ----
  // yaml: obs_debug.{loc_mode,use_sim_oracle_pelvis_pose,oracle_shm_path,
  //                  oracle_max_age_s,obs_csv}. CLI overrides yaml.
  YAML::Node odbg = cfg["obs_debug"] ? cfg["obs_debug"] : YAML::Node();
  auto odbg_str = [&](const char* k, const std::string& def) {
    return odbg[k] ? odbg[k].as<std::string>() : def;
  };
  std::string loc_mode_s = odbg_str("loc_mode", "perfect_tracking");  // hardware-safe default
  bool yaml_oracle = odbg["use_sim_oracle_pelvis_pose"] &&
                     odbg["use_sim_oracle_pelvis_pose"].as<bool>();
  if (yaml_oracle) loc_mode_s = "oracle";
  if (Has(argc, argv, "--loc-mode")) loc_mode_s = Flag(argc, argv, "--loc-mode", loc_mode_s);
  if (Has(argc, argv, "--perfect-tracking")) loc_mode_s = "perfect_tracking";
  if (Has(argc, argv, "--oracle-pelvis")) loc_mode_s = "oracle";
  // Planner mode needs the target frame to match a REAL base: default to live mocap base
  // (external_base) unless the operator picked a loc mode explicitly (e.g. --oracle-pelvis
  // for the sim closed-loop rehearsal, where sim ground truth plays the mocap role).
  const bool loc_mode_explicit = Has(argc, argv, "--loc-mode") ||
      Has(argc, argv, "--perfect-tracking") || Has(argc, argv, "--oracle-pelvis");
  if (planner_mode && !loc_mode_explicit) loc_mode_s = "external_base";
  if (kernel_mode) loc_mode_s = "kernel_imu_local_actor_test";
  a3_pingpong::LocMode loc_mode = a3_pingpong::LocMode::kFabricated;
  if (kernel_mode) loc_mode = a3_pingpong::LocMode::kKernelImuLocal;
  else if (loc_mode_s == "perfect_tracking" || loc_mode_s == "B" || loc_mode_s == "b")
    loc_mode = a3_pingpong::LocMode::kPerfectTracking;
  else if (loc_mode_s == "oracle" || loc_mode_s == "C" || loc_mode_s == "c")
    loc_mode = a3_pingpong::LocMode::kOracle;
  else if (loc_mode_s == "external_base" || loc_mode_s == "mocap")
    loc_mode = a3_pingpong::LocMode::kExternalBase;
  else if (loc_mode_s == "fabricated" || loc_mode_s == "A" || loc_mode_s == "a")
    loc_mode = a3_pingpong::LocMode::kFabricated;
  else { std::cerr << "unknown loc_mode '" << loc_mode_s << "'\n"; return 2; }
  if (stationary_v17_r1_replay &&
      loc_mode != a3_pingpong::LocMode::kExternalBase) {
    std::cerr << "stationary V17-r1 replay requires fresh external-base "
                 "localization; loc-mode overrides are forbidden\n";
    return 2;
  }
  if (v17_r10_gate3 &&
      loc_mode != a3_pingpong::LocMode::kExternalBase &&
      loc_mode != a3_pingpong::LocMode::kOracle) {
    std::cerr << "V17-r10 Gate3 requires drift-observing external_base or "
                 "sim-only oracle localization\n";
    return 2;
  }
  if (schema24_training_screen_gate3 &&
      loc_mode != a3_pingpong::LocMode::kExternalBase &&
      loc_mode != a3_pingpong::LocMode::kOracle) {
    std::cerr << "Schema24 Gate3 screen requires drift-observing "
                 "external_base or sim-only oracle localization\n";
    return 2;
  }
  const std::string oracle_shm =
      Flag(argc, argv, "--oracle-shm", odbg_str("oracle_shm_path", "/dev/shm/pp_oracle_pelvis"));
  const double oracle_max_age_s = std::stod(
      Flag(argc, argv, "--oracle-max-age", odbg["oracle_max_age_s"]
               ? std::to_string(odbg["oracle_max_age_s"].as<double>()) : "0.1"));
  const bool no_data_csv = Has(argc, argv, "--no-data-csv");
  const std::string obs_csv_path = no_data_csv ? "" : Flag(argc, argv, "--obs-csv", odbg_str("obs_csv", ""));
  const std::string session_id = Flag(argc, argv, "--session-id", "");

  Mode default_mode = Has(argc, argv, "--start")
                          ? ParseStartMode(Flag(argc, argv, "--start", ""), Mode::kPassive)
                          : Mode::kPassive;
  if (stationary_v17_r1_replay &&
      (default_mode != Mode::kPassive || !Has(argc, argv, "--official-stand"))) {
    std::cerr << "stationary V17-r1 replay requires --start passive "
                 "--official-stand\n";
    return 2;
  }
  const bool serve_requested = serve_only || kernel_mode ||
      Has(argc, argv, "--serve") ||
      Has(argc, argv, "--serve-timeline") ||
      Has(argc, argv, "--serve-gripper-socket");
  if (default_mode == Mode::kServe) {
    std::cerr
        << "--start serve is forbidden: start PASSIVE/PD_STAND, set the "
           "SERVER role, then use PREPARE_SERVE through the Runner contract\n";
    return 2;
  }
  if (Has(argc, argv, "--serve-drive-clip") ||
      Has(argc, argv, "--serve-drive-manifest") ||
      Has(argc, argv, "--serve-adaptive-branch") ||
      Has(argc, argv, "--serve-slow-clip") ||
      Has(argc, argv, "--serve-slow-manifest") ||
      Has(argc, argv, "--serve-clip") ||
      Has(argc, argv, "--serve-manifest") ||
      Has(argc, argv, "--serve-transition") ||
      Has(argc, argv, "--serve-entry") ||
      Has(argc, argv, "--serve-timed") ||
      Has(argc, argv, "--serve-recovery")) {
    std::cerr
        << "the build_4 palm/adaptive/drive/slow serve flags are retired; "
           "the active path is the single --serve-timeline named SDK31 CSV\n";
    return 2;
  }
  if (serve_requested && reference_playback_selected) {
    std::cerr << "serve and reference-playback are mutually exclusive\n";
    return 2;
  }
  if (serve_requested &&
      default_mode != Mode::kPassive &&
      default_mode != Mode::kPdStand) {
    std::cerr
        << "the integrated serve Runner must start PASSIVE or PD_STAND; "
           "role-aware actions own every transition to SERVE/MOTION\n";
    return 2;
  }
  if (serve_requested &&
      std::abs(policy_hz - 50.0) > 1.0e-9) {
    std::cerr
        << "integrated serve Runner requires an unchanged 50 Hz ONNX "
           "contract; the 100 Hz Driver holds each receive policy command for "
           "the intervening tick outside direct 100 Hz SERVE playback\n";
    return 2;
  }
  double driver_hz =
      serve_requested ? a3_pingpong::kServePolicyHz : policy_hz;
  const std::string teleop_policy_dir = Flag(argc, argv, "--teleop-policy-dir", "");
  std::unique_ptr<a3_pingpong::PpHumanLikePolicy> locomotion;
  if (!teleop_policy_dir.empty()) {
    if (std::abs(policy_hz-50.0)>1e-9)
      throw std::invalid_argument("teleop requires the existing 50 Hz receive policy contract");
    locomotion = std::make_unique<a3_pingpong::PpHumanLikePolicy>(teleop_policy_dir);
  }
  const auto wall_seconds = [] { return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); };
  const auto steady_seconds = [] { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); };
  // Optional PD_STAND warmup: hold nominal for N s (robot settles upright),
  // then auto-switch to the requested mode. Matches a safe bring-up + lets a
  // non-interactive run reach MOTION from a stable stand.
  const double warmup_sec = std::stod(Flag(argc, argv, "--warmup-sec", "0"));
  const Mode target_mode = default_mode;
  const std::uint64_t runner_boot_id = NewRunnerBootId();
  a3_pingpong::PpTeleopInput teleop_input(runner_boot_id);
  a3_pingpong::PpRunnerControl runner_control(
      warmup_sec > 0 ? Mode::kPdStand : default_mode,
      runner_boot_id, session_id, 16, serve_only);

  // --- backend ---
  auto backend = std::make_unique<robot_io::A3AimrtBackend>();
  const std::string backend_cfg =
      BuildBackendCfg(cfg["backend"], aimrt_override_arg, cfgdir, no_publish);
  std::cout << "[pingpong] backend cfg: " << backend_cfg << "\n";
  if (!backend->Init(backend_cfg)) { std::cerr << "backend Init failed\n"; return 1; }
  backend->SetRunnerControlCallback(
      [&runner_control](const std::vector<double>& values) {
        runner_control.EnqueueFlatRequest(values);
      });
  if (locomotion) backend->SetLocomotionInputCallback(
      [&](const std::vector<double>& values) {
        teleop_input.Receive(values, wall_seconds(), steady_seconds());
      });
  std::cout << "[pingpong] A3AimrtBackend initialised; model=" << model_path;
  if (!deploy_cfg_path.empty()) {
    std::cout << " deploy_cfg=" << deploy_cfg_path
              << " policy_dir=" << policy_dir;
  }
  std::cout << "\n";

  // --- our front-end ---
  a3_pingpong::PpPolicyConfig pcfg;
  pcfg.deploy_cfg_path = deploy_cfg_path;
  pcfg.dt = policy_dt;
  // Planner mode MUST start idle (level 0): the swing level is driven ONLY by the engage
  // machine. Starting at level 1 makes PlannerEngageStep_ see "already swinging" on the
  // first tick and skip the real engage (target/velocity never frozen -> a dead swing).
  pcfg.level = planner_mode ? 0 : level;
  pcfg.legs_passive = Has(argc, argv, "--legs-passive");  // hold legs (hoisted demo)
  // Also hold the WAIST (slots 0..2) at nominal — keeps the torso CoM over the feet
  // when the static legs can't rebalance the policy's forward waist_pitch command.
  // ARMS-ONLY swing. With --official-stand the held waist uses official gains too.
  pcfg.waist_passive = Has(argc, argv, "--waist-passive");
  // AUTO LEG-HOLD: dynamically hold legs+waist at level 0 (stable ready stand, no
  // frozen-windup foot-lift) and release them at level 1 (full-body self-balancing swing).
  // Overrides the manual flags; the initial hold follows the START level.
  pcfg.auto_leg_hold = Has(argc, argv, "--auto-leg-hold");
  if (pcfg.auto_leg_hold) pcfg.legs_passive = pcfg.waist_passive = (level == 0);
  // ARM HOLD (stage cosmetics): during level-0 holds, after 1 s of sustained quiet,
  // ramp the ARM q_des to nominal (kills the model_17400 hold arm-twist without
  // retraining); swings untouched, instant release on any disturbance.
  pcfg.arm_hold_nominal = Has(argc, argv, "--arm-hold-nominal");
  pcfg.arm_hold_blend_s = std::stod(Flag(argc, argv, "--arm-hold-blend", "2.5"));
  // Planner-mode post-swing policy-hold budget before the STATIC official-stand handoff
  // (quiescence-gated as before; this only moves the earliest switch time). Shorter =
  // the official stand (arms at nominal, proven gains) takes over sooner after each
  // swing — the sanctioned way to bound the model_17400 post-swing arm twist, since
  // every runner-side arm override during that hold measurably topples the robot.
  pcfg.hold_recover_s = std::stod(Flag(argc, argv, "--hold-recover", "2.5"));
  // GROUND held joints use AGI's official ground-stand gains (the ONLY config verified to
  // stand free on the ground) when --official-stand is set; the held POSE is identical
  // (nominal == official_stand_q), only the GAINS change. Released joints use the policy PD.
  // (Banner only — the gain loop recomputes this per-tick from the live hold state.)
  const bool legs_official_gains = pcfg.legs_passive && official_stand;
  const bool waist_official_gains = pcfg.waist_passive && official_stand;
  // LEG q_des CLAMP: cap how far the released (level-1) legs may deviate from the
  // nominal upright stand. The trained swing commands a deep crouch (hip_pitch/ankle
  // -0.6..-0.9 rad) that sinks the real robot; clamp to nominal ± band so the legs
  // stay weight-bearing while the arms+waist swing. 0 = off (full policy legs).
  pcfg.leg_clamp_rad = std::stod(Flag(argc, argv, "--leg-clamp-rad", "0.0"));
  // LEG q_des LOW-PASS: EMA-smooth the released leg q_des so stiff --leg-stand-gains track a
  // smooth reference instead of the policy jitter (which they amplify into a twitch). 1.0=off;
  // 0.2-0.3 = moderate. Clamp to (0,1].
  pcfg.leg_smooth_alpha = std::min(1.0, std::max(0.02, std::stod(Flag(argc, argv, "--leg-smooth-alpha", "1.0"))));
  pcfg.swing_speed = swing_speed;
  pcfg.use_base_estimator = Has(argc, argv, "--base-estimator");  // leg-FK pelvis height (ground)
  pcfg.loc_mode = loc_mode;
  // DEFAULT ON since 2026-07-03: with yaw_align the base yaw is engage-relative (starts at
  // identity, tracks the robot's REAL turning) — matching training's rotate-by-current-yaw
  // target transform. Required for turning models (model_9000 turns ~84 deg by design).
  // --no-imu-yaw reverts to the legacy identity-yaw transform (only sensible for a
  // non-turning model, e.g. p4); --use-imu-yaw is still accepted (now a no-op).
  pcfg.use_imu_yaw_for_targets = !Has(argc, argv, "--no-imu-yaw");
  // Scripted swing direction: default forehand; --backhand mirrors the target to
  // +y and selects the baked backhand clip. Toggle live with f/b. (No live planner.)
  pcfg.start_backhand = Has(argc, argv, "--backhand");
  pcfg.oracle_max_age_s = oracle_max_age_s;
  // SINGLE-SWING / REST: avoid the periodic clock's end->windup reference SNAP (untracked
  // in training; topples the free-base backhand). --single-swing: one swing per '1' press,
  // then held stand. --swing-rest S: swing, rest S seconds at held stand, swing again
  // (continuous demo, every swing from a clean windup start).
  pcfg.single_swing = Has(argc, argv, "--single-swing");
  if (Has(argc, argv, "--swing-rest"))
    pcfg.swing_rest_s = std::stod(Flag(argc, argv, "--swing-rest", "1.5"));
  // LIVE PLANNER: one clip per engage + a short settle before re-engaging (the wbc_runner
  // swing_rest semantics). single_swing gives the linear clock + clip-end completion the
  // engage machine relies on; swing_rest_s>=0 arms the inter-swing rest timer.
  pcfg.planner_mode = planner_mode;
  pcfg.kernel_mode = kernel_mode;
  if (kernel_mode) {
    pcfg.yaw_align = true;
    std::cout << "[kernel] IMU entry heading + stance-foot odometry; estimated XY/vxy and FK height; no OptiTrack/Planner; not authoritative localization\n";
  }
  pcfg.gate3_qdes_audit_only = gate3_qdes_audit_only;
  if (planner_mode) {
    pcfg.policy_native = policy_native;
    pcfg.single_swing = true;
    if (!Has(argc, argv, "--swing-rest")) pcfg.swing_rest_s = 0.5;
    pcfg.engage_min_tts_s = std::stod(Flag(argc, argv, "--engage-min-tts", "1.0"));
    // 110-D deep prefix-skip (s): engage up to this far into the clip's near-static ready
    // prefix (late gate = windup - clamp(skip, 0.10*windup, 0.45*windup)); 0 restores the
    // Prefix schedule for non-rally_v14 recipes. model_21800/rally_v14 samples
    // at its fixed dynamic boundary regardless of this value; this is timing,
    // not command admission. See PpPolicyConfig::engage_prefix_skip_s.
    pcfg.engage_prefix_skip_s = std::stod(Flag(argc, argv, "--prefix-skip", "0.20"));
    pcfg.command_timeout_s = std::stod(Flag(argc, argv, "--cmd-timeout", "0.5"));
    pcfg.planner_invalid_grace_s = std::stod(Flag(argc, argv, "--invalid-grace", "0.25"));
    pcfg.pending_expire_after_strike_s =
        std::stod(Flag(argc, argv, "--pending-expire-after-strike", "0.25"));
    // 110-D per-clip trained-velocity-box gate slack (m/s per axis); see PpPolicyConfig.
    pcfg.gate_vel_margin = std::stod(Flag(argc, argv, "--vel-gate-margin", "0.30"));
    // x-readiness engage gate (2026-07-09): reject swings until |station_x − base_x| <=
    // this ("move to station, WAIT, then strike"); <= 0 disables (legacy x-free models).
    pcfg.gate_station_x_max = std::stod(Flag(argc, argv, "--gate-x-max", "0.15"));
    pcfg.gate_station_step_max =
        std::stod(Flag(argc, argv, "--gate-station-step-max", "0.85"));
    pcfg.gate_station_step_margin =
        std::stod(Flag(argc, argv, "--station-step-margin", "0.05"));
    pcfg.station_ready_x_max = std::stod(Flag(argc, argv, "--ready-x-max", "0.10"));
    pcfg.station_ready_y_max = std::stod(Flag(argc, argv, "--ready-y-max", "0.10"));
    pcfg.station_ready_speed_max =
        std::stod(Flag(argc, argv, "--ready-speed-max", "0.20"));
    pcfg.station_ready_hold_s = std::stod(Flag(argc, argv, "--ready-dwell", "0.12"));
    pcfg.station_takeover_blend_s =
        std::stod(Flag(argc, argv, "--station-takeover-blend", "0.15"));
    pcfg.station_ready_enable = !Has(argc, argv, "--no-station-ready");
    // RallyV8 mostly-stationary inversion A/B escape (recipe-gated ON by default).
    pcfg.stay_if_reachable_enable = !Has(argc, argv, "--no-stay-if-reachable");
    pcfg.station_only = Has(argc, argv, "--station-only");
    if (pcfg.station_only && !pcfg.station_ready_enable) {
      std::cerr << "--station-only requires station readiness; remove --no-station-ready\n";
      return 2;
    }
    // Mid-swing target streaming: OFF unless the model trained midswing_resample_prob > 0
    // (the 13200 baseline trained 0.0 — see PpPolicyConfig::stream_target).
    pcfg.stream_target = Has(argc, argv, "--stream-target");
    // C0 contract: C0-B is the default and executes the planner's solved velocity.
    // Explicit --demo selects C0-A: replace velocity with the trained per-clip box center
    // while keeping planner WHERE + WHEN. Keep --vel-box-center as a deprecated alias so
    // existing Gate-3 scripts remain runnable during migration.
    pcfg.vel_cmd_box_center = demo_mode || legacy_vel_box_center;
    pcfg.schema31_sim_optional_reach = schema31_sim_optional_reach;
    pcfg.gate3_force_every_flight = gate3_force_every_flight;
  }
  if (stationary_v17_r1_replay) {
    pcfg.onnx_load_profile =
        a3_pingpong::PpOnnxLoadProfile::kV17R1StationaryMujocoReplay;
    pcfg.fixed_station_replay = !moving_station_replay;
    pcfg.moving_station_replay = moving_station_replay;
    pcfg.fixed_y_homing_replay = fixed_y_homing_replay;
    pcfg.fixed_station_tolerance_m = 0.020;
    if (fixed_y_homing_replay)
      pcfg.station_ready_y_max = 0.030;
  }
  if (v17_r10_gate3) {
    pcfg.onnx_load_profile = a3_pingpong::PpOnnxLoadProfile::kV17R10P0Gate3;
  }
  if (schema24_training_screen_gate3) {
    pcfg.onnx_load_profile =
        a3_pingpong::PpOnnxLoadProfile::kSchema24TrainingScreenGate3;
  }
  // YAW-ALIGN default ON (hardware fix: boot-drifted IMU yaw polluted motion_anchor_ori_b
  // by a constant -12..-38 deg in MDU captures -> the policy fought a fictional torso yaw
  // error with legs/waist and fell during free-standing swings; no-op in sim). Opt out for
  // A/B debugging only.
  pcfg.yaw_align = kernel_mode || !Has(argc, argv, "--no-yaw-align");
  auto pp = std::make_unique<a3_pingpong::PpPolicy>(model_path, pcfg);
  if (pp->onnx().uses_late_reveal_raw_tts_contract()) {
    if (std::abs(pp->onnx().hitter_pingpong_policy_dt_s() - policy_dt) >
        1.0e-12) {
      std::cerr << "late-reveal Hitter policy period mismatch: ONNX dt="
                << pp->onnx().hitter_pingpong_policy_dt_s()
                << " s, runtime dt=" << policy_dt << " s; refusing to run\n";
      return 2;
    }
    if (std::abs(swing_speed - 1.0) > 1.0e-12) {
      std::cerr << "late-reveal Hitter Planner requires --swing-speed 1.0; "
                   "a stretched clock would corrupt raw TTS\n";
      return 2;
    }
  }
  if (stationary_v17_r1_replay &&
      !pp->onnx().is_v17_r1_stationary_replay()) {
    std::cerr << "stationary replay loader did not bind the exact V17-r1 "
                 "artifact; refusing to run\n";
    return 2;
  }
  if (v17_r10_gate3 && !pp->onnx().is_v17_r10_p0_gate3()) {
    std::cerr << "V17-r10 Gate3 loader did not bind the exact recipe-10 P0 "
                 "contract; refusing to run\n";
    return 2;
  }
  if (schema24_training_screen_gate3 &&
      !pp->onnx().is_schema24_training_screen_gate3()) {
    std::cerr << "Schema24 Gate3-screen loader did not bind the exact "
                 "TRAINING_SCREENED non-deployable contract; refusing to run\n";
    return 2;
  }
  if (stationary_v17_r1_replay)
    std::cout
        << "[stationary-replay] PROFILE ACCEPTED: "
        << (moving_station_replay
                ? "v17_r1_model16600_moving_station_recovery_noncert; "
                : (fixed_y_homing_replay
                       ? "v17_r1_model16600_fixed_y_homing_noncert; "
                       : "v17_r1_model16600_fixed_station_3ball_noncert; "))
        << "x86 MuJoCo only, certification=false, "
           "hardware_authorized=false\n";
  if (v17_r10_gate3)
    std::cout
        << "[v17-r10-gate3] PROFILE ACCEPTED: immutable session station, "
           "schema-2 three-revision planner, ball-clock release, frozen target; "
           "x86 simulation only, hardware_authorized=false\n";
  if (schema24_training_screen_gate3)
    std::cout
        << "[schema24-training-screen-gate3] PROFILE ACCEPTED: immutable "
           "session HOME, coherent question bank, table-clearance training "
           "debt; x86 MuJoCo screening only, deployment_qualified=false, "
           "hardware_authorized=false\n";
  if (pp->onnx().has_bounded_qdes_contract() &&
      std::abs(pp->onnx().qdes_projector_dt_s() - policy_dt) > 1.0e-9) {
    std::cerr << "bounded_qdes runtime period mismatch: ONNX projector dt="
              << pp->onnx().qdes_projector_dt_s()
              << " s but policy_driver.policy_hz=" << policy_hz
              << " Hz (dt=" << policy_dt << " s); refusing to run\n";
    return 2;
  }

  // Optional two-policy composition. This stays inside this Runner: Hope's
  // receive policy produces the full command first, then the robot-side HumanLike
  // 3630->15 policy replaces its owned lower-body joints before the publisher.
  const bool hybrid_lower_requested =
      Has(argc, argv, "--hybrid-lower") ||
      Has(argc, argv, "--hybrid-lower-model");
  const bool hybrid_lower_station_hold =
      Has(argc, argv, "--hybrid-lower-station-hold");
  const double hybrid_lower_station_kp = std::max(
      0.0, std::stod(Flag(argc, argv, "--hybrid-lower-station-kp", "0.8")));
  const double hybrid_lower_station_vmax = std::max(
      0.0, std::stod(Flag(argc, argv, "--hybrid-lower-station-vmax", "0.20")));
  const double hybrid_lower_heading_kp = std::max(
      0.0, std::stod(Flag(argc, argv, "--hybrid-lower-heading-kp", "1.0")));
  const double hybrid_lower_yaw_max = std::max(
      0.0, std::stod(Flag(argc, argv, "--hybrid-lower-yaw-max", "0.40")));
  std::unique_ptr<a3_pingpong::PpHybridLowerPolicy> hybrid_lower;
  if (hybrid_lower_requested) {
    const std::string owner_name =
        Flag(argc, argv, "--hybrid-lower-owner", "roll_pitch_legs");
    a3_pingpong::PpHybridLowerOwner owner;
    if (owner_name == "legs") {
      owner = a3_pingpong::PpHybridLowerOwner::kLegs;
    } else if (owner_name == "roll_legs" ||
               owner_name == "waist_roll_legs") {
      owner = a3_pingpong::PpHybridLowerOwner::kWaistRollAndLegs;
    } else if (owner_name == "roll_pitch_legs" ||
               owner_name == "waist_rp_legs") {
      owner =
          a3_pingpong::PpHybridLowerOwner::kWaistRollPitchAndLegs;
    } else if (owner_name == "waist_legs" || owner_name == "waist+legs") {
      owner = a3_pingpong::PpHybridLowerOwner::kWaistAndLegs;
    } else {
      std::cerr << "--hybrid-lower-owner must be legs, roll_legs, "
                   "roll_pitch_legs, or waist_legs\n";
      return 2;
    }

    const std::string lower_model =
        Flag(argc, argv, "--hybrid-lower-model", "");
    if (lower_model.empty()) {
      std::cerr << "--hybrid-lower requires --hybrid-lower-model PATH\n";
      return 2;
    }

    a3_pingpong::PpHybridLowerConfig hybrid_config;
    hybrid_config.model_path = Resolve(lower_model, cfgdir);
    hybrid_config.backend =
        Flag(argc, argv, "--hybrid-lower-backend", "ort_cpu");
    hybrid_config.owner = owner;
    hybrid_config.outer_policy_dt = 0.01;
    hybrid_config.smart_walk_zero_settled =
        !Has(argc, argv, "--hybrid-lower-walk-phase");
    hybrid_config.command_velocity_xyz = {
        std::stod(Flag(argc, argv, "--hybrid-lower-vx", "0")),
        std::stod(Flag(argc, argv, "--hybrid-lower-vy", "0")),
        std::stod(Flag(argc, argv, "--hybrid-lower-yaw", "0"))};
    hybrid_lower =
        std::make_unique<a3_pingpong::PpHybridLowerPolicy>(hybrid_config);
    if (!hybrid_lower->Initialize()) return 2;
    // HumanLike's observation/action loop is native 100 Hz. receive policy stays
    // at its trained 50 Hz and is held for the intervening 10 ms command.
    driver_hz = 100.0;
  }

  // ---- LIVE PLANNER input wiring (Path B) ----
  // Backend AimRT subscribers (set BEFORE Start()) push decoded Float64MultiArrays into
  // thread-safe holders that PpPolicy reads on the 50 Hz driver thread. The racket topic
  // feeds the engage machine; the base topic feeds LocMode::kExternalBase.
  std::shared_ptr<a3_pingpong::PpBasePoseInput> base_in;
  if (planner_mode) {
    auto racket_in = std::make_shared<a3_pingpong::PpRacketTargetInput>(
        pp->expected_planner_wire_schema());
    base_in = std::make_shared<a3_pingpong::PpBasePoseInput>();
    pp->SetRacketInput(racket_in);
    pp->SetBasePoseInput(base_in);
    backend->SetRacketTargetCallback(
        [racket_in](const std::vector<double>& a) { racket_in->SetFromFlat(a); });
    backend->SetBasePoseCallback(
        [base_in](const std::vector<double>& a) { base_in->SetFromFlat(a); });
    std::cout << "[pingpong] LIVE PLANNER: racket <- /racket/command_flat, base <- "
                 "/a3/base_pose_flat (std_msgs/Float64MultiArray, ros2); body-drive iceoryx\n";
    std::cout << (pcfg.vel_cmd_box_center
                      ? "[pingpong] planner velocity mode: DEMO box-center (--demo); "
                        "planner landing velocity is overridden\n"
                      : "[pingpong] planner velocity mode: PLANNED (default); executing "
                        "planner velocity subject to metadata gates\n");
  }

  // ---- SIM-ONLY oracle localization wiring ----
  // The shm file is produced by scripts/oracle_pose_bridge.py (an rclpy node
  // subscribing /sim/a3/pelvis_pose). On hardware it does not exist, Open() fails,
  // and oracle mode falls back to perfect-tracking with a loud warning.
  if (loc_mode == a3_pingpong::LocMode::kOracle) {
    std::fprintf(stderr,
        "\n*** ORACLE LOCALIZATION ENABLED -- SIMULATION ONLY. DO NOT USE ON "
        "HARDWARE. ***\n    reading true MuJoCo pelvis pose from %s\n\n",
        oracle_shm.c_str());
    auto oracle = std::make_shared<a3_pingpong::PpOraclePose>();
    if (!oracle->Open(oracle_shm)) {
      std::fprintf(stderr,
          "[oracle] shm '%s' not present -> oracle UNAVAILABLE. Start the bridge:\n"
          "    python3 scripts/oracle_pose_bridge.py --shm %s\n"
          "  (oracle mode will fall back to perfect-tracking until then)\n",
          oracle_shm.c_str(), oracle_shm.c_str());
    } else {
      pp->SetOracle(oracle);
    }
  }
  std::cout << "[pingpong] localization mode = " << pp->loc_mode_name() << "\n";
  std::cout << "[pingpong] q_des action contract = "
            << (pp->onnx().has_v12_affine_safe_slew_qdes_contract()
                    ? "v12_affine_safe_slew_qdes_v1 (V11 affine-safe + per-tick slew, "
                      "executed_qdes_raw_v12 feedback, bounded projector OFF)"
                    : (pp->onnx().has_v11_affine_safe_qdes_contract()
                           ? "v11_affine_safe_qdes_v1 (no rate limiter)"
                           : (pp->bounded_qdes_active() ? "bounded/feasible projector"
                                                        : "legacy affine")))
            << "\n";
  std::cout << "[pingpong] recovery envelope (Layer C) mode = " << pp->envelope_mode_name()
            << " (" << a3_pingpong::kRecoveryEnvelopeModeEnv << ")"
            << (pp->envelope_mode() == a3_pingpong::RecoveryEnvelopeMode::kProduction
                    ? "; L2 blocks new flight commits, L3 expires the pending flight"
                    : "; telemetry only, never blocks")
            << "\n";
  std::cout << "[pingpong] racket/base target yaw frame = "
            << (pcfg.use_imu_yaw_for_targets
                    ? "IMU-yaw (absolute; needs a real world-yaw localizer)"
                    : "robot-heading (+x; IMU yaw ignored -- hardware-safe)")
            << "\n";
  const Eigen::VectorXd stand_q =
      a3_pingpong::to_sdk_order(pp->onnx().default_q(), pp->isaac_to_sdk());
  a3_pingpong::RefPlaybackConfig rcfg;
  rcfg.dt = policy_dt;
  rcfg.amplitude_rad = std::stod(Flag(argc, argv, "--ref-amplitude", "0.05"));
  rcfg.frequency_hz = std::stod(Flag(argc, argv, "--ref-frequency", "0.10"));
  rcfg.gain_scale = std::stod(
      Flag(argc, argv, "--ref-gain-scale",
           Flag(argc, argv, "--gain-scale", reference_playback_selected ? "0.25" : "1.0")));
  rcfg.max_abs_err_rad = std::stod(Flag(argc, argv, "--ref-max-err", "0.30"));
  rcfg.stale_ms = std::stod(Flag(argc, argv, "--ref-stale-ms", "250"));
  rcfg.legs_passive = pcfg.legs_passive;
  auto ref = std::make_unique<a3_pingpong::PpReferencePlayback>(pp->isaac_to_sdk(), rcfg);
  ref->SetGroup(ParseRefGroup(Flag(argc, argv, "--ref-group", "0")));
  std::unique_ptr<a3_pingpong::PpServeController> serve;
  a3_io::ServeHalClockReader serve_hal_clock;
  bool serve_hal_clock_enabled = false;
  std::string serve_hal_clock_path;
  std::uint64_t serve_hal_pid = 0;
  std::atomic<std::uint64_t> serve_hal_selected_ns{0}, serve_hal_observed_ns{0};
  std::atomic<bool> serve_hal_selection_missed{false};
  // Opt-in matching Stand/CSV stance. The strict COMPLETE-pose check below
  // rejects a CSV made for another stance before any motion can start.
  if (Has(argc, argv, "--stand-lateral-offset-rad")) {
    try {
      if (!kernel_mode || !official_stand || !serve_requested)
        throw std::invalid_argument("stand lateral offset requires --kernel-mode --official-stand --serve");
      pp->SetStandLateralOffset(std::stod(Flag(argc, argv, "--stand-lateral-offset-rad", "0")));
    } catch (const std::exception& error) {
      std::cerr << "stand stance preflight failed: " << error.what() << "\n";
      return 2;
    }
  }
  std::string serve_timeline_path;
  if (serve_requested) {
    serve_timeline_path =
        Has(argc, argv, "--serve-timeline")
            ? Resolve(Flag(argc, argv, "--serve-timeline", ""), cfgdir)
            : DefaultServeAsset(
                  cfgdir,
                  "a3p_op3_serve025_smooth_center_v14.csv");
    const std::string gripper_socket =
        Flag(argc, argv, "--serve-gripper-socket", "");
    if (!gripper_socket.empty() &&
        !fs::path(gripper_socket).is_absolute()) {
      std::cerr
          << "--serve-gripper-socket must be an absolute private socket\n";
      return 2;
    }
    try {
      a3_pingpong::PpServe025FullbodyTimeline timeline;
      std::string timeline_error;
      if (!timeline.LoadCsv(serve_timeline_path, timeline_error)) {
        throw std::runtime_error(timeline_error);
      }
      if (kernel_mode && official_stand && !Has(argc, argv, "--stand-lateral-offset-rad")) {
        const double offset = a3_pingpong::ServeStandLateralOffset(
            pp->official_stand_q(), timeline.complete().q_sdk);
        pp->SetStandLateralOffset(offset);
        std::cout << "[serve] matched selected CSV Stand: lateral_offset_rad=" << offset << '\n';
      }
      std::string gripper_error;
      std::unique_ptr<a3_pingpong::PpGripperWorker> gripper;
      if (!gripper_socket.empty()) {
        gripper = a3_pingpong::PpGripperWorker::Connect(
            gripper_socket, gripper_error);
        if (!gripper) {
          std::cerr
              << "[serve] WARN: gripper bridge unavailable; "
                 "OPEN/GRAB/RELEASE disabled but arm flow remains enabled: "
              << gripper_error << "\n";
        }
      } else {
        std::cerr
            << "[serve] WARN: no gripper socket; OPEN/GRAB/RELEASE disabled "
               "but arm flow remains enabled\n";
      }
      serve = std::make_unique<a3_pingpong::PpServeController>(
          std::move(timeline), pp->official_stand_q(),
          pp->official_stand_kp(), pp->official_stand_kd(),
          std::move(gripper));
      if (!serve_only) serve->SetPolicyHandoffFrame(
          std::stoul(Flag(argc, argv, "--serve-handoff-frame",
              std::to_string(a3_pingpong::DefaultServeHandoffFrame(kernel_mode, serve_only)))));
      serve->SetPolicyReturnSeconds(std::stod(Flag(argc, argv, "--serve-return-sec", kernel_mode ? "2.5" : "1.0")));
      const std::string release_lead_default = cfg["serve_release_lead_frames"]
          ? std::to_string(cfg["serve_release_lead_frames"].as<unsigned>()) : "0";
      serve->SetReleaseLeadFrames(std::stoul(Flag(argc, argv, "--serve-release-lead-frames", release_lead_default)));
      if (!no_publish) {
        const char* clock_path = std::getenv("HOPE_SERVE_HAL_CLOCK");
        if (clock_path && *clock_path && locomotion && serve->release_frame() == 47) {
          serve_hal_clock_path = clock_path;
          serve_hal_clock_enabled = true;
          a3_io::ServeHalSnapshot receipt;
          const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::steady_clock::now().time_since_epoch()).count();
          if (serve_hal_clock.Open(clock_path) && serve_hal_clock.Read(receipt) && receipt.Ready(now_ns)) {
            serve_hal_pid = receipt.pid;
            std::cout << "[serve_hal_clock] attached pid=" << serve_hal_pid
                      << " telemetry_only=1 csv_clock=fixed_100hz\n";
          } else {
            std::cerr << "[serve_hal_clock] telemetry unavailable at startup; retry attachment in background, no Runner or replay gate\n";
          }
        }
        backend->EnableNativeServeRelease();
        serve->SetNativeReleasePublisher([&backend](std::uint64_t& stamp) {
          return backend->PublishServeRelease(stamp);
        });
      }
    } catch (const std::exception& error) {
      std::cerr << "serve artifact/controller preflight failed: "
                << error.what() << "\n";
      return 2;
    }
    std::cout
        << "[serve] serve025 named SDK31 timeline loaded: csv="
        << serve_timeline_path
        << " handoff_frame=" << serve->handoff_frame()
        << " release_command_frame=" << serve->release_frame()
        << " frames=" << serve->frame_count()
        << " @100Hz, one original CSV FRAME per command tick; no adaptor"
        << "\n"
        << "[serve] fixed flow: SERVER + PD_STAND -> PREPARE_SERVE "
           "(smooth official-stand to CSV frame 0 + immediate GRAB) -> "
           "READY_TO_SERVE -> full31 CSV action; release dispatch uses configured lead, "
           "strike frame=60, contact frame=78. Arm q_des/dq_des follow the selected "
           "SDK31 CSV without a second runtime velocity multiplier. Bounded ankle "
           "pitch support preserves the CSV arm path and stance width. Kp/Kd remain the unscaled official PD_STAND gains and "
           "tau_ff remains zero. RELEASE publishes native protobuf on the body command thread; "
           "OPEN/GRAB use the optional bridge. Publication is not physical release feedback. "
           "Recovery hands the same "
           "Runner to Stand in Kernel mode, or P1/P2 receive policy in normal "
           "play, through the continuous command handoff.\n";
  }
  std::cout << "[pingpong] joint map OK; neck PASSIVE (q=0,kp=" << a3_pingpong::kHeadKp
            << ",kd=" << a3_pingpong::kHeadKd << "); start=" << ModeName(default_mode)
            << " level=" << level << " gain_scale=" << gain_scale.load()
            << " swing_speed=" << swing_speed << "\n";
  if (reference_playback_selected) {
    std::cout << "[ref] selected: starts PASSIVE; press a group key 0..7 then r to move. "
              << "amp=" << ref->config().amplitude_rad
              << "rad freq=" << ref->config().frequency_hz
              << "Hz gain_scale=" << ref->config().gain_scale
              << " max_err=" << ref->config().max_abs_err_rad
              << " stale_ms=" << ref->config().stale_ms
              << " no_publish=" << (no_publish ? "true" : "false") << "\n";
  }

  // --- optional per-tick CSV trace (every joint: des/q/qd/kp/kd) for offline diag ---
  const std::string trace_path = no_data_csv ? "" : Flag(argc, argv, "--trace-csv", "");
  std::ofstream trace;
  if (!trace_path.empty()) {
    trace.open(trace_path);
    if (trace) {
      // The actual-q ledger recomputes excess from the same exported bounds and
      // measured q. Preserve round-trip double precision so CSV formatting
      // cannot manufacture a cross-source mismatch near the tolerance edge.
      trace << std::setprecision(17);
      const auto& nm = a3_pingpong::backend_joint_order();
      trace << "tick,ts,wall_time_ns,mode,level,gain,swing,legs_passive,gravx,gravy,gravz"
               ",planner_status,lifecycle_event,lifecycle_reason,localization_fresh"
               ",base_speed_valid,base_speed_xy,pending_active,pending_clip"
               ",pending_station_x,pending_station_y,pending_strike_time"
               ",current_strike_time,current_tts,engage_raw_tts"
               ",engage_clock_tts0,engage_requested_phase_s,engage_actual_phase_s"
               ",engage_expected_strike_lateness_s,late_phase_clamped"
               ",engage_first_tick_qdes_l2,valid_age_s,ready_timer_active"
               ",ready_reported,ready_dwell_s,lifecycle_seq";
      trace << ",shot_seq,planner_msg_seq,planner_flight_id,planner_revision_id"
               ",planner_stable_revision_count,frozen_command_seq,frozen_producer_epoch"
               ",frozen_flight_id"
               ",frozen_revision_id,frozen_strike_time,frozen_raw_tts"
               ",swing_sign,reach_level,swing_foot_sign"
               ",planner_completion_seq,planner_completed_producer_epoch"
               ",planner_completed_flight_id,planner_completion_kind"
               ",base_x,base_y,base_z,base_qw,base_qx,base_qy,base_qz"
               ",target_x,target_y,target_z,target_vx,target_vy,target_vz"
               ",racket_fk_valid,racket_x,racket_y,racket_z"
               ",racket_vx,racket_vy,racket_vz,racket_nx,racket_ny,racket_nz"
               ",session_id,qdes_projector_active,qdes_projector_rate"
               ",qdes_projector_tracking,qdes_projector_torque"
               ",qdes_projector_infeasible,qdes_projector_max_norm_debt"
               ",qdes_feasible_action_util_max,qdes_feasible_interval_width_min"
               ",qdes_feasible_rate_util_max,qdes_feasible_rate_bound"
               ",qdes_feasible_tracking_bound,qdes_feasible_torque_bound"
               ",actual_q_hard_tolerance_rad,actual_q_hard_audit_only"
               ",actual_q_hard_violation_count,actual_q_hard_max_excess_rad";
      for (const auto& n : nm) trace << ",actual_q_hard_lo_" << n;
      for (const auto& n : nm) trace << ",actual_q_hard_hi_" << n;
      for (const auto& n : nm) trace << ",actual_q_hard_excess_" << n;
      for (const auto& n : nm) trace << ",des_" << n;
      for (const auto& n : nm) trace << ",clamp_viol_" << n;
      for (const auto& n : nm) trace << ",q_" << n;
      for (const auto& n : nm) trace << ",qd_" << n;
      for (const auto& n : nm) trace << ",kp_" << n;
      for (const auto& n : nm) trace << ",kd_" << n;
      // v12_affine_safe_slew_qdes_v1 telemetry + RECOVERY ENVELOPE (Layer C). Appended
      // after the per-joint blocks so every existing column keeps its position.
      trace << ",qdes_slew_scale,qdes_slew_saturated_count,qdes_slew_saturated_leg_count"
               ",qdes_slew_max_clip_rad"
               ",envelope_level,envelope_mode,envelope_home_dist_m,envelope_speed_mps"
               ",envelope_tilt_rad,envelope_block_reason"
               ",envelope_support_anchor_error_m";
      trace << "\n";
      std::cout << "[pingpong] trace CSV -> " << trace_path << "\n";
    } else {
      std::cerr << "[pingpong] WARN: cannot open trace csv " << trace_path << "\n";
    }
  }
  std::ofstream* trace_ptr = trace.is_open() ? &trace : nullptr;

  // --- optional per-tick OBS CSV (the full 180-D obs) for A/B/C comparison ---
  std::ofstream obscsv;
  if (!obs_csv_path.empty()) {
    obscsv.open(obs_csv_path);
    if (obscsv) {
      obscsv << "tick,ts,wall_time_ns,mode,loc_mode,oracle_fresh,oracle_age_s,sync_miss,legs_passive,session_id";
      for (int i = 0; i < pp->onnx().obs_dim(); ++i) obscsv << ",obs_" << i;
      for (int i = 0; i < a3_pingpong::kNumJoints; ++i) obscsv << ",act_" << i;
      obscsv << "\n";
      std::cout << "[pingpong] obs CSV -> " << obs_csv_path
                << " (loc_mode=" << pp->loc_mode_name() << ")\n";
    } else {
      std::cerr << "[pingpong] WARN: cannot open obs csv " << obs_csv_path << "\n";
    }
  }
  std::ofstream* obscsv_ptr = obscsv.is_open() ? &obscsv : nullptr;
  const int loc_mode_int = static_cast<int>(loc_mode);

  // SHADOW free-running clock. The driver's `tick` is PUBLISH-GATED: in SHADOW
  // nothing is published, so tick FREEZES at 0 -> the scripted swing clock never
  // advances -> the policy sits on the windup frame and the action converges to a
  // single (clamped) windup command. That is NOT a representative swing preview and
  // looks like a stuck/saturated policy. Fix: in SHADOW drive the scripted clock
  // from this local free-running counter so the obs evolves through the full swing
  // exactly like MOTION (still no publish -> safe). MOTION keeps the driver tick
  // (which intentionally pauses the swing clock during a safe-halt). Opt out with
  // --shadow-frozen-clock to restore the old frozen behavior.
  std::uint64_t shadow_tick = 0;
  const bool shadow_free_clock = !Has(argc, argv, "--shadow-frozen-clock");

  // GROUND-CONTACT gains. On the GROUND the LEGS bear weight: at a uniform low
  // --gain-scale the leg kp (trained ~150-250) becomes far too soft (e.g. ×0.05 ~
  // 10) -> knees sag -> robot falls forward. So scale the LEGS (stance/balance,
  // slots 19..30) by their OWN --leg-gain-scale (default: follow --gain-scale, i.e.
  // unchanged behavior; set e.g. 0.5-1.0 so the legs can actually hold weight on the
  // ground) while --gain-scale keeps the arms/waist (swing) gentle. Neck keeps its
  // fixed PD (unscaled). On the HOIST you can leave leg-gain following gain-scale.
  std::atomic<double> leg_gain_scale{
      Has(argc, argv, "--leg-gain-scale") ? std::stod(Flag(argc, argv, "--leg-gain-scale", "1.0"))
                                          : -1.0};  // <0 => follow --gain-scale
  // ANKLE is the standing-balance joint (ankle strategy: ankle_pitch torque keeps the
  // CoM over the feet). Its TRAINED kp is among the low ones, so a modest --leg-gain-scale
  // still leaves the ankle too soft -> the robot pitches FORWARD about the ankle. Scale the
  // 4 ankle slots (L pitch/roll = 23,24; R pitch/roll = 29,30) by their OWN
  // --ankle-gain-scale so the ankles can be stiff (hold upright) while hips/knees stay
  // gentle. Default: follow --leg-gain-scale. May exceed 1.0 (stiffer than training) if the
  // real ankle needs more than sim to resist tipping.
  std::atomic<double> ankle_gain_scale{
      Has(argc, argv, "--ankle-gain-scale") ? std::stod(Flag(argc, argv, "--ankle-gain-scale", "1.0"))
                                            : -1.0};  // <0 => follow --leg-gain-scale
  // SQUAT/TILT SAFETY GUARD (auto-leg-hold full-body swing only): revert to level 0 (re-engage the
  // held official stand) if a released leg SINKS (knee bends past nominal by > --squat-guard-rad) or
  // the body TILTS (|gravX| or |gravY| > --tilt-guard). <=0 disables that check. A hoist-test backstop.
  // DEFAULT 1.4 (was 0.6): the trip point is nominal_knee(0.247)+guard, and the v2 clip's OWN
  // reference crouch starts at knee 0.62 with swing flexion commanding ~1.14 — a 0.6 guard sits
  // INSIDE the trained swing envelope, so it fired on a healthy swing at ~5 deg tilt and the
  // kp-2000 official-stand snap-back CATAPULTED the robot backward (the captured 2026-07-02 fall).
  // 1.4 (trip at ~1.65) still catches a genuine leg collapse; real tilt is the tilt guard's job.
  const double squat_guard_rad = std::stod(Flag(argc, argv, "--squat-guard-rad", "1.4"));
  const double tilt_guard = std::stod(Flag(argc, argv, "--tilt-guard", "0.35"));
  // Legacy fall guard (2026-07-04): every free-base test showed a FALLEN robot happily
  // "swinging" on the floor — the only fall-adjacent check (tilt guard above) is gated on
  // --auto-leg-hold and only drops to level 0 (still publishing a stiff stand). If the
  // pelvis tilts past ~60 deg (|gravZ| < 0.5) for fall_guard_ticks consecutive ticks in
  // any publishing mode, drop to PASSIVE (zero gains): stiff commands on a downed robot
  // thrash it against the floor and burn motors. Disable (hoist rigs that pitch the
  // pelvis, debugging) with --no-fall-guard.
  // A field operator remains the final authority, but policy-native execution
  // must retain the automatic fall cutoff as a backstop.
  const bool fall_guard = !Has(argc, argv, "--no-fall-guard");
  const double fall_guard_gz = -0.5;   // body-frame gravity z above this = fallen
  int fall_guard_ticks = 0;
  // LEG WEIGHT-BEARING: the released (level-1) legs default to the POLICY leg PD x --leg-gain-scale,
  // whose kp (~150 knee) is ~13x softer than AGI's official ground-stand knee kp (2000) -> the knees
  // SINK under real body load. --leg-stand-gains keeps the official ground-stand PD on the legs even
  // when RELEASED (policy still drives the CLAMPED q_des), so they bear weight like the level-0 hold
  // while making small swing-coupled moves. REQUIRES --official-stand; pair with a TIGHT --leg-clamp-rad
  // (0.15-0.20) since stiff gains drive the leg firmly to whatever the clamp allows. Default off.
  const bool leg_stand_gains = Has(argc, argv, "--leg-stand-gains");
  // POSE-BLEND on MOTION entry: ramp the published q_des from the pose at the moment
  // MOTION engaged to the policy target over this many seconds, so (now-stiff) legs
  // do NOT snap through the ~1.5-2 rad stand->windup jump. Convex blend of two
  // in-range poses -> stays in range. 0 disables.
  const double motion_blend_sec = std::stod(Flag(argc, argv, "--motion-blend-sec", "0.5"));
  if (!std::isfinite(motion_blend_sec) || motion_blend_sec < 0)
    throw std::invalid_argument("--motion-blend-sec must be finite and nonnegative");
  // V17 field logs showed policy-native silently forcing this to zero, which
  // exposed the full stand->policy q_des discontinuity. Keep at least 0.5 s.
  // The affine policy's low-gain controller needs time to acquire its gravity
  // compensation while the standing impedance fades. Keep this independent of
  // PASSIVE -> PD_STAND, whose reset/stand timing must not change.
  const double default_policy_entry_sec = policy_native
      ? std::max(pp->onnx().has_v11_affine_safe_qdes_contract() ? 3.0 : 0.5,
                 motion_blend_sec)
      : motion_blend_sec;
  const double policy_motion_blend_sec = std::stod(Flag(
      argc, argv, "--policy-entry-blend-sec", std::to_string(default_policy_entry_sec)));
  if (!std::isfinite(policy_motion_blend_sec) || policy_motion_blend_sec < 0)
    throw std::invalid_argument("--policy-entry-blend-sec must be finite and nonnegative");
  const double serve_policy_blend_sec = std::stod(Flag(argc, argv, "--serve-policy-blend-sec", "0.5"));
  if (!std::isfinite(serve_policy_blend_sec) || serve_policy_blend_sec < 0)
    throw std::invalid_argument("--serve-policy-blend-sec must be finite and nonnegative");
  double active_policy_blend_sec = policy_motion_blend_sec;
  const bool v17_command_safety = pp->onnx().is_rally_v17_recipe();
  a3_pingpong::PpCommandSafetyMonitor command_safety;
  bool authoritative_mocap_stale_warned = false;  // driver-thread only (no race)
  Mode prev_mode_for_blend = Mode::kPassive;       // driver-thread only (no race)
  a3_pingpong::PpCommandTransition mode_transition;
  robot_io::RobotCommand last_delivered_command;
  robot_io::RobotState command_state;
  bool last_delivered_valid = false;
  Mode last_delivered_mode = Mode::kPassive;
  Mode computed_mode = Mode::kPassive;
  double transition_elapsed_s = 0.0;
  std::uint64_t motion_enter_tick = 0;
  int prev_level_for_blend = level;                // legacy re-arm state; native mode ignores toggles
  int prev_swing_dir_for_blend = pcfg.start_backhand ? -1 : 1;
  // Trace-only state. It measures the first final q_des jump of each engaged
  // planner shot and has no connection to command admission or generation.
  std::uint64_t trace_previous_shot_seq = 0;
  Eigen::VectorXd trace_previous_q_des;
  // AUTO LEG-HOLD: hold legs+waist at level 0 (stable ready stand, no frozen-windup foot-lift),
  // release them at level 1 (full-body self-balancing swing). The pose-blend (re-armed on the
  // level toggle below) ramps q_des from the current measured pose so the stiff official stand
  // gains do NOT snap the legs on the 1->0 re-engage (the "jump").
  const bool auto_leg_hold = pcfg.auto_leg_hold;

  // --- mode-aware CommandFn (reuses driver's RT loop + watchdog) ---
  a3_pingpong::PpPolicy* ppp = pp.get();
  a3_pingpong::PpReferencePlayback* refp = ref.get();
  a3_pingpong::PpServeController* servep = serve.get();
  a3_pingpong::PpHybridLowerPolicy* hybridp = hybrid_lower.get();
  if (locomotion) driver_hz = 500.0;
  std::atomic<int> teleop_phase{0}; // off, entering, active, stopping
  std::atomic<double> teleop_progress{0};
  std::array<std::atomic<double>,3> teleop_velocity{};
  a3_pingpong::PpTeleopEntry teleop_entry;
  bool teleop_entered = false;
  std::uint64_t teleop_tick = 0;
  int teleop_quiet_ticks = 0, teleop_fall_ticks = 0;
  robot_io::RobotCommand hybrid_hope_command_cache;
  bool hybrid_hope_command_valid = false;
  bool hybrid_run_hope_next = true;
  double hybrid_cached_blend_alpha = 1.0;
  bool hybrid_station_anchor_set = false;
  Eigen::Vector2d hybrid_station_anchor_w = Eigen::Vector2d::Zero();
  double hybrid_heading_anchor_rad = 0.0;
  std::uint64_t hybrid_station_last_log_tick = 0;
  std::atomic<std::uint64_t> native_evaluations{0};
  auto command_fn_50hz = [ppp, refp, servep, hybridp, &native_evaluations, &runner_control, &gain_scale, &leg_gain_scale, &ankle_gain_scale,
                     stand_q, stand_kp, stand_kd,
                     official_stand, auto_leg_hold, policy_native, kernel_mode,
                     squat_guard_rad, tilt_guard, leg_stand_gains,
                     trace_ptr, obscsv_ptr, loc_mode_int, session_id,
                     &shadow_tick, shadow_free_clock, motion_blend_sec,
                     policy_motion_blend_sec, serve_policy_blend_sec, &active_policy_blend_sec, policy_dt,
                     &prev_mode_for_blend, &motion_enter_tick,
                     &prev_level_for_blend, &prev_swing_dir_for_blend,
                     &trace_previous_shot_seq, &trace_previous_q_des,
                     &mode_transition, &last_delivered_command, &last_delivered_valid,
                     &last_delivered_mode, &computed_mode, &transition_elapsed_s, &command_state,
                     fall_guard, fall_guard_gz, &fall_guard_ticks,
                     v17_command_safety, &command_safety,
                     &authoritative_mocap_stale_warned,
                     &hybrid_hope_command_cache,
                     &hybrid_hope_command_valid,
                     &hybrid_run_hope_next,
                     &hybrid_cached_blend_alpha,
                     base_in,
                     hybrid_lower_station_hold,
                     hybrid_lower_station_kp,
                     hybrid_lower_station_vmax,
                     hybrid_lower_heading_kp,
                     hybrid_lower_yaw_max,
                     &hybrid_station_anchor_set,
                     &hybrid_station_anchor_w,
                     &hybrid_heading_anchor_rad,
                     &hybrid_station_last_log_tick](
                        std::uint64_t tick, const robot_io::RobotState& st,
                        robot_io::RobotCommand& cmd) -> bool {
    command_state = st;
    native_evaluations.fetch_add(1, std::memory_order_relaxed);
    Mode m = runner_control.mode();
    const std::uint64_t driver_tick = tick;
    if (hybridp != nullptr) tick = driver_tick / 2;
    const int N = 31;
    bool publish = true;
    const auto wall_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    // Refresh the IMU-derived gravity diagnostic FIRST (every tick, every mode), so the squat/tilt
    // guard and the [status]/trace gravZ see the real base orientation. ComputeCommand (which also
    // sets it) does not run in PASSIVE/PD_STAND, so without this the ground checks read a frozen
    // [0,0,-1].
    ppp->observe_imu(st);
    const bool authoritative_mocap_stale =
        m == Mode::kMotion && ppp->authoritative_mocap_required() &&
        !ppp->authoritative_mocap_fresh();
    if (authoritative_mocap_stale && !authoritative_mocap_stale_warned) {
      std::fprintf(stderr,
          "[pp telemetry] authoritative mocap stale; MOTION remains active and the last "
          "mocap pose is retained\n");
      authoritative_mocap_stale_warned = true;
    } else if (!authoritative_mocap_stale && authoritative_mocap_stale_warned) {
      if (m == Mode::kMotion)
        std::fprintf(stderr, "[pp telemetry] authoritative mocap recovered\n");
      authoritative_mocap_stale_warned = false;
    }
    // ALWAYS-ON FALL GUARD (see flag above): fallen -> PASSIVE, independent of --auto-leg-hold.
    if (fall_guard && (m == Mode::kMotion || m == Mode::kPdStand)) {
      const auto gfg = ppp->last_proj_grav();
      if (gfg[2] > fall_guard_gz) {
        if (++fall_guard_ticks >= 25) {  // ~0.5 s persistent at 50 Hz
          runner_control.SetRuntimeMode(Mode::kPassive);
          fall_guard_ticks = 0;
          std::fprintf(stderr,
              "[pp SAFETY] FALL GUARD: gravZ=%+.2f > %.2f for 0.5 s -> PASSIVE (zero gains). "
              "Stand the robot up, then 's' -> 'm' to re-engage.\n", gfg[2], fall_guard_gz);
        }
      } else {
        fall_guard_ticks = 0;
      }
    }
    // SQUAT/TILT SAFETY GUARD (auto-leg-hold, full-body swing only): if a released leg SINKS (knee
    // bends past nominal by > squat_guard_rad) or the body TILTS (|gravX|/|gravY| > tilt_guard),
    // revert to level 0 so the held official stand re-stiffens the legs. Backstop for hoist tests.
    if (auto_leg_hold && ppp->level() == 1) {
      const auto g = ppp->last_proj_grav();
      bool trip = false; const char* why = "";
      if (tilt_guard > 0.0 && (std::abs(g[0]) > tilt_guard || std::abs(g[1]) > tilt_guard)) {
        trip = true; why = "tilt";
      }
      if (!trip && squat_guard_rad > 0.0 && st.q.size() == N) {
        const auto& nomq = ppp->official_stand_q();  // slots 22=L knee, 28=R knee
        if (std::abs(st.q[22] - nomq[22]) > squat_guard_rad ||
            std::abs(st.q[28] - nomq[28]) > squat_guard_rad) { trip = true; why = "knee-sink"; }
      }
      if (trip) {
        ppp->set_level(0);  // re-engage held stand (the auto-hold below stiffens legs+waist this tick)
        std::fprintf(stderr, "[pp SAFETY] %s guard tripped -> reverted to level 0 (held official "
                             "stand); press 1 to retry once stable\n", why);
      }
    }
    // AUTO LEG-HOLD: flip the leg+waist hold from the live level (0=hold ready, 1=release swing)
    // BEFORE the policy/gain code runs this tick (a guard trip above already forced level 0).
    if (auto_leg_hold) {
      const bool hold = (ppp->level() == 0);
      ppp->set_legs_passive(hold);
      ppp->set_waist_passive(hold);
    }
    // Re-arm the pose-blend on MOTION entry. Legacy diagnostic modes also re-arm
    // it on level/side changes; policy-native field mode must not suppress the
    // first 0.5 s of every ball while the strike clock keeps advancing.
    // The source is the last delivered complete command, including gains and velocity.
    const int cur_level = ppp->level();
    const int cur_swing_dir = ppp->swing_dir();
    const bool level_just_changed = (cur_level != prev_level_for_blend);
    const bool dir_just_changed = (cur_swing_dir != prev_swing_dir_for_blend);
    prev_level_for_blend = cur_level;
    prev_swing_dir_for_blend = cur_swing_dir;
    const bool motion_just_entered = (m == Mode::kMotion && prev_mode_for_blend != Mode::kMotion);
    const bool stand_just_entered = (m == Mode::kPdStand && prev_mode_for_blend != Mode::kPdStand);
    const bool rearm_blend = motion_just_entered ||
        (!policy_native && (level_just_changed || dir_just_changed));
    if (motion_just_entered) active_policy_blend_sec =
        prev_mode_for_blend == Mode::kServe ? serve_policy_blend_sec : policy_motion_blend_sec;
    if (rearm_blend) motion_enter_tick = tick;
    ppp->set_mode_handoff_hold(m == Mode::kMotion &&
        static_cast<double>(tick-motion_enter_tick)*policy_dt < active_policy_blend_sec);
    // Re-capture the IMU yaw-align offsets whenever the POLICY (SHADOW/MOTION) engages
    // from a non-policy mode — the operator may have moved/turned the robot in between.
    const bool policy_just_engaged =
        (m == Mode::kMotion || m == Mode::kShadow) &&
        prev_mode_for_blend != Mode::kMotion && prev_mode_for_blend != Mode::kShadow;
    const bool serve_to_policy =
        policy_just_engaged && prev_mode_for_blend == Mode::kServe;
    if (serve_to_policy && !policy_native) {
      ppp->rearm_static_policy_handoff();
    } else if (policy_just_engaged) {
      ppp->rearm_yaw_align();
    }
    if ((policy_just_engaged || motion_just_entered) && hybridp != nullptr) {
      hybridp->Reset();
      hybrid_run_hope_next = true;
      hybrid_hope_command_valid = false;
      hybrid_station_anchor_set = false;
    }
    if (hybridp != nullptr && m != Mode::kMotion && m != Mode::kShadow) {
      hybrid_run_hope_next = true;
      hybrid_hope_command_valid = false;
      hybrid_station_anchor_set = false;
    }
    const bool serve_just_entered = m == Mode::kServe && prev_mode_for_blend != m;
    prev_mode_for_blend = m;
    computed_mode = m;
    if (m == Mode::kPassive || m == Mode::kShadow) mode_transition.Reset();
    if (m == Mode::kPassive) {  // limp: hold current pose, zero gains
      cmd.q_des = st.q.size() == N ? st.q : Eigen::VectorXd::Zero(N);
      cmd.dq_des = Eigen::VectorXd::Zero(N);
      cmd.tau_ff = Eigen::VectorXd::Zero(N);
      cmd.kp = Eigen::VectorXd::Zero(N);
      cmd.kd = Eigen::VectorXd::Zero(N);
    } else if (m == Mode::kPdStand) {  // hold nominal stand pose (== a3_default_angles)
      cmd.q_des = official_stand ? ppp->official_stand_q() : stand_q;
      cmd.dq_des = Eigen::VectorXd::Zero(N);
      cmd.tau_ff = Eigen::VectorXd::Zero(N);
      if (official_stand) {  // production ground-stand gains (free-standing, Step 2)
        cmd.kp = ppp->official_stand_kp();
        cmd.kd = ppp->official_stand_kd();
      } else {               // gentle flat PD — clean on a HOIST (default)
        cmd.kp = Eigen::VectorXd::Constant(N, stand_kp);
        cmd.kd = Eigen::VectorXd::Constant(N, stand_kd);
      }
    } else if (m == Mode::kReferencePlayback) {
      if (!refp->ComputeCommand(tick, st, cmd)) return false;
    } else if (m == Mode::kServe) {
      if (servep == nullptr) {
        std::fprintf(stderr,
                     "[serve] mode entered without a qualified controller\n");
        return false;
      }
      const bool prepare_requested = runner_control.ConsumeServePrepare();
      if (serve_just_entered || prepare_requested) {
        if (last_delivered_valid) servep->SetEntryCommand(last_delivered_command);
        // Start on the command thread before consuming the old COMPLETE state.
        // Publishing SERVE from the action worker and calling Start afterwards
        // races this callback on every repeat of the serve -> ready -> serve loop.
        servep->Start();
      }
      if (!servep->ComputeCommand(tick, st, cmd)) return false;
      if (servep->state() ==
                 a3_pingpong::ServeControllerState::kAborted) {
        runner_control.SetRuntimeMode(Mode::kPdStand);
        std::fprintf(stderr,
                     "[serve] abort recovery complete -> PD_STAND"
                     " (cleanup_required=%d)\n",
                     servep->cleanup_required() ? 1 : 0);
      } else if (servep->state() ==
                 a3_pingpong::ServeControllerState::kComplete) {
        runner_control.CompleteServe(kernel_mode);
        std::fprintf(stderr, runner_control.serve_only()
            ? "[serve] CSV complete -> PD_STAND (Pure Serve)\n"
            : kernel_mode ? "[serve] measured stand recovery complete -> PD_STAND (Kernel; arms lowered)\n"
            : "[serve] measured stand recovery complete -> MOTION; entering P1/P2 "
              "receive policy through continuous handoff\n");
      } else if (servep->state() ==
                 a3_pingpong::ServeControllerState::kFault) {
        runner_control.SetRuntimeMode(Mode::kPdStand);
        const std::string serve_fault = servep->fault_reason();
        std::fprintf(stderr,
                     "[serve] controller fault -> PD_STAND; reason=%s; "
                     "restart/inspect before retry\n",
                     serve_fault.empty() ? "unknown" : serve_fault.c_str());
      }
    } else {  // SHADOW or MOTION: run the policy
      const bool run_hope_this_tick =
          hybridp == nullptr || hybrid_run_hope_next ||
          !hybrid_hope_command_valid;
      double motion_blend_alpha = 1.0;
      if (run_hope_this_tick) {
      // In SHADOW the driver's publish-gated `tick` is frozen, so drive the swing
      // from a free-running counter for a representative no-publish preview (the obs
      // then evolves through the swing like MOTION). MOTION uses the driver tick.
      const bool shadow = (m == Mode::kShadow);
      const std::uint64_t clk = (shadow && shadow_free_clock) ? shadow_tick : tick;
      if (shadow && shadow_free_clock) ++shadow_tick;
      if (!ppp->ComputeCommand(clk, st, cmd)) return false;
      // per-group gain: legs (slots 19..30) by --leg-gain-scale so they can bear
      // weight on the ground; arms+waist by --gain-scale (gentle swing); neck keeps
      // its fixed PD (unscaled). leg_gain<0 => follow gain-scale (hoist / legacy).
      const double g_arm = gain_scale.load();
      const double g_leg_o = leg_gain_scale.load();
      const double g_leg = (g_leg_o >= 0.0) ? g_leg_o : g_arm;
      const double g_ank_o = ankle_gain_scale.load();
      const double g_ank = (g_ank_o >= 0.0) ? g_ank_o : g_leg;  // ankle: own gain, else follow leg
      // Per-tick (so --auto-leg-hold's level toggle takes effect): legs/waist that are
      // HELD with --official-stand get AGI's ground-stand gains; released joints get the
      // policy gains scaled below. ppp->legs_passive()/waist_passive() reflect the live hold.
      // --leg-stand-gains keeps the WEIGHT-BEARING official gains on the legs even when
      // RELEASED (policy still drives the clamped q_des) so the knees don't sink under load.
      const bool leg_official = official_stand && (ppp->legs_passive() || leg_stand_gains);
      const bool waist_held_off = official_stand && ppp->waist_passive();
      if (cmd.kp.size() == N && cmd.kd.size() == N) {
        for (int i = 0; i < N; ++i) {
          if (i == a3_pingpong::kHeadSlot0 || i == a3_pingpong::kHeadSlot1) continue;  // neck fixed PD
          const bool is_leg = (i >= a3_pingpong::kLegSlotStart &&
                               i < a3_pingpong::kLegSlotStart + a3_pingpong::kLegSlotCount);
          const bool is_ankle = (i == 23 || i == 24 || i == 29 || i == 30);  // L/R ankle pitch+roll
          const bool is_waist = (i >= a3_pingpong::kWaistSlotStart &&
                                 i < a3_pingpong::kWaistSlotStart + a3_pingpong::kWaistSlotCount);
          if ((is_leg && leg_official) || (is_waist && waist_held_off)) {
            // GROUND weight-bearing joint (held, or released under --leg-stand-gains):
            // overwrite with AGI's official ground-stand gains VERBATIM (the config proven
            // to stand free on the ground); ignore --gain/--leg/--ankle-gain-scale so a
            // stray scale can't soften the stance.
            cmd.kp[i] = ppp->official_stand_kp()[i];
            cmd.kd[i] = ppp->official_stand_kd()[i];
            continue;
          }
          const double s = is_ankle ? g_ank : (is_leg ? g_leg : g_arm);
          cmd.kp[i] *= s; cmd.kd[i] *= s;
        }
      }
      // The complete post-compose command is blended below, including gains.
      if (m == Mode::kMotion && active_policy_blend_sec > 1e-6) {
        const double elapsed = static_cast<double>(tick - motion_enter_tick) * policy_dt;
        motion_blend_alpha = std::clamp(elapsed / active_policy_blend_sec, 0.0, 1.0);
      }
        if (hybridp != nullptr) {
          hybrid_hope_command_cache = cmd;
          hybrid_hope_command_valid = true;
          hybrid_cached_blend_alpha = motion_blend_alpha;
        }
      } else {
        // Hold receive policy's complete post-gain/post-blend command for the
        // intervening 10 ms. HumanLike below still sees the newest plant state
        // and evaluates a fresh 100 Hz lower-body action.
        cmd = hybrid_hope_command_cache;
        motion_blend_alpha = hybrid_cached_blend_alpha;
      }
      // Compose after Hope's gain scaling, then blend the complete result below.
      // AGI owns the selected joints once the entry blend finishes. This emits
      // one final 31-DOF RobotCommand through the existing Runner publisher.
      if (hybridp != nullptr && hybrid_lower_station_hold && base_in != nullptr) {
        a3_pingpong::PpBaseSample base_sample;
        std::array<double, 3> lower_command =
            hybridp->config().command_velocity_xyz;
        if (base_in->Latest(base_sample, 0.20, true)) {
          const double qw = base_sample.quat[0];
          const double qx = base_sample.quat[1];
          const double qy = base_sample.quat[2];
          const double qz = base_sample.quat[3];
          const double yaw = std::atan2(
              2.0 * (qw * qz + qx * qy),
              1.0 - 2.0 * (qy * qy + qz * qz));
          if (!hybrid_station_anchor_set) {
            hybrid_station_anchor_w = base_sample.pos.head<2>();
            hybrid_heading_anchor_rad = yaw;
            hybrid_station_anchor_set = true;
          }

          Eigen::Vector2d station_target_w = hybrid_station_anchor_w;
          const auto planner = ppp->planner_trace_snapshot(tick);
          if (planner.pending_active &&
              std::isfinite(planner.pending_station_x) &&
              std::isfinite(planner.pending_station_y)) {
            station_target_w = Eigen::Vector2d(
                planner.pending_station_x, planner.pending_station_y);
          }
          Eigen::Vector2d velocity_w =
              hybrid_lower_station_kp *
              (station_target_w - base_sample.pos.head<2>());
          const double speed = velocity_w.norm();
          if (speed > hybrid_lower_station_vmax && speed > 1.0e-9)
            velocity_w *= hybrid_lower_station_vmax / speed;
          const double cy = std::cos(yaw);
          const double sy = std::sin(yaw);
          lower_command[0] += cy * velocity_w[0] + sy * velocity_w[1];
          lower_command[1] += -sy * velocity_w[0] + cy * velocity_w[1];
          double yaw_error = hybrid_heading_anchor_rad - yaw;
          yaw_error = std::atan2(std::sin(yaw_error), std::cos(yaw_error));
          lower_command[2] += std::clamp(
              hybrid_lower_heading_kp * yaw_error,
              -hybrid_lower_yaw_max, hybrid_lower_yaw_max);

          if (driver_tick >= hybrid_station_last_log_tick + 100) {
            hybrid_station_last_log_tick = driver_tick;
            std::printf(
                "[hybrid-lower station] base=(%+.3f,%+.3f,%+.1fdeg) "
                "target=(%+.3f,%+.3f) cmd=(%+.3f,%+.3f,%+.3f)\n",
                base_sample.pos[0], base_sample.pos[1],
                yaw * 180.0 / 3.14159265358979323846,
                station_target_w[0], station_target_w[1],
                lower_command[0], lower_command[1], lower_command[2]);
          }
        }
        hybridp->SetCommandVelocityTarget(lower_command);
      }
      if (!kernel_mode && hybridp != nullptr &&
          !hybridp->Apply(
              st, cmd,
              !ppp->legs_passive(), !ppp->waist_passive())) {
        throw std::runtime_error(
            "hybrid-lower inference/compose failed");
      }
      if (hybridp != nullptr)
        hybrid_run_hope_next = !run_hope_this_tick;
      publish = (m == Mode::kMotion);  // SHADOW computes but does not publish
      // --- OBS CSV row (only when the policy ran, so obs is current) ---
      if (obscsv_ptr) {
        const auto d = ppp->take_obs_debug();
        if (d.valid) {
          auto& o = *obscsv_ptr;
          o << tick << ',' << ppp->last_time_step() << ',' << wall_time_ns << ','
            << static_cast<int>(m) << ','
            << loc_mode_int << ',' << (d.oracle_fresh ? 1 : 0) << ',' << d.oracle_age_s
            << ',' << d.sync_miss << ',' << (ppp->legs_passive() ? 1 : 0)
            << ',' << session_id;
          for (int i = 0; i < d.obs.size(); ++i) o << ',' << d.obs[i];
          const auto& a = ppp->last_action();
          for (int i = 0; i < a3_pingpong::kNumJoints; ++i)
            o << ',' << (a.size() == a3_pingpong::kNumJoints ? a[i] : 0.0);
          o << '\n';
          static int oc = 0; if (++oc % 25 == 0) o.flush();  // single RT writer
        }
      }
    }
    if (publish && (m == Mode::kPdStand || m == Mode::kMotion)) {
      const bool entry = stand_just_entered || (m == Mode::kMotion && rearm_blend);
      const double duration = m == Mode::kMotion ? active_policy_blend_sec : motion_blend_sec;
      if (entry) {
        mode_transition.Reset();
        transition_elapsed_s = 0.0;
        if (duration > 1e-6) {
          robot_io::RobotCommand source = last_delivered_command;
          if (!last_delivered_valid || last_delivered_mode == Mode::kPassive ||
              last_delivered_mode == Mode::kShadow) {
            source = cmd;
            source.q_des = st.q;
            source.dq_des = st.dq;
            source.tau_ff.setZero();
            source.kp.setZero();
            source.kd.setZero();
          }
          if (m == Mode::kMotion && policy_native &&
              ppp->onnx().has_v11_affine_safe_qdes_contract())
            mode_transition.BeginImpedance(source, cmd, duration);
          else
            mode_transition.Begin(source, cmd, duration);
        }
      }
      mode_transition.Apply(transition_elapsed_s, cmd);
      transition_elapsed_s += hybridp != nullptr ? 0.01 : policy_dt;
    }

    // Final command safety runs after every override and entry blend. An
    // exception is caught by A3PolicyDriver and becomes a latched safe-halt.
    if (publish && cmd.q_des.size() == N) {
      if (m == Mode::kMotion && v17_command_safety) {
        command_safety.ValidateAndAdvance(cmd.q_des, st.q, st.dq);
      } else {
        command_safety.Seed(cmd.q_des);
      }
    }

    // --- CSV trace row (all modes; final post-gain command + measured state) ---
    if (trace_ptr) {
      auto& o = *trace_ptr;
      const auto g = ppp->last_proj_grav();
      const auto planner = ppp->planner_trace_snapshot(tick);
      const bool actor_mode =
          m == Mode::kMotion || m == Mode::kShadow;
      const double trace_reach_level =
          actor_mode ? planner.reach_level : 0.0;
      const double trace_swing_foot_sign =
          actor_mode ? planner.swing_foot_sign : 0.0;
      const bool has = (st.q.size() == N && st.dq.size() == N &&
                        cmd.q_des.size() == N && cmd.kp.size() == N && cmd.kd.size() == N);
      double engage_first_tick_qdes_l2 = 0.0;
      if (has && planner.shot_seq != 0 &&
          planner.shot_seq != trace_previous_shot_seq &&
          trace_previous_q_des.size() == N) {
        engage_first_tick_qdes_l2 = (cmd.q_des - trace_previous_q_des).norm();
      }
      o << tick << ',' << ppp->last_time_step() << ',' << wall_time_ns << ','
        << static_cast<int>(m) << ','
        << ppp->level() << ',' << gain_scale.load() << ',' << ppp->swing_speed() << ','
        << (ppp->legs_passive() ? 1 : 0) << ','
        << g[0] << ',' << g[1] << ',' << g[2] << ','
        << planner.status << ',' << planner.lifecycle_event << ',' << planner.lifecycle_reason << ','
        << (planner.localization_fresh ? 1 : 0) << ','
        << (planner.base_speed_valid ? 1 : 0) << ',' << planner.base_speed_xy << ','
        << (planner.pending_active ? 1 : 0) << ',' << planner.pending_clip << ','
        << planner.pending_station_x << ',' << planner.pending_station_y << ','
        << planner.pending_strike_time << ',' << planner.current_strike_time << ','
        << planner.current_tts << ',' << planner.engage_raw_tts << ','
        << planner.engage_clock_tts0 << ',' << planner.engage_requested_phase_s << ','
        << planner.engage_actual_phase_s << ','
        << planner.engage_expected_strike_lateness_s << ','
        << (planner.late_phase_clamped ? 1 : 0) << ','
        << engage_first_tick_qdes_l2 << ','
        << planner.valid_age_s << ','
        << (planner.ready_timer_active ? 1 : 0) << ','
        << (planner.ready_reported ? 1 : 0) << ',' << planner.ready_dwell_s << ','
        << planner.lifecycle_seq << ',' << planner.shot_seq << ',' << planner.planner_msg_seq << ','
        << planner.planner_flight_id << ',' << planner.planner_revision_id << ','
        << planner.planner_stable_revision_count << ','
        << planner.frozen_command_seq << ',' << planner.frozen_producer_epoch << ','
        << planner.frozen_flight_id << ','
        << planner.frozen_revision_id << ',' << planner.frozen_strike_time << ','
        << planner.frozen_raw_tts << ','
        << planner.swing_sign << ',' << trace_reach_level << ','
        << trace_swing_foot_sign << ','
        << planner.planner_completion_seq << ','
        << planner.planner_completed_producer_epoch << ','
        << planner.planner_completed_flight_id << ','
        << planner.planner_completion_kind << ','
        << planner.base_pos_w[0] << ',' << planner.base_pos_w[1] << ',' << planner.base_pos_w[2] << ','
        << planner.base_quat_w[0] << ',' << planner.base_quat_w[1] << ','
        << planner.base_quat_w[2] << ',' << planner.base_quat_w[3] << ','
        << planner.target_pos_w[0] << ',' << planner.target_pos_w[1] << ','
        << planner.target_pos_w[2] << ',' << planner.target_vel_w[0] << ','
        << planner.target_vel_w[1] << ',' << planner.target_vel_w[2] << ','
        << (planner.racket_fk_valid ? 1 : 0) << ',' << planner.racket_pos_w[0] << ','
        << planner.racket_pos_w[1] << ',' << planner.racket_pos_w[2] << ','
        << planner.racket_vel_w[0] << ',' << planner.racket_vel_w[1] << ','
        << planner.racket_vel_w[2] << ',' << planner.racket_normal_w[0] << ','
        << planner.racket_normal_w[1] << ',' << planner.racket_normal_w[2] << ','
        << session_id << ',' << ppp->qdes_projector_active_count() << ','
        << ppp->qdes_projector_rate_count() << ','
        << ppp->qdes_projector_tracking_count() << ','
        << ppp->qdes_projector_torque_count() << ','
        << ppp->qdes_projector_infeasible_count() << ','
        << ppp->qdes_projector_max_normalized_error() << ','
        << ppp->qdes_feasible_action_utilization_max() << ','
        << ppp->qdes_feasible_interval_width_min() << ','
        << ppp->qdes_feasible_rate_utilization_max() << ','
        << ppp->qdes_feasible_rate_bound_count() << ','
        << ppp->qdes_feasible_tracking_bound_count() << ','
        << ppp->qdes_feasible_torque_bound_count() << ','
        << ppp->actual_q_hard_tolerance_rad() << ','
        << (ppp->actual_q_hard_audit_only() ? 1 : 0) << ','
        << ppp->actual_q_hard_violation_count() << ','
        << ppp->actual_q_hard_max_excess_rad();
      for (int i = 0; i < N; ++i)
        o << ',' << ppp->actual_q_hard_lo_for_backend(i);
      for (int i = 0; i < N; ++i)
        o << ',' << ppp->actual_q_hard_hi_for_backend(i);
      for (int i = 0; i < N; ++i)
        o << ',' << ppp->actual_q_hard_excess_for_backend(i);
      for (int i = 0; i < N; ++i) o << ',' << (has ? cmd.q_des[i] : 0.0);
      const auto& clamp_viol = ppp->last_clamp_viol();
      for (int i = 0; i < N; ++i)
        o << ',' << (clamp_viol.size() == N ? clamp_viol[i] : 0.0);
      for (int i = 0; i < N; ++i) o << ',' << (has ? st.q[i] : 0.0);
      for (int i = 0; i < N; ++i) o << ',' << (has ? st.dq[i] : 0.0);
      for (int i = 0; i < N; ++i) o << ',' << (has ? cmd.kp[i] : 0.0);
      for (int i = 0; i < N; ++i) o << ',' << (has ? cmd.kd[i] : 0.0);
      o << ',' << ppp->qdes_slew_scale() << ',' << ppp->qdes_slew_saturated_count() << ','
        << ppp->qdes_slew_saturated_leg_count() << ',' << ppp->qdes_slew_max_clip_rad()
        << ',' << planner.envelope_level << ',' << planner.envelope_mode << ','
        << planner.envelope_home_dist_m << ',' << planner.envelope_speed_mps << ','
        << planner.envelope_tilt_rad << ',' << planner.envelope_block_reason << ','
        << planner.envelope_support_anchor_error_m;
      o << '\n';
      if (has) {
        trace_previous_q_des = cmd.q_des;
        trace_previous_shot_seq = planner.shot_seq;
      }
      static int fc = 0; if (++fc % 25 == 0) o.flush();  // single RT writer
    }
    return publish;
  };

  // TELEOP owns every 500 Hz callback. Other modes retain their native
  // cadence: Serve and the optional legacy hybrid at 100 Hz, receive at 50 Hz.
  // Serve's clock runs through every mode; operator events never reset its phase.
  std::uint64_t logical_50hz_tick = 0;
  a3_pingpong::PpModeCadence mode_cadence;
  a3_pingpong::PpServeCadence serve_cadence;
  bool held_publish = false;
  robot_io::RobotCommand held_command;
  const a3_deploy::A3PolicyDriver* command_clock = nullptr;
  auto command_fn = [&, command_fn_50hz = std::move(command_fn_50hz)](std::uint64_t driver_tick,
                                                                      const robot_io::RobotState& state,
                                                                      robot_io::RobotCommand& command) mutable -> bool {
    const auto command_now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    const auto scheduled_ns = command_clock ? command_clock->CurrentWakeDeadlineNs() : 0;
    // Share the periodic driver's deadline. Comparing its actual wake against
    // a second deadline turns microsecond jitter into 8/12 ms CSV intervals.
    // Real overruns still follow monotonic elapsed time, never a tick divider.
    const auto cadence_ns = scheduled_ns > 0 ? scheduled_ns : command_now_ns;
    if (serve_cadence.tracking_hal()) {
      if (runner_control.mode() != Mode::kServe) {
        serve_cadence.CancelHalTracking(cadence_ns);
      } else {
        a3_io::ServeHalSnapshot receipt;
        const bool fresh = serve_hal_clock.Read(receipt) && receipt.Ready(command_now_ns);
        if (fresh && !serve_hal_pid) serve_hal_pid = receipt.pid;
        const bool valid = fresh && receipt.pid == serve_hal_pid && receipt.left == 2000;
        const auto result = serve_cadence.ObserveHalSelection(command_now_ns,
            valid ? receipt.release_edge_ns : 0);
        if (result == a3_pingpong::PpServeCadence::Receipt::kSelected) {
          serve_hal_observed_ns.store(command_now_ns, std::memory_order_relaxed);
          serve_hal_selected_ns.store(receipt.release_edge_ns, std::memory_order_release);
        }
      }
    }
    const bool serve_due = serve_cadence.Poll(cadence_ns);
    if (serve_cadence.ConsumeMissedSelection())
      serve_hal_selection_missed.store(true, std::memory_order_release);
    if (locomotion) {
      const Mode mode = runner_control.mode();
      if (mode == Mode::kTeleop) {
        computed_mode = mode;
        prev_mode_for_blend = mode;
        mode_cadence.Reset();
        if (!teleop_entered) {
          robot_io::RobotCommand source = last_delivered_command;
          if (!last_delivered_valid) {
            source.q_des = state.q;
            source.dq_des = state.dq;
            source.kp = source.kd = source.tau_ff = Eigen::VectorXd::Zero(31);
          }
          robot_io::RobotCommand teleop_stand;
          teleop_stand.q_des = ppp->official_stand_q();
          teleop_stand.dq_des = teleop_stand.tau_ff = Eigen::VectorXd::Zero(31);
          teleop_stand.kp = ppp->official_stand_kp();
          teleop_stand.kd = ppp->official_stand_kd();
          teleop_entry.Begin(source, teleop_stand);
          teleop_input.Disarm();
          teleop_tick = 0;
          teleop_quiet_ticks = 0;
          teleop_fall_ticks = 0;
          command = teleop_entry.Step(0., state, *locomotion, Eigen::Vector3d::Zero());
          teleop_entered = true;
        } else {
          const bool stopping = runner_control.TeleopStopRequested();
          const bool active = !teleop_entry.active() && !stopping;
          const auto target = teleop_input.Sample(active, wall_seconds(), steady_seconds());
          command = teleop_entry.Step(teleop_tick * .002, state, *locomotion, target);
          if (stopping && !teleop_entry.active() && locomotion->settled() && state.dq.cwiseAbs().maxCoeff() < .5 &&
              state.imu_gyro.norm() < .15)
            ++teleop_quiet_ticks;
          else teleop_quiet_ticks = 0;
          if (teleop_quiet_ticks >= 150) runner_control.SetRuntimeMode(Mode::kPdStand);
        }
        ppp->observe_imu(state);
        const double gz = -(1.0 - 2.0 * (state.imu_quat_wxyz[1] * state.imu_quat_wxyz[1] +
                                         state.imu_quat_wxyz[2] * state.imu_quat_wxyz[2]));
        teleop_fall_ticks = gz > fall_guard_gz ? teleop_fall_ticks + 1 : 0;
        if (fall_guard && teleop_fall_ticks >= 30) throw std::runtime_error("teleop fall guard");
        teleop_phase.store(runner_control.TeleopStopRequested() ? 3 : (teleop_entry.active() ? 1 : 2));
        teleop_progress.store(std::min(1., teleop_tick * .002 / a3_pingpong::PpTeleopEntry::kSeconds));
        const auto filtered = locomotion->filtered_velocity();
        for (int i = 0; i < 3; ++i) teleop_velocity[i].store(filtered[i]);
        ++teleop_tick;
        return true;
      }
      if (teleop_entered) {
        teleop_entered = false;
        teleop_input.Disarm();
      }
      teleop_phase.store(0);
      teleop_progress.store(0);
      for (auto& value : teleop_velocity) value.store(0);
      const std::int64_t period_ns = (mode == Mode::kServe || hybridp != nullptr)
          ? 10'000'000 : 20'000'000;
      if (mode == Mode::kServe ? serve_due :
          mode_cadence.DueTime(cadence_ns, static_cast<int>(mode), period_ns)) {
        robot_io::RobotCommand next;
        held_publish = command_fn_50hz(logical_50hz_tick++, state, next);
        held_command = next;
        if (mode == Mode::kServe && servep) {
          const auto publication_ns = servep->ConsumeReleaseClockRebase();
          if (publication_ns) {
            if (serve_hal_clock_enabled) serve_cadence.TrackHalSelection(publication_ns);
            else serve_cadence.NotePublication(publication_ns);
          }
        }
      }
      command = held_command;
      return held_publish;
    }
    if (!serve_requested || hybridp != nullptr) { return command_fn_50hz(driver_tick, state, command); }
    const Mode mode = runner_control.mode();
    if (mode == Mode::kServe) {
      mode_cadence.Reset();
      if (serve_due) {
        held_publish = command_fn_50hz(driver_tick, state, held_command);
        if (servep) {
          const auto publication_ns = servep->ConsumeReleaseClockRebase();
          if (publication_ns) {
            if (serve_hal_clock_enabled) serve_cadence.TrackHalSelection(publication_ns);
            else serve_cadence.NotePublication(publication_ns);
          }
        }
      }
      command = held_command;
      return held_publish;
    }
    const bool evaluate = mode_cadence.DueTime(cadence_ns, static_cast<int>(mode), 20'000'000);
    if (evaluate) {
      robot_io::RobotCommand next;
      const bool publish = command_fn_50hz(logical_50hz_tick++, state, next);
      held_command = next;
      held_publish = publish;
    }
    command = held_command;
    return held_publish;
  };

  if (!backend->Start()) {
    std::cerr << "backend Start failed\n";
    return 5;
  }
  std::cout << "[pingpong] backend started\n";

  a3_deploy::A3PolicyDriverOptions dopt;
  dopt.policy_hz = driver_hz;
  dopt.sched.cpu = driver_cpu;
  dopt.command_delivery_observer = [ppp, &last_delivered_command,
      &last_delivered_valid, &last_delivered_mode, &computed_mode, &mode_transition, &command_state](
      const robot_io::RobotCommand& cmd, bool sent) {
    ppp->RecordCommandDelivery(cmd, sent,
        computed_mode == Mode::kMotion && mode_transition.active(), &command_state);
    // Invoked on the same driver thread after SendCommand; failed deliveries
    // must never become the source of the next mode handoff.
    if (sent) {
      last_delivered_command = cmd;
      last_delivered_valid = true;
      last_delivered_mode = computed_mode;
    }
  };
  a3_deploy::CommandFn cfn = command_fn;  // disambiguate the PolicyFn/CommandFn ctor
  a3_deploy::A3PolicyDriver driver(*backend, cfn, dopt);
  command_clock = &driver;
  if (!driver.StartDriver()) { std::cerr << "StartDriver failed\n"; backend->Stop(); return 6; }
  std::cout << "[pingpong] driver started @ " << dopt.policy_hz
            << " Hz (SERVE direct=100 Hz; ONNX logical rate=" << policy_hz
            << " Hz)\n";

  std::signal(SIGINT, OnSig);
  std::signal(SIGTERM, OnSig);

  // Both keyboard s/m/p/h and the fixed remote services enter this bounded
  // queue.  The AimRT callback never writes Runner mode or role directly.
  std::thread runner_action_worker([&]() {
    while (!g_stop.load()) {
      if (servep != nullptr) servep->PollAsync();
      const bool serve_active = servep != nullptr && servep->active();
      // Capability means this Runner loaded the named-schema serve stack.
      // Runtime health is reported by serve_state/gripper_state; do not make
      // the wire schema self-contradictory when either one fault-latches.
      const bool serve_capability = servep != nullptr;
      const int serve_state =
          servep == nullptr ? -1 : static_cast<int>(servep->state());
      const int gripper_state =
          servep == nullptr
              ? -1
              : static_cast<int>(servep->gripper_state());
      const bool cleanup_required =
          servep != nullptr && servep->cleanup_required();
      const auto decisions = runner_control.ProcessPending(
          driver.CommandFaultLatched(), serve_active, serve_capability,
          serve_state, true, gripper_state,
          cleanup_required, locomotion != nullptr,
          teleop_input.Ready(wall_seconds(),steady_seconds()));
      for (const auto& decision : decisions) {
        if (decision.hold_reference) {
          refp->Hold(decision.request.action ==
                             a3_pingpong::RunnerAction::kEmergencyPassive
                         ? "operator_passive"
                         : "operator_mode_hold");
        }
        if (decision.request_serve_abort && servep != nullptr) {
          servep->RequestAbort();
        }
        // PREPARE_SERVE publishes the mode; the command thread initializes the
        // controller and captures its last delivered entry command together.
        if (decision.request_confirm_ball_loaded &&
            servep != nullptr) {
          servep->ConfirmBallLoaded();
        }
        if (decision.request_confirm_grip_secure &&
            servep != nullptr) {
          servep->ConfirmGripSecure();
        }
        if (decision.request_ready_to_serve && servep != nullptr) {
          servep->TriggerReadyToServe();
        }
        if (decision.request_open_gripper && servep != nullptr) {
          std::string open_error;
          if (!servep->RequestOpenGripper(open_error)) {
            std::cerr
                << "[serve] cleanup OPEN admission raced/faulted: "
                << open_error << "\n";
          }
        }
        if (decision.request.action ==
                a3_pingpong::RunnerAction::kEmergencyPassive &&
            serve_active && servep != nullptr) {
          servep->EmergencyStop();
        }
        std::cout << "-> [runner-control] source="
                  << (decision.request.remote ? "FOXGLOVE" : "KEYBOARD")
                  << " request=" << decision.request.request_id
                  << " action="
                  << a3_pingpong::RunnerActionName(decision.request.action)
                  << " result="
                  << a3_pingpong::RunnerActionResultName(decision.result)
                  << " reason="
                  << a3_pingpong::RunnerActionReasonName(decision.reason)
                  << " mode=" << ModeName(runner_control.mode())
                  << " role="
                  << a3_pingpong::LocalRoleName(runner_control.local_role())
                  << "\n";
        if (decision.request.action ==
                a3_pingpong::RunnerAction::kEmergencyPassive &&
            serve_active) {
          std::cout << "-> EMERGENCY PASSIVE during SERVE; restart the "
                       "runner before another serve\n";
        }
        const Mode current = runner_control.mode();
        const bool capability_now = servep != nullptr;
        runner_control.ObserveExternalState(
            !no_publish && current != Mode::kShadow && driver.HasSentCommand(),
            policy_native,
            driver.CommandFaultLatched(), capability_now,
            servep == nullptr ? -1 : static_cast<int>(servep->state()),
            servep == nullptr
                ? -1
                : static_cast<int>(servep->gripper_state()),
            servep != nullptr && servep->cleanup_required());
        // Publish the acknowledgement immediately; the 5 Hz heartbeat below
        // remains the stale/liveness source.  This prevents a following
        // keyboard action from hiding a remote request's result.
        backend->PublishRunnerState(runner_control.EncodeState());
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });

  // ---- consolidated bring-up CONFIG banner (one place to eyeball every knob) ----
  char leg_gain_banner[16];
  {
    const double lgs = leg_gain_scale.load();
    if (lgs >= 0.0) std::snprintf(leg_gain_banner, sizeof leg_gain_banner, "%.2f", lgs);
    else std::snprintf(leg_gain_banner, sizeof leg_gain_banner, "=gain");
  }
  char ankle_gain_banner[16];
  {
    const double ags = ankle_gain_scale.load();
    if (ags >= 0.0) std::snprintf(ankle_gain_banner, sizeof ankle_gain_banner, "%.2f", ags);
    else std::snprintf(ankle_gain_banner, sizeof ankle_gain_banner, "=leg");
  }
  const std::string action_source_banner =
      hybridp == nullptr
          ? "ONNX receive policy (learned 31-DOF action every tick)"
          : "ONNX x2: receive policy upper 50Hz + AGI lower 100Hz (one publisher)";
  const std::string post_onnx_banner =
      hybridp == nullptr
          ? "neck[3,4] HELD | receive policy owns remaining body command"
          : std::string("neck[3,4] HELD | AGI owns ") +
                a3_pingpong::PpHybridLowerOwnerName(hybridp->config().owner) +
                " q/dq/tau/kp/kd | receive policy owns the rest";
  std::printf(
      "[pingpong] ================= RUN CONFIG =================\n"
      "[pingpong]  start_mode   = %-9s  (s=PD_STAND hold/NO swing, m=MOTION publish)\n"
      "[pingpong]  level        = %-9d  (0=hold/windup, 1=SWING)\n"
      "[pingpong]  swing_dir    = %-9s  (f=forehand / b=backhand keys)\n"
      "[pingpong]  target_src   = %s\n"
      "[pingpong]  execution    = %-9s  (policy=50Hz held; SERVE=original 100Hz CSV direct; one Runner/publisher)\n"
      "[pingpong]  action_src   = %s\n"
      "[pingpong]  post_onnx    = %s\n"
      "[pingpong]  loc_mode     = %s\n"
      "[pingpong]  legs_passive = %-9s  (true=legs HELD; validates UPPER-BODY/waist swing only)\n"
      "[pingpong]  leg_hold     = %-9s  (official=AGI ground-stand gains [GROUND, proven] | trained=ONNX leg PD [HOIST])\n"
      "[pingpong]  waist_hold   = %-9s  (official/trained=waist FROZEN at nominal [ARMS-ONLY swing] | swing=policy-driven)\n"
      "[pingpong]  auto_hold    = %-9s  (--auto-leg-hold: level0 HOLDS legs+waist [ready stand], level1 RELEASES [full-body swing])\n"
      "[pingpong]  arm_hold     = %-9s  (--arm-hold-nominal: level0 ramps ARMS to nominal [17400 hold-twist cosmetics], level1 policy arms)\n"
      "[pingpong]  gain_scale   = %-9.2f  (arms/waist swing)   swing_speed = %.2f\n"
      "[pingpong]  leg_gain     = %-9s  ankle_gain = %-7s  (ankle=balance joint; raise if tipping fwd)  motion_blend = %.2fs\n"
      "[pingpong]  leg_stand_g  = %-9s  (--leg-stand-gains: RELEASED legs use official ground-stand PD [weight-bearing, kp~2000 knee] vs policy PD x leg_gain)\n"
      "[pingpong]  safety       = fall_guard=%-3s squat_guard=%.2frad tilt_guard=%.2f leg_clamp=%.2frad leg_smooth=%.2f\n"
      "[pingpong]  publish      = %-9s  (--dry-run/--no-publish => NEVER publishes; SHADOW also no-publish)\n"
      "[pingpong]  model        = %s\n"
      "[pingpong]  trace_csv    = %s\n"
      "[pingpong]  obs_csv      = %s\n"
      "[pingpong] =============================================\n",
      ModeName(default_mode), level, pp->swing_dir_name(),
      planner_mode
          ? "PLANNER  (live: racket <- /racket/command_flat, base <- /a3/base_pose_flat over ros2; engage machine drives swing)"
          : "SCRIPTED (fixed front-right TEST target; NO live planner -- f/b only flips y-sign+clip)",
      policy_native ? "native" : "legacy",
      action_source_banner.c_str(), post_onnx_banner.c_str(),
      pp->loc_mode_name(),
      pcfg.legs_passive ? "true" : "false",
      pcfg.legs_passive ? (legs_official_gains ? "official" : "trained") : "n/a (policy)",
      pcfg.waist_passive ? (waist_official_gains ? "official" : "trained") : "swing",
      pcfg.auto_leg_hold ? "ON" : "off",
      pcfg.arm_hold_nominal ? "ON" : "off",
      gain_scale.load(), swing_speed,
      leg_gain_banner, ankle_gain_banner, policy_motion_blend_sec,
      leg_stand_gains ? "ON" : "off",
      fall_guard ? "ON" : "off", squat_guard_rad, tilt_guard,
      pcfg.leg_clamp_rad, pcfg.leg_smooth_alpha,
      no_publish ? "DISABLED" : "enabled",
      model_path.c_str(),
      trace_path.empty() ? "<none>" : trace_path.c_str(),
      obs_csv_path.empty() ? "<none>" : obs_csv_path.c_str());

  // --- keyboard control (raw, non-blocking) ---
  std::cout << "[keys] p=PASSIVE(limp)  s=PD_STAND(hold, NO swing)  h=SHADOW(compute, no publish)"
               "  m=MOTION(publish)\n";
  if (servep != nullptr) {
    std::cout
        << "[serve keys] S=SERVER R=RECEIVER "
           "v=PREPARE(strong stand+immediate GRAB+raise concurrently) "
           "l/g=legacy no-op "
           "Space=READY TO SERVE/play "
           "o=OPEN(manual) x=phase-aware abort "
           "p=EMERGENCY limp\n";
  }
  std::cout << "[keys] 0=level0(hold/windup)  1=level1(SWING)  f=forehand  b=backhand"
               "  [=gain-  ]=gain+  ,=swing slower  .=swing faster  q=quit\n";
  std::cout << "[ref keys] (only in --reference-playback) 0=head hold 1=waist 2=R shoulder"
               " 3=R elbow/wrist 4=R arm 5=waist+R arm 6=legs hold 7=upper body"
               "  r=start ref  x=hold ref  c=clear ref fault\n";
  termios old_tio{};
  bool tty = isatty(STDIN_FILENO);
  if (tty) {
    tcgetattr(STDIN_FILENO, &old_tio);
    termios raw = old_tio;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0; raw.c_cc[VTIME] = 1;  // 100ms poll
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
  }
  std::thread kb([&]() {
    while (!g_stop.load()) {
      char c = 0;
      if (tty && read(STDIN_FILENO, &c, 1) == 1) {
        switch (c) {
          case 'p':
            runner_control.EnqueueLocalAction(
                a3_pingpong::RunnerAction::kEmergencyPassive);
            break;
          case 's':
            runner_control.EnqueueLocalAction(
                a3_pingpong::RunnerAction::kEnterPdStand);
            break;
          case 'h':
            runner_control.EnqueueLocalAction(
                a3_pingpong::RunnerAction::kEnterShadow);
            break;
          case 'm':
            runner_control.EnqueueLocalAction(
                a3_pingpong::RunnerAction::kEnterMotion);
            break;
          case 'S':
            runner_control.EnqueueLocalAction(
                a3_pingpong::RunnerAction::kSetServer);
            break;
          case 'R':
            runner_control.EnqueueLocalAction(
                a3_pingpong::RunnerAction::kSetReceiver);
            break;
          case 'v':
            if (servep == nullptr) {
              std::cout << "-> SERVE unavailable; launch with --serve\n";
            } else {
              runner_control.EnqueueLocalAction(
                  a3_pingpong::RunnerAction::kPrepareServe);
            }
            break;
          case 'l':
            runner_control.EnqueueLocalAction(
                a3_pingpong::RunnerAction::kConfirmBallLoaded);
            break;
          case 'g':
            runner_control.EnqueueLocalAction(
                a3_pingpong::RunnerAction::kConfirmGripSecure);
            break;
          case ' ':
            runner_control.EnqueueLocalAction(
                a3_pingpong::RunnerAction::kReadyToServe);
            break;
          case 'o':
            runner_control.EnqueueLocalAction(
                a3_pingpong::RunnerAction::kOpenGripper);
            break;
          case '0':
          case '1':
          case '2':
          case '3':
          case '4':
          case '5':
          case '6':
          case '7':
            if (runner_control.mode() == Mode::kTeleop) {
              std::cout << "-> return to PD_STAND before reference/policy diagnostics\n";
              break;
            }
            if (reference_playback_selected ||
                runner_control.mode() == Mode::kReferencePlayback) {
              const int gi = c - '0';
              refp->SetGroup(a3_pingpong::RefPlaybackGroupFromInt(gi));
              refp->Hold("group_selected_hold");
              runner_control.SetRuntimeMode(Mode::kReferencePlayback);
              std::cout << "-> ref group " << gi << " ("
                        << a3_pingpong::RefPlaybackGroupName(refp->group())
                        << "), HOLD; press r to move\n";
            } else if (planner_mode) {
              std::cout << "-> level key ignored (PLANNER drives swing engage)\n";
            } else if (c == '0') {
              ppp->set_level(0); std::cout << "-> level 0 (hold)\n";
            } else if (c == '1') {
              ppp->set_level(1); std::cout << "-> level 1 (forehand)\n";
            }
            break;
          case '[':
          case ']':
            if (runner_control.mode() == Mode::kServe && servep != nullptr &&
                servep->active()) {
              std::cout << "gain change rejected while SERVE owns PD gains\n";
            } else {
              const double delta = c == '[' ? -0.1 : 0.1;
              gain_scale.store(std::min(
                  1.0, std::max(0.0, gain_scale.load() + delta)));
              std::cout << "gain_scale=" << gain_scale.load() << "\n";
            }
            break;
          case ',':
          case '.': // swing-speed rescales the in-flight clock RETROACTIVELY (t is scaled from
                    // the engage origin): mid-swing it snaps tts and breaks the wait-until-tts
                    // strike alignment. Planner mode owns the clock — ignore, like 0/1/f/b.
                    if (planner_mode) { std::cout << "-> swing-speed key ignored (PLANNER owns the clock)\n"; break; }
                    ppp->set_swing_speed(ppp->swing_speed() + (c == '.' ? 0.1 : -0.1));
                    std::cout << "swing_speed=" << ppp->swing_speed() << "\n"; break;
          case 'f': if (planner_mode) { std::cout << "-> f/b ignored (PLANNER picks the side)\n"; break; }
                    ppp->set_swing_dir(+1);
                    std::cout << "-> swing dir = FOREHAND (scripted target -y, clip0)\n"; break;
          case 'b': if (planner_mode) { std::cout << "-> f/b ignored (PLANNER picks the side)\n"; break; }
                    ppp->set_swing_dir(-1);
                    std::cout << "-> swing dir = BACKHAND (scripted target +y, clip1)\n"; break;
          case 'r':
            if (runner_control.mode() == Mode::kTeleop) {
              std::cout << "-> return to PD_STAND before REFERENCE_PLAYBACK\n";
              break;
            }
            if (runner_control.mode() == Mode::kServe && servep != nullptr &&
                servep->active()) {
              std::cout << "-> REFERENCE_PLAYBACK rejected while SERVE owns q_des\n";
            } else {
              refp->Start();
              runner_control.SetRuntimeMode(Mode::kReferencePlayback);
              std::cout << "-> REFERENCE_PLAYBACK moving group="
                        << a3_pingpong::RefPlaybackGroupName(refp->group())
                        << "\n";
            }
            break;
          case 'x':
            if (runner_control.mode() == Mode::kTeleop) {
              runner_control.EnqueueLocalAction(a3_pingpong::RunnerAction::kEnterPdStand);
              break;
            }
            if (runner_control.mode() == Mode::kServe && servep != nullptr &&
                servep->active()) {
              runner_control.EnqueueLocalAction(
                  a3_pingpong::RunnerAction::kEnterPdStand);
            } else {
              refp->Hold("operator_hold");
              runner_control.SetRuntimeMode(Mode::kReferencePlayback);
              std::cout << "-> REFERENCE_PLAYBACK hold current pose\n";
            }
            break;
          case 'c': refp->ClearFault();
                    std::cout << "-> ref fault cleared; press r to move\n"; break;
          case 'q': g_stop.store(true); break;
          default: break;
        }
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }
  });

  // Authoritative Runner heartbeat.  It carries actual local role/mode and
  // the real PpServeController enum; the HDU observer only decodes it.
  std::thread runner_state_publisher([&]() {
    while (!g_stop.load()) {
      const Mode current = runner_control.mode();
      const bool command_publishing =
          !no_publish && current != Mode::kShadow && driver.HasSentCommand();
      const int serve_state =
          servep == nullptr ? -1 : static_cast<int>(servep->state());
      runner_control.ObserveExternalState(
          command_publishing, policy_native,
          driver.CommandFaultLatched(),
          servep != nullptr, serve_state,
          servep == nullptr
              ? -1
              : static_cast<int>(servep->gripper_state()),
          servep != nullptr && servep->cleanup_required());
      backend->PublishRunnerState(runner_control.EncodeState());
      if (locomotion) backend->PublishLocomotionState({
          1.,static_cast<double>(runner_boot_id),1.,static_cast<double>(teleop_phase.load()),
          teleop_input.Age(steady_seconds()),teleop_progress.load(),
          teleop_velocity[0].load(),teleop_velocity[1].load(),teleop_velocity[2].load(),
          teleop_input.armed()?1.:0.});
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
  });

  // --- status loop ---
  std::uint64_t last_ticks = 0;
  std::uint64_t last_native_evaluations = 0;
  std::uint64_t last_reported_halts = 0;
  std::uint64_t last_reported_serve = 0;
  auto t_start = std::chrono::steady_clock::now();
  auto t_prev = t_start;
  bool clamp_rate_warned = false;  // one-shot high-clamp-rate warning (waist_roll audit)
  bool warming = warmup_sec > 0;
  if (warming)
    std::printf("[pingpong] warmup: PD_STAND for %.1fs, then -> %s\n", warmup_sec,
                ModeName(target_mode));
  while (!g_stop.load()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    // Filesystem attachment stays off the real-time command thread.
    if (serve_hal_clock_enabled && !serve_hal_clock.IsOpen())
      (void)serve_hal_clock.Open(serve_hal_clock_path.c_str());
    const std::uint64_t ticks = driver.PolicyTickCount();
    const std::uint64_t halts = driver.SafeHaltCount();
    auto now = std::chrono::steady_clock::now();
    if (warming && runner_control.mode() == Mode::kTeleop) warming = false;
    if (warming &&
        std::chrono::duration<double>(now - t_start).count() >= warmup_sec) {
      runner_control.SetRuntimeMode(target_mode);
      warming = false;
      std::printf("[pingpong] warmup done -> %s\n", ModeName(target_mode));
    }
    double dt = std::chrono::duration<double>(now - t_prev).count();
    double hz = dt > 0 ? (ticks - last_ticks) / dt : 0;
    const auto evaluations = native_evaluations.load(std::memory_order_relaxed);
    const double native_hz = dt > 0 ? (evaluations - last_native_evaluations) / dt : 0;
    last_native_evaluations = evaluations;
    const auto g = ppp->last_proj_grav();
    const Mode cur_mode = runner_control.mode();
    if (const auto selected = serve_hal_selected_ns.exchange(0, std::memory_order_acq_rel))
      std::printf("[serve_hal_clock] selected_ns=%llu observed_ns=%llu telemetry_only=1 csv_clock=fixed_100hz\n",
          static_cast<unsigned long long>(selected),
          static_cast<unsigned long long>(serve_hal_observed_ns.load(std::memory_order_relaxed)));
    if (serve_hal_selection_missed.exchange(false, std::memory_order_acq_rel))
      std::printf("[serve_hal_clock] selection unavailable before next CSV row; replay continued on fixed CSV clock, physical release unverified\n");
    if (servep != nullptr) {
      const auto d = servep->TakeDiag();
      const auto& a = d.playback;
      if (a.finished && a.sequence != last_reported_serve) {
        last_reported_serve = a.sequence;
        std::printf("[serve_playback] cycle=%llu frames=%zu scope=command_evaluation "
            "first_ns=%llu last_ns=%llu interval_min_ms=%.6f interval_max_ms=%.6f "
            "max_phase_error_ms=%.6f release_frame_ns=%llu contact_reference_frame_ns=%llu "
            "start_support_rad=%.9f start_support_velocity=%.9f "
            "csv_q_delta=%.9f csv_dq_delta=%.9f arm_csv_q_delta=%.9f arm_csv_dq_delta=%.9f "
            "tracking_max_rad=%.9f arm_tracking_max_rad=%.9f "
            "release_dispatch_ns=%llu release_first_publish_ns=%llu\n",
            static_cast<unsigned long long>(a.sequence), a.frames,
            static_cast<unsigned long long>(a.first_ns), static_cast<unsigned long long>(a.last_ns),
            a.min_interval_ns / 1.e6, a.max_interval_ns / 1.e6, a.max_phase_error_ns / 1.e6,
            static_cast<unsigned long long>(a.release_frame_ns),
            static_cast<unsigned long long>(a.contact_reference_frame_ns),
            a.start_support_rad, a.start_support_velocity,
            a.max_csv_q_delta_rad, a.max_csv_dq_delta_rad_s,
            a.max_arm_csv_q_delta_rad, a.max_arm_csv_dq_delta_rad_s,
            a.max_tracking_error_rad, a.max_arm_tracking_error_rad,
            static_cast<unsigned long long>(d.release_dispatch_monotonic_ns),
            static_cast<unsigned long long>(d.release_first_publish_monotonic_ns));
        for (const auto& sample : a.samples) {
          if (sample.evaluation_ns == 0) continue;
          std::printf("[serve_sample] cycle=%llu frame=%zu evaluation_monotonic_ns=%llu state_timestamp_ns=%lld",
              static_cast<unsigned long long>(a.sequence), sample.frame,
              static_cast<unsigned long long>(sample.evaluation_ns),
              static_cast<long long>(sample.state_timestamp_ns));
          const auto print_values = [](const char* name, const auto& values) {
            std::printf(" %s=", name);
            for (std::size_t i = 0; i < values.size(); ++i)
              std::printf("%s%.9f", i ? "," : "", values[i]);
          };
          print_values("q", sample.q);
          print_values("dq", sample.dq);
          print_values("q_des", sample.q_des);
          print_values("dq_des", sample.dq_des);
          print_values("imu_quat", sample.imu_quat);
          print_values("gyro", sample.gyro);
          std::printf("\n");
        }
      }
    }
    if (cur_mode == Mode::kServe || cur_mode == Mode::kPdStand)
      std::printf("[cadence] mode=%s callback_hz=%.1f native_evaluation_hz=%.1f release_command_frame=%zu\n",
                  ModeName(cur_mode), hz, native_hz, servep ? servep->release_frame() : 0);
    if (cur_mode == Mode::kServe) {
      // Actor-side sync_miss is not a measurement of the Serve input pipeline.
      // Read the actual backend counters on the non-control status thread.
      const auto sync = backend->SyncStatistics();
      std::printf("[serve_sync] frames=%llu complete=%llu aligned=%llu "
          "latest_group_skew_ms=%.6f latest_driver_state_age_ms=%.6f "
          "latest_channel_age_ms=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f "
          "driver_stale_polls=%llu driver_unaligned_polls=%llu driver_incomplete_polls=%llu\n",
          static_cast<unsigned long long>(sync.tick_total),
          static_cast<unsigned long long>(sync.frame_complete_total),
          static_cast<unsigned long long>(sync.frame_aligned_total),
          sync.last_skew_ns / 1.e6, driver.LastFrameAgeNs() / 1.e6,
          sync.last_age_waist_ns / 1.e6, sync.last_age_leg_ns / 1.e6,
          sync.last_age_arm_ns / 1.e6, sync.last_age_neck_ns / 1.e6,
          sync.last_age_pelvis_imu_ns / 1.e6, sync.last_age_torso_imu_ns / 1.e6,
          static_cast<unsigned long long>(driver.Watchdog().StaleCount()),
          static_cast<unsigned long long>(driver.Watchdog().UnalignedCount()),
          static_cast<unsigned long long>(driver.IncompleteFrameCount()));
    }
    if (halts != last_reported_halts) {
      const auto timing = driver.GetStats();
      std::printf("[driver_timing] max_wake_lateness_ms=%.3f max_run_ms=%.3f work_overruns=%llu\n",
                  timing.max_wake_lateness_ns / 1.e6, timing.max_run_ns / 1.e6,
                  static_cast<unsigned long long>(timing.overrun_count));
      std::printf("[driver_health] mode=%s halt_ticks_delta=%llu "
                  "stale_polls=%llu unaligned_polls=%llu incomplete_polls=%llu "
                  "unaligned_source_streak=%d latest_age_ms=%.3f latest_skew_ms=%.3f\n",
                  ModeName(cur_mode), static_cast<unsigned long long>(halts - last_reported_halts),
                  static_cast<unsigned long long>(driver.Watchdog().StaleCount()),
                  static_cast<unsigned long long>(driver.Watchdog().UnalignedCount()),
                  static_cast<unsigned long long>(driver.IncompleteFrameCount()),
                  driver.Watchdog().CurrentUnalignedStreak(),
                  driver.LastFrameAgeNs() / 1.e6, driver.LastFrameSkewNs() / 1.e6);
      last_reported_halts = halts;
    }
    if (cur_mode == Mode::kServe && servep != nullptr) {
      const auto d = servep->TakeDiag();
      std::printf(
          "[status] mode=SERVE state=%s phase=%s frame=%zu/%zu "
          "transition=%zu/%zu gripper=%s gripper_fault=%s "
          "cleanup_required=%d "
          "ready_ticks=%d local_ready=%d q_or_tracking_err=%.3f "
          "tracking_warning=%d "
          "joint_speed=%.3f tilt=%.3f yaw_rate=%.3f "
          "gravx=%.3f stance_pitch_offset=(hip=%.3f,knee=%.3f,ankle=%.3f,waist=%.3f) "
          "support_err=%.3f waist_err=%.3f "
          "release_ack=%d release_dispatch_ns=%llu "
          "release_first_publish_ns=%llu abort_after_strike=%d fault=%s "
          "rate=%.1fHz ticks=%llu halts=%llu\n",
          a3_pingpong::ServeControllerStateName(d.state),
          d.phase.c_str(), d.frame, servep->frame_count(),
          d.transition_tick,
          a3_pingpong::kServe025FullbodyTransitionTicks,
          a3_pingpong::ServeGripperStateName(d.gripper_state),
          d.gripper_fault_reason.empty()
              ? "-"
              : d.gripper_fault_reason.c_str(),
          d.cleanup_required ? 1 : 0,
          d.ready_ticks, d.local_ready ? 1 : 0,
          d.max_q_error_rad, d.tracking_warning ? 1 : 0,
          d.max_joint_speed_rad_s,
          d.tilt_rad, d.yaw_rate_rad_s,
          d.projected_gravity_x, d.hip_pitch_offset_rad,
          d.knee_pitch_offset_rad, d.ankle_pitch_offset_rad,
          d.waist_pitch_offset_rad, d.sagittal_support_error_rad,
          d.max_waist_error_rad,
          d.release_acknowledged ? 1 : 0,
          static_cast<unsigned long long>(
              d.release_dispatch_monotonic_ns),
          static_cast<unsigned long long>(
              d.release_first_publish_monotonic_ns),
          d.abort_after_strike ? 1 : 0,
          d.fault_reason.empty() ? "-" : d.fault_reason.c_str(), hz,
          static_cast<unsigned long long>(ticks),
          static_cast<unsigned long long>(halts));
    } else if (cur_mode == Mode::kReferencePlayback) {
      std::printf("[status] mode=%s ref_group=%s ref_moving=%d ref_fault=%d rate=%.1fHz "
                  "ticks=%llu halts=%llu command_fault_latched=%d ref_amp=%.3f "
                  "ref_freq=%.3f ref_gain=%.2f\n",
                  ModeName(cur_mode), a3_pingpong::RefPlaybackGroupName(refp->group()),
                  refp->moving() ? 1 : 0, refp->faulted() ? 1 : 0, hz,
                  (unsigned long long)ticks, (unsigned long long)halts,
                  driver.CommandFaultLatched() ? 1 : 0,
                  refp->config().amplitude_rad, refp->config().frequency_hz,
                  refp->config().gain_scale);
      PrintRefDiagBlock(refp->TakeDiag());
    } else {
      // Consume the rolling diag window + obs-debug ONCE this tick (take_diag
      // resets the window) and reuse for the status/[fullbody] lines + blocks.
      const auto diag = ppp->take_diag();
      const auto obsd = ppp->take_obs_debug();
      const Eigen::VectorXd& act = ppp->last_action();
      const double act_max = act.size() ? act.cwiseAbs().maxCoeff() : 0.0;
      // Peak single-joint commanded/measured range within a backend-slot group
      // (waist 0..2, Lleg 19..24, Rleg 25..30) for the full-body verification.
      auto grp_amp = [&diag](int lo, int hi, double& cmdR, double& measR) {
        cmdR = measR = 0.0;
        if (!diag.valid) return;
        for (int i = lo; i <= hi; ++i) {
          cmdR = std::max(cmdR, diag.des_range[i]);
          measR = std::max(measR, diag.meas_range[i]);
        }
      };
      double waist_c, waist_m, lleg_c, lleg_m, rleg_c, rleg_m;
      grp_amp(0, 2, waist_c, waist_m);
      grp_amp(19, 24, lleg_c, lleg_m);
      grp_amp(25, 30, rleg_c, rleg_m);
      // sdir=swing direction, maxact=max|action| (near 0 => ONNX not driving),
      // clamp=#joints clamped THIS tick, legs_passive=leg cmds held nominal (NOT a
      // full-body test), sync_miss=cumulative dropped/unaligned packets (must be 0).
      // ts advancing + |act| oscillating => the swing clock is live.
      std::printf("[status] mode=%s level=%d sdir=%s gain=%.2f sspeed=%.2f rate=%.1fHz ticks=%llu "
                  "halts=%llu command_fault_latched=%d sync_miss=%llu ts=%d |act|=%.2f "
                  "maxact=%.2f clamp=%d safe=%d qdes_audit_only=%d legs_passive=%d "
                  "gravZ=%.2f baseZ=%.3f "
                  "grav=[%.2f %.2f %.2f]\n",
                  ModeName(cur_mode), ppp->level(), ppp->swing_dir_name(),
                  gain_scale.load(), ppp->swing_speed(), hz, (unsigned long long)ticks,
                  (unsigned long long)halts, driver.CommandFaultLatched() ? 1 : 0,
                  (unsigned long long)obsd.sync_miss,
                  ppp->last_time_step(), act.norm(), act_max, ppp->last_clamp_count(),
                  ppp->last_safe_interval_violation_count(),
                  ppp->gate3_qdes_audit_only() ? 1 : 0,
                  ppp->legs_passive() ? 1 : 0, g[2], ppp->last_base_pos()[2], g[0], g[1], g[2]);
      // Full-body command-vs-measured peak amplitude per group. legs_passive=1 =>
      // Lleg/Rleg cmdR ~ 0 (held nominal) => this is NOT a full-body test. With
      // legs_passive=0, leg cmdR>0 proves the policy DRIVES the legs; small
      // measR/cmdR while HOISTED is EXPECTED (feet bear no load) -> WARN not FAIL.
      std::printf("[fullbody] legs_passive=%d | waist cmdR=%.3f measR=%.3f | "
                  "Lleg cmdR=%.3f measR=%.3f | Rleg cmdR=%.3f measR=%.3f  (rad, peak this window)\n",
                  ppp->legs_passive() ? 1 : 0, waist_c, waist_m, lleg_c, lleg_m, rleg_c, rleg_m);
      PrintDiagBlock(diag, ppp->legs_passive());  // per-joint cmd-vs-meas block (SHADOW/MOTION)
      PrintObsDebugBlock(obsd, ppp->last_action(),
                         ppp->planner_mode() ? ppp->planner_status() : std::string{});  // obs slices + stats
      PrintClampAudit("periodic", *ppp);
      PrintQdesProjectorAudit("periodic", *ppp);
      PrintQdesSlewAudit("periodic", *ppp);
      // one-shot warning if any joint is hitting its clamp on a large fraction of
      // ticks (the documented waist_roll mismatch): the policy keeps commanding
      // beyond the A3 limit. NOT a fault (clamp keeps it safe) — a tuning flag.
      const int wj = ppp->worst_clamped_slot();
      if (!clamp_rate_warned && wj >= 0 && ppp->clamp_ticks() > 100) {
        const double frac = static_cast<double>(ppp->clamp_count_for(wj)) /
                            static_cast<double>(ppp->clamp_ticks());
        if (frac > 0.20) {
          clamp_rate_warned = true;
          std::printf("[pingpong] WARN clamp-rate: %s clamped on %.0f%% of ticks "
                      "(max viol %.3f rad) -> policy commands beyond its A3 limit "
                      "(safe: clamped). See the waist_roll audit in the runbook.\n",
                      a3_pingpong::backend_joint_order()[wj].c_str(), 100.0 * frac,
                      ppp->clamp_max_viol_for(wj));
        }
      }
    }
    if (hybridp != nullptr) {
      const auto hd = hybridp->diag();
      std::printf(
          "[hybrid-lower] owner=%s outer_ticks=%llu actor_steps=%llu "
          "failures=%llu infer_mean=%.3fms infer_max=%.3fms\n",
          a3_pingpong::PpHybridLowerOwnerName(hybridp->config().owner),
          static_cast<unsigned long long>(hd.outer_ticks),
          static_cast<unsigned long long>(hd.inference_steps),
          static_cast<unsigned long long>(hd.failures),
          hd.mean_inference_ms,
          hd.max_inference_ms);
    }
    last_ticks = ticks; t_prev = now;
  }

  std::cout << "[pingpong] stopping...\n";
  PrintClampAudit("final", *ppp);
  PrintQdesProjectorAudit("final", *ppp);
  PrintQdesSlewAudit("final", *ppp);
  if (hybridp != nullptr) {
    const auto hd = hybridp->diag();
    std::printf(
        "[hybrid-lower] FINAL owner=%s outer_ticks=%llu actor_steps=%llu "
        "failures=%llu infer_mean=%.3fms infer_max=%.3fms\n",
        a3_pingpong::PpHybridLowerOwnerName(hybridp->config().owner),
        static_cast<unsigned long long>(hd.outer_ticks),
        static_cast<unsigned long long>(hd.inference_steps),
        static_cast<unsigned long long>(hd.failures),
        hd.mean_inference_ms,
        hd.max_inference_ms);
  }
  if (kb.joinable()) kb.join();
  if (runner_action_worker.joinable()) runner_action_worker.join();
  if (runner_state_publisher.joinable()) runner_state_publisher.join();
  if (tty) tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
  driver.StopDriver();
  backend->Stop();
  std::cout << "[pingpong] done\n";
  return 0;
}
