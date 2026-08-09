// Minimal entry point for running model_15200 (ping-pong, 180-obs/31-act) on the
// A3 via AGI's native runner. Reuses robot_io::A3AimrtBackend (iceoryx/ros2 sync)
// + a3_deploy::A3PolicyDriver (50 Hz RT loop + watchdog + safe-halt) UNCHANGED;
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
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <termios.h>
#include <unistd.h>

#include <yaml-cpp/yaml.h>

#include "a3_deploy/a3_policy_driver.hpp"
#include "a3_pingpong/pp_command_safety.hpp"
#include "a3_pingpong/pp_policy.hpp"
#include "a3_pingpong/pp_reference_playback.hpp"
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
#if defined(__x86_64__) || defined(_M_X64)
constexpr bool kGate3QdesAuditOnlySupported = true;
#else
constexpr bool kGate3QdesAuditOnlySupported = false;
#endif

std::atomic<bool> g_stop{false};
void OnSig(int) { g_stop.store(true); }

enum class Mode {
  kPassive,
  kPdStand,
  kShadow,
  kMotion,
  kReferencePlayback,
  kServe,
};
const char* ModeName(Mode m) {
  switch (m) {
    case Mode::kPassive: return "PASSIVE";
    case Mode::kPdStand: return "PD_STAND";
    case Mode::kShadow: return "SHADOW(no-publish)";
    case Mode::kMotion: return "MOTION";
    case Mode::kReferencePlayback: return "REFERENCE_PLAYBACK";
    case Mode::kServe: return "SERVE";
  }
  return "?";
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
      "a3_deploy/a3_deploy_example/assets/a3_runtime/motions/" + filename,
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
  if (n == a3_pingpong::kObsDim110 || n == a3_pingpong::kObsDim113 ||
      n == a3_pingpong::kObsDim118) {
    // hitter_pure: [96:99] grav, [99:101] e_base,x, [101:103] Δstation(world),
    // [103:106] racket rel base(world), [106:109] vel_w, [109] tts. No swing_type.
    std::printf("   e_base_x=[%+.3f %+.3f]  base_target_dxy=[%+.4f %+.4f]  "
                "racket_rel_base=[%+.4f %+.4f %+.4f]\n",
                o[99], o[100], o[101], o[102], o[103], o[104], o[105]);
    std::printf("   racket_target_vel_w=[%+.3f %+.3f %+.3f]  tts=%.3f  [%s]\n",
                o[106], o[107], o[108], o[109], src);
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
  const std::string cfg_path = Flag(argc, argv, "--runtime-cfg", "");
  if (cfg_path.empty()) {
    std::cerr << "usage: " << argv[0]
              << " --runtime-cfg PATH [--aimrt-cfg PATH]"
                 " [--policy-dir PATH]"
                 " [--start passive|pd_stand|shadow|motion|serve]"
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
                 " [--trace-csv PATH] [--obs-csv PATH] [--session-id ID] [--shadow-frozen-clock]\n"
                 "       [--leg-gain-scale F] [--ankle-gain-scale F] [--motion-blend-sec S]"
                 " [--squat-guard-rad R] [--tilt-guard G] [--leg-clamp-rad R]"
                 " [--leg-stand-gains] [--leg-smooth-alpha A]\n"
                 "       [--serve] [--serve-clip FIXED_CSV"
                 " --serve-manifest FIXED_JSON]"
                 "\n"
                 "       [--stationary-v17-r1-replay] (x86 MuJoCo-only "
                 "non-certifying binary)"
                 " [--allow-trained-lateral-recovery]"
                 " [--allow-fixed-y-homing]"
                 "\n"
                 "       [--v17-r10-gate3] (x86 MuJoCo/Gate3-only R10 P0)"
                 " [--gate3-qdes-audit-only] (x86 MuJoCo Gate3 telemetry; no "
                 "q_des fail-fast/clamp)"
                 "\n";
    return 2;
  }
  const fs::path cfgdir = fs::path(cfg_path).parent_path();
  YAML::Node cfg = YAML::LoadFile(cfg_path);

  const std::string run_mode = Flag(argc, argv, "--mode", "");
  const bool reference_playback_selected =
      Has(argc, argv, "--reference-playback") || run_mode == "reference-playback";
  const bool no_publish = Has(argc, argv, "--no-publish") || Has(argc, argv, "--dry-run");
  const bool stationary_v17_r1_replay =
      Has(argc, argv, "--stationary-v17-r1-replay");
  const bool v17_r10_gate3 = Has(argc, argv, "--v17-r10-gate3");
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
  if (stationary_v17_r1_replay && v17_r10_gate3) {
    std::cerr << "legacy stationary replay and V17-r10 Gate3 are mutually exclusive\n";
    return 2;
  }

  // LIVE PLANNER mode (Path B): racket target from /racket/command_flat + mocap base pose
  // from /a3/base_pose_flat, both over the AimRT ros2 backend; body-drive stays iceoryx.
  const bool planner_mode = Has(argc, argv, "--planner");
  const bool policy_native = Has(argc, argv, "--policy-native");
  const bool gate3_qdes_audit_only =
      Has(argc, argv, "--gate3-qdes-audit-only");
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
  a3_pingpong::LocMode loc_mode = a3_pingpong::LocMode::kFabricated;
  if (loc_mode_s == "perfect_tracking" || loc_mode_s == "B" || loc_mode_s == "b")
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
  const std::string oracle_shm =
      Flag(argc, argv, "--oracle-shm", odbg_str("oracle_shm_path", "/dev/shm/pp_oracle_pelvis"));
  const double oracle_max_age_s = std::stod(
      Flag(argc, argv, "--oracle-max-age", odbg["oracle_max_age_s"]
               ? std::to_string(odbg["oracle_max_age_s"].as<double>()) : "0.1"));
  const std::string obs_csv_path = Flag(argc, argv, "--obs-csv", odbg_str("obs_csv", ""));
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
  const bool serve_requested =
      Has(argc, argv, "--serve") || default_mode == Mode::kServe ||
      Has(argc, argv, "--serve-clip") || Has(argc, argv, "--serve-manifest") ||
      Has(argc, argv, "--serve-slow-clip") ||
      Has(argc, argv, "--serve-slow-manifest");
  if (Has(argc, argv, "--serve-drive-clip") ||
      Has(argc, argv, "--serve-drive-manifest") ||
      Has(argc, argv, "--serve-adaptive-branch") ||
      Has(argc, argv, "--serve-slow-clip") ||
      Has(argc, argv, "--serve-slow-manifest")) {
    std::cerr
        << "the retired adaptive/drive/slow serve flags are forbidden; "
           "production now loads one qualified fixed clip through "
           "--serve-clip/--serve-manifest\n";
    return 2;
  }
  if (serve_requested && reference_playback_selected) {
    std::cerr << "serve and reference-playback are mutually exclusive\n";
    return 2;
  }
  // Optional PD_STAND warmup: hold nominal for N s (robot settles upright),
  // then auto-switch to the requested mode. Matches a safe bring-up + lets a
  // non-interactive run reach MOTION from a stable stand.
  const double warmup_sec = std::stod(Flag(argc, argv, "--warmup-sec", "0"));
  const Mode target_mode = default_mode;
  std::atomic<Mode> mode{warmup_sec > 0 ? Mode::kPdStand : default_mode};

  // --- backend ---
  auto backend = std::make_unique<robot_io::A3AimrtBackend>();
  const std::string backend_cfg =
      BuildBackendCfg(cfg["backend"], aimrt_override_arg, cfgdir, no_publish);
  std::cout << "[pingpong] backend cfg: " << backend_cfg << "\n";
  if (!backend->Init(backend_cfg)) { std::cerr << "backend Init failed\n"; return 1; }
  std::cout << "[pingpong] A3AimrtBackend initialised; model=" << model_path;
  if (!deploy_cfg_path.empty()) {
    std::cout << " deploy_cfg=" << deploy_cfg_path
              << " policy_dir=" << policy_dir;
  }
  std::cout << "\n";

  // --- our front-end ---
  a3_pingpong::PpPolicyConfig pcfg;
  pcfg.deploy_cfg_path = deploy_cfg_path;
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
  // YAW-ALIGN default ON (hardware fix: boot-drifted IMU yaw polluted motion_anchor_ori_b
  // by a constant -12..-38 deg in MDU captures -> the policy fought a fictional torso yaw
  // error with legs/waist and fell during free-standing swings; no-op in sim). Opt out for
  // A/B debugging only.
  pcfg.yaw_align = !Has(argc, argv, "--no-yaw-align");
  auto pp = std::make_unique<a3_pingpong::PpPolicy>(model_path, pcfg);
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
  if (pp->onnx().has_bounded_qdes_contract() &&
      std::abs(pp->onnx().qdes_projector_dt_s() - policy_dt) > 1.0e-9) {
    std::cerr << "bounded_qdes runtime period mismatch: ONNX projector dt="
              << pp->onnx().qdes_projector_dt_s()
              << " s but policy_driver.policy_hz=" << policy_hz
              << " Hz (dt=" << policy_dt << " s); refusing to run\n";
    return 2;
  }

  // ---- LIVE PLANNER input wiring (Path B) ----
  // Backend AimRT subscribers (set BEFORE Start()) push decoded Float64MultiArrays into
  // thread-safe holders that PpPolicy reads on the 50 Hz driver thread. The racket topic
  // feeds the engage machine; the base topic feeds LocMode::kExternalBase.
  std::shared_ptr<a3_pingpong::PpBasePoseInput> base_in;
  std::shared_ptr<a3_pingpong::PpBallStateInput> serve_ball_in;
  if (planner_mode) {
    auto racket_in = std::make_shared<a3_pingpong::PpRacketTargetInput>();
    base_in = std::make_shared<a3_pingpong::PpBasePoseInput>();
    pp->SetRacketInput(racket_in);
    pp->SetBasePoseInput(base_in);
    backend->SetRacketTargetCallback(
        [racket_in](const std::vector<double>& a) { racket_in->SetFromFlat(a); });
    backend->SetBasePoseCallback(
        [base_in](const std::vector<double>& a) { base_in->SetFromFlat(a); });
    if (serve_requested) {
      serve_ball_in = std::make_shared<a3_pingpong::PpBallStateInput>();
      backend->SetBallStateCallback(
          [serve_ball_in](const std::vector<double>& a) {
            serve_ball_in->SetFromFlat(a);
          });
    }
    std::cout << "[pingpong] LIVE PLANNER: racket <- /racket/command_flat, base <- "
                 "/a3/base_pose_flat (std_msgs/Float64MultiArray, ros2); body-drive iceoryx\n";
    if (serve_requested) {
      std::cout << "[pingpong] SERVE observer: ball state <- "
                   "/serve/ball_state_flat (position-only 31-sample fit; "
                   "local-receipt freshness)\n";
    }
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
  std::string serve_clip_path;
  std::string serve_manifest_path;
  if (serve_requested) {
    if (!planner_mode || !policy_native) {
      std::cerr << "serve-to-rally handoff requires --planner --policy-native\n";
      return 2;
    }
    if (!pp->onnx().is_rally_v17_recipe()) {
      std::cerr << "serve handoff is qualified only for an ONNX artifact whose "
                   "training recipe is rally_v17\n";
      return 2;
    }
    if (std::abs(policy_hz - a3_pingpong::kServePolicyHz) > 1.0e-9) {
      std::cerr << "serve clip is qualified only at exactly 50 Hz\n";
      return 2;
    }
    const double serve_arm_gain_scale = gain_scale.load();
    const double serve_leg_gain_scale =
        Has(argc, argv, "--leg-gain-scale")
            ? std::stod(Flag(argc, argv, "--leg-gain-scale", "1.0"))
            : serve_arm_gain_scale;
    const double serve_ankle_gain_scale =
        Has(argc, argv, "--ankle-gain-scale")
            ? std::stod(Flag(argc, argv, "--ankle-gain-scale", "1.0"))
            : serve_leg_gain_scale;
    if (std::abs(serve_arm_gain_scale - 1.0) > 1.0e-12 ||
        std::abs(serve_leg_gain_scale - 1.0) > 1.0e-12 ||
        std::abs(serve_ankle_gain_scale - 1.0) > 1.0e-12) {
      std::cerr
          << "serve qualification fixes arm/leg/ankle runtime gain scales at "
             "1.0 so its final PD blend exactly matches V17 static gains\n";
      return 2;
    }
    serve_clip_path = Has(argc, argv, "--serve-clip")
                          ? Resolve(Flag(argc, argv, "--serve-clip", ""), cfgdir)
                          : DefaultServeAsset(cfgdir, "pp_serve_v1_fixed.csv");
    serve_manifest_path =
        Has(argc, argv, "--serve-manifest")
            ? Resolve(Flag(argc, argv, "--serve-manifest", ""), cfgdir)
            : DefaultServeAsset(
                  cfgdir, "pp_serve_v1_fixed.manifest.json");
    try {
      a3_pingpong::PpServeClip fixed_clip =
          a3_pingpong::PpServeClip::Load(
          serve_clip_path, serve_manifest_path, pp->onnx().default_q());
      a3_pingpong::ServeControllerConfig scfg;
      // The deploy state packet has no foot-contact channel.  Runtime READY
      // therefore uses every observable term (joints, IMU heading/tilt/rate,
      // and fresh mocap base position/velocity); the exact MuJoCo qualifier
      // additionally requires both feet in contact and low foot slip.
      scfg.require_external_base = true;
      scfg.base_max_age_s = pcfg.external_base_max_age_s;
      serve = std::make_unique<a3_pingpong::PpServeController>(
          std::move(fixed_clip), stand_q, scfg, base_in, serve_ball_in);
    } catch (const std::exception& error) {
      std::cerr << "serve artifact/controller preflight failed: "
                << error.what() << "\n";
      return 2;
    }
    std::cout << "[serve] palm-only deterministic controller qualified: clip="
              << serve_clip_path << " manifest=" << serve_manifest_path
              << " fixed_sha256=" << serve->clip().clip_sha256()
              << " frames=" << serve->clip().size() << "\n"
              << "[serve] left end effector: one rigid palm, 0 hand/finger DOF; "
                 "release is arm acceleration plus palm drop-away; "
                 "preflight requires the RallyV17 Gate3 station "
                 "(-0.50,-0.7625) m within 0.05 m and +X heading; "
              << "frame " << a3_pingpong::kServeBranchSelectionFrame
              << " requires a fresh in-envelope ball estimate, then executes "
                 "the single fixed strike; stale/out-of-envelope estimates "
                 "finish a toss-only safe return and never hand off to V17; "
                 "arm Kp/Kd scale=2.5/1.25, left proximal boost=1.35/1.15 "
                 "(qualified, fixed); final 0.5 s quintic gain blend -> "
                 "a3_pd_stand_static\n";
    if (default_mode == Mode::kServe) serve->Start();
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
  const std::string trace_path = Flag(argc, argv, "--trace-csv", "");
  std::ofstream trace;
  if (!trace_path.empty()) {
    trace.open(trace_path);
    if (trace) {
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
               ",planner_stable_revision_count,frozen_command_seq,frozen_flight_id"
               ",frozen_revision_id,frozen_strike_time,frozen_raw_tts"
               ",base_x,base_y,base_z,base_qw,base_qx,base_qy,base_qz"
               ",target_x,target_y,target_z,target_vx,target_vy,target_vz"
               ",racket_fk_valid,racket_x,racket_y,racket_z"
               ",racket_vx,racket_vy,racket_vz,racket_nx,racket_ny,racket_nz"
               ",session_id,qdes_projector_active,qdes_projector_rate"
               ",qdes_projector_tracking,qdes_projector_torque"
               ",qdes_projector_infeasible,qdes_projector_max_norm_debt"
               ",qdes_feasible_action_util_max,qdes_feasible_interval_width_min"
               ",qdes_feasible_rate_util_max,qdes_feasible_rate_bound"
               ",qdes_feasible_tracking_bound,qdes_feasible_torque_bound";
      for (const auto& n : nm) trace << ",des_" << n;
      for (const auto& n : nm) trace << ",clamp_viol_" << n;
      for (const auto& n : nm) trace << ",q_" << n;
      for (const auto& n : nm) trace << ",qd_" << n;
      for (const auto& n : nm) trace << ",kp_" << n;
      for (const auto& n : nm) trace << ",kd_" << n;
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
  // V17 field logs showed policy-native silently forcing this to zero, which
  // exposed the full stand->policy q_des discontinuity. Keep at least 0.5 s.
  const double policy_motion_blend_sec =
      policy_native ? std::max(0.5, motion_blend_sec) : motion_blend_sec;
  const bool v17_command_safety = pp->onnx().is_rally_v17_recipe();
  a3_pingpong::PpCommandSafetyMonitor command_safety;
  bool authoritative_mocap_stale_warned = false;  // driver-thread only (no race)
  Mode prev_mode_for_blend = Mode::kPassive;       // driver-thread only (no race)
  std::uint64_t stand_enter_tick = 0;              // PD_STAND entry blend (2026-07-04)
  Eigen::VectorXd stand_blend_q_start;
  std::uint64_t motion_enter_tick = 0;
  Eigen::VectorXd blend_q_start;
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
  auto command_fn = [ppp, refp, servep, &mode, &gain_scale, &leg_gain_scale, &ankle_gain_scale,
                     stand_q, stand_kp, stand_kd,
                     official_stand, auto_leg_hold, policy_native,
                     squat_guard_rad, tilt_guard, leg_stand_gains,
                     trace_ptr, obscsv_ptr, loc_mode_int, session_id,
                     &shadow_tick, shadow_free_clock, motion_blend_sec,
                     policy_motion_blend_sec, policy_dt,
                     &prev_mode_for_blend, &motion_enter_tick, &blend_q_start,
                     &prev_level_for_blend, &prev_swing_dir_for_blend,
                     &trace_previous_shot_seq, &trace_previous_q_des,
                     &stand_enter_tick, &stand_blend_q_start,
                     fall_guard, fall_guard_gz, &fall_guard_ticks,
                     v17_command_safety, &command_safety,
                     &authoritative_mocap_stale_warned](
                        std::uint64_t tick, const robot_io::RobotState& st,
                        robot_io::RobotCommand& cmd) -> bool {
    Mode m = mode.load();
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
          mode.store(Mode::kPassive);
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
    // so q_des ramps from the CURRENT measured pose -> no snap when stiff official stand gains
    // (re)engage at the 1->0 toggle, NOR when the reference jumps to the other clip's windup on a
    // dir switch (the swing clock restarts at windup in PpPolicy; this blends the command to match).
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
    if (rearm_blend) motion_enter_tick = tick;
    // Re-capture the IMU yaw-align offsets whenever the POLICY (SHADOW/MOTION) engages
    // from a non-policy mode — the operator may have moved/turned the robot in between.
    const bool policy_just_engaged =
        (m == Mode::kMotion || m == Mode::kShadow) &&
        prev_mode_for_blend != Mode::kMotion && prev_mode_for_blend != Mode::kShadow;
    const bool serve_to_policy =
        policy_just_engaged && prev_mode_for_blend == Mode::kServe;
    if (serve_to_policy) {
      ppp->rearm_static_policy_handoff();
    } else if (policy_just_engaged) {
      ppp->rearm_yaw_align();
    }
    prev_mode_for_blend = m;
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
      // pose-blend on PD_STAND ENTRY (2026-07-04): 's' pressed mid-swing used to slam the
      // stiff (kp~2000 knee) static stand target onto a moving robot with NO ramp — the
      // same catapult class as the auto-leg-hold level drop, and the one transition the
      // MOTION-only blend below did not cover. Ramp q_des from the entry pose.
      if (motion_blend_sec > 1e-6 && st.q.size() == N) {
        if (stand_just_entered) { stand_blend_q_start = st.q; stand_enter_tick = tick; }
        if (stand_blend_q_start.size() == N) {
          const double elapsed = static_cast<double>(tick - stand_enter_tick) * policy_dt;
          const double a = std::min(1.0, std::max(0.0, elapsed / motion_blend_sec));
          if (a < 1.0) cmd.q_des = (1.0 - a) * stand_blend_q_start + a * cmd.q_des;
        }
      }
    } else if (m == Mode::kReferencePlayback) {
      if (!refp->ComputeCommand(tick, st, cmd)) return false;
    } else if (m == Mode::kServe) {
      if (servep == nullptr) {
        std::fprintf(stderr,
                     "[serve] mode entered without a qualified controller\n");
        return false;
      }
      if (!servep->ComputeCommand(tick, st, cmd)) return false;
      if (servep->ConsumeHandoffRequest()) {
        // This tick still publishes the clip's exact V17 default q_des.  The
        // next tick enters MOTION through the dedicated static-handoff path:
        // planner/yaw state is reset, V17 affine previous-action history is
        // zeroed, and the first policy command remains this exact default.
        mode.store(Mode::kMotion);
        std::fprintf(stderr,
                     "[serve] strict 0.5 s handoff READY -> V17 MOTION armed\n");
      } else if (servep->state() ==
                 a3_pingpong::ServeControllerState::kAborted) {
        // A phase-aware abort has already reached the exact default pose.
        mode.store(Mode::kPdStand);
        std::fprintf(stderr,
                     "[serve] phase-aware abort complete -> PD_STAND\n");
      }
    } else {  // SHADOW or MOTION: run the policy
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
      // pose-blend on MOTION entry OR level toggle: ramp q_des from the entry/toggle pose
      // to the (new) target over motion_blend_sec, so stiff legs don't snap through the
      // windup jump NOR through the 1->0 official-stand re-engage. (convex combo of two
      // in-range poses -> in range; no clamp needed.)
      if (m == Mode::kMotion && policy_motion_blend_sec > 1e-6) {
        if (rearm_blend && st.q.size() == N) blend_q_start = st.q;
        if (blend_q_start.size() == N && cmd.q_des.size() == N) {
          const double elapsed = static_cast<double>(tick - motion_enter_tick) * policy_dt;
          const double a = std::min(1.0, std::max(0.0, elapsed / policy_motion_blend_sec));
          if (a < 1.0) cmd.q_des = (1.0 - a) * blend_q_start + a * cmd.q_des;
        }
      }
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
        << planner.frozen_command_seq << ',' << planner.frozen_flight_id << ','
        << planner.frozen_revision_id << ',' << planner.frozen_strike_time << ','
        << planner.frozen_raw_tts << ','
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
        << ppp->qdes_feasible_torque_bound_count();
      for (int i = 0; i < N; ++i) o << ',' << (has ? cmd.q_des[i] : 0.0);
      const auto& clamp_viol = ppp->last_clamp_viol();
      for (int i = 0; i < N; ++i)
        o << ',' << (clamp_viol.size() == N ? clamp_viol[i] : 0.0);
      for (int i = 0; i < N; ++i) o << ',' << (has ? st.q[i] : 0.0);
      for (int i = 0; i < N; ++i) o << ',' << (has ? st.dq[i] : 0.0);
      for (int i = 0; i < N; ++i) o << ',' << (has ? cmd.kp[i] : 0.0);
      for (int i = 0; i < N; ++i) o << ',' << (has ? cmd.kd[i] : 0.0);
      o << '\n';
      if (has) {
        trace_previous_q_des = cmd.q_des;
        trace_previous_shot_seq = planner.shot_seq;
      }
      static int fc = 0; if (++fc % 25 == 0) o.flush();  // single RT writer
    }
    return publish;
  };

  if (!backend->Start()) { std::cerr << "backend Start failed\n"; return 5; }
  std::cout << "[pingpong] backend started\n";

  a3_deploy::A3PolicyDriverOptions dopt;
  dopt.policy_hz = policy_hz;
  a3_deploy::CommandFn cfn = command_fn;  // disambiguate the PolicyFn/CommandFn ctor
  a3_deploy::A3PolicyDriver driver(*backend, cfn, dopt);
  if (!driver.StartDriver()) { std::cerr << "StartDriver failed\n"; backend->Stop(); return 6; }
  std::cout << "[pingpong] driver started @ " << dopt.policy_hz << " Hz\n";

  std::signal(SIGINT, OnSig);
  std::signal(SIGTERM, OnSig);

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
  std::printf(
      "[pingpong] ================= RUN CONFIG =================\n"
      "[pingpong]  start_mode   = %-9s  (s=PD_STAND hold/NO swing, m=MOTION publish)\n"
      "[pingpong]  level        = %-9d  (0=hold/windup, 1=SWING)\n"
      "[pingpong]  swing_dir    = %-9s  (f=forehand / b=backhand keys)\n"
      "[pingpong]  target_src   = %s\n"
      "[pingpong]  execution    = %-9s  (native=ball-clock release; policy owns rally lifecycle; SERVE uses exact-default static handoff)\n"
      "[pingpong]  action_src   = ONNX policy (LEARNED 31-DOF action every tick; q_des = default_q + a*action_scale)\n"
      "[pingpong]  post_onnx    = neck[3,4] HELD q=0 kp40 kd2 | legs %-6s | q_des CLAMPED to A3 limits (nothing else overridden)\n"
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
      pcfg.legs_passive ? "HELD" : "policy", pp->loc_mode_name(),
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
    std::cout << "[serve keys] v=approach rigid-palm READY / confirm ball-on-palm"
                 "  x=phase-aware abort  p=EMERGENCY limp\n";
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
            refp->Hold("operator_passive");
            mode.store(Mode::kPassive);
            if (servep != nullptr && servep->active()) {
              std::cout << "-> EMERGENCY PASSIVE during SERVE; restart the "
                           "runner before another serve\n";
            } else {
              std::cout << "-> PASSIVE\n";
            }
            break;
          case 's':
            if (mode.load() == Mode::kServe && servep != nullptr &&
                servep->active()) {
              servep->RequestAbort();
              std::cout << "-> SERVE phase-aware abort requested; PD_STAND "
                           "after safe recovery\n";
            } else {
              refp->Hold("operator_pd_stand");
              mode.store(Mode::kPdStand);
              std::cout << "-> PD_STAND\n";
            }
            break;
          case 'h':
            if (mode.load() == Mode::kServe && servep != nullptr &&
                servep->active()) {
              std::cout << "-> SHADOW rejected while SERVE owns q_des; press x\n";
            } else {
              mode.store(Mode::kShadow);
              std::cout << "-> SHADOW (no publish)\n";
            }
            break;
          case 'm':
            if (mode.load() == Mode::kServe && servep != nullptr &&
                servep->active()) {
              std::cout << "-> MOTION rejected before strict SERVE handoff\n";
            } else {
              mode.store(Mode::kMotion);
              std::cout << "-> MOTION (PUBLISHING)\n";
            }
            break;
          case 'v':
            if (servep == nullptr) {
              std::cout << "-> SERVE unavailable; launch with --serve\n";
            } else if (servep->state() ==
                       a3_pingpong::ServeControllerState::kAwaitBall) {
              servep->ConfirmBallOnPalm();
              std::cout << "-> ball-on-rigid-palm CONFIRMED; toss/swing armed\n";
            } else if (servep->state() ==
                       a3_pingpong::ServeControllerState::kFault) {
              std::cout << "-> SERVE fault is latched; restart runner\n";
            } else if (servep->active()) {
              std::cout << "-> SERVE already active; wait or press x\n";
            } else {
              const double arm_scale = gain_scale.load();
              const double leg_override = leg_gain_scale.load();
              const double leg_scale =
                  leg_override >= 0.0 ? leg_override : arm_scale;
              const double ankle_override = ankle_gain_scale.load();
              const double ankle_scale =
                  ankle_override >= 0.0 ? ankle_override : leg_scale;
              if (std::abs(arm_scale - 1.0) > 1.0e-12 ||
                  std::abs(leg_scale - 1.0) > 1.0e-12 ||
                  std::abs(ankle_scale - 1.0) > 1.0e-12) {
                std::cout
                    << "-> SERVE rejected: restore arm/leg/ankle gain scales "
                       "to exactly 1.0\n";
              } else {
                servep->Start();
                mode.store(Mode::kServe);
                std::cout << "-> SERVE pre-positioning both arms; do not place "
                             "the ball until AWAIT_BALL_ON_PALM\n";
              }
            }
            break;
          case '0':
          case '1':
          case '2':
          case '3':
          case '4':
          case '5':
          case '6':
          case '7':
            if (reference_playback_selected || mode.load() == Mode::kReferencePlayback) {
              const int gi = c - '0';
              refp->SetGroup(a3_pingpong::RefPlaybackGroupFromInt(gi));
              refp->Hold("group_selected_hold");
              mode.store(Mode::kReferencePlayback);
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
            if (mode.load() == Mode::kServe && servep != nullptr &&
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
            if (mode.load() == Mode::kServe && servep != nullptr &&
                servep->active()) {
              std::cout << "-> REFERENCE_PLAYBACK rejected while SERVE owns q_des\n";
            } else {
              refp->Start();
              mode.store(Mode::kReferencePlayback);
              std::cout << "-> REFERENCE_PLAYBACK moving group="
                        << a3_pingpong::RefPlaybackGroupName(refp->group())
                        << "\n";
            }
            break;
          case 'x':
            if (mode.load() == Mode::kServe && servep != nullptr &&
                servep->active()) {
              servep->RequestAbort();
              std::cout << "-> SERVE phase-aware abort requested\n";
            } else {
              refp->Hold("operator_hold");
              mode.store(Mode::kReferencePlayback);
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

  // --- status loop ---
  std::uint64_t last_ticks = 0;
  auto t_start = std::chrono::steady_clock::now();
  auto t_prev = t_start;
  bool clamp_rate_warned = false;  // one-shot high-clamp-rate warning (waist_roll audit)
  bool warming = warmup_sec > 0;
  if (warming)
    std::printf("[pingpong] warmup: PD_STAND for %.1fs, then -> %s\n", warmup_sec,
                ModeName(target_mode));
  while (!g_stop.load()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    const std::uint64_t ticks = driver.PolicyTickCount();
    const std::uint64_t halts = driver.SafeHaltCount();
    auto now = std::chrono::steady_clock::now();
    if (warming &&
        std::chrono::duration<double>(now - t_start).count() >= warmup_sec) {
      mode.store(target_mode);
      warming = false;
      std::printf("[pingpong] warmup done -> %s\n", ModeName(target_mode));
    }
    double dt = std::chrono::duration<double>(now - t_prev).count();
    double hz = dt > 0 ? (ticks - last_ticks) / dt : 0;
    const auto g = ppp->last_proj_grav();
    const Mode cur_mode = mode.load();
    if (cur_mode == Mode::kServe && servep != nullptr) {
      const auto d = servep->TakeDiag();
      std::printf(
          "[status] mode=SERVE state=%s phase=%s frame=%zu/%zu "
          "ready_ticks=%d local_ready=%d q_or_tracking_err=%.3f "
          "joint_speed=%.3f tilt=%.3f yaw_rate=%.3f "
          "branch=%s ball_vx=%.4f ball_age=%.3f ball_n=%d branch_reason=%s "
          "toss_only_abort=%d abort_after_commit=%d fault=%s "
          "rate=%.1fHz ticks=%llu halts=%llu\n",
          a3_pingpong::ServeControllerStateName(d.state),
          a3_pingpong::ServePhaseName(d.phase), d.frame,
          servep->clip().size(), d.ready_ticks, d.local_ready ? 1 : 0,
          d.max_q_error, d.max_joint_speed, d.tilt_rad, d.yaw_rate_rad_s,
          d.selected_branch.empty() ? "pending" : d.selected_branch.c_str(),
          d.ball_vx_mps, d.ball_age_s, d.ball_estimator_samples,
          d.branch_reason.empty() ? "-" : d.branch_reason.c_str(),
          d.toss_only_abort ? 1 : 0,
          d.abort_after_commit ? 1 : 0,
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
    last_ticks = ticks; t_prev = now;
  }

  std::cout << "[pingpong] stopping...\n";
  PrintClampAudit("final", *ppp);
  PrintQdesProjectorAudit("final", *ppp);
  if (kb.joinable()) kb.join();
  if (tty) tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
  driver.StopDriver();
  backend->Stop();
  std::cout << "[pingpong] done\n";
  return 0;
}
