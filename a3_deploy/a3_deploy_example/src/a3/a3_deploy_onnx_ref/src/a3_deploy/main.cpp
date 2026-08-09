// Copyright (c) 2026, AgiBot Inc. All rights reserved.
// 对应 notes/a3_pr9_spec.md §3 (Main program wiring)
//
// a3_deploy_onnx_ref — A3 policy-loop binary.
//
// Wires together the PR 7/8/9 components into a single runnable entry point:
//   A3AimrtBackend  (RobotIOBackend impl + sync loop)
//   A3PolicyRuntime (ONNX Runtime CPU by default; TensorRT optional)
//   A3CsvMotionReference (CSV reference motion -> online tokenizer prefix)
//   A3ObsBuilder    (proprioception ring + obs_dict assembly)
//   A3PolicyDriver  (50Hz RT task + watchdog + safe-halt)
//
// Scope: launch on x86 host against mujoco_sim (PR 10 integration). Not a
// full end-to-end test — PR 10 will wire the loopback path in Zach's env.
#include "a3_deploy/a3_action_decoder.hpp"
#include "a3_deploy/a3_csv_motion_reference.hpp"
#include "a3_deploy/a3_encoder_decoder_runtime.hpp"
#include "a3_deploy/a3_encoder_obs_builder.hpp"
#include "a3_deploy/expand_to_backend.hpp"
#include "a3_deploy/a3_manual_control.hpp"
#include "a3_deploy/a3_motion_library.hpp"
#include "a3_deploy/a3_obs_builder.hpp"
#include "a3_deploy/a3_policy_driver.hpp"
#include "a3_deploy/a3_policy_runtime.hpp"
#include "a3_deploy/numeric_safety.hpp"
#include "a3_deploy/safe_halt.hpp"
#include "a3_deploy/a3_teleop_reference.hpp"
#include "a3_deploy/a3_zmq_smpl_source.hpp"
#include "a3_policy_parameters.hpp"
#include "math_utils.hpp"
#include "robot_io/a3_layout_extra.hpp"
#include "robot_io/a3_aimrt_backend.hpp"
#include "robot_io/robot_io_backend.hpp"

#include <yaml-cpp/yaml.h>

#include <sys/select.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

using a3_deploy::DeployMode;
using a3_deploy::DeployModeName;
using a3_deploy::HandleManualKey;
using a3_deploy::LoadDeployMode;
using a3_deploy::ManualControlState;
using a3_deploy::ManualKeyOutcome;
using a3_deploy::ManualMotionShortcuts;
using a3_deploy::ParseManualKey;
using a3_deploy::PrintMotionHelp;

constexpr double kRadToDeg = 57.295779513082320876798154814105;
constexpr std::int64_t kNsPerSec = 1'000'000'000LL;

std::int64_t MsToNs(double ms) {
  return static_cast<std::int64_t>(std::llround(ms * 1'000'000.0));
}

std::int64_t NowMonotonicNs() {
  timespec ts{};
  if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
    throw std::runtime_error("clock_gettime(CLOCK_MONOTONIC) failed");
  }
  return static_cast<std::int64_t>(ts.tv_sec) * kNsPerSec +
         static_cast<std::int64_t>(ts.tv_nsec);
}

std::int64_t NowMonotonicNsNoThrow() noexcept {
  timespec ts{};
  if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return 0;
  return static_cast<std::int64_t>(ts.tv_sec) * kNsPerSec +
         static_cast<std::int64_t>(ts.tv_nsec);
}

std::int64_t ModNs(std::int64_t value, std::int64_t period_ns) {
  auto mod = value % period_ns;
  if (mod < 0) mod += period_ns;
  return mod;
}

std::int64_t NextSystemTimeAtPhaseNs(std::int64_t now_ns,
                                     std::int64_t period_ns,
                                     std::int64_t phase_ns,
                                     std::int64_t min_delay_ns) {
  const auto earliest_ns = now_ns + min_delay_ns;
  const auto adjusted_ns = earliest_ns - phase_ns;
  auto periods = adjusted_ns / period_ns;
  if (adjusted_ns % period_ns != 0) ++periods;
  auto target_ns = periods * period_ns + phase_ns;
  if (target_ns <= earliest_ns) target_ns += period_ns;
  return target_ns;
}

// --- CLI parsing -----------------------------------------------------------
std::string ParseRuntimeCfgFlag(int argc, char** argv) {
  const std::string prefix = "--runtime-cfg=";
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg.rfind(prefix, 0) == 0) return arg.substr(prefix.size());
    if (arg == "--runtime-cfg" && i + 1 < argc) return argv[i + 1];
  }
  return {};
}

bool HasFlag(int argc, char** argv, const char* flag) {
  for (int i = 1; i < argc; ++i)
    if (std::string(argv[i]) == flag) return true;
  return false;
}

bool ValidateKnownFlags(int argc, char** argv, std::string* error) {
  const std::array<std::string, 3> bool_flags = {
      "--dry-run", "--probe", "--auto-start"};
  const std::array<std::string, 4> value_flags = {
      "--runtime-cfg", "--probe-source", "--aimrt-cfg",
      "--frame-log-interval"};
  auto contains = [](const auto& values, const std::string& needle) {
    return std::find(values.begin(), values.end(), needle) != values.end();
  };

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto eq_pos = arg.find('=');
    const std::string name =
        eq_pos == std::string::npos ? arg : arg.substr(0, eq_pos);

    if (contains(bool_flags, name)) {
      if (eq_pos != std::string::npos) {
        if (error) *error = name + " does not take a value";
        return false;
      }
      continue;
    }
    if (contains(value_flags, name)) {
      if (eq_pos != std::string::npos) {
        if (eq_pos + 1 >= arg.size()) {
          if (error) *error = name + " requires a value";
          return false;
        }
        continue;
      }
      if (i + 1 >= argc || std::string(argv[i + 1]).rfind("--", 0) == 0) {
        if (error) *error = name + " requires a value";
        return false;
      }
      ++i;
      continue;
    }

    if (arg.rfind("--", 0) == 0) {
      if (error) *error = "unknown argument: " + arg;
      return false;
    }
    if (error) *error = "unexpected positional argument: " + arg;
    return false;
  }
  return true;
}

std::string ParseStringFlag(int argc, char** argv, const char* name,
                            const std::string& fallback) {
  const std::string prefix = std::string(name) + "=";
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg.rfind(prefix, 0) == 0) return arg.substr(prefix.size());
    if (arg == name && i + 1 < argc) return argv[i + 1];
  }
  return fallback;
}

bool ParseUint64Flag(int argc, char** argv, const char* name,
                     std::uint64_t* out, std::string* error) {
  const std::string prefix = std::string(name) + "=";
  for (int i = 1; i < argc; ++i) {
    std::string value;
    const std::string arg = argv[i];
    if (arg.rfind(prefix, 0) == 0) {
      value = arg.substr(prefix.size());
    } else if (arg == name) {
      if (i + 1 >= argc) {
        if (error) *error = std::string(name) + " requires a value";
        return false;
      }
      value = argv[i + 1];
    } else {
      continue;
    }

    try {
      if (value.empty() || value.front() == '-') {
        throw std::invalid_argument("not a non-negative integer");
      }
      std::size_t parsed = 0;
      const auto v = std::stoull(value, &parsed, 10);
      if (parsed != value.size()) {
        throw std::invalid_argument("trailing characters");
      }
      if (out) *out = static_cast<std::uint64_t>(v);
      return true;
    } catch (const std::exception& e) {
      if (error) {
        *error = std::string(name) + " must be a non-negative integer; got '" +
                 value + "'";
      }
      return false;
    }
  }
  return true;
}

void PrintUsage(const char* progname) {
  std::cerr << "Usage: " << progname << " --runtime-cfg=/path/to/config.yaml"
               " [--dry-run] [--probe] [--auto-start]"
               " [--probe-source a3|smpl|both]"
               " [--aimrt-cfg PATH] [--frame-log-interval N]"
               "\n"
            << "\nSee src/a3/a3_deploy_onnx_ref/config/a3_runtime_config.yaml\n"
               "for an annotated example.\n"
            << "\n--dry-run Start transport/backend only; do not load policy or publish commands.\n"
               "--probe  Run receive/sync + policy inference latency probe; do not publish commands.\n"
               "--probe-source a3|smpl|both  Select probe inference source; default both.\n"
               "--auto-start Use the simulation/debug flow: PD warmup then policy inference.\n"
               "             Without this flag, startup is PASSIVE and keyboard-controlled.\n"
               "--aimrt-cfg PATH  Override backend.aimrt_cfg_path from runtime config.\n"
               "--frame-log-interval N  Override logging.frame_log_interval (0 disables).\n";
}

// --- Config accessors ------------------------------------------------------
template <typename T>
T RequiredKey(const YAML::Node& node, const std::string& path) {
  if (!node || node.IsNull()) {
    throw std::runtime_error("required config key missing: " + path);
  }
  return node.as<T>();
}

template <typename T>
T OptionalKey(const YAML::Node& node, const T& fallback) {
  if (!node || node.IsNull()) return fallback;
  return node.as<T>();
}

std::string ModelNameFromPath(const std::string& model_path) {
  const std::string filename =
      std::filesystem::path(model_path).filename().string();
  return filename.empty() ? model_path : filename;
}

std::string NormalizeConfigToken(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) {
                   if (c == '-') return static_cast<char>('_');
                   return static_cast<char>(std::tolower(c));
                 });
  return value;
}

enum class LatencyLogMode {
  kCompact,
  kVerbose,
};

LatencyLogMode ParseLatencyLogModeFromEnv() {
  const char* raw = std::getenv("A3_LATENCY_LOG");
  if (!raw || raw[0] == '\0') return LatencyLogMode::kCompact;

  const std::string value = NormalizeConfigToken(raw);
  if (value == "verbose" || value == "full" || value == "debug") {
    return LatencyLogMode::kVerbose;
  }
  if (value == "compact" || value == "brief" || value == "short") {
    return LatencyLogMode::kCompact;
  }

  std::cerr << "[logging] WARN: unknown A3_LATENCY_LOG=" << raw
            << "; using compact latency logs\n";
  return LatencyLogMode::kCompact;
}

const char* LatencyLogModeName(LatencyLogMode mode) {
  switch (mode) {
    case LatencyLogMode::kVerbose:
      return "verbose";
    case LatencyLogMode::kCompact:
    default:
      return "compact";
  }
}

// Encode the A3AimrtBackend config string from the YAML "backend:" block.
std::string BuildBackendConfigString(const YAML::Node& backend,
                                     const std::string& aimrt_cfg_override,
                                     bool force_publish_disabled,
                                     double policy_hz) {
  std::stringstream ss;
  bool first = true;
  auto add = [&](const std::string& kv) {
    if (!first) ss << ',';
    ss << kv;
    first = false;
  };
  const std::string aimrt_cfg =
      aimrt_cfg_override.empty()
          ? RequiredKey<std::string>(backend["aimrt_cfg_path"],
                                     "backend.aimrt_cfg_path")
          : aimrt_cfg_override;
  add("cfg_file_path=" + aimrt_cfg);
  const std::string sync_mode =
      backend["sync_mode"] ? backend["sync_mode"].as<std::string>()
                           : std::string{"min_skew_pair"};
  add("sync_mode=" + sync_mode);
  const double sync_hz =
      backend["sync_hz"] ? backend["sync_hz"].as<double>() : (policy_hz * 2.0);
  add("sync_hz=" + std::to_string(sync_hz));
  if (backend["align_delay_ms"]) {
    add("align_delay_ms=" +
        std::to_string(backend["align_delay_ms"].as<double>()));
  }
  if (backend["phase_ms"]) {
    add("phase_ms=" + std::to_string(backend["phase_ms"].as<double>()));
  }
  if (backend["auto_phase"]) {
    add(std::string{"auto_phase="} +
        (backend["auto_phase"].as<bool>() ? "true" : "false"));
  }
  if (backend["max_skew_ms"]) {
    add("max_skew_ms=" + std::to_string(backend["max_skew_ms"].as<double>()));
  }
  if (backend["max_sample_age_ms"]) {
    add("max_sample_age_ms=" +
        std::to_string(backend["max_sample_age_ms"].as<double>()));
  }
  if (backend["sync_ready_after_input_ms"]) {
    add("sync_ready_after_input_ms=" +
        std::to_string(backend["sync_ready_after_input_ms"].as<double>()));
  }
  if (backend["sync_release_margin_ms"]) {
    add("sync_release_margin_ms=" +
        std::to_string(backend["sync_release_margin_ms"].as<double>()));
  }
  if (backend["max_group_internal_skew_ms"]) {
    add("max_group_internal_skew_ms=" +
        std::to_string(backend["max_group_internal_skew_ms"].as<double>()));
  }
  if (backend["max_group_pair_skew_ms"]) {
    add("max_group_pair_skew_ms=" +
        std::to_string(backend["max_group_pair_skew_ms"].as<double>()));
  }
  if (backend["group_pair_search_depth"]) {
    add("group_pair_search_depth=" +
        std::to_string(backend["group_pair_search_depth"].as<int>()));
  }
  if (backend["max_backtrack"]) {
    add("max_backtrack=" +
        std::to_string(backend["max_backtrack"].as<int>()));
  }
  const bool publish_enabled =
      force_publish_disabled
          ? false
          : OptionalKey<bool>(backend["publish_enabled"], true);
  add(std::string{"publish_enabled="} + (publish_enabled ? "true" : "false"));
  return ss.str();
}

a3_deploy::OnEndPolicy ParseOnEndPolicy(
    const std::string& s,
    const std::string& config_path = "reference_motion.on_end") {
  if (s == "hold_last") return a3_deploy::OnEndPolicy::kHoldLast;
  if (s == "wrap")      return a3_deploy::OnEndPolicy::kWrap;
  if (s == "stop")      return a3_deploy::OnEndPolicy::kStop;
  throw std::runtime_error(
      config_path + " must be one of {hold_last, wrap, stop}; got: " +
      s);
}

bool ValidateCsvPath(const std::filesystem::path& csv_path,
                     const std::string& config_key) {
  std::error_code ec;
  if (!std::filesystem::exists(csv_path, ec) || ec) {
    std::cerr << config_key << " does not exist: " << csv_path.string();
    if (ec) std::cerr << " (" << ec.message() << ")";
    std::cerr << "\n";
    return false;
  }
  ec.clear();
  if (!std::filesystem::is_regular_file(csv_path, ec) || ec) {
    std::cerr << config_key << " must be a regular CSV file: "
              << csv_path.string();
    if (ec) std::cerr << " (" << ec.message() << ")";
    std::cerr << "\n";
    return false;
  }
  if (csv_path.extension() != ".csv") {
    std::cerr << config_key << " must end with .csv: "
              << csv_path.string() << "\n";
    return false;
  }
  return true;
}

std::string ResolveRuntimePathString(const std::string& raw,
                                     const std::filesystem::path& cfg_path) {
  if (raw.empty()) return raw;
  const std::filesystem::path path(raw);
  if (path.is_absolute() || std::filesystem::exists(path)) return raw;

  std::error_code ec;
  std::filesystem::path cursor = std::filesystem::weakly_canonical(cfg_path, ec);
  if (ec) {
    cursor = std::filesystem::absolute(cfg_path, ec);
  }
  if (ec) return raw;
  cursor = cursor.parent_path();

  while (!cursor.empty()) {
    const auto candidate = cursor / path;
    if (std::filesystem::exists(candidate)) {
      return candidate.lexically_normal().string();
    }
    const auto parent = cursor.parent_path();
    if (parent == cursor) break;
    cursor = parent;
  }
  return raw;
}

int ResolveMotionShortcutIndex(const YAML::Node& node,
                               const std::vector<std::string>& motion_names,
                               const std::string& config_key) {
  if (!node || node.IsNull()) return -1;
  const std::string requested = node.as<std::string>();
  for (std::size_t i = 0; i < motion_names.size(); ++i) {
    if (motion_names[i] == requested || motion_names[i] + ".csv" == requested) {
      return static_cast<int>(i);
    }
  }
  std::cerr << config_key << " does not match any loaded motion: "
            << requested << "\n";
  return -1;
}

// --- Shutdown plumbing -----------------------------------------------------
std::atomic<bool> g_stop_requested{false};
std::mutex        g_stop_mu;
std::condition_variable g_stop_cv;

void RequestShutdown() {
  g_stop_requested.store(true, std::memory_order_release);
  g_stop_cv.notify_all();
}

std::int64_t NowSystemNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

struct AtomicLatencyBucket {
  static constexpr std::int64_t kNoMin =
      std::numeric_limits<std::int64_t>::max();
  static constexpr std::int64_t kNoMax =
      std::numeric_limits<std::int64_t>::min();

  std::atomic<std::uint64_t> count{0};
  std::atomic<std::int64_t> total_ns{0};
  std::atomic<std::int64_t> min_ns{kNoMin};
  std::atomic<std::int64_t> max_ns{kNoMax};
};

struct InferenceTimingStats {
  AtomicLatencyBucket state_transport_apparent{};
  AtomicLatencyBucket state_ready{};
  AtomicLatencyBucket state_sync_wait{};
  AtomicLatencyBucket policy_wait{};
  AtomicLatencyBucket infer{};
  AtomicLatencyBucket infer_a3{};
  AtomicLatencyBucket infer_smpl{};
  AtomicLatencyBucket state2action{};
  AtomicLatencyBucket state2action_a3{};
  AtomicLatencyBucket state2action_smpl{};
};

void RecordLatencyNs(AtomicLatencyBucket& bucket, std::int64_t ns) {
  bucket.total_ns.fetch_add(ns, std::memory_order_relaxed);

  auto old_min = bucket.min_ns.load(std::memory_order_relaxed);
  while (ns < old_min &&
         !bucket.min_ns.compare_exchange_weak(old_min, ns,
                                              std::memory_order_release,
                                              std::memory_order_relaxed)) {
  }
  auto old_max = bucket.max_ns.load(std::memory_order_relaxed);
  while (ns > old_max &&
         !bucket.max_ns.compare_exchange_weak(old_max, ns,
                                              std::memory_order_release,
                                              std::memory_order_relaxed)) {
  }
  bucket.count.fetch_add(1, std::memory_order_release);
}

a3_sync::A3SyncStatistics::LatencyBucket ConsumeLatencyBucket(
    AtomicLatencyBucket& bucket) {
  a3_sync::A3SyncStatistics::LatencyBucket out;
  out.count = bucket.count.exchange(0, std::memory_order_acq_rel);
  if (out.count == 0) return out;
  out.total_ns = bucket.total_ns.exchange(0, std::memory_order_acq_rel);
  out.min_ns = bucket.min_ns.exchange(AtomicLatencyBucket::kNoMin,
                                      std::memory_order_acq_rel);
  out.max_ns = bucket.max_ns.exchange(AtomicLatencyBucket::kNoMax,
                                      std::memory_order_acq_rel);
  if (out.min_ns == AtomicLatencyBucket::kNoMin ||
      out.max_ns == AtomicLatencyBucket::kNoMax) {
    return {};
  }
  return out;
}

struct StateTimingSample {
  bool valid{false};
  std::int64_t transport_ns{0};
  std::int64_t ready_ns{0};
  bool split_valid{false};
  std::int64_t sync_wait_ns{0};
  std::int64_t policy_wait_ns{0};
};

StateTimingSample CaptureStateTimingSample(
    const robot_io::RobotState& state,
    std::int64_t infer_start_system_ns) {
  StateTimingSample sample;
  if (state.timestamp_ns <= 0 || state.state_data_ready_ns <= 0) {
    return sample;
  }
  sample.valid = true;
  sample.transport_ns = state.state_data_ready_ns - state.timestamp_ns;
  sample.ready_ns = infer_start_system_ns - state.state_data_ready_ns;
  if (state.state_sync_ready_ns > 0) {
    sample.sync_wait_ns = state.state_sync_ready_ns - state.state_data_ready_ns;
    sample.policy_wait_ns = infer_start_system_ns - state.state_sync_ready_ns;
    sample.split_valid = sample.sync_wait_ns >= 0 && sample.policy_wait_ns >= 0;
  }
  return sample;
}

void RecordStateTimingSample(InferenceTimingStats& stats,
                             const StateTimingSample& sample) {
  if (!sample.valid) return;
  RecordLatencyNs(stats.state_transport_apparent, sample.transport_ns);
  RecordLatencyNs(stats.state_ready, sample.ready_ns);
  if (sample.split_valid) {
    RecordLatencyNs(stats.state_sync_wait, sample.sync_wait_ns);
    RecordLatencyNs(stats.policy_wait, sample.policy_wait_ns);
  }
}

void RecordPolicyTimingSample(InferenceTimingStats& stats,
                              const robot_io::RobotState& state,
                              std::int64_t infer_start_system_ns,
                              std::int64_t infer_ns) {
  RecordLatencyNs(stats.infer, infer_ns);
  const auto timing = CaptureStateTimingSample(state, infer_start_system_ns);
  RecordStateTimingSample(stats, timing);
  if (timing.valid) {
    RecordLatencyNs(stats.state2action,
                    timing.transport_ns + timing.ready_ns + infer_ns);
  }
}

void RecordProbeComparisonTimingSample(InferenceTimingStats& stats,
                                       const robot_io::RobotState& state,
                                       std::int64_t infer_start_system_ns,
                                       std::int64_t a3_infer_ns,
                                       std::int64_t smpl_infer_ns) {
  RecordLatencyNs(stats.infer_a3, a3_infer_ns);
  RecordLatencyNs(stats.infer_smpl, smpl_infer_ns);
  const auto timing = CaptureStateTimingSample(state, infer_start_system_ns);
  RecordStateTimingSample(stats, timing);
  if (timing.valid) {
    RecordLatencyNs(stats.state2action_a3,
                    timing.transport_ns + timing.ready_ns + a3_infer_ns);
    RecordLatencyNs(stats.state2action_smpl,
                    timing.transport_ns + timing.ready_ns + smpl_infer_ns);
  }
}

bool AppendCompactLatencyBucket(
    const char* label,
    const a3_sync::A3SyncStatistics::LatencyBucket& bucket) {
  if (bucket.count == 0) return false;
  const auto to_ms = [](std::int64_t ns) {
    return static_cast<double>(ns) / 1'000'000.0;
  };
  const auto old_flags = std::cout.flags();
  const auto old_precision = std::cout.precision();
  std::cout << " | " << label << "="
            << std::fixed << std::setprecision(2)
            << to_ms(bucket.min_ns) << "/"
            << (static_cast<double>(bucket.total_ns) /
                static_cast<double>(bucket.count) / 1'000'000.0)
            << "/" << to_ms(bucket.max_ns);
  std::cout.flags(old_flags);
  std::cout.precision(old_precision);
  return true;
}

bool AppendAvgMaxLatencyBucket(
    const char* label,
    const a3_sync::A3SyncStatistics::LatencyBucket& bucket) {
  if (bucket.count == 0) return false;
  const auto old_flags = std::cout.flags();
  const auto old_precision = std::cout.precision();
  const double avg_ms =
      static_cast<double>(bucket.total_ns) /
      static_cast<double>(bucket.count) / 1'000'000.0;
  const double max_ms = static_cast<double>(bucket.max_ns) / 1'000'000.0;
  std::cout << " " << label << "=" << std::fixed << std::setprecision(2)
            << avg_ms << "/" << max_ms;
  std::cout.flags(old_flags);
  std::cout.precision(old_precision);
  return true;
}

void AppendInferenceTimingLog(InferenceTimingStats& stats,
                              bool include_sync_breakdown,
                              LatencyLogMode log_mode) {
  const auto transport = ConsumeLatencyBucket(stats.state_transport_apparent);
  const auto ready = ConsumeLatencyBucket(stats.state_ready);
  const auto sync_wait = ConsumeLatencyBucket(stats.state_sync_wait);
  const auto policy_wait = ConsumeLatencyBucket(stats.policy_wait);
  const auto infer = ConsumeLatencyBucket(stats.infer);
  const auto infer_a3 = ConsumeLatencyBucket(stats.infer_a3);
  const auto infer_smpl = ConsumeLatencyBucket(stats.infer_smpl);
  const auto state2action = ConsumeLatencyBucket(stats.state2action);
  const auto state2action_a3 = ConsumeLatencyBucket(stats.state2action_a3);
  const auto state2action_smpl = ConsumeLatencyBucket(stats.state2action_smpl);
  if (transport.count == 0 && ready.count == 0 && sync_wait.count == 0 &&
      policy_wait.count == 0 && infer.count == 0 && infer_a3.count == 0 &&
      infer_smpl.count == 0 &&
      state2action.count == 0 && state2action_a3.count == 0 &&
      state2action_smpl.count == 0) {
    return;
  }

  if (log_mode == LatencyLogMode::kCompact) {
    const auto& primary_total =
        (state2action_a3.count > 0) ? state2action_a3 : state2action;
    const auto& primary_infer =
        (infer_a3.count > 0) ? infer_a3 : infer;
    std::cout << " | lat_ms avg/max";
    AppendAvgMaxLatencyBucket(
        (state2action_a3.count > 0) ? "a3" : "total", primary_total);
    AppendAvgMaxLatencyBucket("rx", transport);
    AppendAvgMaxLatencyBucket("ready", ready);
    AppendAvgMaxLatencyBucket("sync", sync_wait);
    AppendAvgMaxLatencyBucket("policy", policy_wait);
    AppendAvgMaxLatencyBucket("infer", primary_infer);
    return;
  }

  std::cout << "\n         latency_ms min/avg/max";
  AppendCompactLatencyBucket("transport", transport);
  AppendCompactLatencyBucket("ready", ready);
  if (include_sync_breakdown) {
    AppendCompactLatencyBucket("sync_wait", sync_wait);
    AppendCompactLatencyBucket("policy_wait", policy_wait);
  }
  AppendCompactLatencyBucket("infer", infer);
  AppendCompactLatencyBucket("infer_a3", infer_a3);
  AppendCompactLatencyBucket("infer_smpl", infer_smpl);
  AppendCompactLatencyBucket("total", state2action);
  AppendCompactLatencyBucket("total_a3", state2action_a3);
  AppendCompactLatencyBucket("total_smpl", state2action_smpl);
}

void AppendHeaderSkewLog(
    const a3_sync::A3SyncStatistics::LatencyBucket& bucket) {
  if (bucket.count == 0) return;
  const auto to_ms = [](std::int64_t ns) {
    return static_cast<double>(ns) / 1'000'000.0;
  };
  const auto old_flags = std::cout.flags();
  const auto old_precision = std::cout.precision();
  std::cout << " hdr=" << std::fixed << std::setprecision(2)
            << to_ms(bucket.min_ns) << "/"
            << (static_cast<double>(bucket.total_ns) /
                static_cast<double>(bucket.count) / 1'000'000.0)
            << "/" << to_ms(bucket.max_ns) << "ms";
  std::cout.flags(old_flags);
  std::cout.precision(old_precision);
}

void AppendGroupPairSkewLog(
    const a3_sync::A3SyncStatistics::LatencyBucket& bucket) {
  if (bucket.count == 0) return;
  const auto to_ms = [](std::int64_t ns) {
    return static_cast<double>(ns) / 1'000'000.0;
  };
  const auto old_flags = std::cout.flags();
  const auto old_precision = std::cout.precision();
  std::cout << " pair=" << std::fixed << std::setprecision(2)
            << to_ms(bucket.min_ns) << "/"
            << (static_cast<double>(bucket.total_ns) /
                static_cast<double>(bucket.count) / 1'000'000.0)
            << "/" << to_ms(bucket.max_ns) << "ms";
  std::cout.flags(old_flags);
  std::cout.precision(old_precision);
}

void AppendResampleDiagLog(
    const a3_sync::A3SyncStatistics::LatencySnapshot& latency) {
  const auto total = latency.resample_interpolated + latency.resample_held;
  if (total == 0) return;
  std::cout << " resample=i" << latency.resample_interpolated
            << "/h" << latency.resample_held;
}

void AppendRawLatestInputDiagLog(
    const a3_sync::A3SyncStatistics::Snapshot& current,
    bool include_offsets) {
  const std::array<bool, 6> stamp_valid{
      current.latest_waist_stamp_valid,
      current.latest_leg_stamp_valid,
      current.latest_arm_stamp_valid,
      current.latest_neck_stamp_valid,
      current.latest_pelvis_imu_stamp_valid,
      current.latest_torso_imu_stamp_valid,
  };
  const bool all_stamped =
      std::all_of(stamp_valid.begin(), stamp_valid.end(),
                  [](bool valid) { return valid; });
  if (all_stamped) {
    const std::array<std::int64_t, 6> stamps{
        current.latest_waist_stamp_ns,
        current.latest_leg_stamp_ns,
        current.latest_arm_stamp_ns,
        current.latest_neck_stamp_ns,
        current.latest_pelvis_imu_stamp_ns,
        current.latest_torso_imu_stamp_ns,
    };
    const auto [lo, hi] = std::minmax_element(stamps.begin(), stamps.end());
    const auto to_ms = [](std::int64_t ns) {
      return static_cast<double>(ns) / 1'000'000.0;
    };
    const auto base = *lo;
    const auto old_flags = std::cout.flags();
    const auto old_precision = std::cout.precision();
    std::cout << " raw_skew="
              << std::fixed << std::setprecision(2)
              << to_ms(*hi - *lo) << "ms";
    if (include_offsets) {
      std::cout << " raw_offsets={waist:"
                << to_ms(current.latest_waist_stamp_ns - base)
                << ",leg:" << to_ms(current.latest_leg_stamp_ns - base)
                << ",arm:" << to_ms(current.latest_arm_stamp_ns - base)
                << ",neck:" << to_ms(current.latest_neck_stamp_ns - base)
                << ",pelvis:" << to_ms(current.latest_pelvis_imu_stamp_ns - base)
                << ",torso:" << to_ms(current.latest_torso_imu_stamp_ns - base)
                << "}";
    }
    std::cout.flags(old_flags);
    std::cout.precision(old_precision);
  }
}

void HandleSigint(int /*signo*/) {
  RequestShutdown();
}

void InstallSigintHandler() {
  std::signal(SIGINT, HandleSigint);
  std::signal(SIGTERM, HandleSigint);
}

void WaitForShutdown() {
  std::unique_lock<std::mutex> lk(g_stop_mu);
  g_stop_cv.wait(lk, [] { return g_stop_requested.load(); });
}

class ScopedRawTerminal {
 public:
  explicit ScopedRawTerminal(int fd) : fd_(fd) {
    if (!isatty(fd_)) return;
    if (tcgetattr(fd_, &orig_) != 0) return;
    termios raw = orig_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    enabled_ = (tcsetattr(fd_, TCSANOW, &raw) == 0);
  }

  ~ScopedRawTerminal() {
    if (enabled_) tcsetattr(fd_, TCSANOW, &orig_);
  }

  bool enabled() const noexcept { return enabled_; }

 private:
  int fd_;
  bool enabled_ = false;
  termios orig_{};
};

std::thread StartKeyboardControlThread(
    ManualControlState& control,
    const std::vector<std::string>& motion_names,
    const std::vector<std::string>& remote_motion_names,
    ManualMotionShortcuts motion_shortcuts) {
  return std::thread(
      [&control, motion_names, remote_motion_names, motion_shortcuts]() {
    constexpr int kFd = STDIN_FILENO;
    if (!isatty(kFd)) {
      std::cerr << "[mode] WARN: stdin is not a TTY; manual mode will remain "
                   "idle/no-output unless changed by code\n";
      return;
    }

    ScopedRawTerminal term(kFd);
    if (!term.enabled()) {
      std::cerr << "[mode] WARN: failed to enter raw keyboard mode; startup "
                   "IDLE/no-output is still active\n";
      return;
    }

    auto read_with_timeout = [&](char* out, int timeout_us) {
      fd_set rfds;
      FD_ZERO(&rfds);
      FD_SET(kFd, &rfds);
      timeval tv{};
      tv.tv_sec = 0;
      tv.tv_usec = timeout_us;
      const int ready = select(kFd + 1, &rfds, nullptr, nullptr, &tv);
      if (ready <= 0 || !FD_ISSET(kFd, &rfds)) return false;
      return read(kFd, out, 1) > 0;
    };

    auto parse_key = [&](char ch) {
      if (ch != '\x1b') return ParseManualKey(ch);

      char seq0 = '\0';
      char seq1 = '\0';
      if (!read_with_timeout(&seq0, 20'000) ||
          (seq0 != '[' && seq0 != 'O') ||
          !read_with_timeout(&seq1, 20'000)) {
        return a3_deploy::ManualKey::kUnknown;
      }
      switch (seq1) {
        case 'A': return a3_deploy::ManualKey::kMotionForward;
        case 'B': return a3_deploy::ManualKey::kMotionBackward;
        case 'C': return a3_deploy::ManualKey::kMotionTurnRight;
        case 'D': return a3_deploy::ManualKey::kMotionTurnLeft;
        default: return a3_deploy::ManualKey::kUnknown;
      }
    };

    PrintMotionHelp(control, motion_names, remote_motion_names, std::cout,
                    &motion_shortcuts);
    while (!g_stop_requested.load(std::memory_order_acquire)) {
      fd_set rfds;
      FD_ZERO(&rfds);
      FD_SET(kFd, &rfds);
      timeval tv{};
      tv.tv_sec = 0;
      tv.tv_usec = 100'000;
      const int ready = select(kFd + 1, &rfds, nullptr, nullptr, &tv);
      if (ready < 0) {
        if (errno == EINTR) continue;
        break;
      }
      if (ready == 0 || !FD_ISSET(kFd, &rfds)) continue;

      char ch = '\0';
      const ssize_t n = read(kFd, &ch, 1);
      if (n <= 0) continue;
      const ManualKeyOutcome outcome =
          HandleManualKey(control, parse_key(ch), motion_names,
                          remote_motion_names, std::cout, &motion_shortcuts);
      if (outcome == ManualKeyOutcome::kQuit) {
        RequestShutdown();
      }
    }
  });
}

std::uint64_t CounterDelta(std::uint64_t current,
                           std::uint64_t previous) noexcept {
  return (current >= previous) ? (current - previous) : current;
}

struct SyncHealthLogState {
  bool valid{false};
  a3_sync::A3SyncStatistics::Snapshot previous{};
};

void AppendSyncHealthLog(robot_io::A3AimrtBackend* backend,
                         SyncHealthLogState& state,
                         bool latest_frame_mode) {
  if (!backend) return;

  const auto current = backend->SyncStatistics();
  const auto latency = backend->ConsumeLatencyStatistics();
  const auto& previous = state.previous;
  const auto ticks =
      state.valid ? CounterDelta(current.tick_total, previous.tick_total)
                  : current.tick_total;
  if (ticks == 0) {
    std::cout << " | sync=idle";
    state.previous = current;
    state.valid = true;
    return;
  }

  const auto complete =
      state.valid ? CounterDelta(current.frame_complete_total,
                                 previous.frame_complete_total)
                  : current.frame_complete_total;
  const auto aligned =
      state.valid ? CounterDelta(current.frame_aligned_total,
                                 previous.frame_aligned_total)
                  : current.frame_aligned_total;
  const auto missing_waist =
      state.valid ? CounterDelta(current.missing_waist, previous.missing_waist)
                  : current.missing_waist;
  const auto missing_leg =
      state.valid ? CounterDelta(current.missing_leg, previous.missing_leg)
                  : current.missing_leg;
  const auto missing_arm =
      state.valid ? CounterDelta(current.missing_arm, previous.missing_arm)
                  : current.missing_arm;
  const auto missing_neck =
      state.valid ? CounterDelta(current.missing_neck, previous.missing_neck)
                  : current.missing_neck;
  const auto missing_pelvis =
      state.valid ? CounterDelta(current.missing_pelvis_imu,
                                 previous.missing_pelvis_imu)
                  : current.missing_pelvis_imu;
  const auto missing_torso =
      state.valid ? CounterDelta(current.missing_torso_imu,
                                 previous.missing_torso_imu)
                  : current.missing_torso_imu;
  const auto stale_waist =
      state.valid ? CounterDelta(current.stale_waist, previous.stale_waist)
                  : current.stale_waist;
  const auto stale_leg =
      state.valid ? CounterDelta(current.stale_leg, previous.stale_leg)
                  : current.stale_leg;
  const auto stale_arm =
      state.valid ? CounterDelta(current.stale_arm, previous.stale_arm)
                  : current.stale_arm;
  const auto stale_neck =
      state.valid ? CounterDelta(current.stale_neck, previous.stale_neck)
                  : current.stale_neck;
  const auto stale_pelvis =
      state.valid ? CounterDelta(current.stale_pelvis_imu,
                                 previous.stale_pelvis_imu)
                  : current.stale_pelvis_imu;
  const auto stale_torso =
      state.valid ? CounterDelta(current.stale_torso_imu,
                                 previous.stale_torso_imu)
                  : current.stale_torso_imu;
  const auto missing_total = missing_waist + missing_leg + missing_arm +
                             missing_neck + missing_pelvis + missing_torso;
  const auto stale_total = stale_waist + stale_leg + stale_arm + stale_neck +
                           stale_pelvis + stale_torso;
  const bool sync_problem =
      missing_total > 0 || stale_total > 0 || complete < ticks ||
      aligned < ticks;
  const bool resample_problem = latency.resample_held > 0;

  const char* health = "ok";
  if (complete == 0 || aligned == 0) {
    health = "bad";
  } else if (sync_problem) {
    health = "warn";
  }

  const auto old_flags = std::cout.flags();
  const auto old_precision = std::cout.precision();
  std::cout << " | sync=" << health
            << " n=" << ticks
            << " dt=" << std::fixed << std::setprecision(2)
            << (static_cast<double>(current.last_tick_interval_ns) / 1e6)
            << "ms";
  if (latest_frame_mode) {
    std::cout << " skew=" << (static_cast<double>(current.last_skew_ns) / 1e6)
              << "ms";
  }
  std::cout.flags(old_flags);
  std::cout.precision(old_precision);
  if (sync_problem) {
    std::cout << " complete=" << complete << "/" << ticks
              << " aligned=" << aligned << "/" << ticks;
  }
  AppendHeaderSkewLog(latency.state_header_skew);
  if (!latest_frame_mode) {
    AppendGroupPairSkewLog(latency.group_pair_skew);
  }

  if (!sync_problem && (latest_frame_mode || !resample_problem)) {
    state.previous = current;
    state.valid = true;
    return;
  }

  std::cout << "\n         sync_diag";
  if (missing_total > 0) {
    std::cout << " missing={waist:" << missing_waist
              << ",leg:" << missing_leg
              << ",arm:" << missing_arm
              << ",neck:" << missing_neck
              << ",pelvis:" << missing_pelvis
              << ",torso:" << missing_torso << "}";
  }
  if (stale_total > 0) {
    std::cout << " stale={waist:" << stale_waist
              << ",leg:" << stale_leg
              << ",arm:" << stale_arm
              << ",neck:" << stale_neck
              << ",pelvis:" << stale_pelvis
              << ",torso:" << stale_torso << "}";
  }
  if (sync_problem) {
    const auto old_flags2 = std::cout.flags();
    const auto old_precision2 = std::cout.precision();
    auto age_ms = [](std::int64_t ns) {
      return ns < 0 ? -1.0 : static_cast<double>(ns) / 1e6;
    };
    std::cout << " age_ms={waist:" << std::fixed << std::setprecision(1)
              << age_ms(current.last_age_waist_ns)
              << ",leg:" << age_ms(current.last_age_leg_ns)
              << ",arm:" << age_ms(current.last_age_arm_ns)
              << ",neck:" << age_ms(current.last_age_neck_ns)
              << ",pelvis:" << age_ms(current.last_age_pelvis_imu_ns)
              << ",torso:" << age_ms(current.last_age_torso_imu_ns) << "}";
    std::cout.flags(old_flags2);
    std::cout.precision(old_precision2);
  }
  AppendRawLatestInputDiagLog(current, sync_problem);
  if (!latest_frame_mode) {
    AppendResampleDiagLog(latency);
  }

  state.previous = current;
  state.valid = true;
}

void WaitForShutdownWithFrameLog(const a3_deploy::A3PolicyDriver& driver,
                                 std::uint64_t warmup_ticks,
                                 std::uint64_t frame_log_interval,
                                 InferenceTimingStats& infer_timing,
                                 LatencyLogMode latency_log_mode,
                                 bool auto_start,
                                 const ManualControlState* manual_control,
                                 robot_io::A3AimrtBackend* sync_backend,
                                 bool latest_frame_mode) {
  if (frame_log_interval == 0) {
    WaitForShutdown();
    return;
  }

  std::uint64_t next_log_frame = frame_log_interval;
  SyncHealthLogState sync_log_state;

  std::unique_lock<std::mutex> lk(g_stop_mu);
  while (!g_stop_requested.load(std::memory_order_acquire)) {
    g_stop_cv.wait_for(lk, std::chrono::milliseconds(100), [] {
      return g_stop_requested.load(std::memory_order_acquire);
    });

    const auto policy_frames = driver.PolicyTickCount();
    const auto safe_halts = driver.SafeHaltCount();
    const auto total_frames = policy_frames + safe_halts;
    if (total_frames < next_log_frame) continue;

    std::cout << "[frames] n=" << total_frames;
    if (!auto_start && manual_control) {
      const DeployMode mode = LoadDeployMode(*manual_control);
      std::cout << " mode=" << DeployModeName(mode);
      if (mode == DeployMode::kPdStand) {
        std::cout << " pd="
                  << manual_control->pd_ticks.load(std::memory_order_acquire);
      } else if (mode == DeployMode::kMotion ||
                 mode == DeployMode::kTeleop) {
        std::cout << " motion="
                  << manual_control->motion_tick.load(std::memory_order_acquire);
        if (mode == DeployMode::kMotion) {
          if (manual_control->remote_motion_active.load(
                  std::memory_order_acquire)) {
            std::cout << " remote="
                      << (manual_control->selected_remote_motion_index.load(
                              std::memory_order_acquire) + 1)
                      << " normal_clip="
                      << (manual_control->selected_motion_index.load(
                              std::memory_order_acquire) + 1);
          } else {
            std::cout << " clip="
                      << (manual_control->selected_motion_index.load(
                              std::memory_order_acquire) + 1);
          }
          std::cout << " playing="
                    << (manual_control->motion_playing.load(
                            std::memory_order_acquire) ? 1 : 0);
        } else if (mode == DeployMode::kTeleop) {
          std::cout << " paused="
                    << (manual_control->teleop_input_paused.load(
                            std::memory_order_acquire) ? 1 : 0);
        }
        if (manual_control->motion_held.load(std::memory_order_acquire)) {
          std::cout << " hold=1";
        }
        if (manual_control->policy_yaw_offset_valid.load(
                std::memory_order_acquire)) {
          const double yaw_offset_deg =
              static_cast<double>(manual_control->policy_yaw_offset_mdeg.load(
                  std::memory_order_acquire)) / 1000.0;
          const auto old_flags = std::cout.flags();
          const auto old_precision = std::cout.precision();
          std::cout << " yaw=" << std::fixed << std::setprecision(1)
                    << yaw_offset_deg;
          std::cout.flags(old_flags);
          std::cout.precision(old_precision);
        }
      }
    } else {
      const auto motion_frames =
          (policy_frames > warmup_ticks) ? (policy_frames - warmup_ticks) : 0;
      const auto warmup_remaining =
          (policy_frames < warmup_ticks) ? (warmup_ticks - policy_frames) : 0;
      if (warmup_remaining > 0) {
        std::cout << " warmup_left=" << warmup_remaining;
      } else {
        std::cout << " motion=" << motion_frames;
      }
    }
    if (safe_halts > 0) {
      std::cout << " halts=" << safe_halts;
    }
    AppendSyncHealthLog(sync_backend, sync_log_state, latest_frame_mode);
    if (auto_start ||
        (manual_control &&
         (LoadDeployMode(*manual_control) == DeployMode::kMotion ||
          LoadDeployMode(*manual_control) == DeployMode::kTeleop))) {
      AppendInferenceTimingLog(infer_timing, !latest_frame_mode,
                               latency_log_mode);
    }
    std::cout << "\n";

    next_log_frame =
        ((total_frames / frame_log_interval) + 1) * frame_log_interval;
  }
}

}  // namespace

int main(int argc, char** argv) {
  std::string cli_error;
  if (!ValidateKnownFlags(argc, argv, &cli_error)) {
    std::cerr << cli_error << "\n";
    PrintUsage(argv[0]);
    return 64;  // EX_USAGE
  }

  const std::string cfg_path = ParseRuntimeCfgFlag(argc, argv);
  if (cfg_path.empty()) {
    PrintUsage(argv[0]);
    return 64;  // EX_USAGE
  }
  if (!std::filesystem::exists(cfg_path)) {
    std::cerr << "runtime-cfg file not found: " << cfg_path << "\n";
    return 66;  // EX_NOINPUT
  }
  std::error_code cfg_ec;
  const std::filesystem::path runtime_cfg_path =
      std::filesystem::weakly_canonical(cfg_path, cfg_ec);
  const std::filesystem::path runtime_cfg_lookup_path =
      cfg_ec ? std::filesystem::path(cfg_path) : runtime_cfg_path;

  YAML::Node cfg;
  try {
    cfg = YAML::LoadFile(cfg_path);
  } catch (const YAML::Exception& e) {
    std::cerr << "failed to parse runtime-cfg: " << e.what() << "\n";
    return 65;  // EX_DATAERR
  }

  std::cout << "a3_deploy_onnx_ref starting, runtime-cfg=" << cfg_path << "\n";

  const std::string aimrt_cfg_override = ResolveRuntimePathString(
      ParseStringFlag(argc, argv, "--aimrt-cfg", ""), runtime_cfg_lookup_path);
  const bool probe_mode = HasFlag(argc, argv, "--probe");
  const std::string default_probe_source = probe_mode ? "both" : "a3";
  const std::string probe_source = NormalizeConfigToken(
      ParseStringFlag(argc, argv, "--probe-source", default_probe_source));
  const bool auto_start = HasFlag(argc, argv, "--auto-start") || probe_mode;
  const bool dry_run = HasFlag(argc, argv, "--dry-run") ||
                       OptionalKey<bool>(cfg["backend"]["dry_run"], false);
  const bool policy_enabled =
      OptionalKey<bool>(cfg["policy_driver"]["policy_enabled"], true);
  if (probe_mode && dry_run) {
    std::cerr << "--probe cannot be used with --dry-run or "
                 "backend.dry_run=true; probe mode must load policy and run "
                 "inference\n";
    return 64;
  }
  if (probe_mode && !policy_enabled) {
    std::cerr << "--probe requires policy_driver.policy_enabled=true; probe "
                 "mode must load policy and run inference\n";
    return 64;
  }
  if (probe_source != "a3" && probe_source != "smpl" &&
      probe_source != "both") {
    std::cerr << "--probe-source must be one of: a3, smpl, both\n";
    return 64;
  }
  if (!probe_mode && probe_source != "a3") {
    std::cerr << "--probe-source is only valid with --probe\n";
    return 64;
  }
  const bool probe_source_smpl = probe_mode && probe_source == "smpl";
  const bool probe_source_both = probe_mode && probe_source == "both";
  const bool probe_needs_smpl_policy = probe_source_smpl || probe_source_both;
  const bool backend_only = dry_run || !policy_enabled;
  const bool force_publish_disabled = backend_only || probe_mode;

  std::uint64_t frame_log_interval =
      OptionalKey<std::uint64_t>(cfg["logging"]["frame_log_interval"], 0ULL);
  std::string frame_log_parse_error;
  if (!ParseUint64Flag(argc, argv, "--frame-log-interval",
                       &frame_log_interval, &frame_log_parse_error)) {
    std::cerr << frame_log_parse_error << "\n";
    return 64;
  }
  const LatencyLogMode latency_log_mode = ParseLatencyLogModeFromEnv();

  std::string onnx_path;
  std::string smpl_onnx_path;
  std::string a3_fast_onnx_path;
  std::string onnx_model_name;
  std::string encoder_model_path;
  std::string decoder_model_path;
  const std::string policy_backend = NormalizeConfigToken(
      OptionalKey<std::string>(cfg["onnx"]["backend"], std::string{"ort_cpu"}));
  const bool rknn_policy_backend =
      (policy_backend == "rknn" || policy_backend == "rk_npu" ||
       policy_backend == "rockchip_npu");
  const std::string onnx_mode = NormalizeConfigToken(
      OptionalKey<std::string>(cfg["onnx"]["mode"], std::string{"monolithic"}));
  const bool encoder_decoder_mode =
      (onnx_mode == "encoder_decoder" || onnx_mode == "encoderdecoder" ||
       onnx_mode == "split");
  if (!backend_only) {
    try {
      if (encoder_decoder_mode) {
        if (rknn_policy_backend) {
          throw std::runtime_error(
              "onnx.mode=encoder_decoder is not supported with "
              "onnx.backend=rknn");
        }
        encoder_model_path = RequiredKey<std::string>(
            cfg["onnx"]["encoder_model_path"], "onnx.encoder_model_path");
        decoder_model_path = RequiredKey<std::string>(
            cfg["onnx"]["decoder_model_path"], "onnx.decoder_model_path");
      } else if (rknn_policy_backend) {
        onnx_path = ResolveRuntimePathString(
            RequiredKey<std::string>(cfg["onnx"]["rknn_model_path"],
                                     "onnx.rknn_model_path"),
            runtime_cfg_lookup_path);
        smpl_onnx_path = ResolveRuntimePathString(
            OptionalKey<std::string>(cfg["onnx"]["smpl_rknn_model_path"],
                                     std::string{}),
            runtime_cfg_lookup_path);
        a3_fast_onnx_path = ResolveRuntimePathString(
            OptionalKey<std::string>(cfg["onnx"]["a3_fast_rknn_model_path"],
                                     std::string{}),
            runtime_cfg_lookup_path);
        if (probe_needs_smpl_policy && smpl_onnx_path.empty()) {
          throw std::runtime_error(
              "--probe-source smpl/both requires onnx.smpl_rknn_model_path");
        }
      } else {
        onnx_path = ResolveRuntimePathString(
            RequiredKey<std::string>(cfg["onnx"]["model_path"],
                                     "onnx.model_path"),
            runtime_cfg_lookup_path);
        smpl_onnx_path = ResolveRuntimePathString(
            OptionalKey<std::string>(cfg["onnx"]["smpl_model_path"],
                                     std::string{}),
            runtime_cfg_lookup_path);
        a3_fast_onnx_path = ResolveRuntimePathString(
            OptionalKey<std::string>(cfg["onnx"]["a3_fast_model_path"],
                                     std::string{}),
            runtime_cfg_lookup_path);
        if (probe_needs_smpl_policy && smpl_onnx_path.empty()) {
          throw std::runtime_error(
              "--probe-source smpl/both requires onnx.smpl_model_path");
        }
      }
    } catch (const std::exception& e) {
      std::cerr << "policy config error: " << e.what() << "\n";
      return 2;
    }
    if (encoder_decoder_mode) {
      encoder_model_path =
          ResolveRuntimePathString(encoder_model_path, runtime_cfg_lookup_path);
      decoder_model_path =
          ResolveRuntimePathString(decoder_model_path, runtime_cfg_lookup_path);
      onnx_model_name = ModelNameFromPath(encoder_model_path) + " + " +
                        ModelNameFromPath(decoder_model_path);
      std::cout << "✓ policy mode selected: encoder_decoder\n"
                << "  encoder=" << encoder_model_path << "\n"
                << "  decoder=" << decoder_model_path << "\n";
    } else {
      onnx_model_name = ModelNameFromPath(onnx_path);
      std::cout << "✓ policy mode selected: monolithic"
                << " backend=" << policy_backend << "\n"
                << "  a3=" << onnx_path << "\n";
      if (!smpl_onnx_path.empty()) {
        std::cout << "  smpl=" << smpl_onnx_path << "\n";
      }
      if (!a3_fast_onnx_path.empty()) {
        std::cout << "  a3_fast=" << a3_fast_onnx_path << "\n";
      }
      std::cout << "✓ policy model selected: name=" << onnx_model_name
                << "\n  path=" << onnx_path << "\n";
    }
  }

  const double configured_policy_hz =
      OptionalKey<double>(cfg["policy_driver"]["policy_hz"], 50.0);
  if (!a3_deploy::numeric_safety::IsFinite(configured_policy_hz) ||
      configured_policy_hz <= 0.0) {
    std::cerr << "invalid policy_driver.policy_hz="
              << configured_policy_hz << "\n";
    return 64;
  }

  // --- Backend -------------------------------------------------------------
  std::unique_ptr<robot_io::RobotIOBackend> backend_ptr;

  auto real = std::make_unique<robot_io::A3AimrtBackend>();
  try {
    if (aimrt_cfg_override.empty() && cfg["backend"]["aimrt_cfg_path"]) {
      cfg["backend"]["aimrt_cfg_path"] = ResolveRuntimePathString(
          cfg["backend"]["aimrt_cfg_path"].as<std::string>(),
          runtime_cfg_lookup_path);
    }
    const std::string backend_cfg =
        BuildBackendConfigString(cfg["backend"], aimrt_cfg_override,
                                 force_publish_disabled, configured_policy_hz);
    if (!real->Init(backend_cfg)) {
      std::cerr << "A3AimrtBackend::Init failed\n";
      return 1;
    }
  } catch (const std::exception& e) {
    std::cerr << "backend config error: " << e.what() << "\n";
    return 1;
  }
  backend_ptr = std::move(real);
  std::cout << "✓ A3AimrtBackend initialised\n";
  if (!aimrt_cfg_override.empty()) {
    std::cout << "✓ AimRT cfg override: " << aimrt_cfg_override << "\n";
  }

  auto& backend = *backend_ptr;

  // --- Teleop reference subscriber sink ----------------------------------
  const YAML::Node teleop_cfg = cfg["teleop"];
  const bool teleop_enabled =
      OptionalKey<bool>(teleop_cfg["enabled"], true);
  a3_deploy::A3TeleopReferenceOptions teleop_options;
  teleop_options.policy_hz = configured_policy_hz;
  teleop_options.future_frame_skip =
      OptionalKey<int>(
          teleop_cfg["future_frame_skip"],
          OptionalKey<int>(cfg["reference_motion"]["future_frame_skip"], 5));
  teleop_options.delay_ns = static_cast<std::int64_t>(
      OptionalKey<double>(teleop_cfg["delay_ms"], 900.0) * 1'000'000.0);
  teleop_options.max_frames =
      OptionalKey<std::size_t>(teleop_cfg["max_frames"], 512ULL);
  a3_deploy::A3TeleopReferenceBuffer teleop_reference(teleop_options);
  a3_deploy::A3TeleopReferenceOptions teleop_fast_options = teleop_options;
  teleop_fast_options.future_frame_skip =
      OptionalKey<int>(teleop_cfg["fast_future_frame_skip"], 1);
  const int teleop_fast_skip_for_delay =
      teleop_fast_options.future_frame_skip > 0
          ? teleop_fast_options.future_frame_skip
          : 1;
  const double default_fast_delay_ms =
      1000.0 * 9.0 * static_cast<double>(teleop_fast_skip_for_delay) /
      configured_policy_hz;
  teleop_fast_options.delay_ns = static_cast<std::int64_t>(
      OptionalKey<double>(teleop_cfg["fast_delay_ms"], default_fast_delay_ms) *
      1'000'000.0);
  a3_deploy::A3TeleopReferenceBuffer teleop_fast_reference(
      teleop_fast_options);
  const std::string teleop_topic = OptionalKey<std::string>(
      teleop_cfg["topic"], std::string{"/ta/whole_body_command"});
  const double teleop_stale_warn_ms =
      OptionalKey<double>(teleop_cfg["stale_warn_ms"], 250.0);
  const bool teleop_yaw_offset_compensation =
      OptionalKey<bool>(teleop_cfg["yaw_offset_compensation"], false);
  const double teleop_fallback_policy_blend =
      OptionalKey<double>(teleop_cfg["fallback_policy_blend"], 0.25);
  const double teleop_fallback_max_delta_rad =
      OptionalKey<double>(teleop_cfg["fallback_max_delta_rad"], 0.08);
  const bool teleop_fallback_use_pd_stand_gains =
      OptionalKey<bool>(teleop_cfg["fallback_use_pd_stand_gains"], true);
  std::atomic<bool> teleop_first_frame_logged{false};

  if (teleop_enabled) {
    if (auto* aimrt_backend =
            dynamic_cast<robot_io::A3AimrtBackend*>(backend_ptr.get())) {
      aimrt_backend->SetTeleopTopic(teleop_topic);
      aimrt_backend->SetTeleopFrameCallback(
          [&teleop_reference, &teleop_fast_reference,
           &teleop_first_frame_logged](
              const a3_deploy::A3TeleopFrame& frame) {
            if (!teleop_first_frame_logged.exchange(
                    true, std::memory_order_acq_rel)) {
              std::cerr << "[teleop] first whole_body_command received: "
                        << "stamp_ns=" << frame.stamp_ns << "\n";
            }
            teleop_reference.PushFrame(frame);
            teleop_fast_reference.PushFrame(frame);
          });
      std::cout << "✓ teleop reference enabled: topic=" << teleop_topic
                << " delay_ms="
                << (static_cast<double>(teleop_options.delay_ns) / 1e6)
                << " future_frame_skip="
                << teleop_options.future_frame_skip
                << " fast_delay_ms="
                << (static_cast<double>(teleop_fast_options.delay_ns) / 1e6)
                << " fast_future_frame_skip="
                << teleop_fast_options.future_frame_skip
                << "\n";
    } else {
      std::cerr << "[teleop] WARN: teleop enabled but backend is not "
                   "A3AimrtBackend; subscriber not configured\n";
    }
  }

  a3_deploy::A3ZmqSmplSource smpl_zmq_source;
  const YAML::Node smpl_zmq_cfg = cfg["smpl_zmq"];
  const bool smpl_zmq_requested =
      OptionalKey<bool>(smpl_zmq_cfg["enabled"], false);
  const bool smpl_policy_configured =
      encoder_decoder_mode || !smpl_onnx_path.empty();
  const bool smpl_zmq_enabled = smpl_zmq_requested && smpl_policy_configured;
  if (!backend_only && smpl_zmq_enabled) {
    a3_deploy::A3ZmqSmplSourceOptions smpl_options;
    smpl_options.enabled = true;
    smpl_options.host =
        OptionalKey<std::string>(smpl_zmq_cfg["host"], smpl_options.host);
    smpl_options.port =
        OptionalKey<int>(smpl_zmq_cfg["port"], smpl_options.port);
    smpl_options.topic =
        OptionalKey<std::string>(smpl_zmq_cfg["topic"], smpl_options.topic);
    smpl_options.conflate =
        OptionalKey<bool>(smpl_zmq_cfg["conflate"], smpl_options.conflate);
    smpl_options.verbose =
        OptionalKey<bool>(smpl_zmq_cfg["verbose"], smpl_options.verbose);
    smpl_options.joint_order = a3_deploy::ParseA3SmplJointOrder(
        OptionalKey<std::string>(smpl_zmq_cfg["joint_order"],
                                 std::string{"isaaclab"}));
    if (!smpl_zmq_source.Start(smpl_options)) {
      std::cerr << "A3ZmqSmplSource::Start failed\n";
      return 2;
    }
  } else if (!backend_only && smpl_zmq_requested) {
    std::cout << "[teleop] SMPL ZMQ source disabled; key 2 will be rejected"
              << " (missing monolithic "
              << (rknn_policy_backend ? "smpl_rknn_model_path"
                                      : "smpl_model_path")
              << ")\n";
  } else if (!backend_only && smpl_policy_configured) {
    std::cout << "[teleop] SMPL ZMQ source disabled; key 2 will be rejected\n";
  }

  if (backend_only) {
    InstallSigintHandler();
    std::cout << "✓ backend-only dry-run enabled"
              << (dry_run ? " (--dry-run/backend.dry_run)" : " (policy_enabled=false)")
              << "; policy is not loaded and commands are not published\n";
    if (!backend.Start()) {
      std::cerr << "Backend::Start failed\n";
      return 5;
    }
    std::cout << "✓ backend started; Ctrl-C to exit\n";
    WaitForShutdown();
    std::cout << "shutdown requested — stopping backend\n";
    backend.Stop();
    std::cout << "a3_deploy_onnx_ref backend-only exit cleanly\n";
    return 0;
  }

  if (probe_mode) {
    std::cout << "✓ probe mode enabled: receive/sync + policy inference "
                 "latency; command publishers disabled\n";
    if (probe_source_smpl) {
      std::cout << "✓ probe source: smpl monolithic policy "
                   "(synthetic zero SMPL tokenizer)\n";
    } else if (probe_source_both) {
      std::cout << "✓ probe source: a3 + smpl monolithic comparison "
                   "(synthetic zero SMPL tokenizer)\n";
    }
  }

  // --- Policy runtime -----------------------------------------------------
  a3_deploy::A3PolicyRuntimeOptions policy_runtime_options;
  policy_runtime_options.backend = policy_backend;
  policy_runtime_options.use_fp16 =
      OptionalKey<bool>(cfg["onnx"]["fp16"], false);
  policy_runtime_options.intra_op_num_threads =
      OptionalKey<int>(cfg["onnx"]["intra_op_num_threads"], 1);
  policy_runtime_options.inter_op_num_threads =
      OptionalKey<int>(cfg["onnx"]["inter_op_num_threads"], 1);
  policy_runtime_options.rknn_core_mask =
      OptionalKey<std::string>(cfg["onnx"]["rknn_core_mask"],
                               std::string{"auto"});

  std::unique_ptr<a3_deploy::A3PolicyRuntime> policy;
  std::unique_ptr<a3_deploy::A3PolicyRuntime> smpl_policy;
  std::unique_ptr<a3_deploy::A3PolicyRuntime> a3_fast_policy;
  std::unique_ptr<a3_deploy::A3EncoderDecoderRuntime> encoder_decoder_policy;
  const int a3_encoder_index = OptionalKey<int>(
      cfg["onnx"]["encoder_indices"]["a3"], 0);
  const int smpl_encoder_index = OptionalKey<int>(
      cfg["onnx"]["encoder_indices"]["smpl"], 1);
  const int token_transition_ticks = std::max(
      0, OptionalKey<int>(cfg["onnx"]["token_transition_ticks"], 15));

  if (encoder_decoder_mode) {
    a3_deploy::A3EncoderDecoderRuntimeOptions ed_options;
    ed_options.backend = policy_runtime_options.backend;
    ed_options.intra_op_num_threads =
        policy_runtime_options.intra_op_num_threads;
    ed_options.inter_op_num_threads =
        policy_runtime_options.inter_op_num_threads;
    encoder_decoder_policy =
        a3_deploy::CreateA3EncoderDecoderRuntime(ed_options);
    if (!encoder_decoder_policy) {
      std::cerr << "failed to create A3 encoder+decoder runtime backend="
                << policy_runtime_options.backend << "\n";
      return 2;
    }
    if (!encoder_decoder_policy->Initialize(encoder_model_path,
                                            decoder_model_path, ed_options)) {
      std::cerr << "A3EncoderDecoderRuntime::Initialize failed\n";
      return 2;
    }
    if (encoder_decoder_policy->GetEncoderInputDimension() !=
        a3_deploy::kA3EncoderInputTotalFloats) {
      std::cerr << "encoder input dim mismatch: expected "
                << a3_deploy::kA3EncoderInputTotalFloats << ", got "
                << encoder_decoder_policy->GetEncoderInputDimension() << "\n";
      return 2;
    }
    if (encoder_decoder_policy->GetTokenDimension() != a3_deploy::kA3TokenDim) {
      std::cerr << "encoder token dim mismatch: expected "
                << a3_deploy::kA3TokenDim << ", got "
                << encoder_decoder_policy->GetTokenDimension() << "\n";
      return 2;
    }
    if (encoder_decoder_policy->GetDecoderInputDimension() !=
        a3_deploy::kA3DecoderInputTotalFloats) {
      std::cerr << "decoder input dim mismatch: expected "
                << a3_deploy::kA3DecoderInputTotalFloats << ", got "
                << encoder_decoder_policy->GetDecoderInputDimension() << "\n";
      return 2;
    }
    if (encoder_decoder_policy->GetActionDimension() != 29) {
      std::cerr << "decoder output dim mismatch: expected 29, got "
                << encoder_decoder_policy->GetActionDimension() << "\n";
      return 2;
    }
    {
      std::fill_n(encoder_decoder_policy->MutableEncoderInputData(),
                  encoder_decoder_policy->GetEncoderInputDimension(), 0.0f);
      std::fill_n(encoder_decoder_policy->MutableDecoderInputData(),
                  encoder_decoder_policy->GetDecoderInputDimension(), 0.0f);
      if (!encoder_decoder_policy->Encode()) {
        std::cerr << "warmup encoder Infer failed\n";
        return 2;
      }
      if (!encoder_decoder_policy->Decode()) {
        std::cerr << "warmup decoder Infer failed\n";
        return 2;
      }
    }
    std::cout << "✓ A3 encoder+decoder runtime loaded: backend="
              << encoder_decoder_policy->BackendName()
              << " model_name=" << onnx_model_name
              << " a3_index=" << a3_encoder_index
              << " smpl_index=" << smpl_encoder_index
              << " token_transition_ticks=" << token_transition_ticks << "\n";
  } else {
    auto load_monolithic_policy =
        [&](const char* label, const std::string& model_path,
            std::size_t expected_input_dim)
        -> std::unique_ptr<a3_deploy::A3PolicyRuntime> {
      auto runtime = a3_deploy::CreateA3PolicyRuntime(policy_runtime_options);
      if (!runtime) {
        std::cerr << "failed to create A3 " << label
                  << " policy runtime backend="
                  << policy_runtime_options.backend << "\n";
        return nullptr;
      }
      if (!runtime->Initialize(model_path, policy_runtime_options)) {
        std::cerr << "A3PolicyRuntime::Initialize failed (" << label
                  << " model=" << model_path
                  << ", backend=" << policy_runtime_options.backend << ")\n";
        return nullptr;
      }
      if (runtime->GetInputDimension() != expected_input_dim) {
        std::cerr << label << " policy input dim mismatch: expected "
                  << expected_input_dim << ", got "
                  << runtime->GetInputDimension() << "\n";
        return nullptr;
      }
      if (runtime->GetActionDimension() != 29) {
        std::cerr << label << " policy output dim mismatch: expected 29, got "
                  << runtime->GetActionDimension() << "\n";
        return nullptr;
      }
      std::fill_n(runtime->MutableInputData(), runtime->GetInputDimension(),
                  0.0f);
      if (!runtime->Infer()) {
        std::cerr << label << " warmup Infer failed\n";
        return nullptr;
      }
      if (!runtime->CaptureGraph()) {
        std::cerr << label
                  << " CaptureGraph failed; continuing without graph caching\n";
      }
      std::cout << "✓ A3 monolithic " << label
                << " policy runtime loaded: backend="
                << runtime->BackendName()
                << " model_path=" << model_path
                << " input_dim=" << runtime->GetInputDimension() << "\n";
      return runtime;
    };

    policy = load_monolithic_policy("a3", onnx_path,
                                    a3_deploy::kA3ObsDictTotalFloats);
    if (!policy) return 2;
    const bool load_smpl_monolithic_policy =
        (smpl_zmq_enabled || probe_needs_smpl_policy) &&
        !smpl_onnx_path.empty();
    if (load_smpl_monolithic_policy) {
      smpl_policy = load_monolithic_policy(
          "smpl", smpl_onnx_path, a3_deploy::kA3SmplObsDictTotalFloats);
      if (!smpl_policy) return 2;
    }
    if (!a3_fast_onnx_path.empty()) {
      a3_fast_policy = load_monolithic_policy(
          "a3_fast", a3_fast_onnx_path, a3_deploy::kA3ObsDictTotalFloats);
      if (!a3_fast_policy) return 2;
    }
  }
  const bool smpl_policy_runtime_available =
      encoder_decoder_mode || static_cast<bool>(smpl_policy);
  const bool a3_fast_policy_runtime_available =
      !encoder_decoder_mode && static_cast<bool>(a3_fast_policy);
  if (probe_needs_smpl_policy && !smpl_policy_runtime_available) {
    std::cerr << "--probe-source smpl/both requested, but SMPL policy runtime is "
                 "not available\n";
    return 2;
  }
  if (probe_source_both && encoder_decoder_mode) {
    std::cerr << "--probe-source both is only supported in monolithic mode\n";
    return 64;
  }
  if (!backend_only && !a3_fast_policy_runtime_available) {
    if (encoder_decoder_mode) {
      std::cout << "[teleop] A3-fast source disabled; key 3 will be rejected"
                << " (monolithic a3_fast_model_path only)\n";
    } else {
      std::cout << "[teleop] A3-fast source disabled; key 3 will be rejected"
                << " (missing "
                << (rknn_policy_backend ? "onnx.a3_fast_rknn_model_path"
                                        : "onnx.a3_fast_model_path")
                << ")\n";
    }
  }

  // --- Reference motion library -------------------------------------------
  // Load one or more flat CSV motion files and build tokenizer[640] online
  // from the current robot root orientation.
  a3_deploy::A3MotionLibrary motion_library;
  {
    const auto& ref_cfg = cfg["reference_motion"];
    const a3_deploy::OnEndPolicy on_end = ParseOnEndPolicy(
        OptionalKey<std::string>(ref_cfg["on_end"], std::string{"hold_last"}));

    a3_deploy::A3MotionLibraryOptions motion_options;
    motion_options.motion_dir = ResolveRuntimePathString(
        OptionalKey<std::string>(ref_cfg["motion_dir"], std::string{}),
        runtime_cfg_lookup_path);
    motion_options.csv_path = ResolveRuntimePathString(
        OptionalKey<std::string>(ref_cfg["csv_path"], std::string{}),
        runtime_cfg_lookup_path);
    const int initial_index =
        OptionalKey<int>(ref_cfg["initial_index"], 0);
    if (initial_index < 0) {
      std::cerr << "reference_motion.initial_index must be >= 0\n";
      return 4;
    }
    motion_options.initial_index = static_cast<std::size_t>(initial_index);
    motion_options.reference_options.source_fps = OptionalKey<double>(
        ref_cfg["source_fps"],
        motion_options.reference_options.source_fps);
    motion_options.reference_options.target_fps = OptionalKey<double>(
        ref_cfg["target_fps"],
        motion_options.reference_options.target_fps);
    motion_options.reference_options.csv_frame_stride = OptionalKey<int>(
        ref_cfg["csv_frame_stride"],
        motion_options.reference_options.csv_frame_stride);
    motion_options.reference_options.future_frame_skip = OptionalKey<int>(
        ref_cfg["future_frame_skip"],
        motion_options.reference_options.future_frame_skip);
    motion_options.reference_options.on_end = on_end;

    if (!motion_library.Load(motion_options)) {
      std::cerr << "A3MotionLibrary::Load failed\n";
      return 4;
    }
    const auto& initial_clip = motion_library.Clip(motion_library.InitialIndex());
    std::cout << "✓ Reference motions: " << motion_library.Size()
              << " CSV online tokenizer clips, initial=["
              << (motion_library.InitialIndex() + 1) << "/"
              << motion_library.Size() << "] " << initial_clip.name
              << " (" << initial_clip.path.string()
              << "), yaw compensation enabled\n";
  }

  // --- Remote motion library ----------------------------------------------
  // Direction-key clips are intentionally kept out of the normal playback
  // list. They can be triggered while preserving selected_motion_index for
  // the regular motion player.
  a3_deploy::A3MotionLibrary remote_motion_library;
  {
    const auto& ref_cfg = cfg["reference_motion"];
    const a3_deploy::OnEndPolicy on_end = ParseOnEndPolicy(
        OptionalKey<std::string>(ref_cfg["on_end"], std::string{"hold_last"}));

    std::vector<std::string> remote_motion_dirs;
    auto append_remote_dir = [&](const YAML::Node& node) -> bool {
      if (!node || node.IsNull()) return true;
      remote_motion_dirs.push_back(
          ResolveRuntimePathString(node.as<std::string>(),
                                   runtime_cfg_lookup_path));
      return true;
    };
    if (!append_remote_dir(ref_cfg["remote_motion_dir"])) {
      return 4;
    }
    if (ref_cfg["remote_motion_dirs"]) {
      if (!ref_cfg["remote_motion_dirs"].IsSequence()) {
        std::cerr << "reference_motion.remote_motion_dirs must be a list\n";
        return 4;
      }
      for (const auto& node : ref_cfg["remote_motion_dirs"]) {
        if (!append_remote_dir(node)) {
          return 4;
        }
      }
    }
    if (ref_cfg["extra_motion_dirs"]) {
      if (!ref_cfg["extra_motion_dirs"].IsSequence()) {
        std::cerr << "reference_motion.extra_motion_dirs must be a list\n";
        return 4;
      }
      std::cout << "[remote] treating legacy reference_motion.extra_motion_dirs"
                   " as independent remote motion dirs\n";
      for (const auto& node : ref_cfg["extra_motion_dirs"]) {
        remote_motion_dirs.push_back(
            ResolveRuntimePathString(node.as<std::string>(),
                                     runtime_cfg_lookup_path));
      }
    }

    if (!remote_motion_dirs.empty()) {
      a3_deploy::A3MotionLibraryOptions remote_options;
      remote_options.motion_dir = remote_motion_dirs.front();
      for (std::size_t i = 1; i < remote_motion_dirs.size(); ++i) {
        remote_options.extra_motion_dirs.push_back(remote_motion_dirs[i]);
      }
      remote_options.reference_options.source_fps = OptionalKey<double>(
          ref_cfg["source_fps"],
          remote_options.reference_options.source_fps);
      remote_options.reference_options.target_fps = OptionalKey<double>(
          ref_cfg["target_fps"],
          remote_options.reference_options.target_fps);
      remote_options.reference_options.csv_frame_stride = OptionalKey<int>(
          ref_cfg["csv_frame_stride"],
          remote_options.reference_options.csv_frame_stride);
      remote_options.reference_options.future_frame_skip = OptionalKey<int>(
          ref_cfg["future_frame_skip"],
          remote_options.reference_options.future_frame_skip);
      remote_options.reference_options.on_end = on_end;
      if (!remote_motion_library.Load(remote_options)) {
        std::cerr << "Remote A3MotionLibrary::Load failed\n";
        return 4;
      }
      std::cout << "✓ Remote motions: " << remote_motion_library.Size()
                << " independent direction-key clips";
      for (const auto& dir : remote_motion_dirs) {
        std::cout << " (" << dir << ")";
      }
      std::cout << "\n";
    }
  }

  // --- Motion idle standing reference --------------------------------------
  // Manual MOTION mode uses this standing tokenizer before a clip is played
  // and after one-shot normal/remote clips finish. The selected normal motion
  // pointer remains independent of this idle target.
  a3_deploy::A3CsvMotionReference motion_idle_reference;
  bool motion_idle_reference_loaded = false;
  {
    const auto& ref_cfg = cfg["reference_motion"];
    const std::string idle_csv_raw = OptionalKey<std::string>(
        ref_cfg["idle_csv_path"], std::string{});
    if (!idle_csv_raw.empty()) {
      const std::string csv_path_str = ResolveRuntimePathString(
          idle_csv_raw, runtime_cfg_lookup_path);
      const std::filesystem::path csv_path(csv_path_str);
      if (!ValidateCsvPath(csv_path, "reference_motion.idle_csv_path")) {
        return 4;
      }

      a3_deploy::A3CsvMotionReferenceOptions ropt;
      ropt.source_fps = OptionalKey<double>(
          ref_cfg["source_fps"], ropt.source_fps);
      ropt.target_fps = OptionalKey<double>(
          ref_cfg["target_fps"], ropt.target_fps);
      ropt.csv_frame_stride = OptionalKey<int>(
          ref_cfg["csv_frame_stride"], ropt.csv_frame_stride);
      ropt.future_frame_skip = OptionalKey<int>(
          ref_cfg["future_frame_skip"], ropt.future_frame_skip);
      ropt.on_end = ParseOnEndPolicy(
          OptionalKey<std::string>(ref_cfg["idle_on_end"],
                                   std::string{"hold_last"}),
          "reference_motion.idle_on_end");
      if (!motion_idle_reference.Load(csv_path.string(), ropt)) {
        std::cerr << "motion idle A3CsvMotionReference::Load failed\n";
        return 4;
      }
      motion_idle_reference_loaded = true;
      std::cout << "✓ Motion idle reference: standing CSV online tokenizer ("
                << csv_path.string() << ")\n";
    }
  }

  // --- Teleop no-data fallback reference ----------------------------------
  // When TELEOP is entered before /ta/whole_body_command arrives, feed the
  // policy a real standing reference clip instead of a perfectly static
  // synthetic target. That keeps the tokenizer closer to the training
  // distribution while still using the normal ONNX path.
  a3_deploy::A3CsvMotionReference teleop_fallback_reference;
  bool teleop_fallback_reference_loaded = false;
  a3_deploy::A3CsvMotionReference teleop_fast_fallback_reference;
  bool teleop_fast_fallback_reference_loaded = false;
  {
    const auto& fb_cfg = teleop_cfg["fallback_reference"];
    const bool fb_enabled = teleop_enabled &&
        OptionalKey<bool>(fb_cfg["enabled"], false);
    if (fb_enabled) {
      const std::string csv_path_str = ResolveRuntimePathString(
          RequiredKey<std::string>(
              fb_cfg["csv_path"], "teleop.fallback_reference.csv_path"),
          runtime_cfg_lookup_path);
      const std::filesystem::path csv_path(csv_path_str);
      if (!ValidateCsvPath(csv_path,
                           "teleop.fallback_reference.csv_path")) {
        return 4;
      }

      auto load_teleop_fallback =
          [&](a3_deploy::A3CsvMotionReference& reference,
              bool& loaded,
              const char* label,
              const char* future_frame_skip_key,
              int future_frame_skip) -> bool {
        a3_deploy::A3CsvMotionReferenceOptions ropt;
        ropt.source_fps = OptionalKey<double>(
            fb_cfg["source_fps"],
            OptionalKey<double>(cfg["reference_motion"]["source_fps"],
                                ropt.source_fps));
        ropt.target_fps = OptionalKey<double>(
            fb_cfg["target_fps"], teleop_options.policy_hz);
        ropt.csv_frame_stride = OptionalKey<int>(
            fb_cfg["csv_frame_stride"],
            OptionalKey<int>(cfg["reference_motion"]["csv_frame_stride"],
                             ropt.csv_frame_stride));
        ropt.future_frame_skip = OptionalKey<int>(
            fb_cfg[future_frame_skip_key], future_frame_skip);
        ropt.on_end = ParseOnEndPolicy(
            OptionalKey<std::string>(fb_cfg["on_end"],
                                     std::string{"hold_last"}),
            "teleop.fallback_reference.on_end");
        if (!reference.Load(csv_path.string(), ropt)) {
          std::cerr << label << " A3CsvMotionReference::Load failed\n";
          return false;
        }
        loaded = true;
        std::cout << "✓ " << label << ": CSV online tokenizer ("
                  << csv_path.string() << ", future_frame_skip="
                  << ropt.future_frame_skip << ")\n";
        return true;
      };

      if (!load_teleop_fallback(teleop_fallback_reference,
                                teleop_fallback_reference_loaded,
                                "Teleop fallback reference",
                                "future_frame_skip",
                                teleop_options.future_frame_skip)) {
        return 4;
      }
      if (!load_teleop_fallback(teleop_fast_fallback_reference,
                                teleop_fast_fallback_reference_loaded,
                                "Teleop A3-fast fallback reference",
                                "fast_future_frame_skip",
                                teleop_fast_options.future_frame_skip)) {
        return 4;
      }
    }
  }

  // --- Obs builder --------------------------------------------------------
  a3_deploy::A3ObsBuilder obs_builder;

  // --- Dump file for diagnostics (writes per-tick data) -------------------
  FILE* dump_fp = nullptr;
  const int dump_ticks =
      OptionalKey<int>(cfg["logging"]["dump_ticks"], 0);
  const int debug_print_ticks =
      OptionalKey<int>(cfg["logging"]["debug_print_ticks"], 0);
  const double artificial_infer_delay_ms =
      OptionalKey<double>(cfg["logging"]["artificial_infer_delay_ms"], 0.0);
  const double artificial_infer_delay_jitter_ms = OptionalKey<double>(
      cfg["logging"]["artificial_infer_delay_jitter_ms"], 0.0);
  if (artificial_infer_delay_ms < 0.0 ||
      artificial_infer_delay_jitter_ms < 0.0) {
    std::cerr << "logging.artificial_infer_delay_ms and "
                 "logging.artificial_infer_delay_jitter_ms must be >= 0\n";
    return 4;
  }
  if (artificial_infer_delay_ms > 0.0 ||
      artificial_infer_delay_jitter_ms > 0.0) {
    std::cout << "[debug] artificial policy infer delay enabled: base_ms="
              << artificial_infer_delay_ms
              << " jitter_ms=[0," << artificial_infer_delay_jitter_ms
              << "]\n";
  }
  const std::string dump_path = OptionalKey<std::string>(
      cfg["logging"]["dump_path"], std::string{"/tmp/a3_deploy_dump.bin"});
  if (dump_ticks > 0) {
    dump_fp = std::fopen(dump_path.c_str(), "wb");
    if (dump_fp)
      std::cout << "[dump] dumping first " << dump_ticks
                << " policy ticks to " << dump_path << "\n";
    else
      std::cerr << "[dump] WARN: failed to open " << dump_path << "\n";
  }

  const std::uint64_t configured_warmup_ticks =
      OptionalKey<std::uint64_t>(cfg["policy_driver"]["warmup_ticks"],
                                 100ULL);
  const std::uint64_t pd_stand_ticks =
      OptionalKey<std::uint64_t>(cfg["policy_driver"]["pd_stand_ticks"],
                                 150ULL);
  ManualControlState manual_control;
  manual_control.selected_motion_index.store(
      static_cast<int>(motion_library.InitialIndex()),
      std::memory_order_release);
  const std::vector<std::string> motion_names = motion_library.Names();
  const std::vector<std::string> remote_motion_names =
      remote_motion_library.Names();
  ManualMotionShortcuts motion_shortcuts;
  {
    const auto shortcuts_cfg = cfg["reference_motion"]["keyboard_shortcuts"];
    motion_shortcuts.forward = ResolveMotionShortcutIndex(
        shortcuts_cfg["arrow_up"], remote_motion_names,
        "reference_motion.keyboard_shortcuts.arrow_up");
    motion_shortcuts.backward = ResolveMotionShortcutIndex(
        shortcuts_cfg["arrow_down"], remote_motion_names,
        "reference_motion.keyboard_shortcuts.arrow_down");
    motion_shortcuts.turn_left = ResolveMotionShortcutIndex(
        shortcuts_cfg["arrow_left"], remote_motion_names,
        "reference_motion.keyboard_shortcuts.arrow_left");
    motion_shortcuts.turn_right = ResolveMotionShortcutIndex(
        shortcuts_cfg["arrow_right"], remote_motion_names,
        "reference_motion.keyboard_shortcuts.arrow_right");
  }

  // --- Policy callback ----------------------------------------------------
  // RT-hot path. All buffers in this lambda and its captures must be
  // pre-allocated — A3PolicyDriver::RunOnce documents alloc-free contract.
  std::array<float, a3_deploy::kA3ObsDictTotalFloats> obs{};
  std::array<float, a3_deploy::kA3SmplObsDictTotalFloats> smpl_obs{};
  std::array<float, a3_deploy::kA3TokenizerFloatsPerTick> tokenizer_slice{};
  std::array<float, a3_deploy::kA3SmplTokenizerTotalFloats> smpl_tokenizer_slice{};
  std::array<float, a3_deploy::kA3SmplTokenizerTotalFloats> probe_smpl_tokenizer{};
  std::array<float, a3_deploy::kA3EncoderInputTotalFloats> encoder_input{};
  std::array<float, a3_deploy::kA3DecoderInputTotalFloats> decoder_input{};
  std::array<float, a3_deploy::kA3ProprioTotalFloats> proprio_history{};
  std::array<float, a3_deploy::kA3TokenDim> blended_token{};
  std::array<float, a3_deploy::kA3TokenDim> previous_token{};
  bool previous_token_valid = false;
  int token_transition_remaining = 0;
  std::array<float, 29> raw_action{};
  std::array<float, 29> source_switch_raw_action{};
  bool raw_action_valid = false;
  bool source_switch_raw_action_valid = false;
  int action_transition_remaining = 0;
  std::array<double, 29> jp_mujoco{};
  std::array<double, 29> jv_mujoco{};
  InferenceTimingStats infer_timing;
  std::mt19937 infer_delay_rng{0xA3D31A7u};
  std::uniform_real_distribution<double> infer_delay_jitter_dist(
      0.0, artificial_infer_delay_jitter_ms);
  double policy_yaw_offset_rad = 0.0;
  bool policy_yaw_offset_valid = false;
  double teleop_yaw_offset_rad = 0.0;
  bool teleop_yaw_offset_valid = false;
  double teleop_fallback_yaw_offset_rad = 0.0;
  bool teleop_fallback_yaw_offset_valid = false;
  double motion_idle_yaw_offset_rad = 0.0;
  bool motion_idle_yaw_offset_valid = false;
  std::uint64_t motion_idle_tick = 0;
  bool tokenizer_override_active = false;
  const float* tokenizer_override = nullptr;
  bool encoder_source_override_active = false;
  a3_deploy::A3EncoderSource encoder_source_override =
      a3_deploy::A3EncoderSource::kA3;
  const float* smpl_tokenizer_override = nullptr;
  bool remember_policy_action = true;
  std::size_t active_motion_index = motion_library.InitialIndex();
  std::size_t active_remote_motion_index = 0;
  bool active_motion_is_remote = false;
  const a3_deploy::A3CsvMotionReference* policy_reference =
      &motion_library.Reference(active_motion_index);

  auto active_motion_name = [&]() -> const std::string& {
    return active_motion_is_remote
               ? remote_motion_library.Clip(active_remote_motion_index).name
               : motion_library.Clip(active_motion_index).name;
  };

  auto capture_policy_yaw_offset =
      [&](const robot_io::RobotState& state,
          const char* reason) noexcept {
    policy_yaw_offset_rad = 0.0;
    policy_yaw_offset_valid = true;
    manual_control.policy_yaw_offset_valid.store(false,
                                                 std::memory_order_release);
    manual_control.policy_yaw_offset_mdeg.store(0, std::memory_order_release);

    const std::array<double, 4> quat_wxyz = {state.imu_quat_wxyz[0],
                                             state.imu_quat_wxyz[1],
                                             state.imu_quat_wxyz[2],
                                             state.imu_quat_wxyz[3]};
    policy_yaw_offset_rad =
        policy_reference ? policy_reference->ComputeYawOffsetRad(quat_wxyz)
                         : 0.0;
    const double yaw_offset_deg = policy_yaw_offset_rad * kRadToDeg;
    const int yaw_offset_mdeg = static_cast<int>(
        yaw_offset_deg * 1000.0 + (yaw_offset_deg >= 0.0 ? 0.5 : -0.5));
    manual_control.policy_yaw_offset_mdeg.store(yaw_offset_mdeg,
                                                std::memory_order_release);
    manual_control.policy_yaw_offset_valid.store(true,
                                                 std::memory_order_release);
    std::fprintf(stderr, "[policy] motion=%s yaw_offset_deg=%.3f (%s)\n",
                 active_motion_name().c_str(), yaw_offset_deg,
                 reason ? reason : "policy_start");
  };

  auto policy_fn = [&](std::uint64_t                 tick_idx,
                       const robot_io::RobotState&   state,
                       std::array<double, 29>&       q_des_29_out) noexcept {
    // ----- Warmup / stabilization phase --------------------------------------
    // For the configured warmup window after StartDriver, bypass the ONNX
    // policy entirely and PD-track the nominal standing pose
    // (a3_default_angles). command_fn expands this warmup command with the
    // same motion_control_a3 PD_STAND gains used by manual PD_STAND. Purpose:
    //   1. Model warmup runs before Start(), but transport startup and the
    //      first few synchronized frames can still arrive while the robot is
    //      settling. A deterministic PD warmup keeps it upright during that
    //      window.
    //   2. mujoco_sim starts every joint at qpos=0 (no keyframe applied —
    //      MJCF <keyframe> is not auto-loaded by `mj_makeData`). Without a
    //      warmup, the policy's very first obs is "robot flat on the ground,
    //      joints in a non-training-distribution pose" which ships it straight
    //      into the failure mode that ends in twitching.
    //   3. Matches the MC PD_STAND stage (aimrt_motion_control_a3
    //      pd_stand/default.yaml) that the production stack uses before
    //      handing off to the policy.
    //
    // During warmup we do NOT touch obs_builder at all. When the first
    // post-warmup tick calls PushProprioception, A3ObsBuilder's ticks_buffered_
    // == 0 branch fires and seeds all 10 history slots with that frame — the
    // same behaviour as IsaacLab's env.reset() (see
    // scripts/dump_a3_obs_groundtruth.py:199-204). The tokenizer index is
    // also re-based so clip time t=0 starts at the first policy tick, not at
    // StartDriver (otherwise the first 100 ticks of the motion are skipped).
    const std::uint64_t effective_warmup = configured_warmup_ticks;
    if (tick_idx < effective_warmup) {
      std::copy(a3_default_angles.begin(), a3_default_angles.end(),
                q_des_29_out.begin());
      return;
    }
    const std::uint64_t policy_tick = tick_idx - effective_warmup;

    // 1. Gather 29-DOF policy-view joint pos/vel out of the 31-DOF SDK
    //    state via kA3PolicyToSdkIdx (waist + L_arm + R_arm + L_leg + R_leg,
    //    skipping neck slots [3..4]). See notes/a3_dof_orderings.md.
    robot_io::ExtractPolicyView(state.q,  jp_mujoco);
    robot_io::ExtractPolicyView(state.dq, jv_mujoco);

    // 2. Extract gravity_dir + base_ang_vel from pelvis IMU.
    //    state.imu_quat_wxyz is already in WXYZ order (a3_sync_loop converts
    //    ROS XYZW→WXYZ); GetGravityOrientation_d expects WXYZ.
    const std::array<double, 4> quat_wxyz = {state.imu_quat_wxyz[0],
                                             state.imu_quat_wxyz[1],
                                             state.imu_quat_wxyz[2],
                                             state.imu_quat_wxyz[3]};
    const auto gdir = GetGravityOrientation_d(quat_wxyz);
    const std::array<double, 3> bav = {state.imu_gyro[0], state.imu_gyro[1],
                                       state.imu_gyro[2]};

    // 3. Push proprioception (with the PREVIOUS tick's raw action in the
    //    actions slot — zero on the first post-warmup tick because
    //    obs_builder was left untouched during warmup).
    const auto prev_action_mujoco = obs_builder.LastAction();
    obs_builder.PushProprioception(gdir, bav, jp_mujoco, jv_mujoco,
                                   prev_action_mujoco);

    // 4. Fetch/build tokenizer slice for this (warmup-rebased) tick.
    const float* tok = nullptr;
    if (tokenizer_override_active && tokenizer_override != nullptr) {
      tok = tokenizer_override;
    } else {
      const double yaw_offset =
          policy_yaw_offset_valid ? policy_yaw_offset_rad : 0.0;
      if (policy_reference &&
          policy_reference->BuildTokenizerSlice(policy_tick, quat_wxyz,
                                                yaw_offset, tokenizer_slice)) {
        tok = tokenizer_slice.data();
      }
    }

    // 5. Build model input and run inference.
    const auto infer_start_system_ns = NowSystemNs();
    const auto infer_start = std::chrono::steady_clock::now();
    bool infer_ok = false;
    a3_deploy::A3EncoderSource source =
        encoder_source_override_active ? encoder_source_override
                                       : a3_deploy::A3EncoderSource::kA3;
    const float* active_smpl_tokenizer = smpl_tokenizer_override;
    if (probe_source_smpl && !encoder_source_override_active) {
      source = a3_deploy::A3EncoderSource::kSmpl;
      active_smpl_tokenizer = probe_smpl_tokenizer.data();
    }
    const bool use_smpl_source =
        source == a3_deploy::A3EncoderSource::kSmpl &&
        active_smpl_tokenizer != nullptr;
    const bool use_a3_fast_source =
        source == a3_deploy::A3EncoderSource::kA3Fast && a3_fast_policy;
    const bool compare_probe_sources =
        probe_source_both && !encoder_source_override_active &&
        !encoder_decoder_mode && smpl_policy;
    bool comparison_timing_valid = false;
    std::int64_t comparison_a3_infer_ns = 0;
    std::int64_t comparison_smpl_infer_ns = 0;
    if (encoder_decoder_mode) {
      obs_builder.BuildProprioHistory(proprio_history);
      a3_deploy::BuildA3EncoderInput(
          use_smpl_source ? smpl_encoder_index : a3_encoder_index,
          use_smpl_source ? nullptr : tok,
          use_smpl_source ? active_smpl_tokenizer : nullptr,
          encoder_input);
      float* encoder_buf = encoder_decoder_policy->MutableEncoderInputData();
      for (std::size_t i = 0; i < encoder_input.size(); ++i) {
        encoder_buf[i] = encoder_input[i];
      }
      infer_ok = encoder_decoder_policy->Encode();
      if (infer_ok) {
        const float* token = encoder_decoder_policy->EncodedTokenData();
        if (token_transition_remaining > 0 && previous_token_valid) {
          const int total = std::max(1, token_transition_ticks);
          const float alpha = 1.0f - static_cast<float>(token_transition_remaining) /
                                         static_cast<float>(total);
          for (std::size_t i = 0; i < blended_token.size(); ++i) {
            blended_token[i] =
                (1.0f - alpha) * previous_token[i] + alpha * token[i];
          }
          --token_transition_remaining;
        } else {
          for (std::size_t i = 0; i < blended_token.size(); ++i) {
            blended_token[i] = token[i];
          }
          token_transition_remaining = 0;
        }
        a3_deploy::BuildA3DecoderInput(blended_token.data(), proprio_history,
                                       decoder_input);
        float* decoder_buf = encoder_decoder_policy->MutableDecoderInputData();
        for (std::size_t i = 0; i < decoder_input.size(); ++i) {
          decoder_buf[i] = decoder_input[i];
        }
        infer_ok = encoder_decoder_policy->Decode();
        previous_token = blended_token;
        previous_token_valid = true;
      }
      obs_builder.BuildObsDict(tok, obs);
    } else {
      if (compare_probe_sources) {
        obs_builder.BuildObsDict(tok, obs);
        float* a3_in_buf = policy->MutableInputData();
        for (std::size_t i = 0; i < obs.size(); ++i) a3_in_buf[i] = obs[i];
        const auto a3_start = std::chrono::steady_clock::now();
        const bool a3_ok = policy->Infer();
        const auto a3_end = std::chrono::steady_clock::now();

        obs_builder.BuildSmplObsDict(probe_smpl_tokenizer.data(), smpl_obs);
        float* smpl_in_buf = smpl_policy->MutableInputData();
        for (std::size_t i = 0; i < smpl_obs.size(); ++i) {
          smpl_in_buf[i] = smpl_obs[i];
        }
        const auto smpl_start = std::chrono::steady_clock::now();
        const bool smpl_ok = smpl_policy->Infer();
        const auto smpl_end = std::chrono::steady_clock::now();

        comparison_a3_infer_ns = static_cast<std::int64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                a3_end - a3_start).count());
        comparison_smpl_infer_ns = static_cast<std::int64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                smpl_end - smpl_start).count());
        comparison_timing_valid = true;
        infer_ok = a3_ok && smpl_ok;
      } else {
        a3_deploy::A3PolicyRuntime* active_policy = policy.get();
        if (use_smpl_source && smpl_policy) {
          obs_builder.BuildSmplObsDict(active_smpl_tokenizer, smpl_obs);
          active_policy = smpl_policy.get();
          float* in_buf = active_policy->MutableInputData();
          for (std::size_t i = 0; i < smpl_obs.size(); ++i) {
            in_buf[i] = smpl_obs[i];
          }
        } else if (use_a3_fast_source) {
          obs_builder.BuildObsDict(tok, obs);
          active_policy = a3_fast_policy.get();
          float* in_buf = active_policy->MutableInputData();
          for (std::size_t i = 0; i < obs.size(); ++i) in_buf[i] = obs[i];
        } else {
          obs_builder.BuildObsDict(tok, obs);
          float* in_buf = active_policy->MutableInputData();
          for (std::size_t i = 0; i < obs.size(); ++i) in_buf[i] = obs[i];
        }
        infer_ok = active_policy->Infer();
      }
    }
    if (artificial_infer_delay_ms > 0.0 ||
        artificial_infer_delay_jitter_ms > 0.0) {
      double delay_ms = artificial_infer_delay_ms;
      if (artificial_infer_delay_jitter_ms > 0.0) {
        delay_ms += infer_delay_jitter_dist(infer_delay_rng);
      }
      if (delay_ms > 0.0) {
        std::this_thread::sleep_for(
            std::chrono::duration<double, std::milli>(delay_ms));
      }
    }
    const auto infer_end = std::chrono::steady_clock::now();
    if (comparison_timing_valid) {
      RecordProbeComparisonTimingSample(infer_timing, state,
                                        infer_start_system_ns,
                                        comparison_a3_infer_ns,
                                        comparison_smpl_infer_ns);
    } else {
      RecordPolicyTimingSample(
          infer_timing,
          state,
          infer_start_system_ns,
          static_cast<std::int64_t>(
              std::chrono::duration_cast<std::chrono::nanoseconds>(
                  infer_end - infer_start).count()));
    }
    if (!infer_ok) {
      // Inference failure — hold the current policy-view pose rather than
      // snapping all body joints to zero with nonzero gains.
      std::copy(jp_mujoco.begin(), jp_mujoco.end(), q_des_29_out.begin());
      return;
    }
    const float* act_buf = nullptr;
    if (encoder_decoder_mode) {
      act_buf = encoder_decoder_policy->ActionData();
    } else if (use_smpl_source && smpl_policy) {
      act_buf = smpl_policy->ActionData();
    } else if (use_a3_fast_source && a3_fast_policy) {
      act_buf = a3_fast_policy->ActionData();
    } else {
      act_buf = policy->ActionData();
    }
    for (int i = 0; i < 29; ++i) raw_action[i] = act_buf[i];
    if (!encoder_decoder_mode && action_transition_remaining > 0 &&
        source_switch_raw_action_valid) {
      const int total = std::max(1, token_transition_ticks);
      const float alpha =
          1.0f - static_cast<float>(action_transition_remaining) /
                     static_cast<float>(total);
      for (int i = 0; i < 29; ++i) {
        raw_action[i] = (1.0f - alpha) * source_switch_raw_action[i] +
                        alpha * raw_action[i];
      }
      --action_transition_remaining;
      if (action_transition_remaining <= 0) {
        source_switch_raw_action_valid = false;
      }
    }

    // 6. Decode + stash raw action for next tick's proprio.
    a3_deploy::DecodeAction(raw_action, q_des_29_out);
    raw_action_valid = true;
    if (remember_policy_action) {
      obs_builder.RememberAction(raw_action);
    }

    // --- TEMP DEBUG (first 3 post-warmup ticks) ----------------------------
    // Dumps state, obs (per-term oldest & newest history frames), raw policy
    // output, decoded q_des. Proprio layout inside obs (indices relative to
    // obs[640] base):
    //   [  0.. 29] base_ang_vel  (10×3 step-first, oldest→newest)
    //   [ 30..319] joint_pos     (10×29)
    //   [320..609] joint_vel     (10×29)
    //   [610..899] actions       (10×29)
    //   [900..929] gravity_dir   (10×3)
    // => absolute offsets below. Remove after diagnosis.
    if (debug_print_ticks > 0 &&
        policy_tick < static_cast<std::uint64_t>(debug_print_ticks)) {
      std::fprintf(stderr,
          "[PTICK %llu / DTICK %llu] state.q[0..4]={%.4f,%.4f,%.4f,%.4f,%.4f}\n",
          static_cast<unsigned long long>(policy_tick),
          static_cast<unsigned long long>(tick_idx),
          state.q[0], state.q[1], state.q[2], state.q[3], state.q[4]);
      std::fprintf(stderr,
          "[PTICK %llu] state.imu_quat_wxyz={%.4f,%.4f,%.4f,%.4f}  "
          "state.imu_gyro={%.4f,%.4f,%.4f}\n",
          static_cast<unsigned long long>(policy_tick),
          state.imu_quat_wxyz[0], state.imu_quat_wxyz[1],
          state.imu_quat_wxyz[2], state.imu_quat_wxyz[3],
          state.imu_gyro[0], state.imu_gyro[1], state.imu_gyro[2]);
      std::fprintf(stderr,
          "[PTICK %llu] gdir={%.4f,%.4f,%.4f}  bav={%.4f,%.4f,%.4f}\n",
          static_cast<unsigned long long>(policy_tick),
          gdir[0], gdir[1], gdir[2], bav[0], bav[1], bav[2]);
      std::fprintf(stderr,
          "[PTICK %llu] obs tokenizer[0..3]={%.4f,%.4f,%.4f,%.4f}\n",
          static_cast<unsigned long long>(policy_tick),
          obs[0], obs[1], obs[2], obs[3]);
      std::fprintf(stderr,
          "[PTICK %llu] base_ang_vel step0={%.4f,%.4f,%.4f}  step9={%.4f,%.4f,%.4f}\n",
          static_cast<unsigned long long>(policy_tick),
          obs[640], obs[641], obs[642], obs[667], obs[668], obs[669]);
      std::fprintf(stderr,
          "[PTICK %llu] joint_pos step0[0..2]={%.4f,%.4f,%.4f}  step9[0..2]={%.4f,%.4f,%.4f}\n",
          static_cast<unsigned long long>(policy_tick),
          obs[670], obs[671], obs[672], obs[931], obs[932], obs[933]);
      std::fprintf(stderr,
          "[PTICK %llu] actions step0[0..2]={%.4f,%.4f,%.4f}  step9[0..2]={%.4f,%.4f,%.4f}\n",
          static_cast<unsigned long long>(policy_tick),
          obs[1250], obs[1251], obs[1252], obs[1511], obs[1512], obs[1513]);
      std::fprintf(stderr,
          "[PTICK %llu] gravity_dir step0={%.4f,%.4f,%.4f}  step9={%.4f,%.4f,%.4f}\n",
          static_cast<unsigned long long>(policy_tick),
          obs[1540], obs[1541], obs[1542], obs[1567], obs[1568], obs[1569]);
      std::fprintf(stderr,
          "[PTICK %llu] raw_action[0..6]={%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f}\n",
          static_cast<unsigned long long>(policy_tick),
          raw_action[0], raw_action[1], raw_action[2], raw_action[3],
          raw_action[4], raw_action[5], raw_action[6]);
      std::fprintf(stderr,
          "[PTICK %llu] q_des[0..6]={%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f}\n",
          static_cast<unsigned long long>(policy_tick),
          q_des_29_out[0], q_des_29_out[1], q_des_29_out[2],
          q_des_29_out[3], q_des_29_out[4], q_des_29_out[5], q_des_29_out[6]);
      std::fflush(stderr);
    }

    // Binary dump for comparison with IsaacLab ground truth.
    // Per-tick layout: obs[1570 f32] + raw_action[29 f32] + q_des[29 f64]
    //                + jp_mujoco[29 f64] + jv_mujoco[29 f64]
    //                + gdir[3 f64] + bav[3 f64]
    if (dump_fp && policy_tick < static_cast<std::uint64_t>(dump_ticks)) {
      std::fwrite(obs.data(), sizeof(float), 1570, dump_fp);
      std::fwrite(raw_action.data(), sizeof(float), 29, dump_fp);
      std::fwrite(q_des_29_out.data(), sizeof(double), 29, dump_fp);
      std::fwrite(jp_mujoco.data(), sizeof(double), 29, dump_fp);
      std::fwrite(jv_mujoco.data(), sizeof(double), 29, dump_fp);
      std::fwrite(gdir.data(), sizeof(double), 3, dump_fp);
      std::fwrite(bav.data(), sizeof(double), 3, dump_fp);
      std::fflush(dump_fp);
      if (policy_tick == static_cast<std::uint64_t>(dump_ticks - 1)) {
        std::fclose(dump_fp);
        dump_fp = nullptr;
        std::fprintf(stderr, "[dump] complete: %s\n", dump_path.c_str());
      }
    }
  };

  std::array<double, 29> pd_start_q{};
  std::array<double, 29> command_q_des{};
  std::uint64_t mode_entry_tick = 0;
  std::uint64_t observed_epoch =
      manual_control.epoch.load(std::memory_order_acquire);
  std::uint64_t observed_motion_command_epoch =
      manual_control.motion_command_epoch.load(std::memory_order_acquire);
  std::uint64_t observed_teleop_command_epoch =
      manual_control.teleop_command_epoch.load(std::memory_order_acquire);
  DeployMode observed_mode = LoadDeployMode(manual_control);
  std::uint64_t active_motion_tick = 0;
  bool active_motion_playing =
      manual_control.motion_playing.load(std::memory_order_acquire);
  bool observed_teleop_input_paused =
      manual_control.teleop_input_paused.load(std::memory_order_acquire);
  bool pd_stand_initialized = false;
  bool teleop_running_started = false;
  bool teleop_hold_valid = false;
  std::array<float, a3_deploy::kA3TokenizerFloatsPerTick> teleop_hold_slice{};
  bool teleop_stream_stale = false;
  bool teleop_stale_warned = false;
  std::int64_t teleop_stale_latest_stamp_ns = 0;
  double smpl_yaw_offset_rad = 0.0;
  bool smpl_yaw_offset_valid = false;
  bool smpl_running_started = false;
  bool smpl_stream_stale = false;
  bool smpl_stale_warned = false;
  std::int64_t smpl_stale_latest_update_ns = 0;
  bool teleop_wait_log_valid = false;
  bool teleop_wait_log_using_fallback_reference = false;
  bool teleop_wait_log_enabled = true;
  std::string teleop_wait_log_reason;
  a3_deploy::A3TeleopTokenizerStatus teleop_wait_log_status =
      a3_deploy::A3TeleopTokenizerStatus::kNoData;
  a3_deploy::A3EncoderSource active_teleop_source =
      a3_deploy::A3EncoderSource::kA3;
  bool smpl_hold_valid = false;
  std::array<float, a3_deploy::kA3SmplTokenizerTotalFloats> smpl_hold_slice{};

  auto teleop_source_from_value = [](int value) noexcept {
    switch (value) {
      case 1: return a3_deploy::A3EncoderSource::kSmpl;
      case 2: return a3_deploy::A3EncoderSource::kA3Fast;
      default: return a3_deploy::A3EncoderSource::kA3;
    }
  };

  auto teleop_source_value = [](a3_deploy::A3EncoderSource source) noexcept {
    switch (source) {
      case a3_deploy::A3EncoderSource::kSmpl: return 1;
      case a3_deploy::A3EncoderSource::kA3Fast: return 2;
      case a3_deploy::A3EncoderSource::kA3: return 0;
    }
    return 0;
  };

  auto teleop_source_available =
      [&](a3_deploy::A3EncoderSource source) noexcept {
    switch (source) {
      case a3_deploy::A3EncoderSource::kA3:
        return true;
      case a3_deploy::A3EncoderSource::kSmpl:
        return smpl_zmq_enabled && smpl_policy_runtime_available;
      case a3_deploy::A3EncoderSource::kA3Fast:
        return a3_fast_policy_runtime_available;
    }
    return false;
  };

  auto load_selected_motion_index = [&]() noexcept {
    int requested =
        manual_control.selected_motion_index.load(std::memory_order_acquire);
    if (requested < 0) requested = 0;
    const int max_index = static_cast<int>(motion_library.Size()) - 1;
    if (requested > max_index) requested = max_index;
    manual_control.selected_motion_index.store(requested,
                                               std::memory_order_release);
    return static_cast<std::size_t>(requested);
  };

  auto load_selected_remote_motion_index = [&]() noexcept {
    int requested = manual_control.selected_remote_motion_index.load(
        std::memory_order_acquire);
    if (requested < 0) requested = 0;
    const int max_index =
        remote_motion_library.Empty()
            ? 0
            : static_cast<int>(remote_motion_library.Size()) - 1;
    if (requested > max_index) requested = max_index;
    manual_control.selected_remote_motion_index.store(
        requested, std::memory_order_release);
    return static_cast<std::size_t>(requested);
  };

  auto activate_normal_motion =
      [&](const robot_io::RobotState& state,
          const char* reason,
          const char* log_state) {
    (void)state;
    (void)reason;
    active_motion_is_remote = false;
    active_motion_index = load_selected_motion_index();
    policy_reference = &motion_library.Reference(active_motion_index);
    active_motion_tick = 0;
    motion_idle_tick = 0;
    motion_idle_yaw_offset_valid = false;
    obs_builder.Reset();
    policy_yaw_offset_valid = false;
    manual_control.policy_yaw_offset_valid.store(false,
                                                 std::memory_order_release);
    std::fprintf(stderr, "[motion] selected [%zu/%zu] %s (%s)\n",
                 active_motion_index + 1, motion_library.Size(),
                 active_motion_name().c_str(), log_state);
  };

  auto activate_remote_motion =
      [&](const robot_io::RobotState& state,
          const char* reason,
          const char* log_state) {
    (void)state;
    (void)reason;
    active_motion_is_remote = true;
    active_remote_motion_index = load_selected_remote_motion_index();
    policy_reference =
        &remote_motion_library.Reference(active_remote_motion_index);
    active_motion_tick = 0;
    motion_idle_tick = 0;
    motion_idle_yaw_offset_valid = false;
    obs_builder.Reset();
    policy_yaw_offset_valid = false;
    manual_control.policy_yaw_offset_valid.store(false,
                                                 std::memory_order_release);
    std::fprintf(stderr,
                 "[remote] selected [%zu/%zu] %s (%s); normal pointer=[%d/%zu]\n",
                 active_remote_motion_index + 1, remote_motion_library.Size(),
                 active_motion_name().c_str(), log_state,
                 manual_control.selected_motion_index.load(
                     std::memory_order_acquire) + 1,
                 motion_library.Size());
  };

  auto clear_teleop_wait_log = [&]() {
    teleop_wait_log_valid = false;
    teleop_wait_log_reason.clear();
  };

  auto reset_teleop_source_runtime =
      [&](a3_deploy::A3EncoderSource source) {
    teleop_running_started = false;
    teleop_stream_stale = false;
    teleop_stale_warned = false;
    teleop_stale_latest_stamp_ns = 0;
    smpl_running_started = false;
    smpl_stream_stale = false;
    smpl_stale_warned = false;
    smpl_stale_latest_update_ns = 0;
    teleop_fallback_yaw_offset_valid = false;
    clear_teleop_wait_log();
    if (source == a3_deploy::A3EncoderSource::kSmpl) {
      smpl_hold_valid = false;
      smpl_yaw_offset_valid = false;
      smpl_zmq_source.ResetPlaybackToLatestWindow();
    } else {
      teleop_hold_valid = false;
      teleop_yaw_offset_valid = false;
    }
    obs_builder.Reset();
    token_transition_remaining = token_transition_ticks;
    if (!encoder_decoder_mode && raw_action_valid &&
        token_transition_ticks > 0) {
      source_switch_raw_action = raw_action;
      source_switch_raw_action_valid = true;
      action_transition_remaining = token_transition_ticks;
    } else {
      action_transition_remaining = 0;
    }
  };

  auto reset_all_teleop_runtime = [&]() {
    teleop_running_started = false;
    teleop_hold_valid = false;
    teleop_stream_stale = false;
    teleop_stale_warned = false;
    teleop_stale_latest_stamp_ns = 0;
    teleop_yaw_offset_valid = false;
    smpl_running_started = false;
    smpl_hold_valid = false;
    smpl_stream_stale = false;
    smpl_stale_warned = false;
    smpl_stale_latest_update_ns = 0;
    smpl_yaw_offset_valid = false;
    teleop_fallback_yaw_offset_valid = false;
    clear_teleop_wait_log();
  };

  auto reset_motion_idle_runtime = [&]() noexcept {
    motion_idle_tick = 0;
    motion_idle_yaw_offset_valid = false;
  };

  auto build_motion_idle_stand =
      [&](std::uint64_t idle_tick,
          const std::array<double, 4>& quat_wxyz,
          std::array<float, a3_deploy::kA3TokenizerFloatsPerTick>& out)
          -> bool {
    bool using_idle_reference = false;
    if (motion_idle_reference_loaded) {
      if (!motion_idle_yaw_offset_valid) {
        motion_idle_yaw_offset_rad =
            motion_idle_reference.ComputeYawOffsetRad(quat_wxyz);
        motion_idle_yaw_offset_valid = true;
        std::fprintf(stderr,
                     "[motion] idle_yaw_offset_deg=%.3f\n",
                     motion_idle_yaw_offset_rad * kRadToDeg);
      }
      using_idle_reference =
          motion_idle_reference.BuildTokenizerSlice(
              idle_tick, quat_wxyz, motion_idle_yaw_offset_rad, out);
    }
    if (!using_idle_reference) {
      a3_deploy::BuildDefaultStandTokenizerSlice(quat_wxyz, out);
    }
    return using_idle_reference;
  };

  auto build_teleop_stand_fallback =
      [&](std::uint64_t teleop_tick,
          const std::array<double, 4>& quat_wxyz,
          a3_deploy::A3TeleopTokenizerStatus status,
          const char* reason,
          bool source_enabled,
          a3_deploy::A3EncoderSource source,
          std::array<float, a3_deploy::kA3TokenizerFloatsPerTick>& out)
          -> bool {
    bool using_fallback_reference = false;
    const bool use_fast_reference =
        source == a3_deploy::A3EncoderSource::kA3Fast;
    auto& fallback_reference =
        use_fast_reference ? teleop_fast_fallback_reference
                           : teleop_fallback_reference;
    const bool fallback_reference_loaded =
        use_fast_reference ? teleop_fast_fallback_reference_loaded
                           : teleop_fallback_reference_loaded;
    if (fallback_reference_loaded) {
      if (!teleop_fallback_yaw_offset_valid) {
        teleop_fallback_yaw_offset_rad =
            fallback_reference.ComputeYawOffsetRad(quat_wxyz);
        teleop_fallback_yaw_offset_valid = true;
        std::fprintf(stderr,
                     "[teleop] fallback_yaw_offset_deg=%.3f\n",
                     teleop_fallback_yaw_offset_rad * kRadToDeg);
      }
      using_fallback_reference =
          fallback_reference.BuildTokenizerSlice(
              teleop_tick, quat_wxyz, teleop_fallback_yaw_offset_rad, out);
    }
    if (!using_fallback_reference) {
      a3_deploy::BuildDefaultStandTokenizerSlice(quat_wxyz, out);
    }

    const std::string reason_str = reason ? reason : "no_data";
    if (!teleop_wait_log_valid || teleop_wait_log_status != status ||
        teleop_wait_log_using_fallback_reference != using_fallback_reference ||
        teleop_wait_log_enabled != source_enabled ||
        teleop_wait_log_reason != reason_str) {
      teleop_wait_log_valid = true;
      teleop_wait_log_status = status;
      teleop_wait_log_using_fallback_reference = using_fallback_reference;
      teleop_wait_log_enabled = source_enabled;
      teleop_wait_log_reason = reason_str;
      const char* fallback =
          using_fallback_reference ? "stand reference tokenizer"
                                   : "default stand tokenizer";
      if (using_fallback_reference) {
        std::fprintf(stderr, "[teleop] %s: using %s through ONNX\n",
                     reason_str.c_str(), fallback);
      } else {
        std::fprintf(stderr,
                     "[teleop] %s: using %s through ONNX "
                     "(fallback_blend=%.2f, max_delta=%.3f)\n",
                     reason_str.c_str(), fallback,
                     teleop_fallback_policy_blend,
                     teleop_fallback_max_delta_rad);
      }
    }
    return using_fallback_reference;
  };

  auto command_fn = [&](std::uint64_t               tick_idx,
                        const robot_io::RobotState& state,
                        robot_io::RobotCommand&     command_out) noexcept
      -> bool {
    if (auto_start) {
      if (tick_idx < configured_warmup_ticks) {
        policy_yaw_offset_valid = false;
        manual_control.policy_yaw_offset_valid.store(
            false, std::memory_order_release);
      } else if (!policy_yaw_offset_valid) {
        capture_policy_yaw_offset(state, "auto_start");
      }
      policy_fn(tick_idx, state, command_q_des);
      if (tick_idx < configured_warmup_ticks) {
        a3_deploy::ExpandToBackend(command_q_des, a3_pd_stand_kps,
                                   a3_pd_stand_kds, command_out);
      } else {
        a3_deploy::ExpandToBackend(command_q_des, a3_kps, a3_kds,
                                   command_out);
      }
      return true;
    }

    const auto epoch = manual_control.epoch.load(std::memory_order_acquire);
    const DeployMode mode = LoadDeployMode(manual_control);
    if (epoch != observed_epoch || mode != observed_mode) {
      observed_epoch = epoch;
      observed_mode = mode;
      observed_motion_command_epoch =
          manual_control.motion_command_epoch.load(std::memory_order_acquire);
      observed_teleop_command_epoch =
          manual_control.teleop_command_epoch.load(std::memory_order_acquire);
      observed_teleop_input_paused =
          manual_control.teleop_input_paused.load(std::memory_order_acquire);
      mode_entry_tick = tick_idx;
      pd_stand_initialized = false;
      reset_all_teleop_runtime();
      manual_control.pd_ticks.store(0, std::memory_order_release);
      manual_control.motion_tick.store(0, std::memory_order_release);
      manual_control.motion_held.store(false, std::memory_order_release);
      manual_control.policy_yaw_offset_valid.store(
          false, std::memory_order_release);
      policy_yaw_offset_valid = false;
      if (mode == DeployMode::kMotion) {
        active_motion_playing =
            manual_control.motion_playing.load(std::memory_order_acquire);
        const bool requested_remote =
            manual_control.remote_motion_active.load(
                std::memory_order_acquire) &&
            !remote_motion_library.Empty();
        if (active_motion_playing && requested_remote) {
          activate_remote_motion(state, "remote_motion_start", "playing");
        } else if (active_motion_playing) {
          manual_control.remote_motion_active.store(
              false, std::memory_order_release);
          activate_normal_motion(state, "manual_motion_start", "playing");
        } else {
          if (!requested_remote) {
            manual_control.remote_motion_active.store(
                false, std::memory_order_release);
          }
          reset_motion_idle_runtime();
          obs_builder.Reset();
          manual_control.motion_held.store(true, std::memory_order_release);
          std::fprintf(stderr,
                       "[motion] standing idle; selected normal pointer=[%d/%zu]\n",
                       manual_control.selected_motion_index.load(
                           std::memory_order_acquire) + 1,
                       motion_library.Size());
        }
      } else if (mode == DeployMode::kTeleop) {
        obs_builder.Reset();
        smpl_zmq_source.ResetPlayback();
        teleop_reference.Reset();
        teleop_fast_reference.Reset();
        teleop_first_frame_logged.store(false, std::memory_order_release);
        active_teleop_source = teleop_source_from_value(
            manual_control.teleop_source.load(std::memory_order_acquire));
        if (!teleop_source_available(active_teleop_source)) {
          const auto requested_source = active_teleop_source;
          active_teleop_source = a3_deploy::A3EncoderSource::kA3;
          manual_control.teleop_source.store(0, std::memory_order_release);
          std::fprintf(stderr,
                       "[teleop] %s source disabled on entry; using a3\n",
                       a3_deploy::A3EncoderSourceName(requested_source));
        }
        reset_teleop_source_runtime(active_teleop_source);
      }
    }

    const auto motion_command_epoch =
        manual_control.motion_command_epoch.load(std::memory_order_acquire);
    if (motion_command_epoch != observed_motion_command_epoch) {
      observed_motion_command_epoch = motion_command_epoch;
      if (mode == DeployMode::kMotion) {
        const bool requested_playing =
            manual_control.motion_playing.load(std::memory_order_acquire);
        const bool requested_remote =
            manual_control.remote_motion_active.load(
                std::memory_order_acquire) &&
            !remote_motion_library.Empty();
        if (!requested_playing) {
          if (!requested_remote) {
            manual_control.remote_motion_active.store(
                false, std::memory_order_release);
          }
          active_motion_playing = false;
          manual_control.motion_held.store(true, std::memory_order_release);
        } else if (requested_remote) {
          const std::size_t requested_index =
              load_selected_remote_motion_index();
          if (!active_motion_is_remote ||
              requested_index != active_remote_motion_index ||
              !active_motion_playing) {
            activate_remote_motion(state, "remote_motion_select",
                                   "playing");
          }
          active_motion_playing = true;
          manual_control.motion_held.store(false, std::memory_order_release);
        } else {
          manual_control.remote_motion_active.store(
              false, std::memory_order_release);
          const std::size_t requested_index = load_selected_motion_index();
          if (active_motion_is_remote ||
              requested_index != active_motion_index ||
              !active_motion_playing) {
            activate_normal_motion(state, "motion_select", "playing");
          }
          active_motion_playing = true;
          manual_control.motion_held.store(false, std::memory_order_release);
        }
      }
    }

    const auto teleop_command_epoch =
        manual_control.teleop_command_epoch.load(std::memory_order_acquire);
    if (teleop_command_epoch != observed_teleop_command_epoch) {
      observed_teleop_command_epoch = teleop_command_epoch;
      if (mode == DeployMode::kTeleop) {
        const int requested_source_value =
            manual_control.teleop_source.load(std::memory_order_acquire);
        const auto requested_source =
            teleop_source_from_value(requested_source_value);
        if (requested_source != active_teleop_source) {
          const bool accept_source = teleop_source_available(requested_source);
          if (accept_source) {
            active_teleop_source = requested_source;
            reset_teleop_source_runtime(active_teleop_source);
            std::fprintf(stderr, "[teleop] active source -> %s\n",
                         a3_deploy::A3EncoderSourceName(
                             active_teleop_source));
          } else {
            manual_control.teleop_source.store(
                teleop_source_value(active_teleop_source),
                std::memory_order_release);
            std::fprintf(stderr,
                         "[teleop] source %s rejected: not enabled",
                         a3_deploy::A3EncoderSourceName(requested_source));
            const std::string last_error =
                requested_source == a3_deploy::A3EncoderSource::kSmpl
                    ? smpl_zmq_source.LastError()
                    : std::string{};
            if (!last_error.empty()) {
              std::fprintf(stderr, " (%s)", last_error.c_str());
            }
            std::fprintf(stderr, "\n");
          }
        }
        const bool paused =
            manual_control.teleop_input_paused.load(std::memory_order_acquire);
        if (paused != observed_teleop_input_paused) {
          observed_teleop_input_paused = paused;
          teleop_stream_stale = false;
          teleop_stale_warned = false;
          teleop_stale_latest_stamp_ns = 0;
          smpl_stream_stale = false;
          smpl_stale_warned = false;
          smpl_stale_latest_update_ns = 0;
          clear_teleop_wait_log();
          if (!paused) {
            const std::array<double, 4> quat_wxyz = {
                state.imu_quat_wxyz[0], state.imu_quat_wxyz[1],
                state.imu_quat_wxyz[2], state.imu_quat_wxyz[3]};
            if (active_teleop_source != a3_deploy::A3EncoderSource::kSmpl) {
              auto& active_ta_reference =
                  active_teleop_source ==
                          a3_deploy::A3EncoderSource::kA3Fast
                      ? teleop_fast_reference
                      : teleop_reference;
              if (teleop_enabled && active_ta_reference.LatestStampNs() > 0) {
                if (teleop_yaw_offset_compensation) {
                  teleop_yaw_offset_rad =
                      active_ta_reference.ComputeLatestYawOffsetRad(quat_wxyz);
                  teleop_yaw_offset_valid = true;
                  std::fprintf(stderr,
                               "[teleop] resumed: segment_yaw_offset_deg=%.3f\n",
                               teleop_yaw_offset_rad * kRadToDeg);
                } else {
                  teleop_yaw_offset_rad = 0.0;
                  teleop_yaw_offset_valid = true;
                  std::fprintf(stderr,
                               "[teleop] resumed: using ta_pelvis_yaw directly\n");
                }
                active_ta_reference.ResetToLatestFrame();
              } else if (teleop_yaw_offset_valid) {
                std::fprintf(stderr,
                             "[teleop] resumed: keeping existing yaw_offset\n");
              }
              teleop_running_started = false;
            } else {
              if (smpl_zmq_enabled && smpl_zmq_source.HasAnyFrame()) {
                if (teleop_yaw_offset_compensation) {
                  smpl_yaw_offset_rad =
                      smpl_zmq_source.ComputeLatestYawOffsetRad(quat_wxyz);
                  smpl_yaw_offset_valid = true;
                  std::fprintf(stderr,
                               "[teleop] resumed: smpl_yaw_offset_deg=%.3f\n",
                               smpl_yaw_offset_rad * kRadToDeg);
                } else {
                  smpl_yaw_offset_rad = 0.0;
                  smpl_yaw_offset_valid = true;
                  std::fprintf(stderr,
                               "[teleop] resumed: using smpl root yaw directly\n");
                }
                smpl_zmq_source.ResetPlaybackToLatestWindow();
              } else if (smpl_yaw_offset_valid) {
                std::fprintf(stderr,
                             "[teleop] resumed: keeping existing smpl_yaw_offset\n");
              }
              smpl_running_started = false;
            }
            obs_builder.Reset();
          }
        }
      }
    }

    if (mode == DeployMode::kIdle) {
      return false;
    }

    if (mode == DeployMode::kPassive) {
      a3_deploy::BuildSafeHaltCommand(state.q, command_out);
      return true;
    }

    if (mode == DeployMode::kPdStand) {
      if (!pd_stand_initialized) {
        robot_io::ExtractPolicyView(state.q, pd_start_q);
        pd_stand_initialized = true;
      }
      const std::uint64_t elapsed =
          (tick_idx >= mode_entry_tick) ? (tick_idx - mode_entry_tick) : 0;
      manual_control.pd_ticks.store(elapsed, std::memory_order_release);
      const double alpha =
          (pd_stand_ticks == 0)
              ? 1.0
              : std::min(1.0, static_cast<double>(elapsed) /
                                  static_cast<double>(pd_stand_ticks));
      manual_control.pd_stand_ready.store(alpha >= 1.0,
                                          std::memory_order_release);
      for (int i = 0; i < 29; ++i) {
        command_q_des[i] =
            pd_start_q[i] + alpha * (a3_default_angles[i] - pd_start_q[i]);
      }
      a3_deploy::ExpandToBackend(command_q_des, a3_pd_stand_kps,
                                 a3_pd_stand_kds, command_out);
      return true;
    }

    if (mode == DeployMode::kTeleop) {
      const std::uint64_t teleop_tick =
          (tick_idx >= mode_entry_tick) ? (tick_idx - mode_entry_tick) : 0;
      const bool teleop_input_paused =
          manual_control.teleop_input_paused.load(std::memory_order_acquire);
      manual_control.motion_tick.store(teleop_tick, std::memory_order_release);
      manual_control.motion_held.store(teleop_input_paused,
                                       std::memory_order_release);

      const std::array<double, 4> quat_wxyz = {state.imu_quat_wxyz[0],
                                               state.imu_quat_wxyz[1],
                                               state.imu_quat_wxyz[2],
                                               state.imu_quat_wxyz[3]};
      const bool active_source_is_smpl =
          active_teleop_source == a3_deploy::A3EncoderSource::kSmpl;
      const bool active_source_is_a3_fast =
          active_teleop_source == a3_deploy::A3EncoderSource::kA3Fast;
      auto& active_ta_reference =
          active_source_is_a3_fast ? teleop_fast_reference : teleop_reference;
      const std::int64_t latest_teleop_stamp_ns =
          teleop_enabled ? active_ta_reference.LatestStampNs() : 0;
      const std::int64_t teleop_stale_warn_ns =
          static_cast<std::int64_t>(teleop_stale_warn_ms * 1'000'000.0);
      const bool teleop_has_latest = latest_teleop_stamp_ns > 0;
      const std::int64_t teleop_age_ns =
          teleop_has_latest ? (state.timestamp_ns - latest_teleop_stamp_ns) : 0;
      const bool teleop_currently_stale =
          teleop_has_latest && teleop_age_ns > teleop_stale_warn_ns;

      bool teleop_using_fallback_reference = false;
      bool teleop_tokenizer_ok = false;
      a3_deploy::A3TeleopTokenizerStatus teleop_status =
          a3_deploy::A3TeleopTokenizerStatus::kNoData;
      bool smpl_tokenizer_ok = false;

      if (!active_source_is_smpl &&
          !teleop_input_paused && teleop_stream_stale && teleop_has_latest &&
          latest_teleop_stamp_ns != teleop_stale_latest_stamp_ns &&
          !teleop_currently_stale) {
        if (teleop_yaw_offset_compensation) {
          teleop_yaw_offset_rad =
              active_ta_reference.ComputeLatestYawOffsetRad(quat_wxyz);
          teleop_yaw_offset_valid = true;
        } else {
          teleop_yaw_offset_rad = 0.0;
          teleop_yaw_offset_valid = true;
        }
        active_ta_reference.ResetToLatestFrame();
        teleop_stream_stale = false;
        teleop_stale_warned = false;
        teleop_stale_latest_stamp_ns = 0;
        clear_teleop_wait_log();
        teleop_running_started = false;
        obs_builder.Reset();
        token_transition_remaining = token_transition_ticks;
        if (teleop_yaw_offset_compensation) {
          std::fprintf(stderr,
                       "[teleop] reconnected: segment_yaw_offset_deg=%.3f\n",
                       teleop_yaw_offset_rad * kRadToDeg);
        } else {
          std::fprintf(stderr,
                       "[teleop] reconnected: using ta_pelvis_yaw directly\n");
        }
      }

      if (!active_source_is_smpl) {
        bool teleop_holding_last_command = false;
        if (teleop_input_paused && teleop_hold_valid) {
          tokenizer_slice = teleop_hold_slice;
          teleop_tokenizer_ok = true;
          teleop_status = a3_deploy::A3TeleopTokenizerStatus::kRunning;
        } else if (!teleop_input_paused && teleop_enabled) {
          if (!teleop_yaw_offset_valid && teleop_has_latest &&
              !teleop_currently_stale) {
            if (teleop_yaw_offset_compensation) {
              teleop_yaw_offset_rad =
                  active_ta_reference.ComputeLatestYawOffsetRad(quat_wxyz);
            } else {
              teleop_yaw_offset_rad = 0.0;
            }
            teleop_yaw_offset_valid = true;
            active_ta_reference.ResetToLatestFrame();
            teleop_running_started = false;
            obs_builder.Reset();
            token_transition_remaining = token_transition_ticks;
            if (teleop_yaw_offset_compensation) {
              std::fprintf(stderr,
                           "[teleop] initial segment_yaw_offset_deg=%.3f\n",
                           teleop_yaw_offset_rad * kRadToDeg);
            } else {
              std::fprintf(stderr,
                           "[teleop] initial: using ta_pelvis_yaw directly\n");
            }
          }
          const double yaw_offset =
              (teleop_yaw_offset_compensation && teleop_yaw_offset_valid)
                  ? teleop_yaw_offset_rad
                  : 0.0;
          teleop_tokenizer_ok = active_ta_reference.BuildTokenizerSlice(
              state.timestamp_ns, quat_wxyz, yaw_offset, tokenizer_slice,
              &teleop_status);
        }

        if (!teleop_tokenizer_ok &&
            teleop_status == a3_deploy::A3TeleopTokenizerStatus::kBuffering &&
            teleop_hold_valid) {
          tokenizer_slice = teleop_hold_slice;
          teleop_tokenizer_ok = true;
          teleop_holding_last_command = true;
          if (!teleop_wait_log_valid ||
              teleop_wait_log_status != teleop_status ||
              teleop_wait_log_using_fallback_reference ||
              teleop_wait_log_enabled != teleop_enabled ||
              teleop_wait_log_reason != "buffering_hold") {
            teleop_wait_log_valid = true;
            teleop_wait_log_status = teleop_status;
            teleop_wait_log_using_fallback_reference = false;
            teleop_wait_log_enabled = teleop_enabled;
            teleop_wait_log_reason = "buffering_hold";
            std::fprintf(stderr,
                         "[teleop] buffering: holding last whole_body_command\n");
          }
        }
        if (!teleop_tokenizer_ok) {
          const char* reason = "no_data";
          if (teleop_input_paused) {
            reason = "paused";
          } else if (teleop_status ==
                     a3_deploy::A3TeleopTokenizerStatus::kBuffering) {
            reason = "buffering";
          } else if (!teleop_enabled) {
            reason = "disabled";
          }
          teleop_using_fallback_reference = build_teleop_stand_fallback(
              teleop_tick, quat_wxyz, teleop_status, reason, teleop_enabled,
              active_teleop_source, tokenizer_slice);
        } else if (!teleop_holding_last_command) {
          if (!teleop_running_started) {
            obs_builder.Reset();
            teleop_running_started = true;
          }
          clear_teleop_wait_log();
          if (!teleop_input_paused) {
            teleop_hold_slice = tokenizer_slice;
            teleop_hold_valid = true;
          }
        }
        if (!teleop_input_paused && teleop_currently_stale) {
          if (!teleop_stream_stale) {
            teleop_stream_stale = true;
            teleop_stale_warned = false;
            teleop_stale_latest_stamp_ns = latest_teleop_stamp_ns;
          }
          if (!teleop_stale_warned) {
            std::fprintf(stderr,
                         "[teleop] stale %.1f ms: holding last whole_body_command\n",
                         static_cast<double>(teleop_age_ns) / 1e6);
            teleop_stale_warned = true;
          }
        }
      } else {
        const std::int64_t now_monotonic_ns = NowMonotonicNsNoThrow();
        const std::int64_t latest_smpl_update_ns =
            smpl_zmq_enabled ? smpl_zmq_source.LatestUpdateMonotonicNs() : 0;
        const bool smpl_has_latest = latest_smpl_update_ns > 0;
        const std::int64_t smpl_age_ns =
            (smpl_has_latest && now_monotonic_ns > 0)
                ? (now_monotonic_ns - latest_smpl_update_ns)
                : 0;
        const bool smpl_currently_stale =
            smpl_has_latest && now_monotonic_ns > 0 &&
            smpl_age_ns > teleop_stale_warn_ns;
        const bool smpl_ready =
            smpl_zmq_enabled && smpl_zmq_source.HasReadyWindow();
        bool smpl_holding_last_command = false;

        if (!teleop_input_paused && smpl_stream_stale && smpl_ready &&
            smpl_has_latest &&
            latest_smpl_update_ns != smpl_stale_latest_update_ns &&
            !smpl_currently_stale) {
          if (teleop_yaw_offset_compensation) {
            smpl_yaw_offset_rad =
                smpl_zmq_source.ComputeLatestYawOffsetRad(quat_wxyz);
          } else {
            smpl_yaw_offset_rad = 0.0;
          }
          smpl_yaw_offset_valid = true;
          smpl_zmq_source.ResetPlaybackToLatestWindow();
          smpl_stream_stale = false;
          smpl_stale_warned = false;
          smpl_stale_latest_update_ns = 0;
          clear_teleop_wait_log();
          smpl_running_started = false;
          obs_builder.Reset();
          token_transition_remaining = token_transition_ticks;
          if (teleop_yaw_offset_compensation) {
            std::fprintf(stderr,
                         "[teleop] smpl reconnected: yaw_offset_deg=%.3f\n",
                         smpl_yaw_offset_rad * kRadToDeg);
          } else {
            std::fprintf(stderr,
                         "[teleop] smpl reconnected: using root yaw directly\n");
          }
        }

        if (teleop_input_paused && smpl_hold_valid) {
          smpl_tokenizer_slice = smpl_hold_slice;
          smpl_tokenizer_ok = true;
          smpl_holding_last_command = true;
        } else if (!teleop_input_paused && smpl_ready &&
                   !smpl_currently_stale) {
          if (!smpl_yaw_offset_valid) {
            if (teleop_yaw_offset_compensation) {
              smpl_yaw_offset_rad =
                  smpl_zmq_source.ComputeLatestYawOffsetRad(quat_wxyz);
            } else {
              smpl_yaw_offset_rad = 0.0;
            }
            smpl_yaw_offset_valid = true;
            smpl_zmq_source.ResetPlaybackToLatestWindow();
            smpl_running_started = false;
            obs_builder.Reset();
            token_transition_remaining = token_transition_ticks;
            if (teleop_yaw_offset_compensation) {
              std::fprintf(stderr,
                           "[teleop] smpl initial yaw_offset_deg=%.3f\n",
                           smpl_yaw_offset_rad * kRadToDeg);
            } else {
              std::fprintf(stderr,
                           "[teleop] smpl initial: using root yaw directly\n");
            }
          }
          smpl_tokenizer_ok = smpl_zmq_source.BuildTokenizerSlice(
              quat_wxyz,
              (teleop_yaw_offset_compensation && smpl_yaw_offset_valid)
                  ? smpl_yaw_offset_rad
                  : 0.0,
              smpl_tokenizer_slice,
              /*advance_playback=*/!teleop_input_paused);
          if (smpl_tokenizer_ok && !teleop_input_paused) {
            smpl_hold_slice = smpl_tokenizer_slice;
            smpl_hold_valid = true;
          }
        }
        if (!smpl_tokenizer_ok && smpl_hold_valid) {
          smpl_tokenizer_slice = smpl_hold_slice;
          smpl_tokenizer_ok = true;
          smpl_holding_last_command = true;
        }

        if (smpl_tokenizer_ok && !smpl_holding_last_command) {
          if (!smpl_running_started) {
            obs_builder.Reset();
            smpl_running_started = true;
          }
          clear_teleop_wait_log();
        }

        if (!teleop_input_paused && smpl_has_latest && smpl_currently_stale) {
          if (!smpl_stream_stale) {
            smpl_stream_stale = true;
            smpl_stale_warned = false;
            smpl_stale_latest_update_ns = latest_smpl_update_ns;
          }
          if (!smpl_stale_warned) {
            std::fprintf(stderr,
                         "[teleop] smpl stale %.1f ms: holding last command\n",
                         static_cast<double>(smpl_age_ns) / 1e6);
            smpl_stale_warned = true;
          }
        }

        if (!smpl_tokenizer_ok) {
          a3_deploy::A3TeleopTokenizerStatus fallback_status =
              a3_deploy::A3TeleopTokenizerStatus::kNoData;
          const char* reason = "smpl_no_data";
          if (teleop_input_paused) {
            reason = "smpl_paused";
          } else if (!smpl_zmq_enabled) {
            reason = "smpl_disabled";
          } else if (smpl_currently_stale) {
            reason = "smpl_stale";
            fallback_status = a3_deploy::A3TeleopTokenizerStatus::kRunning;
          } else if (smpl_has_latest || smpl_zmq_source.HasAnyFrame()) {
            reason = "smpl_buffering";
            fallback_status = a3_deploy::A3TeleopTokenizerStatus::kBuffering;
          }
          teleop_using_fallback_reference = build_teleop_stand_fallback(
              teleop_tick, quat_wxyz, fallback_status, reason,
              smpl_zmq_enabled, a3_deploy::A3EncoderSource::kA3,
              tokenizer_slice);
        }
      }

      const bool use_smpl_encoder =
          active_teleop_source == a3_deploy::A3EncoderSource::kSmpl &&
          smpl_tokenizer_ok;
      const bool use_a3_fast_encoder =
          active_teleop_source == a3_deploy::A3EncoderSource::kA3Fast &&
          a3_fast_policy_runtime_available &&
          (teleop_tokenizer_ok || teleop_using_fallback_reference);
      const bool policy_source_ok =
          use_smpl_encoder || use_a3_fast_encoder || teleop_tokenizer_ok ||
          teleop_using_fallback_reference;

      tokenizer_override_active = true;
      tokenizer_override = tokenizer_slice.data();
      encoder_source_override_active =
          encoder_decoder_mode || use_smpl_encoder || use_a3_fast_encoder;
      encoder_source_override =
          use_smpl_encoder
              ? a3_deploy::A3EncoderSource::kSmpl
              : (use_a3_fast_encoder ? a3_deploy::A3EncoderSource::kA3Fast
                                     : a3_deploy::A3EncoderSource::kA3);
      smpl_tokenizer_override =
          use_smpl_encoder ? smpl_tokenizer_slice.data() : nullptr;
      const bool previous_remember_policy_action = remember_policy_action;
      remember_policy_action = policy_source_ok;
      policy_fn(configured_warmup_ticks + teleop_tick, state, command_q_des);
      remember_policy_action = previous_remember_policy_action;
      encoder_source_override_active = false;
      smpl_tokenizer_override = nullptr;
      tokenizer_override_active = false;
      tokenizer_override = nullptr;
      if (!policy_source_ok) {
        a3_deploy::ApplyStandFallbackCommandFilter(
            teleop_fallback_policy_blend, teleop_fallback_max_delta_rad,
            command_q_des);
        if (teleop_fallback_use_pd_stand_gains) {
          a3_deploy::ExpandToBackend(command_q_des, a3_pd_stand_kps,
                                     a3_pd_stand_kds, command_out);
        } else {
          a3_deploy::ExpandToBackend(command_q_des, a3_kps, a3_kds,
                                     command_out);
        }
      } else {
        a3_deploy::ExpandToBackend(command_q_des, a3_kps, a3_kds, command_out);
      }
      return true;
    }

    std::uint64_t policy_tick = active_motion_tick;
    bool held_last = false;
    const std::size_t motion_ticks =
        policy_reference ? policy_reference->NumTicks() : 0;
    if (motion_ticks > 0 && policy_tick >= motion_ticks) {
      policy_tick = static_cast<std::uint64_t>(motion_ticks - 1);
      active_motion_tick = policy_tick;
      active_motion_playing = false;
      manual_control.motion_playing.store(false, std::memory_order_release);
      held_last = true;
      reset_motion_idle_runtime();
    }

    const std::array<double, 4> quat_wxyz = {state.imu_quat_wxyz[0],
                                             state.imu_quat_wxyz[1],
                                             state.imu_quat_wxyz[2],
                                             state.imu_quat_wxyz[3]};
    const bool use_idle_stand =
        !active_motion_playing || held_last || policy_reference == nullptr ||
        motion_ticks == 0;
    if (use_idle_stand) {
      manual_control.motion_tick.store(policy_tick, std::memory_order_release);
      manual_control.motion_held.store(true, std::memory_order_release);
      const std::uint64_t idle_tick = motion_idle_tick;
      build_motion_idle_stand(idle_tick, quat_wxyz, tokenizer_slice);
      tokenizer_override_active = true;
      tokenizer_override = tokenizer_slice.data();
      policy_fn(configured_warmup_ticks + idle_tick, state, command_q_des);
      tokenizer_override_active = false;
      tokenizer_override = nullptr;
      ++motion_idle_tick;
      a3_deploy::ExpandToBackend(command_q_des, a3_kps, a3_kds, command_out);
      return true;
    }

    if (motion_idle_tick != 0 || motion_idle_yaw_offset_valid) {
      reset_motion_idle_runtime();
    }
    manual_control.motion_tick.store(policy_tick, std::memory_order_release);
    manual_control.motion_held.store(false, std::memory_order_release);
    if (!policy_yaw_offset_valid) {
      capture_policy_yaw_offset(state, "manual_motion_start");
    }

    // Re-use the proven auto policy path, but shift tick_idx so its internal
    // warmup rebasing observes policy_tick == 0 on the first MOTION frame.
    policy_fn(configured_warmup_ticks + policy_tick, state, command_q_des);
    tokenizer_override_active = false;
    tokenizer_override = nullptr;
    a3_deploy::ExpandToBackend(command_q_des, a3_kps, a3_kds, command_out);

    if (active_motion_playing && motion_ticks > 0) {
      if (policy_tick + 1 >= motion_ticks) {
        active_motion_tick = static_cast<std::uint64_t>(motion_ticks - 1);
        active_motion_playing = false;
        manual_control.motion_playing.store(false, std::memory_order_release);
        manual_control.motion_held.store(true, std::memory_order_release);
        reset_motion_idle_runtime();
      } else {
        active_motion_tick = policy_tick + 1;
      }
    }
    return true;
  };

  // --- Driver --------------------------------------------------------------
  a3_deploy::A3PolicyDriverOptions dopt;
  dopt.send_safe_halt_before_first_command = auto_start;
  dopt.policy_hz = configured_policy_hz;
  if (cfg["policy_driver"]["watchdog"]) {
    const auto& wd = cfg["policy_driver"]["watchdog"];
    if (wd["max_frame_age_ms"]) {
      dopt.watchdog.max_frame_age_ns =
          static_cast<std::int64_t>(wd["max_frame_age_ms"].as<double>() *
                                    1'000'000.0);
    }
    if (wd["max_unaligned_frames"]) {
      dopt.watchdog.max_consecutive_unaligned =
          wd["max_unaligned_frames"].as<int>();
    }
  }
  if (cfg["policy_driver"]["rt"]) {
    const auto& rt = cfg["policy_driver"]["rt"];
    if (rt["sched_fifo_priority"]) {
      dopt.sched.priority = rt["sched_fifo_priority"].as<int>();
    }
    if (rt["cpu_affinity"]) {
      dopt.sched.cpu = rt["cpu_affinity"].as<int>();
    }
  }
  const std::string sync_mode = NormalizeConfigToken(
      OptionalKey<std::string>(cfg["backend"]["sync_mode"], "min_skew_pair"));
  const bool latest_frame_mode = sync_mode == "latest_frame";
  const bool phase_align_to_sync =
      OptionalKey<bool>(cfg["policy_driver"]["phase_align_to_sync"], true);
  auto configure_policy_phase_alignment = [&]() -> bool {
    if (!phase_align_to_sync) {
      if (latest_frame_mode) {
        std::cout << "[policy] latest-frame mode: policy_hz=" << dopt.policy_hz
                  << " trigger=timer sync_mode=" << sync_mode << "\n";
      }
      return true;
    }
    if (sync_mode != "header_interp" && sync_mode != "min_skew_pair" &&
        sync_mode != "latest_frame") {
      std::cerr << "backend.sync_mode must be one of "
                   "{header_interp,min_skew_pair,latest_frame}; got: "
                << sync_mode << "\n";
      return false;
    }
    const double sync_ready_offset_ms =
        OptionalKey<double>(cfg["policy_driver"]["sync_ready_offset_ms"], 0.2);
    if (!a3_deploy::numeric_safety::IsFinite(dopt.policy_hz) ||
        !a3_deploy::numeric_safety::IsFinite(sync_ready_offset_ms) ||
        dopt.policy_hz <= 0.0 ||
        sync_ready_offset_ms < 0.0) {
      std::cerr << "invalid phase-align timing config: policy_hz="
                << dopt.policy_hz
                << " sync_ready_offset_ms=" << sync_ready_offset_ms << "\n";
      return false;
    }
    const auto policy_period_ns =
        static_cast<std::int64_t>(1.0e9 / dopt.policy_hz);
    if (policy_period_ns <= 0) {
      std::cerr << "invalid phase-align period: policy_period_ns="
                << policy_period_ns << "\n";
      return false;
    }
    if (latest_frame_mode) {
      dopt.trigger_on_state = true;
      dopt.trigger_offset_ns = MsToNs(sync_ready_offset_ms);
      dopt.trigger_min_period_ns = policy_period_ns;
      const auto old_flags = std::cout.flags();
      const auto old_precision = std::cout.precision();
      std::cout << "[policy] phase-align enabled: policy_hz="
                << dopt.policy_hz
                << " sync_mode=" << sync_mode
                << " offset_ms=" << sync_ready_offset_ms
                << " trigger=latest_frame_event\n";
      std::cout.flags(old_flags);
      std::cout.precision(old_precision);
      return true;
    } else {
      const double sync_hz =
          OptionalKey<double>(cfg["backend"]["sync_hz"], dopt.policy_hz * 2.0);
      const double align_delay_ms =
          OptionalKey<double>(cfg["backend"]["align_delay_ms"], 2.0);
      const double sync_ready_after_input_ms =
          OptionalKey<double>(cfg["backend"]["sync_ready_after_input_ms"], 0.2);
      const double effective_align_delay_ms =
          (sync_mode == "min_skew_pair") ? sync_ready_after_input_ms
                                         : align_delay_ms;
      const double backend_phase_ms =
          cfg["backend"]["phase_ms"]
              ? cfg["backend"]["phase_ms"].as<double>()
              : [&]() {
                  if (auto* a3_backend =
                          dynamic_cast<robot_io::A3AimrtBackend*>(&backend)) {
                    return static_cast<double>(a3_backend->SyncPhaseNs()) / 1e6;
                  }
                  return 1.5;
                }();

      if (!a3_deploy::numeric_safety::IsFinite(sync_hz) ||
          !a3_deploy::numeric_safety::IsFinite(align_delay_ms) ||
          !a3_deploy::numeric_safety::IsFinite(effective_align_delay_ms) ||
          !a3_deploy::numeric_safety::IsFinite(sync_ready_after_input_ms) ||
          !a3_deploy::numeric_safety::IsFinite(backend_phase_ms) ||
          sync_hz <= 0.0 || align_delay_ms < 0.0 ||
          sync_ready_after_input_ms < 0.0) {
        std::cerr << "invalid phase-align timing config: policy_hz="
                  << dopt.policy_hz << " sync_hz=" << sync_hz
                  << " align_delay_ms=" << align_delay_ms
                  << " sync_ready_after_input_ms="
                  << sync_ready_after_input_ms
                  << " phase_ms=" << backend_phase_ms
                  << " sync_ready_offset_ms=" << sync_ready_offset_ms << "\n";
        return false;
      }

      const double ratio = sync_hz / dopt.policy_hz;
      const auto ratio_int = static_cast<std::int64_t>(std::llround(ratio));
      if (ratio_int < 1 || std::abs(ratio - static_cast<double>(ratio_int)) > 1e-6) {
        std::cerr << "policy_driver.phase_align_to_sync=true requires "
                     "backend.sync_hz / policy_driver.policy_hz to be an "
                     "integer; got sync_hz=" << sync_hz
                  << " policy_hz=" << dopt.policy_hz
                  << " ratio=" << ratio << "\n";
        return false;
      }

      const auto sync_period_ns =
          static_cast<std::int64_t>(1.0e9 / sync_hz);
      if (sync_period_ns <= 0) {
        std::cerr << "invalid phase-align periods: sync_period_ns="
                  << sync_period_ns
                  << " policy_period_ns=" << policy_period_ns << "\n";
        return false;
      }
      auto backend_phase_ns = MsToNs(backend_phase_ms);
      if (backend_phase_ns < 0) backend_phase_ns = 0;
      if (backend_phase_ns >= sync_period_ns) {
        backend_phase_ns = backend_phase_ns % sync_period_ns;
      }

      const auto policy_phase_ns =
          ModNs(backend_phase_ns + MsToNs(effective_align_delay_ms) +
                    MsToNs(sync_ready_offset_ms),
                policy_period_ns);
      try {
        const auto now_system_ns = NowSystemNs();
        const auto now_monotonic_ns = NowMonotonicNs();
        const auto first_policy_system_ns = NextSystemTimeAtPhaseNs(
            now_system_ns, policy_period_ns, policy_phase_ns, MsToNs(2.0));
        dopt.first_wake_monotonic_ns =
            now_monotonic_ns + (first_policy_system_ns - now_system_ns);
        const double first_wake_in_ms =
            static_cast<double>(dopt.first_wake_monotonic_ns - now_monotonic_ns) /
            1e6;
        const auto old_flags = std::cout.flags();
        const auto old_precision = std::cout.precision();
        std::cout << "[policy] phase-align enabled: policy_hz="
                  << dopt.policy_hz << " sync_hz=" << sync_hz
                  << " ratio=" << ratio_int
                  << " sync_mode=" << sync_mode
                  << " sync_release_delay_ms=" << effective_align_delay_ms
                  << " input_ready_delay_ms=" << sync_ready_after_input_ms
                  << " phase_ms=" << backend_phase_ms
                  << " offset_ms=" << sync_ready_offset_ms
                  << " first_wake_in_ms=" << std::fixed
                  << std::setprecision(2) << first_wake_in_ms
                  << " trigger=timer_aligned\n";
        std::cout.flags(old_flags);
        std::cout.precision(old_precision);
      } catch (const std::exception& e) {
        std::cerr << "failed to compute policy phase-align start time: "
                  << e.what() << "\n";
        return false;
      }
    }
    return true;
  };
  std::unique_ptr<a3_deploy::A3PolicyDriver> driver;

  // --- Lifecycle -----------------------------------------------------------
  InstallSigintHandler();

  // Start backend first so DDS/AimRT transport is ready before the driver
  // registers callbacks and publishes commands.
  if (!backend.Start()) {
    std::cerr << "Backend::Start failed\n";
    return 5;
  }
  std::cout << "✓ backend started\n";

  if (!configure_policy_phase_alignment()) {
    backend.Stop();
    return 64;
  }

  driver = std::make_unique<a3_deploy::A3PolicyDriver>(
      backend, command_fn, dopt);
  if (!driver->StartDriver()) {
    std::cerr << "A3PolicyDriver::StartDriver failed\n";
    backend.Stop();
    return 6;
  }
  std::thread keyboard_thread;
  if (auto_start) {
    std::cout << "✓ auto-start mode: PD warmup=" << configured_warmup_ticks
              << " ticks, then policy inference\n";
  } else {
    std::cout << "✓ manual state machine: startup IDLE/no-output; p=passive, "
                 "s=pd_stand, arrows=remote motion, t=teleop, "
                 "1=a3, 2=smpl, q=quit\n";
    keyboard_thread = StartKeyboardControlThread(
        manual_control, motion_names, remote_motion_names, motion_shortcuts);
  }
  std::cout << "✓ entering policy loop; Ctrl-C to exit\n";
  if (frame_log_interval > 0) {
    std::cout << "✓ frame progress logging every " << frame_log_interval
              << " driver frames, latency="
              << LatencyLogModeName(latency_log_mode)
              << " (A3_LATENCY_LOG=verbose for full breakdown)\n";
  }

  auto* sync_backend =
      dynamic_cast<robot_io::A3AimrtBackend*>(&backend);
  WaitForShutdownWithFrameLog(*driver, configured_warmup_ticks,
                              frame_log_interval, infer_timing,
                              latency_log_mode, auto_start,
                              &manual_control, sync_backend,
                              latest_frame_mode);

  std::cout << "shutdown requested — stopping driver\n";
  if (keyboard_thread.joinable()) keyboard_thread.join();
  driver->StopDriver();
  if (dump_fp) {
    std::fclose(dump_fp);
    dump_fp = nullptr;
    std::cout << "[dump] closed: " << dump_path << "\n";
  }
  std::cout << "shutdown requested — stopping backend\n";
  backend.Stop();
  std::cout << "a3_deploy_onnx_ref exiting cleanly"
            << " policy_ticks=" << driver->PolicyTickCount()
            << " safe_halts=" << driver->SafeHaltCount() << "\n";
  return 0;
}
