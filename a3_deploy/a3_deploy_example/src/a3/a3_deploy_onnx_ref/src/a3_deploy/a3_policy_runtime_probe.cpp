#include "a3_deploy/a3_policy_runtime.hpp"

#include "a3_deploy/numeric_safety.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <random>
#include <string>
#include <vector>

namespace {

struct ProbeOptions {
  std::string backend{"ort_cpu"};
  std::string model_path;
  std::string rknn_core_mask{"auto"};
  std::size_t expected_input_dim{0};
  std::size_t expected_action_dim{29};
  int warmup_runs{5};
  int runs{50};
  std::uint32_t seed{1};
  std::string input_pattern{"random"};
};

void PrintUsage(const char* prog) {
  std::cerr
      << "Usage: " << prog
      << " --backend ort_cpu|rknn --model PATH [options]\n"
      << "\nOptions:\n"
      << "  --input-dim N        Expected input dimension; 0 disables check.\n"
      << "  --action-dim N       Expected action dimension; default 29.\n"
      << "  --warmup N           Warmup inference runs; default 5.\n"
      << "  --runs N             Timed inference runs; default 50.\n"
      << "  --seed N             Random input seed; default 1.\n"
      << "  --input zero|random|ramp  Input pattern; default random.\n"
      << "  --rknn-core-mask M   auto, all, 0, 1, 2, 0_1, or 0_1_2.\n";
}

bool ParseSize(const std::string& raw, std::size_t& out) {
  try {
    std::size_t parsed = 0;
    const auto value = std::stoull(raw, &parsed, 10);
    if (parsed != raw.size()) return false;
    out = static_cast<std::size_t>(value);
    return true;
  } catch (...) {
    return false;
  }
}

bool ParseInt(const std::string& raw, int& out) {
  try {
    std::size_t parsed = 0;
    const int value = std::stoi(raw, &parsed, 10);
    if (parsed != raw.size()) return false;
    out = value;
    return true;
  } catch (...) {
    return false;
  }
}

bool ParseUint32(const std::string& raw, std::uint32_t& out) {
  try {
    std::size_t parsed = 0;
    const auto value = std::stoull(raw, &parsed, 10);
    if (parsed != raw.size() ||
        value > std::numeric_limits<std::uint32_t>::max()) {
      return false;
    }
    out = static_cast<std::uint32_t>(value);
    return true;
  } catch (...) {
    return false;
  }
}

bool ParseArgs(int argc, char** argv, ProbeOptions& opts) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto require_value = [&](const char* name) -> const char* {
      if (i + 1 >= argc) {
        std::cerr << name << " requires a value\n";
        return nullptr;
      }
      return argv[++i];
    };

    if (arg == "--help" || arg == "-h") {
      PrintUsage(argv[0]);
      std::exit(0);
    } else if (arg == "--backend") {
      const char* value = require_value("--backend");
      if (!value) return false;
      opts.backend = value;
    } else if (arg == "--model") {
      const char* value = require_value("--model");
      if (!value) return false;
      opts.model_path = value;
    } else if (arg == "--input-dim") {
      const char* value = require_value("--input-dim");
      if (!value || !ParseSize(value, opts.expected_input_dim)) {
        std::cerr << "--input-dim must be a non-negative integer\n";
        return false;
      }
    } else if (arg == "--action-dim") {
      const char* value = require_value("--action-dim");
      if (!value || !ParseSize(value, opts.expected_action_dim)) {
        std::cerr << "--action-dim must be a non-negative integer\n";
        return false;
      }
    } else if (arg == "--warmup") {
      const char* value = require_value("--warmup");
      if (!value || !ParseInt(value, opts.warmup_runs) ||
          opts.warmup_runs < 0) {
        std::cerr << "--warmup must be a non-negative integer\n";
        return false;
      }
    } else if (arg == "--runs") {
      const char* value = require_value("--runs");
      if (!value || !ParseInt(value, opts.runs) || opts.runs <= 0) {
        std::cerr << "--runs must be a positive integer\n";
        return false;
      }
    } else if (arg == "--seed") {
      const char* value = require_value("--seed");
      if (!value || !ParseUint32(value, opts.seed)) {
        std::cerr << "--seed must be a uint32 integer\n";
        return false;
      }
    } else if (arg == "--input") {
      const char* value = require_value("--input");
      if (!value) return false;
      opts.input_pattern = value;
      if (opts.input_pattern != "zero" && opts.input_pattern != "random" &&
          opts.input_pattern != "ramp") {
        std::cerr << "--input must be one of: zero, random, ramp\n";
        return false;
      }
    } else if (arg == "--rknn-core-mask") {
      const char* value = require_value("--rknn-core-mask");
      if (!value) return false;
      opts.rknn_core_mask = value;
    } else {
      std::cerr << "unknown argument: " << arg << "\n";
      return false;
    }
  }

  if (opts.model_path.empty()) {
    std::cerr << "--model is required\n";
    return false;
  }
  return true;
}

void FillInput(const ProbeOptions& opts, float* data, std::size_t count) {
  if (opts.input_pattern == "zero") {
    std::fill_n(data, count, 0.0f);
    return;
  }
  if (opts.input_pattern == "ramp") {
    for (std::size_t i = 0; i < count; ++i) {
      data[i] = static_cast<float>((static_cast<int>(i % 257) - 128)) / 128.0f;
    }
    return;
  }
  std::mt19937 rng(opts.seed);
  std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
  for (std::size_t i = 0; i < count; ++i) {
    data[i] = dist(rng);
  }
}

struct ActionStats {
  double sum{0.0};
  double abs_sum{0.0};
  float min{0.0f};
  float max{0.0f};
  bool finite{true};
};

ActionStats ComputeActionStats(const float* data, std::size_t count) {
  ActionStats stats;
  if (count == 0) return stats;
  stats.min = data[0];
  stats.max = data[0];
  for (std::size_t i = 0; i < count; ++i) {
    const float value = data[i];
    stats.finite = stats.finite && a3_deploy::numeric_safety::IsFinite(value);
    stats.sum += value;
    stats.abs_sum += std::abs(value);
    stats.min = std::min(stats.min, value);
    stats.max = std::max(stats.max, value);
  }
  return stats;
}

}  // namespace

int main(int argc, char** argv) {
  ProbeOptions opts;
  if (!ParseArgs(argc, argv, opts)) {
    PrintUsage(argv[0]);
    return 64;
  }

  a3_deploy::A3PolicyRuntimeOptions runtime_options;
  runtime_options.backend = opts.backend;
  runtime_options.rknn_core_mask = opts.rknn_core_mask;

  auto runtime = a3_deploy::CreateA3PolicyRuntime(runtime_options);
  if (!runtime) {
    return 2;
  }
  if (!runtime->Initialize(opts.model_path, runtime_options)) {
    std::cerr << "policy runtime initialisation failed\n";
    return 2;
  }

  const std::size_t input_dim = runtime->GetInputDimension();
  const std::size_t action_dim = runtime->GetActionDimension();
  if (opts.expected_input_dim != 0 && input_dim != opts.expected_input_dim) {
    std::cerr << "input dim mismatch: expected " << opts.expected_input_dim
              << ", got " << input_dim << "\n";
    return 2;
  }
  if (opts.expected_action_dim != 0 && action_dim != opts.expected_action_dim) {
    std::cerr << "action dim mismatch: expected " << opts.expected_action_dim
              << ", got " << action_dim << "\n";
    return 2;
  }

  FillInput(opts, runtime->MutableInputData(), input_dim);
  for (int i = 0; i < opts.warmup_runs; ++i) {
    if (!runtime->Infer()) {
      std::cerr << "warmup infer failed at run " << i << "\n";
      return 3;
    }
  }

  std::vector<double> latencies_ms;
  latencies_ms.reserve(static_cast<std::size_t>(opts.runs));
  for (int i = 0; i < opts.runs; ++i) {
    const auto start = std::chrono::steady_clock::now();
    if (!runtime->Infer()) {
      std::cerr << "timed infer failed at run " << i << "\n";
      return 3;
    }
    const auto end = std::chrono::steady_clock::now();
    latencies_ms.push_back(
        std::chrono::duration<double, std::milli>(end - start).count());
  }

  std::sort(latencies_ms.begin(), latencies_ms.end());
  const double sum = std::accumulate(latencies_ms.begin(), latencies_ms.end(),
                                     0.0);
  const double mean = sum / static_cast<double>(latencies_ms.size());
  const double p50 = latencies_ms[latencies_ms.size() / 2];
  const double p90 =
      latencies_ms[static_cast<std::size_t>(0.9 * (latencies_ms.size() - 1))];
  const double min = latencies_ms.front();
  const double max = latencies_ms.back();
  const ActionStats stats =
      ComputeActionStats(runtime->ActionData(), action_dim);

  std::cout << std::fixed << std::setprecision(4)
            << "A3 policy runtime probe OK\n"
            << "  backend=" << runtime->BackendName() << "\n"
            << "  model=" << opts.model_path << "\n"
            << "  input_dim=" << input_dim
            << " action_dim=" << action_dim << "\n"
            << "  input_pattern=" << opts.input_pattern
            << " warmup=" << opts.warmup_runs << " runs=" << opts.runs
            << "\n"
            << "  latency_ms min=" << min << " p50=" << p50
            << " p90=" << p90 << " mean=" << mean << " max=" << max
            << "\n"
            << "  action sum=" << stats.sum
            << " abs_sum=" << stats.abs_sum << " min=" << stats.min
            << " max=" << stats.max
            << " finite=" << (stats.finite ? "true" : "false") << "\n";
  return stats.finite ? 0 : 4;
}
