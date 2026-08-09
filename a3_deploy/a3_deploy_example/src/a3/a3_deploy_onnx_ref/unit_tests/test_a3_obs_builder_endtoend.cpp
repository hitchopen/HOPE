// Copyright (c) 2026, AgiBot Inc. All rights reserved.
//
// Level-2 end-to-end bit-exact test of A3ObsBuilder against training-env
// ground truth. See scripts/README_e2e_obs_check.md for the workflow.
//
// This test is **fixture-gated**: it reads fixture files from
// `$A3_E2E_FIXTURE_DIR`. When that env var is unset (default CI), the test
// body is skipped with GTEST_SKIP. To run it locally:
//
//   source .../env_isaaclab/bin/activate
//   python scripts/dump_a3_obs_groundtruth.py --pkl ... --out /tmp/fx
//   export A3_E2E_FIXTURE_DIR=/tmp/fx
//   ./target/release/run_tests --gtest_filter='A3ObsBuilderEndToEnd*'
//
// Expected fixture layout (see dump_a3_obs_groundtruth.py):
//   ground_truth_obs.bin   [N, 1570] float32   training-env 1570 obs_dict
//   proprio_inputs.bin     [N,   93] float32   raw MuJoCo-policy-view
//                                              (gravity|ang_vel|jp|jv|act)
//   tokenizer_inputs.bin   [N,  640] float32   tokenizer slice per tick
//   meta.json                                  provenance + offsets

#include <gtest/gtest.h>

#include "a3_deploy/a3_obs_builder.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace {

const char* kFixtureEnvVar = "A3_E2E_FIXTURE_DIR";

// Must match `OFS` in scripts/dump_a3_obs_groundtruth.py and the per-step
// layout in a3_obs_builder.cpp (gravity, ang_vel, joint_pos, joint_vel,
// actions at offsets 0, 3, 6, 35, 64).
constexpr std::size_t kOfsGravity  = 0;
constexpr std::size_t kOfsAngVel   = 3;
constexpr std::size_t kOfsJointPos = 6;
constexpr std::size_t kOfsJointVel = 35;
constexpr std::size_t kOfsActions  = 64;
constexpr std::size_t kProprioRowFloats = 93;

constexpr std::size_t kObsDim        = 1570;
constexpr std::size_t kTokenizerDim  = 640;
constexpr std::size_t kProprioDim    = 930;

std::vector<float> ReadBinary(const std::string& path,
                              std::size_t expected_floats) {
  std::ifstream f(path, std::ios::binary | std::ios::ate);
  if (!f.is_open()) {
    ADD_FAILURE() << "cannot open fixture: " << path;
    return {};
  }
  const std::streamsize bytes = f.tellg();
  const std::size_t expected_bytes = expected_floats * sizeof(float);
  if (static_cast<std::size_t>(bytes) != expected_bytes) {
    ADD_FAILURE() << "fixture size mismatch: " << path
                  << " got " << bytes << " bytes, expected "
                  << expected_bytes;
    return {};
  }
  std::vector<float> out(expected_floats);
  f.seekg(0, std::ios::beg);
  f.read(reinterpret_cast<char*>(out.data()), bytes);
  return out;
}

// Minimal JSON num_ticks extractor — avoids pulling in a JSON dep for one
// integer. meta.json is produced by dump_a3_obs_groundtruth.py and always
// contains `"num_ticks": <int>` near the top.
int ReadNumTicks(const std::string& meta_path) {
  std::ifstream f(meta_path);
  if (!f.is_open()) return -1;
  std::stringstream buf;
  buf << f.rdbuf();
  const std::string body = buf.str();
  const std::string key = "\"num_ticks\":";
  const auto p = body.find(key);
  if (p == std::string::npos) return -1;
  std::size_t i = p + key.size();
  while (i < body.size() && std::isspace(static_cast<unsigned char>(body[i])))
    ++i;
  int value = 0;
  bool any = false;
  while (i < body.size() && std::isdigit(static_cast<unsigned char>(body[i]))) {
    value = value * 10 + (body[i] - '0');
    ++i;
    any = true;
  }
  return any ? value : -1;
}

// Bucket disagreements by magnitude — lets the failure message classify
// quickly (layout/perm vs fp drift vs totally wrong).
struct DiffReport {
  double max_abs = 0.0;
  std::size_t argmax = 0;
  std::size_t n_exceeds_1e_minus_6 = 0;
  std::size_t n_exceeds_1e_minus_3 = 0;
  std::size_t n_exceeds_1 = 0;
  std::vector<std::tuple<std::size_t, float, float, float>> first_mismatches;
};

DiffReport Compare1570(const float* actual, const float* gt,
                       double atol) {
  DiffReport r;
  for (std::size_t i = 0; i < kObsDim; ++i) {
    const double a = static_cast<double>(actual[i]);
    const double b = static_cast<double>(gt[i]);
    const double d = std::abs(a - b);
    if (d > r.max_abs) { r.max_abs = d; r.argmax = i; }
    if (d > 1e-6) ++r.n_exceeds_1e_minus_6;
    if (d > 1e-3) ++r.n_exceeds_1e_minus_3;
    if (d > 1.0)  ++r.n_exceeds_1;
    if (d > atol && r.first_mismatches.size() < 5) {
      r.first_mismatches.emplace_back(
          i, actual[i], gt[i], static_cast<float>(d));
    }
  }
  return r;
}

}  // namespace

class A3ObsBuilderEndToEndFixture : public ::testing::Test {
 protected:
  void SetUp() override {
    const char* fixture_env = std::getenv(kFixtureEnvVar);
    if (fixture_env == nullptr || fixture_env[0] == '\0') {
      GTEST_SKIP() << kFixtureEnvVar
                   << " is unset — see scripts/README_e2e_obs_check.md for "
                      "how to generate fixtures.";
      return;
    }
    fixture_dir_ = fixture_env;
    std::filesystem::path fx(fixture_dir_);
    meta_path_      = (fx / "meta.json").string();
    gt_path_        = (fx / "ground_truth_obs.bin").string();
    proprio_path_   = (fx / "proprio_inputs.bin").string();
    tokenizer_path_ = (fx / "tokenizer_inputs.bin").string();

    num_ticks_ = ReadNumTicks(meta_path_);
    ASSERT_GT(num_ticks_, 0) << "failed to parse num_ticks from " << meta_path_;

    gt_buf_        = ReadBinary(gt_path_,
                                static_cast<std::size_t>(num_ticks_) * kObsDim);
    proprio_buf_   = ReadBinary(proprio_path_,
                                static_cast<std::size_t>(num_ticks_) *
                                kProprioRowFloats);
    tokenizer_buf_ = ReadBinary(tokenizer_path_,
                                static_cast<std::size_t>(num_ticks_) *
                                kTokenizerDim);

    ASSERT_EQ(gt_buf_.size(),
              static_cast<std::size_t>(num_ticks_) * kObsDim);
    ASSERT_EQ(proprio_buf_.size(),
              static_cast<std::size_t>(num_ticks_) * kProprioRowFloats);
    ASSERT_EQ(tokenizer_buf_.size(),
              static_cast<std::size_t>(num_ticks_) * kTokenizerDim);
  }

  std::string fixture_dir_;
  std::string meta_path_, gt_path_, proprio_path_, tokenizer_path_;
  int num_ticks_ = 0;
  std::vector<float> gt_buf_, proprio_buf_, tokenizer_buf_;
};

// Main bit-exact check. Feeds the dumped raw MuJoCo-policy-view components
// into A3ObsBuilder tick by tick and asserts BuildObsDict output matches
// the training-env ground truth to within `kAtol` (default 0.0 — bit-exact).
TEST_F(A3ObsBuilderEndToEndFixture, BuildObsDictMatchesTrainingGroundTruth) {
  // Tolerance policy: deploy-time C++ runs float32 end-to-end, training dumps
  // float32 too. No quantization, so true bit-exact (0.0) is reachable in
  // principle. If the test fails with a small consistent diff it likely
  // indicates fp32 accumulation order, not a layout bug — still worth
  // reporting, hence kAtol defaults to 0.0.
  const double kAtol = 0.0;

  a3_deploy::A3ObsBuilder builder;

  // Running tracker of the worst tick so the first failure message is
  // informative even if many ticks drift.
  double run_max_abs = 0.0;
  std::size_t run_max_tick = 0;
  std::size_t run_max_idx  = 0;
  std::size_t ticks_with_diff_gt_atol = 0;

  for (int t = 0; t < num_ticks_; ++t) {
    const float* proprio_row   = proprio_buf_.data() + t * kProprioRowFloats;
    const float* tokenizer_row = tokenizer_buf_.data() + t * kTokenizerDim;
    const float* gt_row        = gt_buf_.data() + t * kObsDim;

    // --- Extract the 5 raw MuJoCo-policy-view components -------------
    std::array<double, 3> gravity{}, ang_vel{};
    for (int i = 0; i < 3; ++i) {
      gravity[i] = proprio_row[kOfsGravity + i];
      ang_vel[i] = proprio_row[kOfsAngVel + i];
    }
    std::array<double, 29> jp{}, jv{}, act{};
    for (int i = 0; i < 29; ++i) {
      jp[i]  = proprio_row[kOfsJointPos + i];
      jv[i]  = proprio_row[kOfsJointVel + i];
      act[i] = proprio_row[kOfsActions  + i];
    }

    // --- Drive the obs builder exactly as A3PolicyDriver's callback does:
    //     the `actions` slot is the PREVIOUS tick's raw action (stored via
    //     RememberAction). Feed `act` through LastAction-style path.
    std::array<double, 29> prev_action = builder.LastAction();
    builder.PushProprioception(gravity, ang_vel, jp, jv, prev_action);

    std::array<float, 1570> out{};
    builder.BuildObsDict(tokenizer_row, out);

    // Stash `act` for next tick's proprio — emulates
    //   raw_action_t = net.infer(obs_t); builder.RememberAction(raw_action_t)
    // but using the fixture's recorded raw action instead of running the
    // policy net.
    std::array<float, 29> act_f32{};
    for (int i = 0; i < 29; ++i) act_f32[i] = static_cast<float>(act[i]);
    builder.RememberAction(act_f32);

    // --- Compare -----------------------------------------------------
    DiffReport r = Compare1570(out.data(), gt_row, kAtol);
    if (r.max_abs > run_max_abs) {
      run_max_abs = r.max_abs;
      run_max_tick = static_cast<std::size_t>(t);
      run_max_idx  = r.argmax;
    }
    if (r.max_abs > kAtol) {
      ++ticks_with_diff_gt_atol;
      if (ticks_with_diff_gt_atol <= 3) {
        std::ostringstream oss;
        oss << "tick " << t << " max_abs=" << std::setprecision(6)
            << r.max_abs << " at idx " << r.argmax
            << "  >1e-6: " << r.n_exceeds_1e_minus_6
            << "  >1e-3: " << r.n_exceeds_1e_minus_3
            << "  >1: "    << r.n_exceeds_1;
        for (const auto& m : r.first_mismatches) {
          oss << "\n  [" << std::get<0>(m) << "] c++=" << std::get<1>(m)
              << "  gt=" << std::get<2>(m)
              << "  |diff|=" << std::get<3>(m);
        }
        ADD_FAILURE() << oss.str();
      }
    }
  }

  std::cout << "[  DIFF    ] worst tick=" << run_max_tick
            << "  worst idx=" << run_max_idx
            << "  max_abs=" << std::setprecision(6) << run_max_abs
            << "  ticks with diff > " << kAtol << ": "
            << ticks_with_diff_gt_atol << " / " << num_ticks_ << std::endl;

  EXPECT_EQ(ticks_with_diff_gt_atol, 0u)
      << "End-to-end obs_dict does not match training ground truth at atol="
      << kAtol << ". See output above for per-tick diffs; the idx ranges "
      << "map to the 1570-float obs layout (tokenizer [0..639], proprio "
      << "[640..1569] with 10 history frames × 93 floats each — see "
      << "notes/a3_onnx_io_spec.md).";
}

// Sanity sub-test: verify that the tokenizer prefix ALONE (first 640 of the
// output) always matches the fixture's tokenizer_inputs row. This is a
// single-assertion regression guard — if this fails while the main test
// also fails, the problem is on the tokenizer side (unlikely given PR 9's
// selfcheck); if only this passes while the main fails, the problem is
// localised to the 930-proprio half.
TEST_F(A3ObsBuilderEndToEndFixture, TokenizerPrefixAlwaysMatchesInput) {
  a3_deploy::A3ObsBuilder builder;
  // Push one dummy frame so history is seeded — otherwise BuildObsDict's
  // proprio half is zero-filled, which is fine for this prefix-only check.
  std::array<double, 3> z3{};
  std::array<double, 29> z29{};
  builder.PushProprioception(z3, z3, z29, z29, z29);

  std::size_t n_bad_ticks = 0;
  for (int t = 0; t < num_ticks_; ++t) {
    const float* tokenizer_row = tokenizer_buf_.data() + t * kTokenizerDim;
    std::array<float, 1570> out{};
    builder.BuildObsDict(tokenizer_row, out);
    for (std::size_t i = 0; i < kTokenizerDim; ++i) {
      if (out[i] != tokenizer_row[i]) {
        ++n_bad_ticks;
        if (n_bad_ticks <= 1) {
          ADD_FAILURE() << "tokenizer prefix mismatch at tick " << t
                        << " idx " << i << ": out=" << out[i]
                        << " expected=" << tokenizer_row[i];
        }
        break;
      }
    }
  }
  EXPECT_EQ(n_bad_ticks, 0u);
}
