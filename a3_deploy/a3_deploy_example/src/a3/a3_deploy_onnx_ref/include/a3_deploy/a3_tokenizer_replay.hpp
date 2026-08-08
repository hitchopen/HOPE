// Copyright (c) 2026, AgiBot Inc. All rights reserved.
// 对应 notes/a3_pr9_spec.md §2 (NEW C++ headers)
//
// A3TokenizerReplay: loads a pre-baked "reference motion" tokenizer stream from
// disk (produced offline by scripts/offline_export_a3_tokenizer_bin.py — see
// notes/a3_pr9_spec.md §4) and serves one 640-float tokenizer slice per policy
// tick via At(tick_idx). The bin file is a plain `[num_ticks, 640]` float32
// little-endian C-contiguous blob; shape + metadata live in a sibling
// meta.json. This class is intentionally dependency-light (just std + yaml-cpp
// for meta parsing) so it can be unit-tested without a full backend.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace a3_deploy {

// Number of float32 values per tokenizer slice. Fixed by the ONNX export
// schema — see notes/a3_onnx_io_spec.md §"1570-dim obs_dict layout":
//   [command_multi_future_nonflat(580) | motion_anchor_ori_b_mf_nonflat(60)].
inline constexpr std::size_t kA3TokenizerFloatsPerTick = 640;

// Bytes per slice (float32 little-endian).
inline constexpr std::size_t kA3TokenizerBytesPerTick =
    kA3TokenizerFloatsPerTick * sizeof(float);

// Policy when At() is asked for a tick index beyond the last dumped frame.
enum class OnEndPolicy {
  kHoldLast,  // At(n>=N) returns the last valid slice. Default.
  kWrap,      // At(n) == At(n % N).
  kStop,      // At(n>=N) returns nullptr.
};

// Metadata parsed from meta.json alongside the bin file. Fields are informative
// only — the C++ side is agnostic to their semantics beyond num_ticks.
struct A3TokenizerReplayMeta {
  std::size_t num_ticks = 0;          // Required. Matches bin size / 640 / 4.
  std::int64_t dt_ns = 0;             // Expected 20_000_000 (50Hz).
  std::string pkl_source;             // Informative.
  std::string clip_name;              // Informative.
  std::string checkpoint_path;        // Informative.
  std::string generation_timestamp;   // ISO-8601 string, informative.
  std::string git_commit;             // Informative.

  // Initial robot state from IsaacLab eval reset (motion clip frame 0).
  // Populated from meta.json "initial_state" block. If absent, vectors are
  // empty and consumers should use their own default initial state.
  bool has_initial_state = false;
  std::array<double, 3> init_root_pos{};
  std::array<double, 4> init_root_quat_wxyz{1, 0, 0, 0};
  std::array<double, 3> init_root_lin_vel{};
  std::array<double, 3> init_root_ang_vel{};
  std::array<double, 29> init_joint_pos_mujoco_29{};
  std::array<double, 29> init_joint_vel_mujoco_29{};
};

class A3TokenizerReplay {
 public:
  A3TokenizerReplay() = default;

  // Loads `bin_path` (raw float32 blob, N*640 floats, little-endian) and
  // `meta_path` (JSON-as-YAML parsed by yaml-cpp). Returns false on any error
  // (missing file, bin size not a multiple of 640*4, meta.num_ticks mismatch,
  // etc.). On failure an error message is written to std::cerr and internal
  // state is left empty.
  bool Load(const std::string& bin_path,
            const std::string& meta_path,
            OnEndPolicy on_end = OnEndPolicy::kHoldLast);

  // Returns a pointer to the 640-float slice for tick `tick_idx`. Out-of-range
  // behaviour depends on the OnEndPolicy chosen at Load-time:
  //   kHoldLast: returns the last slice (same as At(num_ticks - 1)).
  //   kWrap:     returns At(tick_idx % num_ticks).
  //   kStop:     returns nullptr.
  // Returns nullptr if no data has been loaded. The returned pointer remains
  // valid until the next Load() call or destruction.
  const float* At(std::size_t tick_idx) const noexcept;

  std::size_t NumTicks() const noexcept { return meta_.num_ticks; }
  OnEndPolicy GetOnEndPolicy() const noexcept { return on_end_; }
  const A3TokenizerReplayMeta& Meta() const noexcept { return meta_; }

 private:
  // Flattened [num_ticks, 640] float32 buffer. Owned.
  std::vector<float> data_;
  A3TokenizerReplayMeta meta_{};
  OnEndPolicy on_end_ = OnEndPolicy::kHoldLast;
};

}  // namespace a3_deploy
