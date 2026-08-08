// Copyright (c) 2026, AgiBot Inc. All rights reserved.
// 对应 notes/a3_pr9_spec.md §2 (NEW C++ headers)
#include "a3_deploy/a3_tokenizer_replay.hpp"

#include <yaml-cpp/yaml.h>

#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>

namespace a3_deploy {

namespace {

bool ReadBinary(const std::string& path, std::vector<float>& out) {
  std::ifstream f(path, std::ios::binary | std::ios::ate);
  if (!f.is_open()) {
    std::cerr << "[a3_tokenizer_replay] failed to open bin: " << path
              << std::endl;
    return false;
  }
  const std::streamsize bytes = f.tellg();
  if (bytes < 0) {
    std::cerr << "[a3_tokenizer_replay] tellg failed on: " << path << std::endl;
    return false;
  }
  if (bytes == 0) {
    std::cerr << "[a3_tokenizer_replay] empty bin file: " << path << std::endl;
    return false;
  }
  if (static_cast<std::size_t>(bytes) % kA3TokenizerBytesPerTick != 0) {
    std::cerr << "[a3_tokenizer_replay] bin size " << bytes
              << " not a multiple of " << kA3TokenizerBytesPerTick
              << " (640 floats)" << std::endl;
    return false;
  }
  const std::size_t n_floats =
      static_cast<std::size_t>(bytes) / sizeof(float);
  out.resize(n_floats);
  f.seekg(0, std::ios::beg);
  f.read(reinterpret_cast<char*>(out.data()), bytes);
  if (!f) {
    std::cerr << "[a3_tokenizer_replay] short read on: " << path << std::endl;
    out.clear();
    return false;
  }
  return true;
}

bool ParseMeta(const std::string& path, A3TokenizerReplayMeta& meta) {
  // yaml-cpp handles JSON-as-YAML cleanly (JSON is a YAML 1.2 subset, and the
  // fields we use are trivial scalars).
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception& e) {
    std::cerr << "[a3_tokenizer_replay] failed to parse meta: " << path
              << " (" << e.what() << ")" << std::endl;
    return false;
  }
  if (!root || !root.IsMap()) {
    std::cerr << "[a3_tokenizer_replay] meta root is not a mapping: " << path
              << std::endl;
    return false;
  }
  if (!root["num_ticks"]) {
    std::cerr << "[a3_tokenizer_replay] meta missing num_ticks: " << path
              << std::endl;
    return false;
  }
  try {
    meta.num_ticks = root["num_ticks"].as<std::size_t>();
  } catch (const YAML::Exception& e) {
    std::cerr << "[a3_tokenizer_replay] num_ticks not a non-negative integer: "
              << e.what() << std::endl;
    return false;
  }
  // Optional fields — tolerate absence.
  if (root["dt_ns"])                meta.dt_ns                = root["dt_ns"].as<std::int64_t>(0);
  if (root["pkl_source"])           meta.pkl_source           = root["pkl_source"].as<std::string>("");
  if (root["clip_name"])            meta.clip_name            = root["clip_name"].as<std::string>("");
  if (root["checkpoint_path"])      meta.checkpoint_path      = root["checkpoint_path"].as<std::string>("");
  if (root["generation_timestamp"]) meta.generation_timestamp = root["generation_timestamp"].as<std::string>("");
  if (root["git_commit"])           meta.git_commit           = root["git_commit"].as<std::string>("");

  // Parse optional exported initial_state block.
  if (root["initial_state"] && root["initial_state"].IsMap()) {
    const auto& is = root["initial_state"];
    auto read_arr = [](const YAML::Node& node, double* out, std::size_t n) {
      if (!node || !node.IsSequence() || node.size() != n) return false;
      for (std::size_t i = 0; i < n; ++i) out[i] = node[i].as<double>();
      return true;
    };
    bool ok = true;
    ok &= read_arr(is["root_pos"], meta.init_root_pos.data(), 3);
    ok &= read_arr(is["root_quat_wxyz"], meta.init_root_quat_wxyz.data(), 4);
    ok &= read_arr(is["root_lin_vel"], meta.init_root_lin_vel.data(), 3);
    ok &= read_arr(is["root_ang_vel"], meta.init_root_ang_vel.data(), 3);
    ok &= read_arr(is["joint_pos_mujoco_29"], meta.init_joint_pos_mujoco_29.data(), 29);
    ok &= read_arr(is["joint_vel_mujoco_29"], meta.init_joint_vel_mujoco_29.data(), 29);
    meta.has_initial_state = ok;
    if (!ok) {
      std::cerr << "[a3_tokenizer_replay] WARN: initial_state present but "
                   "incomplete; runtime will use its default initial state\n";
    }
  }

  return true;
}

}  // namespace

// ---------------------------------------------------------------------------
bool A3TokenizerReplay::Load(const std::string& bin_path,
                             const std::string& meta_path,
                             OnEndPolicy on_end) {
  data_.clear();
  meta_ = A3TokenizerReplayMeta{};
  on_end_ = on_end;

  if (!ParseMeta(meta_path, meta_)) return false;
  if (meta_.num_ticks == 0) {
    std::cerr << "[a3_tokenizer_replay] meta.num_ticks == 0 in " << meta_path
              << std::endl;
    return false;
  }

  std::vector<float> buf;
  if (!ReadBinary(bin_path, buf)) {
    meta_ = A3TokenizerReplayMeta{};
    return false;
  }

  const std::size_t ticks_from_bin = buf.size() / kA3TokenizerFloatsPerTick;
  if (ticks_from_bin != meta_.num_ticks) {
    std::cerr << "[a3_tokenizer_replay] meta.num_ticks=" << meta_.num_ticks
              << " mismatches bin ticks=" << ticks_from_bin
              << " (bin=" << bin_path << ", meta=" << meta_path << ")"
              << std::endl;
    data_.clear();
    meta_ = A3TokenizerReplayMeta{};
    return false;
  }

  data_ = std::move(buf);
  std::cout << "[a3_tokenizer_replay] loaded " << meta_.num_ticks
            << " ticks (" << (data_.size() * sizeof(float))
            << " bytes) from " << bin_path;
  if (!meta_.clip_name.empty()) std::cout << " clip=" << meta_.clip_name;
  std::cout << std::endl;
  return true;
}

// ---------------------------------------------------------------------------
const float* A3TokenizerReplay::At(std::size_t tick_idx) const noexcept {
  if (meta_.num_ticks == 0 || data_.empty()) return nullptr;
  std::size_t idx = tick_idx;
  if (idx >= meta_.num_ticks) {
    switch (on_end_) {
      case OnEndPolicy::kHoldLast:
        idx = meta_.num_ticks - 1;
        break;
      case OnEndPolicy::kWrap:
        idx = tick_idx % meta_.num_ticks;
        break;
      case OnEndPolicy::kStop:
        return nullptr;
    }
  }
  return data_.data() + idx * kA3TokenizerFloatsPerTick;
}

}  // namespace a3_deploy
