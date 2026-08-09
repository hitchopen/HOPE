// Copyright (c) 2026, AgiBot Inc. All rights reserved.
#pragma once

#include "a3_deploy/a3_csv_motion_reference.hpp"

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace a3_deploy {

struct A3MotionLibraryOptions {
  std::string motion_dir;
  std::vector<std::string> extra_motion_dirs;
  std::string csv_path;
  std::size_t initial_index = 0;
  A3CsvMotionReferenceOptions reference_options{};
};

struct A3MotionClip {
  std::string name;
  std::filesystem::path path;
  A3CsvMotionReference reference;
};

std::size_t WrapMotionIndex(std::size_t current,
                            int delta,
                            std::size_t count) noexcept;

class A3MotionLibrary {
 public:
  bool Load(const A3MotionLibraryOptions& options);

  std::size_t Size() const noexcept { return clips_.size(); }
  bool Empty() const noexcept { return clips_.empty(); }
  std::size_t InitialIndex() const noexcept { return initial_index_; }

  const A3MotionClip& Clip(std::size_t index) const noexcept {
    return clips_[index];
  }

  const A3CsvMotionReference& Reference(std::size_t index) const noexcept {
    return clips_[index].reference;
  }

  std::vector<std::string> Names() const;

 private:
  bool LoadOne(const std::filesystem::path& path,
               const A3CsvMotionReferenceOptions& reference_options);

  std::vector<A3MotionClip> clips_;
  std::size_t initial_index_ = 0;
};

}  // namespace a3_deploy
