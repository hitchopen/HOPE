// Copyright (c) 2026, AgiBot Inc. All rights reserved.
#include "a3_deploy/a3_motion_library.hpp"

#include <algorithm>
#include <iostream>
#include <utility>

namespace a3_deploy {
namespace {

bool AppendCsvPathsFromDir(const std::string& motion_dir_str,
                           std::vector<std::filesystem::path>& paths) {
  if (motion_dir_str.empty()) return true;
  const std::filesystem::path motion_dir(motion_dir_str);
  std::error_code ec;
  if (!std::filesystem::exists(motion_dir, ec) || ec ||
      !std::filesystem::is_directory(motion_dir, ec) || ec) {
    std::cerr << "A3MotionLibrary: motion_dir is not a directory: "
              << motion_dir.string();
    if (ec) std::cerr << " (" << ec.message() << ")";
    std::cerr << "\n";
    return false;
  }

  std::vector<std::filesystem::path> dir_paths;
  for (const auto& entry : std::filesystem::directory_iterator(motion_dir)) {
    if (!entry.is_regular_file(ec) || ec) {
      ec.clear();
      continue;
    }
    const auto path = entry.path();
    if (path.extension() == ".csv") dir_paths.push_back(path);
  }
  std::sort(dir_paths.begin(), dir_paths.end(),
            [](const auto& a, const auto& b) {
              return a.filename().string() < b.filename().string();
            });
  paths.insert(paths.end(), dir_paths.begin(), dir_paths.end());
  return true;
}

}  // namespace

std::size_t WrapMotionIndex(std::size_t current,
                            int delta,
                            std::size_t count) noexcept {
  if (count == 0) return 0;
  const long long n = static_cast<long long>(count);
  long long idx = static_cast<long long>(current % count) +
                  static_cast<long long>(delta);
  idx %= n;
  if (idx < 0) idx += n;
  return static_cast<std::size_t>(idx);
}

bool A3MotionLibrary::LoadOne(
    const std::filesystem::path& path,
    const A3CsvMotionReferenceOptions& reference_options) {
  std::error_code ec;
  if (!std::filesystem::exists(path, ec) || ec) {
    std::cerr << "A3MotionLibrary: CSV does not exist: " << path.string();
    if (ec) std::cerr << " (" << ec.message() << ")";
    std::cerr << "\n";
    return false;
  }
  ec.clear();
  if (!std::filesystem::is_regular_file(path, ec) || ec) {
    std::cerr << "A3MotionLibrary: expected regular CSV file: "
              << path.string();
    if (ec) std::cerr << " (" << ec.message() << ")";
    std::cerr << "\n";
    return false;
  }
  if (path.extension() != ".csv") {
    std::cerr << "A3MotionLibrary: motion path must end with .csv: "
              << path.string() << "\n";
    return false;
  }

  A3MotionClip clip;
  clip.name = path.stem().string();
  clip.path = path;
  if (!clip.reference.Load(path.string(), reference_options)) {
    std::cerr << "A3MotionLibrary: failed to load motion CSV: "
              << path.string() << "\n";
    return false;
  }
  clips_.push_back(std::move(clip));
  return true;
}

bool A3MotionLibrary::Load(const A3MotionLibraryOptions& options) {
  clips_.clear();
  initial_index_ = 0;

  std::vector<std::filesystem::path> paths;
  if (!options.motion_dir.empty()) {
    if (!AppendCsvPathsFromDir(options.motion_dir, paths)) return false;
  }

  if (paths.empty() && !options.csv_path.empty()) {
    if (!options.motion_dir.empty()) {
      std::cerr << "A3MotionLibrary: motion_dir has no CSV files; falling "
                   "back to reference_motion.csv_path\n";
    }
    paths.push_back(std::filesystem::path(options.csv_path));
  }

  for (const auto& extra_dir : options.extra_motion_dirs) {
    if (!AppendCsvPathsFromDir(extra_dir, paths)) return false;
  }

  if (paths.empty()) {
    std::cerr << "A3MotionLibrary: configure reference_motion.motion_dir or "
                 "reference_motion.csv_path\n";
    return false;
  }
  for (const auto& path : paths) {
    if (!LoadOne(path, options.reference_options)) return false;
  }

  if (clips_.empty()) {
    std::cerr << "A3MotionLibrary: no motions loaded\n";
    return false;
  }
  if (options.initial_index >= clips_.size()) {
    std::cerr << "A3MotionLibrary: reference_motion.initial_index="
              << options.initial_index << " out of range for "
              << clips_.size() << " loaded motions\n";
    return false;
  }
  initial_index_ = options.initial_index;
  return true;
}

std::vector<std::string> A3MotionLibrary::Names() const {
  std::vector<std::string> names;
  names.reserve(clips_.size());
  for (const auto& clip : clips_) names.push_back(clip.name);
  return names;
}

}  // namespace a3_deploy
