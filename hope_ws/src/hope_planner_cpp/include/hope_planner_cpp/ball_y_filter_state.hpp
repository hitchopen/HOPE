#pragma once

#include "hope_planner_cpp/ball_y_filter.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>

namespace hope_planner_cpp {
// Small versioned text receipt: version enabled min_y_m max_y_m.
inline BallYFilter load_ball_y_filter(const std::string& path, BallYFilter defaults) {
  if (path.empty() || !std::filesystem::exists(path)) return defaults;
  std::ifstream input(path);
  int version = 0, enabled = -1;
  BallYFilter state;
  std::string extra;
  if (!(input >> version >> enabled >> state.min_y >> state.max_y) ||
      (input >> extra) || version != 1 || (enabled != 0 && enabled != 1) ||
      !state.valid_bounds()) {
    throw std::runtime_error("Invalid saved ball y filter state: " + path);
  }
  state.enabled = enabled != 0;
  return state;
}

inline void save_ball_y_filter(const std::string& path, const BallYFilter& state) {
  if (path.empty()) return;
  if (!state.valid_bounds()) throw std::invalid_argument("Invalid ball y filter bounds");
  const auto temporary = path + ".tmp";
  try {
    std::ofstream output(temporary, std::ios::trunc);
    output << "1 " << (state.enabled ? 1 : 0) << ' ' << std::setprecision(17)
           << state.min_y << ' ' << state.max_y << '\n';
    output.close();
    if (!output) throw std::runtime_error("Cannot save ball y filter state");
    std::filesystem::rename(temporary, path);
  } catch (...) {
    std::error_code ignored;
    std::filesystem::remove(temporary, ignored);
    throw;
  }
}
}  // namespace hope_planner_cpp
