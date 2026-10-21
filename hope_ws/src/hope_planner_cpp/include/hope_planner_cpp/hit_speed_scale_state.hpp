#pragma once

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>

namespace hope_planner_cpp {
inline double load_hit_speed_scale(const std::string& path, double fallback) {
  if (path.empty() || !std::filesystem::exists(path)) return fallback;
  std::ifstream input(path);
  int version = 0;
  double value = 0;
  std::string extra;
  if (!(input >> version >> value) || (input >> extra) || version != 1 ||
      !std::isfinite(value) || value < 0)
    throw std::runtime_error("Invalid saved hit speed scale: " + path);
  return value;
}

inline void save_hit_speed_scale(const std::string& path, double value) {
  if (path.empty()) return;
  if (!std::isfinite(value) || value < 0)
    throw std::invalid_argument("Invalid hit speed scale");
  const auto temporary = path + ".tmp";
  try {
    std::ofstream output(temporary, std::ios::trunc);
    output << "1 " << std::setprecision(17) << value << '\n';
    output.close();
    if (!output) throw std::runtime_error("Cannot save hit speed scale");
    std::filesystem::rename(temporary, path);
  } catch (...) {
    std::error_code ignored;
    std::filesystem::remove(temporary, ignored);
    throw;
  }
}
}  // namespace hope_planner_cpp
