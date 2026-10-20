#pragma once
#include "hope_planner_cpp/types.hpp"
#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <vector>
namespace hope_planner_cpp {
// Keep int64 acquisition stamps exact. Rejected angular NaNs remain input data
// for the trusted-spin resolver; only invalid XYZ/time rows are format errors.
inline BallSample parse_flight_observation_csv(std::string line) {
  std::replace(line.begin(), line.end(), ',', ' ');
  std::istringstream input(line);
  std::vector<std::string> fields;
  std::string token;
  while (input >> token)
    fields.push_back(token);
  if (fields.size() != 8)
    throw std::invalid_argument("observation CSV requires 8 fields");
  std::size_t used = 0;
  BallSample sample;
  sample.source_time_ns = std::stoll(fields[0], &used);
  if (used != fields[0].size())
    throw std::invalid_argument("invalid exact acquisition stamp");
  auto number = [&](std::size_t i) {
    std::size_t count = 0;
    double v = std::stod(fields[i], &count);
    if (count != fields[i].size())
      throw std::invalid_argument("invalid numeric observation field");
    return v;
  };
  sample.position = Vec3(number(1), number(2), number(3));
  if (!sample.position.allFinite())
    throw std::invalid_argument("invalid measured position");
  if (fields[4] != "0" && fields[4] != "1")
    throw std::invalid_argument("omega validity must be 0 or 1");
  sample.producer_omega_valid = fields[4] == "1";
  sample.producer_omega_rad_s = Vec3(number(5), number(6), number(7));
  sample.producer_omega_provenance = "recorded_producer_validity";
  return sample;
}
} // namespace hope_planner_cpp
