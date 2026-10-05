#include "hope_planner_cpp/question_bank_fixture.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace hope_planner_cpp {
namespace {

std::vector<std::string> split_csv_line(const std::string& line) {
  std::vector<std::string> fields;
  std::istringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) fields.push_back(field);
  if (!line.empty() && line.back() == ',') fields.emplace_back();
  return fields;
}

double parse_finite_double(const std::string& value, const std::string& field) {
  std::size_t consumed = 0;
  const double parsed = std::stod(value, &consumed);
  if (consumed != value.size() || !std::isfinite(parsed)) {
    throw std::invalid_argument("invalid finite fixture field " + field);
  }
  return parsed;
}

std::int64_t parse_integer(const std::string& value, const std::string& field) {
  std::size_t consumed = 0;
  const auto parsed = std::stoll(value, &consumed);
  if (consumed != value.size()) {
    throw std::invalid_argument("invalid integer fixture field " + field);
  }
  return parsed;
}

}  // namespace

QuestionBankFixture QuestionBankFixture::load_csv(
    const std::string& path,
    const std::string& expected_contract,
    const std::string& expected_bank_sha256,
    const std::string& expected_receipt_sha256,
    std::size_t expected_flights) {
  if (path.empty() || expected_contract.empty() ||
      expected_bank_sha256.empty() || expected_receipt_sha256.empty() ||
      expected_flights == 0) {
    throw std::invalid_argument("question fixture binding is incomplete");
  }
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("cannot open question fixture CSV: " + path);
  }
  std::string header_line;
  if (!std::getline(stream, header_line)) {
    throw std::runtime_error("question fixture CSV is empty");
  }
  const auto header = split_csv_line(header_line);
  std::unordered_map<std::string, std::size_t> column;
  for (std::size_t index = 0; index < header.size(); ++index) {
    if (!column.emplace(header[index], index).second) {
      throw std::invalid_argument("duplicate question fixture column: " + header[index]);
    }
  }
  const std::vector<std::string> required{
      "fixture_contract", "bank_sha256", "receipt_sha256", "flight_id",
      "bank_row_id", "clip", "reach_level", "swing_foot_sign",
      "contact_offset_x", "contact_offset_y", "contact_actor_z",
      "target_vx", "target_vy", "target_vz",
      "target_nx", "target_ny", "target_nz",
      "incoming_vx", "incoming_vy", "incoming_vz", "tts_s",
      "intended_land_x", "intended_land_y",
      "outgoing_vx", "outgoing_vy", "outgoing_vz",
      "launch_x", "launch_y", "launch_z_table",
      "launch_vx", "launch_vy", "launch_vz"};
  for (const auto& name : required) {
    if (column.count(name) == 0) {
      throw std::invalid_argument("missing question fixture column: " + name);
    }
  }

  QuestionBankFixture fixture;
  fixture.contract_ = expected_contract;
  fixture.bank_sha256_ = expected_bank_sha256;
  fixture.receipt_sha256_ = expected_receipt_sha256;
  std::string line;
  std::size_t line_number = 1;
  while (std::getline(stream, line)) {
    ++line_number;
    if (line.empty()) continue;
    const auto fields = split_csv_line(line);
    if (fields.size() != header.size()) {
      throw std::invalid_argument(
          "question fixture column count mismatch at line " +
          std::to_string(line_number));
    }
    const auto value = [&](const std::string& name) -> const std::string& {
      return fields[column.at(name)];
    };
    if (value("fixture_contract") != expected_contract ||
        value("bank_sha256") != expected_bank_sha256 ||
        value("receipt_sha256") != expected_receipt_sha256) {
      throw std::invalid_argument(
          "question fixture artifact binding mismatch at line " +
          std::to_string(line_number));
    }

    QuestionBankFixtureRow row;
    row.flight_id = static_cast<std::uint64_t>(
        parse_integer(value("flight_id"), "flight_id"));
    row.bank_row_id = parse_integer(value("bank_row_id"), "bank_row_id");
    row.clip = static_cast<int>(parse_integer(value("clip"), "clip"));
    row.reach_level = static_cast<int>(
        parse_integer(value("reach_level"), "reach_level"));
    row.swing_foot_sign = static_cast<int>(
        parse_integer(value("swing_foot_sign"), "swing_foot_sign"));
    row.contact_position_offset = Vec3(
        parse_finite_double(value("contact_offset_x"), "contact_offset_x"),
        parse_finite_double(value("contact_offset_y"), "contact_offset_y"),
        parse_finite_double(value("contact_actor_z"), "contact_actor_z"));
    row.target_velocity = Vec3(
        parse_finite_double(value("target_vx"), "target_vx"),
        parse_finite_double(value("target_vy"), "target_vy"),
        parse_finite_double(value("target_vz"), "target_vz"));
    row.target_normal = Vec3(
        parse_finite_double(value("target_nx"), "target_nx"),
        parse_finite_double(value("target_ny"), "target_ny"),
        parse_finite_double(value("target_nz"), "target_nz"));
    row.incoming_velocity = Vec3(
        parse_finite_double(value("incoming_vx"), "incoming_vx"),
        parse_finite_double(value("incoming_vy"), "incoming_vy"),
        parse_finite_double(value("incoming_vz"), "incoming_vz"));
    row.time_to_strike_s = parse_finite_double(value("tts_s"), "tts_s");
    row.intended_landing = Vec3(
        parse_finite_double(value("intended_land_x"), "intended_land_x"),
        parse_finite_double(value("intended_land_y"), "intended_land_y"), 0.0);
    row.predicted_outgoing_velocity = Vec3(
        parse_finite_double(value("outgoing_vx"), "outgoing_vx"),
        parse_finite_double(value("outgoing_vy"), "outgoing_vy"),
        parse_finite_double(value("outgoing_vz"), "outgoing_vz"));
    row.launch_position_table = Vec3(
        parse_finite_double(value("launch_x"), "launch_x"),
        parse_finite_double(value("launch_y"), "launch_y"),
        parse_finite_double(value("launch_z_table"), "launch_z_table"));
    row.launch_velocity = Vec3(
        parse_finite_double(value("launch_vx"), "launch_vx"),
        parse_finite_double(value("launch_vy"), "launch_vy"),
        parse_finite_double(value("launch_vz"), "launch_vz"));

    const std::uint64_t expected_flight_id = fixture.rows_.size() + 1;
    if (row.flight_id != expected_flight_id || row.bank_row_id < 0 ||
        (row.clip != 0 && row.clip != 1) || row.reach_level != 0 ||
        row.swing_foot_sign != 0 || row.time_to_strike_s <= 0.0 ||
        row.incoming_velocity.x() >= 0.0 || row.launch_velocity.x() >= 0.0 ||
        row.target_velocity.norm() <= 0.0 ||
        std::abs(row.target_normal.norm() - 1.0) > 1.0e-6 ||
        (row.target_velocity.normalized() - row.target_normal)
                .cwiseAbs().maxCoeff() > 5.0e-7) {
      throw std::invalid_argument(
          "invalid L0 question fixture row at line " +
          std::to_string(line_number));
    }
    fixture.rows_.push_back(std::move(row));
  }
  if (fixture.rows_.size() != expected_flights) {
    throw std::invalid_argument(
        "question fixture flight count mismatch: got " +
        std::to_string(fixture.rows_.size()) + " expected " +
        std::to_string(expected_flights));
  }
  return fixture;
}

const QuestionBankFixtureRow* QuestionBankFixture::row_for_flight(
    std::uint64_t flight_id) const noexcept {
  if (flight_id == 0 || flight_id > rows_.size()) return nullptr;
  const auto& row = rows_[flight_id - 1];
  return row.flight_id == flight_id ? &row : nullptr;
}

}  // namespace hope_planner_cpp
