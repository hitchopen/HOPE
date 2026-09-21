#pragma once

#include "hope_planner_cpp/types.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace hope_planner_cpp {

inline constexpr const char* kSchema31V5FixtureContract =
    "schema31_gate3_v5_l0_fixture_v1";
inline constexpr const char* kSchema31V5BankSha256 =
    "9a4ff9e89f360e5be25e6ee6df571b30aee93a9f7f88a70ae5a308606e294f7e";
inline constexpr const char* kSchema31V5ReceiptSha256 =
    "347d16d386ba372feb3c8bc4c00d23542f69de62f14ccd4ffcb8a881f5b4b493";

struct QuestionBankFixtureRow {
  std::uint64_t flight_id = 0;
  std::int64_t bank_row_id = -1;
  int clip = -1;
  int reach_level = -1;
  int swing_foot_sign = 0;
  Vec3 contact_position_offset = Vec3::Zero();
  Vec3 target_velocity = Vec3::Zero();
  Vec3 target_normal = Vec3::Zero();
  Vec3 incoming_velocity = Vec3::Zero();
  double time_to_strike_s = 0.0;
  Vec3 intended_landing = Vec3::Zero();
  Vec3 predicted_outgoing_velocity = Vec3::Zero();
  Vec3 launch_position_table = Vec3::Zero();
  Vec3 launch_velocity = Vec3::Zero();
};

class QuestionBankFixture {
 public:
  static QuestionBankFixture load_csv(
      const std::string& path,
      const std::string& expected_contract,
      const std::string& expected_bank_sha256,
      const std::string& expected_receipt_sha256,
      std::size_t expected_flights);

  const QuestionBankFixtureRow* row_for_flight(
      std::uint64_t flight_id) const noexcept;
  std::size_t size() const noexcept { return rows_.size(); }
  const std::string& contract() const noexcept { return contract_; }
  const std::string& bank_sha256() const noexcept { return bank_sha256_; }
  const std::string& receipt_sha256() const noexcept { return receipt_sha256_; }

 private:
  std::string contract_;
  std::string bank_sha256_;
  std::string receipt_sha256_;
  std::vector<QuestionBankFixtureRow> rows_;
};

}  // namespace hope_planner_cpp
