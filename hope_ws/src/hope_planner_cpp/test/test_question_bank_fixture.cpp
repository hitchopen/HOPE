#include "hope_planner_cpp/question_bank_fixture.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <functional>
#include <string>

namespace hope_planner_cpp {
namespace {

constexpr char kHeader[] =
    "fixture_contract,bank_sha256,receipt_sha256,flight_id,bank_row_id,clip,"
    "reach_level,swing_foot_sign,contact_offset_x,contact_offset_y,"
    "contact_actor_z,target_vx,target_vy,target_vz,target_nx,target_ny,"
    "target_nz,incoming_vx,incoming_vy,incoming_vz,tts_s,intended_land_x,"
    "intended_land_y,outgoing_vx,outgoing_vy,outgoing_vz,launch_x,launch_y,"
    "launch_z_table,launch_vx,launch_vy,launch_vz\n";

std::filesystem::path write_fixture(const std::string& row) {
  const auto path = std::filesystem::temp_directory_path() /
      ("hope_question_fixture_" + std::to_string(
          std::hash<std::string>{}(row)) + ".csv");
  std::ofstream stream(path);
  stream << kHeader << row;
  return path;
}

TEST(QuestionBankFixture, LoadsStrictContiguousL0Row) {
  const auto path = write_fixture(
      "contract,bank,receipt,1,13511,0,0,0,0.58,-0.33,1.07,"
      "1,0,0,1,0,0,-1.7,0.1,-1,0.5,2.2,-0.1,2,0,1,"
      "2.4,-1.3,0.4,-3.7,0.3,1.7\n");
  const auto fixture = QuestionBankFixture::load_csv(
      path.string(), "contract", "bank", "receipt", 1);
  ASSERT_EQ(fixture.size(), 1u);
  ASSERT_NE(fixture.row_for_flight(1), nullptr);
  EXPECT_EQ(fixture.row_for_flight(1)->bank_row_id, 13511);
  EXPECT_EQ(fixture.row_for_flight(2), nullptr);
  std::filesystem::remove(path);
}

TEST(QuestionBankFixture, RejectsNonL0OrArtifactMismatch) {
  const auto path = write_fixture(
      "contract,bank,receipt,1,13511,0,1,1,0.58,-0.33,1.07,"
      "1,0,0,1,0,0,-1.7,0.1,-1,0.5,2.2,-0.1,2,0,1,"
      "2.4,-1.3,0.4,-3.7,0.3,1.7\n");
  EXPECT_THROW(
      QuestionBankFixture::load_csv(
          path.string(), "contract", "bank", "receipt", 1),
      std::invalid_argument);
  EXPECT_THROW(
      QuestionBankFixture::load_csv(
          path.string(), "contract", "wrong", "receipt", 1),
      std::invalid_argument);
  std::filesystem::remove(path);
}

}  // namespace
}  // namespace hope_planner_cpp
