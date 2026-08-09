#include "hope_planner_cpp/audit_logger.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>

#include <unistd.h>

namespace hope_planner_cpp {
namespace {

std::filesystem::path temporary_csv_path() {
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  return std::filesystem::temp_directory_path() /
      ("hope_planner_audit_" + std::to_string(::getpid()) + "_" +
       std::to_string(stamp) + ".csv");
}

}  // namespace

TEST(AuditLogger, WritesQueuedRowsWithoutTruncatingExistingEvidence) {
  const auto path = temporary_csv_path();
  {
    AuditLogger logger(path.string(), "a,b");
    ASSERT_TRUE(logger.enabled());
    logger.enqueue("1,2");
    logger.stop();
  }

  std::ifstream input(path);
  const std::string contents(
      (std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  EXPECT_EQ(contents, "a,b\n1,2\n");
  EXPECT_THROW(AuditLogger duplicate(path.string(), "new,header"), std::runtime_error);

  std::error_code error;
  std::filesystem::remove(path, error);
}

}  // namespace hope_planner_cpp
