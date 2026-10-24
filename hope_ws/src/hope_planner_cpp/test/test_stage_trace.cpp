#include "hope_planner_cpp/bounded_stage_trace.hpp"
#include <gtest/gtest.h>
#include <filesystem>
#include <sstream>
#include <unistd.h>

TEST(StageTrace, BoundedOverwriteRetainsExactNanosecondsAndReportsLoss) {
  const auto path = std::filesystem::temp_directory_path() /
      ("hope-stage-trace-" + std::to_string(getpid()));
  hope_planner_cpp::BoundedStageTrace trace;
  EXPECT_FALSE(trace.enabled());
  trace.configure(path.string(), 2);
  for (int i = 0; i < 4; ++i)
    trace.record("receipt", 1792412345678901234LL + i, i, 3, 100 + i, 200 + i, 7, "why\nnow");
  trace.flush();
  std::ifstream file(path);
  std::stringstream data; data << file.rdbuf();
  const auto text = data.str();
  EXPECT_NE(text.find("capacity=2 total=4 overwritten=2"), std::string::npos);
  EXPECT_EQ(text.find("1792412345678901234"), std::string::npos);
  EXPECT_NE(text.find("1792412345678901236"), std::string::npos);
  EXPECT_NE(text.find("1792412345678901237"), std::string::npos);
  EXPECT_NE(text.find("why_now"), std::string::npos);
  std::filesystem::remove(path);
}

TEST(StageTrace, BadDestinationAndUnboundedCapacityFailBeforeWork) {
  hope_planner_cpp::BoundedStageTrace trace;
  EXPECT_THROW(trace.configure("/nonexistent/hope/trace"), std::invalid_argument);
  EXPECT_THROW(trace.configure("/tmp/unused-stage", 65537), std::invalid_argument);
  EXPECT_THROW(trace.configure("/tmp/unused-stage", 0), std::invalid_argument);
  EXPECT_FALSE(trace.enabled());
}
