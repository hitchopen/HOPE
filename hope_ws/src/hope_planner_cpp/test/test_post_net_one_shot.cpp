#include "hope_planner_cpp/post_net_one_shot.hpp"

#include <gtest/gtest.h>

namespace hope_planner_cpp {
namespace {

BallSample sample(double time_s, double x) {
  BallSample value;
  value.source_time_s = time_s;
  value.position = Vec3(x, -0.5, 0.4);
  return value;
}

TEST(PostNetOneShot, CommitsExactlyOnceAtFixedPostNetTime) {
  PostNetOneShot trigger(1.37, 0.05);
  EXPECT_FALSE(trigger.observe(sample(10.00, 1.50)).net_crossed);
  const auto crossing = trigger.observe(sample(10.02, 1.30));
  ASSERT_TRUE(crossing.net_crossed);
  EXPECT_NEAR(crossing.net_cross_source_time_s, 10.013, 1.0e-12);
  EXPECT_FALSE(crossing.commit_due);

  EXPECT_FALSE(trigger.observe(sample(10.06, 1.10)).commit_due);
  const auto due = trigger.observe(sample(10.07, 1.00));
  EXPECT_TRUE(due.commit_due);
  EXPECT_EQ(due.flight_sequence, 1U);
  trigger.mark_committed();
  EXPECT_FALSE(trigger.observe(sample(10.10, 0.90)).commit_due);
}

TEST(PostNetOneShot, RearmsOnlyAfterAnOutgoingNetCrossing) {
  PostNetOneShot trigger(1.37, 0.0);
  trigger.observe(sample(20.00, 1.50));
  auto event = trigger.observe(sample(20.01, 1.20));
  ASSERT_TRUE(event.net_crossed);
  ASSERT_TRUE(event.commit_due);
  trigger.mark_committed();

  // Noise below the net does not create another flight.
  trigger.observe(sample(20.02, 1.30));
  event = trigger.observe(sample(20.03, 1.10));
  EXPECT_FALSE(event.net_crossed);
  EXPECT_EQ(trigger.flight_sequence(), 1U);

  // The outgoing ball crosses +X, then the next incoming crossing is flight 2.
  trigger.observe(sample(20.04, 1.50));
  event = trigger.observe(sample(20.05, 1.20));
  EXPECT_TRUE(event.net_crossed);
  EXPECT_EQ(event.flight_sequence, 2U);
}

}  // namespace
}  // namespace hope_planner_cpp
