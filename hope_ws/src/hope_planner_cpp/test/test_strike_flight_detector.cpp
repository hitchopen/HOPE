#include "hope_planner_cpp/strike_flight_detector.hpp"
#include <gtest/gtest.h>
using namespace hope_planner_cpp;
namespace {
constexpr std::int64_t origin=1780000000123456789LL;
StrikeFlightUpdate point(StrikeFlightDetector& d,std::int64_t ns,double x,double z=.5) {
  BallSample s;s.source_time_ns=ns;s.position=Vec3(x,-.7,z);return d.observe(s);
}
}
TEST(StrikeFlightDetector, RatesBounceDropoutAndIntegerCrossing) {
  for(int hz:{150,200,300}) {
    StrikeFlightDetector d({});int starts=0,crosses=0;std::int64_t net=0;
    for(int i=0;i<hz/5;++i) {
      const auto ns=origin+std::llround(i*1e9/hz);const double t=double(ns-origin)*1e-9;
      auto u=point(d,ns,1.8-5*t,std::abs(.3-3*t));starts+=u.incoming_started;
      if(u.net_crossed){++crosses;net=u.net_cross_ns;}
      if(t>.09)break; // no post-net observations needed to establish identity
    }
    EXPECT_EQ(starts,1);EXPECT_EQ(crosses,1);EXPECT_NEAR(net-origin,86000000,1);
    EXPECT_FALSE(point(d,origin+100000000,1.3).incoming_started);
  }
}
TEST(StrikeFlightDetector, NetJitterDoesNotDuplicateAndOutgoingRearms) {
  StrikeFlightDetector d({});int starts=0,crosses=0;std::int64_t ns=origin;
  auto feed=[&](double x){auto u=point(d,ns,x);ns+=5000000;starts+=u.incoming_started;crosses+=u.net_crossed;};
  for(double x:{1.5,1.45,1.4,1.36,1.38,1.36,1.35,1.36,1.34})feed(x);
  EXPECT_EQ(starts,1);EXPECT_EQ(crosses,1);
  for(int i=0;i<18;++i)feed(1.34+.04*i);
  for(int i=0;i<22;++i)feed(2.02-.04*i);
  EXPECT_EQ(starts,2);EXPECT_EQ(crosses,2);
}
TEST(StrikeFlightDetector, LateAcquisitionDuplicatesAndContinuity) {
  StrikeFlightDetector d({});point(d,origin,1.25);point(d,origin+5000000,1.225);
  auto u=point(d,origin+10000000,1.2);ASSERT_TRUE(u.incoming_started);ASSERT_TRUE(u.net_crossed);
  EXPECT_TRUE(u.late_acquisition);EXPECT_NEAR(u.net_cross_ns-origin,-24000000,1);
  EXPECT_FALSE(point(d,origin+10000000,1.2).incoming_started);
  EXPECT_TRUE(point(d,origin+400000000,1.).source_epoch_reset);
  d.reset_phase();point(d,origin,1.8);EXPECT_TRUE(point(d,origin-2000000,1.7).source_epoch_reset);
}

TEST(StrikeFlightDetector, RejectsUnregisteredNetGeometry) {
  StrikeFlightDetectorConfig c;c.net_x=1.4;
  EXPECT_THROW(StrikeFlightDetector detector(c),std::invalid_argument);
}
