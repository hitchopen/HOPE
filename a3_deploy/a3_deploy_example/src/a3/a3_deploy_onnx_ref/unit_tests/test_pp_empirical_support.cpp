#include "a3_pingpong/pp_empirical_support.hpp"
#include <gtest/gtest.h>
#include <cassert>
#include <iostream>
using namespace a3_pingpong;
TEST(PpEmpiricalSupport, DefaultsIntervalsAndMalformedEvidence) {
  auto none=ParseEmpiricalLateralSupport(YAML::Node());ASSERT_TRUE(!none.enabled);
  ASSERT_TRUE((EffectiveLateralSupport(none,0,{-.68,-.24})==std::array<double,2>{-.68,-.24}));
  auto good=YAML::Load("contract: super_gate3_empirical_lateral_v1\nforehand_y_range: [-.85,-.24]\nbackhand_y_range: [-.24,.60]\nevidence_sha256: aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n");
  auto s=ParseEmpiricalLateralSupport(good);ASSERT_TRUE(s.enabled);
  EXPECT_EQ(EffectiveLateralSupport(s,0,{-.68,-.24}), s.forehand);
  EXPECT_EQ(EffectiveLateralSupport(s,1,{-.24,.28}), s.backhand);
  int rejected=0;
  for(auto key:{"forehand_y_range","backhand_y_range","evidence_sha256","contract"}){
    auto n=YAML::Clone(good);n.remove(key);try{ParseEmpiricalLateralSupport(n);}catch(const std::exception&){rejected++;}
  }
  for(auto bad:{"[0,-1]","[.nan,1]","[0,.inf]","[0]","[0,0]"}){
    auto n=YAML::Clone(good);n["backhand_y_range"]=YAML::Load(bad);try{ParseEmpiricalLateralSupport(n);}catch(const std::exception&){rejected++;}
  }
  ASSERT_TRUE(rejected==9);

}
