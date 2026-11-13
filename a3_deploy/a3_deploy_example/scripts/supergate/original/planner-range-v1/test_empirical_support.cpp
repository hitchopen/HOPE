#include "a3_pingpong/pp_empirical_support.hpp"
#include "hope_planner_cpp/swing_side_support.hpp"
#include <cassert>
#include <iostream>
using namespace a3_pingpong;
int main(){
  auto none=ParseEmpiricalLateralSupport(YAML::Node());assert(!none.enabled);
  assert((EffectiveLateralSupport(none,0,{-.68,-.24})==std::array<double,2>{-.68,-.24}));
  auto good=YAML::Load("contract: super_gate3_empirical_lateral_v1\nforehand_y_range: [-.85,-.24]\nbackhand_y_range: [-.24,.60]\nevidence_sha256: aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n");
  auto s=ParseEmpiricalLateralSupport(good);assert(s.enabled);
  for(double home:{-.9,-.7625,-.5}) for(auto band:{s.forehand,s.backhand}){
    for(double dy:{band[0]+1e-9,(band[0]+band[1])/2,band[1]-1e-9}){
      double sign=hope_planner_cpp::select_swing_sign_with_support(home+dy,home,-.24,.04,0,{s.forehand[0],s.forehand[1]},{s.backhand[0],s.backhand[1]});assert(std::isfinite(sign));
      auto effective=EffectiveLateralSupport(s,sign>0?0:1,{-.68,-.24});assert(dy>=effective[0]&&dy<=effective[1]);
    }
  }
  int rejected=0;
  for(auto key:{"forehand_y_range","backhand_y_range","evidence_sha256","contract"}){
    auto n=YAML::Clone(good);n.remove(key);try{ParseEmpiricalLateralSupport(n);}catch(const std::exception&){rejected++;}
  }
  for(auto bad:{"[0,-1]","[.nan,1]","[0,.inf]","[0]","[0,0]"}){
    auto n=YAML::Clone(good);n["backhand_y_range"]=YAML::Load(bad);try{ParseEmpiricalLateralSupport(n);}catch(const std::exception&){rejected++;}
  }
  assert(rejected==9);
  assert(!std::isfinite(hope_planner_cpp::select_swing_sign_with_support(-.7625+.60001,-.7625,-.24,.04,-1,{-.85,-.24},{-.24,.60})));
  std::cout<<"PASS unchanged default, paired live-base interval boundaries, hysteresis support, invalid/missing config rejected\n";
}
