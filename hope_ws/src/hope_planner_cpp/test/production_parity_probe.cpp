#include "hope_planner_cpp/racket_target_planner.hpp"
#include "hope_planner_cpp/trajectory_predictor.hpp"
#include <iomanip>
#include <iostream>
using namespace hope_planner_cpp;
int main() {
  std::cout<<std::setprecision(17)<<"{\"racket\":[";
  bool comma=false;
  for(double height:{.2,.4,.7}) for(double vy:{-.4,0.,.4}) for(double duration:{.4,.6}) {
    StrikeTarget s;s.valid=true;s.ball_position=Vec3(.15,-.7,height);
    s.ball_velocity=Vec3(-3,vy,-1);s.strike_source_time_s=123.456;
    const auto command=RacketTargetPlanner({}, {}, {}).plan(s,Vec3(2.055,-.7625,0),duration);
    if(comma)std::cout<<',';comma=true;
    std::cout<<'['<<command.valid<<','<<command.clears_net<<','<<command.bypasses_net_posts;
    for(const auto &v:{command.position,command.velocity,command.normal,command.outgoing_ball_velocity})
      for(int j=0;j<3;++j) std::cout<<','<<v[j];
    std::cout<<']';
  }
  std::cout<<"],\"receiving_flight\":[";comma=false;
  for(double vz:{-1.2,-.5,.5}) {
    BallState state;state.valid=true;state.position=Vec3(1.4,-.55,.35);state.velocity=Vec3(-4.2,.35,vz);state.source_time_s=123;
    PlannerConfig config;config.adaptive_predict_horizon=true;config.max_predict_time_cap_s=3.;
    auto target=TrajectoryPredictor({},config,{}).predict(state,.15);
    if(comma)std::cout<<',';comma=true;
    std::cout<<'['<<target.valid<<','<<target.strike_source_time_s;
    for(const auto &v:{target.ball_position,target.ball_velocity})for(int j=0;j<3;++j)std::cout<<','<<v[j];
    std::cout<<']';
  }
  std::cout<<"]}\n";
}
