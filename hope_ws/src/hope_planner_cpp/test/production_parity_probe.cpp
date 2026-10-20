#include "hope_planner_cpp/racket_target_planner.hpp"
#include "hope_planner_cpp/trajectory_predictor.hpp"
#include "hope_planner_cpp/spin_estimator.hpp"
#include "hope_planner_cpp/batch_physics_estimator.hpp"
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
  std::cout<<"],\"all_spin_modes\":["; comma=false;
  for (auto mode : {SpinPhysicsMode::kLegacyNoSpin, SpinPhysicsMode::kNakashimaBounce,
       SpinPhysicsMode::kNakashimaBounceAndMagnus, SpinPhysicsMode::kVenueGripBounce,
       SpinPhysicsMode::kVenueGripBounceAndMagnus})
    for(double z : {-.02,.02,.2,.6}) for(double y : {-1.6,-.7,.05})
      for(double vz : {-2.,0.,1.}) for(double spin : {0.,30.,-60.}) {
        BallState state;state.valid=true;state.position=Vec3(1.2,y,z);
        state.velocity=Vec3(-4.,.2,vz);state.source_time_s=123.;
        PlannerConfig config;config.adaptive_predict_horizon=true;
        const auto result=TrajectoryPredictor({},config,{}).predict_with_spin(
          state,.15,Vec3(0.,spin,0.),mode);
        if(comma)std::cout<<',';comma=true;
        std::cout<<'['<<int(mode)<<','<<result.valid<<','<<result.predicted_bounces
                 <<','<<result.strike_source_time_s;
        for(const auto& v : {result.ball_position,result.ball_velocity})
          for(int j=0;j<3;++j)std::cout<<','<<v[j];
        std::cout<<']';
      }
  std::cout<<"],\"spin_and_batch\":[";comma=false;
  for (double rate : {150.,300.}) {
    SpinEstimator spin;
    BatchPhysicsEstimator batch({},{});
    for (int i=0;i<70;++i) {
      BallSample sample;sample.source_time_s=i/rate;
      if(i>=30)sample.source_time_s+=.04; // alias/gap edge, retained legacy behavior
      const double t=sample.source_time_s;
      sample.position=Vec3(1.7-3*t,-.7,.8-4.905*t*t);
      sample.orientation=Eigen::Quaterniond(Eigen::AngleAxisd(30*t,Vec3::UnitY()));
      sample.orientation_valid=(i%13!=0);
      spin.push(sample);batch.push(sample);
      const auto omega=spin.estimate();const auto state=batch.estimate();
      if(comma)std::cout<<',';comma=true;
      std::cout<<'['<<omega.valid<<','<<omega.coherence<<','<<state.valid;
      for(const auto& v : {omega.omega_rad_s,state.position,state.velocity})
        for(int j=0;j<3;++j)std::cout<<','<<v[j];
      std::cout<<']';
    }
  }
  std::cout<<"]}\n";
}
