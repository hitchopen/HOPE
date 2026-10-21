#include "hope_planner_cpp/incoming_trajectory.hpp"
#include "hope_planner_cpp/batch_physics_estimator.hpp"
#include "hope_planner_cpp/spin_estimator.hpp"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
using namespace hope_planner_cpp;
int main(int argc,char**argv){
 if(argc!=3)return 2;
 CommitPredictionConfig p;
 p.observed_physics.drag_k=.1276482540395671;p.observed_physics.magnus_k=.004099236435745766;p.observed_physics.restitution_h=.5716640639694555;p.observed_physics.restitution_v=.9554747160275279;p.observed_physics.table_tangential_gain=.42833593603054454;p.observed_physics.table_friction_cap_mu=.22361815544013583;p.future_physics=p.observed_physics;
 p.estimator.spin_enabled=true;p.estimator.single_flight_only=true;p.estimator.max_sample_gap_s=.02;p.estimator.min_samples=24;p.estimator.min_span_s=.08;p.estimator.post_bounce_min_span_s=.08;p.estimator.post_bounce_exclusion_s=.008;
 p.spin_estimator.window_s=.18;p.future_contact.spin_enabled=true;p.future_contact.magnus_enabled=true;p.future_contact.tangential_gain=.42833593603054454;p.planner.x_hit=std::stod(argv[2]);p.planner.adaptive_predict_horizon=true;
 IncomingTrajectoryConfig c;c.adaptive.enabled=true;c.adaptive.transport_budget_s=.03;c.estimator_max_sample_gap_s=.02;c.direction_fit_samples=16;c.adaptive.max_sample_age_s=.1;c.pre_roll_samples=48;
 IncomingTrajectory tr(c,p);std::ifstream f(argv[1]);std::string line;double timer=0;
 std::cout<<"commit,source,strike,lead,age,count,x,y,z,spin,reason\n"<<std::setprecision(17);
 auto consume=[&](const IncomingTrajectoryUpdate&u){if(!u.snapshot_ready)return false;auto&s=u.snapshot;BatchPhysicsEstimator e(p.observed_physics,p.estimator,p.spin_estimator);SpinEstimator spin(p.spin_estimator);for(size_t i=0;i<s.sample_count;++i){e.push(s.samples[i]);spin.push(s.samples[i]);}auto state=e.estimate();auto w=spin.estimate();FutureContactPredictor pr(p.future_physics,p.planner,p.table,p.future_contact);auto hit=pr.predict(state,p.planner.x_hit,w);std::cout<<s.one_shot.commit_source_time_s<<','<<state.source_time_s<<','<<hit.strike_source_time_s<<','<<hit.strike_source_time_s-s.one_shot.commit_source_time_s<<','<<s.one_shot.commit_source_time_s-s.latest_sample()->source_time_s<<','<<s.sample_count<<','<<hit.ball_position.x()<<','<<hit.ball_position.y()<<','<<hit.ball_position.z()<<','<<w.valid<<','<<(hit.valid?s.commit_reason:state.reason)<<'\n';return true;};
 while(std::getline(f,line)){std::replace(line.begin(),line.end(),',',' ');std::istringstream ss(line);double t,x,y,z,qx,qy,qz,qw; if(!(ss>>t>>x>>y>>z>>qx>>qy>>qz>>qw))continue;BallSample s;s.source_time_s=t;s.source_time_ns=llround(t*1e9);s.receipt_wall_ns=s.source_time_ns;s.position=Vec3(x,y,z);s.orientation=Eigen::Quaterniond(qw,qx,qy,qz);s.orientation_valid=true;if(timer==0)timer=t;while(timer<t){if(consume(tr.poll(timer)))return 0;timer+=.001;}if(consume(tr.observe(s)))return 0;}
 return 0;
}
