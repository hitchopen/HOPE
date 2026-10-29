// Data-only adapter: all forward and inverse physics runs in production C++.
#include "hope_planner_cpp/racket_target_planner.hpp"
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
using namespace hope_planner_cpp;
void vector_out(const Vec3& v) { for (int i=0;i<3;++i) std::cout << ' ' << v[i]; }
int main() {
  std::cout << std::setprecision(17);
  std::string mode;
  while (std::cin >> mode) {
    std::size_t count;
    if (!(std::cin >> count) || count < 34) return 2;
    std::vector<double> p(count);
    for (double& value : p) if (!(std::cin >> value)) return 3;
    BallPhysics physics;
    PlannerConfig config;
    TableParams table;
    physics.drag_k=p[0]; physics.magnus_k=p[1]; physics.gravity=Vec3(0,0,-p[2]); physics.radius=p[3];
    config.ball_inertia_coeff=p[4]; config.paddle_tangential_gain=p[5]; config.paddle_angle_gain=p[6];
    config.paddle_friction_cap_mu=p[7]; config.restitution_exp_g1=p[8]; config.restitution_exp_g2=p[9];
    config.restitution_racket=p[10]; physics.restitution_v=p[11]; physics.table_tangential_gain=p[12];
    physics.table_friction_cap_mu=p[14]; config.return_dt_integrate=p[15]; config.max_predict_time_s=p[16];
    table.net_x=p[17]; table.net_height=p[18]; table.width=p[19]; table.length=p[20];
    config.return_restitution_min=p[21]; config.return_restitution_max=p[22]; config.return_seed_iterations=int(p[23]);
    config.return_newton_iterations=int(p[24]); config.return_fd_relative_step=p[25];
    config.return_position_tolerance_m=p[26]; config.return_time_tolerance_s=p[27]; config.return_max_step_mps=p[28];
    config.return_line_search_iterations=int(p[29]); config.return_net_clearance_margin=p[30];
    config.return_numeric_epsilon=p[31]; config.delta_t_flight=p[32]; table.net_overhang=p[33];
    config.return_alternative_flight_times.assign(p.begin()+34,p.end());
    StrikeTarget strike;
    for(int i=0;i<3;++i) std::cin >> strike.ball_position[i];
    for(int i=0;i<3;++i) std::cin >> strike.ball_velocity[i];
    for(int i=0;i<3;++i) std::cin >> strike.ball_spin_rad_s[i];
    strike.valid=true; strike.strike_source_time_s=123.;
    RacketTargetPlanner planner(physics,config,table);
    if(mode=="forward") {
      Vec3 racket; for(int i=0;i<3;++i) std::cin >> racket[i];
      const auto result=planner.evaluate_return(strike,racket);
      std::cout << result.valid << ' ' << result.net_crossed;
      for(const Vec3& v : {result.racket_normal,result.outgoing_velocity,result.outgoing_spin_rad_s,
                           result.bounce_position,result.bounce_velocity,result.net_position}) vector_out(v);
      std::cout << ' ' << result.flight_time_s << '\n';
    } else if(mode=="plan") {
      Vec3 target=Vec3::Zero(); double time;
      std::cin >> target.x() >> target.y() >> time;
      const auto result=planner.plan(strike,target,time);
      std::cout << result.valid << ' ' << result.clears_net << ' ' << result.bypasses_net_posts;
      for(const Vec3& v : {result.velocity,result.normal,result.outgoing_ball_velocity,
                           result.outgoing_ball_spin_rad_s,result.predicted_landing}) vector_out(v);
      std::cout << ' ' << result.return_flight_time_s << '\n';
    } else return 4;
  }
}
