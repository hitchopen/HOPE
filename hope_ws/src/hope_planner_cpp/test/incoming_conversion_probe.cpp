// Offline production-path probe. Inputs are independent simulated observations.
// No ROS publisher, policy actuation, or hardware backend is constructed.
#include "hope_planner_cpp/v2_incoming_coordinator.hpp"
#include "hope_planner_cpp/racket_target_planner.hpp"
#include "hope_planner_cpp/schema2_packer.hpp"
#include "hope_planner_cpp/v2_managed_revocation.hpp"
#include "a3_pingpong/pp_planner_input.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>

using namespace hope_planner_cpp;
int main(int argc, char** argv) {
  std::ofstream paths;
  if (argc != 1 && argc != 3) return 2;
  if (argc == 3) {
    if (std::string(argv[1]) != "--waypoints") return 2;
    paths.open(argv[2]);
    if (!paths) return 2;
    paths << std::setprecision(17) << "id,source_ns,dt,x,y,z,vx,vy,vz\n";
  }
  std::cout << std::setprecision(17);
  std::cout << "id,mode,supported,eligible,command_valid,reason,cutoff_ns,contact_ns,"
    "cx,cy,cz,cvx,cvy,cvz,sx,sy,sz,svx,svy,svz,strike_time,"
    "rx,ry,rz,rvx,rvy,rvz,nx,ny,nz,ox,oy,oz,schema,deadline,"
    "decoded,dx,dy,dz,dvx,dvy,dvz,publication_allowed,revoked_before_engagement,"
    "bounce_upper,plane_lower,plane_std,plane_quality,position_variance,min_variance,plane_vx\n";
  int id, mode, count;
  double plane_x;
  while (std::cin >> id >> mode >> plane_x >> count) {
    TrajectorySnapshot packet;
    if (count < 1 || static_cast<std::size_t>(count) > packet.samples.size() ||
        mode < 0 || mode > 2 || !std::isfinite(plane_x)) return 2;
    IncomingSourceContract contract{"simulation", "table", "fixture", "simulation", "clock", "ball"};
    V2IncomingCoordinator owner{{}, {}, contract};
    owner.update_reference([] { return std::make_pair(7u, true); });
    for (int index=0; index<count; ++index) {
      BallSample sample;
      double angle;
      int orientation;
      if (!(std::cin >> sample.source_time_ns >> sample.position.x() >> sample.position.y()
          >> sample.position.z() >> angle >> orientation)) return 2;
      sample.frame_id="table"; sample.calibration_id="fixture";
      sample.clock_domain="simulation"; sample.clock_mapping_id="clock"; sample.ball_id="ball";
      sample.sequence=index+1;
      sample.orientation_valid=sample.orientation_authoritative=orientation!=0;
      sample.orientation=Eigen::Quaterniond(Eigen::AngleAxisd(angle,Vec3::UnitY()));
      owner.observe(sample,sample.source_time_ns);
      packet.samples[index]=sample;
    }
    packet.sample_count=count;
    packet.packet.present=true; packet.packet.session_id="simulation";
    packet.packet.producer_instance_id="independent-source";
    packet.packet.payload_hash="offline-exact-source-prefix";
    packet.packet.frame_id="table"; packet.packet.flight_sequence=1;
    packet.packet.trajectory_epoch=91; packet.one_shot.commit_due=true;
    PlaneSelection selected;
    selected.valid=true; selected.epoch=7; selected.flight=42;
    selected.mode=static_cast<PlaneMode>(mode);
    selected.reference_x=plane_x-.58; selected.plane_x=plane_x;
    const auto plane=incoming_plane(selected,"table","fixture",false);
    const auto now=packet.samples[count-1].source_time_ns;
    std::string reason;
    auto query=owner.packet(packet,plane,now,reason);
    IncomingEvaluation result;
    if (query) result=V2IncomingCoordinator::evaluate(*query,now);
    if (paths.is_open()) for (const auto& waypoint : result.forecast.nominal.waypoints) {
      paths << id << ',' << now << ',' << waypoint.dt_s;
      for (int axis=0; axis<6; ++axis) paths << ',' << waypoint.state[axis];
      paths << '\n';
    }
    const auto strike=v2_strike_target(result.candidate,now);
    RacketTargetPlanner planner({}, {}, {});
    const auto command=planner.plan(strike,Vec3(2.055,-.7625,.02),.7);
    if (query) reason=command.valid ? "VALID" : strike.valid ? command.reason : result.candidate.reason;
    a3_pingpong::PpRacketTargetInput receiver(4,true);
    receiver.SetExpectedHomeEpoch(7);
    bool published=false,revoked=false;
    std::vector<double> wire;
    if (query && command.valid) {
      published=owner.publish(*query,result,now,[&](auto& candidate) {
        const auto packed=Schema2Packer::pack(&command,strike.ball_position.y()>-.7?1:-1,
            strike.strike_source_time_s,.76,now,{1,42,1},count,
            (now-packet.samples[0].source_time_ns)*1e-9);
        if (!packed.valid) return false;
        wire.assign(packed.values.begin(),packed.values.end());
        wire[0]=mode==0?4:5; wire.push_back(7);
        if (mode!=0) { wire.push_back(selected.reference_x); wire.push_back(mode); }
        receiver.SetFromFlat(wire);
        candidate.command_sequence=1; candidate.command_flight_id=42; candidate.command_revision_id=1;
        return receiver.Latest().has_valid;
      });
    }
    const auto decoded=receiver.Latest();
    if (published) {
      auto status=result.candidate; status.status=IncomingStatus::Revoked;
      const auto invalid=managed_revocation(status,{2,42,1},now+1000000);
      if (invalid) receiver.SetFromFlat(*invalid);
      revoked=!receiver.GuardEngagement(decoded.cmd).allowed;
    }
    const auto& candidate=result.candidate;
    const Vec3 position=candidate.contact?candidate.contact->contact.pre.position:Vec3::Zero();
    const Vec3 velocity=candidate.contact?candidate.contact->contact.pre.velocity:Vec3::Zero();
    std::cout << id << ',' << mode << ',' << candidate.strike_contact_supported << ','
      << candidate.command_eligible << ',' << command.valid << ',' << reason << ',' << now << ','
      << (candidate.contact?candidate.contact->arrival_ns:0);
    auto vector=[](const Vec3& value) { std::cout << ',' << value.x() << ',' << value.y() << ',' << value.z(); };
    vector(position); vector(velocity); vector(strike.ball_position); vector(strike.ball_velocity);
    std::cout << ',' << strike.strike_source_time_s;
    vector(command.position); vector(command.velocity); vector(command.normal); vector(command.outgoing_ball_velocity);
    std::cout << ',' << (wire.empty()?0:wire[0]) << ',' << (wire.empty()?0:wire[10]) << ',' << decoded.has_valid;
    vector(decoded.cmd.pos_w); vector(decoded.cmd.vel_w);
    double bounce_upper=-1,plane_lower=-1,plane_std=-1,position_variance=-1,min_variance=-1,plane_vx=0;
    bool plane_quality=false;
    for (const auto& event: result.forecast.nominal.events) {
      if (event.kind=="table_bounce") bounce_upper=event.dt_s+3*event.time_std_s;
      if (event.kind=="receiving_plane") {
        plane_lower=event.dt_s-3*event.time_std_s; plane_std=event.time_std_s;
        plane_quality=event.quality==LinearizationQuality::Valid;
        position_variance=event.covariance.diagonal().head<3>().maxCoeff();
        min_variance=event.covariance.diagonal().minCoeff(); plane_vx=event.pre[3];
        break;
      }
    }
    std::cout << ',' << published << ',' << revoked << ',' << bounce_upper << ',' << plane_lower
      << ',' << plane_std << ',' << plane_quality << ',' << position_variance << ','
      << min_variance << ',' << plane_vx << '\n';
  }
  return std::cin.eof() && (!paths.is_open() || paths.good()) ? 0 : 2;
}
