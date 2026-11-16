#include "a3_deploy/session_home_contract.hpp"
#include "hope_planner_cpp/wire_identity_state.hpp"
#include <iostream>
#ifdef HOPE_VERIFY_PLANNER_PACKER
#include "hope_planner_cpp/schema2_packer.hpp"
#endif

int main(int argc,char** argv) {
  if(argc!=2)return 2;
  try {
    // Test state belongs exclusively to the caller's new evidence directory.
    const std::string dir=argv[1];
    const auto first=hope_planner_cpp::reserve_wire_identity_range(dir+"/test-identity.state",1000);
    const auto second=hope_planner_cpp::reserve_wire_identity_range(dir+"/test-identity.state",1);
    if(second.begin!=first.end)throw std::runtime_error("identity restart/rollback failed");
#ifdef HOPE_VERIFY_PLANNER_PACKER
    hope_planner_cpp::Schema2Packer simulation;
    for(int i=1;i<=6;++i) {
      const auto identity=simulation.next_identity(true,i*1000000000LL,true);
      if(identity.flight_id!=static_cast<std::uint64_t>(i) || identity.revision_id!=1)
        throw std::runtime_error("fresh simulation wire/physical flight identity mismatch");
    }
#endif
    bool diagnosed=false;
    try {hope_planner_cpp::reserve_wire_identity_range(dir+"/absent-parent/state",1000);}
    catch(const std::runtime_error& e) {
      const std::string message=e.what();std::cout<<message<<"\n";
      diagnosed=message.find("open path=")!=std::string::npos && message.find("errno=2")!=std::string::npos;
    }
    if(!diagnosed)throw std::runtime_error("open errno/path were lost");
    a3_deploy::SessionHomeReceipt runner{123,-.5,-.7625,.57,55,66,true};
    a3_deploy::SessionHomeMailbox planner;
    if(!planner.accept(runner.pack(1,true),10))throw std::runtime_error("real Runner HOME v2 rejected");
    const auto& home=planner.value;
    if(home.epoch!=runner.epoch || !home.same_geometry(runner) || !planner.posture_stable)
      throw std::runtime_error("HOME epoch/frame/calibration disagreement");
    if(!planner.fresh(20,100,55,66) || planner.fresh(20,100,56,66) ||
       planner.fresh(20,100,55,67) || planner.fresh(1000,100,55,66))
      throw std::runtime_error("HOME world/calibration/freshness validation changed");
    if(!runner.matches_command(4,home.epoch,0,home.x+home.plane_x))
      throw std::runtime_error("genuine schema4 fixed-plane command rejected");
    if(runner.matches_command(4,0,0,.07) || runner.matches_command(4,123,1,.07) ||
       runner.matches_command(4,123,0,.08))throw std::runtime_error("bad command accepted");
    auto changed=runner;changed.x+=.01;
    if(planner.accept(changed.pack(2,true),20))throw std::runtime_error("HOME recaptured within epoch");
    std::cout<<"PASS identity persistence, exact errno, Runner/Planner HOME pack/parse, schema4 plane, epoch/frame rejection\n";
    return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}
}
