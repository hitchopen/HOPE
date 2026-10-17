// Native, subscription-free contract probe against this checkout's actual receiver.
#include "hope_planner_cpp/strike_target_selector.hpp"
#include "hope_planner_cpp/racket_target_planner.hpp"
#include "hope_planner_cpp/schema2_packer.hpp"
#include <a3_pingpong/pp_planner_input.hpp>
#include <a3_pingpong/pp_planner_lifecycle.hpp>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>
using namespace hope_planner_cpp;
void require(bool value,const char* message) {if(!value)throw std::runtime_error(message);}
int main(int argc,char** argv) {
 try {
  std::vector<double> wire;
  if(argc==2) {std::ifstream in(argv[1]);double v;while(in>>v)wire.push_back(v);}
  else {
    const auto now=static_cast<std::int64_t>(a3_pingpong::PpNowWallSec()*1e9);
    StrikeTargetSelectorConfig c;c.registration="fixture";c.synthetic=true;
    c.frozen_plane=SelectorFrozenPlane{0,1,0,0,true};StrikeTargetSelector selector(c);
    for(int i=0;i<30;++i){BallSample s;s.source_time_ns=now-160000000+i*5000000LL;
      s.position=Vec3(1.8-.025*i,-.7,.4);selector.observe(s,"table",s.source_time_ns,s.source_time_ns);}
    auto selected=selector.poll(now,now);require(selected && selected->strike.valid,"public batch fallback failed");
    auto command=RacketTargetPlanner(c.physics,c.planner,{}).plan(selected->strike,Vec3(2.055,-.7625,0),.5);
    auto packed=Schema2Packer::pack(&command,1,command.strike_source_time_s,.76,now,{1,1,1},selected->sample_count,selected->sample_span_s);
    require(packed.valid,"public Stage3/schema2 packing failed");wire.assign(packed.values.begin(),packed.values.end());
  }
  require(wire.size()==19,"expected exactly 19 schema2 slots");
  a3_pingpong::PpRacketTargetInput receiver(2);receiver.SetFromFlat(wire);
  auto first=receiver.Latest();require(first.has_valid && first.control_time_to_strike_s>0,"current receiver rejected emitted packet");
  require(first.cmd.command_seq==static_cast<std::uint64_t>(wire[14]) && first.cmd.flight_id==static_cast<std::uint64_t>(wire[15]) && first.cmd.revision_id==1,"identity mismatch");
  auto next=wire;next[14]+=1;next[15]+=1;receiver.SetFromFlat(next);
  auto second=receiver.Latest();require(second.seq==first.seq+1,"monotonic next physical flight rejected");
  require(a3_pingpong::planner_same_flight_identity({second.cmd.producer_epoch,second.cmd.flight_id},{second.cmd.producer_epoch,second.cmd.flight_id}),"lifecycle identity mismatch");
  std::this_thread::sleep_for(std::chrono::milliseconds(65));
  receiver.SetFromFlat(wire);auto rolled=receiver.Latest();
  require(rolled.seq==second.seq && rolled.cmd.flight_id==second.cmd.flight_id,"restart rollback unexpectedly accepted");
  std::cout<<"Current receiver: emitted packet accepted; monotonic identity accepted; delayed restart rollback rejected.\n";
  return 0;
 } catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
