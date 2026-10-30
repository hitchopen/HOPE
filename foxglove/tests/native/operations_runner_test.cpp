#include "operations_runner.hpp"
#include <iostream>
using hope_operations::J;
using hope_operations::RunnerCommands;
void require(bool ok,const char *why){if(!ok)throw std::runtime_error(why);}
J ack(std::uint64_t id,const char *action,const char *result="APPLIED") {
  return {{"error",""},{"value",{{"last_action_id",id},{"last_action",action},
    {"last_action_result",result},{"last_action_reason","MODE_CHANGED"},{"boot_id",42},
    {"state_sequence",7},{"run_mode","PD_STAND"},{"local_role","SERVER"}}}};
}
int main(){try {
  const std::map<std::string,int> actions{{"SET_SERVER",1},{"SET_RECEIVER",2},{"ENTER_PD_STAND",3},
    {"ENTER_MOTION",4},{"EMERGENCY_PASSIVE",5},{"PREPARE_SERVE",7},{"CONFIRM_BALL_LOADED",8},
    {"READY_TO_SERVE",9},{"OPEN_GRIPPER",10},{"CONFIRM_GRIP_SECURE",12},{"ENTER_TELEOP",13}};
  for(const auto &action:actions)for(const auto *result:{"APPLIED","ALREADY_SET","ACCEPTED_PENDING","REJECTED_WRONG_MODE"}) {
    RunnerCommands runner;runner.configure(2.,100);J reply;std::vector<double> wire;
    runner.start(action.first,10.,[&](const J &v){reply=v;},[&](const auto &v){wire=v;});
    require(wire==std::vector<double>({2.,101.,double(action.second),0.}),"wire parity");
    runner.observe(ack(101,action.first.c_str(),result),10.1);runner.poll(10.1);
    require(reply["success"]==(std::string(result)!="REJECTED_WRONG_MODE"),"result parity");
    require(reply["message"]==fmt::format("request=101 action={} result={} reason=MODE_CHANGED boot=42 seq=7 mode=PD_STAND role=SERVER",action.first,result),"text parity");
    require(!runner.busy(),"completion leak");
  }
  RunnerCommands runner;runner.configure(2.,(1ULL<<52));
  std::vector<std::vector<double>> sent;std::vector<J> replies;
  auto publish=[&](const auto &v){sent.push_back(v);};auto finish=[&](const J &v){replies.push_back(v);};
  runner.start("ENTER_MOTION",1.,finish,publish);
  require(sent.back()[1]==1.,"ID wrap");
  runner.observe(ack(1,"ENTER_MOTION"),1.01);
  runner.start("EMERGENCY_PASSIVE",1.02,finish,publish);
  runner.observe(ack(2,"EMERGENCY_PASSIVE"),1.03);runner.poll(1.03);
  require(replies.size()==2&&replies[0]["success"]&&replies[1]["success"],"ACK retention");
  runner.start("ENTER_MOTION",2.,finish,publish,0);
  require(!replies.back()["success"].get<bool>()&&sent.size()==2,"pre-stop request replayed");
  runner.start("ENTER_PD_STAND",2.,finish,publish,runner.epoch());
  runner.start("ENTER_MOTION",2.,finish,publish,runner.epoch());
  require(sent.size()==3&&!replies.back()["success"].get<bool>(),"ordinary lane not bounded");
  runner.poll(4.);
  require(replies.back()["message"]=="request=3 action=ENTER_PD_STAND not confirmed within 2.0s; latest Runner state did not acknowledge it","timeout parity");
  require(!runner.busy(),"timeout leak");
  RunnerCommands draining;draining.configure(3.,200);
  auto delivery=[&](const J &v){
    require(draining.busy(),"reply delivery not counted as busy");
    require(!draining.seal_if_idle(),"shutdown raced reply delivery");
    replies.push_back(v);
  };
  draining.start("EMERGENCY_PASSIVE",10.,delivery,publish);
  require(!draining.seal_if_idle(),"shutdown raced pending emergency ACK");
  draining.poll(12.);require(draining.busy(),"emergency ended before deadline");
  draining.poll(13.);require(!draining.busy(),"emergency timeout did not drain");
  require(draining.seal_if_idle(),"idle shutdown refused");
  auto sent_count=sent.size();
  draining.start("EMERGENCY_PASSIVE",14.,finish,publish);
  require(sent.size()==sent_count&&!replies.back()["success"].get<bool>(),"late command crossed shutdown seal");
  bool rejected=false;try{RunnerCommands::code("ENTER_SHADOW");}catch(const std::invalid_argument&){rejected=true;}
  require(rejected,"expanded action allowlist");
  std::cout<<"native Runner parity, ACK retention, emergency supersession, busy and timeout passed\n";
}catch(const std::exception &error){std::cerr<<error.what()<<'\n';return 1;} }
