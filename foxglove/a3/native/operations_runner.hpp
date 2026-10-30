#pragma once
// Fixed Runner execution/ACK contract. No lifecycle or diagnostic admission.
#include "operations_observer_core.hpp"
#include <functional>
#include <mutex>
namespace hope_operations {
class RunnerCommands {
 public:
  using Complete=std::function<void(const J&)>;
  using Publish=std::function<void(const std::vector<double>&)>;
  static int code(const std::string &action) {
    static const std::map<std::string,int> codes{{"SET_SERVER",1},{"SET_RECEIVER",2},
      {"ENTER_PD_STAND",3},{"ENTER_MOTION",4},{"EMERGENCY_PASSIVE",5},{"PREPARE_SERVE",7},
      {"CONFIRM_BALL_LOADED",8},{"READY_TO_SERVE",9},{"OPEN_GRIPPER",10},
      {"CONFIRM_GRIP_SECURE",12},{"ENTER_TELEOP",13}};
    const auto it=codes.find(action);
    if(it==codes.end())throw std::invalid_argument("unsupported Runner action");
    return it->second;
  }
  void configure(double timeout,std::uint64_t seed) {
    if(!std::isfinite(timeout)||timeout<=0||seed==0||seed>(1ULL<<52))
      throw std::invalid_argument("invalid native Runner configuration");
    std::lock_guard<std::mutex> lock(mutex_);
    if(configured_)throw std::runtime_error("Runner already configured");
    timeout_=timeout;sequence_=seed;configured_=true;
  }
  void start(const std::string &action,double now,Complete complete,const Publish &publish,
             std::uint64_t expected_epoch=0,double received=-1.) {
    const auto action_code=code(action);J refusal;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if(!configured_||closing_)refusal={{"success",false},{"message","native Runner unavailable"}};
      else if(action!="EMERGENCY_PASSIVE"&&expected_epoch!=epoch_)
        refusal={{"success",false},{"message","Runner action superseded by emergency; not published or replayed"}};
      else if(std::any_of(pending_.begin(),pending_.end(),[&](const Pending &p){
                return (p.action=="EMERGENCY_PASSIVE")== (action=="EMERGENCY_PASSIVE");}))
        refusal={{"success",false},{"message","another explicit operator action is already in progress"}};
      else {
        if(action=="EMERGENCY_PASSIVE")++epoch_;
        if(++sequence_>(1ULL<<52))sequence_=1;
        Pending pending{sequence_,action,now,{},std::move(complete)};
        // One native publisher and ID allocator serialize normal/urgent sends.
        try {publish({2.,static_cast<double>(sequence_),static_cast<double>(action_code),0.});}
        catch(const std::exception &error) {
          complete=std::move(pending.complete);
          refusal={{"success",false},{"message",std::string("Runner action was not published: ")+error.what()}};
        }
        if(refusal.is_null()){
          dispatch_last_=received>=0?std::max(0.,now-received):0.;dispatch_max_=std::max(dispatch_max_,dispatch_last_);
          pending_.push_back(std::move(pending));++dispatched_;return;
        }
      }
    }
    complete(refusal);
  }
  void observe(const J &snapshot,double now) {
    std::lock_guard<std::mutex> lock(mutex_);snapshot_=snapshot;
    const auto &state=snapshot.at("value");if(state.is_null())return;
    for(auto &p:pending_)if(p.result.is_null()&&state.at("last_action_id")==p.id) {
      const std::string result=state.at("last_action_result");
      p.result={{"success",result=="APPLIED"||result=="ALREADY_SET"||result=="ACCEPTED_PENDING"},
        {"message",fmt::format("request={} action={} result={} reason={} boot={} seq={} mode={} role={}",
          p.id,state.at("last_action").get<std::string>(),result,state.at("last_action_reason").get<std::string>(),
          state.at("boot_id").get<std::uint64_t>(),state.at("state_sequence").get<std::uint64_t>(),
          state.at("run_mode").get<std::string>(),state.at("local_role").get<std::string>())}};
      ack_last_=std::max(0.,now-p.started);ack_max_=std::max(ack_max_,ack_last_);
    }
  }
  // Keep each ACK at ingress, even if another ACK replaces the latest state.
  // Dispatch completions outside input callbacks and outside the mutex.
  void poll(double now) {
    std::vector<Pending> done;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for(auto it=pending_.begin();it!=pending_.end();) {
        if(it->result.is_null()&&now-it->started>=timeout_) {
          auto error=snapshot_.value("error",std::string("NO RUNNER STATE"));
          it->result={{"success",false},{"message",fmt::format(
            "request={} action={} not confirmed within {:.1f}s; {}",it->id,it->action,timeout_,
            error.empty()?"latest Runner state did not acknowledge it":error)}};++timeouts_;
        }
        if(!it->result.is_null()){done.push_back(std::move(*it));it=pending_.erase(it);}else ++it;
      }
      completing_+=done.size();
    }
    // Pending ACKs are not finished until their operator replies have been
    // sent. Shutdown must not race the callbacks after pending_ is erased.
    struct Delivery {
      RunnerCommands &owner;std::size_t count;
      ~Delivery(){std::lock_guard<std::mutex> lock(owner.mutex_);owner.completing_-=count;}
    } delivery{*this,done.size()};
    for(auto &p:done)p.complete(p.result);
  }
  bool busy(){std::lock_guard<std::mutex> lock(mutex_);return !pending_.empty()||completing_;}
  bool seal_if_idle(){
    std::lock_guard<std::mutex> lock(mutex_);
    if(!pending_.empty()||completing_)return false;
    closing_=true;return true;
  }
  std::uint64_t epoch(){std::lock_guard<std::mutex> lock(mutex_);return epoch_;}
  J metrics(){std::lock_guard<std::mutex> lock(mutex_);return {
    {"pending",pending_.size()},{"dispatched",dispatched_},{"ack_timeouts",timeouts_},
    {"admission_to_dispatch_last_s",dispatch_last_},{"admission_to_dispatch_max_s",dispatch_max_},
    {"ack_latency_last_s",ack_last_},{"ack_latency_max_s",ack_max_}};}
 private:
  struct Pending{std::uint64_t id;std::string action;double started;J result;Complete complete;};
  std::mutex mutex_;bool configured_=false,closing_=false;std::size_t completing_=0;
  double timeout_=2.,ack_last_=0.,ack_max_=0.,dispatch_last_=0.,dispatch_max_=0.;
  std::uint64_t sequence_=0,dispatched_=0,timeouts_=0,epoch_=0;
  J snapshot_=J::object();std::vector<Pending> pending_;
};
}
