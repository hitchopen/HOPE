#pragma once
#include "operations_observer_core.hpp"
#include "operations_observer_topics.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <functional>
#include <mutex>
#include <set>

namespace hope_operations {
inline std::int64_t monotonic_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
class NativeObserver {
 public:
  using Reply=std::function<void(const J&)>;
  explicit NativeObserver(rclcpp::Node &node):node_(node) {
    auto positive=[&](const char* key,double fallback) {
      double value=node.declare_parameter(key,fallback);
      if(!std::isfinite(value)||value<=0) throw std::invalid_argument(std::string(key)+" must be positive and finite");
      return value;
    };
    model_.base_stale=positive("base_stale_after_s",.5);
    model_.ball_stale=positive("ball_stale_after_s",.1);
    model_.runner_stale=positive("runner_stale_after_s",1.);
    auto period=positive("publish_period_s",.2);
    runtime_age_=positive("runtime_snapshot_max_age_s",2.);
    marker_period_=static_cast<std::int64_t>(std::nearbyint(1e9/positive("ball_marker_rate_hz",30.)));
    for(const auto &topic:observer_topics) {
      const std::string type=topic.type;
      if(type=="String") publisher<std_msgs::msg::String>(topic);
      else if(type=="Bool") publisher<std_msgs::msg::Bool>(topic);
      else if(type=="UInt64") publisher<std_msgs::msg::UInt64>(topic);
      else publisher<std_msgs::msg::Float64>(topic);
    }
    // One authoritative reader per source; match the consolidated Python QoS.
    runner_group_=node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    base_group_=node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sensor_group_=node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions opts; opts.callback_group=runner_group_;
    runner_=node.create_subscription<Array>("/hope/runner/state_hdu_flat",10,[this](Array::ConstSharedPtr m){update("runner",m->data);},opts);
    opts.callback_group=base_group_;
    base_=node.create_subscription<Array>("/a3/base_pose_flat",10,[this](Array::ConstSharedPtr m){update("base",m->data);},opts);
    opts.callback_group=sensor_group_;
    command_=node.create_subscription<Array>("/racket/command_flat",rclcpp::SensorDataQoS().keep_last(4),[this](Array::ConstSharedPtr m){observe("ObserverModule._command_callback",[&]{update("command",m->data);});},opts);
    marker_=node.create_publisher<visualization_msgs::msg::Marker>("/hope/ball/marker",10);
    ball_=node.create_subscription<geometry_msgs::msg::PoseArray>("/poses",rclcpp::SensorDataQoS().keep_last(4),
      [this](geometry_msgs::msg::PoseArray::ConstSharedPtr m){observe("ObserverModule._ball_callback",[&]{ball(*m);});},opts);
    for(const std::string name:{"UCB_P1","UCB_P2"})
      players_subs_.push_back(node.create_subscription<geometry_msgs::msg::PoseStamped>("/hope/mocap/players/"+name,rclcpp::SensorDataQoS(),
        [this,name](geometry_msgs::msg::PoseStamped::ConstSharedPtr m){
          double received=monotonic_ns()*1e-9;
          auto &p=m->pose.position;auto &q=m->pose.orientation;
          std::lock_guard<std::mutex> lock(mutex_);
          players_.update(name,{p.x,p.y,p.z},{q.x,q.y,q.z,q.w},
            static_cast<std::int64_t>(m->header.stamp.sec)*1000000000LL+m->header.stamp.nanosec,m->header.frame_id,received);
        },opts));
    timer_=node.create_wall_timer(std::chrono::duration<double>(period),[this]{publish();});
  }
  J snapshot(const std::string &name) {
    std::lock_guard<std::mutex> lock(mutex_); return wire(name).snapshot();
  }
  J player_status(double now) {std::lock_guard<std::mutex> lock(mutex_);return players_.status(now);}
  void wait(const std::string &name,std::uint64_t generation,double seconds,Reply reply) {
    if(!std::isfinite(seconds)||seconds<0||seconds>3600) throw std::invalid_argument("invalid wire wait");
    { std::lock_guard<std::mutex> lock(mutex_);
      if(wire(name).generation==generation && seconds>0) {
        if(waiters_.size()>=64) throw std::runtime_error("wire wait backlog full");
        waiters_.push_back({name,generation,monotonic_ns()+static_cast<std::int64_t>(seconds*1e9),std::move(reply)});return;
      }
    }
    reply(true);
  }
  void expire_waiters() {
    std::vector<Reply> ready;
    {std::lock_guard<std::mutex> lock(mutex_);
      auto now=monotonic_ns();
      for(auto it=waiters_.begin();it!=waiters_.end();) {
        if(now>=it->deadline){ready.push_back(std::move(it->reply));it=waiters_.erase(it);}else ++it;
      }
    }
    for(auto &reply:ready)reply(false);
  }
  void runtime(J value) {
    std::lock_guard<std::mutex> lock(mutex_);runtime_=std::move(value);
  }
  void fault_sink(std::function<void(const std::string&,const std::string&)> sink) { fault_=std::move(sink); }
  void runner_sink(std::function<void(const J&)> sink) { runner_sink_=std::move(sink); }
 private:
  using Array=std_msgs::msg::Float64MultiArray;
  struct Waiter {std::string name;std::uint64_t generation;std::int64_t deadline;Reply reply;};
  template<class F>void observe(const std::string &source,F callback) {
    try {callback();if(fault_)fault_(source,"");}
    catch(const std::exception &e) {
      {std::lock_guard<std::mutex> lock(mutex_);
        if(source=="ObserverModule._ball_callback")model_.balls.clear();
        else {model_.command.value=nullptr;model_.command.received=0;model_.command.error=std::string("TELEMETRY FAULT: ")+e.what();}
      }
      if(fault_)fault_(source,e.what());
    }
  }
  Wire &wire(const std::string &name) {
    if(name=="base")return model_.base;
    if(name=="runner")return model_.runner;
    if(name=="command")return model_.command;
    throw std::invalid_argument("unknown wire");
  }
  void update(const std::string &name,const std::vector<double>& data) {
    auto received=monotonic_ns();std::vector<Reply> ready;J runner_snapshot;
    {std::lock_guard<std::mutex> lock(mutex_);
      if(name=="base")model_.base.update(data,received,"BASE PACKET",decode_base);
      else if(name=="runner")model_.runner.update(data,received,"RUNNER STATE",decode_runner);
      else model_.command.update(data,received,"RACKET PACKET",decode_racket,false);
      if(name=="runner")runner_snapshot=model_.runner.snapshot();
      for(auto it=waiters_.begin();it!=waiters_.end();) {
        if(it->name==name){ready.push_back(std::move(it->reply));it=waiters_.erase(it);}else ++it;
      }
    }
    // Only active command/calibration waiters cross IPC. Ordinary sensor
    // packets never wake Python or incur JSON serialization/socket writes.
    if(!runner_snapshot.is_null()&&runner_sink_)runner_sink_(runner_snapshot);
    for(auto &reply:ready)reply(true);
  }
  void ball(const geometry_msgs::msg::PoseArray &m) {
    if(m.header.frame_id.empty()||m.poses.empty())return;
    auto p=m.poses.front().position;if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))return;
    auto received=monotonic_ns();
    {std::lock_guard<std::mutex> lock(mutex_);model_.ball(received);}
    if(last_marker_>0&&received-last_marker_<marker_period_)return;
    last_marker_=received;
    visualization_msgs::msg::Marker marker;marker.header=m.header;marker.ns="hope_ball";marker.id=0;
    marker.type=visualization_msgs::msg::Marker::SPHERE;marker.action=visualization_msgs::msg::Marker::ADD;
    marker.pose.position=p;marker.pose.orientation.w=1.;marker.scale.x=marker.scale.y=marker.scale.z=.04;
    marker.color.r=1.;marker.color.g=.36;marker.color.b=.04;marker.color.a=1.;marker.lifetime.nanosec=200000000;
    marker_->publish(marker);
  }
  template<class M>void publisher(const Topic &t) {
    rclcpp::PublisherOptions options;options.use_default_callbacks=false;
    auto p=node_.create_publisher<M>(t.topic,10,options);
    bool continuous=std::string(t.type)=="Float64"||std::string(t.type)=="UInt64"||continuous_.count(t.key);
    outputs_[t.key]=[p,continuous,last=J(),sent=std::int64_t(0)](const J &v,std::int64_t now)mutable {
      if(!continuous&&v==last&&now-sent<399999999LL)return;
      M m;m.data=v.get<decltype(m.data)>();p->publish(m);last=v;sent=now;
    };
  }
  void publish() {
    try {
      auto now=monotonic_ns();J runtime;Observer snapshot;
      {std::lock_guard<std::mutex> lock(mutex_);
        runtime=runtime_;snapshot=model_;
        if(runtime.value("fresh",false)) {
          double age=runtime.value("age_s",0.)+(now-runtime.value("sent_ns",std::int64_t(0)))*1e-9;
          if(age>runtime_age_) {runtime["fresh"]=false;runtime["error"]="snapshot refresh overdue";}
        }
      }
      if(runtime.value("telemetry_failed",false)) {
        invalidate();return; // Worker has already reported the contained fault.
      }
      J values=snapshot.view(now,node_.now().nanoseconds(),runtime);
      for(const auto &t:observer_topics)outputs_.at(t.key)(values.at(t.key),now);
      if(fault_)fault_("ObserverModule._publish_snapshot","");
    }catch(const std::exception &e) {
      invalidate();
      if(fault_)fault_("ObserverModule._publish_snapshot",e.what());
    }
  }
  void invalidate() {
    for(const auto* key:{"observer_alive","runtime_known","planner_alive","base_fresh","ball_live","command_valid","runner_alive"})
      try{outputs_.at(key)(false,monotonic_ns());}catch(...){}
  }
  rclcpp::Node &node_;Observer model_;std::mutex mutex_;
  Players players_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> players_subs_;
  J runtime_=J::object();double runtime_age_=2.;std::int64_t marker_period_=0,last_marker_=0;
  std::vector<Waiter> waiters_;
  std::map<std::string,std::function<void(const J&,std::int64_t)>> outputs_;
  const std::set<std::string> continuous_={"observer_alive","mdu_active","session_active","planner_alive","base_fresh","ball_live","command_valid","runner_alive","runner_mode","runner_session_matches","runner_command_fault"};
  std::function<void(const std::string&,const std::string&)> fault_;
  std::function<void(const J&)> runner_sink_;
  rclcpp::CallbackGroup::SharedPtr runner_group_,base_group_,sensor_group_;
  rclcpp::Subscription<Array>::SharedPtr runner_,base_,command_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr ball_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
