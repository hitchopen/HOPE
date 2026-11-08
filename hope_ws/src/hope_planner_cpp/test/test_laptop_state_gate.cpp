#include "hope_planner_cpp/flight_packetizer_node.hpp"
#include "hope_planner_cpp/flight_packet.hpp"
#include <gtest/gtest.h>
#include <std_msgs/msg/string.hpp>
#include "local_status_client.hpp"
#include <chrono>
#include <thread>
#include <tuple>

namespace hope_planner_cpp {
namespace {
using namespace std::chrono_literals;

// Robot state topics are deliberately noise: only local HTTP intent and ball
// observations may affect the classifier or its two ball output channels.
class IndependentBallGate : public ::testing::TestWithParam<std::tuple<int,int,bool>> {
 protected:
  void SetUp() override {
    ::setenv("ROS_LOCALHOST_ONLY","1",1); ::setenv("ROS_DOMAIN_ID","228",1);
    rclcpp::init(0,nullptr);
    auto opts=rclcpp::NodeOptions();
    opts.append_parameter_override("play_state_status_port",0);
    opts.append_parameter_override("session_id","serve_follow_through");
    opts.append_parameter_override("input_pose_topic","/poses/planner_full");
    opts.append_parameter_override("gated_pose_topic","/poses");
    opts.append_parameter_override("adaptive_commit_enabled",std::get<2>(GetParam()));
    opts.append_parameter_override("adaptive_transport_budget_s",.03);
    opts.append_parameter_override("adaptive_max_sample_age_s",.10);
    opts.append_parameter_override("batch_post_bounce_min_span_s",.08);
    node=std::make_shared<FlightPacketizerNode>(opts);
    io=std::make_shared<rclcpp::Node>("serve_follow_through_io");
    input=io->create_publisher<geometry_msgs::msg::PoseArray>("/poses/planner_full",rclcpp::QoS(64).best_effort());
    runner=io->create_publisher<std_msgs::msg::Float64MultiArray>("/hope/runner/state_hdu_flat",10);
    serve=io->create_publisher<std_msgs::msg::String>("/hope/runner/serve_state",10);
    output=io->create_subscription<geometry_msgs::msg::PoseArray>("/poses",rclcpp::QoS(64).best_effort(),
        [this](geometry_msgs::msg::PoseArray::ConstSharedPtr m){forwarded.push_back(*m);});
    flights=io->create_subscription<hope_msgs::msg::BallFlightPacket>("/ball/flight_packet",rclcpp::QoS(10).best_effort(),
        [this](hope_msgs::msg::BallFlightPacket::ConstSharedPtr m){packets.push_back(*m);});
    ex=std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    ex->add_node(node);ex->add_node(io);
    for(int i=0;i<200 && !input->get_subscription_count();++i)spin(10);
    ASSERT_GT(input->get_subscription_count(),0U);
    ASSERT_EQ(runner->get_subscription_count(),0U);
    ASSERT_EQ(serve->get_subscription_count(),0U);
    feedback.data.resize(21,0);feedback.data[0]=2;feedback.data[1]=42;feedback.data[7]=1;
    feedback.data[13]=feedback.data[14]=-1;
    feedback.data[20]=123.;
    spin(10);ASSERT_NE(local_intent(node->local_status_port(),"SERVE").find("202 Accepted"),std::string::npos);spin(10);
  }
  void TearDown() override {
    ex->remove_node(node);ex->remove_node(io);ex.reset();
    node.reset();io.reset();rclcpp::shutdown();
  }
  void spin(int ms) {
    auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(ms);
    do {ex->spin_some();std::this_thread::sleep_for(1ms);} while(std::chrono::steady_clock::now()<end);
  }
  void mode(int value) { feedback.data[2]+=1;feedback.data[3]=value;runner->publish(feedback); }
  void stage(const char* value) { std_msgs::msg::String s;s.data=value;serve->publish(s); }
  std::string status() { return local_status_get(node->local_status_port()); }
  std::shared_ptr<FlightPacketizerNode> node;
  rclcpp::Node::SharedPtr io;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> ex;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr input;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr runner;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr serve;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr output;
  rclcpp::Subscription<hope_msgs::msg::BallFlightPacket>::SharedPtr flights;
  std_msgs::msg::Float64MultiArray feedback;
  std::vector<geometry_msgs::msg::PoseArray> forwarded;
  std::vector<hope_msgs::msg::BallFlightPacket> packets;
};

TEST_P(IndependentBallGate, EveryRobotModeAndLifecycleIsIgnoredDuringServeAndReturn) {
  const auto [hz,robot_mode,adaptive]=GetParam();
  auto lifecycle=io->create_publisher<std_msgs::msg::String>("/hope/lifecycle/state",10);
  EXPECT_EQ(lifecycle->get_subscription_count(),0U);
  EXPECT_EQ(io->count_publishers("/hope/runner/control_request_hdu_flat"),0U);
  const double dt=1./hz,return_at=std::ceil(.85*hz)/hz;
  Vec3 p(.04,-.7,.4),v(3.,0,-2.);bool returned=false;
  const auto start=io->now().nanoseconds();const auto wall=std::chrono::steady_clock::now();
  for(int i=0;i<=std::lround((return_at+.45)*hz);++i) {
    const double t=i*dt;
    const auto due=wall+std::chrono::nanoseconds(std::llround(t*1e9));
    while(std::chrono::steady_clock::now()<due){ex->spin_some();std::this_thread::sleep_for(1ms);}
    if(i%(hz/5)==0) {
      feedback.data[1]+=1;mode(robot_mode); // Even boot changes cannot reset this shot.
      const char* phases[]={"FAULT","ABORTED","PREPARING_STAND","COMPLETE"};stage(phases[(i/(hz/5))%4]);
      std_msgs::msg::String state;state.data=i%(hz/2)?"STOPPED":"RUNNING";lifecycle->publish(state);
    }
    if(!returned && t>=return_at-1e-8) {
      ASSERT_NE(status().find("\"state\":\"SERVE\""),std::string::npos);
      ASSERT_TRUE(packets.empty());ASSERT_TRUE(forwarded.empty());
      v=Vec3(-5.,0,1.6);returned=true;
    }
    geometry_msgs::msg::PoseArray m;m.header.stamp=rclcpp::Time(start+std::llround(t*1e9));
    m.header.frame_id="world";m.poses.resize(1);
    m.poses[0].position.x=p.x();m.poses[0].position.y=p.y();m.poses[0].position.z=p.z();
    input->publish(m);ex->spin_some();
    double remaining=dt;
    if(p.z()+v.z()*dt-4.905*dt*dt<.02 && v.z()<0 && p.x()>=0 && p.x()<=2.74) {
      const double contact=(v.z()+std::sqrt(v.z()*v.z()+19.62*(p.z()-.02)))/9.81;
      p+=v*contact+Vec3(0,0,-4.905*contact*contact);
      v.z()=(v.z()-9.81*contact)*-.9;p.z()=.02;remaining-=contact;
    }
    p+=v*remaining+Vec3(0,0,-4.905*remaining*remaining);v.z()-=9.81*remaining;
  }
  spin(20);ASSERT_EQ(packets.size(),3U);ASSERT_FALSE(forwarded.empty());
  ASSERT_NE(status().find("\"state\":\"OPPONENT_RALLY\""),std::string::npos)<<status();
  for(const auto& packet:packets) {
    EXPECT_EQ(packet.payload_hash,packets.front().payload_hash);
    EXPECT_EQ(packet.payload_hash,flight_packet_message_payload_hash(packet));
    EXPECT_GE(packet.segment_start_exposure_unix_ns,start+std::llround(return_at*1e9)-10'000);
  }
  EXPECT_EQ(io->count_publishers("/hope/runner/control_request_hdu_flat"),0U);
  EXPECT_EQ(io->count_publishers("/hope/ball/play_state"),0U);
}

INSTANTIATE_TEST_SUITE_P(RatesModesAndSchedulers,IndependentBallGate,
    ::testing::Combine(::testing::Values(150,300),::testing::Values(0,1,3,5,6),::testing::Values(false,true)));

class HeldServeRestart : public IndependentBallGate {};

TEST_P(HeldServeRestart, HeldServeNewBallisticLaunchOpensBothChannelsWithOrWithoutReady) {
  for(bool manual_ready:{false,true}) {
    forwarded.clear();packets.clear();
    ASSERT_NE(local_intent(node->local_status_port(),"SERVE").find("202 Accepted"),std::string::npos);spin(10);
    const auto hz=std::get<0>(GetParam());
    const double dt=1./hz,catch_at=.90,launch_at=1.90;
    Vec3 p(.04,-.7,.4),v(3.,0,-2.);
    const auto start=io->now().nanoseconds();const auto wall=std::chrono::steady_clock::now();
    bool launched=false;
    for(int i=0;i<=std::lround((launch_at+.82)*hz);++i) {
      const double t=i*dt;
      const auto due=wall+std::chrono::nanoseconds(std::llround(t*1e9));
      while(std::chrono::steady_clock::now()<due){ex->spin_some();std::this_thread::sleep_for(1ms);}
      if(i%(hz/5)==0) {mode(t<1.?5:3);stage(t<1.?"FOLLOW_THROUGH":"COMPLETE");}
      if(!launched && t>=launch_at-1e-8) {
        const auto s=status();
        ASSERT_NE(s.find("\"state\":\"INVALID\""),std::string::npos)<<s;
        ASSERT_NE(s.find("\"substate\":\"Ball stopped\""),std::string::npos)<<s;
        ASSERT_TRUE(forwarded.empty());ASSERT_TRUE(packets.empty());
        if(manual_ready) {
          ASSERT_NE(local_intent(node->local_status_port(),"READY").find("202 Accepted"),std::string::npos);spin(1);
        }
        v=Vec3(-3.,0,-2.);launched=true;
      }
      geometry_msgs::msg::PoseArray m;
      m.header.stamp=rclcpp::Time(start+std::llround(t*1e9));m.header.frame_id="world";
      m.poses.resize(1);m.poses[0].position.x=p.x();m.poses[0].position.y=p.y();m.poses[0].position.z=p.z();
      input->publish(m);spin(1);
      if(!manual_ready && t>=launch_at && t<launch_at+.05) {
        const auto s=status();EXPECT_NE(s.find("\"state\":\"INVALID\""),std::string::npos)<<s;
        EXPECT_TRUE(forwarded.empty());EXPECT_TRUE(packets.empty());
      }
      if(t>=launch_at+.20) {
        const auto s=status();
        ASSERT_NE(s.find("\"state\":\"OPPONENT_SERVE\""),std::string::npos)<<s;
        EXPECT_NE(s.find("\"forwarding\":true"),std::string::npos)<<s;
        EXPECT_NE(s.find("\"return_confirmed\":false"),std::string::npos)<<s;
        EXPECT_NE(s.find("\"striker_bounces\":1"),std::string::npos)<<s;
      }
      if(t>=catch_at-1e-8 && !launched)continue; // One second of fresh held poses.
      double remaining=dt;
      if(p.z()+v.z()*dt-4.905*dt*dt<.02 && v.z()<0 && p.x()>=0 && p.x()<=2.74) {
        const double contact=(v.z()+std::sqrt(v.z()*v.z()+19.62*(p.z()-.02)))/9.81;
        p+=v*contact+Vec3(0,0,-4.905*contact*contact);
        v.z()=(v.z()-9.81*contact)*-.9;p.z()=.02;remaining-=contact;
      }
      p+=v*remaining+Vec3(0,0,-4.905*remaining*remaining);v.z()-=9.81*remaining;
    }
    spin(20);

    ASSERT_FALSE(forwarded.empty());ASSERT_EQ(packets.size(),3U);
    EXPECT_GE(rclcpp::Time(forwarded.back().header.stamp).nanoseconds(),start+std::llround((launch_at+.80)*1e9));
    for(const auto& m:forwarded)EXPECT_GE(rclcpp::Time(m.header.stamp).nanoseconds(),start+std::llround(launch_at*1e9));
    for(const auto& packet:packets) {
      EXPECT_GE(packet.segment_start_exposure_unix_ns,start+std::llround(launch_at*1e9)-1000);
      EXPECT_EQ(packet.payload_hash,packets.front().payload_hash);
      EXPECT_EQ(packet.payload_hash,flight_packet_message_payload_hash(packet));
      for(const auto& s:packet.samples)EXPECT_GE(s.exposure_unix_stamp_ns,start+std::llround(launch_at*1e9));
    }
    const auto s=status();EXPECT_NE(s.find("\"receiver_bounces\":1"),std::string::npos)<<s;
  }
}

INSTANTIATE_TEST_SUITE_P(RatesAndSchedulers,HeldServeRestart,::testing::Values(
    std::make_tuple(150,1,false),std::make_tuple(300,5,false),
    std::make_tuple(150,1,true),std::make_tuple(300,5,true)));

class OccludedBallGate : public IndependentBallGate {};

TEST_P(OccludedBallGate, PrelaunchWaitAndShortIncomingLossRecoverFromFreshObservations) {
  geometry_msgs::msg::PoseArray held;held.header.frame_id="world";held.header.stamp=io->now();
  held.poses.resize(1);held.poses[0].position.x=.3;held.poses[0].position.y=-.7;held.poses[0].position.z=.6;
  input->publish(held);spin(10);
  geometry_msgs::msg::PoseArray missing;input->publish(missing);spin(150);
  auto s=status();ASSERT_NE(s.find("\"serve_armed\":true"),std::string::npos)<<s;
  EXPECT_TRUE(forwarded.empty());EXPECT_TRUE(packets.empty());
  ASSERT_NE(local_intent(node->local_status_port(),"READY").find("202 Accepted"),std::string::npos);spin(5);
  const int hz=std::get<0>(GetParam());const double dt=1./hz;
  const auto start=io->now().nanoseconds();const auto wall=std::chrono::steady_clock::now();
  std::int64_t resumed_at=0;bool paused=false,resumed=false;
  for(int i=0;i<=std::lround(.85*hz);++i) {
    const double t=i*dt;const auto due=wall+std::chrono::nanoseconds(std::llround(t*1e9));
    while(std::chrono::steady_clock::now()<due){ex->spin_some();std::this_thread::sleep_for(1ms);}
    if(t>=.08 && t<.12) {
      input->publish(missing);spin(1);
      if(!paused) {
        spin(2);s=status();ASSERT_NE(s.find("\"state\":\"OPPONENT_RALLY\""),std::string::npos)<<s;
        ASSERT_NE(s.find("\"forwarding\":false"),std::string::npos)<<s;
        forwarded.clear();paused=true;
      }
      EXPECT_TRUE(forwarded.empty());EXPECT_TRUE(packets.empty());continue;
    }
    if(paused && !resumed){resumed=true;resumed_at=start+std::llround(t*1e9);}
    geometry_msgs::msg::PoseArray m;m.header.frame_id="world";
    m.header.stamp=rclcpp::Time(start+std::llround(t*1e9));m.poses.resize(1);
    auto& p=m.poses[0].position;p.x=2.7-3*t;p.y=-.7;
    const double contact=(-2+std::sqrt(4+19.62*(.4-.02)))/9.81,after=t-contact;
    p.z=t<=contact?.4-2*t-4.905*t*t:.02+(2+9.81*contact)*.9*after-4.905*after*after;
    input->publish(m);spin(1);
  }
  spin(20);ASSERT_TRUE(resumed);ASSERT_FALSE(forwarded.empty());ASSERT_FALSE(packets.empty());
  for(const auto& m:forwarded)EXPECT_GE(rclcpp::Time(m.header.stamp).nanoseconds(),resumed_at);
  for(const auto& packet:packets) {
    EXPECT_GE(packet.segment_start_exposure_unix_ns,resumed_at-1000);
    for(const auto& sample:packet.samples)EXPECT_GE(sample.exposure_unix_stamp_ns,resumed_at);
  }
}

INSTANTIATE_TEST_SUITE_P(RatesAndSchedulers,OccludedBallGate,::testing::Values(
    std::make_tuple(150,1,false),std::make_tuple(300,5,false),
    std::make_tuple(150,1,true),std::make_tuple(300,5,true)));

TEST(LaptopStateGate, GatesBothBallChannelsAndSynchronizesOnlyLocalIntent) {
  ::setenv("ROS_LOCALHOST_ONLY", "1", 1); ::setenv("ROS_DOMAIN_ID", "228", 1);
  rclcpp::init(0,nullptr);
  auto opts = rclcpp::NodeOptions();
  opts.append_parameter_override("play_state_status_port",0);
  opts.append_parameter_override("input_pose_topic","/poses/planner_full");
  opts.append_parameter_override("gated_pose_topic","/poses");
  opts.append_parameter_override("session_id","state_gate_test");
  opts.append_parameter_override("retransmit_delays_ms", std::vector<std::int64_t>{0,250,500});
  auto node=std::make_shared<FlightPacketizerNode>(opts);
  auto io=std::make_shared<rclcpp::Node>("laptop_state_gate_io");
  auto poses=io->create_publisher<geometry_msgs::msg::PoseArray>("/poses/planner_full",rclcpp::QoS(64).best_effort());
  auto serve=io->create_publisher<std_msgs::msg::String>("/hope/runner/serve_state",rclcpp::QoS(1).reliable());
  int count=0, packets=0;
  std::string state;
  auto gated=io->create_subscription<geometry_msgs::msg::PoseArray>("/poses",rclcpp::QoS(64).best_effort(),
      [&](geometry_msgs::msg::PoseArray::ConstSharedPtr){++count;});
  auto flights=io->create_subscription<hope_msgs::msg::BallFlightPacket>("/ball/flight_packet",rclcpp::QoS(10).best_effort(),
      [&](hope_msgs::msg::BallFlightPacket::ConstSharedPtr){++packets;});
  rclcpp::executors::SingleThreadedExecutor ex; ex.add_node(node); ex.add_node(io);
  auto spin=[&](int ms) {
    auto until=std::chrono::steady_clock::now()+std::chrono::milliseconds(ms);
    do {ex.spin_some();std::this_thread::sleep_for(1ms);} while(std::chrono::steady_clock::now()<until);
    state=local_status_get(node->local_status_port());
  };
  for (int i=0;i<200 && poses->get_subscription_count()==0;++i) spin(10);
  ASSERT_GT(poses->get_subscription_count(),0U);
  auto feed=[&](double x, double vx, int frames=65, double z=.6, double vz=.5) {
    const auto start=io->now().nanoseconds();
    const auto wall=std::chrono::steady_clock::now();
    for(int i=0;i<frames;++i) {
      const auto due=wall+std::chrono::nanoseconds(std::llround(i/150.*1e9));
      while(std::chrono::steady_clock::now()<due){ex.spin_some();std::this_thread::sleep_for(1ms);}
      if (vx < 0 && i % 30 == 0) {
        std_msgs::msg::String terminal; terminal.data="ABORTED"; serve->publish(terminal);
      }
      double t=i/150.; geometry_msgs::msg::PoseArray m;
      auto ns=start+std::llround(t*1e9); m.header.stamp.sec=ns/1'000'000'000;
      m.header.stamp.nanosec=ns%1'000'000'000; m.header.frame_id="world";
      geometry_msgs::msg::Pose p; p.position.x=x+vx*t; p.position.y=-.7;
      const double contact=(vz+std::sqrt(vz*vz+19.62*(z-.02)))/9.81;
      const double after=t-contact;
      p.position.z=t<=contact ? z+vz*t-4.905*t*t :
          .02-(vz-9.81*contact)*.9*after-4.905*after*after;
      // Invalid quaternion is intentional.
      m.poses.push_back(p); poses->publish(m); spin(1);
    }
  };
  feed(.3,5,35);
  EXPECT_NE(state.find("\"state\":\"RALLY\""),std::string::npos) << state;
  EXPECT_EQ(count,0); EXPECT_EQ(packets,0);
  EXPECT_NE(local_intent(node->local_status_port(),"READY").find("202 Accepted"),std::string::npos);spin(10);
  std_msgs::msg::String m;
  feed(2.7,-3,100,.4,-2);
  spin(10); // Drain the already-published final pose before counting suppression.
  EXPECT_GT(count,0); EXPECT_GT(packets,0);
  EXPECT_NE(state.find("\"state\":\"OPPONENT_SERVE\""),std::string::npos) << state;
  EXPECT_EQ(io->count_publishers("/hope/ball/play_state"),0U); // State never enters A3's DDS graph.
  const int before=count, packet_before=packets;
  EXPECT_NE(local_intent(node->local_status_port(),"SERVE").find("202 Accepted"),std::string::npos);spin(10);
  EXPECT_NE(state.find("\"state\":\"SERVE\""),std::string::npos) << state;
  EXPECT_NE(state.find("\"forwarding\":false"),std::string::npos);
  spin(550);  // Queued retries from the old incoming shot must be cancelled.
  EXPECT_EQ(count,before); EXPECT_EQ(packets,packet_before);
  m.data="FAULT"; serve->publish(m); spin(30);
  EXPECT_NE(state.find("\"state\":\"SERVE\""),std::string::npos);
  feed(.3,5,35); // A visible ego serve remains suppressed despite controller FAULT.
  // One stale exposure is ignored; continued absence expires the real-input watchdog.
  geometry_msgs::msg::PoseArray stale;
  stale.header.stamp = rclcpp::Time(io->now().nanoseconds()-2'000'000'000LL);
  stale.poses.resize(1); stale.poses[0].position.z=.6;
  poses->publish(stale); spin(30);
  EXPECT_NE(state.find("\"state\":\"SERVE\""),std::string::npos) << state;
  spin(110);
  EXPECT_NE(state.find("ball_observation_missing"),std::string::npos) << state;
  EXPECT_EQ(packets,packet_before); EXPECT_EQ(count,before);
  ex.remove_node(node); ex.remove_node(io); rclcpp::shutdown();
}

TEST(LaptopStateGate, LocalReadyWaitsForQualifiedFlightAndLocalStopStaysClosed) {
  ::setenv("ROS_LOCALHOST_ONLY","1",1); ::setenv("ROS_DOMAIN_ID","228",1);
  rclcpp::init(0,nullptr);
  auto opts=rclcpp::NodeOptions();
  opts.append_parameter_override("play_state_status_port",0);
  opts.append_parameter_override("session_id","ready_gate_test");
  opts.append_parameter_override("input_pose_topic","/poses/planner_full");
  opts.append_parameter_override("gated_pose_topic","/poses");
  auto node=std::make_shared<FlightPacketizerNode>(opts);
  auto io=std::make_shared<rclcpp::Node>("receiver_ready_test");
  auto state=io->create_publisher<std_msgs::msg::Float64MultiArray>("/hope/runner/state_hdu_flat",10);
  auto poses=io->create_publisher<geometry_msgs::msg::PoseArray>("/poses/planner_full",rclcpp::QoS(10).best_effort());
  int count=0; std::string status;
  auto received=io->create_subscription<geometry_msgs::msg::PoseArray>("/poses",rclcpp::QoS(10).best_effort(),
      [&](geometry_msgs::msg::PoseArray::ConstSharedPtr){++count;});
  rclcpp::executors::SingleThreadedExecutor ex;ex.add_node(node);ex.add_node(io);
  auto spin=[&](int ms){auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(ms);
    do{ex.spin_some();std::this_thread::sleep_for(1ms);}while(std::chrono::steady_clock::now()<end);status=local_status_get(node->local_status_port());};
  for(int i=0;i<200 && poses->get_subscription_count()==0;++i)spin(10);
  EXPECT_EQ(state->get_subscription_count(),0U);
  EXPECT_NE(local_intent(node->local_status_port(),"READY").find("202 Accepted"),std::string::npos);spin(20);
  EXPECT_NE(status.find("\"state\":\"OPPONENT_SERVE\""),std::string::npos) << status;
  EXPECT_NE(status.find("\"receive_armed\":true"),std::string::npos) << status;
  geometry_msgs::msg::PoseArray m;m.header.stamp=io->now();m.poses.resize(1);
  m.poses[0].position.x=2.6;m.poses[0].position.y=-.7;m.poses[0].position.z=.5;
  poses->publish(m);spin(20);
  EXPECT_EQ(count,0); // One exposure cannot qualify an incoming flight.
  poses->publish(m);spin(20);EXPECT_EQ(count,0); // Duplicate never forwarded.
  m.header.stamp=io->now();m.poses[0].position.z=-.2;poses->publish(m);spin(20);
  EXPECT_EQ(count,0); // No invalid below-table sample, even when armed.
  EXPECT_NE(local_intent(node->local_status_port(),"STOP").find("202 Accepted"),std::string::npos);spin(20);
  std_msgs::msg::Float64MultiArray remote;remote.data={2,42,9,3};state->publish(remote);spin(20);
  EXPECT_NE(status.find("\"state\":\"INVALID\""),std::string::npos) << status;
  m.header.stamp=io->now();m.poses[0].position.z=.5;poses->publish(m);spin(20);
  EXPECT_EQ(count,0);
  for(int i=0;i<30;++i) {
    double t=i/150.; m.header.stamp=io->now();
    m.poses[0].position.x=2.6-5*t;m.poses[0].position.z=.5+.5*t-4.905*t*t;
    poses->publish(m);spin(7);
  }
  EXPECT_EQ(count,0); // Even a qualified incoming arc cannot leave local STOP.
  EXPECT_NE(status.find("\"state\":\"INVALID\""),std::string::npos) << status;
  ex.remove_node(node);ex.remove_node(io);rclcpp::shutdown();
}
TEST(LaptopStateGate, FreshOpponentServePassesAfterAnInvalidPointWithoutReady) {
  ::setenv("ROS_LOCALHOST_ONLY","1",1); ::setenv("ROS_DOMAIN_ID","228",1);
  rclcpp::init(0,nullptr);
  for(int hz:{150,300}) {
    auto opts=rclcpp::NodeOptions();
    opts.append_parameter_override("play_state_status_port",0);
    opts.append_parameter_override("session_id","ego_return_test");
    opts.append_parameter_override("input_pose_topic","/poses/planner_full");
    opts.append_parameter_override("gated_pose_topic","/poses");
    auto node=std::make_shared<FlightPacketizerNode>(opts);
    auto io=std::make_shared<rclcpp::Node>("ego_return_test_io");
    auto input=io->create_publisher<geometry_msgs::msg::PoseArray>("/poses/planner_full",rclcpp::QoS(64).best_effort());
    int poses=0,packets=0;
    auto output=io->create_subscription<geometry_msgs::msg::PoseArray>("/poses",rclcpp::QoS(64).best_effort(),
        [&](geometry_msgs::msg::PoseArray::ConstSharedPtr){++poses;});
    auto flights=io->create_subscription<hope_msgs::msg::BallFlightPacket>("/ball/flight_packet",rclcpp::QoS(10).best_effort(),
        [&](hope_msgs::msg::BallFlightPacket::ConstSharedPtr){++packets;});
    rclcpp::executors::SingleThreadedExecutor ex;ex.add_node(node);ex.add_node(io);
    auto spin=[&](int ms){auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(ms);
      do{ex.spin_some();std::this_thread::sleep_for(1ms);}while(std::chrono::steady_clock::now()<end);};
    for(int i=0;i<200 && input->get_subscription_count()==0;++i)spin(10);
    ASSERT_GT(input->get_subscription_count(),0U);
    const auto start=io->now().nanoseconds();const auto wall=std::chrono::steady_clock::now();
    // Opponent volleys the ego shot before its receiving bounce. The point
    // ends. A separate opponent serve appears after a gap with no button input.
    for(int i=0;i<=std::lround(1.74*hz);++i) {
      const double t=i/double(hz);
      const auto due=wall+std::chrono::nanoseconds(std::llround(t*1e9));
      while(std::chrono::steady_clock::now()<due){ex.spin_some();std::this_thread::sleep_for(1ms);}
      if(i==std::lround(.75*hz)) {
        const auto invalid=local_status_get(node->local_status_port());
        EXPECT_NE(invalid.find("\"state\":\"INVALID\""),std::string::npos) << invalid;
        EXPECT_EQ(poses,0);EXPECT_EQ(packets,0);
      }
      if(t>.55 && t<1.) continue;
      geometry_msgs::msg::PoseArray m;m.header.stamp=rclcpp::Time(start+std::llround(t*1e9));
      m.header.frame_id="world";m.poses.resize(1);auto& p=m.poses[0].position;
      p.y=-.7;
      if(t<1.) {
        const double after=t-.3;
        p.x=t<=.3 ? .3+6*t : 2.1-3.5*after;
        p.z=t<=.3 ? .6+.5*t-4.905*t*t : .30855+2*after-4.905*after*after;
      } else {
        const double flight=t-1.;
        const double contact=(-2+std::sqrt(4+19.62*(.4-.02)))/9.81;
        const double after=flight-contact;
        p.x=2.7-3*flight;
        p.z=flight<=contact ? .4-2*flight-4.905*flight*flight :
            .02+(2+9.81*contact)*.9*after-4.905*after*after;
      }
      input->publish(m);ex.spin_some();
      if(i==std::lround(.25*hz)) {
        EXPECT_EQ(poses,0);EXPECT_EQ(packets,0);
        EXPECT_NE(local_status_get(node->local_status_port()).find("\"state\":\"RALLY\""),std::string::npos);
      }
    }
    spin(20);
    auto state=local_status_get(node->local_status_port());
    EXPECT_NE(state.find("\"state\":\"OPPONENT_SERVE\""),std::string::npos) << state;
    EXPECT_NE(state.find("\"return_confirmed\":false"),std::string::npos) << state;
    EXPECT_GT(poses,0);EXPECT_GT(packets,0);
    ex.remove_node(node);ex.remove_node(io);
  }
  rclcpp::shutdown();
}
TEST(LaptopStateGate, ForecastAdvisoryDoesNotClosePositionOnlyIncomingPoseStream) {
  ::setenv("ROS_LOCALHOST_ONLY","1",1); ::setenv("ROS_DOMAIN_ID","228",1);
  rclcpp::init(0,nullptr);
  for(int hz:{150,300}) {
    auto opts=rclcpp::NodeOptions();
    opts.append_parameter_override("play_state_status_port",0);
    opts.append_parameter_override("input_pose_topic","/poses/planner_full");
    opts.append_parameter_override("gated_pose_topic","/poses");
    auto node=std::make_shared<FlightPacketizerNode>(opts);
    auto io=std::make_shared<rclcpp::Node>("forecast_advisory_test_io");
    auto input=io->create_publisher<geometry_msgs::msg::PoseArray>("/poses/planner_full",rclcpp::QoS(64).best_effort());
    auto plane=io->create_publisher<std_msgs::msg::Float64MultiArray>("/hope/planner/hit_plane",
        rclcpp::QoS(1).reliable().transient_local());
    std::int64_t forwarded_stamp=0;
    auto output=io->create_subscription<geometry_msgs::msg::PoseArray>("/poses",rclcpp::QoS(64).best_effort(),
        [&](geometry_msgs::msg::PoseArray::ConstSharedPtr m){forwarded_stamp=rclcpp::Time(m->header.stamp).nanoseconds();});
    rclcpp::executors::SingleThreadedExecutor ex;ex.add_node(node);ex.add_node(io);
    auto spin=[&](int ms){auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(ms);
      do{ex.spin_some();std::this_thread::sleep_for(1ms);}while(std::chrono::steady_clock::now()<end);};
    for(int i=0;i<200 && (!input->get_subscription_count() || !plane->get_subscription_count());++i)spin(10);
    ASSERT_GT(input->get_subscription_count(),0U);ASSERT_GT(plane->get_subscription_count(),0U);
    std_msgs::msg::Float64MultiArray h;h.data={1,io->now().seconds(),.5};plane->publish(h);spin(20);
    const auto start=io->now().nanoseconds();const auto wall=std::chrono::steady_clock::now();
    for(int i=0;i<=std::lround(.2*hz);++i) {
      double t=i/double(hz);const auto due=wall+std::chrono::nanoseconds(std::llround(t*1e9));
      while(std::chrono::steady_clock::now()<due){ex.spin_some();std::this_thread::sleep_for(1ms);}
      geometry_msgs::msg::PoseArray m;m.header.stamp=rclcpp::Time(start+std::llround(t*1e9));
      m.header.frame_id="world";m.poses.resize(1);auto& p=m.poses[0].position;
      p.x=2.5-8*t;p.y=-.7;p.z=1.2+.2*t-4.905*t*t; // No valid orientation/spin.
      input->publish(m);ex.spin_some();
    }
    spin(20);auto status=local_status_get(node->local_status_port());
    EXPECT_NE(status.find("\"forecast_advisory\":\"predicted_target_without_required_bounce\""),std::string::npos) << status;
    EXPECT_NE(status.find("\"state\":\"OPPONENT_RALLY\""),std::string::npos) << status;
    EXPECT_NE(status.find("\"forwarding\":true"),std::string::npos) << status;
    EXPECT_GE(forwarded_stamp,start+190'000'000LL); // Frames after the warning still pass.
    spin(120);status=local_status_get(node->local_status_port());
    EXPECT_NE(status.find("\"state\":\"OPPONENT_RALLY\""),std::string::npos);
    EXPECT_NE(status.find("\"forwarding\":false"),std::string::npos); // Loss pauses output, retains state.
    EXPECT_NE(status.find("\"forecast_advisory\":\"\""),std::string::npos);
    ex.remove_node(node);ex.remove_node(io);
  }
  rclcpp::shutdown();
}
TEST(LaptopStateGate, SlowIncomingRallyClosesBothChannelsAndCancelsRetries) {
  ::setenv("ROS_LOCALHOST_ONLY","1",1); ::setenv("ROS_DOMAIN_ID","228",1);
  rclcpp::init(0,nullptr);
  for(int hz:{150,300}) {
    auto opts=rclcpp::NodeOptions();
    opts.append_parameter_override("play_state_status_port",0);
    opts.append_parameter_override("input_pose_topic","/poses/planner_full");
    opts.append_parameter_override("gated_pose_topic","/poses");
    opts.append_parameter_override("retransmit_delays_ms",std::vector<std::int64_t>{0,900,1100});
    auto node=std::make_shared<FlightPacketizerNode>(opts);
    auto io=std::make_shared<rclcpp::Node>("slow_rally_test_io");
    auto input=io->create_publisher<geometry_msgs::msg::PoseArray>("/poses/planner_full",rclcpp::QoS(64).best_effort());
    int poses=0,packets=0;
    auto output=io->create_subscription<geometry_msgs::msg::PoseArray>("/poses",rclcpp::QoS(64).best_effort(),
        [&](geometry_msgs::msg::PoseArray::ConstSharedPtr){++poses;});
    auto flights=io->create_subscription<hope_msgs::msg::BallFlightPacket>("/ball/flight_packet",rclcpp::QoS(10).best_effort(),
        [&](hope_msgs::msg::BallFlightPacket::ConstSharedPtr){++packets;});
    rclcpp::executors::SingleThreadedExecutor ex;ex.add_node(node);ex.add_node(io);
    auto spin=[&](int ms){auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(ms);
      do{ex.spin_some();std::this_thread::sleep_for(1ms);}while(std::chrono::steady_clock::now()<end);};
    for(int i=0;i<200 && !input->get_subscription_count();++i)spin(10);
    ASSERT_GT(input->get_subscription_count(),0U);
    const auto start=io->now().nanoseconds();const auto wall=std::chrono::steady_clock::now();
    constexpr double stop=65./150.;
    int closed_poses=-1,closed_packets=-1;
    for(int i=0;i<=std::lround(.76*hz);++i) {
      const double t=i/double(hz),flight=std::min(t,stop),slow=std::max(0.,t-stop);
      const auto due=wall+std::chrono::nanoseconds(std::llround(t*1e9));
      while(std::chrono::steady_clock::now()<due){ex.spin_some();std::this_thread::sleep_for(1ms);}
      geometry_msgs::msg::PoseArray m;m.header.stamp=rclcpp::Time(start+std::llround(t*1e9));
      m.header.frame_id="world";m.poses.resize(1);auto& p=m.poses[0].position;
      const double contact=(.5+std::sqrt(.25+19.62*(.6-.02)))/9.81,after=flight-contact;
      p.x=2.5-5*flight-.08*slow;p.y=-.7;
      p.z=flight<=contact?.6+.5*flight-4.905*flight*flight:
          .02-(.5-9.81*contact)*.9*after-4.905*after*after;
      input->publish(m);ex.spin_some();
      if(i==std::lround(.63*hz)) {
        spin(2);const auto status=local_status_get(node->local_status_port());
        ASSERT_NE(status.find("\"state\":\"INVALID\""),std::string::npos)<<status;
        ASSERT_NE(status.find("rally_speed_below_threshold"),std::string::npos)<<status;
        EXPECT_NE(status.find("\"forwarding\":false"),std::string::npos);
        ASSERT_GT(poses,0);ASSERT_GT(packets,0);
        closed_poses=poses;closed_packets=packets;
      }
    }
    spin(1200); // Due retries from the old shot cannot survive invalidation.
    EXPECT_EQ(poses,closed_poses);EXPECT_EQ(packets,closed_packets);
    EXPECT_EQ(io->count_publishers("/hope/ball/play_state"),0U);
    ex.remove_node(node);ex.remove_node(io);
  }
  rclcpp::shutdown();
}
TEST(LaptopStateGate, BackwardClockRecoversWithoutRestartAndStatusHeartbeatStaysFresh) {
  ::setenv("ROS_LOCALHOST_ONLY","1",1); ::setenv("ROS_DOMAIN_ID","228",1);
  rclcpp::init(0,nullptr);
  for(int hz:{150,300}) {
    auto opts=rclcpp::NodeOptions();opts.append_parameter_override("play_state_status_port",0);
    opts.append_parameter_override("input_pose_topic","/poses/planner_full");
    opts.append_parameter_override("gated_pose_topic","/poses");
    opts.append_parameter_override("retransmit_delays_ms",std::vector<std::int64_t>{0,1200});
    auto node=std::make_shared<FlightPacketizerNode>(opts);
    auto io=std::make_shared<rclcpp::Node>("clock_recovery_test_io");
    auto input=io->create_publisher<geometry_msgs::msg::PoseArray>("/poses/planner_full",rclcpp::QoS(64).best_effort());
    std::vector<hope_msgs::msg::BallFlightPacket> packets;int poses=0;
    auto output=io->create_subscription<geometry_msgs::msg::PoseArray>("/poses",rclcpp::QoS(64).best_effort(),
        [&](geometry_msgs::msg::PoseArray::ConstSharedPtr){++poses;});
    auto flights=io->create_subscription<hope_msgs::msg::BallFlightPacket>("/ball/flight_packet",rclcpp::QoS(10).best_effort(),
        [&](hope_msgs::msg::BallFlightPacket::ConstSharedPtr m){packets.push_back(*m);});
    rclcpp::executors::SingleThreadedExecutor ex;ex.add_node(node);ex.add_node(io);
    auto spin=[&](int ms){auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(ms);
      do{ex.spin_some();std::this_thread::sleep_for(1ms);}while(std::chrono::steady_clock::now()<end);};
    for(int i=0;i<200 && !input->get_subscription_count();++i)spin(10);
    ASSERT_GT(input->get_subscription_count(),0U);
    spin(1100);auto status=local_status_get(node->local_status_port());
    EXPECT_NE(status.find("200 OK"),std::string::npos); // Cached JSON still gets its heartbeat.
    EXPECT_NE(status.find("startup"),std::string::npos);

    bool rebased=false;std::int64_t recovered_boundary=0;
    auto feed=[&](int frames,std::int64_t offset_ns,bool recover) {
      const auto start=io->now().nanoseconds()+offset_ns;const auto wall=std::chrono::steady_clock::now();
      for(int i=0;i<frames;++i) {
        const double t=i/double(hz);const auto due=wall+std::chrono::nanoseconds(std::llround(t*1e9));
        while(std::chrono::steady_clock::now()<due){ex.spin_some();std::this_thread::sleep_for(1ms);}
        geometry_msgs::msg::PoseArray m;m.header.stamp=rclcpp::Time(start+std::llround(t*1e9));
        m.header.frame_id="world";m.poses.resize(1);auto& p=m.poses[0].position;
        p.x=2.5-5*t;p.y=-.7;p.z=1.+.4*t-4.905*t*t;
        input->publish(m);spin(1);
        if(recover) {
          const auto text=local_status_get(node->local_status_port());
          if(text.find("source_clock_rebased")!=std::string::npos && !rebased) {
            rebased=true;recovered_boundary=rclcpp::Time(m.header.stamp).nanoseconds();
            EXPECT_NE(text.find("\"forwarding\":false"),std::string::npos);
          }
        }
      }
    };
    feed(std::lround(.35*hz),40'000'000LL,false);
    ASSERT_GT(packets.size(),0U);ASSERT_GT(poses,0);
    const auto old_id=packets.front().flight_sequence;const auto old_count=packets.size();
    const int old_poses=poses;
    // Shift from +40 to -80 ms while keeping each exposure inside the existing
    // wall-age limits. This simulates a source correction without stepping any
    // host/robot clock, and exercises the real ROS admission path.
    feed(std::lround(.4*hz),-80'000'000LL,true);
    ASSERT_TRUE(rebased);EXPECT_GT(poses,old_poses);
    status=local_status_get(node->local_status_port());
    EXPECT_NE(status.find("\"state\":\"OPPONENT_RALLY\""),std::string::npos)<<status;
    ASSERT_GT(packets.size(),old_count);
    for(std::size_t i=old_count;i<packets.size();++i) {
      EXPECT_GT(packets[i].flight_sequence,old_id);
      EXPECT_GE(packets[i].samples.front().exposure_unix_stamp_ns,recovered_boundary-1'000'000'000LL/hz);
    }
    const auto count_after_recovery=packets.size();spin(1300);
    EXPECT_EQ(packets.size(),count_after_recovery); // Old-epoch and expired new retries cancelled.
    // A clock that is still unsynchronised remains blocked by absolute age.
    feed(std::lround(.2*hz),-2'000'000'000LL,true);
    EXPECT_EQ(packets.size(),count_after_recovery);
    EXPECT_NE(local_status_get(node->local_status_port()).find("\"state\":\"OPPONENT_RALLY\""),std::string::npos);
    EXPECT_NE(local_status_get(node->local_status_port()).find("\"forwarding\":false"),std::string::npos);
    ex.remove_node(node);ex.remove_node(io);
  }
  rclcpp::shutdown();
}
}  // namespace
}  // namespace hope_planner_cpp
