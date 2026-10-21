#include "hope_planner_cpp/planner_node.hpp"
#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <map>
#include <filesystem>
#include <fstream>
#include <unistd.h>
using namespace std::chrono_literals;
namespace hope_planner_cpp {
namespace {
class PlannerRuntimePlane : public ::testing::Test {
 protected:
  void SetUp() override {
    setenv("ROS_DOMAIN_ID", "227", 1); setenv("ROS_LOCALHOST_ONLY", "1", 1);
    int argc = 0; rclcpp::init(argc, nullptr);
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    state_dir = std::filesystem::temp_directory_path() / ("hope-scale-test-" + std::to_string(getpid()));
    std::filesystem::remove_all(state_dir);
    std::filesystem::create_directory(state_dir);
    opts.append_parameter_override("hit_speed_scale_state_file", (state_dir / "scale.state").string());
    opts.append_parameter_override("racket_flat_schema", 4);
    opts.append_parameter_override("session_home_input_topic", "/test/runtime_home");
    opts.append_parameter_override("base_pose_flat_input_topic", "/test/runtime_base");
    opts.append_parameter_override("robot_pose_topic", "");
    opts.append_parameter_override("flight_packet_input_enabled", true);
    opts.append_parameter_override("post_net_one_shot_enabled", true);
    opts.append_parameter_override("spin_shadow_enabled", false);
    opts.append_parameter_override("max_target_turn_deg", 180.0);
    planner = std::make_shared<PlannerNode>(opts);
    io = std::make_shared<rclcpp::Node>("runtime_plane_test_io");
    base = io->create_publisher<std_msgs::msg::Float64MultiArray>("/test/runtime_base", 10);
    home = io->create_publisher<std_msgs::msg::Float64MultiArray>("/test/runtime_home", 10);
    ball = io->create_publisher<hope_msgs::msg::BallFlightPacket>("/ball/flight_packet", rclcpp::QoS(10).best_effort());
    commands = io->create_subscription<std_msgs::msg::Float64MultiArray>("/racket/command_flat", 10,
        [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) { packets.push_back(msg->data); });
    typed_commands = io->create_subscription<hope_msgs::msg::RacketCommand>("/racket/command", 10,
        [this](hope_msgs::msg::RacketCommand::SharedPtr msg) { typed_packets.push_back(*msg); });
    diagnostics = io->create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/planner/diagnostics", 10,
        [this](diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
          for (const auto& status : msg->status) if (status.name == "hope_planner_cpp/realtime")
            for (const auto& field : status.values) telemetry[field.key] = field.value;
        });
    scales = io->create_client<rcl_interfaces::srv::SetParametersAtomically>("/hope/planner/set_hit_speed_scale");
    modes = io->create_client<rcl_interfaces::srv::SetParametersAtomically>("/hope/planner/set_plane_mode");
    executor->add_node(io); executor->add_node(planner);
    for (int i=0; i<200 && (!base->get_subscription_count() || !home->get_subscription_count() || !ball->get_subscription_count()); ++i) spin(10ms);
    ASSERT_GT(base->get_subscription_count(), 0u); ASSERT_GT(home->get_subscription_count(), 0u); ASSERT_GT(ball->get_subscription_count(), 0u);
    ASSERT_TRUE(modes->wait_for_service(1s));
  }
  void TearDown() override {
    executor->remove_node(planner); executor->remove_node(io); planner.reset(); io.reset(); rclcpp::shutdown();
    std::filesystem::remove_all(state_dir);
  }
  void spin(std::chrono::milliseconds duration) {
    const auto end=std::chrono::steady_clock::now()+duration;
    do { executor->spin_some(); std::this_thread::sleep_for(1ms); } while(std::chrono::steady_clock::now()<end);
  }
  static std::int64_t now() { return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count(); }
  bool mode(const std::string& value) {
    auto request=std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();
    request->parameters={rclcpp::Parameter("mode",value).to_parameter_msg()};
    auto future=modes->async_send_request(request);
    if(executor->spin_until_future_complete(future,2s)!=rclcpp::FutureReturnCode::SUCCESS) return false;
    return future.get()->result.successful;
  }
  bool scale_request(const std::vector<rcl_interfaces::msg::Parameter>& parameters) {
    auto request=std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();
    request->parameters=parameters;
    auto future=scales->async_send_request(request);
    if(executor->spin_until_future_complete(future,2s)!=rclcpp::FutureReturnCode::SUCCESS) return false;
    return future.get()->result.successful;
  }
  rclcpp::NodeOptions opts;
  std::filesystem::path state_dir;
  bool scale(double value) { return scale_request({rclcpp::Parameter("scale", value).to_parameter_msg()}); }
  void body(double x, bool stable, double offset=.58, std::uint64_t epoch=100, bool capable=true, bool stale=false) {
    auto stamp=now()-(stale ? 1000000000LL : 0);
    std_msgs::msg::Float64MultiArray pose, receipt;
    pose.data={2,1,double(++seq),double(stamp/1000000000LL),double(stamp%1000000000LL),x,-.7625,.95,1,0,0,0,1,47,7,8};
    receipt.data=a3_deploy::SessionHomeReceipt{epoch,-.5,-.7625,offset,7,8,capable}.pack(seq,stable);
    base->publish(pose); home->publish(receipt); spin(15ms);
  }
  hope_msgs::msg::BallFlightPacket flight(int id) {
    hope_msgs::msg::BallFlightPacket p;
    p.schema_version=1; p.session_id="runtime_test";p.producer_instance_id="test_source";
    p.trajectory_epoch=id;p.flight_sequence=id;p.payload_hash_algorithm=kBallFlightPacketHashAlgorithm;
    p.frame_id="world";p.segment_boundary_reason="initial_incoming";p.net_x=1.37;p.post_net_delay_s=.05;
    p.transmit_count=3;
    const auto start=now()-59*2777778LL;
    p.segment_start_exposure_unix_ns=start;p.net_cross_exposure_unix_ns=start+83333333;
    p.commit_exposure_unix_ns=start+133333333;p.freeze_wall_unix_ns=now();p.publish_wall_unix_ns=now();
    for(int i=0;i<60;++i) {
      hope_msgs::msg::BallFlightSample sample;sample.exposure_unix_stamp_ns=start+i*2777778LL;
      // A ballistic arc stays above the table through every test plane;
      // mode selection must not depend on field-specific bounce coefficients.
      const double t=i*2777778e-9;
      sample.position.x=1.55-3.2*t;sample.position.y=-.55+.0002*i;
      sample.position.z=1.2+.4*t-4.905*t*t;
      sample.orientation.w=1.;sample.orientation_valid=true;p.samples.push_back(sample);
    }
    p.payload_hash=flight_packet_message_payload_hash(p);return p;
  }
  std::vector<double> shoot(int id, double x, bool stable, double offset=.58, std::uint64_t epoch=100, bool capable=true, bool stale=false) {
    body(x,stable,offset,epoch,capable,stale); auto p=flight(id); last_ball=p; const auto count=packets.size();ball->publish(p);
    for(int i=0;i<150 && packets.size()==count;++i) spin(2ms);
    EXPECT_EQ(packets.size(),count+1);
    return packets.size()>count ? packets.back() : std::vector<double>{};
  }
  void expect_plane(const std::vector<double>& packet,int schema,double x,int selected=0) {
    ASSERT_EQ(packet.size(), schema==4 ? 20u : 22u);ASSERT_EQ(packet[1],1.);
    EXPECT_EQ(packet[0],schema);EXPECT_NEAR(packet[3],x,1e-8);
    if(schema==5) {EXPECT_EQ(packet[21],selected);EXPECT_NEAR(packet[20]+.58,packet[3],1e-8);}
  }
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::shared_ptr<PlannerNode> planner; rclcpp::Node::SharedPtr io;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr base,home;
  rclcpp::Publisher<hope_msgs::msg::BallFlightPacket>::SharedPtr ball;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr commands;
  rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr modes;
  rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr scales;
  rclcpp::Subscription<hope_msgs::msg::RacketCommand>::SharedPtr typed_commands;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics;
  std::vector<hope_msgs::msg::RacketCommand> typed_packets;
  std::map<std::string, std::string> telemetry;
  hope_msgs::msg::BallFlightPacket last_ball;
  std::vector<std::vector<double>> packets;std::uint64_t seq=0;
};
TEST_F(PlannerRuntimePlane, ScaleSurvivesRestartAndWriteFailureKeepsActiveValue) {
  ASSERT_TRUE(scale(.73));
  executor->remove_node(planner); planner.reset();
  planner = std::make_shared<PlannerNode>(opts);
  executor->add_node(planner);
  spin(1200ms);
  EXPECT_DOUBLE_EQ(std::stod(telemetry.at("hit_speed_scale")), .73);
  EXPECT_EQ(telemetry.at("hit_speed_scale_persistent"), "true");
  std::filesystem::remove_all(state_dir);
  EXPECT_FALSE(scale(.8));
  spin(1200ms);
  EXPECT_DOUBLE_EQ(std::stod(telemetry.at("hit_speed_scale")), .73);
}

TEST_F(PlannerRuntimePlane, CorruptSavedScaleFailsStartup) {
  executor->remove_node(planner); planner.reset();
  { std::ofstream file(state_dir / "scale.state"); file << "1 -3\n"; }
  EXPECT_THROW(std::make_shared<PlannerNode>(opts), std::runtime_error);
  std::filesystem::remove(state_dir / "scale.state");
  planner = std::make_shared<PlannerNode>(opts);
  executor->add_node(planner);
}

TEST_F(PlannerRuntimePlane, SwitchAfterStartupChangesOnlyNextBallWithoutRestart) {
  // This is the same live node throughout: no restart, reset, Stand or Ready RPC.
  expect_plane(shoot(1,-.4,false),4,.08);
  ASSERT_TRUE(mode("per_ball"));
  const auto first=packets.front();body(-.35,true);
  for(int i=0;i<3;++i) {last_ball.transmit_index=i;ball->publish(last_ball);spin(10ms);}
  spin(40ms);
  ASSERT_EQ(packets.size(),1u);EXPECT_EQ(packets.front(),first);
  expect_plane(shoot(2,-.4,false),5,.18,1);
  ASSERT_TRUE(mode("stable_per_ball"));
  expect_plane(shoot(3,-.3,false),5,.18,2);
  expect_plane(shoot(4,-.35,true),5,.23,2);
  EXPECT_FALSE(mode("continuous"));
  ASSERT_TRUE(mode("fixed"));
  expect_plane(shoot(5,-.35,true),4,.08);
  ASSERT_EQ(packets.size(),5u);
  for(std::size_t i=0;i<packets.size();++i) {
    EXPECT_EQ(packets[i][15],i+1);EXPECT_EQ(packets[i][16],1.);
  }
}
TEST_F(PlannerRuntimePlane, HitSpeedScaleUpdatesLiveTargetsAndBothWireOutputs) {
  EXPECT_DOUBLE_EQ(planner->get_parameter("hit_speed_scale").as_double(), 1.0);
  const auto original=shoot(1,-.4,false);
  ASSERT_EQ(original.size(),20u);ASSERT_EQ(original[1],1.);
  ASSERT_GT(std::abs(original[6])+std::abs(original[7])+std::abs(original[8]),0.01);
  ASSERT_TRUE(scale(.5));
  // A scale update itself publishes nothing, including retransmitted committed balls.
  const auto committed=packets.front();
  for(int i=0;i<3;++i) {last_ball.transmit_index=i;ball->publish(last_ball);spin(10ms);}
  ASSERT_EQ(packets.size(),1u);EXPECT_EQ(packets.front(),committed);
  int id=2;
  for (double factor : {.5, 1.4, 0., 1.}) {
    ASSERT_TRUE(scale(factor));
    const auto packet=shoot(id++,-.4,false);
    ASSERT_EQ(packet.size(),20u);ASSERT_EQ(packet[1],1.);
    for(int axis=0;axis<3;++axis) {
      EXPECT_NEAR(packet[3+axis],original[3+axis],1e-6);
      EXPECT_NEAR(packet[6+axis],original[6+axis]*factor,1e-6);
    }
    spin(20ms);ASSERT_FALSE(typed_packets.empty());ASSERT_TRUE(typed_packets.back().valid);
    EXPECT_DOUBLE_EQ(typed_packets.back().velocity.x,packet[6]);
    EXPECT_DOUBLE_EQ(typed_packets.back().velocity.y,packet[7]);
    EXPECT_DOUBLE_EQ(typed_packets.back().velocity.z,packet[8]);
  }
  ASSERT_TRUE(scale(.8));
  EXPECT_FALSE(scale(-1.));EXPECT_FALSE(scale(NAN));EXPECT_FALSE(scale(INFINITY));
  EXPECT_FALSE(scale_request({}));
  const auto valid=rclcpp::Parameter("scale",.2).to_parameter_msg();
  EXPECT_FALSE(scale_request({valid,valid}));
  EXPECT_FALSE(scale_request({rclcpp::Parameter("scale",2).to_parameter_msg()}));
  EXPECT_FALSE(scale_request({rclcpp::Parameter("unexpected",.2).to_parameter_msg()}));
  spin(1100ms);
  EXPECT_DOUBLE_EQ(std::stod(telemetry.at("hit_speed_scale")),.8);
  EXPECT_DOUBLE_EQ(std::stod(telemetry.at("hit_speed_scale_last_command")),1.);
  EXPECT_EQ(telemetry.at("hit_speed_scale_configurable"),"true");
  // Small values must round-trip diagnostics without being rounded down to zero.
  ASSERT_TRUE(scale(1e-20));spin(1100ms);
  EXPECT_DOUBLE_EQ(std::stod(telemetry.at("hit_speed_scale")),1e-20);
}
TEST_F(PlannerRuntimePlane, StaleBodyLegacyRunnerAndNewPolicyOffset) {
  ASSERT_TRUE(mode("per_ball")); // can select after START before HOME is ready
  auto stale=shoot(1,-.4,true,.58,100,true,true);ASSERT_GE(stale.size(),2u);EXPECT_EQ(stale[1],0.);
  auto legacy=shoot(2,-.4,true,.58,101,false);ASSERT_GE(legacy.size(),2u);EXPECT_EQ(legacy[1],0.);
  EXPECT_FALSE(mode("per_ball")); ASSERT_TRUE(mode("fixed"));
  expect_plane(shoot(3,-.4,true,.58,101,false),4,.08);
  body(-.4,true,.62,102,true);ASSERT_TRUE(mode("per_ball"));
  auto other=shoot(4,-.4,true,.62,102,true);ASSERT_EQ(other.size(),22u);ASSERT_EQ(other[1],1.);
  EXPECT_NEAR(other[3],.22,1e-8);EXPECT_EQ(other[19],102.);
}
} // namespace
} // namespace hope_planner_cpp
