#include "hope_planner_cpp/planner_node.hpp"
#include "a3_pingpong/pp_planner_input.hpp"
#include <gtest/gtest.h>
#include <chrono>
#include <thread>

using namespace hope_planner_cpp;
using namespace std::chrono_literals;
namespace {
TEST(IncomingStartup, RejectsLiveModeUnqualifiedSourcesAndOutputRemapping) {
  int argc = 0; rclcpp::init(argc, nullptr);
  for (const std::string fault : {"live", "unqualified", "command_remap", "forecast_remap"}) {
    rclcpp::NodeOptions options;
    options.append_parameter_override("prediction_backend", "v2_incoming");
    options.append_parameter_override("flight_packet_input_enabled", true);
    options.append_parameter_override("post_net_one_shot_enabled", true);
    options.append_parameter_override("racket_flat_schema", 4);
    for (const auto* field : {"session_id", "frame_id", "calibration_id", "clock_domain",
                              "clock_mapping_id", "ball_id"})
      options.append_parameter_override(std::string("v2_incoming.")+field, "fixture");
    options.append_parameter_override("v2_incoming.base_calibration_id", 7);
    options.append_parameter_override("v2_incoming.base_world_id", 8);
    if (fault == "live") options.append_parameter_override("v2_incoming.output_mode", "live");
    if (fault == "unqualified") options.append_parameter_override("v2_incoming.calibration_id", "");
    if (fault == "command_remap" || fault == "forecast_remap")
      options.arguments({"--ros-args", "-r", std::string(V2IncomingRuntime::prefix)+
          (fault == "command_remap" ? "command_flat:=/racket/command_flat" : "prediction:=/live/prediction")});
    EXPECT_THROW(std::make_shared<PlannerNode>(options), std::exception) << fault;
  }
  rclcpp::shutdown();
}
class IncomingRos : public ::testing::TestWithParam<std::string> {
 protected:
  void SetUp() override {
    setenv("ROS_DOMAIN_ID", "228", 1); setenv("ROS_LOCALHOST_ONLY", "1", 1);
    int argc = 0; rclcpp::init(argc, nullptr);
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    rclcpp::NodeOptions options;
    options.append_parameter_override("prediction_backend", "v2_incoming");
    options.append_parameter_override("v2_incoming.continuous_output", continuous_output);
    options.append_parameter_override("racket_flat_schema", 4);
    options.append_parameter_override("flight_packet_input_enabled", true);
    options.append_parameter_override("post_net_one_shot_enabled", true);
    options.append_parameter_override("session_home_input_topic", "/test/incoming/home");
    options.append_parameter_override("base_pose_flat_input_topic", "/test/incoming/base");
    options.append_parameter_override("v2_incoming.observation_topic", "/test/incoming/observation");
    options.append_parameter_override("robot_pose_topic", "");
    options.append_parameter_override("plane_reference_mode", GetParam());
    options.append_parameter_override("max_target_turn_deg", 180.);
    options.append_parameter_override("v2_incoming.session_id", "ros-test");
    options.append_parameter_override("v2_incoming.frame_id", "table");
    options.append_parameter_override("v2_incoming.calibration_id", "fixture");
    options.append_parameter_override("v2_incoming.clock_domain", "system");
    options.append_parameter_override("v2_incoming.clock_mapping_id", "clock");
    options.append_parameter_override("v2_incoming.ball_id", "ball");
    options.append_parameter_override("v2_incoming.base_calibration_id", 7);
    options.append_parameter_override("v2_incoming.base_world_id", 8);
    planner = std::make_shared<PlannerNode>(options);
    io = std::make_shared<rclcpp::Node>("incoming_test_source");
    base = io->create_publisher<std_msgs::msg::Float64MultiArray>("/test/incoming/base", 10);
    home = io->create_publisher<std_msgs::msg::Float64MultiArray>("/test/incoming/home", 10);
    observations = io->create_publisher<hope_msgs::msg::BallObservationV2>("/test/incoming/observation", 8);
    packets = io->create_publisher<hope_msgs::msg::BallFlightPacket>("/ball/flight_packet", rclcpp::QoS(8).best_effort());
    command_sub = io->create_subscription<std_msgs::msg::Float64MultiArray>(
        std::string(V2IncomingRuntime::prefix)+"command_flat", 10,
        [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr message) {
          if (message->data.at(1)==1.) commands.push_back(message->data);
          else invalid_commands.push_back(message->data);
          receiver.SetFromFlat(message->data);
        });
    prediction_sub = io->create_subscription<hope_msgs::msg::FlightPredictionV2>(
        std::string(V2IncomingRuntime::prefix)+"prediction", 64,
        [this](hope_msgs::msg::FlightPredictionV2::ConstSharedPtr message) { predictions.push_back(*message); });
    candidate_sub = io->create_subscription<hope_msgs::msg::StrikeCandidateV1>(
        std::string(V2IncomingRuntime::prefix)+"candidate", 64,
        [this](hope_msgs::msg::StrikeCandidateV1::ConstSharedPtr message) { candidates.push_back(*message); });
    revocation_sub = io->create_subscription<hope_msgs::msg::StrikeCandidateV1>(
        std::string(V2IncomingRuntime::prefix)+"revocation", rclcpp::QoS(64).reliable().transient_local(),
        [this](hope_msgs::msg::StrikeCandidateV1::ConstSharedPtr message) { revocations.push_back(*message); });
    executor->add_node(planner); executor->add_node(io);
    receiver.SetExpectedHomeEpoch(100);
    for (int i=0; i<200 && (!observations->get_subscription_count() || !packets->get_subscription_count()); ++i) spin(5ms);
    ASSERT_GT(observations->get_subscription_count(), 0u);
    ASSERT_GT(packets->get_subscription_count(), 0u);
    body(-.4); spin(20ms);
  }
  void TearDown() override {
    executor->remove_node(planner); executor->remove_node(io);
    planner.reset(); io.reset(); executor.reset(); rclcpp::shutdown();
  }
  static std::int64_t now() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
  }
  void spin(std::chrono::milliseconds duration) {
    const auto end=std::chrono::steady_clock::now()+duration;
    do { executor->spin_some(); std::this_thread::sleep_for(200us); }
    while (std::chrono::steady_clock::now() < end);
  }
  void body(double x) {
    const auto stamp=now(); std_msgs::msg::Float64MultiArray pose, receipt;
    pose.data={2,1,double(++sequence),double(stamp/1000000000LL),double(stamp%1000000000LL),
        x,-.7625,.95,1,0,0,0,1,47,7,8};
    receipt.data=a3_deploy::SessionHomeReceipt{100,-.5,-.7625,.58,7,8,true}.pack(sequence,true);
    base->publish(pose); home->publish(receipt);
  }
  void flight(bool mismatch=false, bool send_packets=true) {
    ModelState physical; physical.position=Vec3(2.5,-.7,.32); physical.velocity=Vec3(-10,0,-.5);
    hope_msgs::msg::BallFlightPacket packet;
    packet.schema_version=1; packet.session_id="ros-test"; packet.producer_instance_id="packetizer";
    packet.trajectory_epoch=91; packet.flight_sequence=1; packet.payload_hash_algorithm=kBallFlightPacketHashAlgorithm;
    packet.frame_id="table"; packet.segment_boundary_reason="initial_incoming";
    packet.net_x=1.37; packet.post_net_delay_s=.05; packet.transmit_count=3;
    const auto start=now(); auto previous=start; bool sent=false;
    while (now()-start < 420000000) {
      const auto stamp=now(); const double dt=(stamp-previous)*1e-9;
      const auto old=physical.position;
      EventGeometry geometry; advance_ball(physical,dt,{},geometry);
      if (!packet.net_cross_exposure_unix_ns && old.x()>1.37 && physical.position.x()<=1.37) {
        const double fraction=(old.x()-1.37)/(old.x()-physical.position.x());
        packet.net_cross_exposure_unix_ns=previous+std::llround(fraction*(stamp-previous));
      }
      previous=stamp;
      BallSample sample; sample.source_time_ns=stamp; sample.position=physical.position;
      sample.sequence=++source_sequence; sample.frame_id="table"; sample.calibration_id="fixture";
      sample.clock_domain="system"; sample.clock_mapping_id="clock"; sample.ball_id="ball";
      observations->publish(encode_observation(sample));
      if (!sent) {
        hope_msgs::msg::BallFlightSample wire;
        wire.exposure_unix_stamp_ns=stamp; wire.position.x=physical.position.x();
        wire.position.y=physical.position.y(); wire.position.z=physical.position.z(); wire.orientation.w=1;
        packet.samples.push_back(wire);
        if (packet.net_cross_exposure_unix_ns && stamp-packet.net_cross_exposure_unix_ns>=50000000) {
          packet.segment_start_exposure_unix_ns=packet.samples.front().exposure_unix_stamp_ns;
          packet.commit_exposure_unix_ns=stamp; packet.freeze_wall_unix_ns=packet.publish_wall_unix_ns=now();
          if (mismatch) packet.samples[5].position.z+=.001;
          packet.payload_hash=flight_packet_message_payload_hash(packet);
          if (send_packets) packets->publish(packet);
          sent=true; commit_source=stamp;
        }
      }
      if (source_sequence%5==0) body(commands.empty() ? -.4 : -.2);
      if (send_packets && sent && source_sequence%10==0) { packet.transmit_index=1; packets->publish(packet); }
      spin(3ms);
    }
    ASSERT_TRUE(sent);
    spin(60ms);
  }
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::shared_ptr<PlannerNode> planner; rclcpp::Node::SharedPtr io;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr base,home;
  rclcpp::Publisher<hope_msgs::msg::BallObservationV2>::SharedPtr observations;
  rclcpp::Publisher<hope_msgs::msg::BallFlightPacket>::SharedPtr packets;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_sub;
  rclcpp::Subscription<hope_msgs::msg::FlightPredictionV2>::SharedPtr prediction_sub;
  rclcpp::Subscription<hope_msgs::msg::StrikeCandidateV1>::SharedPtr candidate_sub,revocation_sub;
  std::vector<std::vector<double>> commands, invalid_commands;
  std::vector<hope_msgs::msg::FlightPredictionV2> predictions;
  std::vector<hope_msgs::msg::StrikeCandidateV1> candidates,revocations;
  a3_pingpong::PpRacketTargetInput receiver{4,true};
  std::uint64_t sequence=0, source_sequence=0; std::int64_t commit_source=0;
  bool continuous_output=true;
};
TEST_P(IncomingRos, ManagedCommandAndContinuousForecastShareFlightAndFrozenPlane) {
  flight();
  ASSERT_EQ(commands.size(),1u);
  ASSERT_EQ(commands[0][1],1.);
  ASSERT_TRUE(receiver.Latest().has_valid);
  const bool fixed=GetParam()=="fixed";
  EXPECT_EQ(commands[0][0],fixed ? 4 : 5);
  EXPECT_NEAR(commands[0][3],fixed ? .08 : .18,1e-8);
  const auto later=std::count_if(predictions.begin(),predictions.end(),[&](const auto& p) {
    return p.prediction_reference_time_ns>commit_source && p.current_state_available;
  });
  EXPECT_GT(later,5);
  EXPECT_TRUE(std::any_of(candidates.begin(),candidates.end(),[&](const auto& c) {
    return c.source_time_ns>commit_source && c.plane_flight_id!=0 && !c.plane_tentative &&
        std::abs(c.plane_x-(fixed ? .08 : .18))<1e-8;
  }));
  EXPECT_FALSE(revocations.empty());
  EXPECT_EQ(invalid_commands.size(),1u);
  EXPECT_TRUE(receiver.Latest().flight_revoked);
  EXPECT_EQ(io->count_publishers("/racket/command_flat"),0u);
}
TEST_P(IncomingRos, InconsistentPacketHistoryAbstainsWhileContinuousTrackingContinues) {
  flight(true);
  EXPECT_TRUE(commands.empty());
  EXPECT_FALSE(receiver.Latest().has_valid);
  EXPECT_GT(predictions.size(),10u);
}
TEST_P(IncomingRos, ContinuousForecastRunsWithoutPacketCommandConsumer) {
  flight(false, false);
  EXPECT_TRUE(commands.empty());
  EXPECT_GT(predictions.size(),10u);
  EXPECT_TRUE(std::any_of(candidates.begin(),candidates.end(),[&](const auto& c) {
    return c.physical_state_available && c.strike_contact_supported &&
        !c.command_eligible && c.plane_flight_id==0 &&
        c.plane_tentative==(GetParam()!="fixed");
  }));
  EXPECT_TRUE(std::any_of(predictions.begin(),predictions.end(),[](const auto& p) {
    return p.forecast_status=="REVOKED" || p.forecast_status=="EXPIRED";
  }));
}
INSTANTIATE_TEST_SUITE_P(PlaneModes,IncomingRos,::testing::Values("fixed","per_ball","stable_per_ball"));
class IncomingOutputDisabled : public IncomingRos {
  void SetUp() override { continuous_output=false; IncomingRos::SetUp(); }
};
TEST_P(IncomingOutputDisabled, DisablingContinuousOutputPreservesOneShotAndRevocations) {
  flight();
  ASSERT_EQ(commands.size(),1u);
  EXPECT_TRUE(receiver.Latest().has_valid);
  EXPECT_TRUE(predictions.empty()); EXPECT_TRUE(candidates.empty());
  EXPECT_FALSE(revocations.empty());
  EXPECT_EQ(invalid_commands.size(),1u);
  EXPECT_TRUE(receiver.Latest().flight_revoked);
}
INSTANTIATE_TEST_SUITE_P(PlaneModes,IncomingOutputDisabled,::testing::Values("fixed","per_ball","stable_per_ball"));
}  // namespace
