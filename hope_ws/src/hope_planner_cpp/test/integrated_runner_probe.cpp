// Non-actuating ROS callback consumer of the actual production mailbox.
#include "a3_pingpong/pp_planner_input.hpp"
#include "hope_planner_cpp/bounded_stage_trace.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("integrated_runner_probe");
  hope_planner_cpp::BoundedStageTrace accounting;
  accounting.configure(node->declare_parameter<std::string>("test.stage_trace_path",""));
  a3_pingpong::PpRacketTargetInput receiver(node->declare_parameter<int>("test.expected_schema",2));
  auto output=node->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/hope/test/integrated_runner/decoded",rclcpp::QoS(10).reliable());
  auto input=node->create_subscription<std_msgs::msg::Float64MultiArray>(
      node->declare_parameter<std::string>("test.input_topic", "/hope/shadow/integrated_planner/racket_target"),rclcpp::QoS(1).reliable(),
      [&](std_msgs::msg::Float64MultiArray::ConstSharedPtr message) {
        const auto receipt=node->now().nanoseconds();
        const auto steady=std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        const auto before_sequence=accounting.enabled()?receiver.Latest().seq:0;
        receiver.SetFromFlat(message->data);
        const auto s=receiver.Latest();
        const auto input_sequence=message->data.size()>14 && std::isfinite(message->data[14]) &&
            message->data[14]>=0 && message->data[14]<=9007199254740991. ?
            static_cast<std::uint64_t>(message->data[14]):0;
        accounting.record("decoder_receipt",0,input_sequence,s.has_valid?s.cmd.flight_id:0,
                          receipt,steady,0,s.seq!=before_sequence?"STATE_CHANGED":"RETAINED_OR_IGNORED");
        accounting.record(s.invalid_after?"decoder_invalid_after":"decoder_result",0,s.seq,s.has_valid?s.cmd.flight_id:0,
                          node->now().nanoseconds(),std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now().time_since_epoch()).count(),
                          0,"input_sequence="+std::to_string(input_sequence));
        if (!s.has_valid) return;
        const auto &c=s.cmd;
        std_msgs::msg::Float64MultiArray decoded;
        decoded.data={double(s.seq),double(c.flight_id),double(c.revision_id),
          c.pos_w.x(),c.pos_w.y(),c.pos_w.z(),c.vel_w.x(),c.vel_w.y(),c.vel_w.z(),
          s.control_time_to_strike_s,s.invalid_after?1.:0.,double(c.estimator_samples),c.estimator_span_s};
        const auto now=node->now().nanoseconds();
        decoded.data.push_back(double(now/1000000000LL));
        decoded.data.push_back(double(now%1000000000LL));
        output->publish(decoded);
      });
  rclcpp::spin(node);accounting.flush();rclcpp::shutdown();return 0;
}
