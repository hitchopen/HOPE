// Non-actuating ROS callback consumer of the actual production mailbox.
#include "a3_pingpong/pp_planner_input.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("integrated_runner_probe");
  a3_pingpong::PpRacketTargetInput receiver(2);
  auto output=node->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/hope/test/integrated_runner/decoded",rclcpp::QoS(10).reliable());
  auto input=node->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/hope/shadow/integrated_planner/racket_target",rclcpp::QoS(1).reliable(),
      [&](std_msgs::msg::Float64MultiArray::ConstSharedPtr message) {
        receiver.SetFromFlat(message->data);
        const auto s=receiver.Latest();
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
  rclcpp::spin(node);rclcpp::shutdown();return 0;
}
