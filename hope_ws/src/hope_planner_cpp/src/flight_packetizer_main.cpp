#include "hope_planner_cpp/flight_packetizer_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hope_planner_cpp::FlightPacketizerNode>());
  rclcpp::shutdown();
  return 0;
}
