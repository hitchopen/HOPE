#include "hope_planner_cpp/hybrid_trajectory_node.hpp"
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try { rclcpp::spin(std::make_shared<hope_planner_cpp::HybridTrajectoryNode>()); }
  catch(const std::exception &e) { fprintf(stderr,"hybrid trajectory: %s\n",e.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
