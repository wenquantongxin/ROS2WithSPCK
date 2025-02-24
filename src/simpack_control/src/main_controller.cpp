// main_controller.cpp
// 位置 /home/yaoyao/Documents/myProjects/ROS2WithSPCK/src/simpack_control/src/main_controller.cpp

#include <rclcpp/rclcpp.hpp>
#include "simpack_control/ControllerNode.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
