  // main_simpack.cpp
  // 位置 /home/yaoyao/Documents/myProjects/ROS2WithSPCK/src/simpack_control/src/main_simpack.cpp

#include <rclcpp/rclcpp.hpp>
#include "simpack_control/SimpackNode.hpp"  // 使用相对于 include 目录的路径

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // 使用多线程执行器
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  // 创建节点对象
  auto node = std::make_shared<SimpackNode>();

  // 将节点加入多线程执行器并开始spin
  executor->add_node(node);
  executor->spin();
  
  rclcpp::shutdown();
  return 0;
}
