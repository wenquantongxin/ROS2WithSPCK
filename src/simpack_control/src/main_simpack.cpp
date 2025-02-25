// 文件名: main_simpack.cpp
// 位置 /home/yaoyao/Documents/myProjects/ROS2WithSPCK/src/simpack_control/src/main_simpack.cpp

#include <rclcpp/rclcpp.hpp>
#include "simpack_control/SimpackNode.hpp" 

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

/* MultiThreadedExecutor 的用法:

  MultiThreadedExecutor 会创建一个线程池（默认情况下，线程数等于系统CPU核心数）;

  在ROS2多线程执行器中，并行回调执行：这些线程池中的线程会并行执行各种回调函数。
  
  例如，如果有5个回调同时准备好执行，而线程池有8个线程，则可能有5个回调同时在不同线程中执行。
  
  同一节点的不同回调可并行：来自同一个节点的不同回调（如不同的订阅者回调或定时器回调）可以同时在不同线程中执行。
  
*/ 