// 文件路径: /home/yaoyao/Documents/myProjects/ROS2WithSPCK/src/simpack_control/src/main_onlinevaluation.cpp

#include "simpack_control/OnlineEvaluationNode.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // 创建节点实例
  rclcpp::NodeOptions options;
  auto node = std::make_shared<OnlineEvaluationNode>(options);
  
  // 运行节点，直到收到终止信号
  rclcpp::spin(node);
  
  // 清理
  rclcpp::shutdown();
  return 0;
}