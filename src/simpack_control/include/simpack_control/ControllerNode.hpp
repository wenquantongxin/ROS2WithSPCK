#pragma once

#include <rclcpp/rclcpp.hpp>
#include "simpack_interfaces/msg/simpack_y.hpp"
#include "simpack_interfaces/msg/simpack_u.hpp"

#include "simpack_control/PiecewiseTrajectory.hpp" 

#include "PIDController.hpp"  // 你的PID类

class ControllerNode : public rclcpp::Node
{
public:
  explicit ControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ControllerNode() = default;

private:
  void sensorCallback(const simpack_interfaces::msg::SimpackY::SharedPtr msg);
  
  // 发布控制量
  rclcpp::Publisher<simpack_interfaces::msg::SimpackU>::SharedPtr pub_controls_;
  // 订阅 y-outputs
  rclcpp::Subscription<simpack_interfaces::msg::SimpackY>::SharedPtr sub_sensors_;

  // 8 个PID
  std::vector<PIDController> pids_;
  // 哪些y-index对应车轮(跟你在Test_SpckRT.cpp相同)
  std::vector<int> wheelIndex_;
  // 左右轮标记
  std::vector<bool> isLeftWheel_;

  double dt_;
  double wheelRadius_;
  
  // 目标速度, 也可引入PiecewiseTrajectory
  // 这里仅简单示例, 实际可做成class + 读表
  double getTargetSpeedLeft(double t);
  double getTargetSpeedRight(double t);

  // 需要添加的新成员变量
  PiecewiseConstantSecondDeriv trajLeft_;    // 左侧车轮的轨迹
  PiecewiseConstantSecondDeriv trajRight_;   // 右侧车轮的轨迹
};
