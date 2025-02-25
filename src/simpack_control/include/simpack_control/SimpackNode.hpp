#ifndef SIMPACK_CONTROL__SIMPACKNODE_HPP_
#define SIMPACK_CONTROL__SIMPACKNODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "simpack_interfaces/msg/simpack_u.hpp"
#include "simpack_interfaces/msg/simpack_y.hpp"

extern "C" {
  #include "/opt/Simpack-2021x/run/realtime/spck_rt.h"
}

class SimpackNode : public rclcpp::Node
{
public:
  explicit SimpackNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~SimpackNode();

private:
  bool initSimpack();
  void DelUnY();

  void controlCallback(const simpack_interfaces::msg::SimpackU::SharedPtr msg);
  void timerCallback();

  // =============== ROS 相关 ===============
  // 控制指令的订阅
  rclcpp::Subscription<simpack_interfaces::msg::SimpackU>::SharedPtr sub_controls_;

  // 传感器数据的发布
  rclcpp::Publisher<simpack_interfaces::msg::SimpackY>::SharedPtr pub_sensors_;

  // 定时器 (用于固定步长推进仿真、发布y)
  rclcpp::TimerBase::SharedPtr timer_;

  // 专门用于控制订阅回调的 CallbackGroup
  rclcpp::CallbackGroup::SharedPtr callback_group_controls_;

  // =============== SIMPACK 相关 ===============
  int nu_;        // 输入维度
  int ny_;        // 输出维度
  double* u_;     // SIMPACK输入数组
  double* y_;     // SIMPACK输出数组

  // 仿真时间相关
  double simTime_;      // 当前仿真时刻 (单位: 秒)
  double dt_;           // 仿真步长
  double sim_duration_; // 仿真总时长
  int max_steps_;       // 最大步数
  int step_count_;      // 已执行步数

  // 读写文件
  std::ostringstream logBuffer_U;
  std::ostringstream logBuffer_Y;
};

#endif  // SIMPACK_CONTROL__SIMPACKNODE_HPP_
