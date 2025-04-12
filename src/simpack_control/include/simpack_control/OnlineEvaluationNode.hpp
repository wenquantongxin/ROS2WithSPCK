// 文件路径: /home/yaoyao/Documents/myProjects/ROS2WithSPCK/src/simpack_control/include/simpack_control/OnlineEvaluationNode.hpp

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <deque>
#include <vector>
#include <utility>  // for std::pair
#include "simpack_interfaces/msg/simpack_y.hpp"
#include "simpack_interfaces/msg/simpack_w.hpp"

/**
 * @brief 在线计算车辆舒适性(Sperling) + 脱轨系数
 *
 * 功能:
 *   1. 订阅 /simpack/y, 获取加速度和轮轨力等数据
 *   2. Sperling (cubic) 指标: 5 秒滑动窗, 频域加权 => WZc
 *   3. 脱轨系数: Q/P 的 2 秒滑动平均 => derailment_w01, derailment_w02
 *   4. 发布 /simpack/w 消息
 */
class OnlineEvaluationNode : public rclcpp::Node
{
public:
  explicit OnlineEvaluationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~OnlineEvaluationNode();

private:
  // -------- ROS 通信 --------
  rclcpp::Subscription<simpack_interfaces::msg::SimpackY>::SharedPtr sub_y_;
  rclcpp::Publisher<simpack_interfaces::msg::SimpackW>::SharedPtr pub_w_;

  // -------- Sperling 算法相关 --------
  double fs_;                  
  double sperling_win_length_; 
  double derailment_win_length_;

  // 保存5s内的数据
  std::deque<double> time_buffer_;
  std::deque<double> accy_buffer_;
  std::deque<double> accz_buffer_;

  // 上一次有效Sperling计算结果
  double last_sperling_y_;
  double last_sperling_z_;
  bool has_valid_result_;

  // -------- 脱轨系数 相关(2 秒平均) --------
  std::deque<double> contact_time_buffer_;
  std::deque<double> w01_fy_buffer_;
  std::deque<double> w01_fz_buffer_;
  std::deque<double> w02_fy_buffer_;
  std::deque<double> w02_fz_buffer_;

  double last_derailment_w01_;
  double last_derailment_w02_;
  bool   has_valid_derailment_;

  // -------- 回调函数 --------
  void simpackYCallback(const simpack_interfaces::msg::SimpackY::SharedPtr msg);

  // -------- 核心处理函数 --------
  std::pair<double, double> computeSperlingCubic(
    const std::vector<double> & accY,
    const std::vector<double> & accZ,
    double fs);

  std::vector<double> linearInterpolation(
    const std::vector<double> & t_in,
    const std::vector<double> & x_in,
    const std::vector<double> & t_uniform);

  double Bsv(double f);
  double Bsl(double f);
};
