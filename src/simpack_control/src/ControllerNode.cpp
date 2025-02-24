  // ControllerNode.cpp
  // 位置 /home/yaoyao/Documents/myProjects/ROS2WithSPCK/src/simpack_control/src/ControllerNode.cpp

#include "simpack_control/ControllerNode.hpp"
#include <cmath>
#include <algorithm>
#include "simpack_control/PiecewiseTrajectory.hpp"  // 引入 PiecewiseTrajectory 头文件

ControllerNode::ControllerNode(const rclcpp::NodeOptions & options)
: Node("controller_node", options),
  dt_(0.002),
  wheelRadius_(0.43),
  trajLeft_({0.0, 3.0, 6.0, 100.0}, {16.67, 16.67, 16.87, 16.87}),
  trajRight_({0.0, 3.0, 6.0, 100.0}, {16.67, 16.67, 16.82, 16.82})
{
  // 初始化 8 个 PID
  double kp = 20000.0, ki = 4.0, kd = 0.0, n = 5000.0;
  pids_.reserve(8);
  for(int i = 0; i < 8; ++i){
    pids_.emplace_back(kp, ki, kd, n);
  }

  // 订阅 传感器消息 (SimpackY)
  // 话题名称可根据你的情况调整，如 "/simpack/y"
  sub_sensors_ = create_subscription<simpack_interfaces::msg::SimpackY>(
      "/simpack/y",
      10,
      std::bind(&ControllerNode::sensorCallback, this, std::placeholders::_1));

  // 发布 控制消息 (SimpackU)
  // 话题名称可根据你的情况调整，如 "/simpack/u"
  pub_controls_ = create_publisher<simpack_interfaces::msg::SimpackU>(
      "/simpack/u",
      10);

  // 初始化 PiecewiseTrajectory：设置目标速度曲线
  std::vector<double> times = {0.0, 3.0, 6.0, 100.0};
  std::vector<double> vals_left = {16.67, 16.67, 16.87, 16.87};  // 左侧车轮速度曲线
  std::vector<double> vals_right = {16.67, 16.67, 16.82, 16.82};  // 右侧车轮速度曲线

  trajLeft_ = PiecewiseConstantSecondDeriv(times, vals_left);
  trajRight_ = PiecewiseConstantSecondDeriv(times, vals_right);

  RCLCPP_INFO(this->get_logger(), "ControllerNode started. wheelRadius=%.2f, dt=%.4f",
              wheelRadius_, dt_);
}

void ControllerNode::sensorCallback(const simpack_interfaces::msg::SimpackY::SharedPtr msg)
{
  // 获取当前仿真时刻
  double sim_time = msg->sim_time;

  // 计算左右侧车轮期望速度 (通过目标速度曲线计算)
  double targetLeft  = getTargetSpeedLeft(sim_time);
  double targetRight = getTargetSpeedRight(sim_time);

  // 准备一个 SimpackU 消息, 用来发布 8个车轮的扭矩
  simpack_interfaces::msg::SimpackU control_msg;
  control_msg.sim_time = sim_time;  // 若需要带上同样的时间戳

  // 以第1组轮对 WL01 / WR01 为例:
  {
    // 左车轮(索引0) => y_wl01
    double rawSpeed = msg->y_wl01;    // simpack中可能是负值, 下面要乘 -1
    double actualSpeed = -rawSpeed * wheelRadius_;
    double error = targetLeft - actualSpeed;
    double torque = pids_[0].update(error, dt_);
    control_msg.u0 = torque;
  }
  {
    // 右车轮(索引1) => y_wr01
    double rawSpeed = msg->y_wr01;
    double actualSpeed = -rawSpeed * wheelRadius_;
    double error = targetRight - actualSpeed;
    double torque = pids_[1].update(error, dt_);
    control_msg.u1 = torque;
  }

  // 第2组轮对 WL02 / WR02
  {
    double rawSpeed = msg->y_wl02;
    double actualSpeed = -rawSpeed * wheelRadius_;
    double error = targetLeft - actualSpeed;
    double torque = pids_[2].update(error, dt_);
    control_msg.u2 = torque;
  }
  {
    double rawSpeed = msg->y_wr02;
    double actualSpeed = -rawSpeed * wheelRadius_;
    double error = targetRight - actualSpeed;
    double torque = pids_[3].update(error, dt_);
    control_msg.u3 = torque;
  }

  // 第3组轮对 WL03 / WR03
  {
    double rawSpeed = msg->y_wl03;
    double actualSpeed = -rawSpeed * wheelRadius_;
    double error = targetLeft - actualSpeed;
    double torque = pids_[4].update(error, dt_);
    control_msg.u4 = torque;
  }
  {
    double rawSpeed = msg->y_wr03;
    double actualSpeed = -rawSpeed * wheelRadius_;
    double error = targetRight - actualSpeed;
    double torque = pids_[5].update(error, dt_);
    control_msg.u5 = torque;
  }

  // 第4组轮对 WL04 / WR04
  {
    double rawSpeed = msg->y_wl04;
    double actualSpeed = -rawSpeed * wheelRadius_;
    double error = targetLeft - actualSpeed;
    double torque = pids_[6].update(error, dt_);
    control_msg.u6 = torque;
  }
  {
    double rawSpeed = msg->y_wr04;
    double actualSpeed = -rawSpeed * wheelRadius_;
    double error = targetRight - actualSpeed;
    double torque = pids_[7].update(error, dt_);
    control_msg.u7 = torque;
  }

  // (可选) 对扭矩做限幅
  // e.g.: if (control_msg.u0 > 30000.0) control_msg.u0 = 30000.0;
  //       if (control_msg.u0 < -30000.0) control_msg.u0 = -30000.0;
  //       ... repeat for all

  // 发布控制消息
  pub_controls_->publish(control_msg);
}

// 在 ControllerNode 类中
double ControllerNode::getTargetSpeedLeft(double t)
{
  return trajLeft_.getValue(t);  // 使用 trajLeft_ 计算左侧目标速度
}

double ControllerNode::getTargetSpeedRight(double t)
{
  return trajRight_.getValue(t);  // 使用 trajRight_ 计算右侧目标速度
}
