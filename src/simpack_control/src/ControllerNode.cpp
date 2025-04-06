// 文件名称: ControllerNode.cpp
// 位置 /home/yaoyao/Documents/myProjects/ROS2WithSPCK/src/simpack_control/src/ControllerNode.cpp

#include <cmath>
#include <algorithm>
#include "simpack_control/ControllerNode.hpp"
#include "simpack_control/PiecewiseTrajectory.hpp"  // 引入 PiecewiseTrajectory 头文件

ControllerNode::ControllerNode(const rclcpp::NodeOptions & options)
: Node("controller_node", options),
  dt_(0.002),
  wheelRadius_(0.43),
  // trajLeft_ / trajRight_ 默认构造
   trajLeft_({0.0, 3.0, 6.0, 100.0}, {16.667, 16.667, 16.708, 16.708}),
  trajRight_({0.0, 3.0, 6.0, 100.0}, {16.667, 16.667, 16.708, 16.708})
{
  // 初始化 8 个 PID
  double kp = 20000.0, ki = 4.0, kd = 0.0, n = 5000.0;
  pids_.reserve(8);
  for(int i = 0; i < 8; ++i){
    pids_.emplace_back(kp, ki, kd, n);
  }

  // 订阅 传感器消息 "/simpack/y"
  sub_sensors_ = create_subscription<simpack_interfaces::msg::SimpackY>(
      "/simpack/y",
      10, // 消息队列"深度"(depth)
      std::bind(&ControllerNode::sensorCallback, this, std::placeholders::_1));

  // 发布 控制消息 "/simpack/u"
  pub_controls_ = create_publisher<simpack_interfaces::msg::SimpackU>(
      "/simpack/u",
      10);

  // 初始化 PiecewiseTrajectory：设置目标速度曲线
  //std::vector<double> times = {0.0, 3.0, 6.0, 100.0};
  //std::vector<double> vals_left =  {16.667, 16.667, 16.708, 16.708};  // 左侧车轮速度曲线 
  //std::vector<double> vals_right = {16.667, 16.667, 16.625, 16.625};  // 右侧车轮速度曲线 

  // 直接在初始化列表中对 trajLeft_ 和 trajRight_ 进行有参构造
  /* 
  由 Analysis_SpeedCtrl.ipynb 生成
    ===== 速度转折点(含段首/段中/段尾)列表 =====
          time_s  speed_m_s  speed_km_h    distance_m
  0     0.000000   2.777778        10.0      0.000000
  1    21.500000  22.222222        80.0    281.210667
  2   192.731900  22.222222        80.0   4086.364000
  3   206.616810  33.333333       120.0   4472.587484
  4   228.030106  33.333333       120.0   5186.364000
  5   241.926061  44.444444       160.0   5727.354510
  6   263.053775  44.444444       160.0   6666.364000
  7   285.153775  55.555556       200.0   7771.247333
  8   324.845875  55.555556       200.0   9976.364000
  9   335.845875  61.111111       220.0  10618.005667
  10  390.800829  61.111111       220.0  13976.364000
  11  412.919952  50.000000       180.0  15205.217979
  12  442.342873  50.000000       180.0  16676.364000
  13  502.842873   2.777778        10.0  18407.047778
  14  599.796713   2.777778        10.0  18676.364000
*/

// 定义时间、速度序列
std::vector<double> predefined_times = {
    0.000000,   21.500000, 192.731900, 206.616810, 228.030106,
  241.926061, 263.053775, 285.153775, 324.845875, 335.845875,
  390.800829, 412.919952, 442.342873, 502.842873, 599.796713
};

std::vector<double> predefined_speeds = {
   2.777778,  22.222222,  22.222222,  33.333333,  33.333333,
  44.444444,  44.444444,  55.555556,  55.555556,  61.111111,
  61.111111,  50.000000,  50.000000,   2.777778,   2.777778
};

// 赋值给 trajLeft_ / trajRight_
trajLeft_  = PiecewiseConstantSecondDeriv(predefined_times, predefined_speeds);
trajRight_ = PiecewiseConstantSecondDeriv(predefined_times, predefined_speeds);

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

  // 以第1组轮对 y_w01_rotw / y_w02_rotw 为例:
  {
    // 左车轮(索引0) => 原y_wl01
    double rawSpeed = msg->y_w01_rotw;    // SPCK 直接读出的是负值, 需要乘-1
    double actualSpeed = -rawSpeed * wheelRadius_;
    double error = targetLeft - actualSpeed;
    double torque = pids_[0].update(error, dt_);
    control_msg.u0 = torque;
  }
  {
    // 右车轮(索引1) => 原y_wr01
    double rawSpeed = msg->y_w02_rotw;
    double actualSpeed = -rawSpeed * wheelRadius_;
    double error = targetRight - actualSpeed;
    double torque = pids_[1].update(error, dt_);
    control_msg.u1 = torque;
  }
  // 第2组轮对 WL02 / WR02
  {
    double rawSpeed = msg->y_w03_rotw;
    double actualSpeed = -rawSpeed * wheelRadius_;
    double error = targetLeft - actualSpeed;
    double torque = pids_[2].update(error, dt_);
    control_msg.u2 = torque;
  }
  {
    double rawSpeed = msg->y_w04_rotw;
    double actualSpeed = -rawSpeed * wheelRadius_;
    double error = targetRight - actualSpeed;
    double torque = pids_[3].update(error, dt_);
    control_msg.u3 = torque;
  }
  // 第3组轮对 WL03 / WR03
  {
    double rawSpeed = msg->y_w05_rotw;
    double actualSpeed = -rawSpeed * wheelRadius_;
    double error = targetLeft - actualSpeed;
    double torque = pids_[4].update(error, dt_);
    control_msg.u4 = torque;
  }
  {
    double rawSpeed = msg->y_w06_rotw;
    double actualSpeed = -rawSpeed * wheelRadius_;
    double error = targetRight - actualSpeed;
    double torque = pids_[5].update(error, dt_);
    control_msg.u5 = torque;
  }

  // 第4组轮对 WL04 / WR04
  {
    double rawSpeed = msg->y_w07_rotw;
    double actualSpeed = -rawSpeed * wheelRadius_;
    double error = targetLeft - actualSpeed;
    double torque = pids_[6].update(error, dt_);
    control_msg.u6 = torque;
  }
  {
    double rawSpeed = msg->y_w08_rotw;
    double actualSpeed = -rawSpeed * wheelRadius_;
    double error = targetRight - actualSpeed;
    double torque = pids_[7].update(error, dt_);
    control_msg.u7 = torque;
  }
  // (可选) 对扭矩做限幅
  // if (control_msg.u0 > 30000.0) control_msg.u0 = 30000.0;
  // if (control_msg.u0 < -30000.0) control_msg.u0 = -30000.0;
  // ......

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
