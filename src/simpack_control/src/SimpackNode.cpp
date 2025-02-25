// 文件名: SimpackNode.cpp
// 文件位置: /.../ROS2WithSPCK/src/simpack_control/src/SimpackNode.cpp

#include "simpack_control/SimpackNode.hpp"

// 包含自定义消息自“simpack_interfaces”包的头文件
#include "simpack_interfaces/msg/simpack_u.hpp"
#include "simpack_interfaces/msg/simpack_y.hpp"

#include <cstring>
#include <chrono>  // for chrono literals
#include <memory>
#include <iostream>
#include <fstream>   // 用于文件输出
#include <vector>
#include <cmath>
#include <string>
#include <sstream>    // std::ostringstream
#include <cstdlib>   // 常用工具函数

// 宏定义, 主要是 SIMPACK 传参
#define SPCK_PATH "/opt/Simpack-2021x"
#define MODEL_FILE "/home/yaoyao/Documents/myProjects/ROS2WithSPCK/SPCK_Model/Vehicle4WDB_RealtimeCRV.spck"
#define SPCK_MODE E_RT_MODE__KEEP_SLV
#define SPCK_CPUS ""
#define SPCK_RT_PRIO 40
#define SPCK_VERBOSE 0
#define SPCK_POSIX_TIMEOUT 5

// 默认仿真步长 = 2ms (500Hz)
static const double DEFAULT_DT = 0.002;

// 默认仿真总时长 = 50s
static const double DEFAULT_SIM_DURATION = 50.0;

// 方便后续计时
using namespace std::chrono_literals;

// 注意：需要保证与 SIMPACK 模型 y-vector 顺序匹配！
// 例如，如果 $Y_Yw01 处于 y_[0], $Y_WL01 处于 y_[8], ...
// 显式列出索引，便于后续赋值

static const int INDEX_YW01 = 0;
static const int INDEX_YW02 = 1;
static const int INDEX_YW03 = 2;
static const int INDEX_YW04 = 3;

static const int INDEX_YAW01 = 4;
static const int INDEX_YAW02 = 5;
static const int INDEX_YAW03 = 6;
static const int INDEX_YAW04 = 7;

static const int INDEX_WL01 = 8;
static const int INDEX_WR01 = 9;
static const int INDEX_WL02 = 10;
static const int INDEX_WR02 = 11;
static const int INDEX_WL03 = 12;
static const int INDEX_WR03 = 13;
static const int INDEX_WL04 = 14;
static const int INDEX_WR04 = 15;

static const int INDEX_YT01 = 16;
static const int INDEX_YT02 = 17;
static const int INDEX_YAWT01 = 18;
static const int INDEX_YAWT02 = 19;

static const int INDEX_SPEEDDIFF_FRONT_A = 20;
static const int INDEX_SPEEDDIFF_FRONT_B = 21;
static const int INDEX_SPEEDDIFF_FRONT_C = 22;
static const int INDEX_SPEEDDIFF_FRONT_D = 23;
static const int INDEX_SPEEDDIFF_REAR_A  = 24;
static const int INDEX_SPEEDDIFF_REAR_B  = 25;
static const int INDEX_SPEEDDIFF_REAR_C  = 26;
static const int INDEX_SPEEDDIFF_REAR_D  = 27;

// 日志记录 - Y 
// 定义数组记录列名。注意: 数组大小必须是 28+1=29, 第一个是 "Time"，后面是 28 个 y-output。
static const char* Y_columnNames[29] = {
"\"Time\"",
"\"$Y_Yw01\"", "\"$Y_Yw02\"", "\"$Y_Yw03\"", "\"$Y_Yw04\"",
"\"$Y_Yaw01\"", "\"$Y_Yaw02\"", "\"$Y_Yaw03\"", "\"$Y_Yaw04\"",
"\"$Y_WL01\"", "\"$Y_WR01\"", "\"$Y_WL02\"", "\"$Y_WR02\"",
"\"$Y_WL03\"", "\"$Y_WR03\"", "\"$Y_WL04\"", "\"$Y_WR04\"",
"\"$Y_Yt01\"", "\"$Y_Yt02\"", "\"$Y_Yawt01\"", "\"$Y_Yawt02\"",
"\"$Y_SpeedDiff_FrontA\"", "\"$Y_SpeedDiff_FrontB\"", "\"$Y_SpeedDiff_FrontC\"", "\"$Y_SpeedDiff_FrontD\"",
"\"$Y_SpeedDiff_RearA\"",  "\"$Y_SpeedDiff_RearB\"",  "\"$Y_SpeedDiff_RearC\"",  "\"$Y_SpeedDiff_RearD\""
};
// 日志记录 - U 
static const char* U_columnNames[9] = {
  "\"Time\"",
  "\"$UI_00\"", "\"$UI_01\"", "\"$UI_02\"", "\"$UI_03\"",
  "\"$UI_04\"", "\"$UI_05\"", "\"$UI_06\"", "\"$UI_07\""};

SimpackNode::SimpackNode(const rclcpp::NodeOptions & options)
: Node("simpack_node", options),
  simTime_(0.0),
  dt_(DEFAULT_DT),
  sim_duration_(DEFAULT_SIM_DURATION),
  nu_(0),
  ny_(0),
  u_(nullptr),
  y_(nullptr),
  step_count_(0)
{
  // 1) 创建一个独立CallbackGroup，用来处理控制输入的回调
  callback_group_controls_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  // 2) 设置控制订阅的SubscriptionOptions，指定其使用上述CallbackGroup
  rclcpp::SubscriptionOptions control_sub_options;
  control_sub_options.callback_group = callback_group_controls_;

  // 3) 创建控制订阅，并使用相对“低延迟”的QoS配置（示例用 best_effort）
  sub_controls_ = this->create_subscription<simpack_interfaces::msg::SimpackU>(
    "/simpack/u",
    rclcpp::QoS(1).reliable(),
    std::bind(&SimpackNode::controlCallback, this, std::placeholders::_1),
    control_sub_options);

  // 4) 初始化SIMPACK
  if(!initSimpack()){
    RCLCPP_ERROR(this->get_logger(), "Simpack init failed!");
    // 若初始化失败可抛异常或置标志位
    return;
  }

  // 5.1) 计算最大步数
  max_steps_ = static_cast<int>(sim_duration_ / dt_ + 0.5);
  // 5.2) 创建日志文件 - Y 
  {
    std::ifstream checkFile_Y("./PostAnalysis/Result_Y_RosRt.log");
    if (!checkFile_Y.good()) {
      // 文件不存在,创建一个空文件
      std::ofstream createFile("./PostAnalysis/Result_Y_RosRt.log");
      createFile.close();
    }
    checkFile_Y.close();
  
  std::ofstream logFile_Y("./PostAnalysis/Result_Y_RosRt.log");
  if (!logFile_Y.is_open()) {
    std::cerr << "[Error 错误] Failed to open ./PostAnalysis/Result_Y_RosRt.log for writing.\n";
    return; // 或其他适当的错误处理，如抛出异常
  }
  
  // 先写一个标题行，包含 Time 以及 28 个 y-output
  // 不再使用 # time(s) y0 y1 ... y27, 而是改为以双引号+Tab 分隔
  for (int i = 0; i < 29; ++i) {
    logFile_Y << Y_columnNames[i];
    if (i < 28) {
      logFile_Y << "\t";  // 列间用TAB分隔
    }
    else {
      logFile_Y << "\n";  // 最后一列后换行
    }
  }
  logFile_Y.close(); // 确保关闭文件
  std::ostringstream logBuffer_Y; //后续不直接写文件, 改为写内存缓冲 - Y
  }
  // 5.3) 创建日志文件 - U 
  {
    std::ifstream checkFile_U("./PostAnalysis/Result_U_RosRt.log");
    if (!checkFile_U.good()) {
      // 文件不存在,创建一个空文件
      std::ofstream createFile("./PostAnalysis/Result_U_RosRt.log");
      createFile.close();
    }
  checkFile_U.close();
  
  std::ofstream logFile_U("./PostAnalysis/Result_U_RosRt.log");
  if (!logFile_U.is_open()) {
    std::cerr << "[Error 错误] Failed to open ./PostAnalysis/Result_U_RosRt.log for writing.\n";
    return; // 或其他适当的错误处理，如抛出异常
  }
  
  // 先写一个标题行，包含 Time 以及 8 个 u-input
  for (int i = 0; i < 9; ++i) {
    logFile_U << U_columnNames[i];
    if (i < 8) {
      logFile_U << "\t";  // 列间用TAB分隔
    }
    else {
      logFile_U << "\n";  // 最后一列后换行
    }
  }
  logFile_U.close(); // 确保关闭文件
  std::ostringstream logBuffer_U; //后续不直接写文件, 改为写内存缓冲 - U
  }

  // 6) 创建发布者
  pub_sensors_ = this->create_publisher<simpack_interfaces::msg::SimpackY>(
      "/simpack/y", 10);
 
  // 7) 创建定时器：保持 0.002s 间隔，用于推进仿真 & 发布 y
  auto period = std::chrono::duration<double>(dt_); // 0.002s
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&SimpackNode::timerCallback, this));
 
  RCLCPP_INFO(this->get_logger(),
      "SimpackNode constructed. dt=%.4f, max_steps=%d",
      dt_, max_steps_);
}

// 析构函数
SimpackNode::~SimpackNode()
{
  DelUnY(); // 删除 u_ 和 y_ 指针, 避免野指针
  RCLCPP_INFO(this->get_logger(), "SimpackNode 节点关闭!");
}

// 初始化SIMPACK
bool SimpackNode::initSimpack()
{
  int ierr = SpckRtInitPM(SPCK_MODE, SPCK_PATH, MODEL_FILE,
                          SPCK_CPUS, SPCK_RT_PRIO, SPCK_VERBOSE,
                          nullptr, SPCK_POSIX_TIMEOUT);
  if (ierr != 0) {
    RCLCPP_ERROR(this->get_logger(), "SpckRtInitPM failed, ierr=%d", ierr);
    return false;
  }

  // 获取u,y的维度
  SpckRtGetUYDim(&nu_, &ny_);
  if (nu_ <= 0 || ny_ <= 0) {
    RCLCPP_ERROR(this->get_logger(), "Invalid dimension: nu=%d, ny=%d", nu_, ny_);
    return false;
  }

  // 分配内存
  u_ = new double[nu_];
  y_ = new double[ny_];
  std::memset(u_, 0, sizeof(double)*nu_);
  std::memset(y_, 0, sizeof(double)*ny_);

  // 启动solver
  ierr = SpckRtStart();
  if (ierr != 0){
    RCLCPP_ERROR(this->get_logger(), "SpckRtStart failed, ierr=%d", ierr);
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "initSimpack done. nu=%d, ny=%d", nu_, ny_);
  return true;
}

// 删除指针
void SimpackNode::DelUnY()
{
  // SpckRtFinish();
  if (u_) { delete[] u_; u_ = nullptr; }
  if (y_) { delete[] y_; y_ = nullptr; }
}

// ================== 回调: 接收控制输入 ==================
void SimpackNode::controlCallback(
  const simpack_interfaces::msg::SimpackU::SharedPtr msg)
{
  // 将 msg->u0..u7 赋给内部 u_[0..7]
  // 请确认: SIMPACK 模型确实 8 个输入
  if (nu_ == 8) {
    u_[0] = msg->u0;
    u_[1] = msg->u1;
    u_[2] = msg->u2;
    u_[3] = msg->u3;
    u_[4] = msg->u4;
    u_[5] = msg->u5;
    u_[6] = msg->u6;
    u_[7] = msg->u7;
  } else {
    RCLCPP_WARN(this->get_logger(),
      "controlCallback: nu_=%d mismatch with expected 8", nu_);
  }
}

// ================== 定时器: 每步仿真循环 ==================
void SimpackNode::timerCallback()
{
  // 如果达到最大步数，就停止
  if (step_count_ >= max_steps_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Reached max_steps(%d), no further sim steps!", max_steps_);

  // 文件写入代码 - Y文件
  {
    std::ofstream logFile_Y("./PostAnalysis/Result_Y_RosRt.log", std::ios::app);  // 以追加模式打开
    if(!logFile_Y.is_open()){
      std::cerr << "无法打开 ./PostAnalysis/Result_Y_RosRt.log 文件\n";
      RCLCPP_ERROR(this->get_logger(), "无法打开 ./PostAnalysis/Result_Y_RosRt.log 文件");
      // 这里应考虑返回错误状态或抛出异常
    } else {
      logFile_Y << logBuffer_Y.str();
      logFile_Y.close();
      RCLCPP_INFO(this->get_logger(), "Y仿真数据已写入 ./PostAnalysis/Result_Y_RosRt.log");
    }
  }
  
  // 文件写入代码 - U文件
  {
    std::ofstream logFile_U("./PostAnalysis/Result_U_RosRt.log", std::ios::app);  // 以追加模式打开
    if(!logFile_U.is_open()){
      std::cerr << "无法打开 ./PostAnalysis/Result_U_RosRt.log 文件\n";
      RCLCPP_ERROR(this->get_logger(), "无法打开 ./PostAnalysis/Result_U_RosRt.log 文件");
      // 这里应考虑返回错误状态或抛出异常
    } else {
      logFile_U << logBuffer_U.str();
      logFile_U.close();
      RCLCPP_INFO(this->get_logger(), "U仿真数据已写入 ./PostAnalysis/Result_U_RosRt.log");
    }
  }

  // 在仿真结束时主动退出整个 ROS 进程；但如果将来想让该节点与其他节点共存，而不想整体关闭 ROS
  // 可以考虑只 cancel()定时器并做一些收尾工作，不调用rclcpp::shutdown()
  SpckRtFinish();
  timer_->cancel();
  rclcpp::shutdown();
  return;
  }

  // 1) 读取 y[]
  SpckRtGetY(y_);

  // 2) 发布 y: 28个量
  simpack_interfaces::msg::SimpackY sensor_msg;
  sensor_msg.sim_time = simTime_; // 当前仿真时刻

  // 这里演示只给轮速赋值( WL01..WR04 )，其余请按需补充
  // 请确认: 索引正确
  if (ny_ >= 16) {
    sensor_msg.y_wl01 = y_[INDEX_WL01];
    sensor_msg.y_wr01 = y_[INDEX_WR01];
    sensor_msg.y_wl02 = y_[INDEX_WL02];
    sensor_msg.y_wr02 = y_[INDEX_WR02];
    sensor_msg.y_wl03 = y_[INDEX_WL03];
    sensor_msg.y_wr03 = y_[INDEX_WR03];
    sensor_msg.y_wl04 = y_[INDEX_WL04];
    sensor_msg.y_wr04 = y_[INDEX_WR04];
    // ...
    // 其余 20 个量: y_yw01..y_speed_diff_rear_d
    // 按索引依次赋值即可
  } else {
    RCLCPP_WARN_ONCE(this->get_logger(),
      "ny_=%d is smaller than expected for full assignment!", ny_);
  }
  pub_sensors_->publish(sensor_msg);

  // 3) 写 u_ 到 SIMPACK
  SpckRtSetU(u_);

  // 3.5) 记录到 logBuffer_Y 和 logBuffer_U 
  logBuffer_Y << simTime_;
  for (int j=0; j<ny_; ++j) {
    logBuffer_Y << "\t" << y_[j];
  }
  logBuffer_Y << "\n";

  logBuffer_U << simTime_;
  for (int j=0; j<nu_; ++j) {
    logBuffer_U << "\t" << u_[j];
  }
  logBuffer_U << "\n";

  // 4) 前进仿真
  double nextTime = (step_count_ + 1) * dt_;
  int ierr = SpckRtAdvance(nextTime);
  if (ierr == 1){
    RCLCPP_ERROR(this->get_logger(), "SpckRtAdvance solver error");
  } else if (ierr == 2){
    RCLCPP_WARN(this->get_logger(), "SpckRtAdvance timeout");
  }

  step_count_++;
  simTime_ = nextTime;
}