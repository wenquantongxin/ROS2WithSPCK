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
static const double DEFAULT_SIM_DURATION = 500.0;

// 方便后续计时
using namespace std::chrono_literals;

// 注意：需要保证与 SIMPACK 模型 y-vector 顺序、ROS 2 msg 顺序匹配
// 显式列出索引，便于后续赋值

static const int INDEX_SPCKTIME = 0;
static const int INDEX_CB_Vx = 1;

static const int INDEX_CB_X = 2;
static const int INDEX_CB_Y = 3;
static const int INDEX_CB_Z = 4;
static const int INDEX_CB_Roll= 5;
static const int INDEX_CB_Yaw = 6;
static const int INDEX_CB_Pit = 7;

static const int INDEX_W01_RotW = 8;  // 车轮01 转速
static const int INDEX_W02_RotW = 9;
static const int INDEX_W03_RotW = 10;
static const int INDEX_W04_RotW = 11;
static const int INDEX_W05_RotW = 12;
static const int INDEX_W06_RotW = 13;
static const int INDEX_W07_RotW = 14;
static const int INDEX_W08_RotW = 15;

static const int INDEX_F01_X = 16;
static const int INDEX_F01_Y = 17;
static const int INDEX_F01_Z = 18;
static const int INDEX_F01_Roll= 19;
static const int INDEX_F01_Yaw = 20;
static const int INDEX_F01_Pit = 21;

static const int INDEX_F02_X = 22;
static const int INDEX_F02_Y = 23;
static const int INDEX_F02_Z = 24;
static const int INDEX_F02_Roll= 25;
static const int INDEX_F02_Yaw = 26;
static const int INDEX_F02_Pit = 27;

static const int INDEX_WS01_X = 28;
static const int INDEX_WS01_Y = 29;
static const int INDEX_WS01_Z = 30;
static const int INDEX_WS01_Roll= 31;
static const int INDEX_WS01_Yaw = 32;
static const int INDEX_WS01_Pit = 33;

static const int INDEX_WS02_X = 34;
static const int INDEX_WS02_Y = 35;
static const int INDEX_WS02_Z = 36;
static const int INDEX_WS02_Roll= 37;
static const int INDEX_WS02_Yaw = 38;
static const int INDEX_WS02_Pit = 39;

static const int INDEX_WS03_X = 40;
static const int INDEX_WS03_Y = 41;
static const int INDEX_WS03_Z = 42;
static const int INDEX_WS03_Roll= 43;
static const int INDEX_WS03_Yaw = 44;
static const int INDEX_WS03_Pit = 45;

static const int INDEX_WS04_X = 46;
static const int INDEX_WS04_Y = 47;
static const int INDEX_WS04_Z = 48;
static const int INDEX_WS04_Roll= 49;
static const int INDEX_WS04_Yaw = 50;
static const int INDEX_WS04_Pit = 51;

static const int INDEX_W01_RotA = 52;  // 车轮01 转动角度
static const int INDEX_W02_RotA = 53;
static const int INDEX_W03_RotA = 54;
static const int INDEX_W04_RotA = 55;
static const int INDEX_W05_RotA = 56;
static const int INDEX_W06_RotA = 57;
static const int INDEX_W07_RotA = 58;
static const int INDEX_W08_RotA = 59;

static const int INDEX_Bar01_Pit = 60;  // 杆件01 转动角度
static const int INDEX_Bar02_Pit = 61;
static const int INDEX_Bar03_Pit = 62;
static const int INDEX_Bar04_Pit = 63;
static const int INDEX_Bar05_Pit = 64;
static const int INDEX_Bar06_Pit = 65;
static const int INDEX_Bar07_Pit = 66;
static const int INDEX_Bar08_Pit = 67;

static const int INDEX_WS01_Vy = 68;
static const int INDEX_WS02_Vy = 69;
static const int INDEX_WS03_Vy = 70;
static const int INDEX_WS04_Vy = 71;

static const int INDEX_WS01_Vyaw = 72;
static const int INDEX_WS02_Vyaw = 73;
static const int INDEX_WS03_Vyaw = 74;
static const int INDEX_WS04_Vyaw = 75;

// 日志记录 - Y 
// 定义数组记录列名。注意: 数组大小必须是 76+1=77, 第一个是 "Time"，后面是 76(原28)个 y-output。
static const char* Y_columnNames[77] = {
"\"Time\"", 
"\"y_spcktime\"", 
"\"y_cb_vx\"", "\"y_cb_x\"", "\"y_cb_y\"", "\"y_cb_z\"", "\"y_cb_roll\"", "\"y_cb_yaw\"", "\"y_cb_pitch\"", 
"\"y_w01_rotw\"", "\"y_w02_rotw\"", "\"y_w03_rotw\"", "\"y_w04_rotw\"", "\"y_w05_rotw\"", "\"y_w06_rotw\"", "\"y_w07_rotw\"", "\"y_w08_rotw\"", 
"\"y_f01_x\"", "\"y_f01_y\"", "\"y_f01_z\"", "\"y_f01_roll\"", "\"y_f01_yaw\"", "\"y_f01_pitch\"", 
"\"y_f02_x\"", "\"y_f02_y\"", "\"y_f02_z\"", "\"y_f02_roll\"", "\"y_f02_yaw\"", "\"y_f02_pitch\"", 
"\"y_ws01_x\"", "\"y_ws01_y\"", "\"y_ws01_z\"", "\"y_ws01_roll\"", "\"y_ws01_yaw\"", "\"y_ws01_pitch\"", 
"\"y_ws02_x\"", "\"y_ws02_y\"", "\"y_ws02_z\"", "\"y_ws02_roll\"", "\"y_ws02_yaw\"", "\"y_ws02_pitch\"", 
"\"y_ws03_x\"", "\"y_ws03_y\"", "\"y_ws03_z\"", "\"y_ws03_roll\"", "\"y_ws03_yaw\"", "\"y_ws03_pitch\"", 
"\"y_ws04_x\"", "\"y_ws04_y\"", "\"y_ws04_z\"", "\"y_ws04_roll\"", "\"y_ws04_yaw\"", "\"y_ws04_pitch\"", 
"\"y_w01_rota\"", "\"y_w02_rota\"", "\"y_w03_rota\"", "\"y_w04_rota\"", 
"\"y_w05_rota\"", "\"y_w06_rota\"", "\"y_w07_rota\"", "\"y_w08_rota\"", 
"\"y_bar01_pitch\"", "\"y_bar02_pitch\"", "\"y_bar03_pitch\"", "\"y_bar04_pitch\"", 
"\"y_bar05_pitch\"", "\"y_bar06_pitch\"", "\"y_bar07_pitch\"", "\"y_bar08_pitch\"", 
"\"y_ws01_vy\"", "\"y_ws02_vy\"", "\"y_ws03_vy\"", "\"y_ws04_vy\"", 
"\"y_ws01_vyaw\"", "\"y_ws02_vyaw\"", "\"y_ws03_vyaw\"", "\"y_ws04_vyaw\""
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
  
  // 先写一个标题行，包含 Time 以及 76(原28) 个 y-output
  // 不再使用 # time(s) y0 y1 ... y27, 而是改为以双引号+Tab 分隔
  for (int i = 0; i < 77; ++i) {
    logFile_Y << Y_columnNames[i];
    if (i < 76) {
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

  // 2) 发布 y: 76个量
  simpack_interfaces::msg::SimpackY sensor_msg;
  sensor_msg.sim_time = simTime_; // 当前仿真时刻

  // 建议: 如果要确保一次性完整赋值全部 76 个量，检查条件最好改为 (ny_ >= 76)
  if (ny_ >= 76) 
  {
    // 0. SIMPACK 内部时间
    sensor_msg.y_spcktime   = y_[INDEX_SPCKTIME];

    // 1. 车体状态
    sensor_msg.y_cb_vx      = y_[INDEX_CB_Vx];  // 整车纵向速度
    sensor_msg.y_cb_x       = y_[INDEX_CB_X];
    sensor_msg.y_cb_y       = y_[INDEX_CB_Y];
    sensor_msg.y_cb_z       = y_[INDEX_CB_Z];
    sensor_msg.y_cb_roll    = y_[INDEX_CB_Roll];
    sensor_msg.y_cb_yaw     = y_[INDEX_CB_Yaw];
    sensor_msg.y_cb_pitch   = y_[INDEX_CB_Pit];

    // 2. 车轮转速 (RotW)
    sensor_msg.y_w01_rotw   = y_[INDEX_W01_RotW];
    sensor_msg.y_w02_rotw   = y_[INDEX_W02_RotW];
    sensor_msg.y_w03_rotw   = y_[INDEX_W03_RotW];
    sensor_msg.y_w04_rotw   = y_[INDEX_W04_RotW];
    sensor_msg.y_w05_rotw   = y_[INDEX_W05_RotW];
    sensor_msg.y_w06_rotw   = y_[INDEX_W06_RotW];
    sensor_msg.y_w07_rotw   = y_[INDEX_W07_RotW];
    sensor_msg.y_w08_rotw   = y_[INDEX_W08_RotW];

    // 3. 前后2个构架 (x, y, z, roll, yaw, pitch)
    sensor_msg.y_f01_x      = y_[INDEX_F01_X];
    sensor_msg.y_f01_y      = y_[INDEX_F01_Y];
    sensor_msg.y_f01_z      = y_[INDEX_F01_Z];
    sensor_msg.y_f01_roll   = y_[INDEX_F01_Roll];
    sensor_msg.y_f01_yaw    = y_[INDEX_F01_Yaw];
    sensor_msg.y_f01_pitch  = y_[INDEX_F01_Pit];

    sensor_msg.y_f02_x      = y_[INDEX_F02_X];
    sensor_msg.y_f02_y      = y_[INDEX_F02_Y];
    sensor_msg.y_f02_z      = y_[INDEX_F02_Z];
    sensor_msg.y_f02_roll   = y_[INDEX_F02_Roll];
    sensor_msg.y_f02_yaw    = y_[INDEX_F02_Yaw];
    sensor_msg.y_f02_pitch  = y_[INDEX_F02_Pit];

    // 4. 四个轮对 (WS01 ~ WS04)
    sensor_msg.y_ws01_x     = y_[INDEX_WS01_X];
    sensor_msg.y_ws01_y     = y_[INDEX_WS01_Y];
    sensor_msg.y_ws01_z     = y_[INDEX_WS01_Z];
    sensor_msg.y_ws01_roll  = y_[INDEX_WS01_Roll];
    sensor_msg.y_ws01_yaw   = y_[INDEX_WS01_Yaw];
    sensor_msg.y_ws01_pitch = y_[INDEX_WS01_Pit];

    sensor_msg.y_ws02_x     = y_[INDEX_WS02_X];
    sensor_msg.y_ws02_y     = y_[INDEX_WS02_Y];
    sensor_msg.y_ws02_z     = y_[INDEX_WS02_Z];
    sensor_msg.y_ws02_roll  = y_[INDEX_WS02_Roll];
    sensor_msg.y_ws02_yaw   = y_[INDEX_WS02_Yaw];
    sensor_msg.y_ws02_pitch = y_[INDEX_WS02_Pit];

    sensor_msg.y_ws03_x     = y_[INDEX_WS03_X];
    sensor_msg.y_ws03_y     = y_[INDEX_WS03_Y];
    sensor_msg.y_ws03_z     = y_[INDEX_WS03_Z];
    sensor_msg.y_ws03_roll  = y_[INDEX_WS03_Roll];
    sensor_msg.y_ws03_yaw   = y_[INDEX_WS03_Yaw];
    sensor_msg.y_ws03_pitch = y_[INDEX_WS03_Pit];

    sensor_msg.y_ws04_x     = y_[INDEX_WS04_X];
    sensor_msg.y_ws04_y     = y_[INDEX_WS04_Y];
    sensor_msg.y_ws04_z     = y_[INDEX_WS04_Z];
    sensor_msg.y_ws04_roll  = y_[INDEX_WS04_Roll];
    sensor_msg.y_ws04_yaw   = y_[INDEX_WS04_Yaw];
    sensor_msg.y_ws04_pitch = y_[INDEX_WS04_Pit];

    // 5. 车轮转动角度 (RotA)
    sensor_msg.y_w01_rota   = y_[INDEX_W01_RotA];
    sensor_msg.y_w02_rota   = y_[INDEX_W02_RotA];
    sensor_msg.y_w03_rota   = y_[INDEX_W03_RotA];
    sensor_msg.y_w04_rota   = y_[INDEX_W04_RotA];
    sensor_msg.y_w05_rota   = y_[INDEX_W05_RotA];
    sensor_msg.y_w06_rota   = y_[INDEX_W06_RotA];
    sensor_msg.y_w07_rota   = y_[INDEX_W07_RotA];
    sensor_msg.y_w08_rota   = y_[INDEX_W08_RotA];

    // 6. 八个杆件 (pitch)
    sensor_msg.y_bar01_pitch = y_[INDEX_Bar01_Pit];
    sensor_msg.y_bar02_pitch = y_[INDEX_Bar02_Pit];
    sensor_msg.y_bar03_pitch = y_[INDEX_Bar03_Pit];
    sensor_msg.y_bar04_pitch = y_[INDEX_Bar04_Pit];
    sensor_msg.y_bar05_pitch = y_[INDEX_Bar05_Pit];
    sensor_msg.y_bar06_pitch = y_[INDEX_Bar06_Pit];
    sensor_msg.y_bar07_pitch = y_[INDEX_Bar07_Pit];
    sensor_msg.y_bar08_pitch = y_[INDEX_Bar08_Pit];

    // 7. 各个轮对的 横向位移速度 Vy, 摇头角速度 Vyaw
    sensor_msg.y_ws01_vy   = y_[INDEX_WS01_Vy];
    sensor_msg.y_ws02_vy   = y_[INDEX_WS02_Vy];
    sensor_msg.y_ws03_vy   = y_[INDEX_WS03_Vy];
    sensor_msg.y_ws04_vy   = y_[INDEX_WS04_Vy];

    sensor_msg.y_ws01_vyaw = y_[INDEX_WS01_Vyaw];
    sensor_msg.y_ws02_vyaw = y_[INDEX_WS02_Vyaw];
    sensor_msg.y_ws03_vyaw = y_[INDEX_WS03_Vyaw];
    sensor_msg.y_ws04_vyaw = y_[INDEX_WS04_Vyaw];
  }
  
  else {
    // 如果 ny_ 不足 76，做相应提示或只赋值一部分
    RCLCPP_WARN_ONCE(
      this->get_logger(),
      "ny_=%d < 76, 并非所有的 y[] 都被赋值, 请检查赋值维度",
      ny_);
  }
  
  // 发布
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