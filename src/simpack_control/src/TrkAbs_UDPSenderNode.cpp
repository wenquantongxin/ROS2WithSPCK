/*
    文件名: TrkAbs_UDPSenderNode.cpp

    功能: 与 TrkRel_UDPSenderNode.cpp 类似，不过在发送前先将车辆的
         (y_cb, y_f0x, y_ws0x 等) 轨道相对坐标变换到全局绝对坐标。
         最终发送出去的数据已是“绝对坐标系”下的 (X, Y, Z, roll, yaw, pitch)，
         并进一步从 SIMPACK Rail 的 z向下系转换到惯用的 z 向上系。

    坐标系：
        Z 向上 + 右手系 + 单位 m + (roll,yaw,pitch)按右手坐标系旋转方向
*/

#include <memory>
#include <string>
#include <cstring>       // for memset, strerror
#include <fstream>       // for ifstream
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>      // for close()
#include <errno.h>
#include <vector>
#include <algorithm>     // for std::lower_bound

#include "rclcpp/rclcpp.hpp"
#include "simpack_interfaces/msg/simpack_y.hpp" // 自定义话题

// 使用 nlohmann/json 解析 trajectory_data.json
#include <nlohmann/json.hpp>
using nlohmann::json;

using std::placeholders::_1;

//===================================================================
//  工具函数：将 SIMPACK Rail (z向下) 坐标姿态 => z向上右手坐标系
//===================================================================
/**
 * @brief 将 SIMPACK Rail 坐标系下的 (x, y, z, roll, yaw, pitch)
 *        转换到惯常 z向上坐标系下的 (x, y, z, roll, yaw, pitch)
 *
 * Rail 坐标：z向下为正，x向前，y向右，右手定则
 * 目标坐标：z向上为正，x向前，y向右，仍为右手定则
 * 变化规则： x -> x，y -> y，z -> -z，同时3个转角都取反
 */
inline void convertFromSimpackRailToZUp(
  double &x, double &y, double &z,
  double &roll, double &yaw, double &pitch)
{
  // 位置：z 轴取负即可
  z = -z;

  // 姿态角：全部取负
  roll  = -roll;
  yaw   = -yaw;
  pitch = -pitch;
}

class AbsCoordinateUDPSenderNode : public rclcpp::Node
{
public:
  AbsCoordinateUDPSenderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("trkabs_udpsender_node", options), send_count_(0)
  {
    // ============= 1) 读取轨道数据 =============
    loadTrackData();

    // ============= 2) 设置目标IP、端口 =============
    target_ip_   = "192.168.1.131";  // 可根据实际情况修改
    target_port_ = 10099;           // 可根据实际情况修改

    // ============= 3) 创建UDP socket =============
    udp_socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create UDP socket: %s", strerror(errno));
      rclcpp::shutdown();
      return;
    }

    // 设置目标地址
    memset(&remote_addr_, 0, sizeof(remote_addr_));
    remote_addr_.sin_family = AF_INET;
    remote_addr_.sin_port   = htons(target_port_);
    if (::inet_pton(AF_INET, target_ip_.c_str(), &remote_addr_.sin_addr) <= 0) {
      RCLCPP_ERROR(this->get_logger(), "Invalid target IP address: %s", target_ip_.c_str());
      ::close(udp_socket_fd_);
      rclcpp::shutdown();
      return;
    }

    // ============= 4) 创建订阅者，订阅 /simpack/y =============
    subscription_ = this->create_subscription<simpack_interfaces::msg::SimpackY>(
      "/simpack/y",
      10,
      std::bind(&AbsCoordinateUDPSenderNode::topic_callback, this, _1)
    );

    RCLCPP_INFO(this->get_logger(),
                "trkabs_udpsender_node started. Send to %s:%d",
                target_ip_.c_str(), target_port_);
  }

  ~AbsCoordinateUDPSenderNode()
  {
    if (udp_socket_fd_ >= 0) {
      ::close(udp_socket_fd_);
    }
  }

private:

  // ------------------ 载入轨道数据 ------------------
  void loadTrackData()
  {
    // 轨道文件路径请根据实际情况修改
    std::string json_path = "/home/yaoyao/Documents/myProjects/ROS2WithSPCK/SPCK_Track/trajectory_data.json";

    std::ifstream f(json_path);
    if(!f.is_open()){
      RCLCPP_ERROR(this->get_logger(), "Failed to open trajectory_data.json: %s", json_path.c_str());
      throw std::runtime_error("Cannot open trajectory_data.json");
    }

    json jdata = json::parse(f);

    // 读取数组
    s_vals_   = jdata["s"].get<std::vector<double>>();
    xvals_    = jdata["x"].get<std::vector<double>>();
    yvals_    = jdata["y"].get<std::vector<double>>();
    zvals_    = jdata["z"].get<std::vector<double>>();
    psi_vals_ = jdata["psi"].get<std::vector<double>>();
    slope_vals_ = jdata["slope"].get<std::vector<double>>(); // 如果json可能没有 slope 字段，则做一个contains("slope") 判断。
    
    // phi_vals 可能没有，也可能有
    if(jdata.contains("phi")) {
      phi_vals_ = jdata["phi"].get<std::vector<double>>();
    } else {
      phi_vals_.resize(s_vals_.size(), 0.0);
    }

    RCLCPP_INFO(this->get_logger(), "Track data loaded. Size = %zu", s_vals_.size());
  }

  // ------------------ 工具函数：Z-Y-X欧拉角转3×3矩阵 ------------------
  void euler_zyx_to_matrix(double yaw, double pitch, double roll, double R[3][3]) const
  {
    // Rz(yaw) * Ry(pitch) * Rx(roll)
    double cy = std::cos(yaw);
    double sy = std::sin(yaw);
    double cp = std::cos(pitch);
    double sp = std::sin(pitch);
    double cr = std::cos(roll);
    double sr = std::sin(roll);

    // 这里直接写出乘法结果 (行或列主序皆可, 小心下标):
    R[0][0] = cy*cp;
    R[0][1] = cy*sp*sr - sy*cr;
    R[0][2] = cy*sp*cr + sy*sr;

    R[1][0] = sy*cp;
    R[1][1] = sy*sp*sr + cy*cr;
    R[1][2] = sy*sp*cr - cy*sr;

    R[2][0] = -sp;
    R[2][1] = cp*sr;
    R[2][2] = cp*cr;
  }

  // ------------------ 生成4×4齐次变换矩阵 ------------------
  void make_transform(double yaw, double pitch, double roll,
                      double px, double py, double pz,
                      double T[4][4]) const
  {
    double R[3][3];
    euler_zyx_to_matrix(yaw, pitch, roll, R);

    // 填充 T
    T[0][0] = R[0][0]; T[0][1] = R[0][1]; T[0][2] = R[0][2]; T[0][3] = px;
    T[1][0] = R[1][0]; T[1][1] = R[1][1]; T[1][2] = R[1][2]; T[1][3] = py;
    T[2][0] = R[2][0]; T[2][1] = R[2][1]; T[2][2] = R[2][2]; T[2][3] = pz;
    T[3][0] = 0.0;     T[3][1] = 0.0;     T[3][2] = 0.0;     T[3][3] = 1.0;
  }

  // ------------------ 4×4矩阵相乘: out = A * B ------------------
  void mul_transform(const double A[4][4], const double B[4][4], double out[4][4]) const
  {
    for(int i=0; i<4; i++){
      for(int j=0; j<4; j++){
        out[i][j] = 0.0;
        for(int k=0; k<4; k++){
          out[i][j] += A[i][k] * B[k][j];
        }
      }
    }
  }

  // ------------------ 从4×4变换矩阵中获取 (X, Y, Z) 平移 ------------------
  void get_translation(const double T[4][4], double &x, double &y, double &z) const
  {
    x = T[0][3];
    y = T[1][3];
    z = T[2][3];
  }

  // ------------------ 从4×4变换矩阵中提取 (roll, yaw, pitch) (Z-Y-X顺序) ------------------
  void extract_euler_zyx_from_matrix(const double T[4][4],
                                     double &roll, double &yaw, double &pitch) const
  {
    /*
      由于 msg 中姿态角顺序为 (roll, yaw, pitch),
      并且做 Z-Y-X (yaw-pitch-roll) 逆解时的常见公式：

         pitch = -asin(r20)
         yaw   = atan2(r10, r00)
         roll  = atan2(r21, r22)

      这里把 roll 写在第一个引用参数、yaw 写第二、pitch 第三，
      与 .msg 里的 (roll, yaw, pitch) 保持对应。
    */
    double r00 = T[0][0];
    double r10 = T[1][0];
    double r20 = T[2][0];
    double r21 = T[2][1];
    double r22 = T[2][2];

    double tmpPitch = std::atan2(-r20, std::sqrt(r00*r00 + r10*r10));
    double tmpYaw   = std::atan2(r10, r00);
    double tmpRoll  = std::atan2(r21, r22);

    // 按 (roll, yaw, pitch) 顺序返回
    roll  = tmpRoll;
    yaw   = tmpYaw;
    pitch = tmpPitch;
  }

  // ------------------ 获取最近轨道索引: 线性或二分搜索 ------------------
  int findNearestIndex(double s_query) const
  {
    if(s_query <= s_vals_.front()) return 0;
    if(s_query >= s_vals_.back())  return (int)(s_vals_.size() - 1);

    // 二分搜索
    auto it = std::lower_bound(s_vals_.begin(), s_vals_.end(), s_query);
    int idx = (int)std::distance(s_vals_.begin(), it);

    if(idx == 0) return 0;
    if(idx >= (int)s_vals_.size()) return (int)s_vals_.size()-1;

    // 比较 idx 和 idx-1 哪个更近
    double diff1 = std::fabs(s_vals_[idx] - s_query);
    double diff2 = std::fabs(s_vals_[idx-1] - s_query);
    return (diff2 < diff1) ? (idx-1) : idx;
  }

  // ------------------ 里程 + 局部姿态 => 全局绝对坐标姿态 (Rail坐标下) ------------------
  void getGlobalPose(double s_val,
                     double y_local, double z_local,
                     double roll_local, double yaw_local, double pitch_local,
                     double &X_abs, double &Y_abs, double &Z_abs,
                     double &roll_abs, double &yaw_abs, double &pitch_abs) const
  {
    // 1) 找到最近轨道断面
    int idx = findNearestIndex(s_val);

    double X_T = xvals_[idx];
    double Y_T = yvals_[idx];
    double Z_T = zvals_[idx];

    double yaw_T   = psi_vals_[idx];
    
    // double pitch_T = 0.0;  // 删除
    // 改为:
    double slopeVal = slope_vals_[idx];
    // pitch_T 可用小角度近似 pitch_T = atan(slopeVal)
    double pitch_T = -std::atan(slopeVal); // 需要做 pitch_T = -std::atan(slopeVal)

    double roll_T  = phi_vals_[idx];

    // 2) 轨道系->全局系
    double T_T2G[4][4];
    make_transform(yaw_T, pitch_T, roll_T, X_T, Y_T, Z_T, T_T2G);

    // 3) 部件局部->轨道系 (注意 x_local=0, y_local, z_local 视Simpack定义)
    double T_W2T[4][4];
    make_transform(yaw_local, pitch_local, roll_local, 0.0, y_local, z_local, T_W2T);

    // 4) T_W2G = T_T2G * T_W2T
    double T_W2G[4][4];
    mul_transform(T_T2G, T_W2T, T_W2G);

    // 5) 提取绝对 (X_abs, Y_abs, Z_abs) & (roll_abs, yaw_abs, pitch_abs)
    get_translation(T_W2G, X_abs, Y_abs, Z_abs);
    // 注意这里参数顺序：roll, yaw, pitch
    extract_euler_zyx_from_matrix(T_W2G, roll_abs, yaw_abs, pitch_abs);
  }

private:
  // 订阅回调函数：接收 /simpack/y 消息，进行坐标变换并通过UDP发送
  void topic_callback(const simpack_interfaces::msg::SimpackY::SharedPtr msg)
  {
    // ========== 先取出一些直接使用的量 ==========
    double sim_time   = msg->sim_time;   // 仿真时间
    double y_spcktime = msg->y_spcktime; // SIMPACK内部时间
    double y_cb_vx    = msg->y_cb_vx;    // 车辆纵向运行速度(不改符号)

    // =====================================================================
    // STEP 1: 先用 getGlobalPose() 得到 Rail 系下的“绝对”坐标与姿态（z向下）
    // =====================================================================

    // ========== (1) 车体 ===========
    double cbX_abs, cbY_abs, cbZ_abs;
    double cbRoll_abs, cbYaw_abs, cbPitch_abs;
    getGlobalPose(msg->y_cb_x, msg->y_cb_y, msg->y_cb_z,
                  msg->y_cb_roll, msg->y_cb_yaw, msg->y_cb_pitch,
                  cbX_abs, cbY_abs, cbZ_abs,
                  cbRoll_abs, cbYaw_abs, cbPitch_abs);

    // ========== (2) 前转向架 #1 ==========
    double f01X_abs, f01Y_abs, f01Z_abs;
    double f01Roll_abs, f01Yaw_abs, f01Pitch_abs;
    getGlobalPose(msg->y_f01_x, msg->y_f01_y, msg->y_f01_z,
                  msg->y_f01_roll, msg->y_f01_yaw, msg->y_f01_pitch,
                  f01X_abs, f01Y_abs, f01Z_abs,
                  f01Roll_abs, f01Yaw_abs, f01Pitch_abs);

    // ========== (3) 后转向架 #2 ==========
    double f02X_abs, f02Y_abs, f02Z_abs;
    double f02Roll_abs, f02Yaw_abs, f02Pitch_abs;
    getGlobalPose(msg->y_f02_x, msg->y_f02_y, msg->y_f02_z,
                  msg->y_f02_roll, msg->y_f02_yaw, msg->y_f02_pitch,
                  f02X_abs, f02Y_abs, f02Z_abs,
                  f02Roll_abs, f02Yaw_abs, f02Pitch_abs);

    // ========== (4) 4 个轮对 ws01..ws04 ==========
    auto transform_ws = [this](double sx, double sy, double sz,
                               double sroll, double syaw, double spitch){
      double gx, gy, gz;
      double gr, gyw, gp;
      getGlobalPose(sx, sy, sz, sroll, syaw, spitch, gx, gy, gz, gr, gyw, gp);
      return std::array<double,6>{ gx, gy, gz, gr, gyw, gp };
    };

    auto ws01_abs = transform_ws(msg->y_ws01_x, msg->y_ws01_y, msg->y_ws01_z,
                                 msg->y_ws01_roll, msg->y_ws01_yaw, msg->y_ws01_pitch);

    auto ws02_abs = transform_ws(msg->y_ws02_x, msg->y_ws02_y, msg->y_ws02_z,
                                 msg->y_ws02_roll, msg->y_ws02_yaw, msg->y_ws02_pitch);

    auto ws03_abs = transform_ws(msg->y_ws03_x, msg->y_ws03_y, msg->y_ws03_z,
                                 msg->y_ws03_roll, msg->y_ws03_yaw, msg->y_ws03_pitch);

    auto ws04_abs = transform_ws(msg->y_ws04_x, msg->y_ws04_y, msg->y_ws04_z,
                                 msg->y_ws04_roll, msg->y_ws04_yaw, msg->y_ws04_pitch);

    // =====================================================================
    // STEP 2: 再把这些绝对位姿从 “Rail(z向下)系” 转换为 “z向上右手系”
    // =====================================================================
    convertFromSimpackRailToZUp(cbX_abs, cbY_abs, cbZ_abs,
                                cbRoll_abs, cbYaw_abs, cbPitch_abs);

    convertFromSimpackRailToZUp(f01X_abs, f01Y_abs, f01Z_abs,
                                f01Roll_abs, f01Yaw_abs, f01Pitch_abs);

    convertFromSimpackRailToZUp(f02X_abs, f02Y_abs, f02Z_abs,
                                f02Roll_abs, f02Yaw_abs, f02Pitch_abs);

    auto convert_ws = [&](std::array<double,6> &ws) {
      convertFromSimpackRailToZUp(ws[0], ws[1], ws[2],
                                  ws[3], ws[4], ws[5]);
    };
    auto ws01_zup = ws01_abs;
    auto ws02_zup = ws02_abs;
    auto ws03_zup = ws03_abs;
    auto ws04_zup = ws04_abs;
    convert_ws(ws01_zup);
    convert_ws(ws02_zup);
    convert_ws(ws03_zup);
    convert_ws(ws04_zup);

    // =====================================================================
    // STEP 3: 将需要发送的 92 = 1(时间) + 76(原DoFs数据) + 14(转矩/平稳性/安全性指标) + 1(里程) 量打包 (已是 z轴 向上, 且 roll,yaw,pitch 对应顺序)
    // 下列 payload.push_back 注释是从 0 开始计数的，并且正好对应于从1计数的 SIMPACK_Y 索引
    // =====================================================================
    std::vector<double> payload;
    payload.reserve(92);

    // (0, payload从0计数) sim_time
    payload.push_back(sim_time);
    // (1) y_spcktime
    payload.push_back(y_spcktime);
    // (2) y_cb_vx 车辆纵向速度
    payload.push_back(y_cb_vx);

    // (3..8) 车体 (x, y, z, roll, yaw, pitch)
    payload.push_back(cbX_abs);
    payload.push_back(cbY_abs);
    payload.push_back(cbZ_abs);
    payload.push_back(cbRoll_abs);
    payload.push_back(cbYaw_abs);
    payload.push_back(cbPitch_abs);

    // (9..16) 8个车轮旋转速度 rotw
    payload.push_back(msg->y_w01_rotw);
    payload.push_back(msg->y_w02_rotw);
    payload.push_back(msg->y_w03_rotw);
    payload.push_back(msg->y_w04_rotw);
    payload.push_back(msg->y_w05_rotw);
    payload.push_back(msg->y_w06_rotw);
    payload.push_back(msg->y_w07_rotw);
    payload.push_back(msg->y_w08_rotw);

    // (17..22) 前转向架 (x, y, z, roll, yaw, pitch)
    payload.push_back(f01X_abs);
    payload.push_back(f01Y_abs);
    payload.push_back(f01Z_abs);
    payload.push_back(f01Roll_abs);
    payload.push_back(f01Yaw_abs);
    payload.push_back(f01Pitch_abs);

    // (23..28) 后转向架 (x, y, z, roll, yaw, pitch)
    payload.push_back(f02X_abs);
    payload.push_back(f02Y_abs);
    payload.push_back(f02Z_abs);
    payload.push_back(f02Roll_abs);
    payload.push_back(f02Yaw_abs);
    payload.push_back(f02Pitch_abs);

    // (29..52) 4个轮对 (同理: x,y,z, roll,yaw,pitch)
    // ws01 (29..34)
    payload.push_back(ws01_zup[0]);
    payload.push_back(ws01_zup[1]);
    payload.push_back(ws01_zup[2]);
    payload.push_back(ws01_zup[3]);
    payload.push_back(ws01_zup[4]);
    payload.push_back(ws01_zup[5]);
    // ws02 (35..40)
    payload.push_back(ws02_zup[0]);
    payload.push_back(ws02_zup[1]);
    payload.push_back(ws02_zup[2]);
    payload.push_back(ws02_zup[3]);
    payload.push_back(ws02_zup[4]);
    payload.push_back(ws02_zup[5]);
    // ws03 (41..46)
    payload.push_back(ws03_zup[0]);
    payload.push_back(ws03_zup[1]);
    payload.push_back(ws03_zup[2]);
    payload.push_back(ws03_zup[3]);
    payload.push_back(ws03_zup[4]);
    payload.push_back(ws03_zup[5]);
    // ws04 (47..52)
    payload.push_back(ws04_zup[0]);
    payload.push_back(ws04_zup[1]);
    payload.push_back(ws04_zup[2]);
    payload.push_back(ws04_zup[3]);
    payload.push_back(ws04_zup[4]);
    payload.push_back(ws04_zup[5]);

    // (53..60) 8车轮转角 rota
    payload.push_back(msg->y_w01_rota);
    payload.push_back(msg->y_w02_rota);
    payload.push_back(msg->y_w03_rota);
    payload.push_back(msg->y_w04_rota);
    payload.push_back(msg->y_w05_rota);
    payload.push_back(msg->y_w06_rota);
    payload.push_back(msg->y_w07_rota);
    payload.push_back(msg->y_w08_rota);

    // (61..68) 8根连杆 pitch
    payload.push_back(msg->y_bar01_pitch);
    payload.push_back(msg->y_bar02_pitch);
    payload.push_back(msg->y_bar03_pitch);
    payload.push_back(msg->y_bar04_pitch);
    payload.push_back(msg->y_bar05_pitch);
    payload.push_back(msg->y_bar06_pitch);
    payload.push_back(msg->y_bar07_pitch);
    payload.push_back(msg->y_bar08_pitch);

    // (69..72) 4个轮对 vy
    payload.push_back(msg->y_ws01_vy);
    payload.push_back(msg->y_ws02_vy);
    payload.push_back(msg->y_ws03_vy);
    payload.push_back(msg->y_ws04_vy);

    // (73..76) 4个轮对 vyaw
    payload.push_back(msg->y_ws01_vyaw);
    payload.push_back(msg->y_ws02_vyaw);
    payload.push_back(msg->y_ws03_vyaw);
    payload.push_back(msg->y_ws04_vyaw);

    // (77..78) 舒适性测点加速度
    payload.push_back(msg->y_comfort_accy);
    payload.push_back(msg->y_comfort_accz);
 
    // (79..82) 一位端轮对左右车轮的轮轨力
    payload.push_back(msg->y_w01_contact_fy);
    payload.push_back(msg->y_w01_contact_fz);
    payload.push_back(msg->y_w02_contact_fy);
    payload.push_back(msg->y_w02_contact_fz);

    // (83..90) 8个车轮的输入力矩
    payload.push_back(msg->y_w01_torque);
    payload.push_back(msg->y_w02_torque);
    payload.push_back(msg->y_w03_torque);
    payload.push_back(msg->y_w04_torque);
    payload.push_back(msg->y_w05_torque);
    payload.push_back(msg->y_w06_torque);
    payload.push_back(msg->y_w07_torque);
    payload.push_back(msg->y_w08_torque);

    // (91) 车辆运行里程
    payload.push_back(msg->y_tracks);

    // ============= 转为字节指针，UDP发送 =============
    const char* raw_ptr    = reinterpret_cast<const char*>(payload.data());
    size_t total_bytes     = payload.size() * sizeof(double);

    ssize_t bytes_sent = ::sendto(
      udp_socket_fd_,
      raw_ptr,
      total_bytes,
      0,
      reinterpret_cast<struct sockaddr*>(&remote_addr_),
      sizeof(remote_addr_)
    );

    if (bytes_sent < 0) {
      RCLCPP_ERROR(this->get_logger(), "UDP sendto failed: %s", strerror(errno));
      return;
    }

    // 打印日志 (可选，便于观察发送节奏)
    send_count_++;
    if (send_count_ % 2000 == 0) {
      RCLCPP_INFO(this->get_logger(), 
                  "Sent %ld bytes (count=%lu) [abs-coord in z-up] to %s:%d", 
                  bytes_sent, send_count_,
                  target_ip_.c_str(), target_port_);
    }
  }

private:
  // =========== UDP相关 ==============
  int udp_socket_fd_;
  std::string target_ip_;
  int target_port_;
  struct sockaddr_in remote_addr_;
  uint64_t send_count_;

  // =========== 轨道数据缓存 ===========
  std::vector<double> s_vals_;
  std::vector<double> xvals_;
  std::vector<double> yvals_;
  std::vector<double> zvals_;
  std::vector<double> psi_vals_;
  std::vector<double> phi_vals_; // 如果没有则默认0
  std::vector<double> slope_vals_; // 添加此行声明坡度数据向量

  rclcpp::Subscription<simpack_interfaces::msg::SimpackY>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AbsCoordinateUDPSenderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
