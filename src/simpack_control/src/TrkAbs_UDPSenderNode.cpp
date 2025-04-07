/*
    文件名: TrkAbs_UDPSenderNode.cpp

    功能:
     - 订阅 /simpack/y 进行“绝对坐标”转换后，通过 UDP 发送
     - 订阅 /simpack/w 获取在线计算的 Sperling 及脱轨系数
     - 在原 payload (91 项) 之后，额外附加 4 项 /simpack/w 数据:
       sperling_y, sperling_z, derailment_w01, derailment_w02
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

// 订阅的消息
#include "simpack_interfaces/msg/simpack_y.hpp" 
#include "simpack_interfaces/msg/simpack_w.hpp" 

// 使用 nlohmann/json 解析轨道文件
#include <nlohmann/json.hpp>
using nlohmann::json;

using std::placeholders::_1;

//===================================================================
//  工具函数：将 SIMPACK Rail (z向下) 坐标姿态 => z向上右手坐标系
//===================================================================
/**
 * @brief 将 SIMPACK Rail 坐标系下的 (x, y, z, roll, yaw, pitch)
 *        转换到惯常 z向上坐标系 (x, y, z, roll, yaw, pitch)
 *
 * Rail坐标: z 向下为正; 目标坐标: z 向上为正; 同为右手系
 * 转换: z -> -z, roll/yaw/pitch 统统取负
 */
inline void convertFromSimpackRailToZUp(
  double &x, double &y, double &z,
  double &roll, double &yaw, double &pitch)
{
  z = -z;
  roll  = -roll;
  yaw   = -yaw;
  pitch = -pitch;
}

class AbsCoordinateUDPSenderNode : public rclcpp::Node
{
public:
  AbsCoordinateUDPSenderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("trkabs_udpsender_node", options),
    udp_socket_fd_(-1),
    send_count_(0),
    last_sperling_y_(0.0),
    last_sperling_z_(0.0),
    last_derailment_w01_(0.0),
    last_derailment_w02_(0.0),
    has_w_data_(false)
  {
    // ============= 1) 读取轨道数据 =============
    loadTrackData();

    // ============= 2) 设置目标IP、端口 =============
    target_ip_   = "192.168.1.131";  // 示例: 自行修改
    target_port_ = 10099;           // 示例: 自行修改

    // ============= 3) 创建UDP socket =============
    udp_socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create UDP socket: %s", strerror(errno));
      rclcpp::shutdown();
      return;
    }
    memset(&remote_addr_, 0, sizeof(remote_addr_));
    remote_addr_.sin_family = AF_INET;
    remote_addr_.sin_port   = htons(target_port_);
    if (::inet_pton(AF_INET, target_ip_.c_str(), &remote_addr_.sin_addr) <= 0) {
      RCLCPP_ERROR(this->get_logger(), "Invalid target IP address: %s", target_ip_.c_str());
      ::close(udp_socket_fd_);
      rclcpp::shutdown();
      return;
    }

    // ============= 4-1) 订阅 /simpack/y =============
    subscription_y_ = this->create_subscription<simpack_interfaces::msg::SimpackY>(
      "/simpack/y",
      10,
      std::bind(&AbsCoordinateUDPSenderNode::topic_callback_y, this, _1));

    // ============= 4-2) 订阅 /simpack/w =============
    subscription_w_ = this->create_subscription<simpack_interfaces::msg::SimpackW>(
      "/simpack/w",
      10,
      std::bind(&AbsCoordinateUDPSenderNode::topic_callback_w, this, _1));

    RCLCPP_INFO(this->get_logger(),
                "trkabs_udpsender_node started. UDP -> %s:%d",
                target_ip_.c_str(), target_port_);
  }

  ~AbsCoordinateUDPSenderNode()
  {
    if (udp_socket_fd_ >= 0) {
      ::close(udp_socket_fd_);
    }
    RCLCPP_INFO(this->get_logger(), "AbsCoordinateUDPSenderNode destroyed.");
  }

private:
  // ------------------ 载入轨道数据 ------------------
  void loadTrackData()
  {
    std::string json_path = "/home/yaoyao/Documents/myProjects/ROS2WithSPCK/SPCK_Track/trajectory_data.json";

    std::ifstream f(json_path);
    if(!f.is_open()){
      RCLCPP_ERROR(this->get_logger(), "Failed to open trajectory_data.json: %s", json_path.c_str());
      throw std::runtime_error("Cannot open trajectory_data.json");
    }

    json jdata = json::parse(f);

    s_vals_   = jdata["s"].get<std::vector<double>>();
    xvals_    = jdata["x"].get<std::vector<double>>();
    yvals_    = jdata["y"].get<std::vector<double>>();
    zvals_    = jdata["z"].get<std::vector<double>>();
    psi_vals_ = jdata["psi"].get<std::vector<double>>();
    // slope
    if(jdata.contains("slope")) {
      slope_vals_ = jdata["slope"].get<std::vector<double>>();
    } else {
      slope_vals_.resize(s_vals_.size(), 0.0);
    }
    // phi
    if(jdata.contains("phi")) {
      phi_vals_ = jdata["phi"].get<std::vector<double>>();
    } else {
      phi_vals_.resize(s_vals_.size(), 0.0);
    }

    RCLCPP_INFO(this->get_logger(), "Track data loaded. Size=%zu", s_vals_.size());
  }

  // ------------------ 从 /simpack/w 读Sperling & derailment ------------------
  void topic_callback_w(const simpack_interfaces::msg::SimpackW::SharedPtr wmsg)
  {
    // 缓存最新的 4 个指标
    last_sperling_y_    = wmsg->sperling_y;
    last_sperling_z_    = wmsg->sperling_z;
    last_derailment_w01_ = wmsg->derailment_w01;
    last_derailment_w02_ = wmsg->derailment_w02;
    has_w_data_         = true;
  }

  // ------------------ 主回调: /simpack/y => 坐标变换 & UDP发送 ------------------
  void topic_callback_y(const simpack_interfaces::msg::SimpackY::SharedPtr msg)
  {
    // 1) 取一些常用量
    double sim_time   = msg->sim_time;
    double y_spcktime = msg->y_spcktime;
    double y_cb_vx    = msg->y_cb_vx;

    // 2) 分别计算车体/转向架/轮对的“绝对(rail下)坐标”
    double cbX_abs, cbY_abs, cbZ_abs;
    double cbRoll_abs, cbYaw_abs, cbPitch_abs;
    getGlobalPose(msg->y_cb_x, msg->y_cb_y, msg->y_cb_z,
                  msg->y_cb_roll, msg->y_cb_yaw, msg->y_cb_pitch,
                  cbX_abs, cbY_abs, cbZ_abs,
                  cbRoll_abs, cbYaw_abs, cbPitch_abs);

    double f01X_abs, f01Y_abs, f01Z_abs;
    double f01Roll_abs, f01Yaw_abs, f01Pitch_abs;
    getGlobalPose(msg->y_f01_x, msg->y_f01_y, msg->y_f01_z,
                  msg->y_f01_roll, msg->y_f01_yaw, msg->y_f01_pitch,
                  f01X_abs, f01Y_abs, f01Z_abs,
                  f01Roll_abs, f01Yaw_abs, f01Pitch_abs);

    double f02X_abs, f02Y_abs, f02Z_abs;
    double f02Roll_abs, f02Yaw_abs, f02Pitch_abs;
    getGlobalPose(msg->y_f02_x, msg->y_f02_y, msg->y_f02_z,
                  msg->y_f02_roll, msg->y_f02_yaw, msg->y_f02_pitch,
                  f02X_abs, f02Y_abs, f02Z_abs,
                  f02Roll_abs, f02Yaw_abs, f02Pitch_abs);

    auto transform_ws = [this](double sx, double sy, double sz,
                               double sroll, double syaw, double spitch)
    {
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

    // 3) 转换到 z向上坐标
    convertFromSimpackRailToZUp(cbX_abs, cbY_abs, cbZ_abs, cbRoll_abs, cbYaw_abs, cbPitch_abs);
    convertFromSimpackRailToZUp(f01X_abs, f01Y_abs, f01Z_abs, f01Roll_abs, f01Yaw_abs, f01Pitch_abs);
    convertFromSimpackRailToZUp(f02X_abs, f02Y_abs, f02Z_abs, f02Roll_abs, f02Yaw_abs, f02Pitch_abs);

    auto conv_ws = [&](std::array<double,6> &ws){
      convertFromSimpackRailToZUp(ws[0], ws[1], ws[2], ws[3], ws[4], ws[5]);
    };
    conv_ws(ws01_abs);
    conv_ws(ws02_abs);
    conv_ws(ws03_abs);
    conv_ws(ws04_abs);

    // 4) 打包 payload (共 92 + 4 = 96 项)
    std::vector<double> payload;
    payload.reserve(96);

    // (0) sim_time
    payload.push_back(sim_time);
    // (1) y_spcktime
    payload.push_back(y_spcktime);
    // (2) y_cb_vx
    payload.push_back(y_cb_vx);

    // (3..8) 车体
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

    // (17..22) 前转向架
    payload.push_back(f01X_abs);
    payload.push_back(f01Y_abs);
    payload.push_back(f01Z_abs);
    payload.push_back(f01Roll_abs);
    payload.push_back(f01Yaw_abs);
    payload.push_back(f01Pitch_abs);

    // (23..28) 后转向架
    payload.push_back(f02X_abs);
    payload.push_back(f02Y_abs);
    payload.push_back(f02Z_abs);
    payload.push_back(f02Roll_abs);
    payload.push_back(f02Yaw_abs);
    payload.push_back(f02Pitch_abs);

    // (29..52) 4个轮对
    auto ws2payload = [&](const std::array<double,6> &ws){
      payload.push_back(ws[0]);
      payload.push_back(ws[1]);
      payload.push_back(ws[2]);
      payload.push_back(ws[3]);
      payload.push_back(ws[4]);
      payload.push_back(ws[5]);
    };
    ws2payload(ws01_abs); // (29..34)
    ws2payload(ws02_abs); // (35..40)
    ws2payload(ws03_abs); // (41..46)
    ws2payload(ws04_abs); // (47..52)

    // (53..60) 8 车轮转角 rota
    payload.push_back(msg->y_w01_rota);
    payload.push_back(msg->y_w02_rota);
    payload.push_back(msg->y_w03_rota);
    payload.push_back(msg->y_w04_rota);
    payload.push_back(msg->y_w05_rota);
    payload.push_back(msg->y_w06_rota);
    payload.push_back(msg->y_w07_rota);
    payload.push_back(msg->y_w08_rota);

    // (61..68) 8 根连杆 pitch
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

    // (77..78) 舒适性加速度
    payload.push_back(msg->y_comfort_accy);
    payload.push_back(msg->y_comfort_accz);

    // (79..82) 轮轨力
    payload.push_back(msg->y_w01_contact_fy);
    payload.push_back(msg->y_w01_contact_fz);
    payload.push_back(msg->y_w02_contact_fy);
    payload.push_back(msg->y_w02_contact_fz);

    // (83..90) 8个车轮力矩
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

    // ------------------ 新增 4 项 /simpack/w 数据 ------------------
    double sperling_y    = (has_w_data_ ? last_sperling_y_    : 0.0);
    double sperling_z    = (has_w_data_ ? last_sperling_z_    : 0.0);
    double derail_w01    = (has_w_data_ ? last_derailment_w01_ : 0.0);
    double derail_w02    = (has_w_data_ ? last_derailment_w02_ : 0.0);

    // (92) sperling_y
    payload.push_back(sperling_y);
    // (93) sperling_z
    payload.push_back(sperling_z);
    // (94) derailment_w01
    payload.push_back(derail_w01);
    // (95) derailment_w02
    payload.push_back(derail_w02);

    // ============= 发送 =============
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

    send_count_++;
    if (send_count_ % 5000 == 0) {
      RCLCPP_INFO(this->get_logger(),
                  "Sent %ld bytes (count=%lu) [abs-coord z-up] to %s:%d",
                  bytes_sent, send_count_,
                  target_ip_.c_str(), target_port_);
    }
  }

  // ------------------ 核心: 里程 + 局部姿态 => Rail绝对姿态 (z向下系) ------------------
  void getGlobalPose(double s_val,
                     double y_local, double z_local,
                     double roll_local, double yaw_local, double pitch_local,
                     double &X_abs, double &Y_abs, double &Z_abs,
                     double &roll_abs, double &yaw_abs, double &pitch_abs) const
  {
    int idx = findNearestIndex(s_val);

    double X_T = xvals_[idx];
    double Y_T = yvals_[idx];
    double Z_T = zvals_[idx];
    double yaw_T   = psi_vals_[idx];
    // slope => pitch
    double slopeVal = slope_vals_[idx];
    double pitch_T  = -std::atan(slopeVal); // rail系pitch
    double roll_T   = phi_vals_[idx];

    // 轨道系->全局系
    double T_T2G[4][4];
    make_transform(yaw_T, pitch_T, roll_T, X_T, Y_T, Z_T, T_T2G);

    // 部件局部->轨道系
    double T_W2T[4][4];
    make_transform(yaw_local, pitch_local, roll_local, 0.0, y_local, z_local, T_W2T);

    double T_W2G[4][4];
    mul_transform(T_T2G, T_W2T, T_W2G);

    get_translation(T_W2G, X_abs, Y_abs, Z_abs);
    extract_euler_zyx_from_matrix(T_W2G, roll_abs, yaw_abs, pitch_abs);
  }

  // ------------------ 轨道检索 ------------------
  int findNearestIndex(double s_query) const
  {
    if(s_query <= s_vals_.front()) return 0;
    if(s_query >= s_vals_.back())  return (int)(s_vals_.size() - 1);

    auto it = std::lower_bound(s_vals_.begin(), s_vals_.end(), s_query);
    int idx = (int)std::distance(s_vals_.begin(), it);
    if(idx == 0) return 0;
    if(idx >= (int)s_vals_.size()) return (int)s_vals_.size()-1;

    double diff1 = std::fabs(s_vals_[idx] - s_query);
    double diff2 = std::fabs(s_vals_[idx-1] - s_query);
    return (diff2 < diff1) ? (idx-1) : idx;
  }

  // ------------------ 欧拉角 => 3×3 旋转矩阵 (Z-Y-X) ------------------
  void euler_zyx_to_matrix(double yaw, double pitch, double roll, double R[3][3]) const
  {
    double cy = std::cos(yaw);
    double sy = std::sin(yaw);
    double cp = std::cos(pitch);
    double sp = std::sin(pitch);
    double cr = std::cos(roll);
    double sr = std::sin(roll);

    // Rz(yaw)*Ry(pitch)*Rx(roll)
    R[0][0] = cy*cp;                R[0][1] = cy*sp*sr - sy*cr; R[0][2] = cy*sp*cr + sy*sr;
    R[1][0] = sy*cp;                R[1][1] = sy*sp*sr + cy*cr; R[1][2] = sy*sp*cr - cy*sr;
    R[2][0] = -sp;                  R[2][1] = cp*sr;            R[2][2] = cp*cr;
  }

  // ------------------ 构造4×4齐次变换矩阵 ------------------
  void make_transform(double yaw, double pitch, double roll,
                      double px, double py, double pz,
                      double T[4][4]) const
  {
    double R[3][3];
    euler_zyx_to_matrix(yaw, pitch, roll, R);

    T[0][0] = R[0][0]; T[0][1] = R[0][1]; T[0][2] = R[0][2]; T[0][3] = px;
    T[1][0] = R[1][0]; T[1][1] = R[1][1]; T[1][2] = R[1][2]; T[1][3] = py;
    T[2][0] = R[2][0]; T[2][1] = R[2][1]; T[2][2] = R[2][2]; T[2][3] = pz;
    T[3][0] = 0.0;     T[3][1] = 0.0;     T[3][2] = 0.0;     T[3][3] = 1.0;
  }

  // ------------------ 4×4矩阵乘法 ------------------
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

  // ------------------ 从 4×4 矩阵取平移部分 (X, Y, Z) ------------------
  void get_translation(const double T[4][4], double &x, double &y, double &z) const
  {
    x = T[0][3];
    y = T[1][3];
    z = T[2][3];
  }

  // ------------------ 从 4×4 矩阵中提取 (roll, yaw, pitch) (Z-Y-X) ------------------
  void extract_euler_zyx_from_matrix(const double T[4][4],
                                     double &roll, double &yaw, double &pitch) const
  {
    // 约定: roll=Rx, pitch=Ry, yaw=Rz => 逆序: Rz^-1 * Ry^-1 * Rx^-1
    double r00 = T[0][0];
    double r10 = T[1][0];
    double r20 = T[2][0];
    double r21 = T[2][1];
    double r22 = T[2][2];

    // pitch = -asin(r20)
    // yaw   = atan2(r10, r00)
    // roll  = atan2(r21, r22)
    double tmpPitch = std::atan2(-r20, std::sqrt(r00*r00 + r10*r10));
    double tmpYaw   = std::atan2(r10, r00);
    double tmpRoll  = std::atan2(r21, r22);

    roll  = tmpRoll;
    yaw   = tmpYaw;
    pitch = tmpPitch;
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
  std::vector<double> phi_vals_;
  std::vector<double> slope_vals_;

  // =========== 订阅者 ===========
  // /simpack/y
  rclcpp::Subscription<simpack_interfaces::msg::SimpackY>::SharedPtr subscription_y_;
  // /simpack/w
  rclcpp::Subscription<simpack_interfaces::msg::SimpackW>::SharedPtr subscription_w_;

  // =========== 缓存 /simpack/w 的最新指标 ===========
  double last_sperling_y_;
  double last_sperling_z_;
  double last_derailment_w01_;
  double last_derailment_w02_;
  bool   has_w_data_; // 是否已订阅到 /simpack/w

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AbsCoordinateUDPSenderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
