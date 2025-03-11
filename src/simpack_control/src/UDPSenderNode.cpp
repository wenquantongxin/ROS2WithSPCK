/*
   文件名: UDPSenderNode.cpp
   文件路径: ...\ROS2WithSPCK\src\simpack_control\src\UDPSenderNode.cpp

*/

#include <memory>
#include <string>
#include <cstring>       // for memset, strerror
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>      // for close()
#include <errno.h>

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "simpack_interfaces/msg/simpack_y.hpp" // 你的自定义消息头文件

using std::placeholders::_1;

class UDPSenderNode : public rclcpp::Node
{
public:
  UDPSenderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("udp_sender_node", options), send_count_(0)
  {
    // 1) 设置目标IP、端口（Windows侧）
    target_ip_   = "192.168.1.131";  // Windows 主机 IP
    target_port_ = 10088;           // Windows 端接收端口

    // 2) 创建UDP socket
    udp_socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create UDP socket: %s", strerror(errno));
      rclcpp::shutdown();
      return;
    }

    // -- 不再 bind() 到本地固定端口，让系统自动分配一个随机端口 --

    // 3) 设置目标地址信息（远端：Windows）
    memset(&remote_addr_, 0, sizeof(remote_addr_));
    remote_addr_.sin_family = AF_INET;
    remote_addr_.sin_port   = htons(target_port_);
    if (::inet_pton(AF_INET, target_ip_.c_str(), &remote_addr_.sin_addr) <= 0) {
      RCLCPP_ERROR(this->get_logger(), "Invalid target IP address: %s", target_ip_.c_str());
      ::close(udp_socket_fd_);
      rclcpp::shutdown();
      return;
    }

    // 4) 创建订阅者 - 订阅 /simpack/y
    subscription_ = this->create_subscription<simpack_interfaces::msg::SimpackY>(
      "/simpack/y",
      10,
      std::bind(&UDPSenderNode::topic_callback, this, _1)
    );

    RCLCPP_INFO(this->get_logger(),
                "UDP Sender Node (Binary). Target = %s:%d",
                target_ip_.c_str(), target_port_);
  }

  ~UDPSenderNode()
  {
    if (udp_socket_fd_ >= 0) {
      ::close(udp_socket_fd_);
    }
  }

private:
  void topic_callback(const simpack_interfaces::msg::SimpackY::SharedPtr msg)
  {
    // 1) 将 77 个 double 连续地填入一个容器
    std::vector<double> payload;
    payload.reserve(77);

    // 依照一定顺序放入数据; 注意需与 Python 解析端保持一致
    payload.push_back(msg->sim_time);      // (1)
    payload.push_back(msg->y_spcktime);    // (2)

    payload.push_back(msg->y_cb_vx);       // (3)

    payload.push_back(msg->y_cb_x);        // (4)
    payload.push_back(msg->y_cb_y);        // (5)
    payload.push_back(msg->y_cb_z);        // (6)
    payload.push_back(msg->y_cb_roll);     // (7)
    payload.push_back(msg->y_cb_yaw);      // (8)
    payload.push_back(msg->y_cb_pitch);    // (9)

    payload.push_back(msg->y_w01_rotw);    // (10)
    payload.push_back(msg->y_w02_rotw);    // (11)
    payload.push_back(msg->y_w03_rotw);    // (12)
    payload.push_back(msg->y_w04_rotw);    // (13)
    payload.push_back(msg->y_w05_rotw);    // (14)
    payload.push_back(msg->y_w06_rotw);    // (15)
    payload.push_back(msg->y_w07_rotw);    // (16)
    payload.push_back(msg->y_w08_rotw);    // (17)

    payload.push_back(msg->y_f01_x);       // (18)
    payload.push_back(msg->y_f01_y);       // (19)
    payload.push_back(msg->y_f01_z);       // (20)
    payload.push_back(msg->y_f01_roll);    // (21)
    payload.push_back(msg->y_f01_yaw);     // (22)
    payload.push_back(msg->y_f01_pitch);   // (23)

    payload.push_back(msg->y_f02_x);       // (24)
    payload.push_back(msg->y_f02_y);       // (25)
    payload.push_back(msg->y_f02_z);       // (26)
    payload.push_back(msg->y_f02_roll);    // (27)
    payload.push_back(msg->y_f02_yaw);     // (28)
    payload.push_back(msg->y_f02_pitch);   // (29)

    payload.push_back(msg->y_ws01_x);      // (30)
    payload.push_back(msg->y_ws01_y);      // (31)
    payload.push_back(msg->y_ws01_z);      // (32)
    payload.push_back(msg->y_ws01_roll);   // (33)
    payload.push_back(msg->y_ws01_yaw);    // (34)
    payload.push_back(msg->y_ws01_pitch);  // (35)

    payload.push_back(msg->y_ws02_x);      // (36)
    payload.push_back(msg->y_ws02_y);      // (37)
    payload.push_back(msg->y_ws02_z);      // (38)
    payload.push_back(msg->y_ws02_roll);   // (39)
    payload.push_back(msg->y_ws02_yaw);    // (40)
    payload.push_back(msg->y_ws02_pitch);  // (41)

    payload.push_back(msg->y_ws03_x);      // (42)
    payload.push_back(msg->y_ws03_y);      // (43)
    payload.push_back(msg->y_ws03_z);      // (44)
    payload.push_back(msg->y_ws03_roll);   // (45)
    payload.push_back(msg->y_ws03_yaw);    // (46)
    payload.push_back(msg->y_ws03_pitch);  // (47)

    payload.push_back(msg->y_ws04_x);      // (48)
    payload.push_back(msg->y_ws04_y);      // (49)
    payload.push_back(msg->y_ws04_z);      // (50)
    payload.push_back(msg->y_ws04_roll);   // (51)
    payload.push_back(msg->y_ws04_yaw);    // (52)
    payload.push_back(msg->y_ws04_pitch);  // (53)

    payload.push_back(msg->y_w01_rota);    // (54)
    payload.push_back(msg->y_w02_rota);    // (55)
    payload.push_back(msg->y_w03_rota);    // (56)
    payload.push_back(msg->y_w04_rota);    // (57)
    payload.push_back(msg->y_w05_rota);    // (58)
    payload.push_back(msg->y_w06_rota);    // (59)
    payload.push_back(msg->y_w07_rota);    // (60)
    payload.push_back(msg->y_w08_rota);    // (61)

    payload.push_back(msg->y_bar01_pitch); // (62)
    payload.push_back(msg->y_bar02_pitch); // (63)
    payload.push_back(msg->y_bar03_pitch); // (64)
    payload.push_back(msg->y_bar04_pitch); // (65)
    payload.push_back(msg->y_bar05_pitch); // (66)
    payload.push_back(msg->y_bar06_pitch); // (67)
    payload.push_back(msg->y_bar07_pitch); // (68)
    payload.push_back(msg->y_bar08_pitch); // (69)

    payload.push_back(msg->y_ws01_vy);     // (70)
    payload.push_back(msg->y_ws02_vy);     // (71)
    payload.push_back(msg->y_ws03_vy);     // (72)
    payload.push_back(msg->y_ws04_vy);     // (73)

    payload.push_back(msg->y_ws01_vyaw);   // (74)
    payload.push_back(msg->y_ws02_vyaw);   // (75)
    payload.push_back(msg->y_ws03_vyaw);   // (76)
    payload.push_back(msg->y_ws04_vyaw);   // (77)

    // 2) 转为字节指针
    const char* raw_ptr = reinterpret_cast<const char*>(payload.data());
    size_t total_bytes   = payload.size() * sizeof(double);

    // 3) 通过 UDP 发送到 Windows 侧
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

    // 打印日志 (可选)
    send_count_++;
    if (send_count_ % 2000 == 0) {
      RCLCPP_INFO(this->get_logger(), 
                  "Sent %ld bytes (count=%lu) to %s:%d", 
                  bytes_sent, send_count_,
                  target_ip_.c_str(), target_port_);
    }
  }

private:
  int udp_socket_fd_;
  std::string target_ip_;
  int target_port_;
  struct sockaddr_in remote_addr_;

  uint64_t send_count_;

  rclcpp::Subscription<simpack_interfaces::msg::SimpackY>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UDPSenderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
