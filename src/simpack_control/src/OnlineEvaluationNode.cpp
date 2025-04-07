// 文件路径: /home/yaoyao/Documents/myProjects/ROS2WithSPCK/src/simpack_control/src/OnlineEvaluationNode.cpp

#include "simpack_control/OnlineEvaluationNode.hpp"

// C/C++ 头文件
#include <cmath>
#include <complex>
#include <algorithm>
#include <iostream>

OnlineEvaluationNode::OnlineEvaluationNode(const rclcpp::NodeOptions & options)
: Node("online_evaluation_node", options),
  last_sperling_y_(0.0),
  last_sperling_z_(0.0),
  has_valid_result_(false),
  last_derailment_w01_(0.0),
  last_derailment_w02_(0.0),
  has_valid_derailment_(false)
{
  RCLCPP_INFO(this->get_logger(), "[OnlineEvaluationNode] Constructor...");

  // 1) 声明并获取参数: fs_, win_length_ (用于 Sperling)
  //    如果要给脱轨系数单独设置窗长，也可另外声明参数
  this->declare_parameter<double>("fs", 50.0);
  this->declare_parameter<double>("win_length", 5.0);

  fs_ = this->get_parameter("fs").as_double();
  win_length_ = this->get_parameter("win_length").as_double();

  RCLCPP_INFO(this->get_logger(),
    "Use fs=%.2f Hz, window=%.2f s (for Sperling).", fs_, win_length_);

  // 2) 创建发布者: /simpack/w
  pub_w_ = this->create_publisher<simpack_interfaces::msg::SimpackW>(
              "/simpack/w", 10);

  // 3) 创建订阅者: /simpack/y
  //    这里QoS可以根据需求 (best_effort / reliable)
  sub_y_ = this->create_subscription<simpack_interfaces::msg::SimpackY>(
              "/simpack/y",
              rclcpp::QoS(10).best_effort(),
              std::bind(&OnlineEvaluationNode::simpackYCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "[OnlineEvaluationNode] Ready.");
}

OnlineEvaluationNode::~OnlineEvaluationNode()
{
  RCLCPP_INFO(this->get_logger(), "[OnlineEvaluationNode] Destructor called!");
}

// ---------------------- 回调: 订阅 /simpack/y ----------------------
void OnlineEvaluationNode::simpackYCallback(const simpack_interfaces::msg::SimpackY::SharedPtr msg)
{
  double current_time = msg->sim_time;  // 实际仿真时间戳(单位 s)

  // 1) 更新 "Sperling" 滑动窗(5s) 的队列
  {
    time_buffer_.push_back(current_time);
    accy_buffer_.push_back(msg->y_comfort_accy);  // 横向加速度
    accz_buffer_.push_back(msg->y_comfort_accz);  // 垂向加速度

    // 弹出超过 window_length_ 以前的数据
    while (!time_buffer_.empty()) {
      if ((current_time - time_buffer_.front()) > win_length_) {
        time_buffer_.pop_front();
        accy_buffer_.pop_front();
        accz_buffer_.pop_front();
      } else {
        break;
      }
    }
  }

  // 2) 更新 "脱轨系数" 滑动窗(2s) 的队列
  {
    contact_time_buffer_.push_back(current_time);
    w01_fy_buffer_.push_back(msg->y_w01_contact_fy);
    w01_fz_buffer_.push_back(msg->y_w01_contact_fz);
    w02_fy_buffer_.push_back(msg->y_w02_contact_fy);
    w02_fz_buffer_.push_back(msg->y_w02_contact_fz);

    // 弹出超过2s的旧数据
    while (!contact_time_buffer_.empty()) {
      if ((current_time - contact_time_buffer_.front()) > 2.0) {
        contact_time_buffer_.pop_front();
        w01_fy_buffer_.pop_front();
        w01_fz_buffer_.pop_front();
        w02_fy_buffer_.pop_front();
        w02_fz_buffer_.pop_front();
      } else {
        break;
      }
    }
  }

  // -------------------- 先计算“脱轨系数” --------------------
  double derailment_w01 = 0.0;
  double derailment_w02 = 0.0;

  {
    double duration2s = 0.0;
    if (!contact_time_buffer_.empty()) {
      duration2s = contact_time_buffer_.back() - contact_time_buffer_.front();
    }

    if (duration2s < 2.0 - 0.001) {
      // 不足2秒 => fallback
      if (!has_valid_derailment_) {
        // 从未有过有效结果 => 置 0
        derailment_w01 = 0.0;
        derailment_w02 = 0.0;
      } else {
        // 使用上一次有效结果
        derailment_w01 = last_derailment_w01_;
        derailment_w02 = last_derailment_w02_;
      }
    } else {
      // >=2s => 进行2秒平均
      size_t N = contact_time_buffer_.size();
      double sum_w01_fy = 0.0;
      double sum_w01_fz = 0.0;
      double sum_w02_fy = 0.0;
      double sum_w02_fz = 0.0;

      for (size_t i = 0; i < N; ++i) {
        sum_w01_fy += w01_fy_buffer_[i];
        sum_w01_fz += w01_fz_buffer_[i];
        sum_w02_fy += w02_fy_buffer_[i];
        sum_w02_fz += w02_fz_buffer_[i];
      }
      double avg_w01_fy = sum_w01_fy / (double)N;
      double avg_w01_fz = sum_w01_fz / (double)N;
      double avg_w02_fy = sum_w02_fy / (double)N;
      double avg_w02_fz = sum_w02_fz / (double)N;

      // 计算 Q/P
      // 避免除以零, 若平均垂向力非常小, 设为 0 或者给一个安全上限
      if (std::fabs(avg_w01_fz) < 1e-6) {
        derailment_w01 = 0.0; // 或可给极大值, 视需求
      } else {
        derailment_w01 = avg_w01_fy / avg_w01_fz;
      }
      if (std::fabs(avg_w02_fz) < 1e-6) {
        derailment_w02 = 0.0;
      } else {
        derailment_w02 = avg_w02_fy / avg_w02_fz;
      }

      // 记录到 last
      last_derailment_w01_ = derailment_w01;
      last_derailment_w02_ = derailment_w02;
      has_valid_derailment_ = true;
    }
  }

  // -------------------- 准备构造输出 SimpackW 消息 --------------------
  simpack_interfaces::msg::SimpackW w_msg;
  w_msg.sim_time        = current_time;         // 作为输出消息时间戳
  w_msg.y_spcktime      = msg->y_spcktime;      // 与输入保持一致
  w_msg.derailment_w01  = derailment_w01;       // 刚计算/回退得到
  w_msg.derailment_w02  = derailment_w02;       // 同上

  // -------------------- 判断是否足够 5s 数据计算 Sperling --------------------
  double duration5s = 0.0;
  if (!time_buffer_.empty()) {
    duration5s = time_buffer_.back() - time_buffer_.front();
  }

  if (duration5s < (win_length_ - 0.001)) {
    // 不足5 s => fallback
    if (!has_valid_result_) {
      w_msg.sperling_y = 0.0;
      w_msg.sperling_z = 0.0;
    } else {
      // 直接重复发布上一次结果
      w_msg.sperling_y = last_sperling_y_;
      w_msg.sperling_z = last_sperling_z_;
    }
    // **无论如何也要发布一次**, 下游不会阻塞
    pub_w_->publish(w_msg);
    return;
  }

  // -------------------- 够 5s => 执行 Sperling (cubic) 计算 --------------------
  // 1) 复制窗口内数据
  std::vector<double> tvec(time_buffer_.begin(), time_buffer_.end());
  std::vector<double> yvec(accy_buffer_.begin(), accy_buffer_.end());
  std::vector<double> zvec(accz_buffer_.begin(), accz_buffer_.end());

  // 2) 构造等步长时间数组(采样率 fs_)
  double t_min = tvec.front();
  double t_max = tvec.back();
  if (t_min >= t_max) {
    // 防御性, 理论上不会出现
    w_msg.sperling_y = (has_valid_result_) ? last_sperling_y_ : 0.0;
    w_msg.sperling_z = (has_valid_result_) ? last_sperling_z_ : 0.0;
    pub_w_->publish(w_msg);
    return;
  }

  double dt_uniform = 1.0 / fs_;
  std::vector<double> t_uniform;
  t_uniform.reserve(static_cast<size_t>((t_max - t_min)/dt_uniform + 2));
  for(double t = t_min; t <= t_max + 1e-10; t += dt_uniform) {
    t_uniform.push_back(t);
  }

  // 3) 线性插值
  std::vector<double> y_uniform = linearInterpolation(tvec, yvec, t_uniform);
  std::vector<double> z_uniform = linearInterpolation(tvec, zvec, t_uniform);

  // 4) 计算 Sperling(cubic)
  auto result = computeSperlingCubic(y_uniform, z_uniform, fs_);
  double new_sperling_y = result.first;   // 横向
  double new_sperling_z = result.second;  // 垂向

  // 5) 存储并发布
  last_sperling_y_ = new_sperling_y;
  last_sperling_z_ = new_sperling_z;
  has_valid_result_ = true;

  w_msg.sperling_y = new_sperling_y;
  w_msg.sperling_z = new_sperling_z;

  pub_w_->publish(w_msg);
}

// ====================== 计算 Sperling (cubic) ======================
std::pair<double, double> OnlineEvaluationNode::computeSperlingCubic(
    const std::vector<double> & accY,
    const std::vector<double> & accZ,
    double fs)
{
  size_t N = accY.size();
  if (N < 2) {
    return {0.0, 0.0};
  }
  double df = fs / static_cast<double>(N);

  // 1) Naive DFT (示例用, N可不大)
  std::vector<std::complex<double>> specY(N), specZ(N);

  for (size_t k=0; k<N; ++k) {
    std::complex<double> sumY(0.0, 0.0);
    std::complex<double> sumZ(0.0, 0.0);
    for (size_t n=0; n<N; ++n) {
      double angle = -2.0 * M_PI * (double)k * (double)n / (double)N;
      double c = std::cos(angle);
      double s = std::sin(angle);
      sumY += std::complex<double>(accY[n]*c, accY[n]*s);
      sumZ += std::complex<double>(accZ[n]*c, accZ[n]*s);
    }
    specY[k] = sumY;
    specZ[k] = sumZ;
  }

  // 单边幅值
  size_t halfN = N/2;
  if (N % 2) {
    halfN = (N-1)/2;
  }
  std::vector<double> freq(halfN+1, 0.0), ampY(halfN+1, 0.0), ampZ(halfN+1, 0.0);

  for (size_t k=0; k<=halfN; k++){
    freq[k] = k*df;
    double magY = std::abs(specY[k]);
    double magZ = std::abs(specZ[k]);
    ampY[k] = (2.0/(double)N) * magY;
    ampZ[k] = (2.0/(double)N) * magZ;
  }

  // 2) 频带 [0.5, min(40, fs/2)]
  double fHigh = std::min(40.0, fs/2.0);
  double fLow  = 0.5;
  int idxLow  = -1;
  int idxHigh = -1;
  for (size_t i=0; i<freq.size(); i++){
    if (idxLow<0 && freq[i]>=fLow) {
      idxLow = (int)i;
    }
    if (freq[i] > fHigh) {
      idxHigh = (int)i - 1;
      break;
    }
  }
  if (idxHigh<0) {
    idxHigh = (int)halfN;
  }
  if (idxLow<0 || idxLow>idxHigh) {
    return {0.0, 0.0};
  }

  // 3) Sperling(cubic): sum of (B_sl * A)^3 => ^(1/10)
  double sumLat  = 0.0;
  double sumVert = 0.0;
  for (int i=idxLow; i<=idxHigh; i++){
    double f    = freq[i];
    double valY = ampY[i];
    double valZ = ampZ[i];
    double bsv = Bsv(f);
    double bsl = Bsl(f);

    sumLat  += std::pow(bsl*valY, 3.0);
    sumVert += std::pow(bsv*valZ, 3.0);
  }

  double wz_lat  = std::pow(sumLat,  0.1);  // ^(1/10)
  double wz_vert = std::pow(sumVert, 0.1);
  return {wz_lat, wz_vert};
}

// ====================== 线性插值函数 ======================
std::vector<double> OnlineEvaluationNode::linearInterpolation(
  const std::vector<double> & t_in,
  const std::vector<double> & x_in,
  const std::vector<double> & t_uniform)
{
  std::vector<double> x_out(t_uniform.size(), 0.0);
  size_t N = t_in.size();
  if (N < 2) {
    return x_out;
  }

  size_t idx = 0;
  for (size_t i=0; i<t_uniform.size(); i++){
    double t = t_uniform[i];
    // 移动 idx，使 t_in[idx] <= t <= t_in[idx+1]
    while ( (idx+1 < N) && (t_in[idx+1] < t) ) {
      idx++;
    }
    if (idx >= N-1) {
      // 超过右端
      x_out[i] = x_in[N-1];
      continue;
    }
    if (t < t_in[0]) {
      // 超过左端
      x_out[i] = x_in[0];
      continue;
    }

    // 线性插值
    double t0 = t_in[idx];
    double t1 = t_in[idx+1];
    double x0 = x_in[idx];
    double x1 = x_in[idx+1];

    double dt = t1 - t0;
    if (std::fabs(dt) < 1e-12) {
      x_out[i] = x0;
    } else {
      double ratio = (t - t0)/dt;
      x_out[i] = x0 + ratio*(x1 - x0);
    }
  }

  return x_out;
}

// ====================== Sperling加权函数 ======================
double OnlineEvaluationNode::Bsv(double f)
{
  // B_{Sv}(f) = 58.8 * sqrt( [1.911*f^2 + (0.25*f^2)^2 ] / ((1-0.277*f^2)^2 + (1.563*f - 0.0368*f^3)^2) )
  double numerator   = 1.911 * f*f + std::pow(0.25 * f*f, 2.0);
  double denominator = std::pow(1.0 - 0.277*f*f, 2.0)
                     + std::pow(1.563*f - 0.0368*f*f*f, 2.0);
  if (denominator < 1e-12) {
    return 0.0;
  }
  double val = 58.8 * std::sqrt(numerator / denominator);
  return val;
}

double OnlineEvaluationNode::Bsl(double f)
{
  // B_{Sl}(f) = 1.25 * B_{Sv}(f)
  return 1.25 * Bsv(f);
}
