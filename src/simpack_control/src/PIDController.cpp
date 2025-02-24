// 文件名: PIDController.cpp

#include "simpack_control/PIDController.hpp"
#include <cmath>

PIDController::PIDController(double kp, double ki, double kd, double n)
  : kp_(kp), ki_(ki), kd_(kd), n_(n),
    integral_(0.0), prevError_(0.0), derivativeTerm_(0.0)
{}

void PIDController::reset()
{
    integral_ = 0.0;
    prevError_ = 0.0;
    derivativeTerm_ = 0.0;
}

double PIDController::update(double error, double dt)
{
    // 积分
    integral_ += error * dt;

    // 微分原始值
    double derivativeRaw = 0.0;
    if (dt > 1e-12) {
        derivativeRaw = (error - prevError_) / dt;
    }

    // 一阶滤波: alpha = (n*dt) / (1 + n*dt)
    double alpha = (n_ * dt) / (1.0 + n_ * dt);
    derivativeTerm_ = (1.0 - alpha)*derivativeTerm_ + alpha*derivativeRaw;

    // PID 输出
    double output = (kp_ * error) + (ki_ * integral_) + (kd_ * derivativeTerm_);

    // 保存本次误差
    prevError_ = error;
    return output;
}
