/*

文件名: PIDController.h

这是一个与 Python 逻辑对应的 并行型 PID + 一阶滤波微分 类，可以直接拷贝到工程中使用。

*/

#pragma once

class PIDController
{
public:
    PIDController(double kp, double ki, double kd, double n);
    ~PIDController() = default;

    void reset();
    double update(double error, double dt);

private:
    double kp_, ki_, kd_, n_;
    double integral_;
    double prevError_;
    double derivativeTerm_;
};
