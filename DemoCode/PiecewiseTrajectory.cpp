// PiecewiseTrajectory.cpp 目标速度曲线生成函数

#include "PiecewiseTrajectory.h"

// ========== TwoHalfParabolas ==========

TwoHalfParabolas::TwoHalfParabolas(double t_start, double t_end,
                                   double f_begin, double f_end)
  : t_start_(t_start), t_end_(t_end),
    f_begin_(f_begin), f_end_(f_end)
{
    T_ = t_end_ - t_start_;
    double df = f_end_ - f_begin_;

    // 若 T=0 或 Δf=0 => 视为常值段, a=0
    if (std::fabs(T_) < 1e-12 || std::fabs(df) < 1e-12) {
        a_ = 0.0;
    } else {
        // a = 4 * (f_end - f_begin) / T^2
        a_ = 4.0 * df / (T_ * T_);
    }
}

double TwoHalfParabolas::getValue(double t) const
{
    // 若 t 不在该段范围内, 返回 DBL_MAX 作为标记值
    if (t < t_start_ || t > t_end_) {
        return DBL_MAX;
    }
    // 若 a=0, 表示该段常值
    if (std::fabs(a_) < 1e-12) {
        return f_begin_;
    }

    double tau = t - t_start_;
    double halfT = 0.5 * T_;

    // 前半段
    if (tau <= halfT) {
        // f(t) = f_begin + 0.5*a_ * tau^2
        return f_begin_ + 0.5 * a_ * (tau * tau);
    }
    else {
        // 后半段
        double tm = halfT;
        double f_mid = f_begin_ + 0.5 * a_ * (tm * tm); // 中点位置
        double v_mid = a_ * tm;                         // 中点速度
        double tau_local = tau - tm;

        // 后半段加速度 = -a, 故 f(t) = f_mid + v_mid*tau_local - 0.5*a_*tau_local^2
        return f_mid + v_mid * tau_local - 0.5 * a_ * (tau_local * tau_local);
    }
}

double TwoHalfParabolas::getDerivative(double t, int order) const
{
    // 同样先判断是否在该段范围内
    if (t < t_start_ || t > t_end_) {
        return DBL_MAX;
    }
    // a=0 (常值段), 任何阶导数都为0
    if (std::fabs(a_) < 1e-12) {
        return 0.0;
    }

    // 前后半段区分
    double tau = t - t_start_;
    double halfT = 0.5 * T_;

    if (order == 1) {
        // 一阶导数(速度):
        // 前半段: v(t) = a * tau
        // 后半段: v(t) = v_mid - a * (tau - T/2)
        if (tau <= halfT) {
            return a_ * tau;
        } else {
            double v_mid = a_ * halfT;
            double tau_local = tau - halfT;
            return v_mid - a_ * tau_local;
        }
    }
    else if (order == 2) {
        // 二阶导数(加速度):
        // 前半段: +a, 后半段: -a
        if (tau <= halfT) {
            return a_;
        } else {
            return -a_;
        }
    }
    else {
        // 3阶及以上 => 0
        return 0.0;
    }
}

// ========== PiecewiseConstantSecondDeriv ==========

PiecewiseConstantSecondDeriv::PiecewiseConstantSecondDeriv(const std::vector<double>& times,
                                                           const std::vector<double>& values)
{
    // 基本合法性检查
    int n = static_cast<int>(times.size());
    if (n < 2 || n != static_cast<int>(values.size())) {
        throw std::runtime_error("PiecewiseConstantSecondDeriv: input size mismatch");
    }

    // 相邻时刻构造一段 TwoHalfParabolas
    for (int i = 0; i < n - 1; ++i) {
        double t_start = times[i];
        double t_end   = times[i+1];
        double f_beg   = values[i];
        double f_end   = values[i+1];

        TwoHalfParabolas seg(t_start, t_end, f_beg, f_end);
        segments_.push_back(seg);
    }

    t_end_total_ = times.back();
    f_end_total_ = values.back();
}

double PiecewiseConstantSecondDeriv::getValue(double t) const
{
    // 若 t >= 最后一段结束, 返回末段常值
    if (t >= t_end_total_) {
        return f_end_total_;
    }
    // 若 t < 第一段开始, 则默认返回第一段开始的值(可改为其它逻辑)
    if (!segments_.empty() && t < segments_.front().start()) {
        return segments_.front().getValue(segments_.front().start());
    }

    // 在各段中查找
    for (auto &seg : segments_) {
        if (t >= seg.start() && t <= seg.end()) {
            double val = seg.getValue(t);
            if (val < DBL_MAX) {
                return val;
            }
        }
    }

    // 理论上不会走到这,除非传入的 t 比第一个段还早, 或浮点误差
    // 保险起见返回第一段的起点值
    return segments_.front().getValue(segments_.front().start());
}

double PiecewiseConstantSecondDeriv::getDerivative(double t, int order) const
{
    // 超过最后一段 => 保持常值 => 导数=0
    if (t >= t_end_total_) {
        return 0.0;
    }
    // 若 t < 第一段开始, 则返回0(因为保持首段常值)
    if (!segments_.empty() && t < segments_.front().start()) {
        return 0.0;
    }

    // 在各段中查找
    for (auto &seg : segments_) {
        if (t >= seg.start() && t <= seg.end()) {
            double val = seg.getDerivative(t, order);
            if (val < DBL_MAX) {
                return val;
            }
        }
    }

    // 若找不到(极少浮点误差情况), 也返回0
    return 0.0;
}

