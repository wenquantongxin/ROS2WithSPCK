#pragma once

#include <vector>
#include <stdexcept>   // 重要：使用 std::runtime_error 需要此头
#include <cfloat>      // DBL_MAX
#include <cmath>       // std::fabs

/*
 * 二段半抛物线:
 * 在时刻区间 [t_start, t_end] 内, 让 f(t) 从 f_begin 平滑过渡到 f_end,
 * 保证段首、段尾的一阶导数=0.
 *
 * a = 4*(f_end - f_begin) / (T^2).
 * 前半段加速度=+a, 后半段加速度=-a.
 */
class TwoHalfParabolas
{
public:
    TwoHalfParabolas(double t_start, double t_end, double f_begin, double f_end);

    // 主函数: 在该段内返回位置值; 若 t 不在 [t_start, t_end] 则返回 DBL_MAX
    double getValue(double t) const;

    // 求导数, order=1 => 一阶, order=2 => 二阶, 3及以上 => 0;
    // 不在该段内返回 DBL_MAX, 以供上层判断
    double getDerivative(double t, int order=1) const;

    double start() const { return t_start_; }
    double end()   const { return t_end_; }

private:
    double t_start_, t_end_;
    double f_begin_, f_end_;
    double T_;
    double a_; // 加速度绝对值(可能为正/负, 实际用 +a / -a 分段)
};

// 拼接多段 TwoHalfParabolas, 实现分段平滑过渡
class PiecewiseConstantSecondDeriv
{
public:
    // times.size() == values.size(), 相邻时刻构造一段 TwoHalfParabolas
    PiecewiseConstantSecondDeriv(const std::vector<double>& times,
                                 const std::vector<double>& values);

    // 返回时刻 t 的 f(t) 值; 超过最后一段则返回末段保持值
    double getValue(double t) const;

    // 返回第 order 阶导数; 超过最后一段则为0(因为保持常值)
    double getDerivative(double t, int order=1) const;

private:
    std::vector<TwoHalfParabolas> segments_;
    double t_end_total_; // 最后一段的结束时间
    double f_end_total_; // 最后一段的结束值
};

