#include <iostream>
#include <fstream>
#include <vector>
#include "PiecewiseTrajectory.h"

int main()
{
    // 准备输入数据，与 Python 示例对应
    double v_begin1 = 60.0 / 3.6; // ~16.6667
    double v_end2   = (60.0 / 3.6) / 300.0 * (300.0 + 1.5/2.0); // ~16.7083

    std::vector<double> t_array = {0.0, 3.0, 6.0, 50};
    std::vector<double> f_array = {v_begin1, v_begin1, v_end2, v_end2};

    // 构造 PiecewiseConstantSecondDeriv
    PiecewiseConstantSecondDeriv traj(t_array, f_array);

    // 打开一个CSV文件
    std::ofstream ofs("CPPRefTrajectory_Data.csv");
    if(!ofs.is_open()) {
        std::cerr << "Failed to open data.csv\n";
        return 1;
    }

    // 写表头
    ofs << "t,f,v,a\n";

    // 采样区间 [0,50], 500 个点
    int n_points = 3000;
    double t_min = 0.0, t_max = 50.0;
    for(int i=0; i<n_points; ++i) {
        double t = t_min + (t_max - t_min) * i / (n_points - 1.0);
        double f_val = traj.getValue(t);
        double v_val = traj.getDerivative(t, 1);
        double a_val = traj.getDerivative(t, 2);
        ofs << t << "," << f_val << "," << v_val << "," << a_val << "\n";
    }

    ofs.close();
    std::cout << "Data written to CPPRefTrajectory_Data.csv\n";
    return 0;
}

/*

 - 编译: 
    g++ -std=c++17 GenRefTrajectory.cpp PiecewiseTrajectory.cpp -o test_traj

 - 运行:
    ./test_traj

 - 可视化(Python):
    读取 CPPRefTrajectory_Data.csv

*/