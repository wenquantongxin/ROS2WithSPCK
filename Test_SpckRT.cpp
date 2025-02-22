/*
 * 文件名: Test_SpckRT.cpp
 *
 * 说明：
 *  - 使用 SIMPACK Realtime (POSIX MQ 模式) + C++ PID 控制器 + PiecewiseTrajectory
 *  - 对 8 个轮对做并行 PID：每步从 SIMPACK 获取各车轮轮速 y[]，经过误差计算 + PID，写入扭矩到 u[]，推进下一步
 *  - 左侧和右侧车轮有不同的目标速度曲线
 *
 
   编译命令：
   g++ Test_SpckRT.cpp PIDController.cpp PiecewiseTrajectory.cpp -std=c++11 -O2 -Wall \
      -L /opt/Simpack-2021x/run/realtime/linux64 \
      -I /opt/Simpack-2021x/run/realtime \
      -lspck_rt -lrt -lm \
      -o Test_SpckRT_demo                                 
   
   运行：
   ./Test_SpckRT_demo                                           

 */

#include <cstdio>    // C 风格的输入输出
#include <cstdlib>   // 常用工具函数
#include <cstring>   // C 风格字符串操作
#include <iostream>  // C++ 风格I/O
#include <fstream>   // 用于文件输出
#include <vector>
#include <cmath>
#include <string>
#include <sstream>    // std::ostringstream

// 你的 PID 控制器与 分段曲线头文件
#include "PIDController.h"
#include "PiecewiseTrajectory.h"

// spck_rt_v1.h (或 spck_rt.h) 是纯 C 头文件, 在 C++ 中需要 extern "C"
extern "C" {
   #include "/opt/Simpack-2021x/run/realtime/spck_rt.h"
}

// ----------------- 宏定义 -----------------------
#define SPCK_PATH       "/opt/Simpack-2021x" // SIMPACK 安装目录
#define MODEL_FILE      "/home/yaoyao/Documents/myProjects/ROS2WithSPCK/Vehicle4WDB_RealtimeCRV.spck" // 模型文件
#define SPCK_MODE       E_RT_MODE__KEEP_SLV  // 实时模式: 保持 solver 进程不退出
#define SPCK_CPUS       ""                   // CPU 绑定字符串 (空表示不绑定)
#define SPCK_RT_PRIO    80                   // 实时优先级, 需系统权限 (非0表示硬实时), 若无权限可改为0
#define SPCK_VERBOSE    0                   // SIMPACK 日志输出详细度(0~5)
#define SPCK_POSIX_TIMEOUT 5              // POSIX MQ 超时时间(秒), 一般略大于步长

// 仿真时长、步长、输入输出
static const double SIM_DURATION = 50.0;     // 仿真总时长(秒)
static const double DT           = 0.002;    // 仿真步长(秒)
static const int    NU_EXPECTED  = 8;        // 模型里定义的 u-inputs 数量
static const int    NY_EXPECTED  = 28;       // 模型里定义的 y-outputs 数量

// 半径相关, 在python中用 860/2000=0.43m
// SIMPACK 中读取到的车轮转速是负值 => 实际线速度 = -y[index]*R
static const double WHEEL_RADIUS = 0.43; // 车轮半径 0.43 m

// SIMPACK 输出 y[] 的索引，y[8..15]分别对应车轮转速 (WL01..WR04)
// 具体请核实 SIMPACK Realtime Outputs 的顺序
static const int INDEX_WL01 =  8;
static const int INDEX_WR01 =  9;
static const int INDEX_WL02 = 10;
static const int INDEX_WR02 = 11;
static const int INDEX_WL03 = 12;
static const int INDEX_WR03 = 13;
static const int INDEX_WL04 = 14;
static const int INDEX_WR04 = 15;

// -------------------- 目标速度曲线 --------------------
// 左侧终末速度, 右侧终末速度
static double v_begin     = 60.0/3.6; // 16.6667
static double v_left_end  = (60.0/3.6)/300.0 * (300.0 + 1.5/2.0);  // 左侧车轮 目标线速度 终末值
static double v_right_end = (60.0/3.6)/300.0 * (300.0 - 1.5/2.0);  // 右侧车轮 目标线速度 终末值

// times + vals_left/vals_right: 0~3s恒值, 3~6s过渡, 6~100s后保持
static std::vector<double> times = {0.0, 3.0, 6.0, 100.0};
static std::vector<double> vals_left  = {v_begin, v_begin, v_left_end,  v_left_end};
static std::vector<double> vals_right = {v_begin, v_begin, v_right_end, v_right_end};

int main()
{
   // 定义数组记录列名
   // 注意: 数组大小必须是 28+1=29, 第一个是 "Time"，后面是 28 个 y-output
   static const char* g_columnNames[29] = {
      "\"Time\"",
      "\"$Y_Yw01\"", "\"$Y_Yw02\"", "\"$Y_Yw03\"", "\"$Y_Yw04\"",
      "\"$Y_Yaw01\"", "\"$Y_Yaw02\"", "\"$Y_Yaw03\"", "\"$Y_Yaw04\"",
      "\"$Y_WL01\"", "\"$Y_WR01\"", "\"$Y_WL02\"", "\"$Y_WR02\"",
      "\"$Y_WL03\"", "\"$Y_WR03\"", "\"$Y_WL04\"", "\"$Y_WR04\"",
      "\"$Y_Yt01\"", "\"$Y_Yt02\"", "\"$Y_Yawt01\"", "\"$Y_Yawt02\"",
      "\"$Y_SpeedDiff_FrontA\"", "\"$Y_SpeedDiff_FrontB\"", "\"$Y_SpeedDiff_FrontC\"", "\"$Y_SpeedDiff_FrontD\"",
      "\"$Y_SpeedDiff_RearA\"",  "\"$Y_SpeedDiff_RearB\"",  "\"$Y_SpeedDiff_RearC\"",  "\"$Y_SpeedDiff_RearD\""
   };

   std::cout << "=== Start POSIX MQ Realtime Simulation ===\n";
   std::cout << "=== 开始基于 POSIX 消息队列的 SIMPACK 实时仿真 ===\n";

   int ierr = 0;            // 用于检测返回错误码
   int nu = 0, ny = 0;      // 存储从 SIMPACK 获得的维度
   double time = 0.0;       // 当前仿真时间
   int steps = static_cast<int>(SIM_DURATION / DT + 0.5); // 总步数(近似四舍五入)

   // 1) 初始化与 SIMPACK 通信 (POSIX MQ)
   ierr = SpckRtInitPM(
      SPCK_MODE,       // mode, E_RT_MODE__KEEP_SLV等
      SPCK_PATH,       // SIMPACK安装目录
      MODEL_FILE,      // 模型文件的绝对路径
      SPCK_CPUS,       // CPU绑定, 空字符串
      SPCK_RT_PRIO,    // 实时优先级
      SPCK_VERBOSE, // int verbose
      nullptr,         // baseName, 让 SIMPACK 自动命名消息队列
      SPCK_POSIX_TIMEOUT // 超时阈值
   );

   if (ierr != 0)
   {
      std::cerr << "[Error] SpckRtInitPM failed, ierr=" << ierr << std::endl;
      return 1; 
   }

   // 2) 获取 u,y 维度
   SpckRtGetUYDim(&nu, &ny);
   std::cout << "Dim of u: " << nu << ", Dim of y: " << ny << std::endl;
   std::cout << "u-input 的维度: " << nu << ", y-output 的维度: " << ny << std::endl;

   // 检查是否与预期匹配
   if (nu != NU_EXPECTED || ny != NY_EXPECTED)
   {
      std::cout << "ny 实际维度与预定义的"<< NY_EXPECTED <<"维度不匹配, 请检查参数定义! " << std::endl;
      std::cerr << "[Warning] The model has nu=" << nu << ", ny=" << ny
               << ", which differs from expected ("
               << NU_EXPECTED << ", " << NY_EXPECTED << ").\n";
      // ny 维度错误提示
   }

   // 3) 分配内存
   double* u = new double[nu];
   double* y = new double[ny];
   std::memset(u, 0, nu*sizeof(double));
   std::memset(y, 0, ny*sizeof(double));

   // 4) 启动实时求解器
   ierr = SpckRtStart();
   if (ierr != 0)
   {
      std::cerr << "[Error] SpckRtStart failed, ierr=" << ierr << std::endl;
      // 调 Finish 收尾
      SpckRtFinish();
      delete[] u;
      delete[] y;
      return 1;
   }

   // 5) 打开日志文件
   std::ofstream logFile("SimResult.log");
   if (!logFile.is_open()) {
      std::cerr << "[Error] Failed to open SimResult.log for writing.\n";
      // 必要时先结束Simpack
      SpckRtFinish();
      delete[] u;
      delete[] y;
      return 1;
   }

   // 先写一个标题行，包含 Time 以及 28 个输出的名字
   // 不再使用 # time(s) y0 y1 ... y27, 而是改为以双引号+Tab 分隔
   for (int i = 0; i < ny + 1; ++i) {
      logFile << g_columnNames[i];
      if (i < 28) {
         logFile << "\t";  // 列间用TAB分隔
      }
      else {
         logFile << "\n";  // 最后一列后换行
      }
   }

   //不直接写文件, 改为写内存缓冲
   std::ostringstream logBuffer;

   // 6) 构建左右车轮的目标速度曲线
   PiecewiseConstantSecondDeriv trajLeft(times,  vals_left);
   PiecewiseConstantSecondDeriv trajRight(times, vals_right);

   // 7) 创建 8 个 PIDController
   double kp=20000.0, ki=4.0, kd=0.0, n=5000.0;
   std::vector<PIDController> pids;
   pids.reserve(8);
   for(int i=0; i<8; ++i) {
      pids.emplace_back(kp, ki, kd, n);
   }
   for (auto &pid : pids) {
      pid.reset();
   }

   // 8) 对应关系
   int yIndex[8] = {
      INDEX_WL01, INDEX_WR01, INDEX_WL02, INDEX_WR02,
      INDEX_WL03, INDEX_WR03, INDEX_WL04, INDEX_WR04
   };
   int uIndex[8] = {0,1,2,3,4,5,6,7};

   // 哪些是左轮，哪些是右轮
   bool isLeftWheel[8] = {true, false, true, false, true, false, true, false};

   // 9) 进入仿真循环
   //    每一步: 
   //      - 获取当前y输出
   //      - 设置u
   //      - 时刻 time += dt
   //      - SpckRtAdvance到下一个时刻
   for(int iStep=0; iStep<=steps; ++iStep)
   {
      time = iStep * DT;

      // 9.1 拿当前 y[]
      SpckRtGetY(y);

      // 9.2 计算左右侧目标速度
      double targetLeft  = trajLeft.getValue(time);
      double targetRight = trajRight.getValue(time);

      // 9.3 并行PID
      //     actual_speed = - y[yIndex[w]] * WHEEL_RADIUS
      //     error = (targetLeft 或 targetRight) - actual_speed
      for(int w=0; w<8; ++w) {
         double rawSpeed = y[ yIndex[w] ];
         double actualSpeed = - rawSpeed * WHEEL_RADIUS;  // 负号
         double curTarget = isLeftWheel[w] ? targetLeft : targetRight;
         double error = curTarget - actualSpeed;

         double torque = pids[w].update(error, DT);
         // (可选)限幅
         // if (torque > 30000) torque = 30000;
         // if (torque < -30000) torque = -30000;

         u[ uIndex[w] ] = torque;
      }

      // 9.4 写u到 SIMPACK
      SpckRtSetU(u);

      // // 9.5 记录日志 (先写 time, 再写 y[0..ny-1])
      // logFile << time;
      // for (int j=0; j<ny; ++j) {
      //    logFile << "\t" << y[j];
      // }
      // logFile << "\n";
      
      // 9.5 记录到 logBuffer
      logBuffer << time;
      for (int j=0; j<ny; ++j) {
         logBuffer << "\t" << y[j];
      }
      logBuffer << "\n";

      // 9.6 推进到下一时刻
      if (iStep < steps) {
         double tNext = (iStep+1)*DT;
         ierr = SpckRtAdvance(tNext);
         if (ierr == 1) {
            std::cerr << "[Error] SpckRtAdvance solver error.\n";
            break;
         } else if (ierr == 2) {
            std::cerr << "[Warning] SpckRtAdvance timeout.\n";
            break;
         }
      }
   }
   
   // 10) 收尾
   // 仿真结束后，统一打开文件，一次性写入记录的数据
   if(!logFile.is_open()){
      std::cerr << "Failed to open SimResult.log\n";
   } else {
      logFile << logBuffer.str();
      logFile.close();
   }

   // API 关闭 SPCK
   SpckRtFinish();
   delete[] u;
   delete[] y;

   std::cout << "Simulation finished! Results in SimResult.log\n";
   std::cout << "SIMPACK 实时联合仿真结束, 结果储存于文件: SimResult.log\n";
   return 0;
}