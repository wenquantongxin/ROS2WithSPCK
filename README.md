# ROS2WithSPCK
ROS2与SIMPACK仿真软件的控制系统交互，进行车辆动力学**实时联合仿真**，同时记录仿真过程中的数据。

## ROS2项目结构与配置
- launch启动：ros2 launch simpack_control start_simpack_control.launch.py
- ROS2日志路径：/home/yaoyao/.ros/log/2025-0X-YY/...

## 节点信息
- simpack_node：与SIMPACK仿真软件交互的节点
- controller_node：控制器节点，配置信息为wheelRadius=0.43, dt=0.0020
- rosbag_recorder：用于记录话题数据的ROS2 bag节点

## 话题与通信 - 订阅的话题
- /rosout
- /parameter_events
- /events/write_split
- /simpack/u
- /simpack/y

## SIMPACK节点配置

- 输入通道数量：nu=8（控制输入）
- 输出通道数量：ny=28（系统输出）
- 时间步长：dt=0.0020
- 最大仿真步数：max_steps=25000

## 数据记录
- ROS2 bag记录位置：PostAnalysis/rosbag2_2025-0X-YY/...
- 数据库文件：PostAnalysis/rosbag2_2025-0X-YY/rosbag2_2025-0X-YY.db3
- 仿真结果文件：./PostAnalysis/Result_ROSwithSPCKrt.log

## 仿真执行信息
- 仿真模型：Vehicle4WDB_RealtimeCRV.spck
- 仿真总时长：约50秒（tcur = 49.9990215702657）
- 实时因子：1.22（表示仿真速度稍慢于实际时间1.22倍）
- 仿真于达到最大步数后停止：Reached max_steps(25000)

## 启动和关闭流程
- 启动顺序：先启动simpack_node，再启动rosbag_recorder，最后启动controller_node
- 关闭顺序：先关闭simpack_node，然后关闭controller_node，最后停止rosbag_recorder


## ROS2工作空间的编译
  
    cd /home/yaoyao/Documents/myProjects/ROS2WithSPCK
    rm -rf build/ install/ log/ 
    colcon build --cmake-args -Wno-dev
    source install/setup.bash  

## launch启动
    cd /home/yaoyao/Documents/myProjects/ROS2WithSPCK
    source install/setup.bash  
    ros2 launch simpack_control start_simpack_control.launch.py

## 分窗口启动 (窗口1: 先启动 SIMPACK 仿真器, 窗口2: 再启动 PIDT4 控制器)
    # 窗口1
    cd /home/yaoyao/Documents/myProjects/ROS2WithSPCK
    source install/setup.bash 
    ros2 run simpack_control simpack_node

    # 窗口2 (可以在局域网的另一个配置了ROS2的主机上运行节点)
    cd /home/yaoyao/Documents/myProjects/ROS2WithSPCK
    source install/setup.bash
    ros2 run simpack_control controller_node

## 实时数据流监控与回放
    cd /home/yaoyao/Documents/myProjects/ROS2WithSPCK && source install/setup.bash 
    ros2 run rqt_topic rqt_topic                              # 监控 ROS 2 主题消息传递的详细数据

    cd /home/yaoyao/Documents/myProjects/ROS2WithSPCK && source install/setup.bash 
    ros2 run rqt_graph rqt_graph                              # 可视化 ROS 2 节点和主题之间的关系   

    cd /home/yaoyao/Documents/myProjects/ROS2WithSPCK && source install/setup.bash 
    ros2 topic list                                           # 列出当前系统中所有活跃的 ROS 2 主题

    cd /home/yaoyao/Documents/myProjects/ROS2WithSPCK && source install/setup.bash 
    ros2 bag play -r 0.5 ./PostAnalysis/rosbag2_2025_XXXX     # ROS Bag 回放

    cd /home/yaoyao/Documents/myProjects/ROS2WithSPCK && source install/setup.bash 
    ros2 run plotjuggler plotjuggler                          # plotjuggler 实时播放与回放 (安装: sudo apt install ros-humble-plotjuggler-ros)
