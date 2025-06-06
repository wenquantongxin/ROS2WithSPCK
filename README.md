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
- 输出通道数量：ny=77（系统输出）
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


## 运行环境配置

在 pypack 环境之中，使用 Anaconda Prompt 安装:

- conda install pytorch torchvision torchaudio pytorch-cuda=12.1 -c pytorch -c nvidia
- conda install pymoo pandas numpy ipywidgets matplotlib openpyxl ipykernel
- pip install -U pymoo ipympl scikit-learn seaborn SALib
- pip install pyopengl pyopengl-accelerate ffmpeg  ffmpeg-python


## ROS2工作空间的编译
  
    cd /home/yaoyao/Documents/myProjects/ROS2WithSPCK
    rm -rf build/ install/ log/ 
    colcon build --cmake-args -Wno-dev
    source install/setup.bash  

## launch启动

    cd /home/yaoyao/Documents/myProjects/ROS2WithSPCK
    source install/setup.bash  
    ros2 launch simpack_control start_simpack_control.launch.py

## 分窗口启动 ROS2 Node (窗口1: 先启动 SIMPACK 仿真器, 窗口2: 再启动 PIDT4 控制器)
    # 窗口 1 (与 SPCK 的数据交互节点)
    cd /home/yaoyao/Documents/myProjects/ROS2WithSPCK
    source install/setup.bash 
    ros2 run simpack_control simpack_node

    # 窗口 2  (可在局域网另一主机上运行此PID控制节点)
    cd /home/yaoyao/Documents/myProjects/ROS2WithSPCK
    source install/setup.bash
    ros2 run simpack_control controller_node

    # 窗口 3  (UDP 跨机通信 Node, 发送至 连接 192.168.1.115:10088, 可能被同局域网“泛洪”式接收)
    cd /home/yaoyao/Documents/myProjects/ROS2WithSPCK
    source install/setup.bash  
    ros2 run simpack_control trkrel_udpsender_node  

    # 窗口 4  (UDP 跨机发送绝对坐标系 Node, 连接 192.168.1.131:10099)
    cd /home/yaoyao/Documents/myProjects/ROS2WithSPCK
    source install/setup.bash  
    ros2 run simpack_control trkabs_udpsender_node 

## 跨机渲染
    # 窗口 1 (UDP发送绝对空间坐标，显示与渲染)
    conda activate pypack && cd /home/yaoyao/Documents/myProjects/ROS2WithSPCK/SPCK_Track && python TrkAbs_RTViz.py     # Ubuntu

    E: && cd E:\ResearchDocuments\ROS2WithSPCK\SPCK_Track && python TrkAbs_RTViz.py                                     # Windows

    # 窗口 2 (UDP发送轨道局部坐标系下坐标，显示与渲染)
    conda activate pypack && cd /home/yaoyao/Documents/myProjects/ROS2WithSPCK/SPCK_Track && python TrkRel_RTViz.py     # Ubuntu

    E: && cd E:\ResearchDocuments\ROS2WithSPCK\SPCK_Track && python TrkRel_RTViz.py                                     # Windows


## Ubuntu 实时数据流监控与回放
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



