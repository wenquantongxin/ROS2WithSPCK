# ROS2WithSPCK
实现ROS 2与SIMPACK的联合实时仿真

- **编译**
  
    cd /home/yaoyao/Documents/myProjects/ROS2WithSPCK
    rm -rf build/ install/ log/ 
    colcon build --cmake-args -Wno-dev
    source install/setup.bash  

- **launch启动**
    cd /home/yaoyao/Documents/myProjects/ROS2WithSPCK
    source install/setup.bash  
    ros2 launch simpack_control start_simpack_control.launch.py

- **窗口1: 先启动 SIMPACK 仿真器**
  
    cd /home/yaoyao/Documents/myProjects/ROS2WithSPCK
    source install/setup.bash 
    ros2 run simpack_control simpack_node

- **窗口2: 再启动 PIDT4 控制器**
  
    cd /home/yaoyao/Documents/myProjects/ROS2WithSPCK
    source install/setup.bash
    ros2 run simpack_control controller_node

- **rqt 监控**
    - ros2 topic list                                           # 列出当前系统中所有活跃的 ROS 2 主题
    - ros2 run rqt_graph rqt_graph                              # 可视化 ROS 2 节点和主题之间的关系
    - ros2 run rqt_topic rqt_topic                              # 监控 ROS 2 主题消息传递的详细数据
    - ros2 bag play -r 0.5 ./PostAnalysis/rosbag2_2025_XXXX     # ROS Bag 回放



