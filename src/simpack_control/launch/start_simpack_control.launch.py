import launch
from launch import LaunchDescription
from launch.actions import LogInfo, ExecuteProcess, SetLaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
import os
from datetime import datetime

def generate_launch_description():
    # 获取当前日期时间作为记录标识
    current_time = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
    
    # 定义bag存储路径（相对路径）
    bag_output_path = os.path.join('PostAnalysis', f'rosbag2_{current_time}')
    
    # 定义终止超时常量
    ros_sigterm_timeout = SetLaunchConfiguration('ros_sigterm_timeout', '5')  # 5秒SIGTERM超时
    ros_sigkill_timeout = SetLaunchConfiguration('ros_sigkill_timeout', '2')  # 2秒SIGKILL超时
    
    # 定义节点
    simpack_node = Node(
        package='simpack_control',
        executable='simpack_node',
        name='simpack_node',
        output='screen',
        emulate_tty=True
    )
    
    # 延时1秒后启动 controller_node
    log_delay = LogInfo(
        msg="Waiting 1 second before starting the controller_node..."
    )
    
    # 控制器节点
    controller_node = Node(
        package='simpack_control',
        executable='controller_node',
        name='controller_node',
        output='screen',
        emulate_tty=True
    )
    
    # 使用 TimerAction 延时1秒后启动控制器
    timer_action = TimerAction(
        period=1.0,  # 延时1秒
        actions=[controller_node]
    )
    
    # 记录开始消息
    log_record = LogInfo(
        msg=f"Starting ros2 bag to record all topics in {bag_output_path}..."
    )
    
    # ROS2 bag 记录
    rosbag_recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-o', bag_output_path],
        output='screen',
        name='rosbag_recorder',
        # 添加参数，使进程在主进程终止时也终止
        sigterm_timeout=LaunchConfiguration('ros_sigterm_timeout'),
        sigkill_timeout=LaunchConfiguration('ros_sigkill_timeout')
    )
    
    # 注册事件处理器：当 simpack_node 退出时，终止 controller_node
    controller_termination_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=simpack_node,
            on_exit=[
                LogInfo(msg="simpack_node 已退出，正在关闭 controller_node..."),
                # 添加延时后终止 controller_node 进程
                TimerAction(
                    period=0.5,  # 延时0.5秒
                    actions=[
                        ExecuteProcess(
                            cmd=["pkill", "-f", "controller_node"],
                            output='screen'
                        )
                    ]
                ),
                # 终止 rosbag 记录进程
                TimerAction(
                    period=1.0,  # 延时1秒，确保所有数据都写入磁盘
                    actions=[
                        LogInfo(msg="正在停止 ros2 bag 记录..."),
                        ExecuteProcess(
                            cmd=["pkill", "-f", "ros2 bag record"],
                            output='screen'
                        )
                    ]
                ),
                # 终止所有剩余的进程，确保干净退出
                TimerAction(
                    period=1.5,  # 最后延时1.5秒
                    actions=[
                        LogInfo(msg="正在终止所有剩余进程..."),
                        ExecuteProcess(
                            cmd=["killall", "-9", "ros2"], 
                            output='screen', 
                            on_exit=[
                                LogInfo(msg=f"所有进程已终止。启动序列完成。数据已保存至 {bag_output_path}")
                            ]
                        )
                    ]
                )
            ]
        )
    )
    
    # 返回启动描述
    return LaunchDescription([
        ros_sigterm_timeout,
        ros_sigkill_timeout,
        simpack_node,
        log_delay,
        timer_action,
        log_record,
        rosbag_recorder,
        controller_termination_handler
    ])