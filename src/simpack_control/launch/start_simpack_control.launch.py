import launch
from launch import LaunchDescription
from launch.actions import (
    LogInfo, ExecuteProcess, SetLaunchConfiguration,
    TimerAction, RegisterEventHandler
)
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import os
from datetime import datetime

def generate_launch_description():
    # 1) 获取当前日期时间作为记录标识
    current_time = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
    
    # 2) 定义 bag 存储路径（相对路径）
    bag_output_path = os.path.join('PostAnalysis', f'rosbag2_{current_time}')
    
    # 3) 定义终止超时常量
    ros_sigterm_timeout = SetLaunchConfiguration('ros_sigterm_timeout', '5')  # 5秒SIGTERM超时
    ros_sigkill_timeout = SetLaunchConfiguration('ros_sigkill_timeout', '2')  # 2秒SIGKILL超时
    
    # 4) 定义节点：Simpack 主节点（立即启动）
    simpack_node = Node(
        package='simpack_control',
        executable='simpack_node',
        name='simpack_node',
        output='screen',
        emulate_tty=True
    )

    # 5) 新增节点：OnlineEvaluationNode（延时启动）
    #   放在 trkrel_udpsender_node 之前启动
    online_evaluation_node = Node(
        package='simpack_control',
        executable='online_evaluation_node',
        name='online_evaluation_node',
        output='screen',
        emulate_tty=True
    )

    # 6) 其他节点： trkrel_udpsender_node, trkabs_udpsender_node, controller_node
    # trkrel_udpsender_node = Node(
    #     package='simpack_control',
    #     executable='trkrel_udpsender_node',
    #     name='trkrel_udpsender_node',
    #     output='screen',
    #     emulate_tty=True
    # )

    trkabs_udpsender_node = Node(
        package='simpack_control',
        executable='trkabs_udpsender_node',
        name='trkabs_udpsender_node',
        output='screen',
        emulate_tty=True
    )

    controller_node = Node(
        package='simpack_control',
        executable='controller_node',
        name='controller_node',
        output='screen',
        emulate_tty=True
    )
    
    # 7) 定义延时与启动流程(每次加 0.2 s)
    # 立即启动 simpack_node
    
    # 0.2秒后启动 online_evaluation_node
    eval_log_delay = LogInfo(
        msg="Waiting 0.2s before starting the online_evaluation_node..."
    )
    eval_timer_action = TimerAction(
        period=0.2,
        actions=[online_evaluation_node]
    )

    # # 0.4秒后启动 trkrel_udpsender_node
    # udp_log_delay_trkrel = LogInfo(
    #     msg="Waiting 0.4s before starting the trkrel_udpsender_node..."
    # )
    # udp_timer_action_trkrel = TimerAction(
    #     period=0.4,
    #     actions=[trkrel_udpsender_node]
    # )

    # 0.6秒后启动 trkabs_udpsender_node
    udp_log_delay_trkabs = LogInfo(
        msg="Waiting 0.6s before starting the trkabs_udpsender_node..."
    )
    udp_timer_action_trkabs = TimerAction(
        period=0.6,
        actions=[trkabs_udpsender_node]
    )
    
    # 0.8秒后启动 controller_node
    log_delay_controller = LogInfo(
        msg="Waiting 0.8s before starting the controller_node..."
    )
    timer_action_controller = TimerAction(
        period=0.8,
        actions=[controller_node]
    )
    
    # 8) 启动 rosbag 记录进程（可立即或延时启动，这里不做额外延时）
    log_record = LogInfo(
        msg=f"Starting ros2 bag to record all topics in {bag_output_path}..."
    )
    rosbag_recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-o', bag_output_path],
        output='screen',
        name='rosbag_recorder',
        sigterm_timeout=LaunchConfiguration('ros_sigterm_timeout'),
        sigkill_timeout=LaunchConfiguration('ros_sigkill_timeout')
    )

    # 9) 当 simpack_node 退出时的终止流程
    simpack_termination_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=simpack_node,
            on_exit=[
                LogInfo(msg="simpack_node 已退出，开始终止所有节点..."),

                # 终止 online_evaluation_node
                TimerAction(
                    period=0.2,
                    actions=[
                        LogInfo(msg="正在终止 online_evaluation_node..."),
                        ExecuteProcess(
                            cmd=["bash", "-c",
                                 "pkill -SIGTERM -f online_evaluation_node && "
                                 "sleep 0.2 && "
                                 "pkill -SIGKILL -f online_evaluation_node 2>/dev/null || true"
                                ],
                            output='screen'
                        )
                    ]
                ),

                # 终止 controller_node
                TimerAction(
                    period=0.4,
                    actions=[
                        LogInfo(msg="正在终止 controller_node..."),
                        ExecuteProcess(
                            cmd=["bash", "-c",
                                 "pkill -SIGTERM -f controller_node && "
                                 "sleep 0.2 && "
                                 "pkill -SIGKILL -f controller_node 2>/dev/null || true"
                                ],
                            output='screen'
                        )
                    ]
                ),
                
                # # 终止 trkrel_udpsender_node
                # TimerAction(
                #     period=0.6,
                #     actions=[
                #         LogInfo(msg="正在终止 trkrel_udpsender_node..."),
                #         ExecuteProcess(
                #             cmd=["bash", "-c",
                #                  "pkill -SIGTERM -f trkrel_udpsender_node && "
                #                  "sleep 0.2 && "
                #                  "pkill -SIGKILL -f trkrel_udpsender_node 2>/dev/null || true"
                #                 ],
                #             output='screen'
                #         )
                #     ]
                # ),
                
                # 终止 trkabs_udpsender_node
                TimerAction(
                    period=0.8,
                    actions=[
                        LogInfo(msg="正在终止 trkabs_udpsender_node..."),
                        ExecuteProcess(
                            cmd=["bash", "-c",
                                 "pkill -SIGTERM -f trkabs_udpsender_node && "
                                 "sleep 0.2 && "
                                 "pkill -SIGKILL -f trkabs_udpsender_node 2>/dev/null || true"
                                ],
                            output='screen'
                        )
                    ]
                ),
                
                # 终止 rosbag 记录进程
                TimerAction(
                    period=1.0,
                    actions=[
                        LogInfo(msg="正在停止 ros2 bag 记录..."),
                        ExecuteProcess(
                            cmd=["bash", "-c",
                                 "pkill -SIGTERM -f 'ros2 bag record' && "
                                 "sleep 0.2 && "
                                 "pkill -SIGKILL -f 'ros2 bag record' 2>/dev/null || true"
                                ],
                            output='screen'
                        )
                    ]
                ),
                
                # 确保所有进程终止的后备方案
                TimerAction(
                    period=1.2,
                    actions=[
                        LogInfo(msg="正在确保所有 ROS2 进程已终止..."),
                        ExecuteProcess(
                            cmd=["pkill", "-9", "-f", "ros2"],
                            output='screen'
                        )
                    ]
                ),
                
                # 验证所有节点已终止
                TimerAction(
                    period=1.4,
                    actions=[
                        LogInfo(msg="正在验证所有节点已终止..."),
                        ExecuteProcess(
                            cmd=["bash", "-c",
                                 "if pgrep -f 'simpack_node|controller_node|trkrel_udpsender_node|trkabs_udpsender_node|online_evaluation_node' > /dev/null; then "
                                 "  echo '警告：仍有节点未终止，尝试强制终止...'; "
                                 "  pkill -9 -f 'simpack_node|controller_node|trkrel_udpsender_node|trkabs_udpsender_node|online_evaluation_node'; "
                                 "else "
                                 "  echo '所有节点已成功终止'; "
                                 "fi"
                                ],
                            output='screen',
                            on_exit=[
                                LogInfo(msg=f"节点终止流程完成。数据已保存至 {bag_output_path}")
                            ]
                        )
                    ]
                )
            ]
        )
    )
    
    # 10) 当 controller_node 异常退出时的处理
    controller_termination_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_node,
            on_exit=[
                LogInfo(msg="controller_node 异常退出，检查是否需要终止其他节点..."),
                # 检查 simpack_node 是否仍在运行，如不在运行则执行终止流程
                ExecuteProcess(
                    cmd=["bash", "-c",
                         "if ! pgrep -f 'simpack_node' > /dev/null; then "
                         "  echo 'simpack_node 已不在运行，开始终止所有其他节点...'; "
                         # "  pkill -f 'trkrel_udpsender_node'; "
                         "  pkill -f 'trkabs_udpsender_node'; "
                         "  pkill -f 'online_evaluation_node'; "
                         "  pkill -f 'ros2 bag record'; "
                         "fi"
                        ],
                    output='screen'
                )
            ]
        )
    )
    
    # 11) 返回启动描述
    return LaunchDescription([
        # 设置 SIGTERM/SIGKILL 超时
        ros_sigterm_timeout,
        ros_sigkill_timeout,

        # (A) 立即启动 simpack_node
        simpack_node,
        
        # (B) 延时依次启动各节点
        eval_log_delay,
        eval_timer_action,

        # udp_log_delay_trkrel,
        # udp_timer_action_trkrel,

        udp_log_delay_trkabs,
        udp_timer_action_trkabs,

        log_delay_controller,
        timer_action_controller,
        
        # (C) 启动 rosbag 记录
        log_record,
        rosbag_recorder,
        
        # (D) 事件处理器：终止流程
        simpack_termination_handler,
        controller_termination_handler
    ])
