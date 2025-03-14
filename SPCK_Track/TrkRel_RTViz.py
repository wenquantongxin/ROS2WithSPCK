# TrkRel_RTViz.py

"""
1. UDP 接收线程 (UDPReceiverThread)

    通过 Python 的 threading.Thread 实现一个后台守护线程，循环调用 recvfrom 接收来自 SIMPACK 侧通过 UDP 发送过来的二进制数据。
    将数据解析为 77 个双精度数值（double），然后存放到全局的 latest_data、previous_data 里。
    解析成功后通过 data_ready.set() 通知主线程有新数据可供渲染。

2. 全局数据与多线程控制
    使用了全局变量：
    latest_data、previous_data：存放最新和前一帧的数据信息，用于插值和渲染。
    data_ready：threading.Event 对象，用于在接收线程与渲染线程之间同步“数据到达”事件。
    running：用于控制线程的运行/退出。
    frame_time：用于插值时的“模拟时刻”。
    interpolation_enabled：决定是否对前后帧进行插值。
    animation_paused：决定是否暂停动画更新。

3. OpenGL 与 GLUT 渲染流程 (TrainVisualization)
    基于 PyOpenGL + GLUT，负责创建窗口、初始化 OpenGL 状态、设置投影和相机，然后在 display() 回调中进行场景绘制。
    在 animation_timer() 定时器回调中更新动画时间（或模拟时刻），并触发重绘。
    提供了 keyboard()、special_keys() 等输入回调，用于用户交互（如开关轨道显示、切换插值、暂停/恢复动画等）。

4. 车辆与轨道模型绘制

    包含了轨道数据 TrackData（在 tools_RailTransform.py 中定义），并使用 make_transform() 来生成从“里程坐标 + 局部姿态”到全局坐标的变换矩阵。
    
    车辆模型分为：
        车体（Car Body）
        两个转向架（Bogie）
        四个轮对（Wheelset）
    分别在 3D 场景中对每个部件进行几何绘制、姿态变换（包括平移 + 旋转），以达到动态显示车辆姿态的效果。
    还绘制了轨道中线、左右钢轨等辅助线。

5. 帧间插值
    如果开启插值(interpolation_enabled=True)，则会在渲染时根据 previous_data 与 latest_data 之间的差值
    线性插值出一个中间状态，以实现更平滑的动画效果。

"""

import socket
import struct
import time
import numpy as np
import os
import math
import threading
import queue
import ctypes
import sys
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

from tools_RailTransform import (TrackData, make_transform, lerp, angle_lerp)

# 全局队列：用于存放接收到的数据
data_queue = queue.Queue(maxsize=10)  # 限制队列大小，避免无限增长
latest_data = None   # 最新的数据帧
previous_data = None # 上一个数据帧，用于插值
data_ready = threading.Event()  # 用于通知渲染线程“数据已更新”
running = True       # 控制线程退出
frame_time = 0.0     # 当前插值后的模拟时刻
last_update_time = 0.0  # 上一次更新帧的时间

# 动画控制
TARGET_FPS = 60      # 目标帧率
FRAME_TIME = 1.0 / TARGET_FPS  # 每帧耗时（秒）
interpolation_enabled = True   # 是否启用插值
animation_paused = False       # 是否暂停动画
use_received_time = True       # 是否使用接收的数据中带的模拟时间


# =========================== 1) UDP接收线程 ===========================
class UDPReceiverThread(threading.Thread):
    """
    一个基于Python线程的UDP接收线程，用于从指定端口接收来自ROS2的二进制数据包。
    数据包被解析为77个double，并存储到latest_data中，以供OpenGL渲染使用。
    """
    def __init__(self, ip="0.0.0.0", port=10088, port_retry=True, max_retries=10):
        threading.Thread.__init__(self)
        self.daemon = True  # 设置为守护线程，主线程退出时该线程也退出
        
        # UDP相关设置
        self.UDP_IP = ip
        self.UDP_PORT = port
        self.port_retry = port_retry
        self.max_retries = max_retries
        
        # 数据格式相关：每次预期收到77个 double（8字节）
        self.EXPECTED_LEN = 77
        self.EXPECTED_BYTES = self.EXPECTED_LEN * 8
        self.fmt = "<" + "d" * self.EXPECTED_LEN    # 小端格式的 77 个 double
        
        # 发送端对每个字段的名称，用于后续调试或记录
        self.col_names = [
            "Time", "y_spcktime", "y_cb_vx", "y_cb_x", "y_cb_y", "y_cb_z",
            "y_cb_roll", "y_cb_yaw", "y_cb_pitch", "y_w01_rotw", "y_w02_rotw",
            "y_w03_rotw", "y_w04_rotw", "y_w05_rotw", "y_w06_rotw", "y_w07_rotw",
            "y_w08_rotw", "y_f01_x", "y_f01_y", "y_f01_z", "y_f01_roll", 
            "y_f01_yaw", "y_f01_pitch", "y_f02_x", "y_f02_y", "y_f02_z", 
            "y_f02_roll", "y_f02_yaw", "y_f02_pitch", "y_ws01_x", "y_ws01_y", 
            "y_ws01_z", "y_ws01_roll", "y_ws01_yaw", "y_ws01_pitch", "y_ws02_x", 
            "y_ws02_y", "y_ws02_z", "y_ws02_roll", "y_ws02_yaw", "y_ws02_pitch", 
            "y_ws03_x", "y_ws03_y", "y_ws03_z", "y_ws03_roll", "y_ws03_yaw", 
            "y_ws03_pitch", "y_ws04_x", "y_ws04_y", "y_ws04_z", "y_ws04_roll", 
            "y_ws04_yaw", "y_ws04_pitch", "y_w01_rota", "y_w02_rota", "y_w03_rota",
            "y_w04_rota", "y_w05_rota", "y_w06_rota", "y_w07_rota", "y_w08_rota",
            "y_bar01_pitch", "y_bar02_pitch", "y_bar03_pitch", "y_bar04_pitch",
            "y_bar05_pitch", "y_bar06_pitch", "y_bar07_pitch", "y_bar08_pitch",
            "y_ws01_vy", "y_ws02_vy", "y_ws03_vy", "y_ws04_vy", 
            "y_ws01_vyaw", "y_ws02_vyaw", "y_ws03_vyaw", "y_ws04_vyaw"
        ]
        
        # 接收统计
        self.data_count = 0
        self.filtered_count = 0
        self.last_spcktime = None
        
        # Create UDP socket with retry capability
        self.create_socket()
        
        print(f"UDP receiver thread initialized, listening on {self.UDP_IP}:{self.UDP_PORT}")

    def create_socket(self):
        """创建UDP套接字并尝试绑定到指定端口，如果端口被占用则尝试递增端口号"""
        retry_count = 0
        current_port = self.UDP_PORT
        
        while retry_count < self.max_retries:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                
                # 允许端口复用
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                
                # 绑定端口
                self.sock.bind((self.UDP_IP, current_port))
                self.sock.settimeout(0.01)  # 10ms超时，保证循环能及时响应
                
                self.UDP_PORT = current_port
                print(f"成功绑定到端口 {self.UDP_PORT}")
                return
                
            except OSError as e:
                if not self.port_retry:
                    raise e
                
                # # 如果端口被占用，递增端口继续尝试
                # retry_count += 1
                # current_port = self.UDP_PORT + retry_count
                # print(f"Port {self.UDP_PORT + retry_count - 1} is in use, trying {current_port}...")
        
        # If we've exhausted all retries, raise an exception
        raise RuntimeError(f"在尝试 {self.max_retries} 次后仍无法绑定UDP端口")

    def run(self):
        """线程主循环：不断接收UDP数据，并解析后存入latest_data以备渲染"""
        global latest_data, previous_data, running, data_ready
        
        print("UDP接收线程开始运行...")
        
        while running:
            try:

                data, addr = self.sock.recvfrom(65535)
                
                # 判断数据长度是否符合预期
                if len(data) != self.EXPECTED_BYTES:
                    print(f"[WARNING] Received data length is {len(data)} bytes, expected {self.EXPECTED_BYTES} -> skipping")
                    continue

                # 按指定格式解析
                values = struct.unpack(self.fmt, data)
                
                self.data_count += 1
                
                # 根据 y_spcktime 来判断是否重复
                current_spcktime = values[1]
                if self.last_spcktime is not None and current_spcktime == self.last_spcktime:
                    self.filtered_count += 1
                    continue
                
                self.last_spcktime = current_spcktime
                
                # 每接收 10000 包时打印一次提示
                if self.data_count % 10000 == 0:
                    print(f"[INFO] Received {self.data_count} packets, current sim_time: {values[0]:.6f}")
                
                # Update data for rendering
                previous_data = latest_data     # Store previous frame for interpolation
                latest_data = values            # Update latest data
                data_ready.set()                # 通知渲染线程“新数据可用”
                
            except socket.timeout:
                # 超时后继续循环，以便可以检测running状态
                pass
            except Exception as e:
                print(f"[ERROR] UDP接收线程异常: {e}")
                if not running:
                    break
        
        print("UDP接收线程退出")
        self.sock.close()

# =========================== 2) 数据插值 ===========================

def interpolate_frames(frame1, frame2, t):
    """
    在frame1和frame2两个数据帧之间做插值，t在[0.0,1.0]之间。
    针对位置索引做线性插值，对角度索引做角度插值，其余字段直接用frame2的值。
    """
    if frame1 is None or frame2 is None:
        return frame2 if frame2 is not None else frame1

    interpolated = np.zeros(len(frame1), dtype=float)
    
    # 模拟时间(索引0)用线性插值
    interpolated[0] = lerp(frame1[0], frame2[0], t)
    
    # 需要做线性插值的位置索引
    position_indices = [3, 4, 5, 17, 18, 19, 23, 24, 25, 29, 30, 31, 35, 36, 37, 41, 42, 43, 47, 48, 49]
    for idx in position_indices:
        interpolated[idx] = lerp(frame1[idx], frame2[idx], t)
    
    # 需要做角度插值的索引
    angle_indices = [6, 7, 8, 20, 21, 22, 26, 27, 28, 32, 33, 34, 38, 39, 40, 44, 45, 46, 50, 51, 52]
    for idx in angle_indices:
        interpolated[idx] = angle_lerp(frame1[idx], frame2[idx], t)
    
    # Copy other values directly
    for i in range(len(frame1)):
        if i not in position_indices and i not in angle_indices and i != 0:
            interpolated[i] = frame2[i]  # Use latest values for non-interpolated fields
    
    return interpolated

# =========================== 3) 车辆几何模型类 ===========================
class TrainModels:
    """
    用于生成并管理车轴、车轮、车体、转向架等几何模型的类。
    可以选择使用OpenGL显示列表提高渲染性能。
    """
    def __init__(self):
        # 轮对和车轴基本尺寸
        self.axle_length = 2.0
        self.axle_radius = 0.065
        self.wheel_radius = 0.43
        self.wheel_thickness = 0.04
        self.wheel_offset = 0.7175  # 车轮中心相对车轴中心的偏移距离
        
        # 车体和转向架尺寸
        self.car_body_length = 25.0
        self.car_body_width = 3.0
        self.car_body_height = 3.0
        
        self.bogie_length = 3.0
        self.bogie_width = 2.5
        self.bogie_height = 0.5
        
        # 先创建基本几何顶点
        self.create_models()
        
        # 显示列表ID，初始为0，表示尚未建立
        self.axle_list = 0
        self.wheel_list = 0
        self.car_body_list = 0
        self.bogie_list = 0
    
    def create_cylinder(self, length, radius, sides=20):
        """生成圆柱的顶点数据（圆柱沿Y轴或X轴）"""
        vertices = []
        
        # 圆的周向分成sides段
        for i in range(sides+1):
            theta = i * (2.0 * np.pi / sides)
            x = radius * np.cos(theta)
            z = radius * np.sin(theta)
            
            # 两端点 + 两侧点
            vertices.append([-length/2, 0, 0])
            vertices.append([length/2, 0, 0])
            vertices.append([length/2, x, z])
            vertices.append([-length/2, x, z])
        
        return np.array(vertices)
    
    def create_wheel(self, thickness, radius, sides=20):
        """生成车轮的顶点数据（车轮沿X轴）"""
        vertices = []
        
        # Wheel is a cylinder along X-axis
        for i in range(sides+1):
            theta = i * (2.0 * np.pi / sides)
            y = radius * np.cos(theta)
            z = radius * np.sin(theta)

            vertices.append([-thickness/2, 0, 0])
            vertices.append([thickness/2, 0, 0])
            vertices.append([thickness/2, y, z])
            vertices.append([-thickness/2, y, z])
        
        return np.array(vertices)
    
    def create_box(self, length, width, height):
        """生成一个长方体的顶点数据，高度方向上关于xy平面对称"""
        vertices = [
            # 前面 (x=length/2)
            [length/2, -width/2, -height/2], [length/2, width/2, -height/2], [length/2, width/2, height/2], [length/2, -width/2, height/2],
            # 后面 (x=-length/2)
            [-length/2, -width/2, -height/2], [-length/2, width/2, -height/2], [-length/2, width/2, height/2], [-length/2, -width/2, height/2],
            # 右侧 (y=width/2)
            [-length/2, width/2, -height/2], [length/2, width/2, -height/2], [length/2, width/2, height/2], [-length/2, width/2, height/2],
            # 左侧 (y=-width/2)
            [-length/2, -width/2, -height/2], [length/2, -width/2, -height/2], [length/2, -width/2, height/2], [-length/2, -width/2, height/2],
            # 顶面 (z=height/2)
            [-length/2, -width/2, height/2], [-length/2, width/2, height/2], [length/2, width/2, height/2], [length/2, -width/2, height/2],
            # 底面 (z=-height/2)
            [-length/2, -width/2, -height/2], [-length/2, width/2, -height/2], [length/2, width/2, -height/2], [length/2, -width/2, -height/2]
        ]
        return np.array(vertices)
    
    def create_models(self):
        """调用创建函数，生成车轴、车轮、车体、转向架的几何顶点数据"""
        # 车轴模型 (cylinder along Y-axis)
        self.axle_vertices = self.create_cylinder(self.axle_length, self.axle_radius)
        
        # 车轮模型 (cylinder along X-axis)
        self.wheel_vertices = self.create_wheel(self.wheel_thickness, self.wheel_radius)
        
        # 车体模型 (box)
        self.car_body_vertices = self.create_box(
            self.car_body_length, self.car_body_width, self.car_body_height)
        
        # 转向架模型 (box)
        self.bogie_vertices = self.create_box(
            self.bogie_length, self.bogie_width, self.bogie_height)
    
    def create_display_lists(self):
        """创建OpenGL显示列表，以提升渲染效率（须在OpenGL初始化后调用）"""
        try:
            # 车轴 display list
            self.axle_list = glGenLists(1)
            glNewList(self.axle_list, GL_COMPILE)
            self.render_axle()
            glEndList()
            
            # 车轮 display list
            self.wheel_list = glGenLists(1)
            glNewList(self.wheel_list, GL_COMPILE)
            self.render_wheel()
            glEndList()
            
            # 车体 display list
            self.car_body_list = glGenLists(1)
            glNewList(self.car_body_list, GL_COMPILE)
            self.render_car_body()
            glEndList()
            
            # 转向架 display list
            self.bogie_list = glGenLists(1)
            glNewList(self.bogie_list, GL_COMPILE)
            self.render_bogie()
            glEndList()
            
            print("车辆几何模型的显示列表已成功创建")
        except Exception as e:
            print(f"创建显示列表时出现错误: {e}")
            # Fallback: Use immediate mode rendering instead of display lists
            self.axle_list = 0
            self.wheel_list = 0
            self.car_body_list = 0
            self.bogie_list = 0
    
    def render_axle(self):
        """使用立即模式绘制车轴"""
        glColor3f(1.0, 0.0, 1.0)  # Magenta
        glBegin(GL_QUADS)
        for i in range(0, len(self.axle_vertices), 4):
            for j in range(4):
                glVertex3fv(self.axle_vertices[i+j])
        glEnd()
    
    def render_wheel(self):
        """使用立即模式绘制车轮"""
        glColor3f(0.0, 1.0, 1.0)  # Cyan
        glBegin(GL_QUADS)
        for i in range(0, len(self.wheel_vertices), 4):
            for j in range(4):
                glVertex3fv(self.wheel_vertices[i+j])
        glEnd()
    
    def render_car_body(self):
        """使用立即模式绘制车体"""
        glColor4f(1.0, 0.5, 0.0, 0.2)  # Semi-transparent orange
        glBegin(GL_QUADS)
        for i in range(0, len(self.car_body_vertices), 4):
            for j in range(4):
                glVertex3fv(self.car_body_vertices[i+j])
        glEnd()
    
    def render_bogie(self):
        """使用立即模式绘制转向架"""
        glColor4f(0.0, 0.8, 0.0, 0.3)  # Semi-transparent green
        glBegin(GL_QUADS)
        for i in range(0, len(self.bogie_vertices), 4):
            for j in range(4):
                glVertex3fv(self.bogie_vertices[i+j])
        glEnd()
    
    def draw_wheelset_immediate(self, T_W2G):
        """
        以立即模式绘制一个车轴 + 两个车轮
        T_W2G: 车轮组局部坐标系到全局坐标系的变换矩阵

        绕Z轴旋转90度以使车轴与轨道方向一致
        """
        axle_rotation = np.array([
            [0, -1, 0, 0],  # Rotate 90 degrees around Z-axis
            [1, 0, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        # Apply the rotation to the axle's transformation
        T_axle = T_W2G @ axle_rotation
        
        # 绘制车轴 (now properly oriented)
        glPushMatrix()
        glMultMatrixf(T_axle.T.flatten())
        self.render_axle()
        glPopMatrix()
        
        # 左轮
        glPushMatrix()
        T_wheel_left = np.eye(4)
        T_wheel_left[1, 3] = self.wheel_offset
        rotation = np.array([
            [0, 0, 1, 0],
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 0, 1]
        ])
        T_final = T_W2G @ T_wheel_left @ rotation
        glMultMatrixf(T_final.T.flatten())
        self.render_wheel()
        glPopMatrix()
        
        # 右轮
        glPushMatrix()
        T_wheel_right = np.eye(4)
        T_wheel_right[1, 3] = -self.wheel_offset
        T_final = T_W2G @ T_wheel_right @ rotation
        glMultMatrixf(T_final.T.flatten())
        self.render_wheel()
        glPopMatrix()
    
    def draw_wheelset(self, T_W2G):
        """
        绘制一个车轴 + 两个车轮
        如果已创建显示列表则使用显示列表，否则使用立即模式
        """
        if self.axle_list > 0 and self.wheel_list > 0:
            # Use display lists if available
            
            # Rotate axle by 90 degrees around Z-axis so it's perpendicular to track
            # Create a rotation matrix for the axle
            axle_rotation = np.array([
                [0, -1, 0, 0],  # Rotate 90 degrees around Z-axis
                [1, 0, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ])
            
            # Apply the rotation to the axle's transformation
            T_axle = T_W2G @ axle_rotation
            
            # 车轴 (now properly oriented)
            glPushMatrix()
            glMultMatrixf(T_axle.T.flatten())
            glCallList(self.axle_list)
            glPopMatrix()
            
            # 左轮
            glPushMatrix()
            T_wheel_left = np.eye(4)
            T_wheel_left[1, 3] = self.wheel_offset
            rotation = np.array([
                [0, 0, 1, 0],
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 0, 1]
            ])
            T_final = T_W2G @ T_wheel_left @ rotation
            glMultMatrixf(T_final.T.flatten())
            glCallList(self.wheel_list)
            glPopMatrix()
            
            # 右轮
            glPushMatrix()
            T_wheel_right = np.eye(4)
            T_wheel_right[1, 3] = -self.wheel_offset
            T_final = T_W2G @ T_wheel_right @ rotation
            glMultMatrixf(T_final.T.flatten())
            glCallList(self.wheel_list)
            glPopMatrix()
        else:
            # 如果无法使用显示列表，则退化到立即模式
            self.draw_wheelset_immediate(T_W2G)
    
    def draw_box_immediate(self, T_B2G, box_vertices):
        """
        立即模式绘制一个长方体（用于车体或转向架）
        """
        glPushMatrix()
        glMultMatrixf(T_B2G.T.flatten())
        
        # Enable blending for transparency
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        
        if box_vertices is self.car_body_vertices:
            self.render_car_body()
        else:
            self.render_bogie()

        glDisable(GL_BLEND)
        glPopMatrix()
    
    def draw_box(self, T_B2G, display_list, box_type="bogie"):
        """
        绘制一个长方体（车体或转向架）
        T_B2G: 该长方体局部坐标系到全局坐标系的变换
        display_list: 如果有对应的显示列表，就直接调用
        box_type: "bogie" 或 "car_body"
        """
        if display_list > 0:
            # Use display lists if available
            glPushMatrix()
            glMultMatrixf(T_B2G.T.flatten())
            
            # Enable blending for transparency
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
            
            glCallList(display_list)
            
            # Disable blending
            glDisable(GL_BLEND)
            glPopMatrix()
        else:
            # Fallback to immediate mode rendering
            if box_type == "bogie":
                self.draw_box_immediate(T_B2G, self.bogie_vertices)
            else:
                self.draw_box_immediate(T_B2G, self.car_body_vertices)

# =========================== 4) OpenGL可视化类 ===========================
class TrainVisualization:
    """
    封装了OpenGL和GLUT的可视化窗口，包含轨道绘制、相机控制、车辆渲染等功能。
    """
    def __init__(self, window_width=1200, window_height=800):
        self.window_width = window_width
        self.window_height = window_height
        
        # 初始化OpenGL
        self.init_opengl()
        
        # 加载轨道数据
        self.track = TrackData()
        
        # 创建车辆模型
        self.train_models = TrainModels()
        
        # 相机初始设置（将Z方向翻转）
        self.camera_distance = 30.0
        self.camera_elevation = -30.0
        self.camera_azimuth = 120.0
        self.camera_target = [0.0, 0.0, 0.0]
        self.auto_camera = True
        self.camera_up_vector = [0, 0, -1]
        
        # 显示选项
        self.show_track = True
        self.show_rails = True
        self.show_fps = True
        
        # 帧率统计
        self.frame_count = 0
        self.fps = 0.0
        self.last_fps_time = time.time()
        
        # 编译轨道显示列表
        self.compile_track_display_list()
        self.train_models.create_display_lists()
        
        # 动画定时控制
        self.start_time = time.time()
        self.last_time = self.start_time
        
        glutTimerFunc(int(FRAME_TIME * 1000), self.animation_timer, 0)
    
    def init_opengl(self):
        """初始化OpenGL渲染环境和GLUT窗口"""
        glutInit(sys.argv)
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
        glutInitWindowSize(self.window_width, self.window_height)
        glutCreateWindow(b"Rel-Coord Train Visualization")
        
        # 注册回调函数
        glutDisplayFunc(self.display)
        glutReshapeFunc(self.reshape)
        glutKeyboardFunc(self.keyboard)
        glutSpecialFunc(self.special_keys)
        glutIdleFunc(self.idle)
        
        # 背景色
        glClearColor(0.9, 0.9, 1.0, 1.0)  # Light blue background
        
        # 启用深度测试
        glEnable(GL_DEPTH_TEST)
        
        # 启用光照
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        
        # 设置光源（方向光）
        light_position = [100.0, 100.0, -100.0, 0.0]  # Directional light (z-inverted)
        glLightfv(GL_LIGHT0, GL_POSITION, light_position)
        
        # 设置材质属性
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
    
    def animation_timer(self, value):
        """动画定时器回调函数，用于更新frame_time并触发重绘"""
        global frame_time, last_update_time
        
        if not animation_paused:
            # Get current time
            current_time = time.time()
            elapsed = current_time - last_update_time
            
            # When using simulation time from UDP
            if use_received_time and latest_data is not None and previous_data is not None:
                # 根据接收到的数据模拟时间做插值
                sim_time_start = previous_data[0]
                sim_time_end = latest_data[0]
                sim_time_range = sim_time_end - sim_time_start
                
                if sim_time_range > 0:
                    # 计算插值系数，映射实际经过的时间到模拟时间差
                    t = min(1.0, elapsed / (FRAME_TIME * 10)) 
                    frame_time = sim_time_start + t * sim_time_range
                else:
                    frame_time = sim_time_end
            else:
                # 如果不使用接收端的模拟时间，则直接按帧计时推进
                frame_time += FRAME_TIME

            glutPostRedisplay()
            
            # 计算 FPS
            self.frame_count += 1
            fps_elapsed = current_time - self.last_fps_time
            if fps_elapsed >= 1.0:  # Update FPS once per second
                self.fps = self.frame_count / fps_elapsed
                self.frame_count = 0
                self.last_fps_time = current_time
        
        # 下一帧
        glutTimerFunc(int(FRAME_TIME * 1000), self.animation_timer, 0)
    
    def compile_track_display_list(self):
        """预先编译轨道的显示列表，提高渲染效率"""
        try:
            # 轨道中心线
            self.track_center_list = glGenLists(1)
            glNewList(self.track_center_list, GL_COMPILE)
            glDisable(GL_LIGHTING)
            glColor3f(0.0, 0.0, 0.0)  # Black
            glBegin(GL_LINE_STRIP)
            for i in range(len(self.track.xvals)):
                glVertex3f(self.track.xvals[i], self.track.yvals[i], self.track.zvals[i])
            glEnd()
            glEnable(GL_LIGHTING)
            glEndList()
            
            # 左右钢轨
            self.rails_list = glGenLists(1)
            glNewList(self.rails_list, GL_COMPILE)
            glDisable(GL_LIGHTING)
            
            # 左轨 (red)
            glColor3f(0.8, 0.0, 0.0)
            glBegin(GL_LINE_STRIP)
            for i in range(len(self.track.left_rail)):
                glVertex3f(self.track.left_rail[i, 0], 
                           self.track.left_rail[i, 1], 
                           self.track.left_rail[i, 2])
            glEnd()
            
            # 右轨 (blue)
            glColor3f(0.0, 0.0, 0.8)
            glBegin(GL_LINE_STRIP)
            for i in range(len(self.track.right_rail)):
                glVertex3f(self.track.right_rail[i, 0], 
                           self.track.right_rail[i, 1], 
                           self.track.right_rail[i, 2])
            glEnd()
            
            glEnable(GL_LIGHTING)
            glEndList()
            
            print("轨道显示列表创建成功")
        except Exception as e:
            print(f"创建轨道显示列表时出现错误: {e}")
            # Fallback: set these to 0 to use immediate mode rendering instead
            self.track_center_list = 0
            self.rails_list = 0
    
    def set_camera(self):
        """根据相机相关参数设置视角"""
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        
        # 计算相机视角
        x = self.camera_target[0] + self.camera_distance * np.cos(np.radians(self.camera_elevation)) * np.cos(np.radians(self.camera_azimuth))
        y = self.camera_target[1] + self.camera_distance * np.cos(np.radians(self.camera_elevation)) * np.sin(np.radians(self.camera_azimuth))
        z = self.camera_target[2] + self.camera_distance * np.sin(np.radians(self.camera_elevation))
        
        # 旋转 180° 以适应 Z-down 坐标系
        gluLookAt(x, y, z,  # Camera position
                  self.camera_target[0], self.camera_target[1], self.camera_target[2],  # Target point
                  self.camera_up_vector[0], self.camera_up_vector[1], self.camera_up_vector[2])  # Up vector (Z-down)
    
    def reshape(self, width, height):
        """窗口大小变化时的回调函数"""
        self.window_width = width
        self.window_height = height
        
        # Update viewport
        glViewport(0, 0, width, height)
        
        # Set projection matrix
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, float(width)/float(height), 0.1, 1000.0)
    
    def keyboard(self, key, x, y):
        """键盘输入的回调函数"""
        global running, animation_paused, interpolation_enabled, use_received_time
        
        if key == b'\x1b':  # ESC key
            running = False
            glutLeaveMainLoop()
        elif key == b't':  # Toggle track display
            self.show_track = not self.show_track
        elif key == b'r':  # Toggle rails display
            self.show_rails = not self.show_rails
        elif key == b'f':  # Toggle FPS display
            self.show_fps = not self.show_fps
        elif key == b'p':  # Pause/Resume animation
            animation_paused = not animation_paused
        elif key == b'i':  # Toggle interpolation
            interpolation_enabled = not interpolation_enabled
        elif key == b's':  # Toggle between simulation time and real time
            use_received_time = not use_received_time
        elif key == b'c':  # Toggle camera auto-follow
            self.auto_camera = not self.auto_camera
        elif key == b'v':  # Toggle Z-axis orientation (up/down)
            self.camera_up_vector = [0, 0, -self.camera_up_vector[2]]  # Flip Z direction
            self.camera_elevation = -self.camera_elevation  # Invert elevation
    
    def special_keys(self, key, x, y):
        """特殊键（方向键、PageUp/Down等）的回调函数，用于调整相机"""
        # Camera control
        if key == GLUT_KEY_UP:
            self.camera_elevation += 5.0
        elif key == GLUT_KEY_DOWN:
            self.camera_elevation -= 5.0
        elif key == GLUT_KEY_LEFT:
            self.camera_azimuth += 5.0
        elif key == GLUT_KEY_RIGHT:
            self.camera_azimuth -= 5.0
        elif key == GLUT_KEY_PAGE_UP:
            self.camera_distance -= 2.0
        elif key == GLUT_KEY_PAGE_DOWN:
            self.camera_distance += 2.0
        
        # Limit elevation range
        self.camera_elevation = max(-85.0, min(85.0, self.camera_elevation))
        
        # Limit camera distance
        self.camera_distance = max(5.0, self.camera_distance)
        
        glutPostRedisplay()
    
    def idle(self):
        """空闲回调函数，用于数据到达后的更新"""
        global data_ready, last_update_time
        
        # Check if new data is available
        if data_ready.is_set():
            data_ready.clear()
            last_update_time = time.time()  # Reset interpolation time
    
    def render_track_immediate(self):
        """如果无法使用显示列表，则使用立即模式绘制轨道"""
        # Track centerline
        glDisable(GL_LIGHTING)
        glColor3f(0.0, 0.0, 0.0)  # Black
        glBegin(GL_LINE_STRIP)
        for i in range(len(self.track.xvals)):
            glVertex3f(self.track.xvals[i], self.track.yvals[i], self.track.zvals[i])
        glEnd()
        
        # Left rail (red)
        glColor3f(0.8, 0.0, 0.0)
        glBegin(GL_LINE_STRIP)
        for i in range(len(self.track.left_rail)):
            glVertex3f(self.track.left_rail[i, 0], 
                       self.track.left_rail[i, 1], 
                       self.track.left_rail[i, 2])
        glEnd()
        
        # Right rail (blue)
        glColor3f(0.0, 0.0, 0.8)
        glBegin(GL_LINE_STRIP)
        for i in range(len(self.track.right_rail)):
            glVertex3f(self.track.right_rail[i, 0], 
                       self.track.right_rail[i, 1], 
                       self.track.right_rail[i, 2])
        glEnd()
        
        glEnable(GL_LIGHTING)
    
    def display(self):
        """GLUT的绘制回调函数"""
        global latest_data, previous_data, frame_time
        
        # Clear buffers
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        
        # Get interpolated data frame
        current_data = None
        
        if latest_data is not None:
            if interpolation_enabled and previous_data is not None:
                # Calculate interpolation factor
                sim_time_start = previous_data[0]
                sim_time_end = latest_data[0]
                
                if sim_time_end > sim_time_start:
                    # Calculate t factor (0.0-1.0) for interpolation
                    t = (frame_time - sim_time_start) / (sim_time_end - sim_time_start)
                    t = max(0.0, min(1.0, t))  # Clamp between 0 and 1
                    
                    # Get interpolated frame
                    current_data = interpolate_frames(previous_data, latest_data, t)
                else:
                    current_data = latest_data
            else:
                current_data = latest_data
        
        # Set camera
        self.set_camera()
        
        # 绘制轨道
        if self.show_track:
            if self.track_center_list > 0:
                glCallList(self.track_center_list)
            else:
                self.render_track_immediate()
        
        # Draw rails
        if self.show_rails:
            if self.rails_list > 0:
                glCallList(self.rails_list)
            else:
                self.render_track_immediate()
        
        # 如果有数据，则绘制车辆
        if current_data is not None:
            # 自动跟随摄像机
            if self.auto_camera:
                s_cb = current_data[3]  # y_cb_x
                y_cb = current_data[4]  # y_cb_y
                z_cb = current_data[5]  # y_cb_z
                
                # Get global position on track
                X_T, Y_T, Z_T, _, _, _ = self.track.get_track_pose(s_cb)
                
                # Update camera target
                self.camera_target = [X_T, Y_T, Z_T]
            
            # Render train with current data
            self.render_train(current_data)
        
        # Display help text on screen
        self.display_help_text()
        
        # Swap buffers
        glutSwapBuffers()
    
    def render_train(self, data):
        """根据数据绘制整列车，包括4个轮对、2个转向架和车体"""
        # 先绘制4个轮对
        for ws_id in range(1, 5):
            # Wheelset position indices
            s_idx = 29 + (ws_id-1) * 6    # y_ws0X_x
            y_idx = 30 + (ws_id-1) * 6    # y_ws0X_y
            z_idx = 31 + (ws_id-1) * 6    # y_ws0X_z
            r_idx = 32 + (ws_id-1) * 6    # y_ws0X_roll
            yw_idx = 33 + (ws_id-1) * 6   # y_ws0X_yaw
            p_idx = 34 + (ws_id-1) * 6    # y_ws0X_pitch
            
            s_ = data[s_idx]
            y_ = data[y_idx]
            z_ = data[z_idx]
            r_ = data[r_idx]
            yw_ = data[yw_idx]
            p_ = data[p_idx]
            
            # Get global position and orientation on track
            X_T, Y_T, Z_T, yaw_T, pitch_T, roll_T = self.track.get_track_pose(s_)
            
            # Calculate transformation matrices
            T_T2G = make_transform(yaw_T, pitch_T, roll_T, X_T, Y_T, Z_T)
            T_W2T = make_transform(yw_, p_, r_, 0.0, y_, z_)
            T_W2G = T_T2G @ T_W2T
            
            # Draw wheelset
            self.train_models.draw_wheelset(T_W2G)
        
        # 再绘制2个转向架
        for bg_id in range(1, 3):
            # Bogie position indices
            s_idx = 17 + (bg_id-1) * 6    # y_f0X_x
            y_idx = 18 + (bg_id-1) * 6    # y_f0X_y
            z_idx = 19 + (bg_id-1) * 6    # y_f0X_z
            r_idx = 20 + (bg_id-1) * 6    # y_f0X_roll
            yw_idx = 21 + (bg_id-1) * 6   # y_f0X_yaw
            p_idx = 22 + (bg_id-1) * 6    # y_f0X_pitch
            
            s_bg = data[s_idx]
            y_bg = data[y_idx]
            z_bg = data[z_idx]
            r_bg = data[r_idx]
            yw_bg = data[yw_idx]
            p_bg = data[p_idx]
            
            # Get global position and orientation on track
            X_T, Y_T, Z_T, yaw_T, pitch_T, roll_T = self.track.get_track_pose(s_bg)
            
            # Calculate transformation matrices
            T_T2G = make_transform(yaw_T, pitch_T, roll_T, X_T, Y_T, Z_T)
            T_bg2T = make_transform(yw_bg, p_bg, r_bg, 0.0, y_bg, z_bg)
            T_bg2G = T_T2G @ T_bg2T
            
            # Draw bogie
            self.train_models.draw_box(T_bg2G, self.train_models.bogie_list, "bogie")
        
        # 最后绘制车体
        s_cb = data[3]    # y_cb_x
        y_cb = data[4]    # y_cb_y
        z_cb = data[5]    # y_cb_z
        r_cb = data[6]    # y_cb_roll
        yw_cb = data[7]   # y_cb_yaw
        p_cb = data[8]    # y_cb_pitch
        
        # Get global position and orientation on track
        X_T, Y_T, Z_T, yaw_T, pitch_T, roll_T = self.track.get_track_pose(s_cb)
        
        # Calculate transformation matrices
        T_T2G = make_transform(yaw_T, pitch_T, roll_T, X_T, Y_T, Z_T)
        T_cb2T = make_transform(yw_cb, p_cb, r_cb, 0.0, y_cb, z_cb)
        T_cb2G = T_T2G @ T_cb2T
        
        # No More Needed --- If 180-degree flip is needed
        cb_hc = -1.8 # 车体几何高度
        T_flipCar = make_transform(0, 0, 0, 0, 0, cb_hc) # T_flipCar = make_transform(0, 0, math.pi, 0, 0, cb_hc)
        T_cb2G = T_cb2G @ T_flipCar
        
        # Draw car body
        self.train_models.draw_box(T_cb2G, self.train_models.car_body_list, "car_body")
    
    def display_help_text(self):
        """在屏幕上显示帮助信息、FPS、插值状态等"""
        # Disable lighting and depth testing for 2D text
        glDisable(GL_LIGHTING)
        glDisable(GL_DEPTH_TEST)
        
        # Switch to orthographic projection
        glMatrixMode(GL_PROJECTION)
        glPushMatrix()
        glLoadIdentity()
        glOrtho(0, self.window_width, 0, self.window_height, -1, 1)
        
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        glLoadIdentity()
        
        # 文字颜色（黑）
        glColor3f(0.0, 0.0, 0.0)  # Black text
        
        # 以下文字为界面帮助信息和状态提示，按需保留英文或自行修改
        self.render_string(10, self.window_height - 20, b"Controls: Arrow keys - Rotate view, Page Up/Down - Zoom")
        self.render_string(10, self.window_height - 40, b"         T - Toggle track, R - Toggle rails, ESC - Exit")
        self.render_string(10, self.window_height - 60, b"         P - Pause/Resume, I - Toggle interpolation")
        self.render_string(10, self.window_height - 80, b"         C - Toggle camera auto-follow, V - Flip Z-axis")
        
        # Status information
        y_pos = self.window_height - 120
        
        # Display animation status
        status_text = f"Paused: {'Yes' if animation_paused else 'No'}, " \
                     f"Interpolation: {'On' if interpolation_enabled else 'Off'}, " \
                     f"Camera mode: {'Auto-follow' if self.auto_camera else 'Manual'}"
        self.render_string(10, y_pos, status_text.encode('utf-8'))
        y_pos -= 20
        
        # Display current frame time
        if latest_data is not None:
            frame_info = f"Sim time: {frame_time:.3f}s, Using: {'Sim Time' if use_received_time else 'Real Time'}"
            self.render_string(10, y_pos, frame_info.encode('utf-8'))
            y_pos -= 20
        
        # Display FPS if enabled
        if self.show_fps:
            fps_text = f"FPS: {self.fps:.1f}"
            self.render_string(10, y_pos, fps_text.encode('utf-8'))
        
        # Restore projection matrix
        glMatrixMode(GL_PROJECTION)
        glPopMatrix()
        
        glMatrixMode(GL_MODELVIEW)
        glPopMatrix()
        
        # Re-enable lighting and depth testing
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
    
    def render_string(self, x, y, string):
        """Render string at specified position"""
        glRasterPos2f(x, y)
        for c in string:
            glutBitmapCharacter(GLUT_BITMAP_9_BY_15, ctypes.c_int(c))

# =========================== 5) 主函数入口 ===========================
def main():
    global running
    
    # 启动UDP接收线程
    udp_thread = UDPReceiverThread()
    udp_thread.start()
    
    # 创建可视化系统
    visualization = TrainVisualization()
    
    # 进入GLUT主循环
    try:
        glutMainLoop()
    except Exception as e:
        print(f"GLUT主循环出现异常: {e}")
    finally:
        running = False
    
    # Wait for UDP thread to end
    udp_thread.join(timeout=1.0)
    
    print("程序正常退出")

if __name__ == "__main__":
    main()
"""

运行：

# Windows 或者 MacOS
    E:
    cd E:\ResearchDocuments\ROS2WithSPCK\SPCK_Track
    python TrkRel_RTViz.py

# Ubuntu
    conda activate pypack
    cd /home/yaoyao/Documents/myProjects/ROS2WithSPCK/SPCK_Track
    python TrkRel_RTViz.py



+-----------------------+      +----------------------+
| UDP接收线程           |      | OpenGL渲染线程       |
| - 监听UDP数据         | ---> | - 3D车辆模型渲染     |
| - 解析数据包          |      | - 轨道与环境渲染     |
| - 数据预处理          |      | - 相机视角控制       |
+-----------------------+      +----------------------+
            |                            ^
            v                            |
      +------------------+               |
      | 共享数据结构     |               |
      | - 最新车辆状态   | --------------+
      | - 位置与姿态信息 |
      +------------------+


"""
