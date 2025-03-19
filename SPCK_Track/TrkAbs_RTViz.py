# 文件名: TrkAbs_RTViz.py

import socket
import struct
import time
import numpy as np
import threading
import queue
import ctypes
import sys
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

from tools_RailTransform import (TrackData, lerp, angle_lerp)

# ---------------------- 全局队列、数据结构 ----------------------
data_queue = queue.Queue(maxsize=10)   # UDP接收到的包排队
latest_data = None                    # 最新帧数据 (长度=77)
previous_data = None                  # 上一帧数据 (用于插值)
data_ready = threading.Event()        # 标记有新数据可用
running = True                        # 控制线程退出
frame_time = 0.0
last_update_time = 0.0

# 动画/渲染控制
TARGET_FPS = 60
FRAME_TIME = 1.0 / TARGET_FPS
interpolation_enabled = True
animation_paused = False
use_received_time = False   # 若想使用 sim_time 做插值可改 True

# ---------------------- 1) UDP接收线程 ----------------------
class UDPReceiverThread(threading.Thread):
    """
    后台接收从 TrkAbs_UDPSenderNode 发送的 77 个 double 的数据包，
    并存储到 latest_data / previous_data 以供渲染。
    """
    def __init__(self, ip="0.0.0.0", port=10099):
        super().__init__()
        self.daemon = True
        self.UDP_IP = ip
        self.UDP_PORT = port
        self.sock = None

        # 数据包应包含 77 个 double (小端序)
        self.EXPECTED_LEN = 77
        self.EXPECTED_BYTES = self.EXPECTED_LEN * 8
        self.fmt = "<" + "d" * self.EXPECTED_LEN

        # 接收统计
        self.data_count = 0

    def run(self):
        global latest_data, previous_data, data_ready, running

        # 绑定 UDP 套接字
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.UDP_IP, self.UDP_PORT))
        self.sock.settimeout(0.01)

        print(f"[UDP] Listening on {self.UDP_IP}:{self.UDP_PORT}")

        while running:
            try:
                data, addr = self.sock.recvfrom(65535)
                if len(data) != self.EXPECTED_BYTES:
                    print(f"[WARNING] Received {len(data)} bytes, expected {self.EXPECTED_BYTES} -> skip")
                    continue

                values = struct.unpack(self.fmt, data)
                self.data_count += 1

                # 每 5000 包提示一次
                if self.data_count % 5000 == 0:
                    print(f"[INFO] Received {self.data_count} packets; "
                          f"first 3 fields: {values[0]:.3f}, {values[1]:.3f}, {values[2]:.3f}")

                # 更新 global
                previous_data = latest_data
                latest_data = values
                data_ready.set()

            except socket.timeout:
                pass
            except Exception as e:
                print(f"[ERROR] UDP thread exception: {e}")
                break

        print("[UDP] Thread exiting...")
        if self.sock:
            self.sock.close()

# ---------------------- 2) 数据插值（可选） ----------------------
def interpolate_frames(frame1, frame2, t):
    """
    在 frame1, frame2 两个数组之间按 t ∈ [0,1] 做插值:
      - 对 x,y,z 用线性插值
      - 对 roll, yaw, pitch 用角度插值
      - 其它字段直接取 frame2
    """
    if frame1 is None or frame2 is None:
        return frame2 if frame2 is not None else frame1

    interpolated = np.zeros(len(frame1), dtype=float)

    # === 参照 SimpackY.msg 的下标 ===
    # 0: sim_time
    # 1: y_spcktime
    # 2: y_cb_vx
    # (3..5):  车体 x,y,z
    # (6..8):  车体 roll,yaw,pitch
    # (9..16): 8车轮转速
    # (17..19): f01 x,y,z
    # (20..22): f01 roll,yaw,pitch
    # (23..25): f02 x,y,z
    # (26..28): f02 roll,yaw,pitch
    # (29..31): ws01 x,y,z
    # (32..34): ws01 roll,yaw,pitch
    # (35..37): ws02 x,y,z
    # (38..40): ws02 roll,yaw,pitch
    # (41..43): ws03 x,y,z
    # (44..46): ws03 roll,yaw,pitch
    # (47..49): ws04 x,y,z
    # (50..52): ws04 roll,yaw,pitch
    # (53..60): 8车轮转角 rota
    # (61..68): 8根连杆 pitch
    # (69..72): 4车轮组 vy
    # (73..76): 4车轮组 vyaw

    # 坐标 (x,y,z) 的索引
    position_indices = [
        3,4,5,       # 车体
        17,18,19,    # f01
        23,24,25,    # f02
        29,30,31,    # ws01
        35,36,37,    # ws02
        41,42,43,    # ws03
        47,48,49,    # ws04
    ]
    # 姿态 (roll,yaw,pitch) 的索引
    angle_indices = [
        6,7,8,       # 车体
        20,21,22,    # f01
        26,27,28,    # f02
        32,33,34,    # ws01
        38,39,40,    # ws02
        44,45,46,    # ws03
        50,51,52,    # ws04
    ]

    # 1) 线性插值(位置)
    for idx in position_indices:
        interpolated[idx] = lerp(frame1[idx], frame2[idx], t)

    # 2) 角度插值(roll, yaw, pitch)
    for idx in angle_indices:
        interpolated[idx] = angle_lerp(frame1[idx], frame2[idx], t)

    # 3) 其它字段直接取 frame2
    for i in range(len(frame1)):
        if i not in position_indices and i not in angle_indices:
            interpolated[i] = frame2[i]

    return interpolated

# ---------------------- 3) 车辆模型类 ----------------------
class TrainModels:
    """
    生成并管理车体、转向架、轮对等几何模型（与之前类似）。
    """
    def __init__(self):
        # 基础参数可自行调整
        self.axle_length = 2.0
        self.axle_radius = 0.065
        self.wheel_radius = 0.43
        self.wheel_thickness = 0.04
        self.wheel_offset = 0.72

        self.car_body_length = 25.0
        self.car_body_width = 3.0
        self.car_body_height = 3.0

        self.bogie_length = 3.0
        self.bogie_width = 2.5
        self.bogie_height = 0.5

        # 创建顶点数据
        self.create_models()

        # OpenGL 显示列表
        self.axle_list = 0
        self.wheel_list = 0
        self.car_body_list = 0
        self.bogie_list = 0

    def create_cylinder(self, length, radius, sides=20):
        vertices = []
        for i in range(sides+1):
            theta = i * (2*np.pi / sides)
            x = radius * np.cos(theta)
            z = radius * np.sin(theta)
            # 每段四个顶点(两端点 + 外壁)
            vertices.append([-length/2, 0, 0])
            vertices.append([length/2, 0, 0])
            vertices.append([length/2, x, z])
            vertices.append([-length/2, x, z])
        return np.array(vertices)

    def create_wheel(self, thickness, radius, sides=20):
        vertices = []
        for i in range(sides+1):
            theta = i * (2*np.pi / sides)
            y = radius * np.cos(theta)
            z = radius * np.sin(theta)
            vertices.append([-thickness/2, 0, 0])
            vertices.append([thickness/2, 0, 0])
            vertices.append([thickness/2, y, z])
            vertices.append([-thickness/2, y, z])
        return np.array(vertices)

    def create_box(self, length, width, height):
        vertices = [
            # 前面
            [length/2, -width/2, -height/2], [length/2, width/2, -height/2],
            [length/2, width/2, height/2],   [length/2, -width/2, height/2],
            # 后面
            [-length/2, -width/2, -height/2], [-length/2, width/2, -height/2],
            [-length/2, width/2, height/2],   [-length/2, -width/2, height/2],
            # 右面
            [-length/2, width/2, -height/2], [length/2, width/2, -height/2],
            [length/2, width/2, height/2],   [-length/2, width/2, height/2],
            # 左面
            [-length/2, -width/2, -height/2], [length/2, -width/2, -height/2],
            [length/2, -width/2, height/2],   [-length/2, -width/2, height/2],
            # 顶面
            [-length/2, -width/2, height/2],  [-length/2, width/2, height/2],
            [length/2, width/2, height/2],    [length/2, -width/2, height/2],
            # 底面
            [-length/2, -width/2, -height/2], [-length/2, width/2, -height/2],
            [length/2, width/2, -height/2],   [length/2, -width/2, -height/2]
        ]
        return np.array(vertices)

    def create_models(self):
        self.axle_vertices = self.create_cylinder(self.axle_length, self.axle_radius)
        self.wheel_vertices = self.create_wheel(self.wheel_thickness, self.wheel_radius)
        self.car_body_vertices = self.create_box(self.car_body_length,
                                                 self.car_body_width,
                                                 self.car_body_height)
        self.bogie_vertices = self.create_box(self.bogie_length,
                                              self.bogie_width,
                                              self.bogie_height)

    def create_display_lists(self):
        try:
            self.axle_list = glGenLists(1)
            glNewList(self.axle_list, GL_COMPILE)
            self.render_axle()
            glEndList()

            self.wheel_list = glGenLists(1)
            glNewList(self.wheel_list, GL_COMPILE)
            self.render_wheel()
            glEndList()

            self.car_body_list = glGenLists(1)
            glNewList(self.car_body_list, GL_COMPILE)
            self.render_car_body()
            glEndList()

            self.bogie_list = glGenLists(1)
            glNewList(self.bogie_list, GL_COMPILE)
            self.render_bogie()
            glEndList()

            print("Train geometry display lists created.")
        except Exception as e:
            print(f"Error creating display lists: {e}")
            self.axle_list = 0
            self.wheel_list = 0
            self.car_body_list = 0
            self.bogie_list = 0

    def render_axle(self):
        glColor3f(1.0, 0.0, 1.0)  # magenta
        glBegin(GL_QUADS)
        for i in range(0, len(self.axle_vertices), 4):
            for j in range(4):
                glVertex3fv(self.axle_vertices[i+j])
        glEnd()

    def render_wheel(self):
        glColor3f(0.0, 1.0, 1.0)
        glBegin(GL_QUADS)
        for i in range(0, len(self.wheel_vertices), 4):
            for j in range(4):
                glVertex3fv(self.wheel_vertices[i+j])
        glEnd()

    def render_car_body(self):
        glColor4f(1.0, 0.5, 0.0, 0.2)
        glBegin(GL_QUADS)
        for i in range(0, len(self.car_body_vertices), 4):
            for j in range(4):
                glVertex3fv(self.car_body_vertices[i+j])
        glEnd()

    def render_bogie(self):
        glColor4f(0.0, 0.8, 0.0, 0.3)
        glBegin(GL_QUADS)
        for i in range(0, len(self.bogie_vertices), 4):
            for j in range(4):
                glVertex3fv(self.bogie_vertices[i+j])
        glEnd()

    def draw_wheelset(self, T_W2G):
        """
        绘制一个轮对 (车轴 + 两个车轮) 在变换矩阵 T_W2G 下的姿态。
        """
        # 车轴需要绕Z旋转90度，令轴平行 X
        axle_rotation = np.array([
            [0, -1,  0, 0],
            [1,  0,  0, 0],
            [0,  0,  1, 0],
            [0,  0,  0, 1]
        ], dtype=float)
        T_axle = T_W2G @ axle_rotation

        glPushMatrix()
        glMultMatrixf(T_axle.T.flatten())
        if self.axle_list > 0:
            glCallList(self.axle_list)
        else:
            self.render_axle()
        glPopMatrix()

        # 左车轮
        glPushMatrix()
        T_left = np.eye(4)
        T_left[1,3] = self.wheel_offset
        rotation = np.array([
            [0,0,1,0],
            [1,0,0,0],
            [0,1,0,0],
            [0,0,0,1]
        ], dtype=float)
        T_final = T_W2G @ T_left @ rotation
        glMultMatrixf(T_final.T.flatten())
        if self.wheel_list > 0:
            glCallList(self.wheel_list)
        else:
            self.render_wheel()
        glPopMatrix()

        # 右车轮
        glPushMatrix()
        T_right = np.eye(4)
        T_right[1,3] = -self.wheel_offset
        T_final = T_W2G @ T_right @ rotation
        glMultMatrixf(T_final.T.flatten())
        if self.wheel_list > 0:
            glCallList(self.wheel_list)
        else:
            self.render_wheel()
        glPopMatrix()

    def draw_box(self, T_B2G, display_list, box_type="bogie"):
        glPushMatrix()
        glMultMatrixf(T_B2G.T.flatten())
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        if display_list > 0:
            glCallList(display_list)
        else:
            if box_type == "bogie":
                self.render_bogie()
            else:
                self.render_car_body()
        glDisable(GL_BLEND)
        glPopMatrix()

# ---------------------- (原先的) “Rail坐标 -> z向上” 变换函数已移除 ----------------------
# 如下函数已不再需要：
# def convert_from_simpack_rail_to_z_up(...):
#     pass

# ---------------------- 4) OpenGL 可视化类 ----------------------
class TrainVisualization:
    def __init__(self, width=1200, height=800):
        self.window_width = width
        self.window_height = height

        # 初始化 OpenGL
        self.init_opengl()

        # 载入轨道(仅用于可视化)
        self.track = TrackData()

        # 车辆模型
        self.train_models = TrainModels()
        self.train_models.create_display_lists()

        # 相机控制参数
        self.camera_distance = 30.0
        self.camera_elevation = -30.0
        self.camera_azimuth = 120.0
        self.camera_target = [0.0, 0.0, 0.0]
        self.auto_camera = True
        self.camera_up_vector = [0, 0, 1]

        # 轨道显示开关
        self.show_track = True
        self.show_rails = True
        self.show_fps = True

        # 帧率统计
        self.frame_count = 0
        self.fps = 0.0
        self.last_fps_time = time.time()

        # 编译轨道显示列表
        self.compile_track_display_list()

        # 动画定时
        self.start_time = time.time()
        self.last_time = self.start_time
        glutTimerFunc(int(FRAME_TIME * 1000), self.animation_timer, 0)

    def init_opengl(self):
        glutInit(sys.argv)
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
        glutInitWindowSize(self.window_width, self.window_height)
        glutCreateWindow(b"Abs-Coord Train Visualization")

        # 注册回调
        glutDisplayFunc(self.display)
        glutReshapeFunc(self.reshape)
        glutKeyboardFunc(self.keyboard)
        glutSpecialFunc(self.special_keys)
        glutIdleFunc(self.idle)

        # 背景与深度测试
        glClearColor(0.9, 0.9, 1.0, 1.0)
        glEnable(GL_DEPTH_TEST)

        # 光照
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        light_position = [100.0, 100.0, -100.0, 0.0]
        glLightfv(GL_LIGHT0, GL_POSITION, light_position)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)

    def compile_track_display_list(self):
        try:
            # 轨道中心线
            self.track_center_list = glGenLists(1)
            glNewList(self.track_center_list, GL_COMPILE)
            glDisable(GL_LIGHTING)
            glColor3f(0.0, 0.0, 0.0)
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

            # 左轨
            glColor3f(0.8, 0.0, 0.0)
            glBegin(GL_LINE_STRIP)
            for i in range(len(self.track.left_rail)):
                glVertex3f(self.track.left_rail[i,0],
                           self.track.left_rail[i,1],
                           self.track.left_rail[i,2])
            glEnd()

            # 右轨
            glColor3f(0.0, 0.0, 0.8)
            glBegin(GL_LINE_STRIP)
            for i in range(len(self.track.right_rail)):
                glVertex3f(self.track.right_rail[i,0],
                           self.track.right_rail[i,1],
                           self.track.right_rail[i,2])
            glEnd()

            glEnable(GL_LIGHTING)
            glEndList()
            print("Track display lists created.")
        except Exception as e:
            print(f"Error building track display lists: {e}")
            self.track_center_list = 0
            self.rails_list = 0

    def animation_timer(self, value):
        global frame_time, last_update_time
        if not animation_paused:
            current_time = time.time()
            elapsed = current_time - last_update_time

            # 如果使用接收端模拟时间
            if use_received_time and latest_data is not None and previous_data is not None:
                sim_time_start = previous_data[0]
                sim_time_end   = latest_data[0]
                sim_range = sim_time_end - sim_time_start
                if sim_range > 0:
                    t = min(1.0, elapsed / (FRAME_TIME * 10))
                    frame_time = sim_time_start + t * sim_range
                else:
                    frame_time = sim_time_end
            else:
                frame_time += FRAME_TIME

            glutPostRedisplay()

            # 计算 FPS
            self.frame_count += 1
            fps_elapsed = current_time - self.last_fps_time
            if fps_elapsed >= 1.0:
                self.fps = self.frame_count / fps_elapsed
                self.frame_count = 0
                self.last_fps_time = current_time

        glutTimerFunc(int(FRAME_TIME * 1000), self.animation_timer, 0)

    def reshape(self, w, h):
        self.window_width = w
        self.window_height = h
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, float(w)/float(h), 0.1, 2000.0)

    def keyboard(self, key, x, y):
        global running, animation_paused, interpolation_enabled, use_received_time
        if key == b'\x1b':  # ESC 退出
            running = False
            glutLeaveMainLoop()
        elif key == b't':
            self.show_track = not self.show_track
        elif key == b'r':
            self.show_rails = not self.show_rails
        elif key == b'f':
            self.show_fps = not self.show_fps
        elif key == b'p':
            animation_paused = not animation_paused
        elif key == b'i':
            interpolation_enabled = not interpolation_enabled
        elif key == b's':
            use_received_time = not use_received_time
        elif key == b'c':
            self.auto_camera = not self.auto_camera
        elif key == b'v':
            # 翻转 Z 方向
            self.camera_up_vector = [0, 0, -self.camera_up_vector[2]]
            self.camera_elevation = -self.camera_elevation

    def special_keys(self, key, x, y):
        if key == GLUT_KEY_UP:
            self.camera_elevation += 5
        elif key == GLUT_KEY_DOWN:
            self.camera_elevation -= 5
        elif key == GLUT_KEY_LEFT:
            self.camera_azimuth += 5
        elif key == GLUT_KEY_RIGHT:
            self.camera_azimuth -= 5
        elif key == GLUT_KEY_PAGE_UP:
            self.camera_distance -= 2
        elif key == GLUT_KEY_PAGE_DOWN:
            self.camera_distance += 2

        self.camera_elevation = max(-85, min(85, self.camera_elevation))
        self.camera_distance = max(5, self.camera_distance)
        glutPostRedisplay()

    def idle(self):
        global data_ready, last_update_time
        if data_ready.is_set():
            data_ready.clear()
            last_update_time = time.time()

    def set_camera(self):
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        x = (self.camera_target[0]
             + self.camera_distance * np.cos(np.radians(self.camera_elevation))
             * np.cos(np.radians(self.camera_azimuth)))
        y = (self.camera_target[1]
             + self.camera_distance * np.cos(np.radians(self.camera_elevation))
             * np.sin(np.radians(self.camera_azimuth)))
        z = (self.camera_target[2]
             + self.camera_distance * np.sin(np.radians(self.camera_elevation)))
        gluLookAt(x, y, z,
                  self.camera_target[0], self.camera_target[1], self.camera_target[2],
                  self.camera_up_vector[0], self.camera_up_vector[1], self.camera_up_vector[2])

    def display(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        # 根据是否插值
        current_data = None
        if latest_data is not None:
            if interpolation_enabled and previous_data is not None:
                sim_time_start = previous_data[0]
                sim_time_end   = latest_data[0]
                if sim_time_end > sim_time_start:
                    t = (frame_time - sim_time_start)/(sim_time_end - sim_time_start)
                    t = max(0.0, min(1.0, t))
                    current_data = interpolate_frames(previous_data, latest_data, t)
                else:
                    current_data = latest_data
            else:
                current_data = latest_data

        self.set_camera()

        # 绘制轨道
        if self.show_track and self.track_center_list > 0:
            glCallList(self.track_center_list)
        if self.show_rails and self.rails_list > 0:
            glCallList(self.rails_list)

        # 如果有数据，则绘制车辆 (车体、转向架、轮对都为绝对坐标)
        if current_data is not None:
            self.render_train(current_data)

        # 显示文本信息
        self.display_info_text()

        glutSwapBuffers()

    # ---------- 在绝对坐标下绘制车辆 ----------
    def render_train(self, data):
        """
        data: 长度=77 的数组，索引对应 SimpackY.msg 各字段:

        0: sim_time
        1: y_spcktime
        2: y_cb_vx
        3..5:   车体 x,y,z
        6..8:   车体 roll,yaw,pitch
        9..16:  8车轮转速
        17..19: f01 x,y,z
        20..22: f01 roll,yaw,pitch
        23..25: f02 x,y,z
        26..28: f02 roll,yaw,pitch
        29..31: ws01 x,y,z
        32..34: ws01 roll,yaw,pitch
        35..37: ws02 x,y,z
        38..40: ws02 roll,yaw,pitch
        41..43: ws03 x,y,z
        44..46: ws03 roll,yaw,pitch
        47..49: ws04 x,y,z
        50..52: ws04 roll,yaw,pitch
        53..60: 8 车轮转角
        61..68: 8 连杆 pitch
        69..72: 4 轮对 vy
        73..76: 4 轮对 vyaw
        """
        # 直接从 data 里取，**不再**做 z->-z 或角度翻转
        cbx, cby, cbz = data[3], data[4], data[5]
        cbroll, cbyaw, cbpitch = data[6], data[7], data[8]

        # 相机跟随车体
        if self.auto_camera:
            self.camera_target = [cbx, cby, cbz]

        # 用于生成 4x4 矩阵的小函数
        def euler_zyx_to_matrix(yaw_, pitch_, roll_):
            """
            计算 Rz(yaw)*Ry(pitch)*Rx(roll)
            """
            cy, sy = np.cos(yaw_), np.sin(yaw_)
            cp, sp = np.cos(pitch_), np.sin(pitch_)
            cr, sr = np.cos(roll_), np.sin(roll_)
            R = np.array([
                [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
                [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
                [-sp,    cp*sr,            cp*cr         ]
            ], dtype=float)
            return R

        def make_transform(yaw_, pitch_, roll_, px, py, pz):
            R = euler_zyx_to_matrix(yaw_, pitch_, roll_)
            T = np.eye(4, dtype=float)
            T[:3,:3] = R
            T[0,3], T[1,3], T[2,3] = px, py, pz
            return T

        # 车体做一些位置微调(例如模型中心参考)
        cb_hc = + 1.8   # Z 向上为正值

        T_cb2G = make_transform(-cbyaw, -cbpitch, -cbroll, cbx, cby, cbz  + cb_hc)

        # 取转向架 #1
        f01x, f01y, f01z = data[17], data[18], data[19]
        f01roll, f01yaw, f01pitch = data[20], data[21], data[22]
        T_f01 = make_transform(-f01yaw, -f01pitch, -f01roll, f01x, f01y, f01z)
        # T_f01 = make_transform(f01yaw, f01pitch, f01roll, f01x, f01y, f01z)

        # 取转向架 #2
        f02x, f02y, f02z = data[23], data[24], data[25]
        f02roll, f02yaw, f02pitch = data[26], data[27], data[28]
        T_f02 = make_transform(-f02yaw, -f02pitch, -f02roll, f02x, f02y, f02z)

        # 轮对 (ws01..ws04)
        def get_ws_transform(base_idx):
            wsx = data[base_idx]
            wsy = data[base_idx+1]
            wsz = data[base_idx+2]
            wsroll = data[base_idx+3]
            wsyaw  = data[base_idx+4]
            wspitch= data[base_idx+5]
            return make_transform(-wsyaw, -wspitch, -wsroll, wsx, wsy, wsz)

        T_ws01 = get_ws_transform(29)
        T_ws02 = get_ws_transform(35)
        T_ws03 = get_ws_transform(41)
        T_ws04 = get_ws_transform(47)

        # ============= 先绘制轮对 =============
        self.train_models.draw_wheelset(T_ws01)
        self.train_models.draw_wheelset(T_ws02)
        self.train_models.draw_wheelset(T_ws03)
        self.train_models.draw_wheelset(T_ws04)

        # ============= 半透明模型 =============
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glDepthMask(GL_FALSE)  # 禁写深度

        # 转向架
        self.train_models.draw_box(T_f01, self.train_models.bogie_list, "bogie")
        self.train_models.draw_box(T_f02, self.train_models.bogie_list, "bogie")

        # 车体
        self.train_models.draw_box(T_cb2G, self.train_models.car_body_list, "car_body")

        glDepthMask(GL_TRUE)
        glDisable(GL_BLEND)

    def display_info_text(self):
        glDisable(GL_LIGHTING)
        glDisable(GL_DEPTH_TEST)

        glMatrixMode(GL_PROJECTION)
        glPushMatrix()
        glLoadIdentity()
        glOrtho(0, self.window_width, 0, self.window_height, -1,1)

        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        glLoadIdentity()

        glColor3f(0,0,0)
        self.render_string(10, self.window_height-20,
                           b"[AbsCoord] Arrow keys/PgUp/Down: camera, ESC: exit")
        self.render_string(10, self.window_height-40,
                           b"T: Toggle track, R: Toggle rails, P: Pause, I: Interp, C: AutoCamera")

        # 状态显示
        y_pos = self.window_height - 60
        status_text = (f"Paused: {animation_paused}, "
                       f"Interpolation: {interpolation_enabled}, "
                       f"Camera: {'Auto' if self.auto_camera else 'Manual'}")
        self.render_string(10, y_pos, status_text.encode('utf-8'))
        y_pos -= 20

        if self.show_fps:
            fps_text = f"FPS: {self.fps:.1f}"
            self.render_string(10, y_pos, fps_text.encode('utf-8'))

        glMatrixMode(GL_PROJECTION)
        glPopMatrix()
        glMatrixMode(GL_MODELVIEW)
        glPopMatrix()

        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)

    def render_string(self, x, y, text):
        glRasterPos2f(x, y)
        for c in text:
            glutBitmapCharacter(GLUT_BITMAP_9_BY_15, ctypes.c_int(c))

# ---------------------- 5) main入口 ----------------------
def main():
    global running

    # 启动UDP接收线程
    udp_thread = UDPReceiverThread()
    udp_thread.start()

    # 创建可视化窗口
    vis = TrainVisualization()

    try:
        glutMainLoop()
    except Exception as e:
        print(f"GLUT main loop exception: {e}")
    finally:
        running = False
        udp_thread.join(timeout=1.0)
        print("Program terminated")

if __name__ == "__main__":
    main()
