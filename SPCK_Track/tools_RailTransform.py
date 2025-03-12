# tools_RailTransform.py

"""
 主要功能：
   1) 坐标变换相关函数：
       rot_x, rot_y, rot_z, euler_zyx_to_matrix, make_transform, transform_points_3d

   2) 轨道姿态获取函数：
       get_track_pose(s, s_vals, xvals, yvals, zvals, psi_vals, phi_vals)

   3) 几何建模：
       create_cylinder_mesh, create_box_mesh

   4) 绘图函数：
       draw_wheelset, draw_box

   5) SIMPACK 文件读取：
       read_simpack_6dof(file_path)

   6) 可选辅助函数：
       compute_points_bounding_box(...) —— 用于计算一组点的包围盒


 -----------------------------------------------------------------------------
 轨道坐标数据:
    - 本工具假设轨道本身的三维位置和朝向存储在 trajectory_data.npz 或 trajectory_data.json 中：
      包含以下数组:  s_vals, xvals, yvals, zvals, psi_vals, (phi_vals)

    - 其中：
      * s_vals : 离散的里程点 (标量或数组)
      * {xvals, yvals, zvals} : 轨道在全局系的离散坐标
      * psi_vals, phi_vals : 轨道在全局系的旋转角度（典型地 yaw, roll），
                             如有需要，也可拓展 pitch_vals 以表示轨道纵向坡度

 -----------------------------------------------------------------------------
 车辆部件(轮对、转向架、车体)的运动:
    - 车辆部件在轨道坐标系下通常有 5 个自由度 (2 个平移 + 3 个转角)。
      因为车辆在轨道前进方向的平移由里程 s_val 已经决定，所以
      局部坐标可以只关心 y_local, z_local 的偏移，以及 roll, pitch, yaw 等旋转。

    - 例如，轮对的局部坐标可表示为:
          ( x_local,  y_local,  z_local ) = ( 0.0,  y_local,  z_local )
      而其姿态可表示为:
          ( roll_local, pitch_local, yaw_local ).

 -----------------------------------------------------------------------------
 坐标转换流程:
    1. 通过 s_query 在轨道数据中插值或查找最近点，得到轨道在全局坐标系下的位置和朝向。
       如 (X_T, Y_T, Z_T, yaw_T, pitch_T=0, roll_T)。
    2. 将车辆部件在轨道坐标下的 5 自由度，与轨道位姿合成齐次变换矩阵。
       例如，先由 (yaw_local, pitch_local, roll_local, 0.0, y_local, z_local)
       生成 T_W2T (部件局部->轨道系)，再把它乘以 T_T2G (轨道系->全局系)。
    3. 得到完整的部件在全局坐标系下的 6D 位姿，再对具体几何模型(轮对、转向架、车体)做坐标变换以可视化。
    4. 车体几何坐标还需要考虑 hc = -1.8 与 旋转 180°, 以与 SIMPACK 中几何显示完全一致; 转向架无需做此变换

 -----------------------------------------------------------------------------
 示例用法 (简要):
    1) 从文件读入 s_vals, xvals, yvals, zvals, psi_vals, phi_vals
    2) 对给定的部件局部信息 (s_query, y_local, z_local, roll_local, pitch_local, yaw_local),
       先用 get_track_pose(s_query) 拿到轨道在全局的 (X_T, Y_T, Z_T, yaw_T, pitch_T, roll_T),
       然后用 make_transform() 拼接变换
    3) 最终用 transform_points_3d() 将几何点云投到全局坐标系下，或直接用绘制函数 draw_XXX()。
    
"""

import numpy as np
import pandas as pd
import math
import matplotlib.pyplot as plt
import json

# =============== 1) 轨道姿态获取 ===============
def get_track_pose(s, s_vals, xvals, yvals, zvals, psi_vals, phi_vals):
    """
    根据轨道里程 s, 在数组 s_vals 中找到与 s 最接近的索引 idx，
    并返回对应轨道在全局坐标系下的 (X_T, Y_T, Z_T, yaw_T, pitch_T, roll_T)。
    其中:
      - yaw_T   = psi_vals[idx]
      - pitch_T = 0.0  (暂时不考虑纵向坡度)
      - roll_T  = phi_vals[idx]
    参数:
      s:        目标里程
      s_vals:   轨道里程数组 (N,)
      xvals:    每个轨道里程对应的全局 X
      yvals:    每个轨道里程对应的全局 Y
      zvals:    每个轨道里程对应的全局 Z
      psi_vals: 轨道朝向 yaw
      phi_vals: 轨道 roll (若有超高)
    返回:
      (X_T, Y_T, Z_T, yaw_T, pitch_T, roll_T)
    """
    idx = np.argmin(np.abs(s_vals - s))
    X_T = xvals[idx]
    Y_T = yvals[idx]
    Z_T = zvals[idx]
    yaw_T   = psi_vals[idx]
    pitch_T = 0.0       # 如果无坡度，可直接写死为 0
    roll_T  = phi_vals[idx]
    return (X_T, Y_T, Z_T, yaw_T, pitch_T, roll_T)


# =============== 2) 欧拉角与坐标变换 ===============
def rot_z(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([
        [ c, -s,  0],
        [ s,  c,  0],
        [ 0,  0,  1]
    ], dtype=float)

def rot_y(pitch):
    c, s = np.cos(pitch), np.sin(pitch)
    return np.array([
        [ c,  0,  s],
        [ 0,  1,  0],
        [-s,  0,  c]
    ], dtype=float)

def rot_x(roll):
    c, s = np.cos(roll), np.sin(roll)
    return np.array([
        [1,  0,  0],
        [0,  c, -s],
        [0,  s,  c]
    ], dtype=float)

def euler_zyx_to_matrix(yaw, pitch, roll):
    """
    以 Z-Y-X (yaw-pitch-roll) 顺序旋转，返回 3x3 旋转矩阵:
      R = Rz(yaw) * Ry(pitch) * Rx(roll)
    """
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)

def make_transform(yaw, pitch, roll, px, py, pz):
    """
    生成 4x4 的齐次变换矩阵:
      [  R(zyx)   T ]
      [    0     1 ]
    其中平移部分为(px, py, pz)，旋转部分由 yaw/pitch/roll 生成。
    """
    T = np.eye(4)
    R = euler_zyx_to_matrix(yaw, pitch, roll)
    T[:3,:3] = R
    T[:3, 3] = [px, py, pz]
    return T

def transform_points_3d(T, xyz):
    """
    对点云 xyz(shape=(N,3)) 用 4x4 齐次变换矩阵 T 变换，返回变换后的新坐标 (N,3)。
    """
    homo = np.hstack([xyz, np.ones((len(xyz),1))])  # (N,4)
    trans = (T @ homo.T).T                          # (N,4)
    return trans[:,:3]


# =============== 3) 几何建模 ===============
def create_cylinder_mesh(length, radius, ntheta=30, nL=10):
    """
    沿 Y 轴方向的圆柱网格: y in [-length/2.. +length/2], 半径=radius.
    ntheta: 周向离散数
    nL:     轴向离散数
    返回 (X, Y, Z)，可直接用于 plot_surface(X, Y, Z)。
    """
    y_ = np.linspace(-length/2, length/2, nL)
    th = np.linspace(0, 2*np.pi, ntheta)
    TH, Y = np.meshgrid(th, y_)  # (nL, ntheta)
    X = radius * np.cos(TH)
    Z = radius * np.sin(TH)
    return X, Y, Z

def create_box_mesh(L, W, H, nx=2, ny=2, nz=2):
    """
    生成一个长方体(或立方体)的6个面网格 (共返回6个元素，每个元素都是(X,Y,Z)).
    其中:
      x in [-L/2, +L/2],
      y in [-W/2, +W/2],
      z in [-H/2, +H/2]

    nx, ny, nz 分别控制在 x, y, z 方向上的网格离散数。

    返回: 列表 box_faces，包含 6 个元组，每个元组是 (X_face, Y_face, Z_face)
    可以配合 plot_surface(...) 绘制。
    """
    x_ = np.linspace(-L/2, L/2, nx)
    y_ = np.linspace(-W/2, W/2, ny)
    z_ = np.linspace(-H/2, H/2, nz)   # 对称设计: z 从 -H/2 到 +H/2

    # 上面 (z=+H/2)
    X_top, Y_top = np.meshgrid(x_, y_)
    Z_top = np.ones_like(X_top)*(+H/2)  # 修正为 +H/2

    # 底面 (z=-H/2)
    X_bot, Y_bot = np.meshgrid(x_, y_)
    Z_bot = np.ones_like(X_bot)*(-H/2)  # 修正为 -H/2

    # 前面 (x=+L/2)
    Yf, Zf = np.meshgrid(y_, z_)
    Xf = np.ones_like(Yf)*(+L/2)

    # 后面 (x=-L/2)
    Yb, Zb = np.meshgrid(y_, z_)
    Xb = np.ones_like(Yb)*(-L/2)

    # 右面 (y=+W/2)
    Xr, Zr = np.meshgrid(x_, z_)
    Yr = np.ones_like(Xr)*(+W/2)

    # 左面 (y=-W/2)
    Xl, Zl = np.meshgrid(x_, z_)
    Yl = np.ones_like(Xl)*(-W/2)

    return [
        (X_top,  Y_top,  Z_top),
        (X_bot,  Y_bot,  Z_bot),
        (Xf,     Yf,     Zf),
        (Xb,     Yb,     Zb),
        (Xr,     Yr,     Zr),
        (Xl,     Yl,     Zl),
    ]

# =============== 4) 绘图函数 ===============
def draw_wheelset(ax, T_W2G,
                  pts_axle, X_axle_shape,
                  pts_wheel, X_wheel_shape,
                  wheel_offset,
                  color_axle='magenta',
                  color_wheel='cyan',
                  alpha=1):
    """
    在给定 Matplotlib 3D坐标轴(ax) 上绘制 1 根车轴 + 2 车轮（基于圆柱网格）。
    参数：
      ax:             matplotlib Axes3D 对象
      T_W2G:          轮对局部坐标系W -> 全局坐标系G 的齐次变换(4x4)
      pts_axle:       车轴网格点 (N,3)
      X_axle_shape:   为了重塑回 plot_surface 所需的 (nx, ny) 形状
      pts_wheel:      车轮网格点 (N,3)
      X_wheel_shape:  同上，用于绘制圆柱的 shape
      wheel_offset:   车轮中心(左右)在 y 方向的偏移量
      color_axle:     车轴颜色
      color_wheel:    车轮颜色
      alpha:          透明度
    """
    # 1) 车轴
    ptsA_G = transform_points_3d(T_W2G, pts_axle)
    XA = ptsA_G[:,0].reshape(X_axle_shape)
    YA = ptsA_G[:,1].reshape(X_axle_shape)
    ZA = ptsA_G[:,2].reshape(X_axle_shape)
    ax.plot_surface(XA, YA, ZA, color=color_axle, alpha=alpha, edgecolor='none')

    # 2) 左轮
    pts_left = pts_wheel.copy()
    pts_left[:,1] += wheel_offset  # 在轮对局部系下，y=+offset
    pts_left_G = transform_points_3d(T_W2G, pts_left)
    XL = pts_left_G[:,0].reshape(X_wheel_shape)
    YL = pts_left_G[:,1].reshape(X_wheel_shape)
    ZL = pts_left_G[:,2].reshape(X_wheel_shape)
    ax.plot_surface(XL, YL, ZL, color=color_wheel, alpha=alpha, edgecolor='none')

    # 3) 右轮
    pts_right = pts_wheel.copy()
    pts_right[:,1] -= wheel_offset
    pts_right_G = transform_points_3d(T_W2G, pts_right)
    XR = pts_right_G[:,0].reshape(X_wheel_shape)
    YR = pts_right_G[:,1].reshape(X_wheel_shape)
    ZR = pts_right_G[:,2].reshape(X_wheel_shape)
    ax.plot_surface(XR, YR, ZR, color=color_wheel, alpha=alpha, edgecolor='none')

def draw_box(ax, T_B2G, box_faces, color='orange', alpha=0.6):
    """
    在给定 Matplotlib 3D坐标轴(ax) 上绘制一个长方体(由 box_faces 给定的6个面)。
    参数：
      ax:         matplotlib Axes3D 对象
      T_B2G:      盒子局部系 -> 全局系 的齐次变换
      box_faces:  create_box_mesh(...) 的返回列表 [ (Xf, Yf, Zf), ... 共6面 ]
      color:      盒子颜色
      alpha:      透明度
    """
    for (Xf, Yf, Zf) in box_faces:
        # 面在局部坐标下的 (N,3) 点集
        pts_face_local = np.column_stack([Xf.ravel(), Yf.ravel(), Zf.ravel()])
        # 变换到全局
        pts_face_global = transform_points_3d(T_B2G, pts_face_local)

        # reshape 回去
        Xg = pts_face_global[:,0].reshape(Xf.shape)
        Yg = pts_face_global[:,1].reshape(Xf.shape)
        Zg = pts_face_global[:,2].reshape(Xf.shape)

        ax.plot_surface(Xg, Yg, Zg, color=color, alpha=alpha, edgecolor='black')


# =============== 5) 读取 SIMPACK 输出 ===============
def read_simpack_6dof(file_path):
    """
    读取 SIMPACK 导出的TSV文件(以制表符分隔)，并返回一个 Pandas DataFrame。
    注意:
      - 原始列名(如 y_ws01_x, y_ws02_x, ...)将保留。
      - float_precision='high' 可减少浮点数精度丢失。
      - 使用 chunksize 逐块读取，再合并，适用于大文件。
    """
    chunks = pd.read_csv(file_path, sep='\t', float_precision='high', chunksize=10000)
    df = pd.DataFrame()
    for c in chunks:
        df = pd.concat([df, c], ignore_index=True)
    return df


# =============== 6) 选做: 计算一组点的包围盒 ===============
def compute_points_bounding_box(pts_list, extra_margin=0.5):
    """
    给定若干(N,3)的点云列表(或单个数组)，自动计算包围盒大小。
    返回 (xlim, ylim, zlim)，每个是 (min_val, max_val)。
    可用于自行设置 ax.set_xlim, ax.set_ylim, ax.set_zlim.
    """
    if isinstance(pts_list, np.ndarray):
        pts_all = pts_list
    else:
        # 若是 list of ndarray，则堆叠
        pts_all = np.vstack(pts_list)

    Xmin, Xmax = pts_all[:,0].min(), pts_all[:,0].max()
    Ymin, Ymax = pts_all[:,1].min(), pts_all[:,1].max()
    Zmin, Zmax = pts_all[:,2].min(), pts_all[:,2].max()

    Xmid = 0.5 * (Xmin + Xmax)
    Ymid = 0.5 * (Ymin + Ymax)
    Zmid = 0.5 * (Zmin + Zmax)
    box_half = 0.5 * max(Xmax - Xmin, Ymax - Ymin, Zmax - Zmin)
    box_half += extra_margin

    return (Xmid - box_half, Xmid + box_half), \
           (Ymid - box_half, Ymid + box_half), \
           (Zmid - box_half, Zmid + box_half)

class TrackData:
    def __init__(self, json_path='trajectory_data.json'):
        """Load track centerline and rail data"""
        try:
            import json
            with open(json_path, 'r') as jf:
                trajectory_data = json.load(jf)
            
            # 从JSON数据转换为NumPy数组
            self.s_vals = np.array(trajectory_data['s'])        # Track mileage array
            self.xvals = np.array(trajectory_data['x'])         # Corresponding global X
            self.yvals = np.array(trajectory_data['y'])         # Global Y
            self.zvals = np.array(trajectory_data['z'])         # Global Z
            self.psi_vals = np.array(trajectory_data['psi'])    # Track yaw
            self.phi_vals = np.array(trajectory_data['phi'])    # Track roll (if superelevation)
            self.left_rail = np.array(trajectory_data['left_rail'])    # (N,3) Left rail
            self.right_rail = np.array(trajectory_data['right_rail'])   # (N,3) Right rail
            print(f"Track data loaded successfully: {len(self.s_vals)} points")
        except Exception as e:
            print(f"[ERROR] Failed to load track data: {e}")
            # Create some virtual track data for testing
            self.s_vals = np.linspace(0, 1000, 1000)
            self.xvals = self.s_vals.copy()
            self.yvals = np.zeros_like(self.s_vals)
            self.zvals = np.zeros_like(self.s_vals)
            self.psi_vals = np.zeros_like(self.s_vals)
            self.phi_vals = np.zeros_like(self.s_vals)
            self.left_rail = np.column_stack([self.xvals, self.yvals + 0.75, self.zvals])
            self.right_rail = np.column_stack([self.xvals, self.yvals - 0.75, self.zvals])
            print("[WARNING] Using virtual track data for testing")
    
    def get_track_pose(self, s):
        """
        Input track mileage s, find the nearest point index idx in s_vals array,
        return the track position and orientation in global frame: (X_T, Y_T, Z_T, yaw_T, pitch_T, roll_T).
        pitch_T is temporarily set to 0.0 (no gradient), roll_T=phi_vals[idx].
        """
        idx = np.argmin(np.abs(self.s_vals - s))
        X_T = self.xvals[idx]
        Y_T = self.yvals[idx]
        Z_T = self.zvals[idx]
        yaw_T = self.psi_vals[idx]
        pitch_T = 0.0       # Set to 0 if no gradient
        roll_T = self.phi_vals[idx]
        return (X_T, Y_T, Z_T, yaw_T, pitch_T, roll_T)
    

# Function to linearly interpolate between two values
def lerp(a, b, t):
    """Linear interpolation between a and b with factor t (0.0-1.0)"""
    return a + (b - a) * t

# Function to linearly interpolate between two angles (in radians)
def angle_lerp(a, b, t):
    """Interpolate between two angles, handling wrap-around properly"""
    # Ensure angles are within [0, 2π]
    a = a % (2.0 * np.pi)
    b = b % (2.0 * np.pi)
    
    # Find the shortest path
    diff = b - a
    if diff > np.pi:
        b -= 2.0 * np.pi
    elif diff < -np.pi:
        b += 2.0 * np.pi
    
    # Linear interpolation
    return a + (b - a) * t