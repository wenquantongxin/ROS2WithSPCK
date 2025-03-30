# tools_SPCKTrk.py

import numpy as np
import pandas as pd

def bloss_kappa_raw(srel, Lseg, R1, R2):
    """
    BLOSS曲线段的“原始”曲率公式，未考虑平滑衔接。
    srel: 当前段内相对于段起点的局部里程
    Lseg: 整段长度
    R1, R2: 分别是前后段对应的曲率半径 (如: 0->1/300, 则R=300)
    返回：该处曲率k
    """
    k1 = 1.0 / R1 if abs(R1) > 1e-12 else 0.0
    k2 = 1.0 / R2 if abs(R2) > 1e-12 else 0.0
    x  = srel / Lseg
    return k1 + (k2 - k1) * (3*x**2 - 2*x**3)

def bloss_kappa_deriv_raw(srel, Lseg, R1, R2):
    """
    BLOSS曲线段曲率k的一阶导数dk/ds
    """
    k1 = 1.0 / R1 if abs(R1) > 1e-12 else 0.0
    k2 = 1.0 / R2 if abs(R2) > 1e-12 else 0.0
    dk = (k2 - k1)
    x  = srel / Lseg
    return dk * (6*x - 6*x**2) / Lseg

def bloss_kappa_2deriv_raw(srel, Lseg, R1, R2):
    """
    BLOSS曲线段曲率k的二阶导数d^2k/ds^2
    """
    k1 = 1.0 / R1 if abs(R1) > 1e-12 else 0.0
    k2 = 1.0 / R2 if abs(R2) > 1e-12 else 0.0
    dk = (k2 - k1)
    x  = srel / Lseg
    return dk * (6 - 12*x) / (Lseg**2)

def bloss_u_raw(srel, Lseg, u1, u2):
    """
    BLOSS形式的超高插值 u(s)
    u1, u2: 分段首尾的超高(比如 0 -> 0.12m)
    """
    x = srel / Lseg
    return u1 + (u2 - u1)*(3*x**2 - 2*x**3)

def bloss_u_deriv_raw(srel, Lseg, u1, u2):
    """
    BLOSS形式的超高插值一阶导 du/ds
    """
    x = srel / Lseg
    return (u2 - u1) * (6*x - 6*x**2) / Lseg

def bloss_u_2deriv_raw(srel, Lseg, u1, u2):
    """
    BLOSS形式的超高插值二阶导 d^2u/ds^2
    """
    x = srel / Lseg
    return (u2 - u1)*(6 -12*x) / (Lseg**2)

def kappa_raw_formula(stype, Lseg, srel, params):
    """
    给定分段类型 stype, 返回该分段在局部里程srel处的曲率k, k', k''.
    stype: 'STR'(直线), 'CIR'(圆曲线), 'BLO'(过渡段)
    params: 若'CIR'则(params[0]=R)，若'BLO'则(params=(R1, R2))
    """
    if stype == 'STR':
        return (0.0, 0.0, 0.0)
    elif stype == 'CIR':
        R = params[0]
        k = 1.0/R if abs(R) > 1e-12 else 0.0
        return (k, 0.0, 0.0)
    elif stype == 'BLO':
        R1, R2 = params
        k   = bloss_kappa_raw(srel, Lseg, R1, R2)
        kp  = bloss_kappa_deriv_raw(srel, Lseg, R1, R2)
        kpp = bloss_kappa_2deriv_raw(srel, Lseg, R1, R2)
        return (k, kp, kpp)
    else:
        return (0.0, 0.0, 0.0)

def u_raw_formula(stype, Lseg, srel, params):
    """
    给定分段类型 stype, 返回该分段在局部里程srel处的超高u, u', u''.
    stype: 'CST'(常值), 'BLO'(BLOSS过渡), 'CIR'(同CST,保持不变)
    params: 若'CST'则(params[0]=u0)，若'BLO'则(params=(u1, u2))
    """
    if stype == 'CST':
        return (params[0], 0.0, 0.0)
    elif stype == 'BLO':
        u1, u2 = params
        uu  = bloss_u_raw(srel, Lseg, u1, u2)
        up  = bloss_u_deriv_raw(srel, Lseg, u1, u2)
        upp = bloss_u_2deriv_raw(srel, Lseg, u1, u2)
        return (uu, up, upp)
    elif stype == 'CIR':
        # 也可视同常值
        return (params[0], 0.0, 0.0)
    else:
        return (0.0, 0.0, 0.0)

def build_s_bounds(segments):
    """
    给定 segments = [(stype, L, ...), ...]，
    计算并返回累积里程边界列表 s_bounds = [0.0, L1, L1+L2, ...].
    """
    s_bounds = [0.0]
    for seg in segments:
        L_ = seg[1]
        s_bounds.append(s_bounds[-1] + L_)
    return s_bounds

def poly5_val(x, c):
    """
    5次多项式 p(x) = c0 + c1*x + c2*x^2 + c3*x^3 + c4*x^4 + c5*x^5
    """
    return c[0] + c[1]*x + c[2]*x**2 + c[3]*x**3 + c[4]*x**4 + c[5]*x**5

def poly5_d1(x, c):
    """
    5次多项式p(x)的一阶导数 p'(x)
    """
    return (c[1]
            + 2*c[2]*x
            + 3*c[3]*x**2
            + 4*c[4]*x**3
            + 5*c[5]*x**4)

def poly5_d2(x, c):
    """
    5次多项式p(x)的二阶导数 p''(x)
    """
    return (2*c[2]
            + 6*c[3]*x
            + 12*c[4]*x**2
            + 20*c[5]*x**3)

def solve_poly5_smoothing(kL, kLp, kLpp, kR, kRp, kRpp, dist):
    """
    在区间[0, dist]上用5次多项式插值：
       p(0)=kL, p'(0)=kLp, p''(0)=kLpp,
       p(dist)=kR, p'(dist)=kRp, p''(dist)=kRpp.
    返回系数 c[0..5].
    """
    M = np.zeros((6,6), dtype=float)
    b = np.zeros(6, dtype=float)

    def p_vec(x):
        return np.array([1, x, x**2, x**3, x**4, x**5], dtype=float)
    def dp_vec(x):
        return np.array([0, 1, 2*x, 3*x**2, 4*x**3, 5*x**4], dtype=float)
    def ddp_vec(x):
        return np.array([0, 0, 2, 6*x, 12*x**2, 20*x**3], dtype=float)

    # 左端条件
    M[0,:] = p_vec(0.0);   b[0] = kL
    M[1,:] = dp_vec(0.0);  b[1] = kLp
    M[2,:] = ddp_vec(0.0); b[2] = kLpp

    # 右端条件
    M[3,:] = p_vec(dist);   b[3] = kR
    M[4,:] = dp_vec(dist);  b[4] = kRp
    M[5,:] = ddp_vec(dist); b[5] = kRpp

    coefs = np.linalg.solve(M, b)
    return coefs

def build_piecewise_function(segments, s_bounds, raw_func, L_smo=3.0):
    """
    通用的“分段 + smoothing(5次多项式)”拼接器。

    segments: [(stype, L, param1, param2, ...), ...]
    s_bounds: 对应分段的累积里程 [s0, s1, s2, ...]
    raw_func: 给定分段原始公式接口, e.g. kappa_raw_formula / u_raw_formula
    L_smo: 平滑段总长度(默认3.0 -> 前后各1.5m)

    返回: pieces(list)，可供 eval_piecewise() 使用。
    """
    half_smo = L_smo * 0.5
    num_segs = len(segments)
    pieces = []

    for i in range(num_segs):
        s0 = s_bounds[i]
        s1 = s_bounds[i+1]
        Lseg = s1 - s0

        has_left_smo  = (i > 0)            # 是否有上一段
        has_right_smo = (i < num_segs - 1) # 是否有下一段

        # 1) 左平滑段
        if has_left_smo and (Lseg > half_smo):
            seg_prev = segments[i-1]
            stype0   = seg_prev[0]
            L0       = seg_prev[1]
            params0  = seg_prev[2:]
            # 上一段末尾(局部s=L0)
            kL, kLp, kLpp = raw_func(stype0, L0, L0, params0)

            seg_curr = segments[i]
            stype1   = seg_curr[0]
            L1       = seg_curr[1]
            params1  = seg_curr[2:]
            # 本段开头(局部s=half_smo)
            kR, kRp, kRpp = raw_func(stype1, L1, half_smo, params1)

            dist = half_smo
            coefs = solve_poly5_smoothing(kL, kLp, kLpp, kR, kRp, kRpp, dist)

            pieces.append({
                's_min': s0,
                's_max': s0 + half_smo,
                'type': 'poly_smooth',
                'coefs': coefs,
                'dist': dist
            })
            left_raw_start = s0 + half_smo
        else:
            left_raw_start = s0

        # 2) 中间原始raw段
        if has_right_smo and (Lseg > half_smo):
            right_raw_end = s1 - half_smo
        else:
            right_raw_end = s1

        if right_raw_end > left_raw_start + 1e-12:
            pieces.append({
                's_min': left_raw_start,
                's_max': right_raw_end,
                'type': 'raw',
                'seg_id': i
            })

        # 3) 右平滑段
        if has_right_smo and (Lseg > half_smo):
            seg_curr = segments[i]
            stype1   = seg_curr[0]
            L1       = seg_curr[1]
            params1  = seg_curr[2:]
            # 本段末尾(局部s=L1 - half_smo)
            kL, kLp, kLpp = raw_func(stype1, L1, L1 - half_smo, params1)

            seg_next = segments[i+1]
            stype2   = seg_next[0]
            L2       = seg_next[1]
            params2  = seg_next[2:]
            # 下一段开头(局部s=0)
            kR, kRp, kRpp = raw_func(stype2, L2, 0.0, params2)

            dist = half_smo
            coefs = solve_poly5_smoothing(kL, kLp, kLpp, kR, kRp, kRpp, dist)

            pieces.append({
                's_min': s1 - half_smo,
                's_max': s1,
                'type': 'poly_smooth',
                'coefs': coefs,
                'dist': dist
            })

    return pieces

def eval_piecewise(s, pieces, segments, raw_func):
    """
    在给定的 pieces 范围内，对任意 s 计算“曲率”或“超高”等值 (只返回0阶值)。
    pieces: build_piecewise_function 的返回
    segments: 原始分段定义
    raw_func: 对应 kappa_raw_formula / u_raw_formula
    """
    # 超出范围处理(这里直接返回0.0，也可根据需求抛异常或别的策略)
    if s < 0:
        return 0.0
    s_max_all = max(p['s_max'] for p in pieces)
    if s > s_max_all:
        return 0.0

    # 找到piece
    for p in pieces:
        if p['s_min'] <= s <= p['s_max'] + 1e-12:
            if p['type'] == 'raw':
                i_seg = p['seg_id']
                # 找到该分段的全局起点
                seg_start_global = sum(seg[1] for seg in segments[:i_seg])
                seg_length       = segments[i_seg][1]
                stype           = segments[i_seg][0]
                params          = segments[i_seg][2:]
                # 计算局部里程
                srel = s - seg_start_global
                if srel < 0:
                    srel = 0
                if srel > seg_length:
                    srel = seg_length
                val3 = raw_func(stype, seg_length, srel, params)
                return val3[0]  # 只返回 kappa 或 u 的数值
            else:
                # poly_smooth
                coefs = p['coefs']
                dist  = p['dist']
                x_    = s - p['s_min']
                return poly5_val(x_, coefs)

    # 理论上不应到这里
    return 0.0

def generate_trajectory(h_segments, u_segments, L_smo, b_ref, ds):
    """
    生成轨道3D坐标和左右股钢轨，同时**返回**可调用的Kappa(s)和U(s)。
      参数:
        h_segments, u_segments: 水平/超高段定义
        L_smo:     平滑段总长
        b_ref:     轨距(中心线->钢轨)
        ds:        积分步长

    返回:
      1) trajectory_data: dict, 包含 's','x','y','z','psi','phi','left_rail','right_rail'
      2) Kappa(s):       水平曲率的函数(可直接在外部用 Kappa(某s) 调用)
      3) U(s):           超高的函数(可直接在外部用 U(某s) 调用)
    """

    # =========== 1) 建立分段边界并构造 piecewise ============
    h_s_bounds = build_s_bounds(h_segments)  # 水平段里程边界
    u_s_bounds = build_s_bounds(u_segments)  # 超高段里程边界

    # 构建“曲率”piecewise
    h_pieces = build_piecewise_function(
        h_segments, 
        h_s_bounds,
        raw_func=kappa_raw_formula, 
        L_smo=L_smo
    )
    # 构建“超高”piecewise
    u_pieces = build_piecewise_function(
        u_segments, 
        u_s_bounds,
        raw_func=u_raw_formula, 
        L_smo=L_smo
    )

    # =========== 2) 定义 Kappa(s), U(s) 给外部调用 ============
    def Kappa(s):
        return eval_piecewise(s, h_pieces, h_segments, kappa_raw_formula)

    def U(s):
        return eval_piecewise(s, u_pieces, u_segments, u_raw_formula)

    # =========== 3) 数值积分 x(s), y(s), z(s), psi(s), phi(s) ============
    s_end = h_s_bounds[-1]  # 轨道总长
    s_vals = np.arange(0, s_end + ds*0.1, ds)

    xvals = [0.0]
    yvals = [0.0]
    zvals = [0.0]  # 无坡度 -> z=0
    psi   = 0.0
    psi_vals = [psi]

    for i in range(1, len(s_vals)):
        sA = s_vals[i-1]
        sB = s_vals[i]
        sM = 0.5*(sA + sB)
        kM = Kappa(sM)
        ds_ = (sB - sA)

        # 更新航向角
        psi_new = psi + kM * ds_

        # x,y积分 (中点法)
        x_new = xvals[-1] + ds_ * np.cos(psi + 0.5*kM*ds_)
        y_new = yvals[-1] + ds_ * np.sin(psi + 0.5*kM*ds_)

        xvals.append(x_new)
        yvals.append(y_new)
        zvals.append(zvals[-1])  # 没有坡度
        psi_vals.append(psi_new)
        psi = psi_new

    xvals = np.array(xvals)
    yvals = np.array(yvals)
    zvals = np.array(zvals)
    psi_vals = np.array(psi_vals)

    # 计算滚转角phi(s)
    phi_vals = []
    for s_ in s_vals:
        u_ = U(s_)
        phi_ = np.arcsin(u_ / b_ref)
        phi_vals.append(phi_)
    phi_vals = np.array(phi_vals)

    # =========== 4) 计算左右钢轨坐标 ============
    left_rail  = []
    right_rail = []
    for i in range(len(s_vals)):
        xC = xvals[i]
        yC = yvals[i]
        zC = zvals[i]

        # 当前点的航向角 psi_
        if i == 0:
            psi_ = psi_vals[0]
        else:
            # 也可以直接用 psi_vals[i],
            # 这里示范用局部差分
            psi_ = np.arctan2(yvals[i] - yvals[i-1],
                              xvals[i] - xvals[i-1])
        phi_ = phi_vals[i]

        half_b = 0.5 * b_ref
        z_shift = half_b * np.sin(phi_)

        # 左股轨道(相对中心线左侧 half_b)
        dxL =  half_b * (-np.sin(psi_))
        dyL =  half_b * ( np.cos(psi_))
        xL  =  xC + dxL
        yL  =  yC + dyL
        zL  =  zC + z_shift

        # 右股轨道(相对中心线右侧 half_b)
        dxR = -half_b * (-np.sin(psi_))
        dyR = -half_b * ( np.cos(psi_))
        xR  =  xC + dxR
        yR  =  yC + dyR
        zR  =  zC - z_shift

        left_rail.append((xL, yL, zL))
        right_rail.append((xR, yR, zR))

    left_rail  = np.array(left_rail)
    right_rail = np.array(right_rail)

    # =========== 5) 封装输出 ============
    trajectory_data = {
        's': s_vals.tolist(),
        'x': xvals.tolist(),
        'y': yvals.tolist(),
        'z': zvals.tolist(),
        'psi': psi_vals.tolist(),
        'phi': phi_vals.tolist(),
        'left_rail': left_rail.tolist(),
        'right_rail': right_rail.tolist()
    }

    # 返回 (trajectory_data, Kappa, U)
    return trajectory_data, Kappa, U

def generate_trajectory_withVertical(h_segments, u_segments, v_segments, 
                                     L_smo, b_ref, ds, z0=0.0):
    """
    在已有的 generate_trajectory 基础上，增加对“竖曲线段” v_segments 的处理。
    """

    # =========== A) 先构建水平曲率的 piecewise ============
    h_s_bounds = build_s_bounds(h_segments)
    h_pieces = build_piecewise_function(
        h_segments, 
        h_s_bounds,
        raw_func=kappa_raw_formula, 
        L_smo=L_smo
    )
    def Kappa(s):
        return eval_piecewise(s, h_pieces, h_segments, kappa_raw_formula)

    # =========== B) 先构建超高的 piecewise ============
    u_s_bounds = build_s_bounds(u_segments)
    u_pieces = build_piecewise_function(
        u_segments,
        u_s_bounds,
        raw_func=u_raw_formula,
        L_smo=L_smo
    )
    def U(s):
        return eval_piecewise(s, u_pieces, u_segments, u_raw_formula)

    # =========== C) 构建竖向坡度的 no-smoothing piecewise ============
    v_pieces, v_s_bounds = build_vertical_pieces_no_smoothing(v_segments)
    def Slope(s):
        return eval_piecewise_vertical(s, v_pieces, v_segments)

    # =========== D) 数值积分 x,y,z, psi, phi ============
    s_end = h_s_bounds[-1]  # 假设水平段的总长==竖向段的总长
    s_vals = np.arange(0, s_end+ds*0.1, ds)

    # 初值
    xvals = [0.0]
    yvals = [0.0]
    zvals = [z0]   # 允许一个 z0 起始高度
    psi   = 0.0
    psi_vals = [psi]

    curr_z = z0

    for i in range(1, len(s_vals)):
        sA  = s_vals[i-1]
        sB  = s_vals[i]
        sM  = 0.5*(sA + sB)
        ds_ = (sB - sA)

        # 1) 航向角
        kM       = Kappa(sM)
        psi_new  = psi + kM*ds_

        # 2) x,y 积分(中点法)
        x_new = xvals[-1] + ds_ * np.cos(psi + 0.5*kM*ds_)
        y_new = yvals[-1] + ds_ * np.sin(psi + 0.5*kM*ds_)

        # 3) z 积分(中点法)
        pM = Slope(sM)
        z_new = curr_z + pM*ds_

        # 更新
        xvals.append(x_new)
        yvals.append(y_new)
        zvals.append(z_new)
        psi_vals.append(psi_new)

        psi = psi_new
        curr_z = z_new

    xvals = np.array(xvals)
    yvals = np.array(yvals)
    zvals = np.array(zvals)
    psi_vals = np.array(psi_vals)

    # 4) 计算滚转角 phi = arcsin(u/b_ref)
    phi_vals = []
    for s_ in s_vals:
        u_ = U(s_)
        phi_ = np.arcsin(u_ / b_ref)
        phi_vals.append(phi_)
    phi_vals = np.array(phi_vals)

    # 5) 计算左右股钢轨坐标
    left_rail = []
    right_rail = []
    for i in range(len(s_vals)):
        xC = xvals[i]
        yC = yvals[i]
        zC = zvals[i]

        if i == 0:
            psi_ = psi_vals[0]
        else:
            # 也可以直接 psi_ = psi_vals[i]
            psi_ = np.arctan2(yvals[i]-yvals[i-1],
                              xvals[i]-xvals[i-1])
        phi_ = phi_vals[i]

        half_b = 0.5 * b_ref
        z_shift = half_b * np.sin(phi_)

        # 左股
        dxL =  half_b * (-np.sin(psi_))
        dyL =  half_b * ( np.cos(psi_))
        xL  =  xC + dxL
        yL  =  yC + dyL
        zL  =  zC + z_shift

        # 右股
        dxR = -half_b * (-np.sin(psi_))
        dyR = -half_b * ( np.cos(psi_))
        xR  =  xC + dxR
        yR  =  yC + dyR
        zR  =  zC - z_shift

        left_rail.append((xL, yL, zL))
        right_rail.append((xR, yR, zR))

    left_rail  = np.array(left_rail)
    right_rail = np.array(right_rail)

    # 6) 封装输出
    trajectory_data = {
        's': s_vals.tolist(),
        'x': xvals.tolist(),
        'y': yvals.tolist(),
        'z': zvals.tolist(),
        'psi': psi_vals.tolist(),
        'phi': phi_vals.tolist(),
        'left_rail': left_rail.tolist(),
        'right_rail': right_rail.tolist()
    }

    # 返回 (trajectory_data, Kappa, U, Slope)
    return trajectory_data, Kappa, U, Slope

# 从SIMPACK文件读取轨道数据的函数
def read_track_data(filename):
    # 用于存储数据的列表
    splined_track_x = []
    splined_track_y = []
    
    try:
        # 跳过前4行，从第5行开始读取数据
        with open(filename, 'r') as f:
            # 跳过前4行
            for _ in range(4):
                next(f)
            
            # 读取其余行
            for line in f:
                parts = line.strip().split(',')
                if len(parts) >= 2:  # 确保行至少有2列
                    try:
                        # 尝试转换为浮点数，对空字符串使用默认值
                        x_val = float(parts[0]) if parts[0].strip() else 0.0
                        y_val = float(parts[1]) if parts[1].strip() else 0.0
                        
                        # 检查值是否为极小的科学计数法（可能是无效数据）
                        if abs(y_val) < 1e-100:
                            y_val = 0.0
                        if abs(x_val) < 1e-100:
                            x_val = 0.0
                            
                        splined_track_x.append(x_val)
                        splined_track_y.append(y_val)
                    except ValueError:
                        # 如果无法转换，则跳过该行，但不打印错误信息
                        continue
    except Exception as e:
        print(f"读取文件时出错: {e}")
        return np.array([]), np.array([])
    
    return np.array(splined_track_x), np.array(splined_track_y)

def vert_slope_raw_formula(vtype, Lseg, srel, params):
    """
    给定分段类型 vtype ('CSL'/'PL2')，
    在该分段局部里程 srel 处，返回:
      p(s)   = 坡度,
      p'(s)  = d(p)/ds,
      p''(s) = d^2(p)/ds^2.
    """
    if vtype == 'CSL':
        # params = (p,)
        p_ = params[0]
        return (p_, 0.0, 0.0)

    elif vtype == 'PL2':
        # params = (p1, p2)
        p1, p2 = params
        x  = srel / Lseg
        p_   = p1 + (p2 - p1)*x
        dp_  = (p2 - p1)/Lseg
        ddp_ = 0.0  # 二阶导数为0
        return (p_, dp_, ddp_)

    else:
        # 其他类型(本示例不处理)
        return (0.0, 0.0, 0.0)

# 从Excel文件中读取轨道分段信息，并构建水平段、超高段、竖向坡度段列表
# 在此处将竖曲线曲率正负翻转，以匹配 SIMPACK 坐标
def read_track_segments(file_path, sheet_name="自定义线路"):
    """
    从Excel文件中读取轨道分段信息，并构建水平段、超高段、竖向坡度段列表。
    返回: (h_segments, u_segments, v_segments)
    """
    # 读取Excel文件
    try:
        df = pd.read_excel(file_path, sheet_name=sheet_name)
    except Exception as e:
        print(f"读取Excel文件时出错: {e}")
        return [], [], []
    
    h_segments = []
    u_segments = []
    v_segments = []

    for idx, row in df.iterrows():
        try:
            # ================== 1) 处理水平 h_segments ==================
            h_type = row['平面线路类型']
            if pd.isna(h_type):
                pass
            else:
                h_type = str(h_type).strip().upper()
                if h_type == 'STR':
                    # STR 需要1个参数(长度)
                    if not pd.isna(row['Par1']):
                        h_segments.append((h_type, float(row['Par1'])))
                elif h_type == 'BLO':
                    # BLO 需要3个参数(长度, R1, R2) => Par1, Par2, Par3
                    if (not pd.isna(row['Par1']) and 
                        not pd.isna(row['Par2']) and 
                        not pd.isna(row['Par3'])):
                        h_segments.append((h_type, 
                                          float(row['Par1']), 
                                          float(row['Par2']), 
                                          float(row['Par3'])))
                elif h_type == 'CIR':
                    # CIR 需要2个参数(长度, R)
                    if (not pd.isna(row['Par1']) and 
                        not pd.isna(row['Par2'])):
                        h_segments.append((h_type, 
                                          float(row['Par1']), 
                                          float(row['Par2'])))

            # ================== 2) 处理超高 u_segments ==================
            u_type = row['超高线路类型']
            if pd.isna(u_type):
                pass
            else:
                u_type = str(u_type).strip().upper()
                if u_type == 'CST':
                    # CST 需要2个参数(H_Par1长度, H_Par2超高)
                    if not pd.isna(row['H_Par1']) and not pd.isna(row['H_Par2']):
                        u_segments.append((u_type,
                                           float(row['H_Par1']),
                                           float(row['H_Par2'])))
                elif u_type == 'BLO':
                    # BLO 需要3个参数(H_Par1长度, H_Par2, H_Par3)
                    if (not pd.isna(row['H_Par1']) and 
                        not pd.isna(row['H_Par2']) and 
                        not pd.isna(row['H_Par3'])):
                        u_segments.append((u_type,
                                           float(row['H_Par1']),
                                           float(row['H_Par2']),
                                           float(row['H_Par3'])))

            # ================== 3) 处理竖向 v_segments ==================
            v_type = row['竖曲线类型']
            if pd.isna(v_type):
                pass
            else:
                v_type = str(v_type).strip().upper()  # 'CSL' or 'PL2'
                # 长度 Z_Par1
                Zp1 = row['Z_Par1'] if not pd.isna(row['Z_Par1']) else 0.0
                Zp2 = row['Z_Par2'] if not pd.isna(row['Z_Par2']) else 0.0
                Zp3 = row['Z_Par3'] if not pd.isna(row['Z_Par3']) else 0.0

                if v_type == 'CSL':
                    # 只需要 (length=Zp1, slope=Zp2)
                    # 如果您在表格中是 "Z_Par1 = 段长, Z_Par2 = 斜率" 的形式
                    L_  = float(Zp1)
                    p_  = -float(Zp2)   # 改为取负 p_  = float(Zp2)
                    v_segments.append((v_type, L_, p_))
                elif v_type == 'PL2':
                    # 需要 (length=Zp1, p1=Zp2, p2=Zp3)
                    L_   = float(Zp1)
                    p1_  = -float(Zp2)   # 改为取负 p1_  = float(Zp2)
                    p2_  = -float(Zp3)   # 改为取负 p2_  = float(Zp3)
                    v_segments.append((v_type, L_, p1_, p2_))

        except Exception as e:
            print(f"处理第 {idx+2} 行时出错: {e}")
    
    return h_segments, u_segments, v_segments

def build_vertical_pieces_no_smoothing(v_segments):
    """
    不做五次多项式衔接，直接按段划分。
    返回一个 pieces 列表，每段只含 'type':'raw', 'seg_id':i, 's_min','s_max'。
    """
    s_bounds = build_s_bounds(v_segments)
    pieces = []
    for i in range(len(v_segments)):
        s0 = s_bounds[i]
        s1 = s_bounds[i+1]
        pieces.append({
            's_min': s0,
            's_max': s1,
            'type': 'raw',
            'seg_id': i
        })
    return pieces, s_bounds

def eval_piecewise_vertical(s, pieces, v_segments):
    """
    在给定的 pieces 范围内，对任意 s 计算坡度p(s) (只返回0阶值)。
    不做段与段之间的平滑过渡，段交界直接跳变。
    """
    if s < 0:
        return 0.0
    s_max_all = max(p['s_max'] for p in pieces)
    if s > s_max_all:
        return 0.0

    for p in pieces:
        if p['s_min'] <= s <= p['s_max']:
            i_seg = p['seg_id']
            seg_start = sum(seg[1] for seg in v_segments[:i_seg])  # 该段起点全局S
            Lseg      = v_segments[i_seg][1]
            vtype     = v_segments[i_seg][0]
            params    = v_segments[i_seg][2:]
            srel      = s - seg_start
            if srel < 0:
                srel = 0
            if srel > Lseg:
                srel = Lseg
            
            # 调用 vert_slope_raw_formula
            val3 = vert_slope_raw_formula(vtype, Lseg, srel, params)
            return val3[0]  # 只返回 p(s)
    
    return 0.0

def read_track_Zs_data(filename):
    """
    从 SIMPACK 导出的竖曲线文本文件(如 'TrkZs_sides_VirtualLine.txt') 中，
    读取中心线、左右钢轨的 (s,z) 数据。文件格式示例：
    
    "Layout"
    "z(s)"
    "spline (center)_x","spline (center)_y","_x","_y","spline (right)_x","spline (right)_y","_x","_y","spline (left)_x","spline (left)_y","_x","_y"
    "[]","[]","[]","[]","[]","[]","[]","[]","[]","[]","[]","[]"
    0,0,0,-0,0,0,0,0,0,0,0,-0
    0.330502,0,0.998328,-0,0.330502,6.0392e-178,0.998328,0,0.330502,-6.0392e-178,0.998328,-0
    ...
    
    其中我们只关心12列中的这几列：
      col0 => spline(center)_x 作为 center_s
      col1 => spline(center)_y 作为 center_z
      col4 => spline(right)_x  作为 right_s
      col5 => spline(right)_y  作为 right_z
      col8 => spline(left)_x   作为 left_s
      col9 => spline(left)_y   作为 left_z
    
    返回:
      (center_s, center_z, right_s, right_z, left_s, left_z),
      均为 np.array 类型。
    """

    center_s = []
    center_z = []
    right_s  = []
    right_z  = []
    left_s   = []
    left_z   = []

    try:
        with open(filename, 'r', encoding='utf-8') as f:
            lines = f.readlines()
    except Exception as e:
        print(f"读取文件失败: {e}")
        # 返回空数组
        return (np.array([]), np.array([]), 
                np.array([]), np.array([]),
                np.array([]), np.array([]))

    if len(lines) < 5:
        # 至少需要跳过4行 header
        print("文件行数太少，无法解析。")
        return (np.array([]), np.array([]), 
                np.array([]), np.array([]),
                np.array([]), np.array([]))

    # 跳过前4行(示例中包含 "Layout", "z(s)", 表头行, 以及全是 "[]" 的行)
    data_lines = lines[4:]  # 从第5行开始都是数值

    for line_idx, line in enumerate(data_lines, start=5):
        # 去掉首尾空白
        line = line.strip()
        if not line:
            # 空行，跳过
            continue

        parts = line.split(',')
        # 理论上应有12列
        if len(parts) < 12:
            # 行内列数不够，跳过
            continue

        # 试着解析我们关心的那几列：
        try:
            # 0,1 => center s,z
            s_c  = float(parts[0]) if parts[0].strip() else 0.0
            z_c  = float(parts[1]) if parts[1].strip() else 0.0
            # 4,5 => right s,z
            s_r  = float(parts[4]) if parts[4].strip() else 0.0
            z_r  = float(parts[5]) if parts[5].strip() else 0.0
            # 8,9 => left s,z
            s_l  = float(parts[8]) if parts[8].strip() else 0.0
            z_l  = float(parts[9]) if parts[9].strip() else 0.0

            # 检查极小数值(近似0)可视需求处理，这里简单保留或强制设为0.0也可
            # if abs(z_c) < 1e-100: z_c = 0.0
            # ...

            center_s.append(s_c)
            center_z.append(z_c)
            right_s.append(s_r)
            right_z.append(z_r)
            left_s.append(s_l)
            left_z.append(z_l)

        except ValueError:
            # 某列无法解析为浮点数，跳过
            continue

    # 转成 numpy 数组返回
    return (np.array(center_s), np.array(center_z),
            np.array(right_s),  np.array(right_z),
            np.array(left_s),   np.array(left_z))







