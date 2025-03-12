# tools_SPCKTrk.py

import numpy as np

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
