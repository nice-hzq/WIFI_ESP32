#tool.py - 通用工具函数
import numpy as np
from core.quaternion import quat_to_rotmat
from core.math_utils import (safe_float, safe_div, moving_average,
                              moving_avg_1d, lowpass_1d, unwrap_deg,
                              angdiff_deg, clean_boolean_runs,
                              filter_acceleration)
from scipy.signal import detrend, medfilt, filtfilt, butter

# Aliases for backward compatibility with underscore-prefixed callers
_unwrap_deg = unwrap_deg
_angdiff_deg = angdiff_deg
_moving_average = moving_average
_lowpass_1d = lowpass_1d
_clean_boolean_runs = clean_boolean_runs

def _steps_from_events(HS_idx, TO_idx):
    """
    将 (TO -> HS_next) 配对为摆动期区间。返回 [(start,end), ...]，均为索引（闭区间）。
    要求 TO 和 HS 单调递增。
    """
    if len(HS_idx) == 0 or len(TO_idx) == 0:
        return []
    swing_windows = []
    j = 0
    for i in range(len(HS_idx)-1):  # HS[i] -> HS[i+1] 是一个步循环
        hs_curr = HS_idx[i]
        hs_next = HS_idx[i+1]
        # 在 (hs_curr, hs_next) 内寻找唯一一个 TO
        while j < len(TO_idx) and TO_idx[j] <= hs_curr:
            j += 1
        if j < len(TO_idx) and hs_curr < TO_idx[j] < hs_next:
            swing_windows.append((TO_idx[j], hs_next))
    return swing_windows

def _foot_clearance_series(pos_nav, HS_idx, TO_idx, z_up=True):
    """
    计算每步摆动期内的最高“足部高度”(Z)。返回 per-step 列表(单位与pos一致，通常m) 与均值。
    pos_nav: (N,3) 由 ZUPT 积分得到的位置（导航系）
    z_up   : True 表示 pos[:,2] 为“朝上”(+Z)；否则取负。
    """
    if pos_nav is None or len(pos_nav) == 0:
        return [], np.nan
    z = pos_nav[:, 2].copy()
    if not z_up:
        z = -z
    swings = _steps_from_events(HS_idx, TO_idx)
    series = []
    for (start, end) in swings:
        start = max(0, start); end = min(len(z)-1, end)
        if end > start:
            series.append(float(np.max(z[start:end+1])))
    mean_val = float(np.mean(series)) if len(series) else np.nan
    return series, mean_val

def _foot_impact_from_acc(acc_nav_lin, HS_idx, fs, window_ms=(20, 60),
                          use_vertical=True, body_mass_kg=None):

    if acc_nav_lin is None or len(acc_nav_lin) == 0 or len(HS_idx) == 0:
        return [], float("nan"), [] if body_mass_kg is not None else None, float("nan")

    N = acc_nav_lin.shape[0]
    g0 = 9.80665
    pre  = int(max(0,  window_ms[0] * 1e-3 * fs))
    post = int(max(1, window_ms[1] * 1e-3 * fs))

    # === 构造“总加速度”（含重力）信号 ===
    if use_vertical:
        # 竖直总加速度：a_z_total = a_z_lin + g0
        sig = acc_nav_lin[:, 2] + g0
    else:
        # 合加速度总模长：|a_lin + g_vec|，g_vec = [0, 0, g0]
        g_vec = np.array([0.0, 0.0, g0], dtype=float)
        sig = np.linalg.norm(acc_nav_lin + g_vec[None, :], axis=1)

    peaks_g = []
    forces_N = [] if body_mass_kg is not None else None

    for hs in HS_idx:
        a = max(0, hs - pre)
        b = min(N - 1, hs + post)
        if b <= a:
            continue

        # 这里用“总加速度”的峰值（一般为正），通常不再取绝对值
        peak_total = float(np.max(sig[a:b+1]))  # 单位 m/s^2

        # 1) 每步 Z 轴总加速度峰值(单位 g)
        peaks_g.append(peak_total / g0)

        # 2) 每步冲击力（若提供体重）
        if body_mass_kg is not None:
            # 近似地面反力：F ≈ m * a_total_peak
            forces_N.append(float(body_mass_kg * peak_total))

    mean_peak_g = float(np.mean(peaks_g)) if len(peaks_g) else float("nan")
    if forces_N is not None and len(forces_N):
        mean_force = float(np.mean(forces_N))
    else:
        mean_force = float("nan")

    return peaks_g, mean_peak_g, forces_N, mean_force

def _smooth_diff(X, win=31):
    """对二维轨迹 X(N,2) 求差分方向并滑动平均，得到每帧的平滑前进方向单位向量 f(N,2)。"""
    if X.ndim != 2 or X.shape[1] != 2 or len(X) < 3:
        # 退化：给个固定向前
        f = np.zeros_like(X); f[:, 0] = 1.0
        return f
    V = np.zeros_like(X)
    V[1:-1] = X[2:] - X[:-2]
    V[0] = X[1] - X[0]
    V[-1] = X[-1] - X[-2]
    # 简单滑窗
    k = max(3, int(win) | 1)  # 奇数窗
    pad = k // 2
    Vpad = np.pad(V, ((pad, pad), (0, 0)), mode='edge')
    ker = np.ones((k, 1)) / k
    Vsm = np.convolve(Vpad[:, 0], ker[:, 0], mode='valid')  # x
    Wsm = np.convolve(Vpad[:, 1], ker[:, 0], mode='valid')  # y
    F = np.stack([Vsm, Wsm], axis=1)
    n = np.linalg.norm(F, axis=1, keepdims=True) + 1e-12
    return F / n

def _smooth_heading_from_midpos(mid_xy, win=31):
    """由中点轨迹 mid_xy(N,2) 估计平滑前进方向的 yaw 角(°)。"""
    if mid_xy is None or len(mid_xy) < 3:
        return None
    V = np.zeros_like(mid_xy)
    V[1:-1] = mid_xy[2:] - mid_xy[:-2]
    V[0]  = mid_xy[1] - mid_xy[0]
    V[-1] = mid_xy[-1] - mid_xy[-2]
    # 简单滑窗
    k = max(3, int(win) | 1); pad = k // 2
    Vpad = np.pad(V, ((pad, pad), (0, 0)), mode='edge')
    ker = np.ones((k,)) / k
    vx = np.convolve(Vpad[:, 0], ker, mode='valid')
    vy = np.convolve(Vpad[:, 1], ker, mode='valid')
    return np.degrees(np.arctan2(vy, vx))

def _compute_step_width(pos_left, pos_right, HS_left, HS_right, use_local_heading=True):
    """
    计算步宽（单位与 pos 一致，通常是米）。
    pos_left/pos_right: (N,3) 位置（导航系，Z向上）
    HS_left/HS_right  : 左/右足跟着地事件索引（升序）
    use_local_heading : True=用行走方向（由中点轨迹估计）来定义侧向轴；False=用全局Y轴为侧向

    返回:
      step_width_series_m: 每次 HS 的步宽（左右交替合并）
      step_width_mean_m  : 均值
      step_width_std_m   : 标准差
      step_width_L_m     : 仅在左HS时的步宽序列
      step_width_R_m     : 仅在右HS时的步宽序列
    """
    if pos_left is None or pos_right is None or len(pos_left) == 0 or len(pos_right) == 0:
        return np.array([]), np.nan, np.nan, np.array([]), np.array([])

    N = min(len(pos_left), len(pos_right))
    PL = pos_left[:N]
    PR = pos_right[:N]

    # 计算“行走方向”（前进轴）与“侧向轴”
    if use_local_heading:
        mid = 0.5 * (PL[:, :2] + PR[:, :2])         # 水平中线轨迹
        fwd = _smooth_diff(mid, win=31)              # (N,2) 平滑前进方向单位向量
        lat = np.stack([-fwd[:,1], fwd[:,0]], axis=1)  # 侧向轴 = 90° 旋转
    else:
        # 假设全局坐标 Y 为侧向轴
        lat = np.tile(np.array([0.0, 1.0]), (N, 1))

    # 在每个 HS 时刻计算：|(PL-PR) 在侧向轴上的投影|
    def _series_at_events(HS):
        vals = []
        for h in HS:
            if 0 <= h < N:
                d_xy = (PL[h, :2] - PR[h, :2])              # 左右足在该时刻的水平差
                lat_h = lat[h]                               # 侧向单位向量
                w = abs(float(np.dot(d_xy, lat_h)))          # 侧向投影的绝对值 = 步宽
                vals.append(w)
        return np.array(vals, dtype=float)

    sw_L = _series_at_events(HS_left)
    sw_R = _series_at_events(HS_right)

    # 合并序列（保持时间顺序）
    # 这里简单拼接后排序（如果需要严格交替，可按事件时间合并）
    times = np.concatenate([HS_left, HS_right]) if len(HS_left)+len(HS_right) > 0 else np.array([], int)
    vals  = np.concatenate([sw_L, sw_R])        if len(sw_L)+len(sw_R) > 0 else np.array([], float)
    if len(times) == len(vals) and len(vals) > 0:
        order = np.argsort(times)
        sw_all = vals[order]
    else:
        sw_all = np.array([], float)

    mean_w = float(np.nanmean(sw_all)) if sw_all.size else np.nan
    std_w  = float(np.nanstd(sw_all))  if sw_all.size else np.nan

    return sw_all, mean_w, std_w, sw_L, sw_R

def _horizontal_dist(p1, p2, use_z=False):
    d = p2 - p1
    return float(np.linalg.norm(d if use_z else d[:2]))

def _pairwise_step_lengths_from_HS(pos, HS_idx):
    """同脚HS序列 → 跨步长(Stride)；相邻左右HS交替 → 步幅(Step) 由外部组合。"""
    stride = []
    for k in range(len(HS_idx) - 1):
        i0, i1 = HS_idx[k], HS_idx[k+1]
        stride.append(_horizontal_dist(pos[i0], pos[i1], use_z=False))
    return np.array(stride)
def _interleave_step_lengths(pos_left, HS_left, pos_right, HS_right):
    """左右脚交替HS → 步幅序列（左→右、右→左）。"""
    # 合并所有HS并按时间排序，标注左右
    tags = [("L", i) for i in HS_left] + [("R", i) for i in HS_right]
    tags.sort(key=lambda x: x[1])
    step_lengths = []
    step_sides = []
    for (s0, i0), (s1, i1) in zip(tags[:-1], tags[1:]):
        # 只统计 L->R 或 R->L 的相邻对
        if s0 == s1:
            continue
        p0 = pos_left[i0] if s0 == "L" else pos_right[i0]
        p1 = pos_left[i1] if s1 == "L" else pos_right[i1]
        step_lengths.append(_horizontal_dist(p0, p1, use_z=False))
        step_sides.append(f"{s0}->{s1}")
    return np.array(step_lengths), step_sides

def _cumulative_path_length(pos, use_z=False):
    dp = np.diff(pos, axis=0)
    seg = np.linalg.norm(dp if use_z else dp[:, :2], axis=1)
    return float(np.sum(seg))
def safe_float(val, default=0.0):
    """若 val 为 None 或 NaN，则返回 default"""
    try:
        if val is None:
            return default
        if isinstance(val, float) and np.isnan(val):
            return default
        return float(val)
    except Exception:
        return default

def _per_step_stat_from_frame(signal_frame, HS, TO, fs, trim_ms=(100,100), reducer=np.median):
    """在每个支撑期中段取统计量（默认中位数）。"""
    if signal_frame is None or len(signal_frame) == 0:
        return np.array([], float)
    pre  = int(max(0, trim_ms[0] * 1e-3 * fs))
    post = int(max(0, trim_ms[1] * 1e-3 * fs))
    out = []
    for hs, to in zip(HS, TO):
        a = min(max(hs + pre, 0), len(signal_frame) - 1)
        b = min(max(to - post, a + 1), len(signal_frame) - 1)
        seg = signal_frame[a:b+1]
        if seg.size:
            out.append(float(reducer(seg)))
    return np.array(out, float)
def as_float_list(x):
    """
    确保返回 List[float]，用于给 C# 里 List<double> 的字段喂数据。
    支持：标量、list/tuple、np.ndarray、None。
    """
    import numpy as np

    if x is None:
        return []
    if isinstance(x, (list, tuple)):
        return [float(v) for v in x]
    if isinstance(x, np.ndarray):
        return [float(v) for v in x.tolist()]
    # 标量情况
    return [float(x)]
def as_list(x):
    if isinstance(x, list):
        return x
    return [float(x)]
def safe_div(numer, denom, eps=1e-8):
    """
    安全除法：
    - 分母为 0 / None / NaN → 返回 0
    - 适配 C# 端不接受 NaN 的情况
    """
    try:
        if denom is None:
            return 0.0
        if abs(denom) < eps:
            return 0.0
        return float(numer / denom)
    except Exception:
        return 0.0
def stride_lengths_from_HS(pos_world, HS_idx, fs, plane_axes=(0, 1), min_step_s=0.50, max_step_s=2.00):
    HS = np.sort(np.asarray(HS_idx, int))
    pos = np.asarray(pos_world, float)
    out = []
    pairs = []
    for k in range(len(HS) - 1):
        i0, i1 = int(HS[k]), int(HS[k + 1])
        cyc = (i1 - i0) / fs
        if not (min_step_s <= cyc <= max_step_s):
            continue
        p0 = pos[i0, list(plane_axes)]
        p1 = pos[i1, list(plane_axes)]
        out.append(float(np.linalg.norm(p1 - p0)))
        pairs.append((i0, i1))
    return np.asarray(out, float), pairs


def step_lengths_TO_to_next_HS(pos_world, TO_idx, HS_idx, fs, plane_axes=(0, 1), min_step_s=0.20, max_step_s=2.00):
    TO = np.asarray(TO_idx, int)
    HS = np.asarray(HS_idx, int)
    pos = np.asarray(pos_world, float)
    out = []
    pairs = []
    for to in TO:
        future_hs = HS[HS > to]
        if len(future_hs) == 0:
            continue
        hs = int(future_hs[0])
        dt = (hs - to) / fs
        if not (min_step_s <= dt <= max_step_s):
            continue
        p0 = pos[to, list(plane_axes)]
        p1 = pos[hs, list(plane_axes)]
        out.append(float(np.linalg.norm(p1 - p0)))
        pairs.append((int(to), hs))
    return np.asarray(out, float), pairs
