# gait_count_phase.py - 步态事件检测（HS/TO/MS）
import numpy as np
from ahrs.common import orientation
from ahrs.filters import Mahony
from matplotlib import pyplot as plt
from matplotlib.pyplot import axes
from scipy.signal import butter, filtfilt, find_peaks, peak_widths, medfilt




def gait_from_feet_gyro_x(
    left_gyr_x, right_gyr_x, fs=100,
    hp_cut=0.3, lp_cut=8.0,
    min_step_s=0.40, max_step_s=1.5,
    ms_height_abs_rad=0.8,
    ms_min_dist_s=0.4,
    to_search=(0.05, 0.40),
    hs_search=(0.10, 0.70),
    smooth_win_s=0.1,
    input_unit='deg',
    return_debug=False,
):
    """
    仅使用脚部 Gyr_x（角速度X）进行步态事件检测：
      - 以 Gyr_x 作为主信号，带通+平滑后检测 MS（正峰）、TO（MS左侧负峰）、HS（MS右侧过零）
      - 额外提供“正向峰值计步”（POS_idx）

    输出:
      {
        'left':  {'HS_idx','TO_idx','MS_idx','cycle_s','stance_s','swing_s','ranges',
                  'POS_idx','POS_step_count'[,'sig','pos_dbg']},
        'right': {...}
      }
    """

    # ---------- 小工具 ----------
    def _bandpass(x):
        b, a = butter(2, [hp_cut/(0.5*fs), lp_cut/(0.5*fs)], btype='band')
        return filtfilt(b, a, x)

    def _smooth_ma(x, win_s):
        w = max(1, int(round(win_s*fs)))
        if w <= 1: return x
        return np.convolve(x, np.ones(w)/w, mode='same')

    def _mov_mean(x, win_s):
        w = max(1, int(round(win_s*fs)))
        return np.convolve(x, np.ones(w)/w, mode='same')

    def _rolling_rms(x, win_s, eps=1e-6):
        w = max(3, int(round(win_s*fs)))
        if w <= 1: return np.abs(x)
        k = np.ones(w)/w
        x2 = np.convolve(x**2, k, mode='same')
        return np.sqrt(np.maximum(x2, eps))

    def _activity_mask(gx, fs, win_s=0.5, k_g=2.0, min_active_s=1.0):
        """仅用陀螺自适应强度估计得到活动掩码"""
        gx_abs = np.abs(gx)
        w = max(3, int(round(win_s * fs)))
        gyro_rms = np.sqrt(np.convolve(gx_abs ** 2, np.ones(w)/w, mode='same'))
        noise = np.median(gyro_rms[:max(1, int(1.0 * fs))])  # 前1秒估噪
        thr = max(1e-6, k_g * noise)
        mask = gyro_rms > thr

        # 最小连续时长清理
        min_len = int(round(min_active_s * fs))
        if min_len > 1:
            run = 0
            for i in range(len(mask)):
                run = run + 1 if mask[i] else 0
                if not mask[i] and run < min_len:
                    mask[i - run:i] = False
            run = 0
            for i in range(len(mask)-1, -1, -1):
                run = run + 1 if mask[i] else 0
                if not mask[i] and run < min_len:
                    mask[i + 1:i + 1 + run] = False
        return mask

    # ===== 仅陀螺的正向峰值计步 =====
    def _count_pos_peaks(sig, act_mask, fs,
                         win_rms_s=0.60,
                         min_height_abs_rad=None,
                         k_height=1.00,
                         k_prom=1.60,
                         width_min_s=0.05,
                         width_max_s=0.60,
                         fill_gap_factor=1.6):
        env_pos = np.maximum(sig, 0.0)
        w = max(5, int(round(win_rms_s * fs)))
        loc_rms = np.sqrt(np.convolve(env_pos ** 2, np.ones(w) / w, mode='same'))

        _min_h = (0.6 * ms_height_abs_rad) if (min_height_abs_rad is None) else float(min_height_abs_rad)
        thr_vec = np.maximum(_min_h, k_height * loc_rms)

        sig_use = sig.copy()
        sig_use[~act_mask] = -1e9
        sig_norm = sig_use - thr_vec

        mad = np.median(np.abs(sig_norm - np.median(sig_norm))) + 1e-6
        P = max(0.2 * np.median(thr_vec[act_mask]) if np.any(act_mask) else 0.0, k_prom * mad)
        distance = max(1, int(np.ceil(ms_min_dist_s * fs)))

        peaks, _ = find_peaks(sig_norm, height=0.0, prominence=P, distance=distance)

        if peaks.size:
            # widths, _, _, _ = peak_widths(sig_use, peaks, rel_height=0.5)
            peaks, props = find_peaks(sig_use, height=0.1, prominence=0.05, distance=5)
            widths, _, _, _ = peak_widths(sig_use, peaks, rel_height=0.5)
            wmin = max(1, int(round(width_min_s * fs)))
            wmax = max(wmin + 1, int(round(width_max_s * fs)))
            keep = (widths >= wmin) & (widths <= wmax)
            peaks = peaks[keep]

        if peaks.size >= 2:
            gap = np.diff(peaks)
            med_gap = np.median(gap)
            fill_idx = []
            for a, b, g in zip(peaks[:-1], peaks[1:], gap):
                if g > fill_gap_factor * med_gap:
                    L = a + int(0.2 * g)
                    R = b - int(0.2 * g)
                    if R > L:
                        sub = sig_norm[L:R + 1]
                        p2, _ = find_peaks(sub,
                                           height=-0.1 * mad,
                                           prominence=max(0.5 * P, 0.1 * mad),
                                           distance=max(1, int(0.5 * distance)))
                        if p2.size:
                            fill_idx.extend(L + p2)
            if fill_idx:
                peaks = np.sort(np.unique(np.r_[peaks, fill_idx]))

        return peaks, {"thr_vec": thr_vec, "sig_norm": sig_norm, "loc_rms": loc_rms, "mad": mad}

    # ---------- 单侧检测（纯陀螺） ----------
    def _detect_one(gx):
        gx = np.asarray(gx, float)
        if input_unit.lower().startswith('deg'):
            gx = np.deg2rad(gx)

        # 1) 预处理（带通 + 平滑）
        sig = _smooth_ma(_bandpass(gx), smooth_win_s)

        # 1.1) 活动掩码（仅陀螺）
        act_mask = _activity_mask(sig, fs, win_s=0.5, k_g=2.0, min_active_s=1.0)

        # 2) MS：自适应高度 + 突出度（仅用陀螺）
        w_rms = max(5, int(round(0.6 * fs)))
        loc_rms = np.sqrt(np.convolve(np.maximum(sig, 0.0) ** 2, np.ones(w_rms) / w_rms, mode='same'))

        ms_height_min = 0.4 * ms_height_abs_rad
        ms_height_loc = 0.9 * np.quantile(loc_rms[act_mask], 0.75) if np.any(act_mask) else 0.0
        ms_height = max(ms_height_min, ms_height_loc)

        mad = np.median(np.abs(sig - np.median(sig)))
        prom_thr = max(0.3 * ms_height, 2.5 * mad)

        distance = max(1, int(np.ceil(ms_min_dist_s * fs)))
        sig_for_peaks = sig.copy()
        sig_for_peaks[~act_mask] = -1e9
        ms_idx, _ = find_peaks(sig_for_peaks, height=float(ms_height), distance=distance, prominence=prom_thr)

        TO_idx, HS_idx = [], []

        # 3) 围绕 MS 搜索候选（无加速度微调）
        for p in ms_idx:
            # ---- TO 候选（MS 左侧负峰）----
            L1 = max(0, p - int(round(to_search[1] * fs)))
            L2 = max(0, min(p - 1, p - int(round(to_search[0] * fs))))
            if L2 > L1:
                t0 = L1 + int(np.argmin(sig[L1:L2 + 1]))
                if sig[t0] < 0 and act_mask[t0]:
                    TO_idx.append(int(t0))

            # ---- HS 候选（MS 右侧过零）----
            R1 = min(len(sig) - 2, p + int(round(hs_search[0] * fs)))
            R2 = min(len(sig) - 2, p + int(round(hs_search[1] * fs)))
            hs_cand = None
            for i in range(R1, R2):
                if sig[i] >= 0 and sig[i + 1] < 0 and act_mask[i + 1]:
                    hs_cand = i + 1
                    break
            if hs_cand is not None:
                HS_idx.append(int(hs_cand))

        # 4) 去重/排序 + 组周期
        HS_idx = np.array(sorted(set(HS_idx)), dtype=int)
        TO_idx = np.array(sorted(set(TO_idx)), dtype=int)

        cycles, stance, swing, ranges = [], [], [], []
        if len(HS_idx) >= 2:
            for k in range(len(HS_idx)-1):
                hs1, hs2 = HS_idx[k], HS_idx[k+1]
                tos = TO_idx[(TO_idx > hs1) & (TO_idx < hs2)]
                if len(tos) == 0:
                    continue
                t0 = int(tos[0])

                cyc = (hs2 - hs1)/fs
                if not (min_step_s <= cyc <= max_step_s):
                    continue
                sta = (t0 - hs1)/fs
                swi = (hs2 - t0)/fs
                if sta <= 0 or swi <= 0:
                    continue
                cycles.append(cyc); stance.append(sta); swing.append(swi)
                ranges.append((hs1, t0, hs2))

        # 5) 正向峰值计步（纯陀螺）
        POS_idx, pos_dbg = _count_pos_peaks(sig, act_mask, fs)
        POS_step_count = int(len(POS_idx))

        out = {
            'HS_idx': np.array(HS_idx, dtype=int),
            'TO_idx': np.array(TO_idx, dtype=int),
            'MS_idx': np.array(ms_idx, dtype=int),
            'cycle_s': np.array(cycles, dtype=float),
            'stance_s': np.array(stance, dtype=float),
            'swing_s': np.array(swing, dtype=float),
            'ranges': ranges,
            'POS_idx': POS_idx,
            'POS_step_count': POS_step_count,
        }
        if return_debug:
            out['sig'] = sig
            out['pos_dbg'] = pos_dbg  # thr_vec / sig_norm / loc_rms / mad
        return out

    return {
        'left':  _detect_one(left_gyr_x),
        'right': _detect_one(right_gyr_x),
    }
def _mahony_quats_from_array(arr, fs=100, kp=1.5, ki=0.05, use_mag=True,
                             normalize_with_quat=True, ref_index=0, gyro_unit='rad',
                             yaw_max_rate_dps=180.0):
    """
    返回: quats_rel(N,4), eulers_rel(N,3 in deg), yaw_deg_cont(N,)
    - 连续的 Yaw（已展开/限速/去毛刺）
    - 自动磁计门控：磁场异常时改用 updateIMU
    - 四元数连续性修正：避免 q/-q 引起的欧拉角跳变
    """
    acc  = arr[:, 0:3].astype(float)
    gyro = arr[:, 3:6].astype(float)
    mag  = arr[:, 6:9].astype(float) if use_mag else None
    if gyro_unit.lower().startswith('deg'):
        gyro = np.deg2rad(gyro)

    N = len(arr)
    mahony = Mahony(Kp=kp, Ki=ki, frequency=fs)

    quats = np.zeros((N,4), dtype=float)
    q = np.array([1.,0.,0.,0.], dtype=float)

    # --- 磁场参考强度：用前1秒的中位数 ---
    if use_mag and mag is not None:
        K = max(1, int(min(N, fs)))  # ~1s
        m_ref = np.median(np.linalg.norm(mag[:K], axis=1))
        m_tol = 0.35  # 允许±35%波动，超出视为异常

    def _update_with_gating(i, q_prev):
        """磁计门控 + 自适应Kp（可选）"""
        if use_mag and mag is not None:
            m_norm = np.linalg.norm(mag[i])
            acc_norm = np.linalg.norm(acc[i])
            mag_ok = (m_norm > 1e-3) and (abs(m_norm - m_ref) <= m_tol * max(m_ref, 1e-6))
            acc_ok = (0.5*9.81 <= acc_norm <= 1.5*9.81)  # 大幅动态/震动时磁计也更不可信
            if mag_ok and acc_ok:
                return mahony.updateMARG(q_prev, gyro[i], acc[i], mag[i])
            else:
                return mahony.updateIMU(q_prev, gyro[i], acc[i])  # 忽略磁计
        else:
            return mahony.updateIMU(q_prev, gyro[i], acc[i])

    # ---- 主循环：估计四元数并做“符号连续性”修正 ----
    for i in range(N):
        q_new = _update_with_gating(i, q)
        q_new = np.array(q_new).flatten()

        # 关键：四元数符号连续（避免 q/-q 翻转）
        if i > 0 and np.dot(q_new, quats[i-1]) < 0.0:
            q_new = -q_new

        quats[i] = q_new
        q = q_new

    # ---------- 重基准（去安装偏移） ----------
    if normalize_with_quat:
        ref_index = int(np.clip(ref_index, 0, N-1))
        qref = quats[ref_index]
        q_ref_conj = np.array([qref[0], -qref[1], -qref[2], -qref[3]])
        quats_rel = np.empty_like(quats)
        # q_rel = q_ref^* ⊗ q
        w1,x1,y1,z1 = q_ref_conj
        for i in range(N):
            w2,x2,y2,z2 = quats[i]
            quats_rel[i] = np.array([
                w1*w2 - x1*x2 - y1*y2 - z1*z2,
                w1*x2 + x1*w2 + y1*z2 - z1*y2,
                w1*y2 - x1*z2 + y1*w2 + z1*x2,
                w1*z2 + x1*y2 - y1*x2 + z1*w2
            ], dtype=float)
    else:
        quats_rel = quats

    # ---------- 欧拉角 ----------
    eulers = np.zeros((N,3), dtype=float)
    for i in range(N):
        e = orientation.q2euler(quats_rel[i])   # rad
        eulers[i] = np.degrees(e)

    # ---------- 连续航向角：unwrap + 限速去毛刺 ----------
    yaw_rad = np.unwrap(np.radians(eulers[:, 2]), discont=np.radians(180))
    yaw_deg = np.degrees(yaw_rad)

    # 物理可达角速度限幅（例如 180°/s）；按采样率换算成每采样最大变化
    step_lim = float(yaw_max_rate_dps) / float(fs)  # deg/step
    dy = np.diff(yaw_deg)
    dy_clamped = np.clip(dy, -step_lim, step_lim)
    yaw_deg_cont = np.r_[yaw_deg[0], yaw_deg[0] + np.cumsum(dy_clamped)]

    # 可选：再做一个轻微滑动平均去毛刺（不会引入相位延迟太多）
    w = max(1, int(round(0.10 * fs)))  # 0.10s窗口
    if w > 1:
        k = np.ones(w) / w
        yaw_deg_cont = np.convolve(yaw_deg_cont, k, mode='same')

    # 把连续Yaw写回eulers用于统一绘图
    eulers[:, 2] = yaw_deg_cont

    return quats_rel, eulers, yaw_deg_cont

def count_steps_during_turns(arr1, fs, turn_windows_s,
                             axis='norm',            # 'norm' | 'x' | 'y' | 'z'
                             hp_cut=0.7, lp_cut=6.0, # 步频常见 0.7~3 Hz，腰部角速度能到 6Hz 足够
                             smooth_win_s=0.1,      # 轻微平滑窗口（s）
                             min_step_dist_s=0.35,   # 最小相邻步间隔（s）
                             k_height=1.2,           # 自适应高度阈系数（乘以MAD）
                             k_prom=1.5,             # 自适应突出度阈系数（乘以MAD）
                             return_debug=False):
    """
    在给定的转身片段内，用腰部陀螺仪（arr1）计步。
    输入:
      arr1: (N,9) [Acc(0:3), Gyr(3:6), Mag(6:9)]
      fs  : 采样率
      turn_windows_s: [(s,e), ...] 片段（单位秒）
      axis: 使用的通道；默认 'norm' 为三轴陀螺模长，更稳健
    返回:
      {
        "turn_windows_s": [(s,e), ...],
        "turn_steps": [步数, ...],
        "step_indices_per_turn": [np.array([...]), ...],  # 全局样本索引
        "debug": { ... }  # 可选
      }
    """
    gx, gy, gz = arr1[:, 3], arr1[:, 4], arr1[:, 5]
    if axis == 'x':
        sig_raw = np.abs(gx)
    elif axis == 'y':
        sig_raw = np.abs(gy)
    elif axis == 'z':
        sig_raw = np.abs(gz)
    else:  # 'norm'（推荐）
        sig_raw = np.sqrt(gx*gx + gy*gy + gz*gz)

    # 去尖刺（中值滤波）+ 带通 + 轻微平滑
    k_med = max(1, int(round(0.06*fs)) | 1)  # 约0.06s，奇数
    if k_med > 1:
        sig_raw = medfilt(sig_raw, kernel_size=k_med)

    b, a = butter(2, [hp_cut/(0.5*fs), lp_cut/(0.5*fs)], btype='band')
    sig_bp = filtfilt(b, a, sig_raw)

    w = max(1, int(round(smooth_win_s*fs)))
    if w > 1:
        sig_bp = np.convolve(sig_bp, np.ones(w)/w, mode='same')

    # 预先计算一个全局MAD作兜底
    mad_global = np.median(np.abs(sig_bp - np.median(sig_bp))) + 1e-6
    dist = max(1, int(round(min_step_dist_s*fs)))

    out_counts, out_indices = [], []
    for (s, e) in turn_windows_s:
        L = int(np.floor(s*fs)); R = int(np.ceil(e*fs))
        L = max(0, L); R = min(len(sig_bp)-1, R)
        seg = sig_bp[L:R+1]
        if seg.size <= 3:
            out_counts.append(0); out_indices.append(np.array([], dtype=int)); continue

        # 窗口内自适应阈值（robust）
        med = np.median(seg)
        mad = np.median(np.abs(seg - med)) + 1e-6
        H = med + k_height * max(mad, mad_global*0.6)     # 高度阈
        P = k_prom * max(mad, mad_global*0.6)            # 突出度阈

        peaks, props = find_peaks(seg, height=H, prominence=P, distance=dist)
        idx_global = L + peaks

        out_counts.append(int(len(idx_global)))
        out_indices.append(idx_global.astype(int))

    result = {
        "turn_windows_s": turn_windows_s,
        "turn_steps": out_counts,
        "step_indices_per_turn": out_indices
    }
    if return_debug:
        result["debug"] = {"sig": sig_bp}
    return result

def detect_turn_segments_waist(
    yaw_deg_cont, fs=100,
    # —— 累计转角窗口与阈值（腰部传感器关键）——
    angle_win_s=1.00,            # 计算累计转角的滑动窗口长度（秒）
    angle_thr_on=35.0,           # 进入转身：窗口内累计转角 ≥ 该阈值（度）
    angle_thr_off=20.0,          # 退出转身：窗口内累计转角 < 该阈值（度）
    # —— 角速度滞回（辅助抑制误报/漏报）——
    yawrate_on=35.0,             # 进入转身的角速度阈值（°/s）
    yawrate_off=20.0,            # 退出转身的角速度阈值（°/s）
    # —— 稳定性/形态要求 ——
    min_turn_dur_s=1.50,         # 最短转身持续时间（秒），过短抖动不算
    min_total_angle_deg=40.0,    # 整段累计转角（绝对值和）至少达到该角度
    off_hold_s=0.15,             # 退出判据需要持续的时间（秒）
    bridge_gap_s=0.40,           # 合并相邻片段的最大小间隙（秒）
    # —— 去毛刺滤波 ——
    med_win_s=0.5,              # 中值滤波窗口（秒），去尖刺
    lp_cut_hz=2.5                # 对 yawrate 的低通（Hz）
):
    """
    输入:
      yaw_deg_cont : 连续航向角（已 unwrap，单位度）
      fs           : 采样率
    输出:
      turn_windows_s : [(t_start, t_end), ...]  (单位秒)
      debug : 用于可视化的中间量字典
    """
    yaw = np.asarray(yaw_deg_cont, float)
    N   = len(yaw)
    t   = np.arange(N)/fs

    # --- 角速度 (°/s) ---
    dyaw = np.diff(yaw, prepend=yaw[0])           # 每采样的角度增量 (°/sample)
    yawrate = dyaw * fs                            # (°/s)

    # 去尖刺 + 低通
    k_med = max(1, int(round(med_win_s*fs))|1)     # 奇数核
    if k_med > 1:
        yawrate = medfilt(yawrate, kernel_size=k_med)
    if lp_cut_hz and lp_cut_hz > 0:
        b, a = butter(2, lp_cut_hz/(0.5*fs), btype='low')
        yawrate = filtfilt(b, a, yawrate)

    # --- 累计转角（滑动窗口内的绝对转角和，单位度） ---
    W = max(1, int(round(angle_win_s*fs)))
    win = np.ones(W, dtype=float)
    abs_dyaw = np.abs(dyaw)
    # 与卷积保持同长度
    cum_angle_win = np.convolve(abs_dyaw, win, mode='same')

    # --- 状态机：由“累计转角+角速度”共同驱动，带滞回和持续判据 ---
    on_mask  = (cum_angle_win >= angle_thr_on) | (np.abs(yawrate) >= yawrate_on)
    off_mask = (cum_angle_win >= angle_thr_off) | (np.abs(yawrate) >= yawrate_off)

    in_turn = False
    start_i = None
    turns = []

    off_hold = max(1, int(round(off_hold_s*fs)))
    off_count = 0
    for i in range(N):
        if not in_turn:
            if on_mask[i]:
                in_turn = True
                start_i = i
                off_count = 0
        else:
            # 只要 off_mask 为 False 持续 off_hold，就退出
            if not off_mask[i]:
                off_count += 1
            else:
                off_count = 0
            if off_count >= off_hold:
                end_i = i
                dur = (end_i - start_i)/fs
                total_angle = np.sum(abs_dyaw[start_i:end_i+1])
                if dur >= min_turn_dur_s and total_angle >= min_total_angle_deg:
                    turns.append((start_i/fs, end_i/fs))
                in_turn = False
                start_i = None
                off_count = 0
    # 尾段闭合
    if in_turn and start_i is not None:
        end_i = N-1
        dur = (end_i - start_i)/fs
        total_angle = np.sum(abs_dyaw[start_i:end_i+1])
        if dur >= min_turn_dur_s and total_angle >= min_total_angle_deg:
            turns.append((start_i/fs, end_i/fs))

    # --- 合并近邻片段 ---
    if turns:
        merged = [turns[0]]
        for s,e in turns[1:]:
            s0,e0 = merged[-1]
            if s - e0 <= bridge_gap_s:
                merged[-1] = (s0, e)
            else:
                merged.append((s,e))
    else:
        merged = []

    # 计算每段持续时间与平均转身时间（秒）
    durations = [e - s for (s, e) in merged]
    avg_turn_time = float(np.mean(durations)) if len(durations) else 0.0

    debug = dict(
        t=t, yaw=yaw, yawrate=yawrate,
        cum_angle_win=cum_angle_win,
        angle_thr_on=angle_thr_on, angle_thr_off=angle_thr_off,
        yawrate_on=yawrate_on, yawrate_off=yawrate_off
    )
    return merged, debug, avg_turn_time

