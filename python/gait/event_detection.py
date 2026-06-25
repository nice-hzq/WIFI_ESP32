
import numpy as np
import pandas as pd
from scipy.signal import find_peaks,butter, filtfilt

from sensor.data_reader import load_calibrated_filtered_arrays


# ================================================================================================
# 统一的步态事件检测函数 (HS/TO)
# ================================================================================================
# def gait_to_hs_from_filtered_gyro_x(
#     left_gyr_x: np.ndarray,
#     right_gyr_x: np.ndarray,
#     fs: float,
#     *,
#     min_step_s: float = 0.50,
#     max_step_s: float = 2.00,
#     ms_height_abs_rad: float = 1.5,
#     ms_min_dist_s: float = 0.6,
#     to_search=(0.08, 0.60),
#     hs_search=(0.05, 0.60),
#     hs_method: str = "zero_cross",
#     hs_zero_band: float = 0.25,
#     hs_slope_thresh: float = 0.08,
#     hs_confirm_s: float = 0.03,
#     input_unit: str = "rad",
#     return_debug: bool = False,
# ):
#     """
#     输入已滤波的脚部 gyr_x，返回左右脚 TO / HS 索引。

#     hs_method:
#       - "zero_cross":   MS 右侧第一个过零点作为 HS（快，适合相位计算）
#       - "platform":     MS 右侧负峰后找接近0且变平的点作为 HS（更准，适合距离计算）
#     """

#     def _detect_one(sig):
#         sig = np.asarray(sig, dtype=float)
#         if input_unit.lower().startswith("deg"):
#             sig = np.deg2rad(sig)

#         n = len(sig)
#         if n < int(fs * (min_step_s + 0.2)):
#             out = {"TO_idx": np.array([], int), "HS_idx": np.array([], int)}
#             if return_debug:
#                 out.update({"MS_idx": np.array([], int), "sig": sig, "ranges": [],
#                             "cycle_s": np.array([], float), "stance_s": np.array([], float),
#                             "swing_s": np.array([], float)})
#             return out

#         dsig = np.diff(sig, prepend=sig[0]) if hs_method == "platform" else None

#         distance = max(1, int(np.ceil(ms_min_dist_s * fs)))
#         MS_idx, _ = find_peaks(sig, height=float(ms_height_abs_rad), distance=distance)

#         TO_list, HS_list = [], []

#         for p in MS_idx:
#             # TO: MS 左侧负峰
#             L1 = max(0, p - int(round(to_search[1] * fs)))
#             L2 = max(0, min(p - 1, p - int(round(to_search[0] * fs))))
#             if L2 > L1:
#                 t0 = L1 + int(np.argmin(sig[L1:L2 + 1]))
#                 if sig[t0] < 0:
#                     TO_list.append(int(t0))

#             # HS: 根据 hs_method 选择算法
#             R1 = min(n - 2, p + int(round(hs_search[0] * fs)))
#             R2 = min(n - 2, p + int(round(hs_search[1] * fs)))
#             if R2 <= R1:
#                 continue

#             if hs_method == "platform":
#                 valley_idx = R1 + int(np.argmin(sig[R1:R2 + 1]))
#                 confirm_len = max(1, int(round(hs_confirm_s * fs)))
#                 hs_found = None
#                 for i in range(valley_idx + 1, R2 - confirm_len + 1):
#                     sig_ok = np.all(np.abs(sig[i:i + confirm_len]) <= hs_zero_band)
#                     slope_ok = np.all(np.abs(dsig[i:i + confirm_len]) <= hs_slope_thresh)
#                     if sig_ok and slope_ok:
#                         hs_found = i
#                         break
#                 if hs_found is None:
#                     S1 = valley_idx + 1
#                     S2 = R2
#                     if S2 > S1:
#                         hs_found = S1 + int(np.argmin(np.abs(sig[S1:S2 + 1])))
#                 if hs_found is not None:
#                     HS_list.append(int(hs_found))
#             else:
#                 for i in range(R1, R2):
#                     if sig[i] >= 0 and sig[i + 1] < 0:
#                         HS_list.append(int(i + 1))
#                         break

#         TO_idx = np.array(sorted(set(TO_list)), dtype=int)
#         HS_idx = np.array(sorted(set(HS_list)), dtype=int)

#         ranges, cycles, stance, swing = [], [], [], []
#         if len(HS_idx) >= 2 and len(TO_idx) >= 1:
#             for k in range(len(HS_idx) - 1):
#                 hs1, hs2 = HS_idx[k], HS_idx[k + 1]
#                 tos = TO_idx[(TO_idx > hs1) & (TO_idx < hs2)]
#                 if tos.size == 0:
#                     continue
#                 t0 = int(tos[0])
#                 cyc = (hs2 - hs1) / fs
#                 if not (min_step_s <= cyc <= max_step_s):
#                     continue
#                 sta = (t0 - hs1) / fs
#                 swi = (hs2 - t0) / fs
#                 if sta <= 0 or swi <= 0:
#                     continue
#                 ranges.append((hs1, t0, hs2))
#                 cycles.append(cyc); stance.append(sta); swing.append(swi)

#         out = {"TO_idx": TO_idx, "HS_idx": HS_idx}
#         if return_debug:
#             out.update({
#                 "MS_idx": np.array(MS_idx, dtype=int),
#                 "ranges": ranges,
#                 "cycle_s": np.array(cycles, float),
#                 "stance_s": np.array(stance, float),
#                 "swing_s": np.array(swing, float),
#                 "sig": sig,
#             })
#         return out

#     return {"left": _detect_one(left_gyr_x), "right": _detect_one(right_gyr_x)}


import numpy as np
from scipy.signal import find_peaks


def gait_to_hs_from_filtered_gyro_x(
    left_gyr_x: np.ndarray,
    right_gyr_x: np.ndarray,
    fs: float,
    *,
    min_step_s: float = 0.50,
    max_step_s: float = 2.00,
    ms_height_abs_rad: float = 1.5,
    ms_min_dist_s: float = 0.6,
    to_search=(0.08, 0.60),
    hs_search=(0.05, 0.60),
    hs_method: str = "platform",
    hs_zero_band: float = 0.25,
    hs_slope_thresh: float = 0.08,
    hs_confirm_s: float = 0.03,
    input_unit: str = "rad",
    return_debug: bool = False,
):
    """
    输入已滤波的脚部 gyr_x，返回左右脚 TO / HS 索引。

    优化点：
    1. MS 检测由固定 height 改为 prominence + 自适应阈值，降低不同速度/幅值下的漏检。
    2. TO / HS 搜索范围由固定秒数改为“局部步态周期比例”，适应快走、慢走。
    3. HS 的 platform 方法中，zero band 和 slope threshold 会根据局部幅值自适应调整。
    4. 保留原函数名和主要参数接口，外部调用方式不需要改变。

    注意：
    - to_search 和 hs_search 在本版本中优先解释为“局部周期比例”。
      例如 hs_search=(0.05, 0.60) 表示在 MS 之后 5%~60% 的局部周期内寻找 HS。
    - 如果传入的窗口参数大于 1，例如 (0.05, 1.20)，则会自动按“秒”处理。
    - hs_method:
        "zero_cross": MS 右侧第一个正到负过零点作为 HS，适合快速相位估计。
        "platform":   MS 右侧负峰后寻找接近 0 且变平的位置，适合较准确定位 HS。
    """

    def _robust_scale(x):
        """鲁棒尺度估计，避免被异常值影响。"""
        x = np.asarray(x, dtype=float)
        med = np.median(x)
        mad = np.median(np.abs(x - med))
        scale = 1.4826 * mad

        if not np.isfinite(scale) or scale < 1e-8:
            scale = np.std(x)

        if not np.isfinite(scale) or scale < 1e-8:
            scale = 1.0

        return scale

    def _merge_close_events(idx, sig, min_sep_samples, mode="hs"):
        """
        合并距离过近的事件点，减少重复检测。

        mode="to": 同一组中选择负峰更明显的点。
        mode="hs": 同一组中选择更接近 0 的点。
        """
        idx = np.array(sorted(set(idx)), dtype=int)
        if idx.size == 0:
            return idx

        groups = []
        current = [int(idx[0])]

        for v in idx[1:]:
            v = int(v)
            if v - current[-1] <= min_sep_samples:
                current.append(v)
            else:
                groups.append(current)
                current = [v]
        groups.append(current)

        merged = []
        for g in groups:
            g = np.array(g, dtype=int)
            if mode == "to":
                best = g[np.argmin(sig[g])]
            else:
                best = g[np.argmin(np.abs(sig[g]))]
            merged.append(int(best))

        return np.array(sorted(merged), dtype=int)

    def _window_to_samples(window, T_local, fs):
        """
        将搜索窗口转换为采样点数。

        如果窗口值 <= 1，则认为是局部周期比例；
        如果窗口值 > 1，则认为是秒。
        """
        a, b = float(window[0]), float(window[1])

        if max(abs(a), abs(b)) <= 1.0:
            s1 = int(round(a * T_local))
            s2 = int(round(b * T_local))
        else:
            s1 = int(round(a * fs))
            s2 = int(round(b * fs))

        s1 = max(1, s1)
        s2 = max(s1 + 1, s2)

        return s1, s2

    def _estimate_global_cycle(MS_idx):
        """根据 MS 峰间距估计全局同侧步态周期。"""
        if len(MS_idx) < 2:
            return int(round(1.0 * fs))

        intervals = np.diff(MS_idx)
        min_samples = int(round(min_step_s * fs))
        max_samples = int(round(max_step_s * fs))

        valid = intervals[
            (intervals >= min_samples) &
            (intervals <= max_samples)
        ]

        if valid.size > 0:
            T = int(round(np.median(valid)))
        else:
            T = int(round(np.median(intervals)))

        T = int(np.clip(T, min_samples, max_samples))
        return T

    def _estimate_local_cycle(MS_idx, j, global_T):
        """
        根据当前 MS 左右相邻峰估计局部周期。
        如果局部峰间距不可用，则退化为全局周期。
        """
        intervals = []

        if j > 0:
            intervals.append(MS_idx[j] - MS_idx[j - 1])

        if j < len(MS_idx) - 1:
            intervals.append(MS_idx[j + 1] - MS_idx[j])

        min_samples = int(round(min_step_s * fs))
        max_samples = int(round(max_step_s * fs))

        valid = [
            d for d in intervals
            if min_samples <= d <= max_samples
        ]

        if len(valid) > 0:
            T = int(round(np.median(valid)))
        else:
            T = int(global_T)

        T = int(np.clip(T, min_samples, max_samples))
        return T

    def _detect_ms(sig):
        """
        自适应检测 MS 候选峰。
        原函数使用固定 height=1.5 rad/s，在慢速步态中容易漏检。
        这里改为 prominence 为主，固定高度只作为辅助参考。
        """
        n = len(sig)

        scale = _robust_scale(sig)
        med = np.median(sig)

        # 不再直接使用 ms_min_dist_s 作为强硬峰间距，
        # 否则快走/跑步时真实峰可能被抑制。
        # 这里给一个更宽松的最小间隔。
        adaptive_dist_s = min(ms_min_dist_s, 0.35)
        distance = max(1, int(round(adaptive_dist_s * fs)))

        # 自适应 prominence。
        # scale 大时要求峰更突出，scale 小时仍保留一个最低要求。
        adaptive_prominence = max(0.08, 0.60 * scale)

        MS_idx, props = find_peaks(
            sig,
            prominence=adaptive_prominence,
            distance=distance
        )

        # 如果没有找到峰，降低 prominence 再尝试一次。
        if len(MS_idx) == 0:
            MS_idx, props = find_peaks(
                sig,
                prominence=max(0.04, 0.30 * scale),
                distance=max(1, int(round(0.25 * fs)))
            )

        # 只保留相对偏正的峰，避免把负区间的小局部最大值当成 MS。
        if len(MS_idx) > 0:
            positive_floor = max(0.0, med + 0.05 * scale)
            pos_idx = MS_idx[sig[MS_idx] > positive_floor]

            if len(pos_idx) > 0:
                MS_idx = pos_idx

        # 如果峰特别多，再用较宽松的高度做一次二次筛选。
        # 这里不强制要求达到 ms_height_abs_rad，因为慢速步态可能达不到。
        if len(MS_idx) > 2:
            local_p75 = np.percentile(sig[MS_idx], 75)
            soft_height = min(float(ms_height_abs_rad), local_p75)
            keep = sig[MS_idx] >= max(0.0, 0.50 * soft_height)

            if np.sum(keep) >= 2:
                MS_idx = MS_idx[keep]

        return np.array(sorted(set(MS_idx)), dtype=int)

    def _detect_one(sig):
        sig = np.asarray(sig, dtype=float)

        if input_unit.lower().startswith("deg"):
            sig = np.deg2rad(sig)

        n = len(sig)

        if n < int(fs * (min_step_s + 0.2)):
            out = {
                "TO_idx": np.array([], dtype=int),
                "HS_idx": np.array([], dtype=int)
            }

            if return_debug:
                out.update({
                    "MS_idx": np.array([], dtype=int),
                    "sig": sig,
                    "ranges": [],
                    "cycle_s": np.array([], dtype=float),
                    "stance_s": np.array([], dtype=float),
                    "swing_s": np.array([], dtype=float),
                    "local_cycle_s": np.array([], dtype=float),
                })

            return out

        dsig = np.diff(sig, prepend=sig[0])

        MS_idx = _detect_ms(sig)
        global_T = _estimate_global_cycle(MS_idx)

        TO_list = []
        HS_list = []
        local_cycle_list = []

        for j, p in enumerate(MS_idx):
            T_local = _estimate_local_cycle(MS_idx, j, global_T)
            local_cycle_list.append(T_local / fs)

            # 根据局部周期自适应设置 TO 搜索窗口
            to_s1, to_s2 = _window_to_samples(to_search, T_local, fs)

            L1 = max(0, p - to_s2)
            L2 = max(0, p - to_s1)

            if L2 > L1:
                t0 = L1 + int(np.argmin(sig[L1:L2 + 1]))

                # TO 通常对应 MS 左侧的负峰。
                # 这里保留 sig[t0] < 0 的基本约束，避免将正区间局部低点当成 TO。
                if sig[t0] < 0:
                    TO_list.append(int(t0))

            # 根据局部周期自适应设置 HS 搜索窗口
            hs_s1, hs_s2 = _window_to_samples(hs_search, T_local, fs)

            R1 = min(n - 2, p + hs_s1)
            R2 = min(n - 2, p + hs_s2)

            if R2 <= R1:
                continue

            # 当前 MS 附近的局部幅值，用于自适应 HS 阈值
            A1 = max(0, p - int(round(0.50 * T_local)))
            A2 = min(n, p + int(round(0.70 * T_local)))
            local_seg = sig[A1:A2]

            if local_seg.size > 0:
                local_amp = max(
                    abs(sig[p]),
                    np.percentile(np.abs(local_seg), 90),
                    _robust_scale(local_seg)
                )
            else:
                local_amp = max(abs(sig[p]), _robust_scale(sig))

            if hs_method == "platform":
                # 先找 MS 右侧负峰
                valley_idx = R1 + int(np.argmin(sig[R1:R2 + 1]))

                confirm_len = max(1, int(round(hs_confirm_s * fs)))

                # 自适应接近 0 的幅值范围
                zero_band_i = max(
                    hs_zero_band,
                    0.08 * local_amp
                )

                # 防止阈值过大
                zero_band_i = min(zero_band_i, 0.50 * max(local_amp, 1e-6))

                # 自适应平坦程度阈值
                slope_thresh_i = max(
                    hs_slope_thresh,
                    0.03 * local_amp
                )

                hs_found = None

                # ——— 第 1 轮：严格条件 ———
                for i in range(valley_idx + 1, R2 - confirm_len + 1):
                    if (np.all(np.abs(sig[i:i + confirm_len]) <= zero_band_i) and
                        np.all(np.abs(dsig[i:i + confirm_len]) <= slope_thresh_i)):
                        hs_found = int(i)
                        break

                # ——— 第 2 轮：阈值放宽 2 倍 ———
                if hs_found is None:
                    zb2 = zero_band_i * 2.0
                    st2 = slope_thresh_i * 2.0
                    for i in range(valley_idx + 1, R2 - confirm_len + 1):
                        if (np.all(np.abs(sig[i:i + confirm_len]) <= zb2) and
                            np.all(np.abs(dsig[i:i + confirm_len]) <= st2)):
                            hs_found = int(i)
                            break

                # ——— 第 3 轮：评分制（不要求全部样本满足，选最优点）———
                if hs_found is None:
                    zb3 = zero_band_i * 3.0
                    st3 = slope_thresh_i * 3.0
                    best_score = -1e9
                    best_i = None
                    half_len = max(1, confirm_len // 2)
                    for i in range(valley_idx + 1, R2 - half_len + 1):
                        win_s = sig[i:i + confirm_len]
                        win_d = dsig[i:i + confirm_len]
                        # 越接近 0 分越高，越平坦分越高
                        score = (
                            -np.mean(np.abs(win_s)) / max(zb3, 1e-6) +
                            -np.mean(np.abs(win_d)) / max(st3, 1e-6)
                        )
                        if score > best_score:
                            best_score = score
                            best_i = i
                    if best_i is not None:
                        hs_found = int(best_i)

                # ——— 第 4 轮：兜底 — 负峰后最接近 0 的点 ———
                if hs_found is None:
                    S1 = valley_idx + 1
                    S2 = R2
                    if S2 > S1:
                        hs_found = S1 + int(np.argmin(np.abs(sig[S1:S2 + 1])))

                if hs_found is not None:
                    HS_list.append(int(hs_found))

            else:
                # zero_cross 方法：MS 右侧寻找第一个正到负过零点
                hs_found = None

                for i in range(R1, R2):
                    if sig[i] >= 0 and sig[i + 1] < 0:
                        hs_found = int(i + 1)
                        break

                # 如果没有明显过零点，则退化为该窗口内最接近 0 的点
                if hs_found is None:
                    hs_found = R1 + int(np.argmin(np.abs(sig[R1:R2 + 1])))

                HS_list.append(int(hs_found))

        # 合并过近事件，避免同一个事件被多个 MS 重复检测
        min_event_sep = max(1, int(round(0.08 * fs)))

        TO_idx = _merge_close_events(
            TO_list,
            sig,
            min_sep_samples=min_event_sep,
            mode="to"
        )

        HS_idx = _merge_close_events(
            HS_list,
            sig,
            min_sep_samples=min_event_sep,
            mode="hs"
        )

        # 计算有效 HS-TO-HS 周期
        ranges = []
        cycles = []
        stance = []
        swing = []

        if len(HS_idx) >= 2 and len(TO_idx) >= 1:
            for k in range(len(HS_idx) - 1):
                hs1 = int(HS_idx[k])
                hs2 = int(HS_idx[k + 1])

                tos = TO_idx[(TO_idx > hs1) & (TO_idx < hs2)]

                if tos.size == 0:
                    continue

                # 一个 HS-HS 周期内通常取第一个 TO
                t0 = int(tos[0])

                cyc = (hs2 - hs1) / fs

                if not (min_step_s <= cyc <= max_step_s):
                    continue

                sta = (t0 - hs1) / fs
                swi = (hs2 - t0) / fs

                if sta <= 0 or swi <= 0:
                    continue

                # 宽松的支撑/摆动比例约束，防止明显错误配对
                stance_ratio = sta / cyc
                swing_ratio = swi / cyc

                if not (0.20 <= stance_ratio <= 0.85):
                    continue

                if not (0.15 <= swing_ratio <= 0.80):
                    continue

                ranges.append((hs1, t0, hs2))
                cycles.append(cyc)
                stance.append(sta)
                swing.append(swi)

        out = {
            "TO_idx": TO_idx,
            "HS_idx": HS_idx
        }

        if return_debug:
            out.update({
                "MS_idx": np.array(MS_idx, dtype=int),
                "ranges": ranges,
                "cycle_s": np.array(cycles, dtype=float),
                "stance_s": np.array(stance, dtype=float),
                "swing_s": np.array(swing, dtype=float),
                "sig": sig,
                "local_cycle_s": np.array(local_cycle_list, dtype=float),
            })

        return out

    return {
        "left": _detect_one(left_gyr_x),
        "right": _detect_one(right_gyr_x)
    }

# 保留旧名称为别名，确保向后兼容
gait_to_hs_from_filtered_gyro_x_for_distance = lambda *a, **kw: gait_to_hs_from_filtered_gyro_x(
    *a, hs_method="platform", **kw
)

# 向后兼容别名
gait_to_hs_from_filtered_gyro_x_adaptive = gait_to_hs_from_filtered_gyro_x



def gait_to_hs_from_filtered_gyro_x_adaptive_balanced(
    left_gyr_x, right_gyr_x, fs, *,
    input_unit='deg', hs_balance_alpha=0.1, return_debug=False, **kwargs
):
    return gait_to_hs_from_filtered_gyro_x(
        left_gyr_x=left_gyr_x,
        right_gyr_x=right_gyr_x,
        fs=fs,
        input_unit=input_unit,
        return_debug=return_debug,
        **kwargs
    )



# ================================================================================================


def compute_gait_phase_metrics_v2(res, fs=50):
    """
    根据 gait_to_hs_from_filtered_gyro_x(..., return_debug=True) 的输出 res，
    计算左右脚：
    1) 步态周期 gait cycle
    2) 支撑相 stance
    3) 摆动相 swing
    4) 双支撑相 double support（通过左右支撑区间重叠计算）

    返回:
        {
            "left": df_left,
            "right": df_right
        }
    """

    def overlap_len(a1, a2, b1, b2):
        """两个区间 [a1,a2], [b1,b2] 的重叠长度（单位：采样点）"""
        left = max(a1, b1)
        right = min(a2, b2)
        return max(0, right - left)

    def _build_side_df(hs_ipsi, to_ipsi, hs_contra, to_contra, side_name):
        hs_ipsi = np.asarray(hs_ipsi, dtype=int)
        to_ipsi = np.asarray(to_ipsi, dtype=int)
        hs_contra = np.asarray(hs_contra, dtype=int)
        to_contra = np.asarray(to_contra, dtype=int)

        rows = []

        if len(hs_ipsi) < 2:
            return pd.DataFrame()

        # 先构造对侧所有"支撑区间" [HS, TO]
        contra_stance_intervals = []
        for i in range(len(hs_contra) - 1):
            chs1 = hs_contra[i]
            chs2 = hs_contra[i + 1]

            tos = to_contra[(to_contra > chs1) & (to_contra < chs2)]
            if len(tos) == 0:
                continue
            cto = int(tos[0])

            if cto > chs1:
                contra_stance_intervals.append((chs1, cto))

        # 再计算本侧每个周期
        for i in range(len(hs_ipsi) - 1):
            hs1 = int(hs_ipsi[i])
            hs2 = int(hs_ipsi[i + 1])

            # 本侧周期内找 TO
            tos = to_ipsi[(to_ipsi > hs1) & (to_ipsi < hs2)]
            if len(tos) == 0:
                continue
            to1 = int(tos[0])

            gait_cycle = (hs2 - hs1) / fs
            stance = (to1 - hs1) / fs
            swing = (hs2 - to1) / fs

            if gait_cycle <= 0 or stance <= 0 or swing <= 0:
                continue

            stance_pct = stance / gait_cycle * 100
            swing_pct = swing / gait_cycle * 100

            # 本侧支撑区间
            ipsi_stance_start = hs1
            ipsi_stance_end = to1

            # 计算与对侧所有支撑区间的重叠
            overlaps = []
            for c_hs, c_to in contra_stance_intervals:
                ov = overlap_len(ipsi_stance_start, ipsi_stance_end, c_hs, c_to)
                if ov > 0:
                    overlaps.append((max(ipsi_stance_start, c_hs), min(ipsi_stance_end, c_to), ov))

            # 总双支撑时间
            ds_total_samples = sum(x[2] for x in overlaps)
            ds_total_s = ds_total_samples / fs
            ds_pct = ds_total_s / gait_cycle * 100

            # 如果有两段重叠，可以近似分成"初始双支撑"和"末期双支撑"
            overlaps_sorted = sorted(overlaps, key=lambda x: x[0])

            ids_s = np.nan
            tds_s = np.nan
            if len(overlaps_sorted) >= 1:
                ids_s = overlaps_sorted[0][2] / fs
            if len(overlaps_sorted) >= 2:
                tds_s = overlaps_sorted[-1][2] / fs
            elif len(overlaps_sorted) == 1:
                # 若只有一段重叠，不强行分成两段
                tds_s = np.nan

            rows.append({
                "side": side_name,
                "HS1_idx": hs1,
                "TO_idx": to1,
                "HS2_idx": hs2,

                "gait_cycle_s": gait_cycle,
                "stance_s": stance,
                "swing_s": swing,

                "stance_pct": stance_pct,
                "swing_pct": swing_pct,

                "initial_double_support_s": ids_s,
                "terminal_double_support_s": tds_s,
                "double_support_total_s": ds_total_s,
                "double_support_pct": ds_pct,
            })

        return pd.DataFrame(rows)

    left_hs = res["left"]["HS_idx"]
    left_to = res["left"]["TO_idx"]
    right_hs = res["right"]["HS_idx"]
    right_to = res["right"]["TO_idx"]

    df_left = _build_side_df(
        hs_ipsi=left_hs,
        to_ipsi=left_to,
        hs_contra=right_hs,
        to_contra=right_to,
        side_name="left"
    )

    df_right = _build_side_df(
        hs_ipsi=right_hs,
        to_ipsi=right_to,
        hs_contra=left_hs,
        to_contra=left_to,
        side_name="right"
    )

    return {
        "left": df_left,
        "right": df_right
    }


def summarize_gait_metrics(metrics):
    summary = {}
    for side in ["left", "right"]:
        df = metrics[side]
        if df is None or len(df) == 0:
            summary[side] = None
            continue

        summary[side] = {
            "mean_gait_cycle_s": df["gait_cycle_s"].mean(),
            "mean_stance_s": df["stance_s"].mean(),
            "mean_swing_s": df["swing_s"].mean(),
            "mean_stance_pct": df["stance_pct"].mean(),
            "mean_swing_pct": df["swing_pct"].mean(),
            "mean_double_support_total_s": df["double_support_total_s"].mean(),
            "mean_double_support_pct": df["double_support_pct"].mean(),
        }
    return summary




def compute_gait_params_from_events(left, right):
    """
    输入:
        left, right: 来自 gait_from_feet_gyro_x 的单侧结果字典
            需要字段: 'HS_idx', 'TO_idx', 'ranges' (每个元素是 (hs1, to, hs2))
            若有 'cycle_s', 'stance_s', 'swing_s' 会直接使用；否则本函数会用推断的 fs 计算。
    返回:
        一个包含各项指标的 dict（中文键名），同时给出均值/列表。
    """
    # --------- 工具函数 ---------
    def _infer_fs(side):
        """通过 HS_idx 与 cycle_s 推断 fs；若无 cycle_s，则回退为 None（后续再推断）"""
        HS = np.asarray(side.get('HS_idx', []), dtype=int)
        cyc = np.asarray(side.get('cycle_s', []), dtype=float)
        if len(HS) >= 2 and len(cyc) >= 1:
            dHS = np.diff(HS[:len(cyc)+1])  # 与cycle_s配对
            pos = dHS > 0
            if np.any(pos):
                fs_est = np.median(dHS[pos] / cyc[pos])
                if np.isfinite(fs_est) and fs_est > 0:
                    return float(fs_est)
        return None

    def _ensure_times(side, fs=50):
        """
        返回 per-stride 的 cycle/stance/swing（秒），
        优先使用 side 内已有的 cycle_s/stance_s/swing_s（且长度>0），
        否则根据 HS_idx / TO_idx 自动构造。
        """
        cyc = side.get('cycle_s', None)
        sta = side.get('stance_s', None)
        swi = side.get('swing_s', None)

        # 1) 若已有非空数组，直接使用
        if cyc is not None and sta is not None and swi is not None:
            cyc_arr = np.asarray(cyc, float)
            sta_arr = np.asarray(sta, float)
            swi_arr = np.asarray(swi, float)
            if len(cyc_arr) > 0 and len(sta_arr) > 0 and len(swi_arr) > 0:
                return cyc_arr, sta_arr, swi_arr

        # 2) 否则：根据 HS/TO 自动构造
        HS = np.asarray(side.get('HS_idx', []), dtype=int)
        TO = np.asarray(side.get('TO_idx', []), dtype=int)

        if fs is None or len(HS) < 2 or len(TO) == 0:
            # 兜底：只能用 HS 近似 stride 时间，stance/swing 暂空
            if fs is not None and len(HS) >= 2:
                cyc = np.diff(HS) / fs
                return np.asarray(cyc, float), None, None
            else:
                return np.array([], float), None, None

        # 根据 HS 与 TO 构造 ranges: (hs1, to, hs2)
        ranges = []
        j = 0
        for i in range(len(HS) - 1):
            hs1 = HS[i]
            hs2 = HS[i + 1]
            # 找到 hs1 和 hs2 之间的第一个 TO
            while j < len(TO) and TO[j] <= hs1:
                j += 1
            if j < len(TO) and hs1 < TO[j] < hs2:
                ranges.append((hs1, TO[j], hs2))

        cyc_list, sta_list, swi_list = [], [], []
        for (hs1, to, hs2) in ranges:
            cyc_list.append((hs2 - hs1) / fs)
            sta_list.append((to - hs1) / fs)
            swi_list.append((hs2 - to) / fs)

        cyc_arr = np.asarray(cyc_list, float)
        sta_arr = np.asarray(sta_list, float)
        swi_arr = np.asarray(swi_list, float)

        # 顺便把 ranges 写回 side，供后面的双支撑计算使用
        side['ranges'] = ranges

        return cyc_arr, sta_arr, swi_arr

    def _build_stance_intervals(side, fs):
        """
        生成该脚每个 stride 内的支撑区间(秒)与 stride 窗口(秒)：
            stance: [hs1, to) ; stride_win: [hs1, hs2)
        """
        ranges = side.get('ranges', [])
        if fs is None or len(ranges) == 0:
            return [], []
        stances, strides = [], []
        for (hs1, to, hs2) in ranges:
            stances.append((hs1 / fs, to / fs))
            strides.append((hs1 / fs, hs2 / fs))
        return stances, strides

    def _overlap_len(a, b):
        """两个闭开区间 [a0,a1) 与 [b0,b1) 的重叠时长"""
        L = max(a[0], b[0]); R = min(a[1], b[1])
        return max(0.0, R - L)

    # # # --------- 1) 估计采样率 fs ---------
    # fsL = _infer_fs(left)
    # fsR = _infer_fs(right)
    fs  = 50
    # if fsL and fsR:
    #     fs = float(np.median([fsL, fsR]))
    # else:
    #     fs = fsL or fsR  # 其一可用
    # 若仍然 None，后续基于 cycle_s 的地方会自动避开

    # --------- 2) 单侧时间参数（秒）---------
    cycL, staL, swiL = _ensure_times(left,  fs)

    cycR, staR, swiR = _ensure_times(right, fs)

    # --------- 3) 总步数 & 有效步数 ----------
    HS_L = np.asarray(left.get('HS_idx', []), dtype=int)
    HS_R = np.asarray(right.get('HS_idx', []), dtype=int)


    total_steps = int(len(HS_L) + len(HS_R))

    # 有效步数 = 能够形成"跨脚HS→对侧HS"的 step 的数量
    # 需要时间戳，若 fs 缺失则无法精确计算
    step_times_L = []  # 左单步: L_HS -> 下一次 R_HS

    step_times_R = []  # 右单步: R_HS -> 下一次 L_HS

    if fs is not None and len(HS_L) > 0 and len(HS_R) > 0:
        tL = HS_L / fs
        tR = HS_R / fs
        i = j = 0
        # 合并两序列，计算相邻跨脚间隔
        while i < len(tL) and j < len(tR):
            if tL[i] < tR[j]:
                # L -> R
                step_times_L.append(tR[j] - tL[i])
                i += 1
            else:
                # R -> L
                step_times_R.append(tL[i] - tR[j])
                j += 1
    effective_steps = int(len(step_times_L) + len(step_times_R))

    # --------- 4) 双支撑相位（每个 stride 的双支撑时长与占比）---------
    ds_time = []     # 每个 stride 的双支撑时长（秒）
    ds_ratio = []    # 占比（相对于该 stride 时长）
    if fs is not None:
        L_stances, L_strides = _build_stance_intervals(left,  fs)
        R_stances, R_strides = _build_stance_intervals(right, fs)
        # 建索引方便查找相邻重叠
        kR = 0
        for (stride_win, l_st) in zip(L_strides, L_stances):
            # 该 stride 窗口内，与右脚所有 stance 的重叠总和
            total_olap = 0.0
            # 遍历可能重叠的右脚 stance（线性前进）
            while kR < len(R_stances) and R_stances[kR][1] <= stride_win[0]:
                kR += 1
            k = kR
            while k < len(R_stances) and R_stances[k][0] < stride_win[1]:
                total_olap += _overlap_len(l_st, R_stances[k])
                k += 1
            # stride 时长
            stride_len = max(1e-9, stride_win[1] - stride_win[0])
            ds_time.append(total_olap)
            ds_ratio.append(np.clip(total_olap / stride_len, 0.0, 1.0))
        # 也可以对右脚 stride 再算一遍并合并，这里用左脚 stride 代表全局统计即可

    # --------- 5) 组织输出 ----------
    out = {
        # 计数
        "总步数": total_steps,
        "有效步数": effective_steps,

        # 支撑相/摆动相（秒 与 占比）
        "支撑相左_秒列表": staL.tolist() if staL is not None else [],
        "支撑相右_秒列表": staR.tolist() if staR is not None else [],
        "摆动相左_秒列表": swiL.tolist() if swiL is not None else [],
        "摆动相右_秒列表": swiR.tolist() if swiR is not None else [],
        "支撑相左_占比列表": (staL/cycL).tolist() if staL is not None and len(cycL)>0 else [],
        "支撑相右_占比列表": (staR/cycR).tolist() if staR is not None and len(cycR)>0 else [],
        "摆动相左_占比列表": (swiL/cycL).tolist() if swiL is not None and len(cycL)>0 else [],
        "摆动相右_占比列表": (swiR/cycR).tolist() if swiR is not None and len(cycR)>0 else [],

        # 双支撑
        "双支撑_秒列表": ds_time,
        "双支撑_占比列表": ds_ratio,

        # 单步时间（秒）
        "单步时间左_秒列表": step_times_L,
        "单步时间右_秒列表": step_times_R,

        # 跨步时间（= stride time，同脚 HS→HS）
        "跨步时间左_秒列表": cycL.tolist(),
        "跨步时间右_秒列表": cycR.tolist(),
    }

    # 汇总均值（若有数据）
    def _m(x):
        return float(np.mean(x)) if (x is not None and len(x)>0) else None

    out.update({
        "支撑相左_秒均值": _m(staL),
        "支撑相右_秒均值": _m(staR),
        "摆动相左_秒均值": _m(swiL),
        "摆动相右_秒均值": _m(swiR),

        "支撑相左_占比均值": _m(out["支撑相左_占比列表"]),
        "支撑相右_占比均值": _m(out["支撑相右_占比列表"]),
        "摆动相左_占比均值": _m(out["摆动相左_占比列表"]),
        "摆动相右_占比均值": _m(out["摆动相右_占比列表"]),

        "双支撑_秒均值": _m(ds_time),
        "双支撑_占比均值": _m(ds_ratio),

        "单步时间左_秒均值": _m(step_times_L),
        "单步时间右_秒均值": _m(step_times_R),

        "跨步时间左_秒均值": _m(cycL),
        "跨步时间右_秒均值": _m(cycR),
        "推断采样率_Hz": float(fs) if fs is not None else None,
    })


    # --------- 打印所有结果 ----------
    # print("======= 步态分析结果 =======")
    # for k, v in out.items():
    #     print(f"{k}: {v}")
    # print("======= 结束 =======")
    return out


def _lp_filtfilt_1d(x: np.ndarray, fs: float, cutoff_hz: float, order: int = 4) -> np.ndarray:
    x = np.asarray(x, dtype=float)
    nyq = 0.5 * fs
    wn = cutoff_hz / nyq
    wn = min(max(wn, 1e-6), 0.999999)
    b, a = butter(order, wn, btype="lowpass")
    return filtfilt(b, a, x)

def filter_intersections_by_stance_swing(
    TO, HS, fs: float,
    *,
    # 你要的最小限制（核心）
    min_swing_s: float = 0.25,     # TO -> next HS 最小摆动时间
    min_stance_s: float = 0.25,    # HS -> next TO 最小支撑时间

    # 可选：最大限制（建议给一个，防止匹配跳太远）
    max_swing_s: float | None = 1.20,
    max_stance_s: float | None = 1.50,

    # 以第一个 TO 为锚点（你说第一个 TO 准）
    anchor: str = "first_to",

    # 输入是否为"秒"而不是索引
    input_is_time: bool = False,
):
    """
    输入：
      TO, HS: 交点事件序列（按时间排序），可以是索引(int)或时间(float秒)
      fs: 采样率
      input_is_time: True 表示 TO/HS 是秒；False 表示是 idx

    输出：
      TO_keep_idx, HS_keep_idx: 筛选后的索引(int)
        - 如果 input_is_time=True，也会把时间转换为 idx(round(t*fs))

    策略：
      以第一个 TO 为起点：
        TO0 -> 找满足 min_swing 的第一个 HS
        HS  -> 找满足 min_stance 的第一个 TO
      周而复始，直到找不到下一事件
    """
    TO = np.asarray(TO)
    HS = np.asarray(HS)

    # 转成"秒"统一处理
    if input_is_time:
        TO_t = TO.astype(float)
        HS_t = HS.astype(float)
    else:
        TO_t = TO.astype(float) / fs
        HS_t = HS.astype(float) / fs

    TO_t = np.sort(TO_t)
    HS_t = np.sort(HS_t)

    if TO_t.size == 0 or HS_t.size == 0:
        # 没法配对
        TO_idx = np.array([], dtype=int)
        HS_idx = np.array([], dtype=int)
        return TO_idx, HS_idx

    # 选择锚点
    if anchor == "first_to":
        t0 = TO_t[0]
        keep_TO = [t0]
        expect = "HS"
        last_t = t0
        ptr_TO = 1   # TO 下一个候选起点
        ptr_HS = 0
    else:
        raise ValueError("anchor currently supports only 'first_to'")

    # 用 while 交替匹配
    while True:
        if expect == "HS":
            # 找 HS：t >= last_t + min_swing
            t_min = last_t + float(min_swing_s)
            t_max = None if (max_swing_s is None) else (last_t + float(max_swing_s))

            # 从 ptr_HS 开始扫
            found = None
            while ptr_HS < HS_t.size:
                th = HS_t[ptr_HS]
                ptr_HS += 1
                if th < t_min:
                    continue
                if t_max is not None and th > t_max:
                    # 超过最大摆动，直接认为断了（也可以 continue 但容易跳到很后面）
                    found = None
                    break
                found = th
                break

            if found is None:
                break

            keep_HS = locals().get("keep_HS", [])
            keep_HS.append(found)
            last_t = found
            expect = "TO"

        else:  # expect == "TO"
            # 找 TO：t >= last_t + min_stance
            t_min = last_t + float(min_stance_s)
            t_max = None if (max_stance_s is None) else (last_t + float(max_stance_s))

            found = None
            while ptr_TO < TO_t.size:
                tt = TO_t[ptr_TO]
                ptr_TO += 1
                if tt < t_min:
                    continue
                if t_max is not None and tt > t_max:
                    found = None
                    break
                found = tt
                break

            if found is None:
                break

            keep_TO.append(found)
            last_t = found
            expect = "HS"

    keep_HS = np.array(locals().get("keep_HS", []), dtype=float)
    keep_TO = np.array(keep_TO, dtype=float)

    # 转回 idx
    TO_idx = np.array(np.round(keep_TO * fs), dtype=int)
    HS_idx = np.array(np.round(keep_HS * fs), dtype=int)


    return TO_idx, HS_idx

def gait_events_from_accnorm_k_intersections(
    left_acc_xyz: np.ndarray,
    right_acc_xyz: np.ndarray,
    fs: float,
    *,
    k_g: float = 1.2,              # 水平线：k倍g
    lp_hz: float | None = 10.0,     # 可选：先低通 acc_norm
    input_unit: str = "g",          # "g" or "mps2"

    # HS 约束：不允许 acc_norm < 1g 的点作为 HS
    hs_min_g: float = 1.00,         # 你要求的 1g 下限（可改 1.00~1.02）

    # 交点去抖（可选，避免阈值附近抖动产生一串交点）
    min_cross_interval_s: float = 0.10,

    return_debug: bool = False,
):
    """
    仅使用与 y=k_g 的交点：
      - 上穿交点 -> TO
      - 下穿交点 -> HS，但要求交点附近信号 >= hs_min_g（避免落在 <1g 的谷底）

    返回：
      {'left': {'TO_idx','HS_idx',...}, 'right': {...}}
    """

    def _to_g(acc_xyz):
        acc_xyz = np.asarray(acc_xyz, dtype=float)
        if input_unit.lower() in ["g", "grav"]:
            return acc_xyz
        return acc_xyz / 9.80665

    def _detect_one(acc_xyz):
        acc_g = _to_g(acc_xyz)
        acc_norm_raw = np.linalg.norm(acc_g, axis=1)

        sig = acc_norm_raw
        if lp_hz is not None and lp_hz > 0:
            sig = _lp_filtfilt_1d(sig, fs, cutoff_hz=float(lp_hz), order=4)

        n = len(sig)
        thr = float(k_g)
        hs_min = float(hs_min_g)

        # d 用来找交点
        d = sig - thr

        TO_t = []  # TO 的交点时间（秒）
        HS_t = []  # HS 的交点时间（秒）

        # 可选：如果你还想保留一个"近似索引"给下游
        TO_idx = []  # int(round(t_cross * fs))
        HS_idx = []

        cross_t = []  # 交点时间（秒）
        cross_type = [] # "up" or "down"

        for i in range(1, n):
            # 1) 必须是真正穿越阈值（符号变化）
            if d[i - 1] * d[i] < 0:
                # 2) 线性插值求交点时间
                alpha = float(-d[i - 1] / (d[i] - d[i - 1]))
                t_cross = (i - 1 + alpha) / fs

                # 3) 用斜率判断上穿 / 下穿
                slope = d[i] - d[i - 1]

                if slope > 0:
                    # -------- 上穿：TO --------
                    TO_t.append(t_cross)

                    # 如果你确实需要 index（不推荐用于画图）
                    TO_idx.append(int(round(t_cross * fs)))

                else:
                    # -------- 下穿：HS --------
                    # 强约束：HS 只能来自 high 侧（>= 1g）
                    if sig[i - 1] >= hs_min:
                        HS_t.append(t_cross)
                        HS_idx.append(int(round(t_cross * fs)))
        TO_t = np.array(TO_t, dtype=float)
        HS_t = np.array(HS_t, dtype=float)

        TO_idx = np.array(TO_idx, dtype=int)
        HS_idx = np.array(HS_idx, dtype=int)

        # 这里假设你 TO_idx/HS_idx 存的是 t_cross（秒）
        TO_idx, HS_idx = filter_intersections_by_stance_swing(
            TO=TO_idx, HS=HS_idx, fs=fs,
            min_swing_s=0.65,
            min_stance_s=0.3,
            max_swing_s=1.28,
            max_stance_s=1.50,
            anchor="first_to",
            input_is_time=False
        )

        out = {"TO_idx": TO_idx, "HS_idx": HS_idx}
        if return_debug:
            out.update({
                "sig": sig,
                "acc_norm_raw": acc_norm_raw,
                "thr_g": thr,
                "hs_min_g": hs_min,
                "cross_t": np.array(cross_t, dtype=float),
                "cross_type": np.array(cross_type, dtype=object),
            })
        return out

    return {"left": _detect_one(left_acc_xyz), "right": _detect_one(right_acc_xyz)}

