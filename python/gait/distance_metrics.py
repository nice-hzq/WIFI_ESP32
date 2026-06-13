
import numpy as np

from gait.event_detection import gait_to_hs_from_filtered_gyro_x_for_distance, gait_to_hs_from_filtered_gyro_x, gait_to_hs_from_filtered_gyro_x_adaptive, gait_to_hs_from_filtered_gyro_x_adaptive_balanced
from gait.distance_new import zupt_mask_from_hs_to, zupt_integrate_position
from gait.tool import stride_lengths_from_HS, step_lengths_TO_to_next_HS
from sensor.data_reader import load_calibrated_filtered_arrays
from orientation.quaternion_manager import QuaternionManager, MahonyOrientationNode, rotate_acc_to_world, \
    remove_gravity_and_static_bias


# =========================================================
# 一些小工具函数
# =========================================================
# def _safe_mean(x):
#     x = np.asarray(x, dtype=float)
#     return float(np.nanmean(x)) if x.size > 0 else None
import numpy as np

def _safe_mean(x):
    """
    安全计算修剪均值：忽略 NaN，并去掉一个最小值和一个最大值后求平均。
    - 若无有效值，返回 None
    - 若有效值数量 <= 2，不去极值，直接返回均值
    - 若有效值数量 >= 3，剔除最小、最大值各一个后求平均
    """
    # 1. 转为浮点数组
    x = np.asarray(x, dtype=float)
    # 2. 移除 NaN
    valid = x[~np.isnan(x)]
    n = valid.size

    if n == 0:
        return None
    if n <= 2:
        # 数据太少，无法去掉极值，退化为普通均值
        return float(np.mean(valid))

    # 3. 排序后去掉两端各一个值
    sorted_valid = np.sort(valid)
    trimmed = sorted_valid[1:-1]   # 去掉最小值和最大值
    return float(np.mean(trimmed))

def _safe_sum(x):
    x = np.asarray(x, dtype=float)
    return float(np.nansum(x)) if x.size > 0 else 0.0

def _next_event_pairs(start_idx, end_idx, fs, min_s=0.2, max_s=2.0):
    """
    对每个 start_idx 找后面最近的一个 end_idx，返回时长数组和索引对
    """
    start_idx = np.asarray(start_idx, dtype=int)
    end_idx = np.asarray(end_idx, dtype=int)

    vals = []
    pairs = []
    for s in start_idx:
        future = end_idx[end_idx > s]
        if len(future) == 0:
            continue
        e = int(future[0])
        dt = (e - s) / fs
        if min_s <= dt <= max_s:
            vals.append(dt)
            pairs.append((int(s), e))
    return np.asarray(vals, dtype=float), pairs


def step_length_from_stride_lengths(stride_length_left, stride_length_right):
    """
    由同脚跨步长转换为左右步长：step_length = stride_length / 2。
    这个函数专门用于左右步长估计，不依赖 TO->HS 逐步计算。
    """
    if stride_length_left is None and stride_length_right is None:
        return None, None
    left = None if stride_length_left is None else float(stride_length_left) / 2.0
    right = None if stride_length_right is None else float(stride_length_right) / 2.0
    return left, right




def foot_lift_heights_from_TO_HS(pos_world, TO_idx, HS_idx, v_axis=2, fs=100, min_s=0.20, max_s=2.00):
    """
    每个摆动期 TO -> next HS 内，求竖直方向最大抬升高度
    高度定义：max(pos[to:hs, v_axis] - pos[to, v_axis])
    """
    TO = np.asarray(TO_idx, int)
    HS = np.asarray(HS_idx, int)
    pos = np.asarray(pos_world, float)

    heights = []
    pairs = []
    for to in TO:
        future_hs = HS[HS > to]
        if len(future_hs) == 0:
            continue
        hs = int(future_hs[0])
        dt = (hs - to) / fs
        if not (min_s <= dt <= max_s):
            continue

        seg = pos[to:hs + 1, v_axis]
        if len(seg) < 2:
            continue
        h = float(np.max(seg - seg[0]))
        heights.append(h)
        pairs.append((int(to), hs))

    return np.asarray(heights, dtype=float), pairs


def step_width_from_alternating_HS(pos_L, hs_L, pos_R, hs_R, ml_axis=0, fs=100, min_s=0.20, max_s=2.00):
    """
    用相邻异侧 HS 的横向距离估计 step width
    做法：
    - 把左右 HS 合并排序
    - 只保留 L-R 或 R-L 的相邻足跟着地事件
    - 在各自 HS 时刻取对应足的位置
    - step width = |x_L - x_R| (ML方向绝对差)
    """
    hs_L = np.asarray(hs_L, dtype=int)
    hs_R = np.asarray(hs_R, dtype=int)

    events = []
    for i in hs_L:
        events.append(("L", int(i)))
    for i in hs_R:
        events.append(("R", int(i)))
    events = sorted(events, key=lambda x: x[1])

    widths = []
    pairs = []

    for k in range(len(events) - 1):
        side0, t0 = events[k]
        side1, t1 = events[k + 1]

        if side0 == side1:
            continue

        dt = (t1 - t0) / fs
        if not (min_s <= dt <= max_s):
            continue

        if side0 == "L":
            xL = float(pos_L[t0, ml_axis])
            xR = float(pos_R[t1, ml_axis])
            w = abs(xL - xR)
        else:
            xR = float(pos_R[t0, ml_axis])
            xL = float(pos_L[t1, ml_axis])
            w = abs(xL - xR)

        widths.append(w)
        pairs.append(((side0, t0), (side1, t1)))

    return np.asarray(widths, dtype=float), pairs

import numpy as np
import copy

def _avg_scalar(v1, v2):
    if v1 is None and v2 is None:
        return None
    if v1 is None:
        return float(v2)
    if v2 is None:
        return float(v1)
    return (float(v1) + float(v2)) / 2.0


def _avg_list(lst1, lst2):
    """
    两个列表逐元素平均。
    如果长度不同，取较短长度部分平均。
    """
    if lst1 is None and lst2 is None:
        return []
    if lst1 is None:
        return copy.deepcopy(lst2)
    if lst2 is None:
        return copy.deepcopy(lst1)

    n = min(len(lst1), len(lst2))
    out = []
    for i in range(n):
        out.append(_avg_scalar(lst1[i], lst2[i]))
    return out


def average_metrics_same_format(metrics_big, metrics_small):
    """
    保持和原始 metrics 完全一致的输出格式：
    {
        'stepWidth': ...,
        'walkingSpeed': {'left': ..., 'right': ...},
        'walkingDistance': ...,
        'stepLength': {'left': ..., 'right': ...},
        'stepLengthDeviation': ...,
        'strideLength': ...,
        'footLiftHeight': {'left': ..., 'right': ...},
        '_debug': {...}
    }
    """

    avg_metrics = {
        "stepWidth": _avg_scalar(
            metrics_big.get("stepWidth"),
            metrics_small.get("stepWidth")
        ),

        "walkingSpeed": {
            "left": _avg_scalar(
                metrics_big.get("walkingSpeed", {}).get("left"),
                metrics_small.get("walkingSpeed", {}).get("left")
            ),
            "right": _avg_scalar(
                metrics_big.get("walkingSpeed", {}).get("right"),
                metrics_small.get("walkingSpeed", {}).get("right")
            ),
        },

        "walkingDistance": _avg_scalar(
            metrics_big.get("walkingDistance"),
            metrics_small.get("walkingDistance")
        ),

        "stepLength": {
            "left": _avg_scalar(
                metrics_big.get("stepLength", {}).get("left"),
                metrics_small.get("stepLength", {}).get("left")
            ),
            "right": _avg_scalar(
                metrics_big.get("stepLength", {}).get("right"),
                metrics_small.get("stepLength", {}).get("right")
            ),
        },

        "stepLengthDeviation": _avg_scalar(
            metrics_big.get("stepLengthDeviation"),
            metrics_small.get("stepLengthDeviation")
        ),

        "strideLength": _avg_scalar(
            metrics_big.get("strideLength"),
            metrics_small.get("strideLength")
        ),

        "footLiftHeight": {
            "left": _avg_scalar(
                metrics_big.get("footLiftHeight", {}).get("left"),
                metrics_small.get("footLiftHeight", {}).get("left")
            ),
            "right": _avg_scalar(
                metrics_big.get("footLiftHeight", {}).get("right"),
                metrics_small.get("footLiftHeight", {}).get("right")
            ),
        },

        "_debug": {
            "stride_L_all": _avg_list(
                metrics_big.get("_debug", {}).get("stride_L_all"),
                metrics_small.get("_debug", {}).get("stride_L_all")
            ),
            "stride_R_all": _avg_list(
                metrics_big.get("_debug", {}).get("stride_R_all"),
                metrics_small.get("_debug", {}).get("stride_R_all")
            ),
            "step_L_all": _avg_list(
                metrics_big.get("_debug", {}).get("step_L_all"),
                metrics_small.get("_debug", {}).get("step_L_all")
            ),
            "step_R_all": _avg_list(
                metrics_big.get("_debug", {}).get("step_R_all"),
                metrics_small.get("_debug", {}).get("step_R_all")
            ),
            "step_widths_all": _avg_list(
                metrics_big.get("_debug", {}).get("step_widths_all"),
                metrics_small.get("_debug", {}).get("step_widths_all")
            ),
            "lift_L_all": _avg_list(
                metrics_big.get("_debug", {}).get("lift_L_all"),
                metrics_small.get("_debug", {}).get("lift_L_all")
            ),
            "lift_R_all": _avg_list(
                metrics_big.get("_debug", {}).get("lift_R_all"),
                metrics_small.get("_debug", {}).get("lift_R_all")
            ),
            "walk_dist_L": _avg_scalar(
                metrics_big.get("_debug", {}).get("walk_dist_L"),
                metrics_small.get("_debug", {}).get("walk_dist_L")
            ),
            "walk_dist_R": _avg_scalar(
                metrics_big.get("_debug", {}).get("walk_dist_R"),
                metrics_small.get("_debug", {}).get("walk_dist_R")
            ),
        }
    }

    return avg_metrics
# =========================================================
# 主函数：直接基于 pos_L / pos_R + res_gyro 计算空间参数
# =========================================================

def stride_speeds_from_HS(
    pos,
    HS_idx,
    fs=100,
    plane_axes=(0, 1),
    ap_axis=None,
    min_stride_time=0.5,
    max_stride_time=2.0,
    min_stride_length=0.2,
    max_stride_length=2.5,
    use_median=True
):
    """
    根据同一只脚相邻 HS 事件计算逐跨步速度。

    pos: 足部位置序列，shape = [N, 3]
    HS_idx: heel strike 索引
    fs: 采样率
    plane_axes: 水平面坐标轴
    ap_axis: 前进方向轴。如果为 None，则使用水平面欧氏距离。
    """

    import numpy as np

    pos = np.asarray(pos)
    HS_idx = np.asarray(HS_idx, dtype=int)

    stride_lengths = []
    stride_times = []
    stride_speeds = []
    stride_pairs = []

    if len(HS_idx) < 2:
        return (
            np.array(stride_speeds),
            np.array(stride_lengths),
            np.array(stride_times),
            stride_pairs
        )

    for i in range(len(HS_idx) - 1):
        idx1 = HS_idx[i]
        idx2 = HS_idx[i + 1]

        if idx2 <= idx1:
            continue

        stride_time = (idx2 - idx1) / fs

        if stride_time < min_stride_time or stride_time > max_stride_time:
            continue

        if ap_axis is not None:
            stride_length = abs(pos[idx2, ap_axis] - pos[idx1, ap_axis])
        else:
            p1 = pos[idx1, list(plane_axes)]
            p2 = pos[idx2, list(plane_axes)]
            stride_length = float(np.linalg.norm(p2 - p1))

        if stride_length < min_stride_length or stride_length > max_stride_length:
            continue

        stride_speed = stride_length / stride_time

        stride_lengths.append(stride_length)
        stride_times.append(stride_time)
        stride_speeds.append(stride_speed)
        stride_pairs.append((idx1, idx2))

    return (
        np.asarray(stride_speeds, dtype=float),
        np.asarray(stride_lengths, dtype=float),
        np.asarray(stride_times, dtype=float),
        stride_pairs
    )

def compute_spatial_gait_metrics(
    pos_L,
    pos_R,
    res_gyro,
    fs=100,
    plane_axes=(0, 1),   # 水平面，默认 x-y
    ml_axis=0,           # 左右方向
    v_axis=2             # 竖直方向
):
    left = res_gyro["left"]
    right = res_gyro["right"]

    # -----------------------------
    # 1) stride length
    # -----------------------------
    stride_L, stride_pairs_L = stride_lengths_from_HS(
        pos_L, left["HS_idx"], fs=fs, plane_axes=plane_axes
    )
    stride_R, stride_pairs_R = stride_lengths_from_HS(
        pos_R, right["HS_idx"], fs=fs, plane_axes=plane_axes
    )

    stride_length_left = _safe_mean(stride_L)
    stride_length_right = _safe_mean(stride_R)
    stride_length = _safe_mean([x for x in [stride_length_left, stride_length_right] if x is not None])

    # -----------------------------
    # 2) step length
    # -----------------------------
    step_L, step_pairs_L = step_lengths_TO_to_next_HS(
        pos_L, left["TO_idx"], left["HS_idx"], fs=fs, plane_axes=plane_axes
    )
    step_R, step_pairs_R = step_lengths_TO_to_next_HS(
        pos_R, right["TO_idx"], right["HS_idx"], fs=fs, plane_axes=plane_axes
    )

    # step_length_left = _safe_mean(step_L)/2
    # step_length_right = _safe_mean(step_R)/2

    step_length_left, step_length_right = step_length_from_stride_lengths(
        stride_length_left,
        stride_length_right,
    )


    step_length_deviation = (
        abs(step_length_left - step_length_right)
        if (step_length_left is not None and step_length_right is not None)
        else None
    )

    # -----------------------------
    # 3) walking distance
    # -----------------------------
    walk_dist_L = _safe_sum(step_L)
    walk_dist_R = _safe_sum(step_R)
    walking_distance = (walk_dist_L + walk_dist_R) / 2.0

    # # -----------------------------
    # # 4) walking speed
    # # -----------------------------
    # # 这里用总步长 / 总步时间
    # step_time_left_arr, _ = _next_event_pairs(
    #     left["TO_idx"], left["HS_idx"], fs=fs, min_s=0.20, max_s=2.00
    # )
    # step_time_right_arr, _ = _next_event_pairs(
    #     right["TO_idx"], right["HS_idx"], fs=fs, min_s=0.20, max_s=2.00
    # )

    # total_time_left = _safe_sum(step_time_left_arr)
    # total_time_right = _safe_sum(step_time_right_arr)
    # total_time_all = total_time_right + total_time_left

    # # walking_speed_left = (walk_dist_L / total_time_left)/2 if total_time_left > 0 else None
    # # walking_speed_right = (walk_dist_R / total_time_right)/2 if total_time_right > 0 else None
    # # walking_speed_left = (walk_dist_L / total_time_all)/2 if total_time_all > 0 else None
    # # walking_speed_right = (walk_dist_R / total_time_all)/2 if total_time_all > 0 else None

    # walking_speed_left = (6/ total_time_all)/2 if total_time_all > 0 else None
    # walking_speed_right = (6/ total_time_all)/2 if total_time_all > 0 else None

    # print("total_time_all",total_time_all)


    # -----------------------------
    # 4) walking speed
    # -----------------------------
    # 推荐：用跨步长 / 跨步时间计算逐跨步速度
    # 如果已知前进方向轴，可以用 ap_axis；否则用水平面欧氏距离。
    # 当前 plane_axes=(0,1), ml_axis=0 时，通常 ap_axis 可以设为 1。
    ap_axis = None
    for ax in plane_axes:
        if ax != ml_axis:
            ap_axis = ax
            break

    stride_speed_L, stride_len_speed_L, stride_time_L, stride_speed_pairs_L = stride_speeds_from_HS(
        pos_L,
        left["HS_idx"],
        fs=fs,
        plane_axes=plane_axes,
        ap_axis=ap_axis
    )

    stride_speed_R, stride_len_speed_R, stride_time_R, stride_speed_pairs_R = stride_speeds_from_HS(
        pos_R,
        right["HS_idx"],
        fs=fs,
        plane_axes=plane_axes,
        ap_axis=ap_axis
    )

    walking_speed_left = _safe_mean(stride_speed_L)
    walking_speed_right = _safe_mean(stride_speed_R)

    all_stride_speeds = []

    if stride_speed_L is not None and len(stride_speed_L) > 0:
        all_stride_speeds.extend(stride_speed_L.tolist())

    if stride_speed_R is not None and len(stride_speed_R) > 0:
        all_stride_speeds.extend(stride_speed_R.tolist())

    walking_speed = _safe_mean(all_stride_speeds)


    # -----------------------------
    # 5) step width
    # -----------------------------
    step_widths, step_width_pairs = step_width_from_alternating_HS(
        pos_L, left["HS_idx"],
        pos_R, right["HS_idx"],
        ml_axis=ml_axis,
        fs=fs
    )
    step_width = _safe_mean(step_widths)

    # -----------------------------
    # 6) foot lift height
    # -----------------------------
    lift_L, lift_pairs_L = foot_lift_heights_from_TO_HS(
        pos_L, left["TO_idx"], left["HS_idx"], v_axis=v_axis, fs=fs
    )
    lift_R, lift_pairs_R = foot_lift_heights_from_TO_HS(
        pos_R, right["TO_idx"], right["HS_idx"], v_axis=v_axis, fs=fs
    )

    foot_lift_height_left = _safe_mean(lift_L)
    foot_lift_height_right = _safe_mean(lift_R)

    # -----------------------------
    # 7) 输出
    # -----------------------------
    metrics = {
        "stepWidth": step_width,
        "walkingSpeed": {
            "left": walking_speed_left,
            "right": walking_speed_right,
        },
        "walkingDistance": walking_distance,
        "stepLength": {
            "left": step_length_left,
            "right": step_length_right,
        },
        "stepLengthDeviation": step_length_deviation,
        "strideLength": stride_length,
        "footLiftHeight": {
            "left": foot_lift_height_left,
            "right": foot_lift_height_right,
        },

        # 可选调试输出
        "_debug": {
            "stride_L_all": stride_L.tolist(),
            "stride_R_all": stride_R.tolist(),
            "step_L_all": step_L.tolist(),
            "step_R_all": step_R.tolist(),
            "step_widths_all": step_widths.tolist(),
            "lift_L_all": lift_L.tolist(),
            "lift_R_all": lift_R.tolist(),
            "walk_dist_L": walk_dist_L,
            "walk_dist_R": walk_dist_R,
        }
    }
    return metrics


def run_spatial_pipeline_from_arrays(arr2, arr3, res_gyro, fs=100):
    """
    arr2: 右脚 IMU 9轴
    arr3: 左脚 IMU 9轴
    res_gyro: 事件检测结果，包含 left/right 的 HS_idx, TO_idx 等
    """

    # =====================================================
    # 1) 左右脚四元数
    # =====================================================
    mgr = QuaternionManager(fs=fs)

    node_R = MahonyOrientationNode(
        name="R_foot",
        fs=fs,
        use_mag=False,
        acc_unit="g",
        gyr_unit="deg",
        kp=0.8,
        ki=1e-5,
    )
    node_R.init_from_static(
        imu9=arr2,
        n_first=200,
        n_init=400,
        estimate_gyro_bias=False,
    )
    mgr.add_existing_node(node_R)
    Q_R = mgr.run_batch({"R_foot": arr2})
    quat_R_wxyz = Q_R["R_foot"]

    node_L = MahonyOrientationNode(
        name="L_foot",
        fs=fs,
        use_mag=False,
        acc_unit="g",
        gyr_unit="deg",
        kp=0.8,
        ki=1e-5,
    )
    node_L.init_from_static(
        imu9=arr3,
        n_first=100,
        n_init=400,
        estimate_gyro_bias=False,
    )
    mgr.add_existing_node(node_L)
    Q_L = mgr.run_batch({"L_foot": arr3})
    quat_L_wxyz = Q_L["L_foot"]

    # =====================================================
    # 2) 转世界系并去重力
    # =====================================================
    acc_world_L = rotate_acc_to_world(
        acc_s=arr3[:, 0:3],
        quat_wxyz=quat_L_wxyz,
        direction="world<-sensor",
        acc_unit="g",
    )
    acc_lin_L, g_vec_L, bias_L = remove_gravity_and_static_bias(
        acc_world_L,
        fs=fs,
        n_init=200,
    )

    acc_world_R = rotate_acc_to_world(
        acc_s=arr2[:, 0:3],
        quat_wxyz=quat_R_wxyz,
        direction="world<-sensor",
        acc_unit="g",
    )
    acc_lin_R, g_vec_R, bias_R = remove_gravity_and_static_bias(
        acc_world_R,
        fs=fs,
        n_init=200,
    )

    # =====================================================
    # 3) ZUPT 积分得到速度、位置
    # =====================================================
    left = res_gyro["left"]
    right = res_gyro["right"]

    zupt_L = zupt_mask_from_hs_to(
        n=len(acc_lin_L),
        HS_idx=left["HS_idx"],
        TO_idx=left["TO_idx"],
        fs=fs,
        min_stance_s=0.25,
        max_stance_s=1.8,
        pad_s=0.02,
    )
    vel_L, pos_L = zupt_integrate_position(
        acc_lin_L,
        fs=fs,
        zupt_mask=zupt_L,
    )

    zupt_R = zupt_mask_from_hs_to(
        n=len(acc_lin_R),
        HS_idx=right["HS_idx"],
        TO_idx=right["TO_idx"],
        fs=fs,
        min_stance_s=0.25,
        max_stance_s=1.8,
        pad_s=0.02,
    )
    vel_R, pos_R = zupt_integrate_position(
        acc_lin_R,
        fs=fs,
        zupt_mask=zupt_R,
    )

    # =====================================================
    # 4) 计算空间参数
    # =====================================================
    metrics = compute_spatial_gait_metrics(
        pos_L=pos_L,
        pos_R=pos_R,
        res_gyro=res_gyro,
        fs=fs,
        plane_axes=(0, 1),  # 默认水平面
        ml_axis=0,
        v_axis=2
    )

    return {
        "metrics": metrics,
        "pos_L": pos_L,
        "pos_R": pos_R,
        "vel_L": vel_L,
        "vel_R": vel_R,
        "quat_L_wxyz": quat_L_wxyz,
        "quat_R_wxyz": quat_R_wxyz,
    }


def run_dual_detector_spatial_average_pipeline(arr2, arr3, fs=100):
    # ==============================
    # 1) 偏大版本
    # ==============================
    res_gyro_big = gait_to_hs_from_filtered_gyro_x_adaptive(
        left_gyr_x=arr3[:, 3],
        right_gyr_x=arr2[:, 3],
        fs=fs,
        input_unit='deg',
        hs_method = "platform",
        return_debug=True
    )

    result_big = run_spatial_pipeline_from_arrays(
        arr2=arr2,
        arr3=arr3,
        res_gyro=res_gyro_big,
        fs=fs
    )

    # ==============================
    # 2) 偏小版本
    # ==============================
    res_gyro_small = gait_to_hs_from_filtered_gyro_x_adaptive_balanced(
        left_gyr_x=arr3[:, 3],
        right_gyr_x=arr2[:, 3],
        fs=fs,
        input_unit='deg',
        hs_balance_alpha=0.1,
        return_debug=True
    )

    result_small = run_spatial_pipeline_from_arrays(
        arr2=arr2,
        arr3=arr3,
        res_gyro=res_gyro_small,
        fs=fs
    )

    # ==============================
    # 3) 平均后版本（结构完全对应）
    # ==============================
    metrics_avg = average_metrics_same_format(
        result_big["metrics"],
        result_small["metrics"]
    )

    return {
        "metrics_big": result_big["metrics"],
        "metrics_small": result_small["metrics"],
        "metrics_avg": metrics_avg,
        "res_gyro_big": res_gyro_big,
        "res_gyro_small": res_gyro_small,
        "result_big": result_big,
        "result_small": result_small,
    }

