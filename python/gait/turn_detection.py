import numpy as np
from gait.event_detection import gait_to_hs_from_filtered_gyro_x_adaptive
from sensor.data_reader import load_calibrated_filtered_arrays
import matplotlib.pyplot as plt


def moving_average(x: np.ndarray, win: int) -> np.ndarray:
    x = np.asarray(x, dtype=float).reshape(-1)
    if win <= 1:
        return x.copy()
    kernel = np.ones(win, dtype=float) / win
    return np.convolve(x, kernel, mode="same")


def detect_tug_turn_by_levels(
    yaw_rad: np.ndarray,
    fs: float,
    *,
    smooth_win: int = 11,
    pre_window_s: tuple = (20.0, 23.0),   # 转身前平台时间窗口
    post_window_s: tuple = (35.0, 39.0),  # 转身后平台时间窗口
    start_thresh_deg: float = 10.0,       # 离开前平台阈值
    end_thresh_deg: float = 10.0,         # 进入后平台阈值
    stable_duration_s: float = 0.5        # 认为“进入平台”需要持续多久
):
    """
    基于前后平台检测 TUG 单次转身片段
    """

    yaw_rad = np.asarray(yaw_rad, dtype=float).reshape(-1)
    n = len(yaw_rad)
    if n < 10:
        return None, {}

    yaw_smooth = moving_average(yaw_rad, smooth_win)
    yaw_unwrap = np.unwrap(yaw_smooth)

    # 1) 平台区间
    pre_s = int(round(pre_window_s[0] * fs))
    pre_e = int(round(pre_window_s[1] * fs))
    post_s = int(round(post_window_s[0] * fs))
    post_e = int(round(post_window_s[1] * fs))

    pre_s = max(0, min(pre_s, n-1))
    pre_e = max(pre_s + 1, min(pre_e, n))
    post_s = max(0, min(post_s, n-1))
    post_e = max(post_s + 1, min(post_e, n))

    pre_level = float(np.mean(yaw_unwrap[pre_s:pre_e]))
    post_level = float(np.mean(yaw_unwrap[post_s:post_e]))

    start_thresh = np.deg2rad(start_thresh_deg)
    end_thresh = np.deg2rad(end_thresh_deg)
    stable_n = max(1, int(round(stable_duration_s * fs)))

    # 2) 找开始点：首次持续离开前平台
    start_idx = None
    for i in range(pre_e, n - stable_n):
        seg = yaw_unwrap[i:i + stable_n]
        if np.all(np.abs(seg - pre_level) > start_thresh):
            start_idx = i
            break

    if start_idx is None:
        return None, {
            "yaw_smooth": yaw_smooth,
            "yaw_unwrap": yaw_unwrap,
            "pre_level": pre_level,
            "post_level": post_level
        }

    # 3) 找结束点：首次持续进入后平台
    end_idx = None
    for i in range(start_idx + stable_n, n - stable_n):
        seg = yaw_unwrap[i:i + stable_n]
        if np.all(np.abs(seg - post_level) < end_thresh):
            end_idx = i
            break

    if end_idx is None:
        return None, {
            "yaw_smooth": yaw_smooth,
            "yaw_unwrap": yaw_unwrap,
            "pre_level": pre_level,
            "post_level": post_level,
            "start_idx": start_idx
        }

    angle_deg = np.rad2deg(yaw_unwrap[end_idx] - yaw_unwrap[start_idx])

    turn = {
        "start_idx": int(start_idx),
        "end_idx": int(end_idx),
        "start_time": float(start_idx / fs),
        "end_time": float(end_idx / fs),
        "duration_s": float((end_idx - start_idx + 1) / fs),
        "angle_deg": float(angle_deg),
        "direction": "left" if angle_deg > 0 else "right"
    }

    debug = {
        "yaw_smooth": yaw_smooth,
        "yaw_unwrap": yaw_unwrap,
        "pre_level": pre_level,
        "post_level": post_level
    }

    return turn, debug


def plot_tug_turn_levels(yaw_rad, fs, turn=None, debug=None, title="Pelvis TUG Turn Detection"):
    yaw_rad = np.asarray(yaw_rad, dtype=float).reshape(-1)
    t = np.arange(len(yaw_rad)) / fs

    plt.figure(figsize=(12, 5))
    plt.plot(t, yaw_rad, label="Yaw (rad)", linewidth=2)

    if debug is not None:
        plt.axhline(debug["pre_level"], color="green", linestyle="--", label="Pre-turn level")
        plt.axhline(debug["post_level"], color="orange", linestyle="--", label="Post-turn level")

    if turn is not None:
        plt.axvline(turn["start_time"], color="red", linewidth=2, label="Turn Start")
        plt.axvline(turn["end_time"], color="purple", linewidth=2, label="Turn End")

    plt.xlabel("Time (s)")
    plt.ylabel("Angle (rad)")
    plt.title(title)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

# def get_best_turn_segment(turn_or_turns):
#     """
#     如果传入:
#     - dict: 直接返回
#     - list/tuple: 返回第 3 个片段
#     """
#     if isinstance(turn_or_turns, dict):
#         return turn_or_turns
#
#     if isinstance(turn_or_turns, (list, tuple)):
#         if len(turn_or_turns) < 3:
#             raise ValueError(f"候选片段不足 3 个，当前只有 {len(turn_or_turns)} 个")
#         return turn_or_turns[-1]
#
#     raise TypeError("输入必须是 dict 或 list/tuple")
def get_best_turn_segment(turn_or_turns):
    """
    如果传入:
    - None: 返回 None
    - dict: 直接返回
    - list/tuple: 返回最后一个片段
    """
    if turn_or_turns is None:
        return None

    if isinstance(turn_or_turns, dict):
        return turn_or_turns

    if isinstance(turn_or_turns, (list, tuple)):
        if len(turn_or_turns) == 0:
            return None
        return turn_or_turns[-1]

    raise TypeError(f"输入必须是 None、dict 或 list/tuple，当前类型是 {type(turn_or_turns)}")
def select_best_turn_segment(turns, min_duration_s=1.0, max_duration_s=5.0):
    """
    从多个候选转身段中选一个最合理的最终片段
    兼容：
    - 单个 dict
    - list[dict]
    """
    if turns is None:
        return None

    # 如果传进来的是单个 dict，直接返回
    if isinstance(turns, dict):
        return turns

    # 如果不是 list/tuple，报错更清楚
    if not isinstance(turns, (list, tuple)):
        raise TypeError(f"turns 应该是 dict 或 list[dict]，当前类型是 {type(turns)}")

    if len(turns) == 0:
        return None

    candidates = [
        t for t in turns
        if isinstance(t, dict) and min_duration_s <= t["duration_s"] <= max_duration_s
    ]

    if not candidates:
        return None

    # 选绝对转角最大的那一个
    best = max(candidates, key=lambda x: abs(x["angle_deg"]))
    return best
def count_turn_steps_from_hs(
    turn_segment: dict | None,
    left_hs_idx,
    right_hs_idx,
    *,
    margin_before: int = 0,
    margin_after: int = 0
) -> dict:
    """
    统计转身片段内的左右脚 HS 数量，并估计转身步数
    """
    if turn_segment is None:
        return {
            "turnStartIdx": None,
            "turnEndIdx": None,
            "turnStepsLeft": 0,
            "turnStepsRight": 0,
            "turnContacts": 0,
            "turnSteps": 0,
            "leftHsInTurn": [],
            "rightHsInTurn": []
        }

    start_idx = int(turn_segment["start_idx"]) - int(margin_before)
    end_idx = int(turn_segment["end_idx"]) + int(margin_after)

    left_hs_idx = np.asarray(left_hs_idx, dtype=int).reshape(-1)
    right_hs_idx = np.asarray(right_hs_idx, dtype=int).reshape(-1)

    left_in_turn = left_hs_idx[(left_hs_idx >= start_idx) & (left_hs_idx <= end_idx)]
    right_in_turn = right_hs_idx[(right_hs_idx >= start_idx) & (right_hs_idx <= end_idx)]

    left_count = int(len(left_in_turn))
    right_count = int(len(right_in_turn))
    total_contacts = left_count + right_count

    # 实用版估计：左右落地事件数 / 2
    estimated_turn_steps = int(round(total_contacts))

    return {
        "turnStartIdx": start_idx,
        "turnEndIdx": end_idx,
        "turnStepsLeft": left_count,
        "turnStepsRight": right_count,
        "turnContacts": total_contacts,
        "turnSteps": estimated_turn_steps,
        "leftHsInTurn": left_in_turn.tolist(),
        "rightHsInTurn": right_in_turn.tolist()
    }

