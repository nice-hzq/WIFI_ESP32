import numpy as np
from matplotlib import pyplot as plt

import numpy as np

from gait.event_detection import gait_to_hs_from_filtered_gyro_x_for_distance
from sensor.data_reader import load_calibrated_filtered_arrays
from orientation.quaternion_manager import QuaternionManager, MahonyOrientationNode, rotate_acc_to_world, \
    remove_gravity_and_static_bias

G0 = 9.80665
def intervals_to_mask(intervals, n: int):
    """把[(s,e),...]转成 bool mask"""
    mask = np.zeros(n, dtype=bool)
    for s, e in intervals:
        mask[s:e+1] = True
    return mask

def step_distance_TO_to_next_HS(
    pos_world: np.ndarray,
    TO_idx: np.ndarray,
    HS_idx: np.ndarray,
    fs: float,
    *,
    plane_axes=(0,1),          # 水平面 x-y
    min_swing_s: float = 0.20, # 最小摆动时间
    max_swing_s: float = 0.80, # 最大摆动时间
):
    pos_world = np.asarray(pos_world, float)
    TO = np.sort(np.asarray(TO_idx, int))
    HS = np.sort(np.asarray(HS_idx, int))

    min_s = int(round(min_swing_s * fs))
    max_s = int(round(max_swing_s * fs))

    step = []
    pairs = []

    for t0 in TO:
        hs_next = HS[HS > t0]
        if hs_next.size == 0:
            continue
        t1 = int(hs_next[0])

        # swing time constraint
        dt = t1 - t0
        if dt < min_s or dt > max_s:
            continue

        p0 = pos_world[t0, list(plane_axes)]
        p1 = pos_world[t1, list(plane_axes)]
        d = float(np.linalg.norm(p1 - p0))
        step.append(d)
        pairs.append((t0, t1))

    return np.array(step, float), pairs


def quat_to_R_wxyz(q: np.ndarray) -> np.ndarray:
    """
    q: (N,4) wxyz
    return R: (N,3,3) mapping body -> world, v_w = R @ v_b
    """
    q = np.asarray(q, float)
    w, x, y, z = q[:,0], q[:,1], q[:,2], q[:,3]

    R = np.empty((len(q), 3, 3), dtype=float)
    R[:,0,0] = 1 - 2*(y*y + z*z)
    R[:,0,1] = 2*(x*y - w*z)
    R[:,0,2] = 2*(x*z + w*y)

    R[:,1,0] = 2*(x*y + w*z)
    R[:,1,1] = 1 - 2*(x*x + z*z)
    R[:,1,2] = 2*(y*z - w*x)

    R[:,2,0] = 2*(x*z - w*y)
    R[:,2,1] = 2*(y*z + w*x)
    R[:,2,2] = 1 - 2*(x*x + y*y)
    return R

def acc_body_to_lin_world(
    acc_body_g: np.ndarray,
    quat_wxyz: np.ndarray,
    *,
    gravity_axis_world: int = 2,  # world Z
):
    """
    acc_body_g: (N,3) in g
    quat_wxyz : (N,4) wxyz (body->world)
    return:
      acc_world: (N,3) m/s^2 (包含重力)
      acc_lin  : (N,3) m/s^2 (去重力后的线加速度)
    说明：这里假设 world 的 +Z 为竖直向上（g 向下）
    """
    acc_body = np.asarray(acc_body_g, float) * G0
    q = np.asarray(quat_wxyz, float)
    n = min(len(acc_body), len(q))
    acc_body = acc_body[:n]
    q = q[:n]

    R = quat_to_R_wxyz(q)
    acc_world = np.einsum("nij,nj->ni", R, acc_body)

    gvec = np.zeros(3, float)
    gvec[gravity_axis_world] = G0
    acc_lin = acc_world - gvec
    return acc_world, acc_lin

from gait.tool import stride_lengths_from_HS, step_lengths_TO_to_next_HS


def zupt_mask_from_hs_to(
    n: int,
    HS_idx: np.ndarray,
    TO_idx: np.ndarray,
    *,
    fs: float,
    min_stance_s: float = 0.25,
    max_stance_s: float = 1.20,
    pad_s: float = 0.02,   # 给 stance 两端留一点余量更稳（可调）

    include_initial_stance: bool = True,   # 起步前静止段
    initial_start_idx: int = 0,

    include_terminal_stance: bool = True,  # 结束后静止段
    terminal_end_idx: int | None = None,   # 结束静止段终点，默认 n-1
    terminal_min_s: float = 0.20,          # 末端静止段至少持续这么久才保留
):
    """
    根据事件构造 ZUPT mask（True 表示零速）

    包含三类零速段：
    1) 起步前静止段: initial_start_idx -> TO(0)-pad
    2) 常规支撑段:   HS(k)+pad -> TO(k)-pad
    3) 结束后静止段: last_HS+pad -> terminal_end_idx
       （只有长度足够时才加入）

    注意：
    - 这里的末端静止段默认从最后一个 HS 之后开始补。
    - 如果你的数据最后一个事件更适合用 last_TO 或其他点，也可以再改。
    """
    HS = np.sort(np.asarray(HS_idx, int))
    TO = np.sort(np.asarray(TO_idx, int))

    mask = np.zeros(n, dtype=bool)

    min_len = int(round(min_stance_s * fs))
    max_len = int(round(max_stance_s * fs))
    pad = int(round(pad_s * fs))
    terminal_min_len = int(round(terminal_min_s * fs))

    if terminal_end_idx is None:
        terminal_end_idx = n - 1
    terminal_end_idx = max(0, min(n - 1, int(terminal_end_idx)))

    # =========================================================
    # 1) 起步前静止段
    # =========================================================
    if include_initial_stance and TO.size > 0:
        first_to = int(TO[0])

        s0 = max(0, int(initial_start_idx))
        e0 = min(n - 1, first_to - pad)

        if e0 > s0:
            mask[s0:e0 + 1] = True

    # =========================================================
    # 2) 常规 HS -> TO 支撑段
    # =========================================================
    for hs in HS:
        cand = TO[TO > hs]
        if cand.size == 0:
            continue

        to = int(cand[0])
        L = to - hs

        if L < min_len or L > max_len:
            continue

        s = max(0, hs + pad)
        e = min(n - 1, to - pad)

        if e > s:
            mask[s:e + 1] = True

    # =========================================================
    # 3) 结束后静止段
    # =========================================================
    if include_terminal_stance and HS.size > 0:
        last_hs = int(HS[-1])

        s1 = max(0, last_hs + pad)
        e1 = terminal_end_idx

        if e1 > s1 and (e1 - s1 + 1) >= terminal_min_len:
            mask[s1:e1 + 1] = True

    return mask
def zupt_integrate_position(
    acc_lin_world: np.ndarray,
    fs: float,
    zupt_mask: np.ndarray,
):
    """
    acc_lin_world: (N,3) m/s^2
    zupt_mask: (N,) True=zero-velocity
    return: vel (N,3), pos (N,3)
    """
    a = np.asarray(acc_lin_world, float)
    zupt = np.asarray(zupt_mask, bool)
    n = len(a)
    dt = 1.0 / fs

    # 1) integrate -> vel (no hard reset yet)
    vel = np.zeros((n, 3), float)
    for i in range(1, n):
        vel[i] = vel[i-1] + 0.5 * (a[i-1] + a[i]) * dt

    # 2) drift correction per swing segment
    non = ~zupt
    i = 0
    while i < n:
        if not non[i]:
            i += 1
            continue
        s = i
        while i < n and non[i]:
            i += 1
        e = i - 1

        L = e - s + 1
        if L < 2:
            vel[s] = 0.0
            continue

        # 关键：用 swing 段首尾速度
        v_start = vel[s].copy()
        v_end   = vel[e].copy()

        for k in range(L):
            alpha = k / (L - 1)
            vel[s + k] = vel[s + k] - ((1 - alpha) * v_start + alpha * v_end)

    # 3) enforce zero velocity only at zupt centers (optional but safe)
    vel[zupt] = 0.0

    # 4) integrate -> pos
    pos = np.zeros((n, 3), float)
    for i in range(1, n):
        pos[i] = pos[i-1] + 0.5 * (vel[i-1] + vel[i]) * dt

    return vel, pos


