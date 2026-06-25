# # -*- coding: utf-8 -*-
# ========================
# 算法参数全局变量（由 C# 传入）
# ========================
import numpy as np
from queue import Queue

from orientation.quaternions import QuaternionManager

# 采样率
fs = 50
# 算法临时目录，例如 C# 传入的 tempDir
tempDir = ""

# 上位机原始 CSV 保存目录 originalDir
originalDir = ""

# 算法报告曲线输出目录 curveDir
curveDir = ""

# 坐标系类型（1=人体，2=火柴人）
coordinateType = 1
# 工作模式
WORK_MODE = None # 默认你可以按需要改成 "full_body" 或 "upper_body"或"lower_body"

# 队列大小定义
_sensor_queue = Queue(maxsize=10)
frame_counter =0
# ==== 校准总体状态 ====
_calibration_in_progress = False
# 0: 无校准, 1: 传感器零偏校准, 2: T-pose 人体姿态校准
_calibration_type = 0

# ========================

# ==== T-pose 校准 ====
_tpose_ref = {}         # T-pose 参考四元数（每个节点一个）
_tpose_sample_buffer = []     # T-pose 采样缓存（用于平均）
_tpose_target_samples = 1000  # 需要的 T-pose 样本帧数
_tpose_use_last_n = 500
_tpose_calib_done = True
# ==== 计算参数 ====
_mahony_filters = {}
_current_quat = {}
_last_ts_ms = {}
_calib_count = 0
_calib_quat_sum = {}
_ref_quat = {}

USE_MAG = True

# ============================================================
# ☆ 1. 零偏校准相关（type = 1）
# ============================================================

# 结果状态
_sensor_bias_ready = None   # 传感器零偏是否已就绪（离线/在线均可）
     # T-pose 是否已就绪
_is_calibrated = False        # 两者都 OK 时为 True

# ==== 传感器零偏校准 ====
_calib_buffer = {}            # 静止数据缓存（按 alias 存 acc/gyro）
_calib_current_samples = 0    # 当前进度

CALIB_FRAMES_PER_FACE = 2000   # 六面中每一面固定100帧
CALIB_TOTAL_SAMPLES = CALIB_FRAMES_PER_FACE * 6
_calibration_callback_counter_sensor = 0

_sensor_calib_sucess = False
# 当前面索引、当前面名
_face_idx = 0
# 六面顺序
CALIB_FACES = ["+Z", "-Z", "+Y", "-Y", "+X", "-X"]
# CALIB_FACES = ["+Z", "+Y", "+X"]
_expected_aliases = None
# 每一面目标帧数
# 只取中间稳定区间（你说你要）
# 或：
CALIB_VALID_RANGE_RATIO = (0.4, 0.8)
# 超时（单面）
_current_face = None
# 六面采集缓冲：每个 face 下，每个 alias 都有 acc/gyro 列表
# _calib_faces_buffer[face][alias] = {"acc": [...], "gyro":[...]}
_calib_faces_buffer = {}

# 最终结果（写回内存）
_gyro_bias_degps = {}   # alias -> (bx,by,bz)
_acc_bias_g = {}        # alias -> (bx,by,bz) in g
_acc_scale_g = {}       # alias -> (sx,sy,sz) in g-per-g (scale)
# ===== Magnetometer calibration =====
_mag_bias = {}
_mag_A = {}
_mag_method = {}
_mag_stats = {}
_mag_cols = {}

# ===== Runtime control =====
_use_magnetometer = False
_mag_weight = {}


from core.quaternion import euler_to_quat

# 每个 IMU 的“安装姿态”——这里约定存的是：
#   身体段坐标系 -> 传感器坐标系 的旋转（欧拉角，单位：度）
INSTALL_POSE_DEG = {
    # 躯干这几个假设与身体坐标一致，不需要旋转
    "S1":  (0.0,   0.0,   0.0),
    "T12": (0.0,   0.0,   0.0),
    "T1":  (0.0,   0.0,   0.0),
    "H":   (0.0,   0.0,   0.0),

    "L1": (90.0, 90.0, 0.0),
    "L2": (90.0, 90.0, 0.0),
    "L3": (90.0, 90.0, 0.0),
    # "L1": (0.0, 0.0, 0.0),
    # "L2": (0.0, 0.0, 0.0),
    # "L3": (0.0, 0.0, 0.0),
    #
    "R1": (90.0, -90.0, 0.0),
    "R2": (90.0, -90.0, 0.0),
    "R3": (90.0, -90.0, 0.0),

    # "R1": (0.0, 0.0, 0.0),
    # "R2": (0.0, 0.0, 0.0),
    # "R3": (0.0, 0.0, 0.0),

    # ===== 关键：腿部全部绕 y 轴 180° =====
    # "L4":  (0.0, 0.0, 0.0),
    # "L5":  (0.0, 0.0, 0.0),
    # "L6":  (0.0, 0.0, 0.0),
    "R4":  (0.0, 180.0, 0.0),
    "R5":  (0.0, 180.0, 0.0),
    "R6":  (0.0, 180.0, 0.0),
    # "R4": (0.0, 0.0, 0.0),
    # "R5": (0.0, 0.0, 0.0),
    # "R6": (0.0, 0.0, 0.0),
    "L4": (0.0, 180.0, 0.0),
    "L5": (0.0, 180.0, 0.0),
    "L6": (0.0, 180.0, 0.0),
    # "R4": (-90.0, 0.0, 0.0),
    # "R5": (-90.0, 180.0, 0.0),
    # "R6": (-90.0, 180.0, 0.0),

    # "R4": (0.0, 180.0, 0.0),
    # "R5": (0.0, 180.0, 0.0),
    # "R6": (0.0, 180.0, 0.0),
}

# 启动时把上面的欧拉角全部转成四元数
INSTALL_POSE_Q = {}
for alias, (rx_deg, ry_deg, rz_deg) in INSTALL_POSE_DEG.items():
    rx = np.radians(rx_deg)
    ry = np.radians(ry_deg)
    rz = np.radians(rz_deg)
    INSTALL_POSE_Q[alias] = euler_to_quat(rx, ry, rz)  # ndarray 或 list 都可以

_last_joint_euler = {}

# 模块级全局：每个关节在 T-pose 时的零位（单位：度）
_joint_zero_euler = {}   # {joint_name: (roll_deg, pitch_deg, yaw_deg)}

# 滤波参数
# ============ 实时滤波状态（每个 alias 一份） ============
_acc_ema = {}
_gyr_ema = {}
_mag_ema = {}

# 可选：中值滤波用的短窗口缓存（每个 alias 一份）
_acc_win = {}
_gyr_win = {}
_mag_win = {}

testsensor ="H"

Q_IDENTITY = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
MOUNT_Q = {
    "S1": Q_IDENTITY,   # 如果你的 S1 也是按 x右 y下 z前 装的，就先这样
    "L4": Q_IDENTITY,   # 左大腿
    "L5": Q_IDENTITY,   # 左小腿
    "L6": Q_IDENTITY, # 左脚
    "R4": Q_IDENTITY,   # 右大腿
    "R5": Q_IDENTITY,   # 右小腿
    "R6": Q_IDENTITY, # 右脚
}

manager = QuaternionManager(fs=50)

# 注册节点
manager.add_node("S1")
manager.add_node("L4")
manager.add_node("L5")
manager.add_node("L6")
manager.add_node("R4")
manager.add_node("R5")
manager.add_node("R6")

print("当前已注册节点:", list(manager.nodes.keys()))
def reset_runtime_state():
    """重置算法运行时状态"""

    # ---------- 通用运行状态 ----------
    global _calibration_in_progress, _calibration_type
    global _sensor_bias_ready, _is_calibrated
    global _tpose_calib_done

    _calibration_in_progress = False
    _calibration_type = 0
    _sensor_bias_ready = False
    _is_calibrated = False
    _tpose_calib_done = False

    # ---------- 六面校准状态 ----------
    global _6face_frames, _calib_face_idx, _calib_current_face
    global _calib_current_samples, _expected_aliases
    global _sensor_calib_done_reported, _calibration_callback_counter_sensor

    _6face_frames = None
    _calib_face_idx = 0
    _calib_current_face = None
    _calib_current_samples = 0
    _expected_aliases = None
    _sensor_calib_done_reported = False
    _calibration_callback_counter_sensor = 0

    # ---------- T-pose 校准状态 ----------
    global _tpose_sample_buffer, _tpose_target_samples, _tpose_current_samples
    global _tpose_ref, _tpose_joint_ref, _tpose_last_progress

    _tpose_sample_buffer = []
    _tpose_target_samples = 1000
    _tpose_current_samples = 0
    _tpose_ref = {}
    _tpose_joint_ref = {}
    _tpose_last_progress = -1

    # ---------- 其他缓存/计数 ----------
    global frame_counter
    frame_counter = 0


# ============================================================
# 配置文件加载（config.json）
# ============================================================
import json as _json


def load_config(config_path: str = None):
    """
    从 config.json 加载静态配置并更新模块全局变量。

    加载规则：
      - 若 config_path 为 None，依次查找：环境变量 CONFIG_PATH → 当前目录 config.json → 项目根目录 config.json
      - 文件不存在时静默跳过，使用模块默认值
      - 已通过代码设置的路径（tempDir/originalDir/curveDir）不会被覆盖
    """
    global fs, USE_MAG, WORK_MODE, coordinateType
    global CALIB_FACES, CALIB_FRAMES_PER_FACE, CALIB_VALID_RANGE_RATIO
    global _tpose_target_samples, _tpose_use_last_n
    global INSTALL_POSE_DEG, INSTALL_POSE_Q

    if config_path is None:
        import os as _os
        candidates = []
        env_path = _os.environ.get("CONFIG_PATH")
        if env_path:
            candidates.append(env_path)
        candidates.append(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "..", "config.json"))
        for c in candidates:
            if _os.path.isfile(c):
                config_path = c
                break
        if config_path is None:
            print("[CONFIG] 未找到 config.json，使用默认值")
            return

    try:
        with open(config_path, "r", encoding="utf-8") as f:
            cfg = _json.load(f)
        print(f"[CONFIG] 加载配置文件: {config_path}")
    except Exception as e:
        print(f"[CONFIG] 读取配置文件失败: {e}")
        return

    # ---- 基本参数 ----
    fs = cfg.get("fs", fs)
    USE_MAG = cfg.get("USE_MAG", USE_MAG)
    coordinateType = cfg.get("coordinateType", coordinateType)
    if cfg.get("WORK_MODE") is not None:
        WORK_MODE = cfg["WORK_MODE"]

    # ---- 校准参数 ----
    cal = cfg.get("calibration", {})
    if cal.get("faces"):
        CALIB_FACES = cal["faces"]
    if cal.get("frames_per_frame"):
        CALIB_FRAMES_PER_FACE = cal["frames_per_face"]
    if cal.get("valid_range_ratio"):
        CALIB_VALID_RANGE_RATIO = tuple(cal["valid_range_ratio"])
    if cal.get("tpose_target_samples"):
        _tpose_target_samples = cal["tpose_target_samples"]
    if cal.get("tpose_use_last_n"):
        _tpose_use_last_n = cal["tpose_use_last_n"]

    # ---- 安装姿态 ----
    if cfg.get("install_pose_deg"):
        INSTALL_POSE_DEG.clear()
        INSTALL_POSE_DEG.update(cfg["install_pose_deg"])
        INSTALL_POSE_Q.clear()
        for alias, (rx_deg, ry_deg, rz_deg) in INSTALL_POSE_DEG.items():
            rx = np.radians(rx_deg)
            ry = np.radians(ry_deg)
            rz = np.radians(rz_deg)
            from core.quaternion import euler_to_quat as _e2q
            INSTALL_POSE_Q[alias] = _e2q(rx, ry, rz)

    # ---- 传感器管理节点 ----
    if cfg.get("mount_aliases"):
        global manager
        from core.quaternion import euler_to_quat as _e2q
        manager = QuaternionManager(fs=fs)
        for alias in cfg["mount_aliases"]:
            manager.add_node(alias)
        print(f"[CONFIG] 注册传感器节点: {cfg['mount_aliases']}")
