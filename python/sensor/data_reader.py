import json
import os
import warnings
import numpy as np
import pandas as pd
import matplotlib
from matplotlib import pyplot as plt

from core import config
from typing import Dict, Tuple, List, Optional

matplotlib.use('Agg')  # 非交互式后端，用于无GUI环境

from core.math_utils import moving_average as _moving_average

def fix_path(path):
    return path.replace("\\", "/")


# ================================================================
# 列名别名映射 — 将 csv_receiver.py (ESP32) 与旧格式统一为规范名
# ================================================================
_COLUMN_ALIAS_MAP = {
    # Accelerometer: 小写 → 规范大写
    'acc_x': 'Acc_x', 'acc_y': 'Acc_y', 'acc_z': 'Acc_z',
    # Gyroscope: 小写 / 缩写 → 规范大写
    'gyro_x': 'Gyr_x', 'gyro_y': 'Gyr_y', 'gyro_z': 'Gyr_z',
    'gyr_x': 'Gyr_x', 'gyr_y': 'Gyr_y', 'gyr_z': 'Gyr_z',
    # Magnetometer: csv_receiver 用 mag_x, 旧格式用 Geo_x
    'mag_x': 'Geo_x', 'mag_y': 'Geo_y', 'mag_z': 'Geo_z',
    'geo_x': 'Geo_x', 'geo_y': 'Geo_y', 'geo_z': 'Geo_z',
    # Angle: csv_receiver 中的角度列
    'angle_x': 'Angle_x', 'angle_y': 'Angle_y', 'angle_z': 'Angle_z',
    # Timestamp 兼容
    'timestamp': 'TimesTamp', 'timestamptimes': 'TimesTamp',
    # Yaw / Heading
    'yaw': 'Yaw', 'heading': 'Yaw',
}


def _normalize_columns(df: pd.DataFrame) -> pd.DataFrame:
    """
    将 DataFrame 的列名统一为规范大写格式。
    兼容:
      - csv_receiver.py 格式: acc_x, gyro_x, mag_x, angle_x ...
      - 旧数据格式:        Acc_x, Gyr_x, Geo_x ...
    """
    rename = {}
    for col in df.columns:
        key = col.strip().lower()
        if key in _COLUMN_ALIAS_MAP:
            canonical = _COLUMN_ALIAS_MAP[key]
            if col != canonical:
                rename[col] = canonical
    if rename:
        df = df.rename(columns=rename)
    return df


# ================================================================
# 设备 ID → 传感器别名映射（可选）
# 用户可按需在此处填写: {"<device_id>": "<alias>"}
# 示例: {"1A240BE878EC": "L4", "245BCDCCD7CB": "L2", ...}
# ================================================================
DEVICE_ID_TO_ALIAS: Dict[str, str] = {}


def _load_device_alias_map(map_path: Optional[str] = None) -> Dict[str, str]:
    """
    从 JSON 文件加载设备 ID → 传感器别名映射。
    JSON 格式: {"<device_id>": "<alias>", ...}
    """
    if map_path is None:
        return {}
    try:
        with open(map_path, "r", encoding="utf-8") as f:
            mapping = json.load(f)
        if isinstance(mapping, dict):
            return {str(k).strip(): str(v).strip() for k, v in mapping.items()}
    except FileNotFoundError:
        pass  # mapping file not existing is normal
    except Exception as e:
        warnings.warn(f"Failed to load device map: {map_path} - {e}")
    return {}


def _extract_sensor_alias(
    filename: str,
    df: Optional[pd.DataFrame] = None,
    device_map: Optional[Dict[str, str]] = None,
) -> str:
    """
    从文件名或 CSV 内容中提取传感器别名。
    优先级:
      1. 文件名后缀 (如 _L4, _S1)
      2. CSV 中的 device_id 列 → 映射表
      3. CSV 中的 device_id 列原值 (fallback)
      4. 文件名本体 (fallback)

    参数:
        filename: CSV 文件名 (不含路径)
        df: 已读取的 DataFrame (可选)
        device_map: device_id → alias 映射字典
    """
    VALID_SUFFIXES = {"H", "T1", "T12",
                      "L1", "L2", "L3", "L4", "L5", "L6",
                      "R1", "R2", "R3", "R4", "R5", "R6",
                      "S1"}

    base = os.path.splitext(filename)[0]
    suffix = base.split("_")[-1].strip()

    # 1) 文件名后缀直接命中
    if suffix in VALID_SUFFIXES:
        return suffix

    # 2) 尝试从 CSV 的 device_id 列获取
    if df is not None and 'device_id' in df.columns:
        device_id = str(df['device_id'].iloc[0]).strip()
        if device_id:
            # 2a) 映射表
            if device_map and device_id in device_map:
                return device_map[device_id]
            # 2b) 全局映射
            if DEVICE_ID_TO_ALIAS and device_id in DEVICE_ID_TO_ALIAS:
                return DEVICE_ID_TO_ALIAS[device_id]
            # 2c) 直接用 device_id
            return device_id

    # 3) fallback: 文件名本体
    return base


# ===== 全局工作模式：'upper_body' / 'lower_body' / 'full_body' =====
# WORK_MODE = "lower_body"   # 默认你可以按需要改成 "full_body" 或 "upper_body"


def Load_rawdata_csv(root_dir, columns):
    """
    从给定目录读取"时间最新"的 CSV 文件，并根据全局 WORK_MODE
    (upper_body / lower_body / full_body) 决定返回哪些传感器的数据。

    传感器别名约定：
        上肢(9个)：H, T1, T12, L1, L2, L3, R1, R2, R3
        下肢(7个)：S1, L4, L5, L6, R4, R5, R6

    参数:
        root_dir (str): 存放 CSV 文件的目录路径
        columns (list): 要读取的列名

    返回:
        - WORK_MODE == "upper_body" 时:
            (H, T1, T12, L1, L2, L3, R1, R2, R3)
        - WORK_MODE == "lower_body" 时:
            (S1, L4, L5, L6, R4, R5, R6)
        - WORK_MODE == "full_body" 时:
            (H, T1, T12, L1, L2, L3, R1, R2, R3,
             S1, L4, L5, L6, R4, R5, R6)

        若某标签没读到，对应位置为 None。
    """

    # 定义四种模式各自需要的别名顺序
    UPPER_ALIASES = ["H", "T1", "T12", "L1", "L2", "L3", "R1", "R2", "R3"]
    LOWER_ALIASES = ["S1", "L4", "L5", "L6", "R4", "R5", "R6"]
    FEET_ONLY_ALIASES = ["L6", "R6"]
    FULL_ALIASES  = UPPER_ALIASES + LOWER_ALIASES

    # 根据工作模式选择当前需要的别名列表
    mode = (config.WORK_MODE or "").lower()
    if mode == "upper_body":
        alias_list = UPPER_ALIASES
    elif mode == "lower_body":
        alias_list = LOWER_ALIASES
    elif mode == "feet_only":
        alias_list = FEET_ONLY_ALIASES
    elif mode == "full_body":
        alias_list = FULL_ALIASES
    else:
        warnings.warn(
            f"未知 WORK_MODE='{config.WORK_MODE}'，已回退为 'full_body'。",
            UserWarning
        )
        alias_list = FULL_ALIASES
        mode = "full_body"

    # —— 找出所有 CSV 文件，并按"文件修改时间"从新到旧排序 ——
    VALID_SUFFIXES = {"H", "T1", "T12",
                      "L1", "L2", "L3", "L4", "L5", "L6",
                      "R1", "R2", "R3", "R4", "R5", "R6",
                      "S1"}

    # 收集所有 CSV 文件，同时记录是否通过后缀识别
    all_csv_files = []  # (filename, known_suffix_or_None)
    for f in os.listdir(root_dir):
        if not f.lower().endswith(".csv"):
            continue
        base = os.path.splitext(f)[0]
        suffix = base.split("_")[-1].strip()
        if suffix in VALID_SUFFIXES:
            all_csv_files.append((f, suffix))
        else:
            # csv_receiver.py 格式: 文件名不含传感器后缀 (如 WT_1A240BE878EC.csv)
            # 仍然接受，稍后从 CSV 内容解析别名
            all_csv_files.append((f, None))

    # 按修改时间从新到旧排序
    all_csv_files.sort(
        key=lambda item: os.path.getmtime(os.path.join(root_dir, item[0])),
        reverse=True,
    )

    if len(all_csv_files) < len(alias_list):
        warnings.warn(
            f"目录下的 CSV 文件数量({len(all_csv_files)})少于当前模式 "
            f"'{mode}' 所需的传感器数量({len(alias_list)})，可能有缺失。",
            UserWarning
        )

    # 尝试加载 device_id → alias 映射表（若存在）
    map_path = os.path.join(root_dir, "device_alias_map.json")
    device_map = _load_device_alias_map(map_path)

    arrays = {}
    # 按当前模式所需传感器数量，取最新的对应个文件
    for file, known_suffix in all_csv_files[:len(alias_list)]:
        file_path = os.path.join(root_dir, file)

        # —— 读取 CSV 并规范化列名 ——
        try:
            df = pd.read_csv(file_path)
        except Exception as e:
            warnings.warn(f"读取 CSV 失败: {file_path} — {e}")
            continue

        df = _normalize_columns(df)

        # 选取请求的列（仅取存在的列，缺失列用 0 填充以保持列索引对齐）
        available_cols = [c for c in columns if c in df.columns]
        missing_cols = [c for c in columns if c not in df.columns]
        if missing_cols:
            warnings.warn(
                f"文件 {file} 缺少列: {missing_cols}，"
                f"将用 0 填充。可用列: {list(df.columns)}"
            )

        if not available_cols:
            warnings.warn(f"文件 {file} 没有可用的传感器数据列，跳过。")
            continue

        # 构建完整数据矩阵: N × len(columns)，缺失列填 0
        N = len(df)
        data = np.zeros((N, len(columns)), dtype=float)
        for i, col in enumerate(columns):
            if col in df.columns:
                data[:, i] = df[col].to_numpy(dtype=float)

        # —— 提取传感器别名 ——
        if known_suffix:
            suffix = known_suffix
        else:
            suffix = _extract_sensor_alias(file, df, device_map)

        arrays[suffix] = data
        # print(f"[READ] {file} -> {suffix}, shape={data.shape}")

    # 按当前模式 alias_list 的既定顺序返回，对应标签不存在则为 None
    result = tuple(arrays.get(alias) for alias in alias_list)
    return result

# def load_calib_params(root_dir):
#     """
#     读取陀螺仪和磁力计的校准参数
#
#     参数:
#         root_dir (str): 存放 JSON 文件的目录
#
#     返回:
#         gyro_biases (dict): {sensor_name: np.array([bx,by,bz])}
#         mag_params (dict): {sensor_name: {"bias": np.array([bx,by,bz]), "scale": np.array(3x3)}}
#     """
#     gyro_biases = {}
#     mag_biases = {}
#     mag_scales = {}
#
#     for file in os.listdir(root_dir):
#         if not file.endswith(".json"):
#             continue
#         file_path = os.path.join(root_dir, file)
#
#         with open(file_path, "r") as f:
#             data = json.load(f)
#
#         # 陀螺仪零偏
#         if "gyro_bias" in file:
#             name = file.split("_")[0]  # 比如 C1, D7, D8, F6
#             gyro_biases[name] = np.array(data["bias"], dtype=float)
#
#         # 磁力计椭球拟合
#         elif "mag_calib_params" in file:
#             name = file.split("_")[0]
#             mag_biases[name] = np.array(data["bias"], dtype=float)
#             mag_scales[name] = np.array(data["A"], dtype=float)
#
#     return gyro_biases, mag_biases, mag_scales


def load_calib_params(root_dir: str
                     ) -> Tuple[Dict[str, np.ndarray],
                                Dict[str, np.ndarray],
                                Dict[str, np.ndarray]]:
    """
    读取陀螺仪零偏与六面加速度计 bias/scale 校准参数（不含磁力计）。

    JSON 文件命名格式：
        <alias>_gyro_bias.json
        <alias>_acc_6face.json

    字段兼容：
        gyro_bias.json:  {"bias": [bx, by, bz]}
        acc_6face.json:  {"bias_g": [bx,by,bz], "scale_g": [sx,sy,sz]}
                         或 {"bias": [bx,by,bz], "scale": [sx,sy,sz]}

    参数:
        root_dir (str): JSON 文件所在目录
        verbose (bool): 是否打印加载日志

    返回:
        gyro_biases (dict): {alias: np.array([bx,by,bz], float)}
        acc_biases  (dict): {alias: np.array([bx,by,bz], float)}   # 单位 g
        acc_scales  (dict): {alias: np.array([sx,sy,sz], float)}   # 无量纲 scale (g-per-g)
    """
    gyro_biases: Dict[str, np.ndarray] = {}
    acc_biases: Dict[str, np.ndarray] = {}
    acc_scales: Dict[str, np.ndarray] = {}

    if root_dir is None:
        root_dir = "./calib_json"

    if not os.path.isdir(root_dir):
        return gyro_biases, acc_biases, acc_scales

    aliases_loaded = set()

    for fname in os.listdir(root_dir):
        fpath = os.path.join(root_dir, fname)
        if (not os.path.isfile(fpath)) or (not fname.endswith(".json")):
            continue

        # ---------- 1) 陀螺仪零偏 ----------
        if fname.endswith("_gyro_bias.json"):
            alias = fname[:-len("_gyro_bias.json")]
            try:
                with open(fpath, "r", encoding="utf-8") as f:
                    data = json.load(f)

                bias = data.get("bias", None)
                if isinstance(bias, (list, tuple)) and len(bias) == 3:
                    gyro_biases[alias] = np.array([float(x) for x in bias], dtype=float)
                    aliases_loaded.add(alias)

            except Exception:
                pass

        # ---------- 2) 六面加速度计 bias/scale ----------
        elif fname.endswith("_acc_6face.json"):
            alias = fname[:-len("_acc_6face.json")]
            try:
                with open(fpath, "r", encoding="utf-8") as f:
                    data = json.load(f)

                # 兼容字段名：bias_g/scale_g 或 bias/scale
                bias_g = data.get("bias_g", data.get("bias", None))
                scale_g = data.get("scale_g", data.get("scale", None))

                ok = (
                    isinstance(bias_g, (list, tuple)) and len(bias_g) == 3 and
                    isinstance(scale_g, (list, tuple)) and len(scale_g) == 3
                )
                if ok:
                    acc_biases[alias] = np.array([float(x) for x in bias_g], dtype=float)
                    acc_scales[alias] = np.array([float(x) for x in scale_g], dtype=float)
                    aliases_loaded.add(alias)

            except Exception:
                pass

    print("[CALIB-LOAD] 校准完成")

    return gyro_biases, acc_biases, acc_scales

def _infer_sensor_name_from_filename(filename: str):
    """
    从文件名推断传感器名称（取最后一个下划线后的部分）。
    例如:
        '2025_10_22_ROM_EVA_ED14DB798F6_L6.csv' → 'L6'
        'test_S1.csv' → 'S1'
        'R5.csv' → 'R5'
    """
    base = os.path.basename(filename)
    stem = os.path.splitext(base)[0]
    if "_" in stem:
        return stem.split("_")[-1]   #  取最后一个下划线后的部分
    return stem


def load_calibrated_filtered_arrays(window_size: int = 5,
                                    columns: list = None,aliases=None,):
    """
    调用 Load_rawdata_csv() 读取原始 CSV → 校准 → 滤波
    默认目录:
      raw_root   = <项目根>/Data/7
      calib_root = <项目根>/Data/calibrate_data/calib_json
    返回:
      arr_S1, arr_R6, arr_L6
    """
    # === 自动定位到项目根目录 ===
    cur_dir = os.path.dirname(os.path.abspath(__file__))   # 当前 data_read.py 所在目录
    project_root = os.path.dirname(cur_dir)                # 上一级目录，即 <项目根>
    # third_level_dir = os.path.dirname(os.path.dirname(os.path.dirname(cur_dir)))
    # second_level_dir = os.path.dirname(os.path.dirname(cur_dir))

    # data_root = os.path.join(second_level_dir,"python","Data","5.25——8")
    # D:\exercise\Sunny_Event\Data\data\2

    data_root = fix_path(config.originalDir)
    # --- 数据和标定目录 ---
    # data_root = os.path.join(project_root, "data")
    # print(data_root)

    # raw_root = os.path.join(third_level_dir, "全身关节项目调试工具", "bin", "Debug", "data")
    # calib_root = os.path.join(project_root,"calib_json")
    calib_root = fix_path(config.tempDir)


    # --- 默认列 ---
    if columns is None:
        columns = [
            "Acc_x", "Acc_y", "Acc_z",
            "Gyr_x", "Gyr_y", "Gyr_z",
            "Geo_x", "Geo_y", "Geo_z"
        ]

    mode =  config.WORK_MODE


    # === 根据 WORK_MODE 读取原始 CSV ===
    if mode == "upper_body":
        (
            arr_H_raw,
            arr_T1_raw,
            arr_T12_raw,
            arr_L1_raw,
            arr_L2_raw,
            arr_L3_raw,
            arr_R1_raw,
            arr_R2_raw,
            arr_R3_raw,
        ) = Load_rawdata_csv(data_root, columns)

        raw_arrays = {
            "H": arr_H_raw,
            "T1": arr_T1_raw,
            "T12": arr_T12_raw,
            "L1": arr_L1_raw,
            "L2": arr_L2_raw,
            "L3": arr_L3_raw,
            "R1": arr_R1_raw,
            "R2": arr_R2_raw,
            "R3": arr_R3_raw,
        }

    elif mode == "lower_body":
        (
            arr_S1_raw,
            arr_L4_raw,
            arr_L5_raw,
            arr_L6_raw,
            arr_R4_raw,
            arr_R5_raw,
            arr_R6_raw,
        ) = Load_rawdata_csv(data_root, columns)

        raw_arrays = {
            "S1": arr_S1_raw,
            "L4": arr_L4_raw,
            "L5": arr_L5_raw,
            "L6": arr_L6_raw,
            "R4": arr_R4_raw,
            "R5": arr_R5_raw,
            "R6": arr_R6_raw,
        }

    elif mode == "feet_only":
        # 简易双足模式: 仅左脚 L6 + 右脚 R6，无腰部传感器
        (
            arr_L6_raw,
            arr_R6_raw,
        ) = Load_rawdata_csv(data_root, columns)

        raw_arrays = {
            "L6": arr_L6_raw,
            "R6": arr_R6_raw,
        }

    else:
        # 其他情况一律按 full_body 处理
        (
            arr_H_raw,
            arr_T1_raw,
            arr_T12_raw,
            arr_L1_raw,
            arr_L2_raw,
            arr_L3_raw,
            arr_R1_raw,
            arr_R2_raw,
            arr_R3_raw,
            arr_S1_raw,
            arr_L4_raw,
            arr_L5_raw,
            arr_L6_raw,
            arr_R4_raw,
            arr_R5_raw,
            arr_R6_raw,
        ) = Load_rawdata_csv(data_root, columns)

        raw_arrays = {
            "H": arr_H_raw,
            "T1": arr_T1_raw,
            "T12": arr_T12_raw,
            "L1": arr_L1_raw,
            "L2": arr_L2_raw,
            "L3": arr_L3_raw,
            "R1": arr_R1_raw,
            "R2": arr_R2_raw,
            "R3": arr_R3_raw,
            "S1": arr_S1_raw,
            "L4": arr_L4_raw,
            "L5": arr_L5_raw,
            "L6": arr_L6_raw,
            "R4": arr_R4_raw,
            "R5": arr_R5_raw,
            "R6": arr_R6_raw,
        }

    # === 把全部需要的节点都纳入待处理字典（自动过滤 None） ===
    raw_data_dict = {
        alias: arr
        for alias, arr in raw_arrays.items()
        if arr is not None
    }
    # === 读取标定参数（兼容旧接口/新接口） ===
    calib = load_calib_params(calib_root)

    gyro_biases = {}
    acc_biases = {}
    acc_scales = {}
    mag_biases = {}
    mag_scales = {}

    if not (isinstance(calib, (list, tuple)) and len(calib) in (3, 4)):
        raise ValueError(
            f"load_calib_params 返回格式异常：type={type(calib)}, len={getattr(calib, '__len__', lambda: 'NA')()}")

    if len(calib) == 4:
        # 你历史里的"4项版本"（如果还存在）：gyro, mag_bias, mag_scale, acc_bias
        gyro_biases, mag_biases, mag_scales, acc_biases = calib
        acc_scales = {}
    else:
        # len == 3：可能是旧版(gyro, mag_bias, mag_scale) 或 新版(gyro, acc_bias, acc_scale)
        a, b, c = calib

        # 用第三项的"值形状"判断是 3x3（磁力计矩阵）还是 3（加速度scale向量）
        is_mag_style = False
        if isinstance(c, dict) and len(c) > 0:
            v = np.asarray(next(iter(c.values())))
            is_mag_style = (v.ndim == 2 and v.shape == (3, 3))

        if is_mag_style:
            # 旧版：gyro + mag
            gyro_biases, mag_biases, mag_scales = a, b, c
            acc_biases, acc_scales = {}, {}
        else:
            # 新版：gyro + acc
            gyro_biases, acc_biases, acc_scales = a, b, c
            mag_biases, mag_scales = {}, {}

    calibrated_data = {}

    # === 列名与常量（支持两套命名） ===
    col_idx = {c: i for i, c in enumerate(columns)}
    gyr_cols = ["Gyr_x", "Gyr_y", "Gyr_z"]
    acc_cols = ["Acc_x", "Acc_y", "Acc_z"]

    # 只有当数据里确实存在磁力计三列时，才认为有 mag
    has_geo = all(c in col_idx for c in ["Geo_x", "Geo_y", "Geo_z"])
    has_mag = all(c in col_idx for c in ["Mag_x", "Mag_y", "Mag_z"])
    mag_cols = ["Geo_x", "Geo_y", "Geo_z"] if has_geo else (["Mag_x", "Mag_y", "Mag_z"] if has_mag else None)

    # 加速度单位与重力常数（按你的数据改：'g' 或 'm/s^2'）
    ACC_UNIT = "g"
    G_CONST = 9.8
    KEEP_G = 1.0 if ACC_UNIT.lower().startswith("g") else G_CONST

    # === 校准 + 滤波（对七个节点统一处理）===
    for sensor_name, data in raw_data_dict.items():
        if data is None:
            raise ValueError(f"未读取到 {sensor_name} 对应的数据，请检查 {data_root}")
        data = data.astype(float, copy=False)

        # 1) Gyro 零偏
        if data.ndim == 2 and all(c in col_idx for c in gyr_cols) and (sensor_name in gyro_biases):
            gidx = [col_idx[c] for c in gyr_cols]
            bias = np.asarray(gyro_biases[sensor_name], float).reshape(1, 3)
            data[:, gidx] -= bias

        # 2) Acc 六面校准：bias + scale（Z轴保留重力）
        if data.ndim == 2 and all(c in col_idx for c in acc_cols):
            aidx = [col_idx[c] for c in acc_cols]

            # 2.1 bias（存在才用）
            if sensor_name in acc_biases:
                bias = np.asarray(acc_biases[sensor_name], float).reshape(1, 3)
                # X/Y：直接减零偏
                data[:, aidx[0]] -= bias[:, 0]
                data[:, aidx[1]] -= bias[:, 1]
                # Z：减"零偏部分"，保留重力项（1g 或 9.80665）
                z_zero = bias[0, 2] - KEEP_G
                data[:, aidx[2]] -= z_zero

            # 2.2 scale（存在才用；六面标定通常给每轴缩放）
            if sensor_name in acc_scales:
                scale = np.asarray(acc_scales[sensor_name], float).reshape(1, 3)
                # 若你的 scale 定义是"乘法因子"，用 *=
                data[:, aidx[0]] *= scale[:, 0]
                data[:, aidx[1]] *= scale[:, 1]
                data[:, aidx[2]] *= scale[:, 2]
                # 如果你确认 scale 是"除法因子"（很多标定是 raw/scale），把上面三行改成 /=

        # 3) Mag 软/硬铁（现在默认可选：只有列存在且参数齐才做）
        if (mag_cols is not None) and all(c in col_idx for c in mag_cols):
            if sensor_name in mag_biases and sensor_name in mag_scales:
                midx = [col_idx[c] for c in mag_cols]
                m_raw = data[:, midx]
                m_corr = m_raw - np.asarray(mag_biases[sensor_name], float).reshape(1, 3)
                A = np.asarray(mag_scales[sensor_name], float)  # 3x3
                data[:, midx] = (A @ m_corr.T).T
            # else: 缺参数就跳过（不报错）

        # 4) 平滑滤波
        data = _moving_average(data, window_size)

        calibrated_data[sensor_name] = data
        # print(f"{tag} done, shape={data.shape}")

    # === 自动对齐长度（只考虑真正要用的节点） ===
    if aliases is None:
        # 按 WORK_MODE 决定默认返回哪些节点
        UPPER_ALIASES = ["H", "T1", "T12", "L1", "L2", "L3", "R1", "R2", "R3"]
        LOWER_ALIASES = ["S1", "L4", "L5", "L6", "R4", "R5", "R6"]
        FEET_ONLY_ALIASES = ["L6", "R6"]
        FULL_ALIASES = UPPER_ALIASES + LOWER_ALIASES

        mode = config.WORK_MODE
        if mode == "upper_body":
            alias_list = UPPER_ALIASES
        elif mode == "lower_body":
            alias_list = LOWER_ALIASES
        elif mode == "feet_only":
            alias_list = FEET_ONLY_ALIASES
        else:
            alias_list = FULL_ALIASES
    else:
        # ★ 用户显式指定想要的节点顺序
        alias_list = list(aliases)

    # 只用 alias_list 中且确实存在的节点来算最小长度
    valid_arrays = [
        calibrated_data[a]
        for a in alias_list
        if a in calibrated_data and calibrated_data[a] is not None and calibrated_data[a].shape[0] > 0
    ]
    if not valid_arrays:
        raise ValueError("在 calibrated_data 中没有找到任何有效的节点数据，用于对齐长度。")

    Nmin = min(arr.shape[0] for arr in valid_arrays)

    # === 按 alias_list 顺序从 calibrated_data 中取出并截断 ===
    result = []
    for alias in alias_list:
        arr = calibrated_data.get(alias)
        if arr is not None:
            result.append(arr[:Nmin])
        else:
            # 如果某个 alias 不存在，就返回 None（也可以改成抛异常，看你需要）
            result.append(None)

    return tuple(result)

