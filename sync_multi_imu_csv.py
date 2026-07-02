#!/usr/bin/env python3
"""
sync_multi_imu_csv.py —— 多 WT901WIFI 传感器 CSV 后处理同步脚本

功能:
  1. 读取文件夹中多个单传感器原始 CSV（每个 CSV 来自一个 WT901WIFI）
  2. 每个传感器内部清理：按 esp32_rx_ms 排序、删除重复时间戳
  3. 检查 sensor_ms 是否有跳变或丢帧，打印诊断信息
  4. 以 esp32_rx_ms 为统一时间基准，找到所有传感器共同重叠的时间范围
  5. 根据指定采样率（默认 100 Hz）生成统一时间轴
  6. 对 acc/gyro/angle/mag 分别线性插值到统一时间轴
  7. 输出同步后长表 CSV（synced_long.csv）
  8. 输出同步后宽表 CSV（synced_wide.csv）
  9. 输出同步报告（sync_report.txt）
 10. 原始 CSV 文件完整保留，不做任何修改

设计原则:
  - esp32_rx_ms 用于多传感器统一时间轴对齐
  - 姿态解算 dt 优先使用固定采样率 dt = 1/fs，不依赖 esp32_rx_ms 相邻差值
  - sensor_ms 用于检查单个 WT901 自身采样是否稳定、是否存在丢帧或时间跳变
  - 所有后处理在 Python 侧完成，ESP32 端不做插值同步

用法:
  python sync_multi_imu_csv.py --input_dir raw_csv --output_dir synced_output --fs 100

  python sync_multi_imu_csv.py --input_dir ./data/wt901_data_20260630_105114 --fs 50

选项:
  --input_dir DIR       包含单传感器 CSV 的文件夹路径（必需）
  --output_dir DIR      输出目录（默认: input_dir 同级的 synced_output/）
  --fs FLOAT            目标重采样频率 Hz（默认: 100）
  --no-wide             不生成宽表格式 CSV
  --no-long             不生成长表格式 CSV
  --csv-sep CHAR        CSV 列分隔符（默认: 逗号）
"""

import argparse
import csv
import os
import sys
from datetime import datetime
from pathlib import Path

import numpy as np

# Try to import scipy for interpolation; fall back to numpy-only if unavailable
try:
    from scipy.interpolate import interp1d
    _HAS_SCIPY = True
except ImportError:
    _HAS_SCIPY = False


# ============================================================================
# 标准列名定义
# ============================================================================

# 标准化后的列名（全部小写，供内部使用）
# 与 ESP32 main.cpp outputCSVRow() 输出的 16 列顺序一致
# device_id,sensor_timestamp,sensor_ms,esp32_rx_ms,
#   acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,angle_x,angle_y,angle_z,mag_x,mag_y,mag_z
STD_COLUMN_ORDER = [
    "device_id",
    "sensor_timestamp",
    "sensor_ms",
    "esp32_rx_ms",
    "acc_x", "acc_y", "acc_z",
    "gyro_x", "gyro_y", "gyro_z",
    "angle_x", "angle_y", "angle_z",
    "mag_x", "mag_y", "mag_z",
]

# 需要插值的数值列（不含 device_id、sensor_timestamp 等字符串/标识列）
VALUE_COLUMNS = [
    "acc_x", "acc_y", "acc_z",
    "gyro_x", "gyro_y", "gyro_z",
    "angle_x", "angle_y", "angle_z",
    "mag_x", "mag_y", "mag_z",
]

# ESP32 接收时间戳列名（统一时间基准）
TIME_COL = "esp32_rx_ms"

# 传感器自身时间戳列名（用于丢帧/跳变检测）
SENSOR_TIME_COL = "sensor_ms"


# ============================================================================
# CSV 头部映射 —— 兼容不同 ESP32 固件版本输出的列名
# ============================================================================

# 旧版固件（14 列头部）：device_id,timestamp,Acc_x,Acc_y,Acc_z,
#   Gyr_x,Gyr_y,Gyr_z,Angle_x,Angle_y,Angle_z,Geo_x,Geo_y,Geo_z
# 注意：旧版头部缺少 sensor_ms 和 esp32_rx_ms 列名，但数据仍包含 16 列
#
# 新版固件（16 列头部）：device_id,sensor_timestamp,sensor_ms,esp32_rx_ms,
#   Acc_x,Acc_y,Acc_z,Gyr_x,Gyr_y,Gyr_z,Angle_x,Angle_y,Angle_z,Geo_x,Geo_y,Geo_z

# 从原始列名到标准列名的映射表
HEADER_NAME_MAP = {
    # 设备 ID
    "device_id":     "device_id",
    # 传感器时间戳
    "timestamp":             "sensor_timestamp",
    "sensor_timestamp":      "sensor_timestamp",
    # 传感器本地时间（ms of day）
    "sensor_ms":             "sensor_ms",
    # ESP32 接收时间（millis）
    "esp32_rx_ms":           "esp32_rx_ms",
    # 加速度 —— 支持大小写变体
    "acc_x": "acc_x", "acc_y": "acc_y", "acc_z": "acc_z",
    "Acc_x": "acc_x", "Acc_y": "acc_y", "Acc_z": "acc_z",
    "ACC_X": "acc_x", "ACC_Y": "acc_y", "ACC_Z": "acc_z",
    # 陀螺仪 —— 支持 Gyr / gyro 两种习惯
    "gyr_x":  "gyro_x", "gyr_y":  "gyro_y", "gyr_z":  "gyro_z",
    "Gyr_x":  "gyro_x", "Gyr_y":  "gyro_y", "Gyr_z":  "gyro_z",
    "gyro_x": "gyro_x", "gyro_y": "gyro_y", "gyro_z": "gyro_z",
    "GYRO_X": "gyro_x", "GYRO_Y": "gyro_y", "GYRO_Z": "gyro_z",
    # 姿态角
    "angle_x": "angle_x", "angle_y": "angle_y", "angle_z": "angle_z",
    "Angle_x": "angle_x", "Angle_y": "angle_y", "Angle_z": "angle_z",
    "ANGLE_X": "angle_x", "ANGLE_Y": "angle_y", "ANGLE_Z": "angle_z",
    # 磁力计 —— Geo 是 WT901 协议原始字段名
    "geo_x":  "mag_x", "geo_y":  "mag_y", "geo_z":  "mag_z",
    "Geo_x":  "mag_x", "Geo_y":  "mag_y", "Geo_z":  "mag_z",
    "mag_x":  "mag_x", "mag_y":  "mag_y", "mag_z":  "mag_z",
    "Mag_x":  "mag_x", "Mag_y":  "mag_y", "Mag_z":  "mag_z",
    "MAG_X":  "mag_x", "MAG_Y":  "mag_y", "MAG_Z":  "mag_z",
}


def _parse_timestamp_to_ms(timestamp_str: str) -> float:
    """
    将传感器时间戳字符串解析为当日毫秒数（用于 Format B 缺失 sensor_ms 时）。

    支持格式:
      - "YYYY-MM-DD HH:MM:SS.mmm" (如 "2000-00-00 00:03:58.973")
      - "20YY-MM-DD HH:MM:SS.mmm" (ESP32 生成的格式)
    返回: 从当日 00:00:00.000 开始的毫秒数，解析失败返回 NaN。
    """
    try:
        # 提取时间部分（HH:MM:SS.mmm）
        parts = timestamp_str.strip().split(" ")
        if len(parts) >= 2:
            time_part = parts[-1]  # 取最后一部分
        else:
            time_part = parts[0]

        time_components = time_part.split(":")
        if len(time_components) == 3:
            hour = int(time_components[0])
            minute = int(time_components[1])
            sec_ms = time_components[2].split(".")
            second = int(sec_ms[0])
            millisecond = int(sec_ms[1]) if len(sec_ms) > 1 else 0
            return float(hour * 3600000 + minute * 60000 + second * 1000 + millisecond)
    except (ValueError, IndexError):
        pass
    return float("nan")


# ============================================================================
# CSV 格式检测
# ============================================================================

def _detect_csv_format(filepath: str) -> int:
    """
    检测 CSV 数据列数（不含头部）。

    读取文件头部若干行，返回数据列数：
      - 16: 包含 sensor_ms + esp32_rx_ms（完整格式）
      - 14: 不含 sensor_ms / esp32_rx_ms（精简格式，仅有 acc/gyro/angle/mag）

    返回检测到的数据列数，若无法判定则返回 16（按完整格式处理）。
    """
    with open(filepath, "r", encoding="utf-8", errors="replace") as f:
        # 跳过头部
        f.readline()
        # 读取前 5 行数据
        for _ in range(5):
            line = f.readline().strip()
            if not line:
                continue
            if line.startswith("[DATA]"):
                line = line[len("[DATA]"):]
            parts = line.split(",")
            n = len(parts)
            if n == 16:
                return 16
            elif n == 14:
                return 14
    # 默认按 16 列处理（向后兼容）
    return 16


# ============================================================================
# CSV 读取与清理
# ============================================================================

def parse_single_csv(filepath: str) -> dict[str, np.ndarray]:
    """
    读取单个传感器的 CSV 文件，返回标准化后的数据字典。

    自动检测两种 CSV 格式：
      - Format A (16 列): device_id, timestamp, sensor_ms, esp32_rx_ms,
                          Acc_x, Acc_y, Acc_z, Gyr_x, Gyr_y, Gyr_z,
                          Angle_x, Angle_y, Angle_z, Geo_x, Geo_y, Geo_z
      - Format B (14 列): device_id, timestamp,
                          Acc_x, Acc_y, Acc_z, Gyr_x, Gyr_y, Gyr_z,
                          Angle_x, Angle_y, Angle_z, Geo_x, Geo_y, Geo_z
                         （无 sensor_ms / esp32_rx_ms，需通过 timestamp 估算 sensor_ms，
                           以数据行序号 × 采样间隔生成合成 esp32_rx_ms）

    参数:
        filepath: 单个传感器 CSV 文件路径

    返回:
        {
            "device_id":        str,
            "sensor_timestamp": np.ndarray (字符串),
            "sensor_ms":        np.ndarray (float),
            "esp32_rx_ms":      np.ndarray (float),
            "acc_x": np.ndarray (float), ..., "mag_z": np.ndarray (float),
        }
        数据按 esp32_rx_ms 升序排列，重复时间戳已去重。
    """
    raw_rows = []
    device_id_from_data = None

    # 检测 CSV 格式
    n_cols = _detect_csv_format(filepath)
    has_esp32_time = (n_cols == 16)
    if not has_esp32_time:
        print(f"    检测为 14 列精简格式（无 sensor_ms/esp32_rx_ms），"
              f"将使用 timestamp 估算 sensor_ms，按帧序号生成合成时间轴")

    with open(filepath, "r", encoding="utf-8", errors="replace") as f:
        # 跳过第一行（头部）
        f.readline()

        for line_num, line in enumerate(f, 2):
            line = line.strip()
            if not line:
                continue

            # 去掉可能的 [DATA] 前缀（Serial 模式下）
            if line.startswith("[DATA]"):
                line = line[len("[DATA]"):]

            parts = line.split(",")
            if len(parts) < n_cols:
                continue  # 跳过不完整的行

            try:
                dev_id = parts[0].strip()
                sensor_timestamp = parts[1].strip()

                if has_esp32_time:
                    # Format A: 16 列，位置 2=sensor_ms, 3=esp32_rx_ms
                    sensor_ms = int(parts[2])
                    esp32_rx_ms = int(parts[3])
                    acc_start = 4
                else:
                    # Format B: 14 列，无 sensor_ms/esp32_rx_ms
                    # 从 timestamp 估算 sensor_ms，esp32_rx_ms 稍后按帧序号生成
                    sensor_ms = _parse_timestamp_to_ms(sensor_timestamp)
                    esp32_rx_ms = 0  # 占位，稍后按行序号替换
                    acc_start = 2

                # 数值列：acc, gyro, angle, mag (各 3 列，共 12 列)
                acc  = [float(parts[acc_start + i]) for i in range(3)]
                gyro = [float(parts[acc_start + 3 + i]) for i in range(3)]
                angle = [float(parts[acc_start + 6 + i]) for i in range(3)]
                mag  = [float(parts[acc_start + 9 + i]) for i in range(3)]
            except (ValueError, IndexError) as e:
                print(f"  [警告] {os.path.basename(filepath)} 第 {line_num} 行解析失败: {e}")
                continue

            if device_id_from_data is None:
                device_id_from_data = dev_id
            elif device_id_from_data != dev_id:
                continue  # 跳过不一致的 device_id

            raw_rows.append([
                dev_id, sensor_timestamp, sensor_ms, esp32_rx_ms,
                acc[0], acc[1], acc[2],
                gyro[0], gyro[1], gyro[2],
                angle[0], angle[1], angle[2],
                mag[0], mag[1], mag[2],
            ])

    if not raw_rows:
        raise ValueError(f"未解析到有效数据行: {filepath}")

    # 转为 numpy 结构化数组
    rows_arr = np.array(raw_rows, dtype=object)

    # 提取各列
    sensor_timestamps = rows_arr[:, 1]
    sensor_ms_arr = rows_arr[:, 2].astype(np.float64)
    esp32_rx_ms_arr = rows_arr[:, 3].astype(np.float64)
    acc_x = rows_arr[:, 4].astype(np.float64)
    acc_y = rows_arr[:, 5].astype(np.float64)
    acc_z = rows_arr[:, 6].astype(np.float64)
    gyro_x = rows_arr[:, 7].astype(np.float64)
    gyro_y = rows_arr[:, 8].astype(np.float64)
    gyro_z = rows_arr[:, 9].astype(np.float64)
    angle_x = rows_arr[:, 10].astype(np.float64)
    angle_y = rows_arr[:, 11].astype(np.float64)
    angle_z = rows_arr[:, 12].astype(np.float64)
    mag_x = rows_arr[:, 13].astype(np.float64)
    mag_y = rows_arr[:, 14].astype(np.float64)
    mag_z = rows_arr[:, 15].astype(np.float64)

    # 对于 Format B（无 esp32_rx_ms），使用 sensor_ms 作为统一时间基准
    if not has_esp32_time:
        # 按 sensor_ms 排序，过滤无效值
        valid_mask = ~np.isnan(sensor_ms_arr)
        if not np.any(valid_mask):
            raise ValueError(f"无法从 timestamp 提取有效时间: {filepath}")
        sensor_ms_arr = sensor_ms_arr[valid_mask]
        sensor_timestamps = sensor_timestamps[valid_mask]
        acc_x = acc_x[valid_mask]; acc_y = acc_y[valid_mask]; acc_z = acc_z[valid_mask]
        gyro_x = gyro_x[valid_mask]; gyro_y = gyro_y[valid_mask]; gyro_z = gyro_z[valid_mask]
        angle_x = angle_x[valid_mask]; angle_y = angle_y[valid_mask]; angle_z = angle_z[valid_mask]
        mag_x = mag_x[valid_mask]; mag_y = mag_y[valid_mask]; mag_z = mag_z[valid_mask]

        # 按 sensor_ms 排序
        order = np.argsort(sensor_ms_arr)
        sensor_ms_arr = sensor_ms_arr[order]
        sensor_timestamps = sensor_timestamps[order]
        acc_x = acc_x[order]; acc_y = acc_y[order]; acc_z = acc_z[order]
        gyro_x = gyro_x[order]; gyro_y = gyro_y[order]; gyro_z = gyro_z[order]
        angle_x = angle_x[order]; angle_y = angle_y[order]; angle_z = angle_z[order]
        mag_x = mag_x[order]; mag_y = mag_y[order]; mag_z = mag_z[order]

        # 使用 sensor_ms 作为 esp32_rx_ms（多传感器同步时以此为基准）
        esp32_rx_ms_arr = sensor_ms_arr.copy()
        # 如果 sensor_ms 是绝对时间（毫秒数很大），减去偏移以得到较小数值
        # 便于后续处理和显示
        if sensor_ms_arr[0] > 1000000:
            offset = sensor_ms_arr[0]
            esp32_rx_ms_arr = sensor_ms_arr - offset

        # 去重（基于 esp32_rx_ms = sensor_ms）
        unique_rx, unique_idx = np.unique(esp32_rx_ms_arr, return_index=True)
        if len(unique_rx) < len(esp32_rx_ms_arr):
            dup_count = len(esp32_rx_ms_arr) - len(unique_rx)
            print(f"  [{device_id_from_data}] 检测到 {dup_count} 个重复时间戳，已去重")
            unique_idx.sort()
            esp32_rx_ms_arr = esp32_rx_ms_arr[unique_idx]
            sensor_ms_arr = sensor_ms_arr[unique_idx]
            sensor_timestamps = sensor_timestamps[unique_idx]
            acc_x = acc_x[unique_idx]; acc_y = acc_y[unique_idx]; acc_z = acc_z[unique_idx]
            gyro_x = gyro_x[unique_idx]; gyro_y = gyro_y[unique_idx]; gyro_z = gyro_z[unique_idx]
            angle_x = angle_x[unique_idx]; angle_y = angle_y[unique_idx]; angle_z = angle_z[unique_idx]
            mag_x = mag_x[unique_idx]; mag_y = mag_y[unique_idx]; mag_z = mag_z[unique_idx]
    else:
        # Format A: 按 esp32_rx_ms 排序
        order = np.argsort(esp32_rx_ms_arr)
        esp32_rx_ms_arr = esp32_rx_ms_arr[order]
        sensor_ms_arr = sensor_ms_arr[order]
        sensor_timestamps = sensor_timestamps[order]
        acc_x = acc_x[order]; acc_y = acc_y[order]; acc_z = acc_z[order]
        gyro_x = gyro_x[order]; gyro_y = gyro_y[order]; gyro_z = gyro_z[order]
        angle_x = angle_x[order]; angle_y = angle_y[order]; angle_z = angle_z[order]
        mag_x = mag_x[order]; mag_y = mag_y[order]; mag_z = mag_z[order]

        # 去重
        unique_rx, unique_idx = np.unique(esp32_rx_ms_arr, return_index=True)
        if len(unique_rx) < len(esp32_rx_ms_arr):
            dup_count = len(esp32_rx_ms_arr) - len(unique_rx)
            print(f"  [{device_id_from_data}] 检测到 {dup_count} 个重复 esp32_rx_ms，已去重")
            unique_idx.sort()
            esp32_rx_ms_arr = esp32_rx_ms_arr[unique_idx]
            sensor_ms_arr = sensor_ms_arr[unique_idx]
            sensor_timestamps = sensor_timestamps[unique_idx]
            acc_x = acc_x[unique_idx]; acc_y = acc_y[unique_idx]; acc_z = acc_z[unique_idx]
            gyro_x = gyro_x[unique_idx]; gyro_y = gyro_y[unique_idx]; gyro_z = gyro_z[unique_idx]
            angle_x = angle_x[unique_idx]; angle_y = angle_y[unique_idx]; angle_z = angle_z[unique_idx]
            mag_x = mag_x[unique_idx]; mag_y = mag_y[unique_idx]; mag_z = mag_z[unique_idx]

    return {
        "device_id": device_id_from_data,
        "sensor_timestamp": sensor_timestamps,
        "sensor_ms": sensor_ms_arr,
        "esp32_rx_ms": esp32_rx_ms_arr,
        "acc_x": acc_x, "acc_y": acc_y, "acc_z": acc_z,
        "gyro_x": gyro_x, "gyro_y": gyro_y, "gyro_z": gyro_z,
        "angle_x": angle_x, "angle_y": angle_y, "angle_z": angle_z,
        "mag_x": mag_x, "mag_y": mag_y, "mag_z": mag_z,
    }


# ============================================================================
# 传感器诊断
# ============================================================================

def diagnose_sensor(sensor_data: dict, target_fs: float) -> dict:
    """
    对单个传感器进行数据质量诊断。

    诊断内容:
    - 总帧数
    - 起止时间（sensor_ms 和 esp32_rx_ms）
    - sensor_ms 采样间隔统计（用于检查传感器自身稳定性）
    - esp32_rx_ms 采样间隔统计（用于检查 WiFi 接收稳定性）
    - 估计丢帧数量

    返回诊断报告字典，同时打印到控制台。
    """
    dev = sensor_data["device_id"]
    sms = sensor_data["sensor_ms"]
    rx = sensor_data["esp32_rx_ms"]
    n = len(rx)

    dt_expected_ms = 1000.0 / target_fs

    report = {"device_id": dev, "n_frames": n}

    # ---- sensor_ms 诊断 ----
    sms_diffs = np.diff(sms)
    # 过滤跨天回绕（差值 > 60000 ms 或 <= 0 视为无效）
    sms_valid = sms_diffs[(sms_diffs > 0) & (sms_diffs < 60000)]

    report["sms_mean_interval"] = float(sms_valid.mean()) if len(sms_valid) > 0 else None
    report["sms_median_interval"] = float(np.median(sms_valid)) if len(sms_valid) > 0 else None
    report["sms_std_interval"] = float(sms_valid.std()) if len(sms_valid) > 0 else None
    report["sms_min_interval"] = float(sms_valid.min()) if len(sms_valid) > 0 else None
    report["sms_max_interval"] = float(sms_valid.max()) if len(sms_valid) > 0 else None

    # sensor_ms 丢帧检测：间隔 > 预期间隔 * 2.5
    sms_gaps = sms_valid[sms_valid > dt_expected_ms * 2.5]
    report["sms_long_gaps"] = len(sms_gaps)
    report["sms_max_gap"] = float(sms_gaps.max()) if len(sms_gaps) > 0 else 0.0
    # 估计丢帧数（按预期间隔计算）
    if len(sms_valid) > 0 and sms_valid.mean() > 0:
        missed_frames = max(0, int(np.sum(sms_valid[sms_valid > dt_expected_ms * 2.5]) / dt_expected_ms - len(sms_gaps)))
        report["sms_estimated_missed"] = missed_frames
    else:
        report["sms_estimated_missed"] = 0

    # ---- esp32_rx_ms 诊断 ----
    rx_diffs = np.diff(rx)
    report["rx_mean_interval"] = float(rx_diffs.mean())
    report["rx_median_interval"] = float(np.median(rx_diffs))
    report["rx_std_interval"] = float(rx_diffs.std())
    report["rx_min_interval"] = float(rx_diffs.min())
    report["rx_max_interval"] = float(rx_diffs.max())
    report["rx_start_ms"] = float(rx[0])
    report["rx_end_ms"] = float(rx[-1])
    report["rx_duration_s"] = float((rx[-1] - rx[0]) / 1000.0)
    report["sms_start_ms"] = float(sms[0])
    report["sms_end_ms"] = float(sms[-1])
    report["effective_fs"] = float(1000.0 / rx_diffs.mean()) if rx_diffs.mean() > 0 else 0.0

    rx_gaps = rx_diffs[rx_diffs > dt_expected_ms * 3.0]
    report["rx_long_gaps"] = len(rx_gaps)
    report["rx_max_gap"] = float(rx_gaps.max()) if len(rx_gaps) > 0 else 0.0

    return report


def print_sensor_report(report: dict, target_fs: float) -> None:
    """打印单个传感器的诊断信息。"""
    dev = report["device_id"]
    dt_ms = 1000.0 / target_fs

    print(f"\n  --- [{dev}] {'-' * 45}")
    print(f"  | 总帧数:        {report['n_frames']}")
    print(f"  | esp32_rx_ms:   {report['rx_start_ms']:.0f} -> {report['rx_end_ms']:.0f} ms "
          f"(时长 {report['rx_duration_s']:.2f} s)")
    print(f"  | sensor_ms:     {report['sms_start_ms']:.0f} -> {report['sms_end_ms']:.0f} ms")

    # sensor_ms 间隔
    if report["sms_mean_interval"] is not None:
        print(f"  | sensor_ms 间隔: 均值={report['sms_mean_interval']:.1f} ms, "
              f"中位数={report['sms_median_interval']:.1f} ms, "
              f"std={report['sms_std_interval']:.1f} ms")
        print(f"  |                 最小={report['sms_min_interval']:.1f} ms, "
              f"最大={report['sms_max_interval']:.1f} ms")
        if report["sms_long_gaps"] > 0:
            print(f"  | [WARN] sensor_ms 大间隔 (>{dt_ms*2.5:.0f} ms): "
                  f"{report['sms_long_gaps']} 处, "
                  f"最大 {report['sms_max_gap']:.0f} ms, "
                  f"估计丢帧 ~{report['sms_estimated_missed']} 帧")
        else:
            print(f"  | [OK] sensor_ms 无明显大间隔")
    else:
        print(f"  | sensor_ms 间隔: 无有效数据（可能跨天回绕）")

    # esp32_rx_ms 间隔
    print(f"  | esp32_rx 间隔:  均值={report['rx_mean_interval']:.1f} ms, "
          f"中位数={report['rx_median_interval']:.1f} ms, "
          f"std={report['rx_std_interval']:.1f} ms")
    print(f"  |                 最小={report['rx_min_interval']:.1f} ms, "
          f"最大={report['rx_max_interval']:.1f} ms")
    if report["rx_long_gaps"] > 0:
        print(f"  | [WARN] esp32_rx 大间隔 (>{dt_ms*3:.0f} ms): "
              f"{report['rx_long_gaps']} 处, "
              f"最大 {report['rx_max_gap']:.0f} ms")
    else:
        print(f"  | [OK] esp32_rx 无明显大间隔")

    print(f"  | 等效采样率:    {report['effective_fs']:.1f} Hz (目标 {target_fs} Hz)")
    print(f"  ---{' ' * 50}")


# ============================================================================
# 多传感器同步
# ============================================================================

def find_common_time_range(sensors: dict[str, dict]) -> tuple[float, float]:
    """
    找到所有传感器共同重叠的 esp32_rx_ms 时间范围。

    sync_start = 所有传感器起点中的最大值
    sync_end   = 所有传感器终点中的最小值

    返回 (sync_start_ms, sync_end_ms)。
    """
    starts = []
    ends = []
    for dev, data in sensors.items():
        rx = data["esp32_rx_ms"]
        starts.append(rx[0])
        ends.append(rx[-1])

    t_start = max(starts)
    t_end = min(ends)

    if t_start >= t_end:
        raise ValueError(
            f"传感器时间范围无交集！\n" +
            "\n".join(f"  {dev}: {sensors[dev]['esp32_rx_ms'][0]:.0f} -> "
                      f"{sensors[dev]['esp32_rx_ms'][-1]:.0f} ms"
                      for dev in sorted(sensors.keys()))
        )

    return t_start, t_end


def generate_time_grid(t_start_ms: float, t_end_ms: float, fs: float) -> np.ndarray:
    """
    生成统一时间轴（以 ms 为单位）。

    时间网格从 t_start_ms 到 t_end_ms（含两端），步长 dt_ms = 1000/fs。
    """
    dt_ms = 1000.0 / fs
    n_points = int(np.floor((t_end_ms - t_start_ms) / dt_ms)) + 1
    time_grid = t_start_ms + np.arange(n_points) * dt_ms
    return time_grid


def resample_sensor_to_grid(
    sensor_data: dict,
    time_grid: np.ndarray,
) -> dict[str, np.ndarray]:
    """
    将单个传感器的数值列线性插值到统一时间网格。

    参数:
        sensor_data: 单个传感器的数据字典
        time_grid:   目标时间网格 (ms)

    返回:
        插值后的数据字典，格式与输入相同。
        传感器数据覆盖范围之外的网格点填入 NaN。
    """
    rx = sensor_data["esp32_rx_ms"]

    if len(rx) < 2:
        # 数据点太少，无法插值，全部填 NaN
        n = len(time_grid)
        result = {
            "device_id": sensor_data["device_id"],
            "esp32_rx_ms": time_grid.copy(),
            "sensor_ms": np.full(n, np.nan),
        }
        for col in VALUE_COLUMNS:
            result[col] = np.full(n, np.nan, dtype=np.float32)
        return result

    # 传感器覆盖范围
    mask = (time_grid >= rx[0]) & (time_grid <= rx[-1])
    n = len(time_grid)

    result = {
        "device_id": sensor_data["device_id"],
        "esp32_rx_ms": time_grid.copy(),
    }

    for col in VALUE_COLUMNS:
        interpolated = _linear_interp(rx, sensor_data[col], time_grid)
        interpolated[~mask] = np.nan
        result[col] = interpolated.astype(np.float32)

    # sensor_ms 也插值（用于后续检查，不参与最终输出列）
    result["sensor_ms"] = _linear_interp(rx, sensor_data["sensor_ms"], time_grid)
    result["sensor_ms"][~mask] = np.nan

    return result


def _linear_interp(
    x: np.ndarray, y: np.ndarray, x_new: np.ndarray
) -> np.ndarray:
    """
    线性插值。

    优先使用 scipy.interpolate.interp1d（速度快、鲁棒），
    若 scipy 不可用则降级为 numpy.interp。
    """
    if _HAS_SCIPY:
        f = interp1d(x, y, kind="linear", bounds_error=False,
                     fill_value=np.nan, assume_sorted=True)
        return f(x_new)
    else:
        # numpy.interp: x 必须严格递增
        # 先去重（保留最后一个值，与 np.unique 默认行为一致）
        unique_x, unique_idx = np.unique(x, return_index=True)
        unique_idx.sort()
        unique_y = y[unique_idx]
        return np.interp(x_new, unique_x, unique_y, left=np.nan, right=np.nan)


# ============================================================================
# CSV 输出
# ============================================================================

def write_long_format(
    time_grid: np.ndarray,
    resampled: dict[str, dict],
    output_path: str,
) -> None:
    """
    输出长表格式 CSV。

    格式:
      time_s,esp32_rx_ms,device_id,acc_x,acc_y,acc_z,
      gyro_x,gyro_y,gyro_z,angle_x,angle_y,angle_z,mag_x,mag_y,mag_z

    每个时间点下每个传感器各占一行。跳过全 NaN 行。
    """
    columns = ["time_s", "esp32_rx_ms", "device_id"] + VALUE_COLUMNS
    n_total = 0

    with open(output_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(columns)

        for dev in sorted(resampled.keys()):
            arr = resampled[dev]
            n_rows = 0
            for i in range(len(time_grid)):
                row_vals = [arr[col][i] for col in VALUE_COLUMNS]
                if all(np.isnan(v) for v in row_vals):
                    continue
                time_sec = time_grid[i] / 1000.0
                row = [f"{time_sec:.6f}", f"{time_grid[i]:.3f}", dev]
                row += [f"{v:.6f}" if not np.isnan(v) else "" for v in row_vals]
                writer.writerow(row)
                n_rows += 1
            n_total += n_rows
            print(f"    长表 [{dev}]: {n_rows} 行")

    file_size_mb = os.path.getsize(output_path) / (1024 * 1024)
    print(f"  长表输出: {output_path}")
    print(f"    总行数: {n_total}, 文件大小: {file_size_mb:.2f} MB")


def write_wide_format(
    time_grid: np.ndarray,
    resampled: dict[str, dict],
    output_path: str,
) -> None:
    """
    输出宽表格式 CSV。

    格式:
      time_s,esp32_rx_ms,<dev1>_acc_x,<dev1>_acc_y,...,<dev2>_acc_x,...

    同一行包含所有传感器同一时刻的数据，方便模型输入和多传感器融合。
    跳过全 NaN 行。
    """
    devices = sorted(resampled.keys())

    # 构建列名
    columns = ["time_s", "esp32_rx_ms"]
    for dev in devices:
        for col in VALUE_COLUMNS:
            columns.append(f"{dev}_{col}")

    n_rows = 0
    with open(output_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(columns)

        for i in range(len(time_grid)):
            row = [f"{time_grid[i] / 1000.0:.6f}", f"{time_grid[i]:.3f}"]
            all_nan = True
            for dev in devices:
                for col in VALUE_COLUMNS:
                    v = resampled[dev][col][i]
                    if not np.isnan(v):
                        all_nan = False
                        row.append(f"{v:.6f}")
                    else:
                        row.append("")
            if not all_nan:
                writer.writerow(row)
                n_rows += 1

    file_size_mb = os.path.getsize(output_path) / (1024 * 1024)
    print(f"  宽表输出: {output_path}")
    print(f"    总行数: {n_rows}, 列数: {len(columns)}, 文件大小: {file_size_mb:.2f} MB")


# ============================================================================
# 同步报告
# ============================================================================

def write_sync_report(
    report_path: str,
    input_dir: str,
    csv_files: list[str],
    sensor_reports: list[dict],
    sync_start_ms: float,
    sync_end_ms: float,
    time_grid: np.ndarray,
    fs: float,
    resampled: dict[str, dict],
) -> None:
    """生成详细的同步报告 TXT 文件。"""
    dt_ms = 1000.0 / fs
    duration_s = (sync_end_ms - sync_start_ms) / 1000.0

    lines = []
    lines.append("=" * 70)
    lines.append("多 WT901WIFI 传感器 CSV 后处理同步报告")
    lines.append("=" * 70)
    lines.append(f"生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append(f"输入目录: {input_dir}")
    lines.append(f"目标采样率: {fs} Hz (dt = {dt_ms:.1f} ms = {1/fs:.4f} s)")
    lines.append("")

    # 原始 CSV 列表
    lines.append("-" * 70)
    lines.append("原始 CSV 文件")
    lines.append("-" * 70)
    for fname in csv_files:
        lines.append(f"  {fname}")
    lines.append(f"  共 {len(csv_files)} 个文件")
    lines.append("")

    # 每个传感器详情
    for rep in sensor_reports:
        dev = rep["device_id"]
        lines.append("-" * 70)
        lines.append(f"传感器: {dev}")
        lines.append("-" * 70)
        lines.append(f"  原始帧数:               {rep['n_frames']}")
        lines.append(f"  esp32_rx_ms 范围:       {rep['rx_start_ms']:.0f} → {rep['rx_end_ms']:.0f} ms")
        lines.append(f"  esp32_rx_ms 时长:       {rep['rx_duration_s']:.2f} s")
        lines.append(f"  sensor_ms 范围:         {rep['sms_start_ms']:.0f} → {rep['sms_end_ms']:.0f} ms")

        lines.append(f"  sensor_ms 采样间隔:")
        if rep["sms_mean_interval"] is not None:
            lines.append(f"    均值:   {rep['sms_mean_interval']:.1f} ms")
            lines.append(f"    中位数: {rep['sms_median_interval']:.1f} ms")
            lines.append(f"    标准差: {rep['sms_std_interval']:.1f} ms")
            lines.append(f"    最小:   {rep['sms_min_interval']:.1f} ms")
            lines.append(f"    最大:   {rep['sms_max_interval']:.1f} ms")
            lines.append(f"  sensor_ms 大间隔 (>{dt_ms*2.5:.0f} ms): {rep['sms_long_gaps']} 处")
            if rep["sms_long_gaps"] > 0:
                lines.append(f"    最大间隔: {rep['sms_max_gap']:.0f} ms")
                lines.append(f"    估计丢帧: ~{rep['sms_estimated_missed']} 帧")
        else:
            lines.append(f"    无有效数据")

        lines.append(f"  esp32_rx_ms 采样间隔:")
        lines.append(f"    均值:   {rep['rx_mean_interval']:.1f} ms")
        lines.append(f"    中位数: {rep['rx_median_interval']:.1f} ms")
        lines.append(f"    标准差: {rep['rx_std_interval']:.1f} ms")
        lines.append(f"    最小:   {rep['rx_min_interval']:.1f} ms")
        lines.append(f"    最大:   {rep['rx_max_interval']:.1f} ms")
        lines.append(f"  esp32_rx 大间隔 (>{dt_ms*3:.0f} ms): {rep['rx_long_gaps']} 处")
        if rep["rx_long_gaps"] > 0:
            lines.append(f"    最大间隔: {rep['rx_max_gap']:.0f} ms")
        lines.append(f"  等效采样率: {rep['effective_fs']:.1f} Hz")
        lines.append("")

    # 同步时间范围
    lines.append("-" * 70)
    lines.append("多传感器同步")
    lines.append("-" * 70)
    lines.append(f"  共同时间范围: {sync_start_ms:.1f} → {sync_end_ms:.1f} ms")
    lines.append(f"  同步时长:     {duration_s:.2f} s")
    lines.append(f"  统一时间网格: {len(time_grid)} 点 @ {fs} Hz")
    lines.append(f"  网格步长:     {dt_ms:.1f} ms")
    lines.append("")

    # 各传感器在共同范围内的覆盖率
    lines.append("  各传感器共同时间段内覆盖:")
    for dev in sorted(resampled.keys()):
        arr = resampled[dev]
        valid = np.sum(~np.isnan(arr["acc_x"]))
        total = len(time_grid)
        pct = valid / total * 100 if total > 0 else 0
        lines.append(f"    {dev}: {valid}/{total} 帧 ({pct:.1f}%)")
    lines.append("")

    # 使用建议
    lines.append("-" * 70)
    lines.append("使用建议")
    lines.append("-" * 70)
    lines.append(f"  1. 姿态解算 dt 推荐使用固定值: dt = {1/fs:.4f} s")
    lines.append(f"     不要使用相邻行 esp32_rx_ms 差值作为 dt（可能抖动较大）")
    lines.append(f"  2. esp32_rx_ms 仅用于多传感器统一时间轴同步")
    lines.append(f"  3. sensor_ms 用于检查 WT901 自身采样稳定性")
    lines.append(f"  4. 同步后可基于 acc/gyro/mag 重新进行 Mahony/Madgwick/EKF 姿态解算")
    lines.append(f"  5. 宽表格式 (synced_wide.csv) 适合直接作为 ML 模型输入")
    lines.append("")
    lines.append("=" * 70)
    lines.append("报告结束")
    lines.append("=" * 70)

    with open(report_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))

    print(f"\n  同步报告: {report_path}")


# ============================================================================
# 主入口
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="多 WT901WIFI 传感器 CSV 后处理同步 —— 将多个单传感器 CSV "
                    "重采样到统一时间轴，输出长表和宽表格式",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python sync_multi_imu_csv.py --input_dir ./raw_csv
  python sync_multi_imu_csv.py --input_dir ./data/wt901_20260630_105114 --fs 100
  python sync_multi_imu_csv.py --input_dir ./raw_csv --output_dir ./synced --fs 50
  python sync_multi_imu_csv.py --input_dir ./raw_csv --no-wide
        """,
    )
    parser.add_argument(
        "--input_dir", required=True,
        help="包含单传感器 CSV 文件的文件夹路径",
    )
    parser.add_argument(
        "--output_dir", default=None,
        help="输出目录（默认: input_dir 同级的 synced_output/）",
    )
    parser.add_argument(
        "--fs", type=float, default=100.0,
        help="目标重采样频率 Hz（默认: 100）",
    )
    parser.add_argument(
        "--no-wide", action="store_true",
        help="不生成宽表格式 CSV",
    )
    parser.add_argument(
        "--no-long", action="store_true",
        help="不生成长表格式 CSV",
    )
    parser.add_argument(
        "--csv-sep", default=",",
        help="CSV 列分隔符（默认: 逗号），通常无需修改",
    )

    args = parser.parse_args()

    # ---- 输入检查 ----
    input_dir = os.path.abspath(args.input_dir)
    if not os.path.isdir(input_dir):
        print(f"[错误] 输入文件夹不存在: {input_dir}")
        sys.exit(1)

    # 查找 CSV 文件
    csv_files = sorted([
        f for f in os.listdir(input_dir)
        if f.lower().endswith(".csv") and os.path.isfile(os.path.join(input_dir, f))
    ])
    if not csv_files:
        print(f"[错误] 输入文件夹中没有找到 CSV 文件: {input_dir}")
        sys.exit(1)

    # ---- 输出目录 ----
    if args.output_dir is None:
        parent_dir = os.path.dirname(input_dir)
        if parent_dir == "":
            parent_dir = "."
        output_dir = os.path.join(parent_dir, "synced_output")
    else:
        output_dir = os.path.abspath(args.output_dir)
    os.makedirs(output_dir, exist_ok=True)

    print("=" * 60)
    print("多 WT901WIFI 传感器 CSV 后处理同步")
    print("=" * 60)
    print(f"  输入目录:     {input_dir}")
    print(f"  输出目录:     {output_dir}")
    print(f"  目标频率:     {args.fs} Hz")
    print(f"  固定 dt:      {1.0/args.fs:.4f} s = {1000.0/args.fs:.1f} ms")
    print(f"  CSV 文件数:   {len(csv_files)}")
    print()

    # ======================================================================
    # [1] 读取所有原始 CSV
    # ======================================================================
    print("[1/6] 读取原始 CSV 文件...")
    all_sensors = {}
    for fname in csv_files:
        fpath = os.path.join(input_dir, fname)
        print(f"  读取: {fname} ...", end=" ")
        try:
            sensor_data = parse_single_csv(fpath)
            dev = sensor_data["device_id"]
            if dev in all_sensors:
                print(f"[警告] device_id 重复 ({dev})，数据将被合并")
                # 合并：将新数据追加到已有数据
                existing = all_sensors[dev]
                for col in STD_COLUMN_ORDER:
                    if col in ("device_id", "sensor_timestamp"):
                        continue
                    if col == "sensor_timestamp":
                        existing[col] = np.concatenate([existing[col], sensor_data[col]])
                    else:
                        existing[col] = np.concatenate([existing[col], sensor_data[col]])
                # 重新排序去重
                rx = existing["esp32_rx_ms"]
                order = np.argsort(rx)
                unique_rx, unique_idx = np.unique(rx[order], return_index=True)
                unique_idx.sort()
                for col in STD_COLUMN_ORDER:
                    if col in ("device_id",):
                        continue
                    if col == "sensor_timestamp":
                        existing[col] = existing[col][order][unique_idx]
                    else:
                        existing[col] = existing[col][order][unique_idx]
                all_sensors[dev] = existing
            else:
                all_sensors[dev] = sensor_data
            print(f"{len(sensor_data['esp32_rx_ms'])} 帧 (ID: {dev})")
        except Exception as e:
            print(f"[错误] {e}")
            continue

    if not all_sensors:
        print("\n[错误] 未成功读取任何传感器数据")
        sys.exit(1)

    print(f"\n  共读取 {len(all_sensors)} 个传感器:")
    for dev in sorted(all_sensors.keys()):
        print(f"    - {dev}: {len(all_sensors[dev]['esp32_rx_ms'])} 帧")

    # ======================================================================
    # [2] 数据质量诊断
    # ======================================================================
    print(f"\n[2/6] 数据质量诊断...")
    sensor_reports = []
    for dev in sorted(all_sensors.keys()):
        rep = diagnose_sensor(all_sensors[dev], args.fs)
        sensor_reports.append(rep)
        print_sensor_report(rep, args.fs)

    # ======================================================================
    # [3] 多传感器交叉检查
    # ======================================================================
    print(f"\n[3/6] 多传感器交叉检查...")
    if len(all_sensors) >= 2:
        devs = sorted(all_sensors.keys())
        for i in range(len(devs)):
            for j in range(i + 1, len(devs)):
                d1, d2 = devs[i], devs[j]
                r1 = all_sensors[d1]["esp32_rx_ms"]
                r2 = all_sensors[d2]["esp32_rx_ms"]
                overlap_start = max(r1[0], r2[0])
                overlap_end = min(r1[-1], r2[-1])
                if overlap_start < overlap_end:
                    print(f"    {d1} <-> {d2}: 重叠 {overlap_start:.0f} -> {overlap_end:.0f} ms "
                          f"({(overlap_end - overlap_start)/1000:.2f} s)")
                else:
                    print(f"    {d1} <-> {d2}: [WARN] 无时间重叠！")
    else:
        print("    仅 1 个传感器，无需交叉检查")

    # ======================================================================
    # [4] 计算共同时间范围 & 生成统一时间轴
    # ======================================================================
    print(f"\n[4/6] 计算共同时间范围...")
    try:
        sync_start, sync_end = find_common_time_range(all_sensors)
    except ValueError as e:
        print(f"\n[错误] {e}")
        sys.exit(1)

    print(f"  sync_start: {sync_start:.1f} ms")
    print(f"  sync_end:   {sync_end:.1f} ms")
    print(f"  同步时长:   {(sync_end - sync_start) / 1000:.2f} s")

    time_grid = generate_time_grid(sync_start, sync_end, args.fs)
    dt_ms = 1000.0 / args.fs
    print(f"  时间网格:   {len(time_grid)} 点 @ {args.fs} Hz (步长 {dt_ms:.1f} ms)")

    # ======================================================================
    # [5] 各传感器插值到统一时间轴
    # ======================================================================
    print(f"\n[5/6] 各传感器插值到统一时间轴...")
    resampled = {}
    for dev in sorted(all_sensors.keys()):
        print(f"  插值: {dev} ({len(all_sensors[dev]['esp32_rx_ms'])} 帧) ...", end=" ")
        result = resample_sensor_to_grid(all_sensors[dev], time_grid)
        resampled[dev] = result
        valid = np.sum(~np.isnan(result["acc_x"]))
        print(f"共同时间段内有效帧: {valid}/{len(time_grid)}")

    # ======================================================================
    # [6] 输出
    # ======================================================================
    print(f"\n[6/6] 输出同步结果...")

    # 长表 CSV
    if not args.no_long:
        print("\n  --- 长表格式 ---")
        long_path = os.path.join(output_dir, "synced_long.csv")
        write_long_format(time_grid, resampled, long_path)

    # 宽表 CSV
    if not args.no_wide:
        print("\n  --- 宽表格式 ---")
        wide_path = os.path.join(output_dir, "synced_wide.csv")
        write_wide_format(time_grid, resampled, wide_path)

    # 同步报告
    print("\n  --- 同步报告 ---")
    report_path = os.path.join(output_dir, "sync_report.txt")
    write_sync_report(
        report_path, input_dir, csv_files, sensor_reports,
        sync_start, sync_end, time_grid, args.fs, resampled,
    )

    # ======================================================================
    # 完成
    # ======================================================================
    print("\n" + "=" * 60)
    print("处理完成！")
    print("=" * 60)
    print(f"  输入目录:         {input_dir}")
    print(f"  输出目录:         {output_dir}")
    print(f"  原始 CSV 文件:    {len(csv_files)} 个（未修改）")
    print(f"  传感器数量:       {len(resampled)}")
    print(f"  同步时间范围:     {sync_start:.0f} -> {sync_end:.0f} ms")
    print(f"  同步时长:         {(sync_end - sync_start)/1000:.2f} s")
    print(f"  时间网格点数:     {len(time_grid)}")
    print(f"  输出频率:         {args.fs} Hz")
    print(f"  dt (固定):        {1.0/args.fs:.4f} s = {1000.0/args.fs:.1f} ms")
    print()
    print("  输出文件:")
    if not args.no_long:
        print(f"    长表: {os.path.join(output_dir, 'synced_long.csv')}")
    if not args.no_wide:
        print(f"    宽表: {os.path.join(output_dir, 'synced_wide.csv')}")
    print(f"    报告: {report_path}")
    print()
    print("  使用建议:")
    print(f"    - 姿态解算时使用 dt = {1.0/args.fs:.4f} s（固定采样间隔）")
    print(f"    - 不要使用相邻行 time 差值作为 dt（可能有 NaN 导致间隔不均）")
    print(f"    - esp32_rx_ms 仅用于时间轴对齐，不用于姿态积分 dt")
    print(f"    - 宽表格式可直接作为多传感器融合 / ML 模型输入")


if __name__ == "__main__":
    main()
