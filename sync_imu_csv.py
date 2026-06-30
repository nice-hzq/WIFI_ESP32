#!/usr/bin/env python3
"""
sync_imu_csv.py —— 多传感器 IMU 数据后处理同步脚本

功能：
  1. 读取 ESP32 WT901WIFI 网关输出的原始 CSV（含 [DATA] 前缀标记行）
  2. 按 device_id 分组
  3. 每个传感器内部按 esp32_rx_ms 排序，去除重复时间戳
  4. 以 esp32_rx_ms 为统一接收时间轴，生成固定频率时间网格（默认 100 Hz）
  5. 对 acc、gyro、angle、mag 分别线性插值，重采样到统一时间轴
  6. 输出长表格式 CSV：time,device_id,acc_x,...,mag_z
  7. 可选输出宽表格式 CSV：同一行包含所有传感器数据

设计原则：
  - 姿态解算 dt 优先使用固定采样率 dt = 1/fs，不依赖 esp32_rx_ms 相邻差值
  - esp32_rx_ms 用于多传感器统一时间轴对齐
  - sensor_ms 可用于检查 WT901 自身采样是否稳定、是否丢帧或跳变
  - ESP32 端不做插值同步，所有后处理在 Python 侧完成

用法：
  python sync_imu_csv.py <input_csv> [选项]

选项：
  -o, --output-dir DIR      输出目录（默认: input_csv 所在目录的 synced/ 子目录）
  --fs FLOAT                目标采样频率 Hz（默认: 50）
  --no-wide                 不生成宽表格式 CSV
  --no-long                 不生长表格式 CSV
  --csv-sep CHAR            CSV 列分隔符（默认: 逗号）
"""

import argparse
import csv
import os
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy.interpolate import interp1d


# ============================================================================
# 常量
# ============================================================================

# ESP32 输出的 CSV 列名（按 16 列顺序）
CSV_COLUMNS = [
    "device_id",
    "sensor_timestamp",
    "sensor_ms",
    "esp32_rx_ms",
    "acc_x", "acc_y", "acc_z",
    "gyro_x", "gyro_y", "gyro_z",
    "angle_x", "angle_y", "angle_z",
    "mag_x", "mag_y", "mag_z",
]

# 需要插值的数值列
VALUE_COLUMNS = [
    "acc_x", "acc_y", "acc_z",
    "gyro_x", "gyro_y", "gyro_z",
    "angle_x", "angle_y", "angle_z",
    "mag_x", "mag_y", "mag_z",
]

# 宽表输出列：每个传感器一套
SENSOR_COLUMNS = VALUE_COLUMNS  # 每个传感器输出 12 个数值列


# ============================================================================
# 解析 ESP32 CSV
# ============================================================================

def parse_esp32_csv(filepath: str) -> dict[str, dict]:
    """
    读取 ESP32 WT901WIFI 网关输出的原始 CSV。

    支持两种行格式：
      - [DATA]...  带 Serial 前缀标记
      - device_id,...  纯 CSV（TCP 直传）

    返回: {device_id: {col: np.array}}, 按 esp32_rx_ms 升序排列
    """
    raw: dict[str, dict[str, list]] = defaultdict(lambda: defaultdict(list))

    with open(filepath, "r", encoding="utf-8") as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue

            # 去掉 Serial [DATA] 前缀（如果有）
            if line.startswith("[DATA]"):
                line = line[len("[DATA]"):]

            parts = line.split(",")
            if len(parts) != len(CSV_COLUMNS):
                continue  # 跳过格式不匹配的行

            row = dict(zip(CSV_COLUMNS, parts))
            dev = row["device_id"].strip()

            # 解析数值
            try:
                raw[dev]["sensor_ms"].append(int(row["sensor_ms"]))
                raw[dev]["esp32_rx_ms"].append(int(row["esp32_rx_ms"]))
                for col in VALUE_COLUMNS:
                    raw[dev][col].append(float(row[col]))
            except (ValueError, KeyError):
                print(f"  [警告] 第 {line_num} 行数值解析失败，已跳过")
                continue

    # 转为 numpy 并排序
    result: dict[str, dict] = {}
    for dev, cols in raw.items():
        arr = {}
        rx = np.array(cols["esp32_rx_ms"], dtype=np.float64)
        order = np.argsort(rx)
        arr["esp32_rx_ms"] = rx[order]
        arr["sensor_ms"] = np.array(cols["sensor_ms"], dtype=np.float64)[order]
        for col in VALUE_COLUMNS:
            arr[col] = np.array(cols[col], dtype=np.float64)[order]
        result[dev] = arr

    return result


# ============================================================================
# 去重
# ============================================================================

def remove_duplicate_timestamps(
    sensor_data: dict[str, dict]
) -> dict[str, dict]:
    """
    去除每个传感器内部 esp32_rx_ms 重复的时间戳。

    策略：对重复时间戳取均值（同一时间点收到的多个数据做平均）。
    """
    for dev, arr in sensor_data.items():
        rx = arr["esp32_rx_ms"]
        unique_rx, inverse, counts = np.unique(rx, return_inverse=True, return_counts=True)

        if len(unique_rx) == len(rx):
            continue  # 无重复

        dup_count = len(rx) - len(unique_rx)
        print(f"  [{dev}] 检测到 {dup_count} 个重复 esp32_rx_ms，已合并取均值")

        new_arr = {"esp32_rx_ms": unique_rx}
        for col in ["sensor_ms"] + VALUE_COLUMNS:
            summed = np.bincount(inverse, weights=arr[col])
            new_arr[col] = summed / counts
        sensor_data[dev] = new_arr

    return sensor_data


# ============================================================================
# 重采样到统一时间轴
# ============================================================================

def resample_to_grid(
    sensor_data: dict[str, dict],
    fs: float,
) -> tuple[np.ndarray, dict[str, dict]]:
    """
    以 esp32_rx_ms 为基准，生成固定频率时间网格，对所有传感器数据线性插值。

    参数:
        sensor_data: {device_id: {col: np.array}}
        fs: 目标采样频率 Hz

    返回:
        (time_grid, resampled): 时间网格 (ms), {device_id: {col: np.array}}
    """
    if not sensor_data:
        raise ValueError("无有效传感器数据")

    dt_ms = 1000.0 / fs

    # 1. 找到所有传感器共同的 esp32_rx_ms 时间范围
    starts, ends = [], []
    for dev, arr in sensor_data.items():
        rx = arr["esp32_rx_ms"]
        starts.append(rx[0])
        ends.append(rx[-1])

    t_start = max(starts)
    t_end = min(ends)

    if t_start >= t_end:
        print("\n  [错误] 各传感器 esp32_rx_ms 时间范围无交集！")
        for dev, arr in sensor_data.items():
            print(f"    {dev}: {arr['esp32_rx_ms'][0]:.0f} -> {arr['esp32_rx_ms'][-1]:.0f} ms")
        raise ValueError("传感器时间范围不重叠，无法生成统一时间轴")

    # 2. 生成统一时间网格
    n_points = int(np.floor((t_end - t_start) / dt_ms)) + 1
    time_grid = t_start + np.arange(n_points) * dt_ms

    print(f"\n  统一时间轴: {t_start:.1f} -> {t_end:.1f} ms")
    print(f"  目标频率: {fs} Hz, dt = {dt_ms:.1f} ms, 网格点数: {n_points}")
    print(f"  有效时长: {(n_points - 1) * dt_ms / 1000:.2f} s")

    # 3. 对每个传感器插值
    resampled: dict[str, dict] = {}
    for dev, arr in sensor_data.items():
        rx = arr["esp32_rx_ms"]

        # 检查数据量
        if len(rx) < 2:
            print(f"  [警告] {dev}: 数据点不足（{len(rx)}），无法插值，跳过")
            continue

        # 只插值在 sensor 有效范围内的网格点
        mask = (time_grid >= rx[0]) & (time_grid <= rx[-1])

        resampled[dev] = {"esp32_rx_ms": time_grid.copy()}

        for col in VALUE_COLUMNS:
            f_interp = interp1d(
                rx, arr[col],
                kind="linear",
                bounds_error=False,
                fill_value=np.nan,
            )
            interpolated = f_interp(time_grid)
            # 将传感器覆盖范围外的点设为 NaN
            interpolated[~mask] = np.nan
            resampled[dev][col] = interpolated.astype(np.float32)

        # sensor_ms 也插值（用于检查）
        f_sms = interp1d(
            rx, arr["sensor_ms"],
            kind="linear",
            bounds_error=False,
            fill_value=np.nan,
        )
        resampled[dev]["sensor_ms"] = f_sms(time_grid)
        resampled[dev]["sensor_ms"][~mask] = np.nan

    return time_grid, resampled


# ============================================================================
# 长表输出
# ============================================================================

def write_long_format(
    time_grid: np.ndarray,
    resampled: dict[str, dict],
    output_path: str,
) -> None:
    """
    输出长表格式 CSV:
    time,device_id,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,
      angle_x,angle_y,angle_z,mag_x,mag_y,mag_z
    """
    columns = ["time", "device_id"] + VALUE_COLUMNS

    with open(output_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(columns)

        for dev in sorted(resampled.keys()):
            arr = resampled[dev]
            for i in range(len(time_grid)):
                row_vals = [arr[col][i] for col in VALUE_COLUMNS]
                # 跳过全 NaN 行
                if all(np.isnan(v) for v in row_vals):
                    continue
                time_sec = time_grid[i] / 1000.0
                writer.writerow([f"{time_sec:.6f}", dev] + [f"{v:.6f}" if not np.isnan(v) else "" for v in row_vals])

    print(f"  长表输出: {output_path}")


# ============================================================================
# 宽表输出
# ============================================================================

def write_wide_format(
    time_grid: np.ndarray,
    resampled: dict[str, dict],
    output_path: str,
) -> None:
    """
    输出宽表格式 CSV:
    time_ms,<dev1>_acc_x,<dev1>_acc_y,...,<dev2>_acc_x,...
    同一行包含所有传感器同一时刻的数据，方便模型输入。
    """
    devices = sorted(resampled.keys())

    # 构建列名
    columns = ["time_ms"]
    for dev in devices:
        for col in VALUE_COLUMNS:
            columns.append(f"{dev}_{col}")

    with open(output_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(columns)

        for i in range(len(time_grid)):
            row = [f"{time_grid[i]:.3f}"]
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

    print(f"  宽表输出: {output_path}")


# ============================================================================
# 数据质量诊断
# ============================================================================

def print_diagnostics(sensor_data: dict[str, dict], fs: float) -> None:
    """打印每个传感器的数据质量诊断信息。"""
    print("\n" + "=" * 60)
    print("数据质量诊断")
    print("=" * 60)

    dt_expected = 1000.0 / fs  # ms

    for dev in sorted(sensor_data.keys()):
        arr = sensor_data[dev]
        rx = arr["esp32_rx_ms"]
        sms = arr["sensor_ms"]

        print(f"\n  [{dev}]")
        print(f"    数据点数: {len(rx)}")
        print(f"    esp32_rx_ms 范围: {rx[0]:.1f} -> {rx[-1]:.1f} ms "
              f"(时长 {(rx[-1] - rx[0]) / 1000:.2f} s)")

        # esp32_rx_ms 间隔统计
        rx_diffs = np.diff(rx)
        print(f"    esp32_rx_ms 间隔: "
              f"均值={rx_diffs.mean():.1f} ms, "
              f"中位数={np.median(rx_diffs):.1f} ms, "
              f"std={rx_diffs.std():.1f} ms, "
              f"min={rx_diffs.min():.1f} ms, "
              f"max={rx_diffs.max():.1f} ms")

        # sensor_ms 自身采样间隔
        sms_diffs = np.diff(sms)
        # sensor_ms 可能跨天回绕，过滤异常值
        sms_diffs_valid = sms_diffs[(sms_diffs > 0) & (sms_diffs < 60000)]
        if len(sms_diffs_valid) > 0:
            print(f"    sensor_ms 间隔: "
                  f"均值={sms_diffs_valid.mean():.1f} ms, "
                  f"中位数={np.median(sms_diffs_valid):.1f} ms, "
                  f"std={sms_diffs_valid.std():.1f} ms, "
                  f"min={sms_diffs_valid.min():.1f} ms, "
                  f"max={sms_diffs_valid.max():.1f} ms")
        else:
            print(f"    sensor_ms 间隔: 无有效数据（可能全部跨天回绕）")

        # 丢帧/跳变检查
        long_gaps = rx_diffs[rx_diffs > dt_expected * 2.5]
        if len(long_gaps) > 0:
            print(f"    [警告] esp32_rx_ms 大间隔 (> {dt_expected * 2.5:.0f} ms): "
                  f"共 {len(long_gaps)} 处, "
                  f"最大 {long_gaps.max():.0f} ms")
        else:
            print(f"    无明显大间隔")

        # 估计实际采样率
        if len(rx) >= 2:
            effective_fs = 1000.0 / rx_diffs.mean() if rx_diffs.mean() > 0 else float("inf")
            print(f"    等效采样率: {effective_fs:.1f} Hz (目标 {fs} Hz)")

    # 多传感器交叉检查
    if len(sensor_data) >= 2:
        print("\n  多传感器交叉检查:")
        devs = sorted(sensor_data.keys())
        for i in range(len(devs)):
            for j in range(i + 1, len(devs)):
                d1, d2 = devs[i], devs[j]
                r1, r2 = sensor_data[d1]["esp32_rx_ms"], sensor_data[d2]["esp32_rx_ms"]
                overlap_start = max(r1[0], r2[0])
                overlap_end = min(r1[-1], r2[-1])
                if overlap_start < overlap_end:
                    print(f"    {d1} <-> {d2}: 重叠范围 "
                          f"{overlap_start:.0f} -> {overlap_end:.0f} ms "
                          f"({(overlap_end - overlap_start) / 1000:.2f} s)")
                else:
                    print(f"    {d1} <-> {d2}: [警告] 时间范围无重叠！")

    print()


# ============================================================================
# 主入口
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="多传感器 IMU 数据后处理同步 —— 将 ESP32 WT901WIFI 网关原始 CSV "
                    "重采样到统一时间轴",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python sync_imu_csv.py data.csv
  python sync_imu_csv.py data.csv --fs 100 -o ./synced/
  python sync_imu_csv.py data.csv --no-wide
        """,
    )
    parser.add_argument("input_csv", help="ESP32 网关输出的原始 CSV 文件路径")
    parser.add_argument(
        "-o", "--output-dir", default=None,
        help="输出目录（默认: 输入文件所在目录下的 synced/ 子目录）",
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
        help="不生长表格式 CSV",
    )
    parser.add_argument(
        "--csv-sep", default=",",
        help="CSV 列分隔符（默认: 逗号），通常无需修改",
    )

    args = parser.parse_args()

    # ---- 输入检查 ----
    if not os.path.isfile(args.input_csv):
        print(f"[错误] 输入文件不存在: {args.input_csv}")
        sys.exit(1)

    # ---- 输出目录 ----
    if args.output_dir is None:
        input_dir = os.path.dirname(os.path.abspath(args.input_csv))
        output_dir = os.path.join(input_dir, "synced")
    else:
        output_dir = args.output_dir
    os.makedirs(output_dir, exist_ok=True)

    input_stem = Path(args.input_csv).stem

    print("=" * 60)
    print("IMU 数据同步后处理")
    print("=" * 60)
    print(f"  输入文件: {args.input_csv}")
    print(f"  输出目录: {output_dir}")
    print(f"  目标频率: {args.fs} Hz")
    print()

    # ---- 1. 解析 ----
    print("[1/5] 解析原始 CSV...")
    sensor_data = parse_esp32_csv(args.input_csv)

    if not sensor_data:
        print("[错误] 未解析到任何有效传感器数据")
        sys.exit(1)

    print(f"  发现 {len(sensor_data)} 个传感器:")
    for dev in sorted(sensor_data.keys()):
        print(f"    - {dev}: {len(sensor_data[dev]['esp32_rx_ms'])} 帧")

    # ---- 2. 去重 ----
    print("\n[2/5] 去除重复时间戳...")
    sensor_data = remove_duplicate_timestamps(sensor_data)

    # ---- 3. 诊断 ----
    print("\n[3/5] 数据质量诊断...")
    print_diagnostics(sensor_data, args.fs)

    # ---- 4. 重采样 ----
    print("[4/5] 重采样到统一时间轴...")
    try:
        time_grid, resampled = resample_to_grid(sensor_data, args.fs)
    except ValueError as e:
        print(f"[错误] 重采样失败: {e}")
        sys.exit(1)

    if not resampled:
        print("[错误] 所有传感器均因数据不足被跳过")
        sys.exit(1)

    # ---- 5. 输出 ----
    print("\n[5/5] 输出 CSV...")

    if not args.no_long:
        long_path = os.path.join(output_dir, f"{input_stem}_long.csv")
        write_long_format(time_grid, resampled, long_path)

    if not args.no_wide:
        wide_path = os.path.join(output_dir, f"{input_stem}_wide.csv")
        write_wide_format(time_grid, resampled, wide_path)

    print("\n" + "=" * 60)
    print("处理完成！")
    print("=" * 60)
    print(f"  传感器数量: {len(resampled)}")
    print(f"  时间网格点数: {len(time_grid)}")
    print(f"  输出频率: {args.fs} Hz")
    print(f"  dt (固定, 建议用于姿态解算): {1000.0 / args.fs:.2f} ms = {1.0 / args.fs:.4f} s")
    print(f"  输出目录: {output_dir}")
    print()
    print("  使用建议:")
    print(f"    - 姿态解算时使用 dt = {1.0 / args.fs:.4f} s（固定采样间隔）")
    print(f"    - 不要使用相邻行 time 差值作为 dt（可能有 NaN 行导致间隔不均）")
    print(f"    - esp32_rx_ms 仅用于时间轴对齐，不用于计算 dt")


if __name__ == "__main__":
    main()
