#!/usr/bin/env python3
"""
ESP32 WT901WIFI Gateway — 上位机 CSV 数据接收脚本（串口模式）

通过 USB 串口连接 ESP32，接收传感器 CSV 数据，按设备 ID 自动分文件写入 CSV。

ESP32 固件要求: 串口 CSV 行需以 [DATA] 前缀输出（固件已实现）。

输出结构:
    wt901_data_YYYYMMDD_HHMMSS/        ← 数据文件夹
    ├── WT1234567890.csv               ← 传感器 1 的数据
    └── WT0987654321.csv               ← 传感器 2 的数据

用法:
    python csv_receiver.py -p COM5
    python csv_receiver.py -p COM5 --verbose
    python csv_receiver.py -p COM5 -b 115200 --output ./my_data
    python csv_receiver.py --list-ports

CSV 格式（14 列）:
    device_id, timestamp,
    Acc_x, Acc_y, Acc_z,
    Gyr_x, Gyr_y, Gyr_z,
    Angle_x, Angle_y, Angle_z,
    Geo_x, Geo_y, Geo_z
"""

import argparse
import csv
import os
import signal
import sys
import time
from datetime import datetime

# ============================================================
# pyserial（必需依赖）
# ============================================================
try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("[ERROR] pyserial is required. Run: pip install pyserial")
    sys.exit(1)

# ============================================================
# 默认配置
# ============================================================
DEFAULT_BAUD      = 115200
SERIAL_RETRY_DELAY = 1.0   # 断线重连间隔（秒）

# CSV 列名
CSV_HEADER = [
    "device_id", "timestamp",
    "Acc_x", "Acc_y", "Acc_z",
    "Gyr_x", "Gyr_y", "Gyr_z",
    "Angle_x", "Angle_y", "Angle_z",
    "Geo_x", "Geo_y", "Geo_z",
]

# ESP32 串口 CSV 行前缀
DATA_PREFIX = "[DATA]"


def generate_dirname() -> str:
    """生成带时间戳的输出文件夹路径（项目根目录 data/ 下）"""
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)
    return os.path.join(project_root, "data", f"wt901_data_{ts}")


def sanitize_filename(name: str) -> str:
    """清理文件名中的非法字符"""
    safe = "".join(c if c.isalnum() or c in "_-" else "_" for c in name)
    return safe.strip("_") or "unknown_device"


def open_serial(port: str, baud: int, timeout: float = 1.0):
    """打开串口连接，失败返回 None"""
    try:
        ser = serial.Serial(port, baud, timeout=timeout)
        print(f"[OK] Serial port {port} opened at {baud} baud")
        return ser
    except (serial.SerialException, OSError) as e:
        print(f"[ERROR] Cannot open serial port {port}: {e}")
        return None


def list_serial_ports():
    """列出系统中可用的串口"""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found.")
        return
    print("Available serial ports:")
    for port in ports:
        print(f"  {port.device} - {port.description}")


def get_device_writer(device_id: str, output_dir: str, writers: dict):
    """
    获取指定设备 ID 对应的 CSV writer 和文件句柄。
    如果该设备首次出现，自动创建对应的 CSV 文件并写入表头。
    """
    if device_id in writers:
        return writers[device_id]

    safe_name = sanitize_filename(device_id)
    filepath = os.path.join(output_dir, f"{safe_name}.csv")
    file_exists = os.path.exists(filepath) and os.path.getsize(filepath) > 0

    f = open(filepath, "a", newline="", encoding="utf-8")
    writer = csv.writer(f)

    if not file_exists:
        writer.writerow(CSV_HEADER)
        f.flush()
        print(f"[CSV] Created: {filepath} -> device: {device_id}")

    entry = {"file": f, "writer": writer, "rows": 0, "path": filepath}
    writers[device_id] = entry
    return entry


def close_all_writers(writers: dict) -> dict:
    """关闭所有打开的 CSV 文件，返回各设备的行数统计"""
    stats = {}
    for device_id, entry in writers.items():
        entry["file"].close()
        stats[device_id] = entry["rows"]
    writers.clear()
    return stats


def process_csv_line(line: str, output_dir: str, writers: dict, verbose: bool):
    """处理一行 CSV 数据：按 device_id 分文件写入。"""
    row = line.split(",")
    device_id = row[0].strip()

    if not device_id:
        print(f"[WARN] Line without device_id, skipped: {line[:60]}")
        return

    entry = get_device_writer(device_id, output_dir, writers)
    entry["writer"].writerow(row)
    entry["file"].flush()
    entry["rows"] += 1

    if verbose:
        print(f"[{entry['rows']:>6d}] [{device_id}] {line}")
    else:
        total = sum(e["rows"] for e in writers.values())
        if total % 100 == 0:
            devices = ", ".join(f"{did}: {e['rows']}" for did, e in writers.items())
            print(f"[INFO] {total} rows total  |  {devices}")


def receive_csv(
    port: str,
    baud: int,
    output_dir: str,
    verbose: bool,
) -> None:
    """
    主循环：打开串口 → 读取字节流 → 过滤 [DATA] 行 → 写入 CSV → 断线重连
    """
    running = True

    def stop_handler(signum, frame):
        nonlocal running
        print("\n[STOP] Shutting down...")
        running = False

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    os.makedirs(output_dir, exist_ok=True)
    print(f"[INFO] Output folder: {output_dir}")

    writers = {}
    ser = None
    buffer = b""
    retry_count = 0

    try:
        while running:
            if ser is None:
                if retry_count > 0 and retry_count % 5 == 0:
                    list_serial_ports()
                print(f"[SERIAL] Opening {port} at {baud} baud...")
                ser = open_serial(port, baud, timeout=1.0)
                if ser is None:
                    retry_count += 1
                    print(f"[SERIAL] Retrying in {SERIAL_RETRY_DELAY}s ...")
                    time.sleep(SERIAL_RETRY_DELAY)
                    continue
                buffer = b""
                retry_count = 0

            try:
                if ser.in_waiting:
                    data = ser.read(ser.in_waiting)
                else:
                    data = ser.read(1)
                    if not data:
                        continue
            except (serial.SerialException, OSError) as e:
                print(f"[SERIAL] Connection lost: {e}")
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
                continue

            buffer += data

            while b"\n" in buffer:
                line_bytes, buffer = buffer.split(b"\n", 1)
                line = line_bytes.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                if line.startswith(DATA_PREFIX):
                    csv_content = line[len(DATA_PREFIX):]
                    if csv_content:
                        process_csv_line(csv_content, output_dir, writers, verbose)
                elif verbose:
                    print(f"[ESP32] {line}")

    finally:
        if ser:
            try:
                ser.close()
            except Exception:
                pass

        print("\n[INFO] Closing CSV files ...")
        stats = close_all_writers(writers)

        total = sum(stats.values())
        print(f"\n[DONE] Total rows: {total}")
        if stats:
            print(f"[DONE] Folder: {output_dir}")
            for device_id, rows in stats.items():
                safe_name = sanitize_filename(device_id)
                print(f"  - {safe_name}.csv  ({device_id}): {rows} rows")


def main():
    parser = argparse.ArgumentParser(
        description="ESP32 WT901WIFI Gateway — CSV Serial Receiver",
    )
    parser.add_argument(
        "--serial-port", "-p",
        required=True,
        help="Serial port name (e.g., COM5 on Windows, /dev/ttyUSB0 on Linux)",
    )
    parser.add_argument(
        "--serial-baud", "-b",
        type=int,
        default=DEFAULT_BAUD,
        help=f"Serial baud rate (default: {DEFAULT_BAUD})",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Output folder path (default: auto-generated with timestamp)",
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Print every received CSV line to console",
    )
    parser.add_argument(
        "--list-ports",
        action="store_true",
        help="List available serial ports and exit",
    )

    args = parser.parse_args()

    if args.list_ports:
        list_serial_ports()
        return

    output_dir = args.output or generate_dirname()

    print("=" * 60)
    print("ESP32 WT901WIFI Gateway — CSV Serial Receiver")
    print("=" * 60)
    print(f"  Port:   {args.serial_port} @ {args.serial_baud} baud")
    print(f"  Output: {output_dir}/")
    print(f"  Verbose: {args.verbose}")
    print("=" * 60)
    print("Press Ctrl+C to stop.")
    print()

    receive_csv(
        port=args.serial_port,
        baud=args.serial_baud,
        output_dir=output_dir,
        verbose=args.verbose,
    )


if __name__ == "__main__":
    main()
