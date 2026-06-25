#!/usr/bin/env python3
"""
ESP32 WT901WIFI Gateway — 上位机 CSV 数据接收脚本（串口模式）

通过 USB 串口连接 ESP32，接收传感器 CSV 数据，按设备 ID 自动分文件写入 CSV。

支持多传感器时间同步：当 --sync-sensors > 1 时，只有所有传感器都有某段时间的数
据后，才会将该时段的数据写入各自 CSV，确保多传感器 CSV 时间范围对齐。

ESP32 固件要求: 串口 CSV 行需以 [DATA] 前缀输出（固件已实现）。

输出结构:
    wt901_data_YYYYMMDD_HHMMSS/        ← 数据文件夹
    ├── WT1234567890.csv               ← 传感器 1 的数据
    └── WT0987654321.csv               ← 传感器 2 的数据

用法:
    python csv_receiver.py -p COM5
    python csv_receiver.py -p COM5 --verbose
    python csv_receiver.py -p COM5 --sync-sensors 2
    python csv_receiver.py -p COM5 --sync-sensors 1   # 不启用同步，立即写入
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
from collections import defaultdict, deque
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
DEFAULT_BAUD       = 115200
SERIAL_RETRY_DELAY = 1.0   # 断线重连间隔（秒）

# 同步默认参数
DEFAULT_SYNC_SENSORS   = 2    # 预期传感器数量（1 = 不启用同步）
DEFAULT_SYNC_WINDOW_MS = 100  # 时间对齐容忍窗口（毫秒）
DEFAULT_SYNC_TIMEOUT   = 10   # 传感器失联超时（秒）

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


# ============================================================
# 时间戳解析
# ============================================================

def parse_timestamp_ms(ts_str: str) -> int:
    """
    将 CSV 行中的时间戳字段转为毫秒值（自午夜起）。

    ESP32 固件使用统一的 millis() 生成时间戳（main.cpp:186-192），
    格式为: "YYYY-MM-DD HH:MM:SS.mmm"

    返回: 毫秒值 (hour*3600000 + minute*60000 + second*1000 + millisecond)
    """
    try:
        time_part = ts_str.split(" ")[1]  # "HH:MM:SS.mmm"
    except IndexError:
        return 0

    try:
        hms, ms = time_part.rsplit(".", 1)
        h, m, s = hms.split(":")
        return int(h) * 3600000 + int(m) * 60000 + int(s) * 1000 + int(ms)
    except (ValueError, IndexError):
        return 0


# ============================================================
# 同步缓冲区 — 多传感器时间对齐
# ============================================================

class SyncBuffer:
    """
    按 device_id 缓存 CSV 行，确保所有传感器都有某个时段的数据后才批量写出。

    算法:
      1. 每收到一行，追加到对应设备的 deque 尾部
      2. 计算"安全区间"：所有设备都覆盖到的时间范围
         safe_end = min(各设备最新时间戳)
      3. flush_aligned() 返回安全区间内的行，从各设备缓冲中移除
      4. stale_cleanup() 丢弃超过 sync_timeout_sec 的过期未匹配数据
      5. 设备失联超时自动视为不活跃，不再阻塞对齐

    由于 ESP32 使用统一的 millis() 时钟，不同传感器的 timestamp 天然对齐。
    """

    def __init__(self, sync_sensors: int, sync_window_ms: int, sync_timeout_sec: int):
        self.sync_sensors = sync_sensors
        self.sync_window_ms = sync_window_ms
        self.sync_timeout_ms = sync_timeout_sec * 1000
        self._buffers: dict[str, deque] = defaultdict(lambda: deque())
        self._rollover_offset = 0       # 处理 millis() 24h 翻转
        self._last_raw_ts = None        # 上一次收到的原始 ts
        self.total_dropped = 0          # 因未对齐被丢弃的总行数
        self.total_timeout_dropped = 0  # 因超时被丢弃的总行数

    # ---- 内部方法 ---- #

    def _normalize_ts(self, raw_ts_ms: int) -> int:
        """
        处理 ESP32 millis() 的 24 小时翻转。

        ESP32 固件将 millis() 模 24h 生成时分秒，因此 CSV 中的时间戳
        每 24 小时会从 23:59:59.999 翻回 00:00:00.000。
        检测到大幅回跳（> 12h）时累加 24h 偏移。
        """
        if self._last_raw_ts is not None:
            if raw_ts_ms < self._last_raw_ts - 43200000:  # 回跳超过 12h
                self._rollover_offset += 86400000           # 累加 24h
        self._last_raw_ts = raw_ts_ms
        return raw_ts_ms + self._rollover_offset

    # ---- 公共接口 ---- #

    def add(self, device_id: str, ts_raw_ms: int, row: list[str]) -> None:
        """将一行 CSV 数据加入缓冲。"""
        ts = self._normalize_ts(ts_raw_ms)
        self._buffers[device_id].append((ts, row))

    def flush_aligned(self) -> dict[str, list[list[str]]]:
        """
        尝试对齐写出。返回 {device_id: [row, ...], ...} 中可安全写入的行。

        只有当活跃设备数 >= sync_sensors 且存在重叠时间区间时才返回数据。
        调用方负责将返回的行写入 CSV 文件。
        """
        if self.sync_sensors <= 1:
            # 不启用同步：立即排出所有缓冲的行
            return self._drain_all_internal()

        if self.active_device_count < self.sync_sensors:
            return {}

        # 收集各设备的最早和最晚时间戳
        min_ts = {}
        max_ts = {}
        for did, buf in self._buffers.items():
            if buf:
                min_ts[did] = buf[0][0]
                max_ts[did] = buf[-1][0]

        if len(max_ts) < self.sync_sensors:
            return {}

        # 安全区间 = [max of mins,  min of maxes]
        # safe_start: 最晚开始的那个设备的时间（之前的数据缺伙伴）
        # safe_end:   最早结束的那个设备的时间（之后的数据还没到齐）
        safe_start = max(min_ts.values())
        safe_end   = min(max_ts.values())

        if safe_end <= safe_start:
            return {}

        result: dict[str, list[list[str]]] = {}
        for did, buf in self._buffers.items():
            rows = []
            while buf and buf[0][0] <= safe_end:
                ts, row = buf.popleft()
                if ts >= safe_start:
                    rows.append(row)
                else:
                    self.total_dropped += 1  # 无匹配的旧数据
            if rows:
                result[did] = rows

        return result

    def stale_cleanup(self) -> int:
        """
        丢弃超过 sync_timeout_sec 的孤立缓冲行。

        返回本轮丢弃的行数。
        """
        all_newest = [buf[-1][0] for buf in self._buffers.values() if buf]
        if not all_newest:
            return 0

        newest = max(all_newest)
        threshold = newest - self.sync_timeout_ms
        dropped = 0

        for buf in self._buffers.values():
            while buf and buf[0][0] < threshold:
                buf.popleft()
                dropped += 1

        self.total_timeout_dropped += dropped
        return dropped

    def shutdown_flush(self) -> tuple[dict[str, list[list[str]]], int]:
        """
        程序退出时尽力写出对齐数据。

        返回:
          (aligned_rows: {device_id: [row, ...]}, unaligned_count: 未对齐被丢弃的行数)
        """
        # 先尝试一次对齐写出
        aligned = self.flush_aligned()

        # 剩余未对齐数据
        unaligned = 0
        for buf in self._buffers.values():
            unaligned += len(buf)
        self._buffers.clear()

        return aligned, unaligned

    def _drain_all_internal(self) -> dict[str, list[list[str]]]:
        """（内部）排出所有缓冲的行，用于 sync_sensors=1 模式。"""
        result: dict[str, list[list[str]]] = {}
        for did, buf in self._buffers.items():
            if buf:
                result[did] = [row for _ts, row in buf]
                buf.clear()
        return result

    # ---- 属性 ---- #

    @property
    def active_device_count(self) -> int:
        """在 sync_timeout 窗口内有数据的设备数（用于判断是否满足同步条件）。"""
        all_newest = [buf[-1][0] for buf in self._buffers.values() if buf]
        if not all_newest:
            return 0
        newest = max(all_newest)
        threshold = newest - self.sync_timeout_ms
        return sum(1 for buf in self._buffers.values()
                   if buf and buf[-1][0] >= threshold)

    @property
    def buffered_rows(self) -> int:
        """当前缓冲中的总行数。"""
        return sum(len(buf) for buf in self._buffers.values())

    @property
    def known_devices(self) -> list[str]:
        """所有已见过的 device_id 列表。"""
        return list(self._buffers.keys())


# ============================================================
# 工具函数
# ============================================================

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


# ============================================================
# 主循环
# ============================================================

def receive_csv(
    port: str,
    baud: int,
    output_dir: str,
    verbose: bool,
    sync_sensors: int = DEFAULT_SYNC_SENSORS,
    sync_window_ms: int = DEFAULT_SYNC_WINDOW_MS,
    sync_timeout_sec: int = DEFAULT_SYNC_TIMEOUT,
) -> None:
    """
    主循环：打开串口 → 读取字节流 → 过滤 [DATA] 行 → 同步缓冲 → 写入 CSV → 断线重连

    当 sync_sensors > 1 时，启用多传感器时间对齐：
      - 数据先进入 SyncBuffer 缓冲
      - 只有当所有预期设备都有同一时段的数据后才批量写入
      - 避免出现"传感器 1 先启动，写了一堆数据后传感器 2 才上线"的不对齐问题

    当 sync_sensors == 1 时，行为与旧版本完全一致（即时写入）。
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

    use_sync = sync_sensors > 1
    sync_buffer = None
    if use_sync:
        sync_buffer = SyncBuffer(sync_sensors, sync_window_ms, sync_timeout_sec)
        print(f"[SYNC] Enabled: expecting {sync_sensors} sensors")
        print(f"[SYNC]   window={sync_window_ms}ms  timeout={sync_timeout_sec}s")
        print(f"[SYNC]   Data will be buffered until all sensors are active.")

    writers = {}
    ser = None
    buffer = b""
    retry_count = 0
    last_status_time = 0.0
    total_written = 0

    try:
        while running:
            # ---- 串口连接管理 ---- #
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

            # ---- 读取串口字节 ---- #
            try:
                if ser.in_waiting:
                    data = ser.read(ser.in_waiting)
                else:
                    data = ser.read(1)
                    if not data:
                        # 无数据时仍然执行一次对齐检查和过期清理
                        _sync_tick(sync_buffer, writers, output_dir)
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

            # ---- 逐行处理 ---- #
            while b"\n" in buffer:
                line_bytes, buffer = buffer.split(b"\n", 1)
                line = line_bytes.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                if line.startswith(DATA_PREFIX):
                    csv_content = line[len(DATA_PREFIX):]
                    if not csv_content:
                        continue

                    row = csv_content.split(",")
                    device_id = row[0].strip()
                    if not device_id:
                        if verbose:
                            print(f"[WARN] Line without device_id, skipped")
                        continue

                    ts_ms = parse_timestamp_ms(row[1].strip()) if len(row) > 1 else 0

                    if use_sync and sync_buffer is not None:
                        # ---- 同步模式：先缓冲，对齐后写入 ---- #
                        sync_buffer.add(device_id, ts_ms, row)

                        aligned = sync_buffer.flush_aligned()
                        if aligned:
                            for did, rows in aligned.items():
                                entry = get_device_writer(did, output_dir, writers)
                                for r in rows:
                                    entry["writer"].writerow(r)
                                    entry["rows"] += 1
                                entry["file"].flush()
                            total_written = sum(e["rows"] for e in writers.values())

                        # 定期清理过期数据
                        dropped = sync_buffer.stale_cleanup()

                        # 状态输出（每秒一次）
                        now = time.time()
                        if now - last_status_time >= 1.0:
                            last_status_time = now
                            buffered = sync_buffer.buffered_rows
                            active = sync_buffer.active_device_count
                            known = sync_buffer.known_devices
                            devices_str = ", ".join(
                                f"{did}: {writers[did]['rows']}" if did in writers else f"{did}: 0"
                                for did in known
                            ) if known else "(none)"
                            print(
                                f"[SYNC] written={total_written}  "
                                f"buffered={buffered}  "
                                f"active={active}/{sync_sensors}  "
                                f"dropped(total)={sync_buffer.total_dropped}  "
                                f"|  {devices_str}"
                            )
                    else:
                        # ---- 非同步模式：即时写入（兼容旧行为）---- #
                        entry = get_device_writer(device_id, output_dir, writers)
                        entry["writer"].writerow(row)
                        entry["file"].flush()
                        entry["rows"] += 1

                        if verbose:
                            print(f"[{entry['rows']:>6d}] [{device_id}] {csv_content}")
                        else:
                            total = sum(e["rows"] for e in writers.values())
                            if total % 100 == 0:
                                devices = ", ".join(
                                    f"{did}: {e['rows']}" for did, e in writers.items()
                                )
                                print(f"[INFO] {total} rows total  |  {devices}")

                elif verbose:
                    print(f"[ESP32] {line}")

            # ---- 每轮循环后检查同步缓冲 ---- #
            if use_sync and sync_buffer is not None:
                _sync_tick(sync_buffer, writers, output_dir)

    finally:
        # ---- 清理 ---- #
        if ser:
            try:
                ser.close()
            except Exception:
                pass

        print("\n[INFO] Closing CSV files ...")

        # 同步模式：最后尽力写出对齐数据
        if use_sync and sync_buffer is not None:
            aligned, unaligned = sync_buffer.shutdown_flush()
            if aligned:
                for did, rows in aligned.items():
                    entry = get_device_writer(did, output_dir, writers)
                    for r in rows:
                        entry["writer"].writerow(r)
                        entry["rows"] += 1
                    entry["file"].flush()
            if unaligned > 0:
                print(f"[WARN] {unaligned} rows discarded on shutdown "
                      f"(no matching data from other sensors)")
            print(f"[SYNC] Total dropped (time misaligned): {sync_buffer.total_dropped}")
            print(f"[SYNC] Total dropped (timeout): {sync_buffer.total_timeout_dropped}")

        stats = close_all_writers(writers)

        total = sum(stats.values())
        print(f"\n[DONE] Total rows written: {total}")
        if stats:
            print(f"[DONE] Folder: {output_dir}")
            for device_id, rows in stats.items():
                safe_name = sanitize_filename(device_id)
                print(f"  - {safe_name}.csv  ({device_id}): {rows} rows")


def _sync_tick(sync_buffer: "SyncBuffer | None", writers: dict,
               output_dir: str) -> None:
    """sync 模式的空闲时处理：尝试对齐 + 清理过期数据。"""
    if sync_buffer is None:
        return

    aligned = sync_buffer.flush_aligned()
    if aligned:
        for did, rows in aligned.items():
            entry = get_device_writer(did, output_dir, writers)
            for r in rows:
                entry["writer"].writerow(r)
                entry["rows"] += 1
            entry["file"].flush()

    sync_buffer.stale_cleanup()


# ============================================================
# 入口
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description="ESP32 WT901WIFI Gateway — CSV Serial Receiver",
    )
    parser.add_argument(
        "--serial-port", "-p",
        default=None,
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

    # ---- 多传感器同步选项 ---- #
    sync_group = parser.add_argument_group("Multi-sensor synchronization")
    sync_group.add_argument(
        "--sync-sensors",
        type=int,
        default=DEFAULT_SYNC_SENSORS,
        help=(
            f"Number of expected sensors (default: {DEFAULT_SYNC_SENSORS}). "
            "Set to 1 to disable sync (write immediately). "
            "Set to 2+ to buffer and align data before writing."
        ),
    )
    sync_group.add_argument(
        "--sync-window-ms",
        type=int,
        default=DEFAULT_SYNC_WINDOW_MS,
        help=(
            f"Time tolerance for matching rows across sensors in ms "
            f"(default: {DEFAULT_SYNC_WINDOW_MS})"
        ),
    )
    sync_group.add_argument(
        "--sync-timeout-sec",
        type=int,
        default=DEFAULT_SYNC_TIMEOUT,
        help=(
            f"Seconds before a silent sensor is considered inactive "
            f"(default: {DEFAULT_SYNC_TIMEOUT})"
        ),
    )

    args = parser.parse_args()

    if args.list_ports:
        list_serial_ports()
        return

    if not args.serial_port:
        parser.error("--serial-port/-p is required (use --list-ports to see available ports)")

    if args.sync_sensors < 1:
        parser.error("--sync-sensors must be >= 1")

    output_dir = args.output or generate_dirname()

    print("=" * 60)
    print("ESP32 WT901WIFI Gateway — CSV Serial Receiver")
    print("=" * 60)
    print(f"  Port:        {args.serial_port} @ {args.serial_baud} baud")
    print(f"  Output:      {output_dir}/")
    print(f"  Verbose:     {args.verbose}")
    print(f"  Sync sensors:{args.sync_sensors}")
    if args.sync_sensors > 1:
        print(f"  Sync window: {args.sync_window_ms} ms")
        print(f"  Sync timeout:{args.sync_timeout_sec} s")
    print("=" * 60)
    print("Press Ctrl+C to stop.")
    print()

    receive_csv(
        port=args.serial_port,
        baud=args.serial_baud,
        output_dir=output_dir,
        verbose=args.verbose,
        sync_sensors=args.sync_sensors,
        sync_window_ms=args.sync_window_ms,
        sync_timeout_sec=args.sync_timeout_sec,
    )


if __name__ == "__main__":
    main()
