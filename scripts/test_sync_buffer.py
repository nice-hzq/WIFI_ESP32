#!/usr/bin/env python3
r"""
测试 SyncBuffer 多传感器时间对齐逻辑。

用法:
    cd D:\project\ESP_32\WIFI_ESP32
    python -m pytest scripts/test_sync_buffer.py -v
    # 或直接运行:
    python scripts/test_sync_buffer.py
"""

import sys
import os

# 确保可以导入 csv_receiver 模块
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from csv_receiver import SyncBuffer, parse_timestamp_ms


def make_row(device_id: str, timestamp: str) -> list[str]:
    """构造一条模拟 CSV 行（14 列）"""
    return [device_id, timestamp] + ["0.0"] * 12


# ============================================================
# parse_timestamp_ms 测试
# ============================================================

def test_parse_timestamp_ms():
    assert parse_timestamp_ms("2026-06-10 00:00:00.000") == 0
    assert parse_timestamp_ms("2026-06-10 00:00:01.000") == 1000
    assert parse_timestamp_ms("2026-06-10 00:01:00.000") == 60000
    assert parse_timestamp_ms("2026-06-10 01:00:00.000") == 3600000
    assert parse_timestamp_ms("2026-06-10 12:30:45.678") == \
        12 * 3600000 + 30 * 60000 + 45 * 1000 + 678
    assert parse_timestamp_ms("2026-06-10 23:59:59.999") == 86399999
    print("  PASS test_parse_timestamp_ms")


# ============================================================
# SyncBuffer — sync_sensors=1（即时模式）
# ============================================================

def test_sync_disabled_immediate_write():
    """sync_sensors=1 时应该立即排出所有数据"""
    buf = SyncBuffer(sync_sensors=1, sync_window_ms=100, sync_timeout_sec=10)

    # 添加一行
    ts = parse_timestamp_ms("2026-06-10 14:30:00.000")
    buf.add("WT_DEV_A", ts, make_row("WT_DEV_A", "2026-06-10 14:30:00.000"))

    result = buf.flush_aligned()
    assert "WT_DEV_A" in result
    assert len(result["WT_DEV_A"]) == 1
    assert buf.buffered_rows == 0
    print("  PASS test_sync_disabled_immediate_write")


# ============================================================
# SyncBuffer — 单设备未满足 sync_sensors
# ============================================================

def test_single_device_buffers_when_waiting():
    """只有一个设备时不应写出数据（sync_sensors=2）"""
    buf = SyncBuffer(sync_sensors=2, sync_window_ms=100, sync_timeout_sec=10)

    ts = parse_timestamp_ms("2026-06-10 14:30:00.000")
    buf.add("WT_DEV_A", ts, make_row("WT_DEV_A", "2026-06-10 14:30:00.000"))

    result = buf.flush_aligned()
    assert result == {}  # 只有一个设备，不满足 sync_sensors=2
    assert buf.buffered_rows == 1
    assert buf.active_device_count == 1
    print("  PASS test_single_device_buffers_when_waiting")


# ============================================================
# SyncBuffer — 双设备时间对齐
# ============================================================

def test_dual_device_aligned_write():
    """双设备有重叠时间区间时正确输出对齐数据"""
    buf = SyncBuffer(sync_sensors=2, sync_window_ms=100, sync_timeout_sec=10)

    # 传感器 A：时间 1000ms ~ 2000ms（每个间隔 20ms = 50Hz）
    for t in range(1000, 2001, 20):
        ts_str = f"2026-06-10 00:00:0{t // 1000}.{t % 1000:03d}"
        buf.add("WT_A", t, make_row("WT_A", ts_str))

    # 传感器 B：时间 1500ms ~ 2500ms
    for t in range(1500, 2501, 10):
        ts_str = f"2026-06-10 00:00:0{t // 1000}.{t % 1000:03d}"
        buf.add("WT_B", t, make_row("WT_B", ts_str))

    # 此时 safe_start = 1500 (A 从 1000 开始, B 从 1500 开始)
    #      safe_end   = 2000 (A 到 2000, B 到 2500)
    # 应该输出 [1500, 2000] 范围内的行
    result = buf.flush_aligned()

    assert "WT_A" in result
    assert "WT_B" in result

    rows_a = result["WT_A"]
    rows_b = result["WT_B"]

    # 写出的是 [1500, 2000] 区间，每个传感器约 51 行 (500ms / 10ms + 1)
    assert len(rows_a) == 51, f"Expected 51 rows from WT_A, got {len(rows_a)}"
    assert len(rows_b) == 51, f"Expected 51 rows from WT_B, got {len(rows_b)}"

    # A 还剩 0 行（1000~1490 被丢弃 + 1500~2000 已写出）
    # B 还剩 [2010, 2500] 的行
    assert buf.buffered_rows == 50  # B: (2500-2000)/10 = 50 行

    # 验证丢弃计数：A 的 [1000, 1490] = 50 行被丢弃
    assert buf.total_dropped == 50, f"Expected 50 dropped, got {buf.total_dropped}"

    print("  PASS test_dual_device_aligned_write")


# ============================================================
# SyncBuffer — 传感器 B 晚启动
# ============================================================

def test_late_sensor_join():
    """模拟传感器 A 先启动 5 秒，传感器 B 后来加入的场景"""
    buf = SyncBuffer(sync_sensors=2, sync_window_ms=100, sync_timeout_sec=10)

    # A 从 0ms 开始发送（50Hz）
    for t in range(0, 10000, 20):  # 0, 20, ..., 9980 → 500 行
        ts_str = f"2026-06-10 00:00:0{t // 1000}.{t % 1000:03d}"
        buf.add("WT_A", t, make_row("WT_A", ts_str))

    # 此时只有 A，不应写出
    result = buf.flush_aligned()
    assert result == {}
    assert buf.buffered_rows == 1000  # A: 10000ms / 10ms = 1000 行
    assert buf.active_device_count == 1

    # B 在 5000ms 时加入
    for t in range(5000, 10001, 10):  # 5000, 5010, ..., 10000 → 501 行
        ts_str = f"2026-06-10 00:00:0{t // 1000}.{t % 1000:03d}"
        buf.add("WT_B", t, make_row("WT_B", ts_str))

    # 现在两个设备都有数据
    result = buf.flush_aligned()

    # safe_start = 5000 (B 的开始), safe_end = 9990 (A 的结束, 早于 B 的 10000)
    assert "WT_A" in result
    assert "WT_B" in result

    rows_a = result["WT_A"]
    rows_b = result["WT_B"]

    assert len(rows_a) == 500  # A: (9990-5000)/10 + 1 = 500 行 [5000, 9990]
    assert len(rows_b) == 500  # B: (9990-5000)/10 + 1 = 500 行 [5000, 9990]

    # A 的 [0, 4990] 被丢弃（500 行）
    assert buf.total_dropped == 500

    # B 剩余 ts=10000 的 1 行等待 A 追上
    assert buf.buffered_rows == 1

    print("  PASS test_late_sensor_join")


# ============================================================
# SyncBuffer — 过期清理
# ============================================================

def test_stale_cleanup():
    """超时后旧数据被清理"""
    buf = SyncBuffer(sync_sensors=2, sync_window_ms=100, sync_timeout_sec=1)  # 1s 超时

    # A 从 0ms 开始发送
    for t in range(0, 2000, 10):
        buf.add("WT_A", t, make_row("WT_A", f"2026-06-10 00:00:0{t//1000}.{t%1000:03d}"))

    # B 没来 —— A 的数据会因超时被清理
    # 先确认 active_device_count = 1
    assert buf.active_device_count == 1

    # 模拟时间推进 —— 需要让 B 的最新的 ts 远超 A
    # 但 B 没有数据，cleanup 使用所有设备的最新的
    # A 最新的 = 1990，threshold = 1990 - 1000 = 990
    dropped = buf.stale_cleanup()
    # A 的 [0, 990) 应该被清理：990/10 = 99 行
    assert dropped == 99, f"Expected 99 dropped, got {dropped}"
    assert buf.buffered_rows == 101  # 200 - 99 = 101

    print("  PASS test_stale_cleanup")


# ============================================================
# SyncBuffer — 24 小时翻转
# ============================================================

def test_millis_rollover():
    """ESP32 millis() 24h 翻转测试"""
    buf = SyncBuffer(sync_sensors=1, sync_window_ms=100, sync_timeout_sec=10)

    # 23:59:59.900
    ts1 = parse_timestamp_ms("2026-06-10 23:59:59.900")
    buf.add("WT_A", ts1, make_row("WT_A", "2026-06-10 23:59:59.900"))

    # 00:00:00.100（翻转后）
    ts2 = parse_timestamp_ms("2026-06-11 00:00:00.100")
    buf.add("WT_A", ts2, make_row("WT_A", "2026-06-11 00:00:00.100"))

    result = buf.flush_aligned()

    # 两行都应该输出
    assert len(result["WT_A"]) == 2
    # 翻转后的归一化时间戳应该是 86400100（86400000 + 100）
    print("  PASS test_millis_rollover")


# ============================================================
# SyncBuffer — 设备断线后 active_device_count 下降
# ============================================================

def test_device_goes_inactive():
    """设备长期不发数据后变为不活跃"""
    buf = SyncBuffer(sync_sensors=2, sync_window_ms=100, sync_timeout_sec=1)  # 1s 超时

    # A 和 B 同时有数据
    buf.add("WT_A", 1000, make_row("WT_A", "2026-06-10 00:00:01.000"))
    buf.add("WT_B", 1000, make_row("WT_B", "2026-06-10 00:00:01.000"))
    assert buf.active_device_count == 2

    # B 停止发送，A 继续
    for t in range(1010, 5000, 10):
        buf.add("WT_A", t, make_row("WT_A", f"2026-06-10 00:00:0{t//1000}.{t%1000:03d}"))

    # B 的最新 = 1000，A 的最新 = 4990
    # threshold = 4990 - 1000 = 3990
    # B 的最新 (1000) < 3990 → B 不活跃
    assert buf.active_device_count == 1

    # flush_aligned 应该返回空（活跃设备只有 1 个，不满足 sync_sensors=2）
    result = buf.flush_aligned()
    assert result == {}

    print("  PASS test_device_goes_inactive")


# ============================================================
# SyncBuffer — shutdown_flush
# ============================================================

def test_shutdown_flush():
    """shutdown_flush 在退出时尽力写出"""
    buf = SyncBuffer(sync_sensors=2, sync_window_ms=100, sync_timeout_sec=10)

    # 双设备对齐数据
    for t in range(1000, 2001, 10):
        buf.add("WT_A", t, make_row("WT_A", f"2026-06-10 00:00:0{t//1000}.{t%1000:03d}"))
    for t in range(1500, 2001, 10):
        buf.add("WT_B", t, make_row("WT_B", f"2026-06-10 00:00:0{t//1000}.{t%1000:03d}"))

    # B 停止，A 继续多写一些（未对齐的）
    for t in range(2010, 2501, 10):
        buf.add("WT_A", t, make_row("WT_A", f"2026-06-10 00:00:0{t//1000}.{t%1000:03d}"))

    aligned, unaligned = buf.shutdown_flush()

    # 对齐部分：A 的 [1500, 2000] (51行) + B 的 [1500, 2000] (51行) = 102 行
    assert "WT_A" in aligned
    assert "WT_B" in aligned
    assert len(aligned["WT_A"]) == 51
    assert len(aligned["WT_B"]) == 51

    # 未对齐部分：A 的 [2010, 2500] = 50 行（被丢弃）
    assert unaligned == 50

    # 缓冲已清空
    assert buf.buffered_rows == 0

    print("  PASS test_shutdown_flush")


# ============================================================
# 运行
# ============================================================

if __name__ == "__main__":
    print("=" * 60)
    print("SyncBuffer Unit Tests")
    print("=" * 60)

    test_parse_timestamp_ms()
    test_sync_disabled_immediate_write()
    test_single_device_buffers_when_waiting()
    test_dual_device_aligned_write()
    test_late_sensor_join()
    test_stale_cleanup()
    test_millis_rollover()
    test_device_goes_inactive()
    test_shutdown_flush()

    print()
    print("=" * 60)
    print("ALL TESTS PASSED")
    print("=" * 60)
