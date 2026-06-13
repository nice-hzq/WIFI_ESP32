# -*- coding: utf-8 -*-
"""
工作线程 — 数据采集 & 步态分析
Worker threads for data collection and gait analysis.

每个线程都有顶层 try/except 保护，确保异常时不会静默死亡导致 UI 卡死。
Critical messages (done/error) 使用阻塞 put 防止丢失。
"""

import os, csv, threading, queue, time, traceback

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

from . import DEFAULT_BAUD, SERIAL_RETRY_DELAY, CSV_HEADER


# ============================================================
# Helpers
# ============================================================
def sanitize_filename(name: str) -> str:
    safe = "".join(c if c.isalnum() or c in "_-" else "_" for c in name)
    return safe.strip("_") or "unknown_device"


# ============================================================
# Data Collection Thread
# ============================================================
class DataCollectionThread(threading.Thread):
    """Serial data collector — reads [DATA] CSV lines from ESP32 via COM port."""

    def __init__(self, serial_port, baud, output_dir, status_queue):
        super().__init__(daemon=True)
        self.serial_port = serial_port
        self.baud = baud
        self.output_dir = output_dir
        self.q = status_queue
        self._stop_event = threading.Event()

        # State (populated during run)
        self._ser = None
        self._writers = {}
        self._total_rows = 0

    def stop(self):
        self._stop_event.set()

    def _post(self, msg_type, **kwargs):
        """Non-critical message — may be dropped if queue is full."""
        try:
            self.q.put_nowait({"type": msg_type, **kwargs})
        except queue.Full:
            pass

    def _post_critical(self, msg_type, **kwargs):
        """Critical message (done/error) — MUST be delivered. Blocks up to 5s."""
        payload = {"type": msg_type, **kwargs}
        try:
            self.q.put(payload, timeout=5.0)
        except queue.Full:
            # Drain queue and retry
            try:
                while True:
                    self.q.get_nowait()
            except queue.Empty:
                pass
            try:
                self.q.put(payload, timeout=1.0)
            except queue.Full:
                pass  # Last resort: message lost, but UI has watchdog

    def _open_serial(self):
        try:
            return serial.Serial(self.serial_port, self.baud, timeout=1.0)
        except (serial.SerialException, OSError) as e:
            self._post("error", message=f"串口打开失败: {e}")
            return None

    def _cleanup(self):
        """Close serial port and all file handles. Always called, even on error."""
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
        for entry in self._writers.values():
            try:
                entry["file"].close()
            except Exception:
                pass
        self._writers.clear()

    # ---- main logic ----
    def _run_impl(self):
        os.makedirs(self.output_dir, exist_ok=True)
        self._post("status", state="connecting", folder=self.output_dir,
                   message=f"正在打开串口 {self.serial_port} @ {self.baud} ...")

        self._writers = {}
        self._ser = None
        buffer = b""
        self._total_rows = 0
        retry_count = 0

        while not self._stop_event.is_set():
            if self._ser is None:
                if retry_count > 0 and retry_count % 5 == 0:
                    ports = serial.tools.list_ports.comports()
                    port_list = ", ".join(p.device for p in ports)
                    self._post("error", message=f"可用端口: {port_list or '无'}")
                self._post("status", state="connecting", folder=self.output_dir,
                           message=f"正在打开串口 {self.serial_port} @ {self.baud} ...")
                self._ser = self._open_serial()
                if self._ser is None:
                    retry_count += 1
                    for _ in range(int(SERIAL_RETRY_DELAY * 10)):
                        if self._stop_event.is_set():
                            break
                        time.sleep(0.1)
                    continue
                buffer = b""
                retry_count = 0
                self._post("status", state="connected", folder=self.output_dir,
                           message=f"已连接 {self.serial_port} @ {self.baud} baud")

            try:
                if self._ser.in_waiting:
                    data = self._ser.read(self._ser.in_waiting)
                else:
                    data = self._ser.read(1)
                    if not data:
                        continue
            except (serial.SerialException, OSError) as e:
                self._post("status", state="disconnected", folder=self.output_dir,
                           message=f"串口断开: {e}")
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None
                continue

            buffer += data

            while b"\n" in buffer:
                line_bytes, buffer = buffer.split(b"\n", 1)
                line = line_bytes.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                if not line.startswith("[DATA]"):
                    continue
                csv_content = line[6:]
                if not csv_content:
                    continue

                row = csv_content.split(",")
                device_id = row[0].strip() if row else ""
                if not device_id:
                    continue

                if device_id not in self._writers:
                    safe_name = sanitize_filename(device_id)
                    filepath = os.path.join(self.output_dir, f"{safe_name}.csv")
                    file_exists = os.path.exists(filepath) and os.path.getsize(filepath) > 0
                    f = open(filepath, "a", newline="", encoding="utf-8")
                    writer = csv.writer(f)
                    if not file_exists:
                        writer.writerow(CSV_HEADER)
                        f.flush()
                    self._writers[device_id] = {"file": f, "writer": writer, "rows": 0, "path": filepath}

                self._writers[device_id]["writer"].writerow(row)
                self._writers[device_id]["rows"] += 1
                self._total_rows += 1

                if self._writers[device_id]["rows"] % 50 == 0:
                    self._writers[device_id]["file"].flush()

                if self._total_rows % 100 == 0:
                    per_dev = {did: e["rows"] for did, e in self._writers.items()}
                    self._post("data", total_rows=self._total_rows, devices=per_dev,
                               folder=self.output_dir)

    # ---- public API ----
    def run(self):
        try:
            self._run_impl()
        except Exception as e:
            self._post_critical("error",
                message=f"采集线程异常: {type(e).__name__}: {e}",
                traceback=traceback.format_exc())
        finally:
            self._post("status", state="stopping", folder=self.output_dir,
                       message="正在停止...")
            self._cleanup()
            per_dev = {did: e["rows"] for did, e in self._writers.items()}
            self._post_critical("done", total_rows=self._total_rows,
                devices=per_dev, folder=self.output_dir)


# ============================================================
# Gait Analysis Thread
# ============================================================
class GaitAnalysisThread(threading.Thread):
    """Runs the gait analysis pipeline on collected CSV data."""

    def __init__(self, data_folder, work_mode, status_queue, script_dir):
        super().__init__(daemon=True)
        self.data_folder = data_folder
        self.work_mode = work_mode
        self.q = status_queue
        self.script_dir = script_dir

    def _post(self, msg_type, **kwargs):
        try:
            self.q.put_nowait({"type": msg_type, **kwargs})
        except queue.Full:
            pass

    def _post_critical(self, msg_type, **kwargs):
        payload = {"type": msg_type, **kwargs}
        try:
            self.q.put(payload, timeout=5.0)
        except queue.Full:
            try:
                while True:
                    self.q.get_nowait()
            except queue.Empty:
                pass
            try:
                self.q.put(payload, timeout=1.0)
            except queue.Full:
                pass

    def _run_impl(self):
        import core.config as config
        from gait.gait_pipeline import run_gait_pipeline

        t0 = time.time()
        self._post("gait_status", state="running", message="正在加载数据并运行步态分析...")

        config.tempDir = os.path.join(self.script_dir, "Data", "temp")
        config.originalDir = self.data_folder
        config.curveDir = os.path.join(self.script_dir, "output")
        config.WORK_MODE = self.work_mode
        config.fs = 100
        os.makedirs(config.tempDir, exist_ok=True)
        os.makedirs(config.curveDir, exist_ok=True)

        import json
        result_json = run_gait_pipeline(fs=100, return_json=True)
        elapsed = time.time() - t0
        result_dict = json.loads(result_json) if isinstance(result_json, str) else result_json
        self._post_critical("gait_done", result=result_dict, elapsed=elapsed,
                           message=f"分析完成 ({elapsed:.1f}s)")

    def run(self):
        self._t0 = time.time()
        try:
            self._run_impl()
        except Exception as e:
            self._post_critical("gait_error",
                message=f"{type(e).__name__}: {e}",
                traceback=traceback.format_exc(),
                elapsed=time.time() - self._t0)
