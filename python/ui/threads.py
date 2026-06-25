# -*- coding: utf-8 -*-
"""
工作线程 — 数据采集 & 步态分析
Worker threads for data collection and gait analysis.

每个线程都有顶层 try/except 保护，确保异常时不会静默死亡导致 UI 卡死。
Critical messages (done/error) 使用阻塞 put 防止丢失。
"""

import os, csv, threading, queue, time, traceback
import numpy as np

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
        config.fs = 50
        os.makedirs(config.tempDir, exist_ok=True)
        os.makedirs(config.curveDir, exist_ok=True)

        import json
        result_json = run_gait_pipeline(fs=50, return_json=True)
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


# ============================================================
# Joint Angle Thread (real-time dual-IMU joint measurement)
# ============================================================
class JointAngleThread(threading.Thread):
    """
    实时关节角度测量线程。

    独立占用串口，运行 Mahony 解算 + 双 IMU 相对姿态 → 关节角度。
    与 DataCollectionThread 互斥（共用同一个 COM 口时不能同时运行）。

    设备识别:
      - 接受 device_map: {device_id: sensor_alias} 用于识别近端/远端传感器
      - device_map 为空时尝试从 calib_dir/device_alias_map.json 加载
      - 仍未匹配时按传感器上线顺序自动分配（第一个=近端，第二个=远端）
    """

    def __init__(self, serial_port, baud, joint_key, status_queue, calib_dir,
                 device_map=None, proximal_alias=None, distal_alias=None):
        super().__init__(daemon=True)
        self.serial_port = serial_port
        self.baud = baud
        self.joint_key = joint_key
        self.q = status_queue
        self.calib_dir = calib_dir
        self._stop_event = threading.Event()

        # 设备映射: {device_id: alias}  例: {"WT...121": "L4", "WT...131": "L5"}
        self.device_map = dict(device_map) if device_map else {}
        self.proximal_alias = proximal_alias or "L4"
        self.distal_alias = distal_alias or "L5"
        self.proximal_label = "大腿 (Thigh)"
        self.distal_label = "小腿 (Shank)"

        # Runtime
        self._ser = None
        self._engine = None
        self._calib = None
        self._buffer = b""
        self._joint_save_path = None        # 会话数据保存路径
        self._warned_uncalib = False        # 是否已提示需要校准
        # device_id → buffer (用于自动识别和标定)
        self._device_bufs: dict = {}     # {device_id: [imu9_frames]}
        self._device_detected: list = []  # 按上线顺序记录的 device_id 列表
        self._prox_id: str = None        # 识别后的近端 device_id
        self._dist_id: str = None        # 识别后的远端 device_id

    def stop(self):
        self._stop_event.set()

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

    def _try_load_device_map(self):
        """尝试从 calib_dir/device_alias_map.json 加载设备映射"""
        map_path = os.path.join(self.calib_dir, "device_alias_map.json")
        if os.path.exists(map_path):
            try:
                import json
                with open(map_path, "r", encoding="utf-8") as f:
                    loaded = json.load(f)
                if isinstance(loaded, dict):
                    self.device_map.update(loaded)
                    print(f"[JOINT-THREAD] 已加载设备映射: {self.device_map}")
            except Exception as e:
                print(f"[JOINT-THREAD] 加载设备映射失败: {e}")

    def _resolve_device(self, device_id: str) -> str:
        """根据 device_id 返回传感器别名；若未映射则返回 None"""
        return self.device_map.get(device_id, None)

    def _identify_sensors(self):
        """
        尝试识别近端和远端传感器。
        优先使用 device_map；若映射不足则按上线顺序自动分配。
        """
        if self._prox_id and self._dist_id:
            return  # 已识别

        # 方法1：通过 device_map 匹配
        prox_candidates = [did for did, alias in self.device_map.items()
                          if alias == self.proximal_alias and did in self._device_bufs]
        dist_candidates = [did for did, alias in self.device_map.items()
                          if alias == self.distal_alias and did in self._device_bufs]

        if prox_candidates and dist_candidates:
            self._prox_id = prox_candidates[0]
            self._dist_id = dist_candidates[0]
            self._post("joint_status", state="identified",
                       message=f"传感器已识别: {self.proximal_alias}={self._prox_id}, "
                               f"{self.distal_alias}={self._dist_id}",
                       proximal_id=self._prox_id, distal_id=self._dist_id,
                       proximal_alias=self.proximal_alias, distal_alias=self.distal_alias)
            return

        # 方法2：按上线顺序自动分配（前两个设备）
        if len(self._device_detected) >= 2:
            self._prox_id = self._device_detected[0]
            self._dist_id = self._device_detected[1]
            self._post("joint_status", state="identified",
                       message=f"自动分配: {self.proximal_alias}={self._prox_id}, "
                               f"{self.distal_alias}={self._dist_id} "
                               f"(未找到 device_alias_map.json，按上线顺序)",
                       proximal_id=self._prox_id, distal_id=self._dist_id,
                       proximal_alias=self.proximal_alias, distal_alias=self.distal_alias)
            return

    def _get_engine_inputs(self):
        """从设备缓冲区取最近一帧近端和远端 IMU 数据"""
        if not self._prox_id or not self._dist_id:
            return None, None
        if self._prox_id not in self._device_bufs or self._dist_id not in self._device_bufs:
            return None, None
        if len(self._device_bufs[self._prox_id]) == 0 or len(self._device_bufs[self._dist_id]) == 0:
            return None, None
        # 取各自最新帧
        imu_p = self._device_bufs[self._prox_id][-1]
        imu_d = self._device_bufs[self._dist_id][-1]
        return imu_p, imu_d

    # ---- Calibration command (called via queue from UI) ----
    def start_calibration(self, calib_mode="lower_body_standing"):
        self._pending_calib_mode = calib_mode
        self._pending_calib = True

    # ---- Main ----
    def _run_impl(self):
        from joint import JointAngleEngine, JointCalibration, JOINT_DEFS
        from joint.joint_calibration import save_calibration
        from joint.joint_angle import online_calibrate_from_buffers
        from ui import JOINT_OPTIONS
        import json

        # 尝试加载设备映射
        self._try_load_device_map()

        # ---- 初始化引擎 ----
        info = JOINT_OPTIONS[self.joint_key]
        self.proximal_alias = info["proximal"]
        self.distal_alias = info["distal"]
        self.proximal_label = info.get("prox_label", self.proximal_alias)
        self.distal_label = info.get("dist_label", self.distal_alias)
        binding_info = {
            "joint_name": self.joint_key,
            "proximal_sensor": self.proximal_alias,
            "distal_sensor": self.distal_alias,
            "flexion_axis": 0,
        }
        from joint.joint_models import JointBinding
        binding = JointBinding(**binding_info)
        engine = JointAngleEngine(binding, fs=50.0)
        self._engine = engine

        # 每次测量都需要重新校准，不自动加载旧标定文件
        self._post("joint_status", state="uncalibrated",
                   message="未标定 — 引擎就绪后请保持静止站立并点击「开始校准」")

        # ---- 打开串口 ----
        self._post("joint_status", state="connecting",
                   message=f"打开 {self.serial_port}...")
        try:
            import serial as _serial
            self._ser = _serial.Serial(self.serial_port, self.baud, timeout=1.0)
        except Exception as e:
            self._post_critical("joint_error", message=f"串口打开失败: {e}")
            return

        self._post("joint_status", state="connected",
                   message=f"已连接 {self.serial_port}，等待传感器数据...")

        # ---- 主循环 ----
        self._pending_calib = False
        self._pending_calib_mode = "lower_body_standing"
        last_device_report = time.time()

        while not self._stop_event.is_set():
            try:
                if self._ser.in_waiting:
                    data = self._ser.read(self._ser.in_waiting)
                else:
                    data = self._ser.read(1)
                    if not data:
                        continue
            except Exception as e:
                self._post("joint_status", state="disconnected", message=f"串口断开: {e}")
                break

            self._buffer += data
            while b"\n" in self._buffer:
                line_bytes, self._buffer = self._buffer.split(b"\n", 1)
                line = line_bytes.decode("utf-8", errors="replace").strip()
                if not line.startswith("[DATA]"):
                    continue
                csv_content = line[6:]
                if not csv_content:
                    continue
                row = csv_content.split(",")
                if len(row) < 14:
                    continue
                device_id = row[0].strip()

                # 提取 IMU9
                try:
                    imu9 = np.array([float(row[i]) for i in range(2, 2+9)], dtype=float)
                except (ValueError, IndexError):
                    continue

                # 记录设备上线
                if device_id not in self._device_bufs:
                    self._device_bufs[device_id] = []
                    self._device_detected.append(device_id)
                    alias = self._resolve_device(device_id) or "?"
                    self._post("joint_status", state="device_detected",
                               message=f"检测到设备: {device_id} → {alias}",
                               device_id=device_id, alias=alias)
                self._device_bufs[device_id].append(imu9)
                # 限制缓冲大小
                if len(self._device_bufs[device_id]) > 500:
                    self._device_bufs[device_id] = self._device_bufs[device_id][-500:]

                # 尝试识别传感器
                self._identify_sensors()

                # 如果已识别，执行角度计算（始终更新引擎，但仅标定后才发送曲线数据）
                if self._prox_id and self._dist_id:
                    imu_p, imu_d = self._get_engine_inputs()
                    if imu_p is not None and imu_d is not None:
                        engine.update(imu_p, imu_d)
                        state = engine.get_state()
                        # 未标定时不输出曲线，避免显示无意义的数值
                        if not engine.has_calibration():
                            if engine.is_ready() and not getattr(self, '_warned_uncalib', False):
                                self._warned_uncalib = True
                                self._post("joint_status", state="waiting_calib",
                                           message="引擎已就绪，请保持静止站立后点击「开始校准」")
                            continue
                        # 每 5 帧发一次（50Hz→10Hz），大幅减少 queue 流量
                        frame_count = getattr(self, '_angle_frame_count', 0) + 1
                        self._angle_frame_count = frame_count
                        if frame_count % 5 == 0:
                            # 降采样: 50Hz 存储 → 每 5 帧取 1 (10Hz)，取最近 600 点 = 60 秒
                            step = 5
                            t_raw = state.history_t
                            f_raw = state.history_flexion
                            a_raw = state.history_abduction
                            r_raw = state.history_rotation
                            n_ds = min(600, len(t_raw) // step)
                            self._post("joint_angle",
                                       joint=self.joint_key,
                                       flexion_deg=round(state.flexion_deg, 2),
                                       abduction_deg=round(state.abduction_deg, 2),
                                       rotation_deg=round(state.rotation_deg, 2),
                                       max_deg=round(state.max_flexion_deg, 2),
                                       min_deg=round(state.min_flexion_deg, 2),
                                       rom_deg=round(state.rom_deg, 2),
                                       history_t=t_raw[-n_ds * step::step],
                                       history_flexion=f_raw[-n_ds * step::step],
                                       history_abduction=a_raw[-n_ds * step::step],
                                       history_rotation=r_raw[-n_ds * step::step],
                                       initialized=engine.is_ready(),
                                       calibrated=engine.has_calibration())

                # 每 3 秒报告一次设备状态
                now = time.time()
                if now - last_device_report > 3.0:
                    last_device_report = now
                    if self._device_detected:
                        dev_info = []
                        for did in self._device_detected:
                            alias = self._resolve_device(did) or "?"
                            buf_n = len(self._device_bufs.get(did, []))
                            role = ""
                            if did == self._prox_id:
                                role = f"→{self.proximal_alias}({self.proximal_label})"
                            elif did == self._dist_id:
                                role = f"→{self.distal_alias}({self.distal_label})"
                            dev_info.append(f"{did}({alias}){role} buf={buf_n}")
                        self._post("joint_status", state="devices",
                                   message=" | ".join(dev_info))
                    else:
                        self._post("joint_status", state="waiting",
                                   message="等待传感器数据...")

                # ---- 处理校准请求 ----
                if self._pending_calib and engine.is_ready():
                    self._pending_calib = False
                    mode = self._pending_calib_mode
                    # 检查缓冲区大小
                    rp, rd = engine.get_rolling_buffer_sizes()
                    need = int(round(3.0 * 50))
                    if rp < need or rd < need:
                        self._post("joint_calib_error",
                                   message=f"缓冲数据不足: {self.proximal_label}={rp} {self.distal_label}={rd} 帧, "
                                           f"需要各 {need} 帧 (3秒)。请等待数据积累。")
                        continue
                    self._post("joint_calib_status", state="calibrating",
                               message=f"正在标定 ({mode})，请保持静止...")
                    try:
                        calib = online_calibrate_from_buffers(engine, calib_mode=mode)
                        save_calibration(calib, self.calib_dir)
                        engine.load_calibration(calib)
                        self._calib = calib
                        self._post("joint_calib_done",
                                   joint=self.joint_key,
                                   message=f"标定完成: {mode}")
                    except Exception as e:
                        self._post("joint_calib_error",
                                   message=f"标定失败: {e}")

        # ---- 保存数据 ----
        self._joint_save_path = self._save_joint_data()

    def _save_joint_data(self):
        """保存当前会话的关节角度数据到 joint_csv/ 带时间戳的文件夹。

        文件夹结构:
          joint_csv/joint_angle_YYYYMMDD_HHMMSS/
            joint_angle_data.csv
            session_info.json

        Returns:
            str: 保存目录路径，若无数据则返回 None
        """
        try:
            import json as _json
            from datetime import datetime as _datetime

            engine = getattr(self, '_engine', None)
            if engine is None or not engine.is_ready():
                return None

            state = engine.get_state()
            n = len(state.history_t) if state.history_t else 0
            if n < 2:
                return None

            # 确保三个列表长度一致
            n = min(n, len(state.history_flexion),
                    len(state.history_abduction), len(state.history_rotation))
            if n < 2:
                return None

            # 创建输出目录: joint_csv/joint_angle_YYYYMMDD_HHMMSS/
            ts = _datetime.now().strftime("%Y%m%d_%H%M%S")
            project_root = os.path.dirname(os.path.dirname(self.calib_dir))
            session_dir = os.path.join(project_root, "joint_csv", f"joint_angle_{ts}")
            os.makedirs(session_dir, exist_ok=True)

            # ---- 保存角度 CSV ----
            csv_path = os.path.join(session_dir, "joint_angle_data.csv")
            with open(csv_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["time_s", "flexion_deg", "abduction_deg", "rotation_deg"])
                t0 = state.history_t[0]
                for i in range(n):
                    t_rel = state.history_t[i] - t0
                    writer.writerow([
                        f"{t_rel:.3f}",
                        f"{state.history_flexion[i]:.4f}",
                        f"{state.history_abduction[i]:.4f}",
                        f"{state.history_rotation[i]:.4f}",
                    ])

            # ---- 保存会话信息 JSON ----
            session_info = {
                "joint_key": self.joint_key,
                "proximal_sensor": self.proximal_alias,
                "distal_sensor": self.distal_alias,
                "proximal_label": getattr(self, 'proximal_label', self.proximal_alias),
                "distal_label": getattr(self, 'distal_label', self.distal_alias),
                "proximal_device_id": self._prox_id or "",
                "distal_device_id": self._dist_id or "",
                "calibrated": engine.has_calibration(),
                "calibration_mode": self._calib.calibration_mode if self._calib else "",
                "sample_count": n,
                "duration_s": round(state.history_t[-1] - state.history_t[0], 2) if n >= 2 else 0,
                "fs": int(engine.fs),
                "created_time": _datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "flexion_rom_deg": round(state.rom_deg, 2),
                "flexion_max_deg": round(state.max_flexion_deg, 2),
                "flexion_min_deg": round(state.min_flexion_deg, 2),
            }
            json_path = os.path.join(session_dir, "session_info.json")
            with open(json_path, "w", encoding="utf-8") as f:
                _json.dump(session_info, f, ensure_ascii=False, indent=2)

            print(f"[JOINT-THREAD] 数据已保存: {csv_path} ({n} 帧)")
            return session_dir

        except Exception as e:
            print(f"[JOINT-THREAD] 保存数据失败: {type(e).__name__}: {e}")
            import traceback as _tb
            _tb.print_exc()
            return None

    def run(self):
        try:
            self._run_impl()
        except Exception as e:
            self._post_critical("joint_error",
                                message=f"线程异常: {type(e).__name__}: {e}",
                                traceback=traceback.format_exc())
        finally:
            if self._ser:
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None
            self._post("joint_status", state="disconnected", message="关节角度测量已停止")
