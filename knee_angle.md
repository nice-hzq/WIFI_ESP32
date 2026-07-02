# Real-time Knee Angle Measurement Flow

> **检查日期**: 2026-06-30
> **修改日期**: 2026-06-30（已实施 11 项改进，详见第 13 节）
> **检查范围**: 全项目代码扫描，重点关注实时膝关节角度测量完整流程
> **检查结论**: 核心算法（四元数相对姿态）是正确的。高/中优先级问题已修复，低优先级问题已文档化。

---

## 1. 功能目标

实时测量膝关节角度（knee flexion/extension angle）。主要依赖大腿 IMU（proximal, L4/R4）和小腿 IMU（distal, L5/R5）的相对姿态，通过四元数相对旋转计算，而不是简单的欧拉角相减。

---

## 2. 相关文件和函数

### 2.1 ESP32 固件层 (C++)

| 文件 | 关键函数/结构 | 作用 |
|------|-------------|------|
| `src/main.cpp` | `handleUdp()`, `outputCSVRow()` | 接收 WT901WIFI UDP 数据，解析帧，附加 `esp32RxMs` 时间戳，通过 TCP/Serial 输出 CSV 行 |
| `src/wt901_parser.h` | `WT901Data` 结构体 | 定义完整 IMU 帧数据结构：deviceId, 时间戳, acc(3), gyro(3), mag(3), angle(3) |
| `src/wt901_parser.cpp` | `parseWT901Frame()` | 解析 54 字节 WT901WIFI 帧，提取全部数据字段 |
| `src/session_manager.h` | `SensorSession` 结构体 | 按源 IP 维护独立帧组装状态机 |
| `src/config.h` | 宏定义 | AP/端口/传感器数量等编译期配置 |

### 2.2 Python 上位机层

| 文件 | 关键类/函数 | 作用 |
|------|-----------|------|
| `python/ui/threads.py` | `JointAngleThread` | **实时关节角度测量线程**：读取串口 `[DATA]` 行，提取 imu9，缓冲并计算膝关节角度 |
| `python/joint/joint_angle.py` | `JointAngleEngine` | **核心引擎**：管理两个 Mahony 节点，执行 `q_knee = inv(q_thigh) * q_shank` 计算 |
| `python/joint/joint_models.py` | `JointBinding`, `JointCalibration`, `JointAngleState` | 数据模型：关节-传感器绑定、标定结果、角度状态 |
| `python/joint/joint_calibration.py` | `calibrate_joint_from_arrays()` | 静止站立标定：计算 `q_rel_0 = inv(q_prox_avg) * q_dist_avg` |
| `python/orientation/quaternion_manager.py` | `MahonyOrientationNode` | 单传感器 Mahony 滤波器节点，在线/批量姿态解算 |
| `python/core/quaternion.py` | `quat_inv()`, `quat_mul()`, `quat_to_euler()` | 四元数运算法库 (w,x,y,z 约定) |
| `python/ui/__init__.py` | `JOINT_OPTIONS`, `CALIB_MODES` | 关节角度常量：传感器别名映射、校准模式定义 |
| `python/ui/app.py` | `GaitAnalysisApp` | 桌面 UI 主类：关节角度测量卡片、曲线显示、校准控制 |
| `python/ui/dialogs.py` | `open_joint_device_mapping_dialog()` | 关节设备 ID → 传感器别名映射配置对话框 |

### 2.3 离线同步脚本（作为对比参考）

| 文件 | 作用 |
|------|------|
| `sync_multi_imu_csv.py` | 离线多传感器 CSV 同步：以 `esp32_rx_ms` 为基准，线性插值到统一时间轴 |
| `sync_imu_csv.py` | 单传感器 CSV 时间排序和清理 |

---

## 3. 实时数据流

```
┌──────────────────────┐
│ WT901WIFI Sensor 1   │  大腿 (L4/R4)，100 Hz，WiFi UDP → ESP32
│ WT901WIFI Sensor 2   │  小腿 (L5/R5)，100 Hz，WiFi UDP → ESP32
└──────────┬───────────┘
           │ UDP :1399
           ▼
┌──────────────────────┐
│ ESP32 Gateway        │  WiFi AP 模式 (192.168.10.1)
│                      │  - 按源 IP 维护独立帧组装状态机
│                      │  - parseWT901Frame() 解析 54 字节帧
│                      │  - 附加 esp32RxMs = millis()
│                      │  - 输出 CSV 行: [DATA]device_id,...,esp32_rx_ms,...
│                      │    ├── TCP :8888（主通道）
│                      │    └── Serial :921600（带 [DATA] 前缀）
└──────────┬───────────┘
           │ Serial (COM port)
           ▼
┌──────────────────────┐
│ Python上位机          │
│ JointAngleThread      │
│                      │
│ 1. 读取串口 [DATA] 行  │
│ 2. 解析 device_id +   │
│    imu9 (acc,gyr,ang) │
│ 3. 按 device_id 缓冲  │
│ 4. 识别 prox/dist     │
│ 5. 取最新一帧          │  ★ 当前无时间同步
│ 6. Mahony 在线更新    │
│ 7. 计算膝关节角度      │
│    q_knee = inv(q_p)  │
│           * q_d       │
│ 8. 10Hz 发送到 UI     │
│ 9. 实时画曲线          │
└──────────┬───────────┘
           │
           ▼
┌──────────────────────┐
│ 数据保存              │
│ joint_csv/            │
│  joint_angle_*/       │
│    joint_angle_data.csv│
│    session_info.json  │
└──────────────────────┘
```

### 详细步骤

1. **WT901WIFI 传感器** — 两个传感器分别固定在大腿和小腿，通过 WiFi 连接 ESP32 AP，UDP 端口 1399 发送原始 54 字节帧，默认 100 Hz
2. **ESP32 UDP 接收** — `handleUdp()` 批量排空所有积压 UDP 包，按源 IP 维护独立帧组装状态机 (`SensorSession`)
3. **帧解析** — `parseWT901Frame()` 解析全部字段：device_id, 时间戳, acc(3), gyro(3), angle(3), mag(3) 等
4. **时间戳附加** — ESP32 在帧解析完成时刻记录 `esp32RxMs = millis()`，不覆盖 WT901 原始时间
5. **CSV 输出** — `outputCSVRow()` 格式化 16 列 CSV 行，通过 TCP (:8888) 和 Serial (`[DATA]` 前缀) 发送
6. **上位机串口接收** — `JointAngleThread` 通过 Python `pyserial` 读取串口，按 `\n` 分割，过滤 `[DATA]` 行
7. **数据提取** — 从 CSV 行提取 `device_id`（col 0）和 imu9（col 4-12：acc_x/y/z, gyro_x/y/z, angle_x/y/z）
8. **设备缓冲** — 按 device_id 各自维护环形缓冲区（最多 500 帧），记录设备上线顺序
9. **传感器识别** — 优先通过 `device_alias_map.json` 匹配设备 ID→别名；否则按上线顺序自动分配（第1个=近端，第2个=远端）
10. **取最新帧** — `_get_engine_inputs()` 取各设备缓冲区最新一帧（`buffer[-1]`），**无时间同步**
11. **Mahony 解算** — 每个传感器独立运行 Mahony 滤波器 (`use_mag=False`, kp=0.5, ki=0.1)，分别输出四元数
12. **膝关节角度** — `q_rel_t = inv(q_prox) * q_dist`，标定补偿后 `q_joint = inv(q_rel_0) * q_rel_t`，提取 roll=屈伸角
13. **降采样发送** — 每 5 帧发一次（50 Hz → 10 Hz），包含 flexion/abduction/rotation 当前值和历史曲线数据
14. **UI 显示** — 实时更新数值卡片 + 三面板曲线（屈伸/外展内收/旋转）
15. **会话保存** — 停止测量时自动保存到 `joint_csv/joint_angle_YYYYMMDD_HHMMSS/`

---

## 4. 输入数据格式

### 4.1 ESP32 输出的 CSV 行（16 列）

```csv
device_id,sensor_timestamp,sensor_ms,esp32_rx_ms,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,angle_x,angle_y,angle_z,mag_x,mag_y,mag_z
```

| 列 | 字段 | 单位 | 说明 |
|----|------|------|------|
| 0 | `device_id` | — | 传感器唯一标识，如 `WT5500010121` |
| 1 | `sensor_timestamp` | — | WT901 原始时间字符串 `20YY-MM-DD HH:MM:SS.mmm` |
| 2 | `sensor_ms` | ms | WT901 时间换算为当天毫秒数（用于检查采样稳定性） |
| 3 | `esp32_rx_ms` | ms | ESP32 `millis()` 接收时间（用于多传感器统一时间轴对齐） |
| 4-6 | `acc_x, acc_y, acc_z` | g | 加速度，量程 ±16g |
| 7-9 | `gyro_x, gyro_y, gyro_z` | °/s | 角速度，量程 ±2000°/s |
| 10-12 | `angle_x, angle_y, angle_z` | ° | WT901 内部解算角度（实时路径中未使用） |
| 13-15 | `mag_x, mag_y, mag_z` | 原始值 | 磁场强度 |

### 4.2 实时关节角度线程提取的数据

`JointAngleThread._run_impl()` 中提取的是：

```python
device_id = row[0].strip()
imu9 = row[4:13]  # [acc(3), gyr(3), angle(3)]
```

**⚠️ 注意**: `imu9` 的第 6-8 索引（col 10-12）是 `angle_x/y/z`，而不是 `mag_x/y/z`。但由于 `MahonyOrientationNode.use_mag=False`，磁力计数据不会被使用，所以这不影响当前计算正确性。若未来启用磁力计 (`use_mag=True`)，需要修改为 `row[4:7] + row[10:13]`（即 acc + gyr + mag 而非 angle）。

### 4.3 实时路径未使用的时间戳字段

当前实时路径中，**`sensor_ms`（col 2）和 `esp32_rx_ms`（col 3）均未被提取或使用**。这两个字段仅在离线 CSV 同步脚本 `sync_multi_imu_csv.py` 中被使用。这是实时路径的一个关键局限——没有利用时间戳来做帧同步。

---

## 5. 大腿和小腿传感器映射

### 5.1 当前实现

关节定义在 `python/ui/__init__.py` → `JOINT_OPTIONS` 和 `python/joint/joint_models.py` → `JOINT_DEFS`：

| 关节 | 近端传感器 (proximal) | 远端传感器 (distal) | 近端标签 | 远端标签 |
|------|---------------------|---------------------|---------|---------|
| `left_knee` | L4 | L5 | 大腿 (Thigh) | 小腿 (Shank) |
| `right_knee` | R4 | R5 | 大腿 (Thigh) | 小腿 (Shank) |
| `left_ankle` | L5 | L6 | 小腿 (Shank) | 脚掌 (Foot) |
| `right_ankle` | R5 | R6 | 小腿 (Shank) | 脚掌 (Foot) |
| `left_hip` | S1 | L4 | 骨盆 (Pelvis) | 大腿 (Thigh) |
| `right_hip` | S1 | R4 | 骨盆 (Pelvis) | 大腿 (Thigh) |

### 5.2 设备 ID → 传感器别名映射

当前支持三种识别方式（`JointAngleThread._identify_sensors()`）：

1. **device_alias_map.json** — 用户通过 UI 对话框配置 `{device_id: alias}` 映射（推荐）
2. **自动分配** — 按传感器上线顺序：第 1 个 = 近端，第 2 个 = 远端（无映射时的 fallback）
3. **代码硬编码** — `proximal_alias` 和 `distal_alias` 默认值（`"L4"` / `"L5"`）

### 5.3 ⚠️ 当前缺失

- **没有显式的 `THIGH_DEVICE_ID` / `SHANK_DEVICE_ID` 配置项**。当前依赖 `device_alias_map.json` 间接映射，但该文件的 key 是传感器别名（L4/L5），不是明确的大腿/小腿标识。
- **建议**：在 UI 或配置文件中增加明确的"大腿传感器设备 ID"和"小腿传感器设备 ID"配置项，避免用户误将两个传感器装反。

---

## 6. 实时同步策略

### 6.1 当前状态：❌ 无实时帧同步

当前 `JointAngleThread._get_engine_inputs()` 的实现非常简单：

```python
def _get_engine_inputs(self):
    imu_p = self._device_bufs[self._prox_id][-1]   # 近端最新帧
    imu_d = self._device_bufs[self._dist_id][-1]   # 远端最新帧
    return imu_p, imu_d
```

**仅取各自缓冲区的最新一帧**，不检查两个传感器帧的时间差。这意味着：

- 如果大腿传感器回传率 100 Hz，小腿传感器回传率也是 100 Hz，但它们到达 ESP32 的时刻可能不同步
- 即使两个传感器通过同一个 ESP32 AP 连接，UDP 包到达顺序、WiFi 抖动都可能导致两帧来自不同时刻
- 极端情况下，某个传感器暂时卡顿，另一传感器持续更新，计算出的角度可能对应不同的身体姿态

### 6.2 离线同步的对比

离线 CSV 同步脚本 `sync_multi_imu_csv.py` 做了完整的时间同步：

1. 以 `esp32_rx_ms` 为统一时间基准
2. 找到所有传感器共同重叠的时间范围
3. 生成统一时间轴（默认 100 Hz）
4. 对 acc/gyro/angle/mag 分别线性插值

### 6.3 建议：加入实时近邻匹配

推荐的实时同步方案（建议后续实现）：

```
方案 A（低成本近邻匹配）：
  1. 缓存每个传感器最近 N 帧数据（含 esp32_rx_ms）
  2. 当新帧到达远端传感器时：
     a. 在近端传感器缓存中找到 esp32_rx_ms 最接近的一帧
     b. 检查时间差 Δt = |esp32_rx_ms_prox - esp32_rx_ms_dist|
     c. 如果 Δt <= 最大允许时间差（推荐 30 ms），使用该匹配对
     d. 如果 Δt > 30 ms，丢弃并等待（或告警）

方案 B（插值）：
  1. 维护两个传感器的时间序列
  2. 当远端新帧到达时，在近端序列的两帧之间线性插值得到"同时刻"IMU数据
  3. 再喂入姿态解算
```

**推荐最大允许时间差**：
- WT901 回传率 100 Hz → 帧间隔 10 ms
- 允许 1-2 帧抖动 → **20-30 ms**
- 超过此值大概率是不同的身体姿态帧，不宜直接配对

### 6.4 当前流程的局限

- ✅ 优点：实现简单，延迟低（每个传感器取最新即可，无需等待）
- ❌ 缺点：两个传感器的帧可能不同步，导致角度计算不准确
- ❌ 缺点：没有异常数据检测（如某传感器突然丢帧 500 ms）
- ❌ 缺点：`esp32_rx_ms` 实时路径中完全未使用

---

## 7. 姿态解算方法

### 7.1 当前实现：Mahony 互补滤波器（重新解算）

**不使用 WT901 自带角度**。系统通过 acc/gyro 重新解算姿态：

```python
# joint_angle.py — JointAngleEngine.__init__()
self._node_prox = MahonyOrientationNode(
    name=f"{binding.joint_name}_prox",
    fs=self.fs,           # 50 Hz
    use_mag=False,         # 不使用磁力计
    acc_unit="g",
    gyr_unit="deg",
    kp=0.5,                # 比例增益
    ki=0.1,                # 积分增益（零偏在线估计）
)
```

Mahony 参数：
| 参数 | 值 | 说明 |
|------|----|------|
| 算法 | Mahony 互补滤波器 (AHRS 库) | acc 修正姿态，gyro 积分传播 |
| `kp` | 0.5 | 比例增益 — 控制 acc 修正强度 |
| `ki` | 0.1 | 积分增益 — 在线估计陀螺零偏 |
| `use_mag` | False | 不使用磁力计（避免室内磁干扰） |
| 四元数格式 | wxyz | 与 ahrs 库一致 |

### 7.2 dt 使用分析

- **Mahony 滤波器 dt**: 通过 `Mahony(frequency=self.fs, ...)` 设置，`fs=50` → `dt = 1/50 = 0.02 s`
- **实际 WT901 回传率**: 通常为 100 Hz，`dt = 0.01 s`
- **实时 update 调用频率**: 每来一帧即调用一次 `update_one()`，实际 dt 取决于数据到达率

**⚠️ dt 不匹配问题**：

| 层面 | dt 设置 | 说明 |
|------|---------|------|
| Mahony 滤波器内部 | `dt = 1/fs = 0.02 s` | 硬编码为 50 Hz |
| WT901 实际回传率 | `dt ≈ 0.01 s` | 100 Hz |
| 实时 update 间隔 | 约 10 ms（100 Hz） | 每收到一帧调一次 |

**结论**：
- 当前 Mahony 滤波器内部使用 `dt = 0.02 s`，与实际数据到达率（~10 ms）不一致
- 姿态解算的积分 dt 应该与实际数据采样率匹配。WT901 回传 100 Hz 时，建议 `fs=100`（dt=0.01s）
- **`esp32_rx_ms` 相邻差值不应作为积分 dt**（抖动较大），应使用固定 dt
- 推荐：**实时姿态解算使用固定 `dt = 0.01 s`（对应 100 Hz）**，`esp32_rx_ms` 仅用于多传感器时间对齐
- `sensor_ms` 用于检查单个 WT901 自身采样是否稳定、是否丢帧或跳变

### 7.3 初始化流程

引擎需要收集 200 帧（~4 秒 @ 50 Hz, 或 ~2 秒 @ 100 Hz）后自动初始化：
- 使用前 400 帧（取前 200 帧偏移后的 400 帧）做静止段初始化
- 计算初始姿态四元数（acc 确定俯仰/横滚）
- 估计陀螺零偏
- 初始化完成后才开始在线姿态更新

---

## 8. 膝关节角度计算方法 ✅ 正确

### 8.1 核心公式（已验证）

`python/joint/joint_angle.py` → `JointAngleEngine._compute_all_angles()`:

```python
def _compute_all_angles(self, q_prox: np.ndarray, q_dist: np.ndarray):
    # Step 1: 当前相对姿态
    q_rel_t = quat_mul(quat_inv(q_prox), q_dist)

    # Step 2: 标定补偿
    if self.calibration is not None:
        q_rel_0 = np.array(self.calibration.q_rel_0, dtype=float)
        q_joint = quat_mul(quat_inv(q_rel_0), q_rel_t)
    else:
        q_joint = q_rel_t

    # Step 3: 提取欧拉角
    q_joint = quat_normalize(q_joint)
    euler_deg = quat_to_euler(q_joint, degrees=True)  # [roll, pitch, yaw]

    return float(euler_deg[0]), float(euler_deg[1]), float(euler_deg[2])
```

这完全符合推荐的膝关节角度计算方法：

```
q_knee  = inv(q_thigh) * q_shank          ← 核心公式 ✓
q_joint = inv(q_rel_0) * q_knee           ← 标定补偿 ✓
flexion = euler_from_quat(q_joint)[0]     ← 提取屈伸角 ✓
```

### 8.2 欧拉角提取

`quat_to_euler()` 在 `python/core/quaternion.py` 中使用标准 ZYX 内旋顺序：

```python
roll  = arctan2(2*(w*x + y*z), 1 - 2*(x² + y²))
pitch = arcsin(2*(w*y - z*x))
yaw   = arctan2(2*(w*z + x*y), 1 - 2*(y² + z²))
```

- `flexion_axis = 0`（roll = 屈伸）✓
- `abduction = pitch`（外展内收）
- `rotation = yaw`（内旋外旋）

### 8.3 ✅ 不是简单欧拉角相减

当前代码**没有使用**以下简化方法：
```python
# ❌ 简化方法（当前未使用）:
knee_angle = shank_angle_x - thigh_angle_x
```

而是使用了正确的四元数相对姿态方法。这种方法避免了：
- 欧拉角顺序（gimbal lock）问题
- 安装方向问题（两个传感器坐标轴不一致时简单相减会出错）
- yaw 漂移对屈伸角的影响

### 8.4 标定补偿

标定时计算初始相对姿态，运行时减去：

```python
# joint_calibration.py
q_rel_0 = quat_mul(quat_inv(q_proximal_avg), q_distal_avg)

# joint_angle.py 运行时
q_joint = quat_mul(quat_inv(q_rel_0), q_rel_t)
```

这等价于：站立时的膝关节角度 = 0°（零点校准），之后的测量值都是相对于站立姿态的相对角度。

---

## 9. 零点校准和安装方向

### 9.1 已有功能 ✅

| 功能 | 实现 | 说明 |
|------|------|------|
| 静止检测 | `_check_static()` | 陀螺 std 和加速度 std 阈值检测 |
| 站立校准 | `lower_body_standing` 模式 | 用户站立静止 3 秒，记录姿态 |
| q_rel_0 计算 | `inv(q_prox_avg) * q_dist_avg` | 站立姿态作为零点参考 |
| 标定保存 | `save_calibration()` → JSON | 持久化到 `temp/<joint>_pose_calib.json` |
| 静止失败提示 | RuntimeError + 中文消息 | 告诉用户传感器未静止 |

### 9.2 当前缺失 ⚠️

| 缺失项 | 说明 | 建议 |
|--------|------|------|
| 传感器安装方向说明 | 用户不知道传感器 XYZ 轴应该朝向哪个方向 | 添加图示，说明 X/Y/Z 轴与人体坐标系的对应关系 |
| 屈伸正负方向约定 | 代码中 roll 为正表示什么？（屈膝 or 伸膝？） | 文档说明：正值 = 屈膝, 负值 = 伸膝（或反之） |
| 坐标轴对齐验证 | 安装后无法验证两个传感器坐标轴是否大致对齐 | 可在校准后显示 q_rel_0 的 roll/pitch/yaw 分量，提示用户是否合理 |
| 自动加载旧标定 | 代码中已禁用了自动加载旧标定（commit dc3141f），每次测量都需要重新校准 | 合理的设计选择，但增加了用户操作步骤 |

---

## 10. 当前存在的问题

### 10.1 实时同步缺失 ❌ （重要）

- 大腿和小腿传感器帧没有做时间对齐
- `esp32_rx_ms` 在实时路径中完全未提取和使用
- `sensor_ms` 也未使用，无法检测单传感器丢帧
- 可能导致不同步的帧被配对计算错误的角度

### 10.2 imu9 字段映射问题 ⚠️

- CSV 行的 col 10-12 是 `angle_x/y/z`（WT901 内部解算角度）
- 代码将其传入 `MahonyOrientationNode.update_one()` 的第 3 个参数（即 mag 位置）
- 由于 `use_mag=False`，当前不影响计算正确性
- 但若未来启用磁力计，会导致使用错误的数据源
- 字段命名和文档注释也可能误导开发者

### 10.3 Mahony dt 与 WT901 回传率不匹配 ⚠️

- Mahony 滤波器配置 `fs=50`（dt=0.02s）
- WT901 实际回传率约 100 Hz（帧间隔 ~10 ms）
- dt 偏大会导致积分步长不准确，影响动态精度

### 10.4 自动初始化需要约 2-4 秒的静止段 ⚠️

- 需要收集 200 帧后才自动初始化 Mahony 节点
- 如果这 200 帧中有运动，初始姿态和陀螺零偏估计不准
- 用户可能不知道需要静止等待

### 10.5 无异常数据检测 ⚠️

- 没有检测 NaN/Inf 值
- 没有检测加速度异常值（如 > 16g 溢出）
- 没有检测陀螺异常值
- 没有检测传感器断连超时

### 10.6 无最大帧时间差限制 ⚠️

- 如果某传感器断开 5 秒后又恢复，`buffer[-1]` 取的是恢复后第一帧
- 另一个传感器的 `buffer[-1]` 可能是 5 秒前的旧数据
- 两个明显不同步的帧直接配对计算

### 10.7 缺少安装方向和坐标轴约定的用户文档 ⚠️

- 用户不清楚传感器 XYZ 轴应该朝向哪里
- 不知道大腿传感器和小腿传感器是否需要特定的对齐方向
- 屈伸正负方向没有明确说明

---

## 11. 后续改进建议

### 11.1 优先级：高

#### A. 修复 imu9 字段映射

**文件**: `python/ui/threads.py` 第 494 行

当前：
```python
imu9 = np.array([float(row[i]) for i in range(4, 4+9)], dtype=float)
# → [acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, angle_x, angle_y, angle_z]
```

建议改为明确提取 acc、gyr、mag：
```python
# 明确提取（虽然 use_mag=False 当前不影响，但提高代码可读性和可维护性）
acc = np.array([float(row[4]), float(row[5]), float(row[6])], dtype=float)
gyr = np.array([float(row[7]), float(row[8]), float(row[9])], dtype=float)
mag = np.array([float(row[13]), float(row[14]), float(row[15])], dtype=float)
imu9 = np.concatenate([acc, gyr, mag])
```

#### B. 提取 esp32_rx_ms 用于帧匹配

**文件**: `python/ui/threads.py` `_run_impl()`

在解析 `[DATA]` 行时额外提取 `esp32_rx_ms`：
```python
esp32_rx_ms = int(row[3])  # 当前未提取
```

将 `(imu9, esp32_rx_ms)` 一起存入缓冲区，供后续近邻匹配使用。

#### C. 加入实时近邻匹配

**文件**: `python/ui/threads.py` `_get_engine_inputs()` + `python/joint/joint_angle.py`

建议方案：
```python
def _get_engine_inputs(self, max_dt_ms=30):
    """取时间最接近的一对帧"""
    buf_p = self._device_bufs[self._prox_id]  # [(imu9, esp32_rx_ms), ...]
    buf_d = self._device_bufs[self._dist_id]
    # ... 在 buf_p 中找与 buf_d[-1] 时间最接近的帧
    # ... 检查时间差是否在 max_dt_ms 内
```

### 11.2 优先级：中

#### D. 调整 Mahony dt 匹配实际回传率

**文件**: `python/ui/threads.py` 第 441 行 + `python/joint/joint_angle.py`

当前 `fs=50.0`，WT901 实际约 100 Hz。建议改为 `fs=100.0` 或在配置中注明。

#### E. 增加异常数据检测

- 加速度模长检测（0.5g < |acc| < 2.5g 为合理范围，静止时应接近 1g）
- NaN/Inf 检测
- 传感器超时检测（如 > 500 ms 无新数据，告警并暂停角度计算）

#### F. 增加最大帧时间差限制

在 `JointAngleEngine.update()` 或 `_get_engine_inputs()` 中增加检查：
- 如果两个传感器最新帧的 `esp32_rx_ms` 差值 > 30 ms，跳过当前配对
- 可配置 `max_sync_dt_ms`

### 11.3 优先级：低

#### G. 完善用户校准流程

- 添加安装方向图示（传感器 XYZ 轴与人体坐标系的关系）
- 校准结果可视化：显示 q_rel_0 的 roll/pitch/yaw 分量，帮助验证安装是否正确
- 添加校准质量评分

#### H. 添加关节角度正负方向说明

- 文档化：flexion 正值 = 屈膝，负值 = 伸膝
- 在 UI 上标注角度极性

#### I. 支持 TCP 连接作为备选数据通道

当前实时关节测量仅通过串口，若串口被数据采集线程占用则无法同时运行。可考虑支持 TCP 直接连接 ESP32（端口 8888），使数据采集和关节测量可以并行运行。

---

## 12. 总结

| 检查项 | 状态 | 说明 |
|--------|------|------|
| 膝关节角度公式 | ✅ **正确** | `q_knee = inv(q_thigh) * q_shank`，非简单欧拉角相减 |
| 姿态解算算法 | ✅ **正确** | Mahony 互补滤波器，重新解算，不使用 WT901 自带角度 |
| 校准流程 | ✅ **已有** | 站立静止校准，计算 q_rel_0，静止检测 |
| 设备映射 | ✅ **已有** | 支持 device_alias_map.json + 自动分配 |
| 实时帧同步 | ✅ **已修复** | esp32_rx_ms 近邻匹配，max 30 ms 阈值 |
| esp32_rx_ms 使用 | ✅ **已修复** | 实时路径提取并用于帧匹配 |
| imu9 字段映射 | ✅ **已修复** | acc/gyr/mag 显式提取（col 4-6, 7-9, 13-15） |
| Mahony dt | ✅ **已修复** | fs=50→100 Hz，dt=0.01s |
| 异常检测 | ✅ **已修复** | NaN/Inf 过滤 + 加速度模长检查 + 传感器超时告警 |
| 安装方向说明 | ✅ **已文档化** | 角度极性约定 + 传感器方向参考（README.md + __init__.py） |
| 角度正负约定 | ✅ **已文档化** | flexion(+)=屈膝, abduction(+)=外展, rotation(+)=外旋 |

**总体评价**：核心算法流程正确，所有已识别的高/中优先级问题均已修复。

---

## 13. 已实施的改进 (2026-06-30)

### A. imu9 字段映射修复
**文件**: `python/ui/threads.py` — `_run_impl()` 数据提取段

- 旧：`imu9 = row[4:13]` → 提取 acc(3)+gyr(3)+angle(3)，angle_x/y/z 被错误地当作 mag 传入
- 新：显式提取 `acc = row[4:7]`, `gyr = row[7:10]`, `mag = row[13:16]`，拼接为正确的 `[ax,ay,az, gx,gy,gz, mx,my,mz]`
- 影响：消除了字段映射不一致，提升了代码可读性和可维护性

### B. esp32_rx_ms 时间戳提取
**文件**: `python/ui/threads.py` — `_run_impl()`

- 新增提取 `esp32_rx_ms = int(row[3])`
- 将 `(imu9, esp32_rx_ms)` 元组存入 `_device_bufs`，替代之前只存 imu9
- 供近邻帧匹配使用

### C. 实时近邻帧匹配
**文件**: `python/ui/threads.py` — `_get_engine_inputs()`

- 旧：直接取各自缓冲区最后一帧（无时间同步）
- 新：以远端传感器最新帧的 `esp32_rx_ms` 为锚点，在近端缓冲区中线性搜索时间最接近的帧
- 最大允许时间差 **30 ms**（可配置 `_max_sync_dt_ms`），超过阈值则跳过本次计算

### D. 姿态解算 dt 调整
**文件**: `python/ui/threads.py` — `JointAngleEngine` 构造

- `fs=50.0` → `fs=100.0`（dt=0.01s），与 WT901 回传率匹配
- 校准缓冲需求同步调整：`3.0 * 50` → `3.0 * 100`
- 历史曲线缓冲区同步调整

### E. 异常数据检测
**文件**: `python/ui/threads.py` + `python/joint/joint_angle.py`

- 数据提取层：NaN/Inf 过滤、加速度模长范围检查（0.3g ~ 3.5g）
- 引擎层：相同的安全检查作为二次防护
- 传感器超时告警：周期性报告中显示各传感器最后数据到达时间，超过 1 秒标注 ⚠超时

### F. 最大帧时间差限制
**文件**: `python/ui/threads.py` — `_get_engine_inputs()`

- 已集成在近邻匹配中：`best_dt > max_dt_ms` 时返回 (None, None)
- 避免将明显不同步的两帧配对计算

### G. 标定诊断信息
**文件**: `python/ui/threads.py` + `python/ui/app.py`

- 标定完成后计算 `q_rel_0` 的 Euler 分解（roll/pitch/yaw）
- UI 显示安装姿态诊断信息，帮助用户验证传感器安装方向是否正确

### H. 角度极性文档化
**文件**: `python/ui/__init__.py` + `python/joint/joint_models.py`

- 添加完整的角度极性约定注释：flexion(+) = 屈膝, abduction(+) = 外展, rotation(+) = 外旋
- 添加传感器安装方向参考说明

### I. TCP 通道说明
**文件**: `python/ui/threads.py` — `JointAngleThread` 类文档

- 在类文档中添加了帧同步策略说明和 TCP 通道的后续支持说明
