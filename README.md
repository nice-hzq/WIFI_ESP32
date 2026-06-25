# ESP32 WT901WIFI 步态分析系统

基于 ESP32 的 WiFi 网关 + Python 步态分析算法库，用于穿戴式 IMU 步态数据采集与分析。

系统分两层：
- **ESP32 固件层** — WiFi AP 网关，接收多个 WT901WIFI 传感器的 UDP 数据，通过串口/TCP 转发至上位机
- **Python 算法层** — 离线步态分析管线，处理 IMU 数据，输出结构化步态报告与姿态曲线

---

## 系统架构

```
┌──────────────────────┐
│ WT901WIFI Sensor 1   │──UDP:1399──┐
│ (Device ID: WT...)   │            │
└──────────────────────┘            │
                                    ├──► ESP32 AP ◄── Serial/TCP ──► 上位机
┌──────────────────────┐            │    192.168.10.1                  │
│ WT901WIFI Sensor 2   │──UDP:1399──┘                                  │
│ (Device ID: WT...)   │                                               │
└──────────────────────┘                                    ┌──────────▼──────────┐
                                     csv_receiver.py ──────►│ wt901_data_.../     │
                                                            │  ├── WT....csv      │
                                                            │  └── WT....csv      │
                                                            └──────────┬──────────┘
                                                                       │
                                            Python 步态分析管线 ◄────────┘
                                            ┌──────────────────────────┐
                                            │ 校准 → 姿态解算 → 事件   │
                                            │ 检测 → 空间参数 → 报告   │
                                            └──────────┬───────────────┘
                                                       │
                                          output/ ◄─────┘
                                          ├── attitude_*.png
                                          ├── gait_events_debug.png
                                          └── (JSON report)
```

---

## 目录结构

```
WIFI_ESP32/
├── platformio.ini                       # PlatformIO 项目配置
├── .gitignore                           # Git 忽略规则
├── README.md                            # 本文件
├── config.json                          # Python 算法静态配置
│
├── src/                                 # ESP32 固件 (C++)
│   ├── main.cpp                         # 主程序: WiFi AP, UDP, 多传感器管理, CSV 输出
│   ├── config.h                         # 固件编译期配置 (WiFi/端口/硬件)
│   ├── wt901_parser.h                   # WT901 帧解析器头文件
│   ├── wt901_parser.cpp                 # 帧解析实现 (54 字节 → WT901Data)
│   ├── session_manager.h                # 多传感器会话管理 API
│   ├── session_manager.cpp              # per-IP 帧组装, 槽位复用
│   └── TCP_service/
│       ├── tcp_service.h                # TCP 数据服务器接口
│       └── tcp_service.cpp              # TCP 非阻塞服务器实现
│
├── scripts/                             # 上位机脚本
│   ├── csv_receiver.py                  # 串口 CSV 接收（多传感器同步）
│   ├── test_sync_buffer.py              # 同步缓冲区测试
│   └── requirements.txt                 # pyserial
│
├── python/                              # 步态分析算法库 (Python)
│   ├── README.md                        # 算法库独立说明
│   ├── requirements.txt                 # numpy, pandas, scipy, AHRS, matplotlib
│   ├── config.json                      # 运行时配置
│   ├── ui_app.py                        # 桌面 UI 入口 (tkinter)
│   ├── test_gait_report.py              # 命令行测试入口
│   │
│   ├── core/                            # 核心模块
│   │   ├── config.py                    # 全局配置变量 + config.json 加载器
│   │   ├── quaternion.py                # 四元数运算库 (mul/conj/rotate/euler↔quat)
│   │   └── math_utils.py               # 滤波/统计/信号处理工具
│   │
│   ├── sensor/                          # 数据 I/O
│   │   └── data_reader.py              # CSV 加载, 列名规范化, 校准, 滤波
│   │
│   ├── orientation/                     # 姿态解算
│   │   ├── quaternion_manager.py       # Mahony 节点管理, 加速度旋转, 去重力
│   │   ├── quaternions.py             # QuaternionManager (多节点管理)
│   │   └── euler_angles.py            # 四元数→欧拉角, 绘图工具
│   │
│   ├── gait/                            # 步态分析管线
│   │   ├── gait_pipeline.py            # 主入口: 调度完整分析流程
│   │   ├── event_detection.py          # HS/TO/MS 步态事件检测 (自适应 gyro)
│   │   ├── distance_metrics.py         # 空间参数: 步长/步宽/步速 (双检测器)
│   │   ├── distance_new.py             # ZUPT 零速修正 + 积分
│   │   ├── count_phase.py              # Mahony 姿态 + 转身检测 + 转身内计步
│   │   ├── temporal_metrics.py         # 时间参数: 步频/支撑相/摆动相/双支撑
│   │   ├── metrics_builder.py          # 最终报告组装 + 距离融合
│   │   ├── turn_detection.py           # TUG 转身检测
│   │   └── tool.py                     # 通用工具 (stride_length/step_width/impact)
│   │
│   ├── output/                          # 曲线输出 (NEW)
│   │   └── attitude_curves.py          # 传感器姿态曲线 (roll/pitch/yaw) → PNG
│   │
│   ├── report/                          # 报告模型
│   │   └── gait_models.py              # GaitAnalysisResult 数据类定义
│   │
│   ├── ui/                              # 桌面 UI
│   │   ├── app.py                       # GaitAnalysisApp (tkinter)
│   │   ├── threads.py                   # 后台工作线程
│   │   ├── dialogs.py                   # 对话框
│   │   └── display.py                   # 结果显示组件
│   │
│   ├── tests/                           # 单元测试
│   │   ├── test_math_utils.py
│   │   └── test_quaternion.py
│   │
│   └── Data/                            # 测试数据目录
│
├── include/                             # 项目公共头文件
├── lib/                                 # 项目私有库
└── test/                                # 单元测试 (C++)
```

---

## 硬件配置

| 项目 | 参数 |
|------|------|
| 主控 | ESP32 (Dev Module) |
| 框架 | Arduino (espressif32 @ 7.0.0) |
| Flash 分区 | min_spiffs |
| 传感器 | WT901WIFI × N（最大 8） |
| 指示灯 | GPIO2（有客户端连接时亮） |
| 串口波特率 | 921600 (ESP32) / 115200 (上位机接收) |

## WiFi 网络配置 (`src/config.h`)

| 参数 | 值 |
|------|------|
| AP SSID | `ESP32_Gait_Gateway` |
| AP 密码 | `12345678` |
| AP IP | `192.168.10.1` |
| 子网掩码 | `255.255.255.0` |
| 最大客户端 | 8 |
| UDP 端口 | `1399` |
| TCP 端口 | `8888` |

---

## ESP32 固件

### 数据协议

**WT901WIFI 帧格式（54 字节）：**

| 字节偏移 | 长度 | 字段 | 说明 |
|----------|------|------|------|
| 0–1 | 2 | 帧头 | 固定 `0x57 0x54` (`WT`) |
| 2–11 | 10 | 设备 ID | 传感器唯一标识（ASCII） |
| 12–17 | 6 | 日期时间 | 年/月/日/时/分/秒 |
| 18–19 | 2 | 毫秒 | 小端 uint16 |
| 20–25 | 6 | 加速度 X/Y/Z | 小端 int16 ×3，量程 ±16g |
| 26–31 | 6 | 角速度 X/Y/Z | 小端 int16 ×3，量程 ±2000°/s |
| 32–37 | 6 | 磁场 X/Y/Z | 小端 int16 ×3 |
| 38–43 | 6 | 角度 X/Y/Z | 小端 int16 ×3，量程 ±180° |
| 44–45 | 2 | 温度 | 小端 int16，单位 0.01°C |
| 46–47 | 2 | 电量原始值 | 小端 uint16 |
| 48–49 | 2 | RSSI | 小端 int16 |
| 50–51 | 2 | 固件版本 | 小端 int16 |
| 52–53 | 2 | 保留 | — |

**CSV 输出格式（14 列）：**

```csv
device_id,timestamp,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,angle_x,angle_y,angle_z,mag_x,mag_y,mag_z
```

| 列 | 字段 | 单位 |
|----|------|------|
| 1 | device_id | — |
| 2 | timestamp | `YYYY-MM-DD HH:MM:SS.mmm`（ESP32 millis 同步） |
| 3–5 | acc_x, acc_y, acc_z | g |
| 6–8 | gyro_x, gyro_y, gyro_z | °/s |
| 9–11 | angle_x, angle_y, angle_z | ° |
| 12–14 | mag_x, mag_y, mag_z | 原始值 |

### 多传感器管理

按源 IP 独立维护帧组装状态机，支持最多 8 个传感器并发：

```
UDP 包到达 → remoteIP → findOrCreateSession(IP)
                       → feedSensorByte(session, byte)  逐字节喂入
                       → 帧满 54B → parseWT901Frame()
                       → outputCSVRow() → Serial [DATA] + TCP
```

核心数据结构：

```cpp
struct SensorSession {
  IPAddress ip;              // 传感器 IP
  uint8_t  frameBuffer[54];  // 帧组装缓冲
  int      frameIndex;       // 当前写入位置（帧头对齐: 0x57 0x54）
  uint32_t lastActivityMs;   // 最后活动时间（槽满时复用最旧）
  char     deviceId[13];     // 首次解析后缓存
  bool     deviceIdKnown;
  bool     active;
};
```

### TCP 数据服务器

基于 `WiFiServer` 的非阻塞 TCP 服务器，同时只接受一个上位机连接：

```cpp
TCPDataServer tcpServer(8888);
tcpServer.begin();
tcpServer.handle();              // 每帧管理连接生命周期
tcpServer.sendLine(csvLine);     // 自动追加 \n
```

### 编译 & 烧录

```bash
platformio run                    # 编译
platformio run --target upload    # 烧录
platformio device monitor         # 串口监视（115200）
```

### 串口输出格式

ESP32 通过串口同时输出两类行：
- **`[DATA]...`** — CSV 数据行（14 列），上位机脚本据此过滤
- **无前缀** — 诊断日志（AP 状态、连接信息、周期状态报告）

---

## 上位机数据接收

### csv_receiver.py

```bash
pip install pyserial

# 基本用法
python scripts/csv_receiver.py -p COM5

# 多传感器同步（2 个传感器时间对齐）
python scripts/csv_receiver.py -p COM5 --sync-sensors 2

# 列出可用串口
python scripts/csv_receiver.py --list-ports
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `-p` / `--serial-port` | 必填 | 串口名 (COM5, /dev/ttyUSB0) |
| `-b` / `--serial-baud` | 115200 | 波特率 |
| `--output` | `wt901_data_YYYYMMDD_HHMMSS/` | 输出目录 |
| `--sync-sensors` | 2 | 预期传感器数（1=不启用同步） |
| `--sync-window-ms` | 100 | 同步窗口（毫秒） |
| `-v` / `--verbose` | 关闭 | 逐行打印 |

输出结构：
```
wt901_data_YYYYMMDD_HHMMSS/
├── WT5500010121.csv
└── WT5500010131.csv
```

---

## Python 步态分析算法库

### 依赖

```bash
pip install -r python/requirements.txt
```

```
numpy~=2.2.6   pandas~=2.3.3   matplotlib~=3.10.7
AHRS~=0.4.0    scipy~=1.15.3
```

### 工作模式

| 模式 | 传感器别名 | 说明 |
|------|-----------|------|
| `lower_body` | S1, L4, L5, L6, R4, R5, R6 | 下肢步态（默认） |
| `upper_body` | H, T1, T12, L1, L2, L3, R1, R2, R3 | 上肢关节 |
| `full_body` | 全部 16 个 | 全身分析 |
| `feet_only` | L6, R6 | 仅双脚（无转身检测） |

### 传感器别名约定

| 部位 | 别名 | 位置 |
|------|------|------|
| 头部 | H | Head |
| 躯干 | T1, T12 | 胸椎 T1, T12 |
| 后背 | S1 | 骶骨 Sacrum |
| 左上肢 | L1, L2, L3 | 左上臂 / 左前臂 / 左手 |
| 右上肢 | R1, R2, R3 | 右上臂 / 右前臂 / 右手 |
| 左下肢 | L4, L5, L6 | 左大腿 / 左小腿 / 左脚 |
| 右下脚 | R4, R5, R6 | 右大腿 / 右小腿 / 右脚 |

### 配置

在代码中设置 `core.config` 或编辑 `python/config.json`：

```python
from core import config

config.originalDir = "./Data/wt901_data_20260615_150535"  # 原始 CSV 目录
config.tempDir     = "./temp"                              # 校准 JSON 目录
config.curveDir    = "./output"                            # 曲线输出目录
config.WORK_MODE   = "lower_body"
config.fs          = 50
```

### 运行分析

```bash
# 命令行测试入口
cd python && python test_gait_report.py

# 桌面 UI
cd python && python ui_app.py
```

### 分析管线

```
CSV 加载 → 校准(gyro bias + acc 6-face) → 滤波(moving avg)
  ├── 姿态解算 (Mahony complementary filter)
  │     └── 四元数 → 欧拉角 (roll/pitch/yaw)
  ├── 步态事件检测 (gyro_x 自适应 HS/TO/MS)
  │     ├── platform HS（偏大）  ─┐
  │     └── zero_cross HS（偏小） ─┤→ 双检测器平均
  ├── 加速度旋转 (sensor → world)
  ├── 去重力 + 静止偏置校正
  ├── ZUPT 零速修正 (HS→TO 支撑期清零)
  ├── 梯形积分 (vel → pos)
  ├── 空间参数 (步长/跨步长/步宽/步速/抬脚高度)
  ├── 时间参数 (步频/支撑相/摆动相/双支撑/步态周期)
  ├── 转身检测 (腰部 yaw 累计转角状态机)
  └── 报告组装 → JSON + PNG 曲线
```

### 输出

运行后 `output/` 目录生成：

| 文件 | 说明 |
|------|------|
| `attitude_<sensor>.png` | 每个传感器的 Roll/Pitch/Yaw 姿态曲线 |
| `attitude_all_sensors_overview.png` | 所有传感器姿态叠图对比 |
| `gait_events_debug.png` | 步态事件检测调试图 (HS/TO/MS 标记) |

函数返回值：结构化 JSON（包含 basicParameters, stepParameters, phaseParameters, jointAngles 等）。

### 校准文件

存放于 `temp/` 目录：

| 文件 | 说明 |
|------|------|
| `<alias>_gyro_bias.json` | 陀螺零偏 `{"bias": [bx, by, bz]}` |
| `<alias>_acc_6face.json` | 六面加速度校准 `{"bias_g": [...], "scale_g": [...]}` |

---

## 技术特性

| 特性 | 说明 |
|------|------|
| 多传感器管理 | ESP32 按源 IP 独立帧组装，最多 8 传感器并发 |
| 时间同步 | ESP32 millis() 覆盖传感器时间，多传感器共享时间基准 |
| 双检测器平均 | platform HS + zero_cross HS 分别估计空间参数后取平均 |
| 自适应事件检测 | 基于局部步态周期比例 + 鲁棒 scale 的 HS/TO/MS 检测 |
| ZUPT 漂移校正 | 摆动段首尾零速约束 + 线性去漂移 |
| 姿态可视化 | 自动生成 Roll/Pitch/Yaw 曲线保存到 output/ |
| 磁力计门控 | 磁场异常时自动切回纯 IMU 模式，避免磁干扰污染 |
| 串口自动重连 | csv_receiver.py 断线后自动重试连接 |

---

## 数据流总览

```
┌──────────────────────────────────────────────────────────┐
│ ESP32 Firmware                                           │
│                                                          │
│ WT901WiFi(s) ──UDP──► per-IP sessions ──► WT901Data     │
│                                          ├──► Serial USB │
│                                          └──► TCP :8888  │
└──────────────────────────────────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────┐
│ csv_receiver.py                                          │
│                                                          │
│ Serial ──► [DATA] filter ──► per-device CSV files        │
│                              wt901_data_<ts>/             │
└──────────────────────────────────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────┐
│ Python Gait Pipeline                                     │
│                                                          │
│ CSV files ──► calibrate ──► Mahony quat ──► Euler        │
│                          ├──► HS/TO detect               │
│                          ├──► acc rotate → world         │
│                          ├──► ZUPT → vel/pos             │
│                          ├──► spatial metrics            │
│                          ├──► temporal metrics           │
│                          ├──► turn detection             │
│                          └──► JSON report + PNG curves   │
└──────────────────────────────────────────────────────────┘
```

## 许可证

本项目用于教育和研究目的。
