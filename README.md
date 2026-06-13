# ESP32 WT901WIFI 双传感器 WiFi 网关

基于 ESP32 的 WiFi 网关系统，同时连接两个 **WT901WIFI IMU 姿态传感器**，通过 UDP 接收传感器数据，解析后以 CSV 格式通过 **串口 (USB)** 上传至上位机。

---

## 系统架构

```
┌─────────────────────┐
│ WT901WIFI Sensor 1  │──UDP:1399──┐
│  (Device ID: WT...) │            │
└─────────────────────┘            │
                                   ├──► ESP32 AP ◄── Serial USB ──► 上位机 (csv_receiver.py)
┌─────────────────────┐            │    192.168.10.1
│ WT901WIFI Sensor 2  │──UDP:1399──┘
│  (Device ID: WT...) │
└─────────────────────┘
```

## 硬件配置

| 项目 | 参数 |
|------|------|
| 主控 | ESP32 (Dev Module) |
| 框架 | Arduino (espressif32 @ 7.0.0) |
| Flash | 4 MB (分区: min_spiffs) |
| 传感器 | WT901WIFI × 2（最大支持 4 个） |
| 指示灯 | GPIO2（有客户端连接时点亮） |

## WiFi 网络配置

| 参数 | 值 |
|------|------|
| SSID | `ESP32_Gait_Gateway` |
| 密码 | `12345678` |
| AP IP | `192.168.10.1` |
| 子网掩码 | `255.255.255.0` |
| 最大客户端数 | 4 |
| UDP 监听端口 | `1399` |
| TCP 服务端口 | `8888` |

## 目录结构

```
WIFI_ESP32/
├── platformio.ini                   # PlatformIO 项目配置
├── .gitignore                       # Git 忽略规则
├── README.md                        # 本文件
│
├── src/
│   ├── main.cpp                     # 主程序：WiFi AP、UDP 接收、多传感器管理、CSV 输出
│   ├── wt901_parser.h               # WT901 帧解析器头文件（WT901Data 结构体定义）
│   ├── wt901_parser.cpp             # WT901 帧解析实现（54 字节帧 → 结构化数据）
│   └── TCP_service/
│       ├── tcp_service.h            # TCP 数据服务器类接口
│       └── tcp_service.cpp          # TCP 服务器实现（非阻塞、单客户端、自动重连）
│
├── scripts/
│   └── csv_receiver.py              # 上位机 Python CSV 数据接收脚本
│
├── include/                         # 项目公共头文件目录
├── lib/                             # 项目私有库目录
└── test/                            # 单元测试目录
```

## 数据协议

### WT901WIFI 帧格式（54 字节）

| 字节偏移 | 长度 | 字段 | 说明 |
|----------|------|------|------|
| 0–1 | 2 | 帧头 | 固定 `0x57 0x54` (`WT`) |
| 2–11 | 10 | 设备 ID | 传感器唯一标识（ASCII） |
| 12 | 1 | 年 | 20YY 格式 |
| 13 | 1 | 月 | 1–12 |
| 14 | 1 | 日 | 1–31 |
| 15 | 1 | 时 | 0–23 |
| 16 | 1 | 分 | 0–59 |
| 17 | 1 | 秒 | 0–59 |
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

### CSV 输出格式（14 列）

ESP32 输出统一 CSV 行（14 列），上位机 Python 脚本按 `device_id` 自动拆分到各自的 CSV 文件。

```csv
device_id,timestamp,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,angle_x,angle_y,angle_z,mag_x,mag_y,mag_z
```

示例数据：
```csv
WT1234567890,2026-06-10 14:32:15.234,0.015,-0.003,1.002,0.12,-0.08,0.05,-1.23,2.15,0.87,32.5,-45.2,18.7
```

| 列 | 字段 | 单位 |
|----|------|------|
| 1 | device_id | — |
| 2 | timestamp | `YYYY-MM-DD HH:MM:SS.mmm` |
| 3–5 | acc_x, acc_y, acc_z | g（重力加速度） |
| 6–8 | gyro_x, gyro_y, gyro_z | °/s（度/秒） |
| 9–11 | angle_x, angle_y, angle_z | °（度） |
| 12–14 | mag_x, mag_y, mag_z | 原始磁场值 |

### 上位机 CSV 存储结构

Python 脚本自动创建以时间戳命名的文件夹，每个传感器的数据写入独立的 CSV 文件：

```
wt901_data_20260610_143000/        ← 数据文件夹
├── WT1234567890.csv               ← 传感器 1 的所有数据
└── WT0987654321.csv               ← 传感器 2 的所有数据
```

## 核心模块说明

### 1. WT901 帧解析器 (`wt901_parser.h/.cpp`)

**数据结构：**
```cpp
struct WT901Data {
  char deviceId[13];        // 设备 ID
  uint16_t year, month, day, hour, minute, second, millisecond;
  float accX, accY, accZ;   // 加速度 (g)
  float gyroX, gyroY, gyroZ; // 角速度 (°/s)
  float magX, magY, magZ;   // 磁场原始值
  float angleX, angleY, angleZ; // 角度 (°)
  float temperature;        // 温度 (°C)
  uint16_t batteryRaw;      // 电量原始值
  int batteryPercent;       // 电量百分比 0–100
  int16_t rssi;             // WiFi 信号强度
  int16_t version;          // 固件版本
};
```

**核心函数：**
- `parseWT901Frame(data, out)` — 解析 54 字节帧，返回 `WT901Data`
- `getSignInt16(num)` — 无符号→有符号整数转换（补码）
- `readInt16LE(data, idx)` — 小端读取有符号 int16
- `readUInt16LE(data, idx)` — 小端读取无符号 uint16
- `getElectricPercentage(quantity)` — 多段映射：原始电量→百分比（0–100）

### 2. 多传感器管理 (`main.cpp`)

**问题：** 两个传感器 UDP 包可能交错到达，单帧 buffer 会导致数据混乱。

**方案：** 按源 IP 维护独立的帧组装状态机。

```cpp
struct SensorSession {
  IPAddress ip;              // 传感器 IP
  uint8_t  frameBuffer[54];  // 帧组装缓冲区
  int      frameIndex;       // 当前写入位置
  uint32_t lastActivityMs;   // 最后活动时间
  char     deviceId[13];     // 设备 ID（首次成功解析后缓存）
  bool     deviceIdKnown;    // 是否已获知设备 ID
  bool     active;           // 槽位是否占用
};

SensorSession sessions[MAX_SENSORS]; // 最多 4 个传感器
```

**数据流：**
```
UDP 包到达 → 获取 remoteIP
          → findOrCreateSession(IP) → 查找/创建 per-IP Session
          → 逐字节 feedSensorByte(session, byte)
          → 帧满 54 字节 → parseWT901Frame()
          → outputCSVRow() → Serial 输出（带 [DATA] 前缀）
```

### 3. TCP 数据服务器 (`TCP_service/tcp_service.h/.cpp`)

**设计要点：**
- 基于 `WiFiServer` / `WiFiClient`，非阻塞运行
- 同时只接受一个上位机连接
- `handle()` 在 `loop()` 中每帧调用，管理连接生命周期
- `sendLine()` 自动追加 `\n`
- 断线自动检测，上位机可随时重连

```cpp
TCPDataServer tcpServer(8888);

void setup() {
  tcpServer.begin();               // 启动 TCP 监听
}

void loop() {
  tcpServer.handle();              // 接受连接 / 检测断线
  if (tcpServer.isConnected()) {
    tcpServer.sendLine(csvLine);   // 发送 CSV 行
  }
}
```

## 使用方法

### 编译 & 烧录

```bash
# 编译
platformio run

# 烧录到 ESP32
platformio run --target upload

# 查看串口输出
platformio device monitor
```

### 传感器连接

1. ESP32 上电后自动启动 AP：`ESP32_Gait_Gateway`
2. WT901WIFI 传感器配置为 Station 模式，连接到上述 WiFi
3. 传感器自动向 `192.168.10.1:1399` 发送 UDP 数据包
4. ESP32 自动识别不同传感器（按源 IP → 提取 Device ID）

### 上位机接收数据

上位机通过 **USB 串口** 连接 ESP32 接收传感器数据。

**前置条件：** ESP32 通过 USB 线连接至上位机，安装 pyserial

```bash
# 安装串口依赖
pip install -r scripts/requirements.txt

# 基本用法（需指定 COM 端口）
python scripts/csv_receiver.py -p COM5

# 指定波特率 + 详细输出
python scripts/csv_receiver.py -p COM5 -b 115200 --verbose

# 列出可用串口
python scripts/csv_receiver.py --list-ports
```

> **注意：** Python 脚本直接占用 COM 口。请勿同时使用 `platformio device monitor`（同一端口只能被一个程序打开）。

**命令行参数：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--serial-port` / `-p` | — | 串口名称，如 `COM5`（**必填**） |
| `--serial-baud` / `-b` | `115200` | 串口波特率 |
| `--output` | `wt901_data_YYYYMMDD_HHMMSS/` | 输出文件夹路径 |
| `--verbose` / `-v` | 关闭 | 逐行打印接收的 CSV 数据 |
| `--list-ports` | — | 列出可用串口并退出 |

**运行示例：**
```
============================================================
ESP32 WT901WIFI Gateway — CSV Serial Receiver
============================================================
  Port:   COM5 @ 115200 baud
  Output: wt901_data_20260610_143000/
  Verbose: True
============================================================
Press Ctrl+C to stop.

[OK] Serial port COM5 opened at 115200 baud
[INFO] Output folder: wt901_data_20260610_143000
[CSV] Created: wt901_data_20260610_143000/WT1234567890.csv -> device: WT1234567890
[     1] [WT1234567890] WT1234567890,2026-06-10 14:32:15.234,0.015,...
...
^C
[STOP] Shutting down...
[DONE] Total rows: 15234
```

### 串口调试输出

ESP32 固件通过串口同时输出 **CSV 数据行**（以 `[DATA]` 前缀标记）和 **诊断日志**（无前缀）。上位机脚本通过 `[DATA]` 前缀自动过滤。

在 Arduino IDE 或 `platformio device monitor`（波特率 115200）中可看到：

```
========== ESP32 WT901WIFI Gateway ==========
[AP] Static IP configured.
[AP] Started successfully.
[AP] SSID: ESP32_Gait_Gateway
[AP] Password: 12345678
[AP] IP: 192.168.10.1
[UDP] Listening on port 1399
[TCP] Server started on port 8888
[Session] IP 192.168.10.2 -> Device ID: WT5500010121
[DATA]WT5500010121,2026-06-13 10:00:00.123,0.015,-0.003,1.002,...
[DATA]WT5500010121,2026-06-13 10:00:00.133,0.014,-0.004,1.001,...
...
---------- Gateway Status ----------
AP IP: 192.168.10.1
Connected WiFi clients: 2
...
```

## 技术特性

| 特性 | 说明 |
|------|------|
| 多传感器支持 | 按源 IP 独立帧组装，最多 4 传感器并发 |
| 串口数据输出 | Serial 输出 CSV 数据（带 `[DATA]` 前缀），与诊断日志自动区分 |
| 上位机串口接收 | Python 脚本通过 USB 串口接收，自动重连 |
| 自动设备识别 | 首次解析成功后缓存 Device ID，后续帧直接使用 |
| 上位机自动重连 | 串口断线（USB 拔出）后自动重试连接 |
| 内存效率 | RAM 占用仅 ~14%，帧解析使用固定栈分配 |

## 电量百分比映射表

| 原始值范围 | 电量百分比 |
|------------|------------|
| > 396 | 100% |
| 394–396 | 90% |
| 388–393 | 75% |
| 383–387 | 60% |
| 380–382 | 50% |
| 378–379 | 40% |
| 374–377 | 30% |
| 371–373 | 20% |
| 369–370 | 15% |
| 351–368 | 10% |
| 341–350 | 5% |
| ≤ 340 | 0% |

## 依赖项

- **Platform:** Espressif 32 (7.0.0+)
- **Framework:** Arduino (espressif32 3.x)
- **Libraries:** WiFi (内置，无需额外安装)
- **上位机:** Python 3.8+ + pyserial (`pip install -r scripts/requirements.txt`)

## 许可证

本项目用于教育和研究目的。
