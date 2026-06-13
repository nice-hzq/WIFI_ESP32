# 步态分析算法库

基于 IMU 传感器数据的步态分析 Python 算法库，作为 C# 上位机的算法后端，处理穿戴式 ROM_EVA 设备的多节点传感器数据，生成结构化步态分析报告。

## 功能概述

- **传感器校准**：六面加速度校准、陀螺仪零偏校准、磁力计椭球校准、T-pose 参考姿态校准
- **姿态估计**：基于 Mahony 互补滤波器的四元数姿态解算
- **步态事件检测**：基于脚部陀螺仪数据的 HS（Heel Strike）检测、转身检测
- **时空参数计算**：步长、步幅、步宽、步速、步频、步态周期、相位占比
- **关节角度分析**：下肢（髋/膝/踝）和上肢（肩部）关节角度范围统计
- **报告生成**：结构化 JSON 报告 + 关节角度曲线图（PNG）+ CSV 数据输出

## 技术栈

- Python 3.10+
- numpy / pandas / scipy — 数据处理与信号滤波
- matplotlib — 图表生成
- AHRS — Mahony 互补滤波器

## 安装

```bash
pip install -r requirements.txt
```

## 数据格式要求

### CSV 输入文件

原始传感器数据以 CSV 文件存放于数据目录中，每个传感器节点一个文件。

**文件命名规范**：`<时间戳>_ROM_EVA_<设备ID>_<传感器别名>.csv`

例如：`2026_04_17_14_40_51_ROM_EVA_621C70E51FCD_R6.csv`

**传感器别名约定**：

| 别名 | 位置 | 所属模式 |
|------|------|----------|
| H | 头部 | upper_body / full_body |
| T1, T12 | 躯干 | upper_body / full_body |
| L1, L2, L3 | 左上臂、左前臂、左手 | upper_body / full_body |
| R1, R2, R3 | 右上臂、右前臂、右手 | upper_body / full_body |
| S1 | 骶骨（后背） | lower_body / full_body |
| L4, L5, L6 | 左大腿、左小腿、左脚 | lower_body / full_body |
| R4, R5, R6 | 右大腿、右小腿、右脚 | lower_body / full_body |

**CSV 列名要求**（至少包含）：

| 列名 | 说明 | 单位 |
|------|------|------|
| Acc_x, Acc_y, Acc_z | 加速度计三轴 | g |
| Gyr_x, Gyr_y, Gyr_z | 陀螺仪三轴 | deg/s |
| Geo_x, Geo_y, Geo_z | 地磁/磁力计三轴（可选） | — |

### 校准 JSON 文件

校准参数以 JSON 格式存放于 `temp/` 目录中：

- `<alias>_gyro_bias.json` — 陀螺仪零偏
- `<alias>_acc_6face.json` — 加速度计六面校准 bias/scale
- `<alias>_mag_calib.json` — 磁力计椭球校准（可选）

## 运行

### 1. 配置参数

运行前在代码中配置以下参数（参考 `test_gait_report.py`）：

```python
from core import config

config.tempDir = "./temp"                      # 校准 JSON 目录
config.originalDir = "./Data/walk_data1"       # 原始 CSV 数据目录
config.curveDir = "./output"                   # 输出目录
config.WORK_MODE = "lower_body"                # 工作模式: lower_body / upper_body / full_body
config.fs = 100                                # 采样率 (Hz)
```

### 2. 运行分析

```bash
# 方式一：运行测试入口
python test_gait_report.py

# 方式二：直接运行步态分析管线
python -m gait.gait_pipeline
```

### 3. C# 调用接口

```csharp
// 返回 GaitAnalysisResult 对象
var result = get_gait_analysis_object();

// 返回 JSON 字符串
var json = print_gait_summary();
```

调用前需通过 `core.config` 设置好路径和工作模式。

## 目录结构

```
python/
├── core/                # 核心模块（配置、四元数、数学工具）
├── sensor/              # 传感器数据读取与校准
├── gait/                # 步态分析管线
├── joint_analysis/      # 关节角度分析
├── orientation/         # 姿态估计
├── report/              # 报告生成
├── Data/                # 测试数据
├── temp/                # 校准参数
├── output/              # 分析结果输出
├── test_gait_report.py  # 主测试入口
└── requirements.txt     # 依赖清单
```

## 输出

分析完成后，`output/` 目录中生成：

| 文件 | 说明 |
|------|------|
| `gait_cycles_mean_std.csv` | 步态周期均值/标准差 |
| `joint_angles_all.csv` | 所有关节角度时序数据 |
| `joint_angles_with_events.png` | 关节角度曲线图（含步态事件标记） |

函数返回值为结构化 JSON 字符串，包含基本参数、步态参数、相位参数、冲击参数、关节极限等。
