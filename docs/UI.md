# 步态分析系统 UI 设计文档

## 技术栈

| 项目 | 说明 |
|---|---|
| **GUI 框架** | Tkinter (Python 标准库) |
| **主题控件** | ttk.Combobox, ttk.Notebook |
| **字体** | Microsoft YaHei UI / Consolas |
| **默认窗口** | 720×880 px, 最小 620×750 px |

## 源码结构

```
python/ui/
├── __init__.py   # 常量 (DEFAULT_BAUD, VALID_ALIASES, MODE_ALIASES, WORK_MODES)
├── app.py        # 主应用类 GaitAnalysisApp — 窗口布局、状态机、业务逻辑
├── threads.py    # 后台线程 — DataCollectionThread / GaitAnalysisThread
├── display.py    # 结果展示 — 三个 Tab 的数据填充
└── dialogs.py    # 弹窗 — 传感器映射对话框
```

入口：`python -m ui.main` 或直接运行 `app.py` 的 `main()`。

---

## 页面整体布局

```
┌─────────────────────────────────────────────────────┐
│  步态分析系统  |  Gait Analysis System               │  ← 标题
├─────────────────────────────────────────────────────┤
│  ┌─ 数据采集 (Data Collection) ───────────────────┐ │
│  │ 串口: [COM3 ▼] [↻刷新]  波特率: [921600]       │ │
│  │ [▶ 开始采集] [■ 停止采集]        [⟳ 新建会话]   │ │
│  │ ● 未连接  |  Ready                              │ │
│  └────────────────────────────────────────────────┘ │
│  ┌─ 最近会话 (Recent Sessions) ───────────────────┐ │
│  │ wt901_data_20260613_143052 | 1260行 | S1,L4..   │ │
│  │ [填入分析目录] [打开文件夹]                      │ │
│  │ 共 3 个会话                                     │ │
│  └────────────────────────────────────────────────┘ │
│  ┌─ 步态分析 (Gait Analysis) ─────────────────────┐ │
│  │ 数据目录: [________________________] [浏览...] [←最近]│
│  │ ⚠ 已识别: L6,R6  |  未映射设备: WTxxx           │ │
│  │                   [配置传感器映射...]            │ │
│  │ 工作模式: [lower_body ▼]    [▶ 运行步态分析]     │ │
│  │ 就绪 (Idle)                                     │ │
│  └────────────────────────────────────────────────┘ │
│  ┌─ 分析结果 (Results) ───────────────────────────┐ │
│  │                               [清除结果]        │ │
│  │ ┌─ 基本参数 ─┬─ 步态参数 ─┬─ 相位参数 ─┐       │ │
│  │ │            │            │            │       │ │
│  │ │  (grid     │  (grid     │  (grid     │       │ │
│  │ │   layout)  │   layout)  │   layout)  │       │ │
│  │ │            │            │            │       │ │
│  │ └───────────┴────────────┴────────────┘       │ │
│  └────────────────────────────────────────────────┘ │
│  ▓ 就绪 (Ready)                          ▓ 状态栏  │
└─────────────────────────────────────────────────────┘
```

---

## Section 1: 数据采集 (Data Collection)

**控件清单：**

| 控件 | 类型 | 变量 | 说明 |
|---|---|---|---|
| 串口选择 | `ttk.Combobox` | `self.port_var` | 自动枚举系统串口，优先选 CH340/CP2102 |
| 刷新串口 | `tk.Button` | — | 重新扫描 COM 端口列表 |
| 波特率 | `tk.Entry` | `self.baud_var` | 默认 921600 |
| 开始采集 | `tk.Button` | `self.btn_start` | 绿色 (#4caf50)，触发 `_start_collection` |
| 停止采集 | `tk.Button` | `self.btn_stop` | 红色 (#f44336)，默认 disabled |
| 新建会话 | `tk.Button` | `self.btn_new_session` | 清空结果 + 重置状态 |
| 状态标签 | `tk.Label` | `self.dc_status_var` | 显示连接/采集/完成 状态 |

**状态流转：**

```
IDLE  →  [开始采集]  →  COLLECTING  →  [停止采集]  →  STOPPING  →  (线程done)  →  IDLE
                                  ↘  (线程error) ↗
```

**连接状态标识：**
- `● 未连接` — 初始状态
- `◌ 连接中...` — 正在打开串口
- `● 已连接 @ COMx` — 连接成功
- `● 采集中 | 总行数: N` — 正在接收数据
- `✗ 连接断开` — 串口异常
- `✓ 采集完成` — 正常结束

**后台线程：** `DataCollectionThread` → 监听串口 `[DATA]` 行 → 按 `device_id` 分文件写 CSV → 通过 Queue 推送状态到主线程。

**Watchdog：** 停止后 8 秒内线程未报告 done，强制复位 UI。

---

## Section 2: 最近会话 (Recent Sessions)

| 控件 | 类型 | 说明 |
|---|---|---|
| 会话列表 | `tk.Listbox` (height=3) | 显示最近采集的会话：文件夹名/行数/设备 |
| 填入分析目录 | `tk.Button` | 将选中会话的文件夹路径填入"数据目录" |
| 打开文件夹 | `tk.Button` | 在资源管理器中打开选中会话文件夹 |
| 状态 | `tk.Label` | "暂无历史会话" 或 "共 N 个会话" |

**行为：** 每次采集完成 (`done` 消息) 自动添加到列表，最多保留 10 条。

---

## Section 3: 步态分析 (Gait Analysis)

| 控件 | 类型 | 变量 | 说明 |
|---|---|---|---|
| 数据目录 | `tk.Entry` | `self.folder_var` | 选择 CSV 数据所在文件夹 |
| 浏览... | `tk.Button` | — | 系统文件夹选择对话框 |
| ← 最近 | `tk.Button` | `self.btn_use_recent` | 一键填入最近采集目录 |
| 传感器状态 | `tk.Label` | `self.map_status_var` | 橙色警告 (#e67e22) |
| 配置映射 | `tk.Button` | `self.btn_map` | 仅在存在未映射设备时显示 |
| 工作模式 | `ttk.Combobox` | `self.mode_var` | `lower_body` / `feet_only` / `upper_body` / `full_body` |
| 运行分析 | `tk.Button` | `self.btn_analyze` | 蓝色 (#2196f3)，触发 `_run_analysis` |
| 状态 | `tk.Label` | `self.ga_status_var` | 就绪/运行中/完成/失败 |

**传感器自动检测逻辑：**

```
扫描 CSV 文件夹
  ├─ 文件名后缀在 VALID_ALIASES 中 → 直接识别
  ├─ 文件名后缀不在列表中 → 读 CSV 第一行 device_id
  │   ├─ 已存在映射 (device_alias_map.json) → 自动解析
  │   └─ 无映射 → 显示"未映射设备"警告 + [配置传感器映射] 按钮
  └─ 无 CSV → 显示"⚠ 未找到 CSV 文件"

自动推荐工作模式:
  {L6, R6}                        → feet_only
  {L6, R6} ⊆ aliases ⊆ lower_body → lower_body
  aliases ⊆ upper_body            → upper_body
```

**分析启动前验证：** 确保所需传感器别名都已识别（有映射），否则弹窗提示缺少哪些。

---

## Section 4: 分析结果 (Results)

### 结构

```
┌────────────────────────────────────────┐
│  [清除结果]                            │  ← 工具栏
├─ 基本参数 ──┬── 步态参数 ──┬── 相位参数 ──┤  ← ttk.Notebook (3 tabs)
│             │              │              │
│ Canvas +    │ Canvas +     │ Canvas +     │  ← 可滚动 (mousewheel)
│ Scrollbar   │ Scrollbar    │ Scrollbar    │
│             │              │              │
│ grid layout │ grid layout  │ grid layout  │  ← 统一 grid 布局
└────────────────────────────────────────┘
```

每个 Tab 使用 `Canvas + Scrollbar + inner Frame` 实现内容滚动，绑定 `MouseWheel` 事件。

### Tab 1: 基本参数

| 分类 | 字段 | 左右 |
|---|---|---|
| 时间/距离 | 行走距离 (Walking Distance) | — |
| | 步行速度 (Walking Speed) | L / R |
| | 步宽 (Step Width) | — |
| 步态周期 | 步态周期 (Gait Cycle) | — |
| | 步频 (Cadence) | L / R |
| | 支撑相时间 (Stride Time) | L / R |
| | 总步数 (Total Steps) | — |
| 转身 | 转身步数 / 转身时长 | 有 S1 时显示，否则显示"已跳过" |

### Tab 2: 步态参数

| 分类 | 字段 | 左右 |
|---|---|---|
| 步长/跨步 | 步长 (Step Length) | L / R |
| | 步长偏差 (Step Length Deviation) | — |
| | 跨步长 (Stride Length) | — |
| 步态时间 | 步时 (Step Time) | L / R |
| 有效步数 | 有效步 (Valid Steps) | L / R |
| 抬脚高度 | 抬脚高度 (Foot Lift Height) | L / R |

### Tab 3: 相位参数

| 字段 | 左右 |
|---|---|
| 支撑相 (Support Phase) | L / R |
| 摆动相 (Swing Phase) | L / R |
| 双支撑相 (Double Support) | — |

末尾显示 meta 信息：`版本 1.0 | 会话时长 12.6s | 时间戳 1778482145990`

---

## 状态机 (UIState)

```
        ┌─────────────────────────────────────┐
        │                                     │
        ▼                                     │
      IDLE  ◄──────  done / error / 新建会话 ──┤
     /    \                                    │
    /      \                                   │
   ▼        ▼                                  │
COLLECTING  ANALYZING                          │
   │          │                                │
   ▼          ▼                                │
STOPPING   (done/error) ───────────────────────┘
```

按钮启用/禁用由 `_transition_to()` 统一控制：

| 状态 | 开始 | 停止 | 新建会话 | 分析 | 清除 |
|---|---|---|---|---|---|
| IDLE | ✅ | ❌ | ✅ | ✅ | ✅ |
| COLLECTING | ❌ | ✅ | ❌ | ❌ | ❌ |
| STOPPING | ❌ | ❌ | ❌ | ❌ | ❌ |
| ANALYZING | ❌ | ❌ | ❌ | ❌ | ❌ |

---

## 消息队列协议

后台线程通过 `queue.Queue` 与主线程通信：

| type | 方向 | 携带字段 | 触发动作 |
|---|---|---|---|
| `status` | 采集线程→UI | state, message, folder | 更新 dc_status_var |
| `data` | 采集线程→UI | total_rows, devices | 每 100 行更新一次计数 |
| `error` | 采集线程→UI | message | 显示错误 |
| `done` | 采集线程→UI | total_rows, devices, folder | 记录会话 + 复位的 UI |
| `gait_status` | 分析线程→UI | message | 更新 ga_status_var |
| `gait_done` | 分析线程→UI | result, elapsed | 渲染结果到 3 个 tab |
| `gait_error` | 分析线程→UI | message, traceback, elapsed | 弹窗显示错误 |

---

## 传感器映射弹窗

当 CSV 文件中的 device_id 不是标准别名时，弹出 `Toplevel` 窗口：

```
┌─ 传感器映射 — Device → Alias Mapping ────────┐
│                                                │
│  将设备 ID 映射到身体传感器别名                 │
│  请为每个设备选择对应的身体位置：               │
│                                                │
│  设备: WTABC123456  文件: WTABC123456.csv [H ▼] │
│  设备: WTDEF789012  文件: WTDEF789012.csv [L6▼] │
│                                                │
│          [保存映射]    [取消]                   │
└────────────────────────────────────────────────┘
```

映射结果保存到 `device_alias_map.json`，格式：`{"WTABC123456": "H", "WTDEF789012": "L6"}`。
