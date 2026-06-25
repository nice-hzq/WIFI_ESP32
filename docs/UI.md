# 步态分析系统 UI 设计文档

## 技术栈

| 项目 | 说明 |
|---|---|
| **GUI 框架** | Tkinter (Python 标准库) |
| **主题控件** | ttk.Combobox, ttk.Notebook, ttk.PanedWindow |
| **字体** | Microsoft YaHei UI / Consolas |
| **当前窗口** | 1020×780 px, 最小 880×600 px |
| **推荐窗口** | 1100×820 px, 最小 900×650 px |

## 源码结构

```
python/ui/
├── __init__.py   # 常量 (DEFAULT_BAUD, VALID_ALIASES, MODE_ALIASES, WORK_MODES, JOINT_OPTIONS)
├── app.py        # 主应用类 GaitAnalysisApp — 窗口布局、状态机、业务逻辑
├── threads.py    # 后台线程 — DataCollectionThread / GaitAnalysisThread / JointAngleThread
├── display.py    # 结果展示 — 四个 Tab 的数据填充
└── dialogs.py    # 弹窗 — 传感器映射对话框
```

入口：`python ui_app.py`

---

## 当前布局分析

### 现状

```
┌──────────────────────────────────────────────────────────┐
│  步态分析系统  |  Gait Analysis System        ● 就绪     │  ← 标题栏
├───────────────┬──────────────────────────────────────────┤
│ 左面板 320px  │  右面板 (expand)                         │
│               │                                          │
│ ┌─ DC ──────┐ │  ┌─────┐ ┌─────┐ ┌─────┐               │
│ │ 串口/波特  │ │  │速度  │ │步频  │ │步数  │   ← 汇总卡片 │
│ │ ▶ ■ ⟳    │ │  └─────┘ └─────┘ └─────┘               │
│ └───────────┘ │  ┌──────────────────────────────────┐   │
│ ┌─ JOINT ───┐ │  │ 📊 分析结果          [清除结果]  │   │
│ │ 关节选择   │ │  ├─基本参数─┬─步态参数─┬─相位参数─┤   │
│ │ 传感器绑定 │ │  │          │          │          │   │
│ │ 串口/波特  │ │  │  grid    │  grid    │  grid    │   │
│ │ 校准模式   │ │  │  layout  │  layout  │  layout  │   │
│ │ ▶ ■ ○    │ │  │          │          │          │   │
│ │ 状态×2    │ │  └──────────┴──────────┴──────────┘   │
│ └───────────┘ │  ┌─ 关节角度 ─────────────────────────┐  │
│ ┌─ HIST ────┐ │  │ 数值卡片 + 三条曲线面板              │  │
│ │ listbox   │ │  └────────────────────────────────────┘  │
│ │ 填入/打开  │ │                                          │
│ └───────────┘ │                                          │
│ ┌─ GA ──────┐│                                          │
│ │ 数据目录   ││                                          │
│ │ 映射状态   ││                                          │
│ │ 工作模式   ││                                          │
│ │ ▶ 运行    ││                                          │
│ └───────────┘│                                          │
├───────────────┴──────────────────────────────────────────┤
│  就绪 (Ready)                                ▓ 状态栏    │
└──────────────────────────────────────────────────────────┘
```

### 已识别的问题

| # | 问题 | 严重程度 | 说明 |
|---|------|---------|------|
| 1 | **左面板内容过多** | 🔴 高 | 4 张卡片垂直堆叠，关节角度卡片尤其拥挤（10+ 控件），用户需要频繁滚动 |
| 2 | **串口配置重复** | 🔴 高 | 数据采集和关节角度各自拥有串口+波特率控件，占用双倍空间且体验割裂 |
| 3 | **汇总卡片信息密度低** | 🟡 中 | 仅 3 个指标（速度/步频/步数），缺少步长、跨步时间、步态周期等关键参数 |
| 4 | **Tab 4 被隐藏在 Notebook 中** | 🟡 中 | 关节角度实时曲线放在第 4 个 Tab 中，测量时用户需要在 Tab 间切换才能同时看到数值和曲线 |
| 5 | **缺少可折叠区域** | 🟡 中 | 所有卡片始终完全展开，不能按需隐藏不用的功能模块 |
| 6 | **标题栏利用不足** | 🟢 低 | 仅显示标题+状态圆点，采集/分析进行中时缺少实时计数信息 |
| 7 | **状态栏信息单一** | 🟢 低 | 只有一行文本，未显示进度或耗时 |
| 8 | **汇总卡片视觉平淡** | 🟢 低 | 3 张卡片等大，无主次区分，缺少视觉层次 |

---

## 优化方案

### 优先级 P0: 串口配置统一化

**目标**: 消除数据采集和关节角度中重复的串口控件。

**方案**: 在左面板顶部新增 `🔌 连接 (Connection)` 卡片，统一管理串口和波特率。数据采集和关节角度共用此配置。

```
┌─ 🔌 连接 (Connection) ────────────────────────────┐
│  串口: [COM3 ▼] [↻]   波特率: [921600]             │
│  ──────────────────────────────────────────────── │
│  数据采集:  ● 未连接         [▶ 开始采集] [■ 停止] │
│  关节测量:  ● 未启动         [▶ 开始测量] [■ 停止] │
└──────────────────────────────────────────────────┘
```

**代码变更**:
- 新增 `_build_connection_card()` 方法
- 移除 `_build_dc_card()` 和 `_build_joint_card()` 中的串口/波特率行
- `port_var` / `baud_var` 提升为共享变量
- 两个线程使用同一个串口配置（互斥启动，已有状态机保证）

### 优先级 P1: 汇总卡片扩展

**目标**: 从 3 个指标扩展到 5-6 个，增加视觉层次。

**方案**: 第一行 4 个主指标，第二行 2-3 个辅助指标（可选，仅在有数据时显示）。

```
┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐
│ 步行速度  │ │  步频    │ │  步长    │ │ 跨步时间  │
│  1.25    │ │  108     │ │  0.68   │ │  1.12    │
│  m/s     │ │ steps/min│ │  m      │ │  s       │
└──────────┘ └──────────┘ └──────────┘ └──────────┘
┌──────────┐ ┌──────────┐
│  总步数   │ │ 步态周期  │
│   42     │ │  1.12    │
│  steps   │ │  s       │
└──────────┘ └──────────┘
```

**设计细节**:
- 主指标使用 `PRIMARY` 色 (#2563EB) 大号字体
- 辅助指标使用 `TEXT_MAIN` 色
- 每张卡片左侧添加 3px 彩色竖线作为视觉锚点
- 无数据时显示 "—" 占位符

**代码变更**:
- 新增 `summary_step_length_var`, `summary_stride_time_var`, `summary_gait_cycle_var`
- `_update_summary_cards()` 扩展字段
- `_create_summary_card()` 添加可选 `accent_color` 参数

### 优先级 P1: 左面板卡片可折叠

**目标**: 让用户按需展开/折叠功能卡片，减少滚动。

**方案**: 点击卡片标题行切换内容区域的显示/隐藏，标题旁显示 `▼` / `▶` 指示器。

```
┌─ 📡 数据采集 ─────────────────────────── ▼ ─┐
│  (内容可见)                                   │
└──────────────────────────────────────────────┘
┌─ 🦵 实时关节角度 ─────────────────────── ▶ ─┐
│  (内容折叠，仅标题可见)                       │
└──────────────────────────────────────────────┘
```

**代码变更**:
- `_create_card()` 返回 `(card, content_frame)` 元组
- 标题 Label 绑定 `<Button-1>` 切换事件
- 默认: DC 和 GA 展开，Joint 和 History 折叠
- 采集进行中时禁止折叠 DC 卡片

### 优先级 P2: 关节角度曲线实时可见

**目标**: 测量关节角度时，曲线面板自动弹出或切换到可见区域。

**方案 A (推荐)**: 测量启动时自动将 Notebook 切换到"关节角度"Tab。

**方案 B**: 在右面板底部新增一个可收起/展开的实时曲线面板（独立于 Notebook）。

推荐方案 A，实现简单且不增加布局复杂度。

**代码变更**:
- `_start_joint()` 中调用 `self.notebook.select(self.tab_joint_outer)` (需保存 outer frame 引用)

### 优先级 P2: 标题栏实时计数

**目标**: 采集/分析进行中时，标题栏显示实时数据行数或进度。

**方案**: 在标题栏状态徽章旁增加一个动态信息标签。

```
步态分析系统                              ● 采集中 | 1,280 行 | 3 设备
```

**代码变更**:
- 新增 `self.title_info_var = tk.StringVar()`
- `_build_title_bar()` 中在 badge 左侧添加 info label
- `_poll_queue()` 中 `data` 消息更新 info label
- `_transition_to(UIState.IDLE)` 时清空 info label

### 优先级 P3: 状态栏进度条

**目标**: 长时间分析时显示确定或不确定进度条。

**方案**: 在分析进行中时，状态栏显示不确定模式 (indeterminate) 的 `ttk.Progressbar`。

**代码变更**:
- 新增 `self.status_progress = ttk.Progressbar(status_bar, mode="indeterminate")`
- `_transition_to(UIState.ANALYZING)` 时 `pack` + `start(10)`
- `_on_gait_done` / `_on_gait_error` 时 `stop` + `pack_forget`

### 优先级 P3: 汇总卡片动画过渡

**目标**: 结果更新时数值有一个短暂的放大/缩小动画，吸引注意力。

**方案**: 使用 `after()` 逐步改变字体大小实现简单的弹跳效果。

**代码变更**:
- `_update_summary_cards()` 中先设置大字重（28pt），50ms 后恢复到 22pt

---

## 优化后整体布局

```
┌──────────────────────────────────────────────────────────────┐
│  步态分析系统                   1,280 行 | 3 设备  ● 采集中   │  ← 增强标题栏
├────────────────┬─────────────────────────────────────────────┤
│ 左面板 300px   │  右面板 (expand)                            │
│                │                                             │
│ ┌ CONNECTION ┐ │  ┌────────┐┌────────┐┌────────┐┌────────┐  │
│ │ 串口/波特   │ │  │ 步行速度││  步频  ││  步长  ││跨步时间 │  │
│ └────────────┘ │  │  1.25  ││  108   ││  0.68  ││  1.12   │  │
│ ┌ DC ── ▼ ───┐ │  │  m/s   ││steps/min││  m    ││   s    │  │
│ │ ▶ ■ ⟳     │ │  └────────┘└────────┘└────────┘└────────┘  │
│ │ 状态       │ │  ┌────────┐┌────────┐                      │
│ └────────────┘ │  │ 总步数  ││步态周期 │                      │
│ ┌ GA ── ▼ ───┐ │  │   42   ││  1.12   │                      │
│ │ 数据目录    │ │  │ steps  ││   s     │                      │
│ │ 映射/模式   │ │  └────────┘└────────┘                      │
│ │ ▶ 运行     │ │  ┌─────────────────────────────────────┐    │
│ └────────────┘ │  │ 📊 分析结果              [导出] [清除]│    │
│ ┌ JOINT ▶ ───┐ │  ├─基本参数─┬─步态参数─┬─相位参数─┬关节角度┤  │
│ │ (折叠)      │ │  │          │          │          │ 实时曲线│  │
│ └────────────┘ │  │  grid    │  grid    │  grid    │ 3面板  │  │
│ ┌ HIST ▶ ────┐ │  │  layout  │  layout  │  layout  │        │  │
│ │ (折叠)      │ │  └──────────┴──────────┴──────────┴────────┘  │
│ └────────────┘ │                                             │
├────────────────┴─────────────────────────────────────────────┤
│  [══════════════════════] 42%   数据采集: 1,280 行 | 3 设备   │  ← 增强状态栏
└──────────────────────────────────────────────────────────────┘
```

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

| 状态 | 开始采集 | 停止采集 | 新建会话 | 运行分析 | 清除结果 | 开始测量 | 停止测量 | 配置映射 |
|---|---|---|---|---|---|---|---|
| IDLE | ✅ | ❌ | ✅ | ✅ | ✅ | ✅ | ❌ | ✅ |
| COLLECTING | ❌ | ✅ | ❌ | ❌ | ❌ | ❌ | ❌ | ❌ |
| STOPPING | ❌ | ❌ | ❌ | ❌ | ❌ | ❌ | ❌ | ❌ |
| ANALYZING | ❌ | ❌ | ❌ | ❌ | ❌ | ❌ | ❌ | ❌ |

---

## 消息队列协议

后台线程通过 `queue.Queue` 与主线程通信：

| type | 方向 | 携带字段 | 触发动作 |
|---|---|---|---|
| `status` | 采集线程→UI | state, message, folder | 更新 dc_status_var |
| `data` | 采集线程→UI | total_rows, devices | 每 100 行更新计数 + 标题栏 |
| `error` | 采集线程→UI | message | 显示错误 |
| `done` | 采集线程→UI | total_rows, devices, folder | 记录会话 + 复位的 UI |
| `gait_status` | 分析线程→UI | message | 更新 ga_status_var |
| `gait_done` | 分析线程→UI | result, elapsed | 渲染结果到 4 个 tab + 汇总卡片 |
| `gait_error` | 分析线程→UI | message, traceback, elapsed | 弹窗显示错误 |
| `joint_status` | 关节线程→UI | state, message (, device_id, alias, proximal_id, distal_id, ...) | 更新 joint_status_var / 标定状态 / 设备映射显示 |
| `joint_angle` | 关节线程→UI | flexion_deg, abduction_deg, rotation_deg, ... | 更新数值 + 重绘曲线 |
| `joint_calib_status` | 关节线程→UI | message | 更新标定进度 |
| `joint_calib_done` | 关节线程→UI | joint, message | 标定完成 |
| `joint_calib_error` | 关节线程→UI | message | 标定失败提示 |
| `joint_error` | 关节线程→UI | message, traceback | 弹窗错误 + 复位 UI |

**🆕 数据自动保存**:
- 每次关节测量停止时，自动将完整角度数据保存到 `joint_csv/joint_angle_YYYYMMDD_HHMMSS/`
- 文件夹包含:
  - `joint_angle_data.csv` — 全分辨率角度时间序列 (time_s, flexion_deg, abduction_deg, rotation_deg)
  - `session_info.json` — 会话元数据 (关节名、传感器绑定、设备 ID、标定状态、ROM 等)

**`joint_status` 详细字段**:

| state | 额外字段 | 触发时机 | UI 处理 |
|---|---|---|---|
| `connecting` | — | 串口打开中 | 更新状态标签 |
| `connected` | — | 串口已连接 | 更新状态标签 |
| `device_detected` | `device_id`, `alias` | 检测到新设备上线 | 重新加载 device_map 并刷新传感器设备 ID 显示 |
| `identified` | `proximal_id`, `distal_id`, `proximal_alias`, `distal_alias` | 传感器识别完成 | 用结构化字段直接更新设备 ID 显示 |
| `devices` | — | 每 3 秒周期性报告 | 刷新设备 ID 显示 |
| `calibrated` | — | 加载已有标定或标定完成 | 更新标定状态为"已标定" |
| `uncalibrated` | — | 未找到标定文件 | 更新标定状态为"未标定" |
| `disconnected` | — | 串口断开 | 复位 UI |

---

## Section 1: 连接 (Connection) — 🆕 新增

**说明**: 统一的串口配置卡片，数据采集和关节角度共享。

| 控件 | 类型 | 变量 | 说明 |
|---|---|---|---|
| 串口选择 | `ttk.Combobox` | `self.port_var` | 自动枚举系统串口，优先 CH340/CP2102 |
| 刷新串口 | `tk.Button` | — | `↻` 重新扫描 COM 端口列表 |
| 波特率 | `tk.Entry` | `self.baud_var` | 默认 921600 |

**设计要点**:
- 下方用分割线隔开，分别显示数据采集和关节角度的启停按钮 + 状态
- 两个功能互斥启动（状态机已有保证）

---

## Section 2: 数据采集 (Data Collection)

**控件清单** (移除串口/波特率后):

| 控件 | 类型 | 变量 | 说明 |
|---|---|---|---|
| 开始采集 | `tk.Button` | `self.btn_start` | 绿色 (#22C55E)，触发 `_start_collection` |
| 停止采集 | `tk.Button` | `self.btn_stop` | 红色 (#EF4444)，默认 inactive |
| 新建会话 | `tk.Button` | `self.btn_new_session` | 清空结果 + 重置状态 |
| 状态标签 | `tk.Label` | `self.dc_status_var` | 显示连接/采集/完成 状态 |

**状态标识**: `● 未连接` → `◌ 连接中...` → `● 已连接 @ COMx` → `● 采集中 | N 行` → `✓ 采集完成`

**Watchdog**: 停止后 8 秒内线程未报告 done，强制复位 UI。

---

## Section 3: 步态分析 (Gait Analysis)

| 控件 | 类型 | 变量 | 说明 |
|---|---|---|---|
| 数据目录 | `tk.Entry` | `self.folder_var` | 选择 CSV 数据所在文件夹 |
| 浏览... | `tk.Button` | — | 系统文件夹选择对话框 |
| ← 最近 | `tk.Button` | `self.btn_use_recent` | 一键填入最近采集目录 |
| 传感器状态 | `tk.Label` | `self.map_status_var` | 绿色 ✓ / 橙色 ⚠ |
| 配置映射 | `tk.Button` | `self.btn_map` | 仅存在未映射设备时显示 |
| 工作模式 | `ttk.Combobox` | `self.mode_var` | `feet_only` / `lower_body` / `upper_body` / `full_body` |
| 运行分析 | `tk.Button` | `self.btn_analyze` | 蓝色 (#2563EB)，触发 `_run_analysis` |
| 状态 | `tk.Label` | `self.ga_status_var` | 就绪/运行中/完成/失败 |

**传感器自动检测逻辑**: 同 v1.0，自动推荐工作模式。

---

## Section 4: 实时关节角度 (Joint Angle) — 折叠优化

**控件清单** (移除串口/波特率后):

| 控件 | 类型 | 变量 | 说明 |
|---|---|---|---|
| 关节选择 | `ttk.Combobox` | `self.joint_var` | left_knee / right_knee / left_ankle / right_ankle / left_hip / right_hip |
| 传感器绑定显示 | `tk.Label` ×6 | `_joint_prox_label_var` / `_joint_prox_var` / `_joint_prox_device_var` / `_joint_dist_label_var` / `_joint_dist_var` / `_joint_dist_device_var` | 身体部位 + 别名 + `→` + 设备 ID (WT****) |
| 配置设备映射 | `tk.Button` | `self.btn_joint_device_map` | ⚙ 配置设备映射，打开映射对话框 |
| 校准模式 | `ttk.Combobox` | `self.calib_mode_var` | 下肢站立校准 / T-pose 全身校准 |
| 开始测量 | `tk.Button` | `self.btn_joint_start` | 绿色 |
| 停止测量 | `tk.Button` | `self.btn_joint_stop` | 红色 |
| 开始校准 | `tk.Button` | `self.btn_joint_calib` | 次要按钮 |
| 标定状态 | `tk.Label` | `self.joint_calib_status_var` | 未标定 / 标定中... / 已标定 ✓ / 标定失败 ✗ |
| 测量状态 | `tk.Label` | `self.joint_status_var` | 状态 + 实时角度值 |

**🆕 设备映射机制**:
- 传感器绑定显示格式: `大腿 (Thigh)  L4 → WT901AB12345`（身体部位 + 别名 + 箭头 + 设备 ID）
- 左列标签根据关节动态变化（如膝关节显示「大腿 (Thigh)」「小腿 (Shank)」，踝关节显示「小腿 (Shank)」「脚掌 (Foot)」）
- 无映射时设备 ID 显示 `—`
- 映射持久化到 `python/temp/device_alias_map.json`（`{device_id: alias}` 格式）
- 切换关节时自动从已保存映射中解析设备名
- 启动测量时映射传递给 `JointAngleThread`，优先匹配，未匹配时按上线顺序自动分配
- 测量中检测到设备上线/识别时实时更新设备 ID 显示
- 非 IDLE 状态下 `⚙ 配置设备映射` 按钮禁用

**🆕 交互优化**:
- **默认折叠**: 不使用关节测量时，卡片折叠仅显示标题
- **自动展开**: 点击 `▶ 开始测量` 时自动展开卡片；停止后不自动折叠
- **自动切Tab**: 测量启动时 Notebook 自动切换到"关节角度" Tab 以显示实时曲线

---

## Section 5: 最近会话 (Recent Sessions)

| 控件 | 类型 | 说明 |
|---|---|---|
| 会话列表 | `tk.Listbox` (height=3) | 显示最近采集的会话 |
| 填入分析目录 | `tk.Button` | 将选中路径填入"数据目录" |
| 打开文件夹 | `tk.Button` | 在资源管理器中打开 |
| 状态 | `tk.Label` | "暂无历史会话" 或 "共 N 个会话" |

**🆕 交互优化**: 双击列表项 = 填入分析目录

---

## Section 6: 汇总卡片 (Summary Cards) — 🆕 扩展

**优化后布局**:

| 行 | 卡片 | 变量 | 颜色 |
|---|---|---|---|
| 第 1 行 | 🏃 步行速度 (Walking Speed) | `summary_speed_var` | PRIMARY |
| 第 1 行 | 🔄 步频 (Cadence) | `summary_cadence_var` | #E74C3C |
| 第 1 行 | 📏 步长 (Step Length) | `summary_step_length_var` | #2ECC71 |
| 第 1 行 | ⏱ 跨步时间 (Stride Time) | `summary_stride_time_var` | #3498DB |
| 第 2 行 | 👣 总步数 (Total Steps) | `summary_steps_var` | TEXT_MAIN |
| 第 2 行 | 🔁 步态周期 (Gait Cycle) | `summary_gait_cycle_var` | TEXT_MAIN |

**步长取左右平均值**；无数据时所有卡片显示 `—`。

**视觉增强**:
- 每张卡片左侧 3px 彩色竖线 (使用 `Canvas` 或 `Frame` 模拟)
- 数值使用 22pt bold，单位 9pt
- 卡片间间距 8px

---

## Section 7: 分析结果 (Results)

### 结构

```
┌─────────────────────────────────────────────┐
│  📊 分析结果 (Results)    [导出] [清除结果]  │  ← 🆕 新增导出按钮
├─ 基本参数 ─┬─ 步态参数 ─┬─ 相位参数 ─┬─ 关节角度 ─┤
│            │            │            │            │
│ Canvas +   │ Canvas +   │ Canvas +   │ 数值卡片   │
│ Scrollbar  │ Scrollbar  │ Scrollbar  │ + 3 曲线   │
│            │            │            │            │
└────────────┴────────────┴────────────┴────────────┘
```

### Tab 1: 基本参数 (Basic Parameters)

| 分类 | 字段 | 左右 |
|---|---|---|
| 速度 | 步行速度 (Walking Speed) | L / R |
| 步态周期 | 步态周期 (Gait Cycle) | — |
| | 步频 (Cadence) | L / R |
| | 跨步时间 (Stride Time) | L / R |
| | 总步数 (Total Steps) | — |
| 转身 | 转身步数 / 转身时长 | 有 S1 时显示 |

### Tab 2: 步态参数 (Step Parameters)

| 分类 | 字段 | 左右 |
|---|---|---|
| 步长/跨步 | 步长 (Step Length) | L / R |
| | 步长偏差 (Step Length Deviation) | — |
| | 跨步长 (Stride Length) | — |
| 步态时间 | 步时 (Step Time) | L / R |
| 有效步数 | 有效步 (Valid Steps) | L / R |
| 抬脚高度 | 抬脚高度 (Foot Lift Height) | L / R |

### Tab 3: 相位参数 (Phase Parameters)

| 字段 | 左右 |
|---|---|
| 支撑相 (Support Phase) | L / R |
| 摆动相 (Swing Phase) | L / R |
| 双支撑相 (Double Support) | — |

### Tab 4: 关节角度 (Joint Angles) — 🆕 增强

| 区域 | 内容 |
|---|---|
| 数值卡片行 | 屈曲/伸展、外展/内收、内旋/外旋、ROM 的当前值 (大字) + 最大值/最小值 |
| 曲线面板 ×3 | Flexion / Abduction / Rotation 实时曲线，共享时间轴 |

---

## 传感器映射弹窗

### 步态分析映射 (`open_mapping_dialog`)

与 v1.0 相同。用于 CSV 离线数据场景，扫描文件夹中的 CSV 文件，将未识别的设备 ID 映射到身体别名。

### 关节设备映射 (`open_joint_device_mapping_dialog`)

用于实时串口关节角度测量场景，在 `python/ui/dialogs.py` 中定义。

**触发入口**: 关节卡片中的 `⚙ 配置设备映射` 按钮

**对话框布局**:
```
┌─────────────────────────────────────────────────┐
│  将设备 ID (WT****) 映射到身体传感器别名          │
│  当前关节: 左膝 (Left Knee)  |  大腿 (Thigh)=L4  小腿 (Shank)=L5  │
│  输入设备 ID，然后选择对应的身体位置。             │
│                                                  │
│  ┌──────────────────┬──────────────┬────────┐   │
│  │ 设备 ID (WT****)  │ 传感器别名    │ 操作    │   │
│  ├──────────────────┼──────────────┼────────┤   │
│  │ WT901AB12345     │ [L4 ▼]      │ ✕ 删除  │   │
│  │ WT901AB67890     │ [L5 ▼]      │ ✕ 删除  │   │
│  └──────────────────┴──────────────┴────────┘   │
│                                                  │
│  [＋ 添加设备]  [填入 大腿 (Thigh)(L4)+小腿 (Shank)(L5)]            │
│                                                  │
│  [保存映射]  [取消]                                │
└─────────────────────────────────────────────────┘
```

**功能特性**:
- 可编辑的表格: 设备 ID 使用 `tk.Entry`（支持手动输入 WT 编号），别名使用 `ttk.Combobox`（`VALID_ALIASES`）
- `＋ 添加设备`: 新增空行
- `填入 L4+L5`: 快捷填充当前关节需要的两个身体部位别名
- `✕ 删除`: 删除该行映射
- 保存时验证: 设备 ID 和别名必须同时填写、同一别名不能重复
- 持久化到 `python/temp/device_alias_map.json`
- 保存后自动回调 UI 刷新传感器设备 ID 显示

**与步态分析映射的区别**:

| 特性 | 步态分析映射 | 关节设备映射 |
|---|---|---|
| 数据来源 | CSV 文件扫描 | 用户手动输入 |
| 设备 ID 来源 | CSV 文件内容自动提取 | 传感器外壳 WT 编号 |
| 持久化位置 | `data/wt901_data_*/device_alias_map.json` | `python/temp/device_alias_map.json` |
| 使用场景 | 离线分析 | 实时测量 |

---

## 实施路线图

| 阶段 | 内容 | 预计改动 |
|---|---|---|
| **Phase 1** (P0) | 串口配置统一为 Connection 卡片 | `app.py` ~60 行 |
| **Phase 2** (P1) | 汇总卡片扩展至 6 个 + 视觉增强 | `app.py` ~50 行 |
| **Phase 3** (P1) | 左面板卡片可折叠 | `app.py` ~40 行 |
| **Phase 4** (P2) | 标题栏实时计数 + 关节测量自动切Tab | `app.py` ~30 行 |
| **Phase 5** (P2) | 状态栏进度条 | `app.py` ~25 行 |
| **Phase 6** (P3) | 汇总卡片动画 + 双击历史列表 + 导出按钮 | `app.py` ~40 行 |

总计预计改动约 250 行，主要集中在 `app.py` 的 `_build_ui()` 和相关方法中。

---

## 设计原则

1. **渐进增强**: 所有新功能有合理的默认行为，不影响现有工作流
2. **信息密度**: 常用信息一眼可见，次要信息按需展开
3. **视觉层次**: 颜色、字号、间距形成清晰的信息层级
4. **状态可见**: 系统状态（采集/分析/空闲/错误）始终清晰可辨
5. **容错设计**: 线程异常、串口断开等边缘情况有合理的 UI 反馈
