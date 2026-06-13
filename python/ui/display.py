# -*- coding: utf-8 -*-
"""
结果展示 — 四个 tab 的填充逻辑
Results display helpers for basic / step / phase / joint tabs.

所有标签统一格式: 中文名 (English)
"""

import tkinter as tk


# ============================================================
# 字段名 → 中文名 映射
# ============================================================
_KEY_CN = {
    "walkingSpeed":   "步行速度 (Walking Speed)",
    "cadence":        "步频 (Cadence)",
    "strideTime":     "跨步时间 (Stride Time)",
    "stepLength":     "步长 (Step Length)",
    "stepTime":       "步时 (Step Time)",
    "validSteps":     "有效步数 (Valid Steps)",
    "footLiftHeight": "抬脚高度 (Foot Lift Height)",
    "flexion":        "屈曲/伸展 (Flexion/Extension)",
    "abduction":      "外展/内收 (Abduction/Adduction)",
    "rotation":       "旋转 (Rotation)",
}


# ============================================================
# Formatting helpers
# ============================================================
def format_val(value, decimals=3, unit=""):
    """Format a numeric value with optional unit, showing '—' for None."""
    if value is None:
        return "—"
    try:
        v = float(value)
        s = str(int(v)) if v == int(v) else f"{v:.{decimals}f}"
        return f"{s} {unit}".strip() if unit else s
    except (ValueError, TypeError):
        return str(value)


def _add_row(frame, row, label_text, value_text):
    """Add a label:value row to a frame at grid row.
    Column 0 gets a fixed minsize so values in column 1 all align left
    regardless of label length."""
    tk.Label(frame, text=label_text, anchor="w", justify="left",
             font=("Microsoft YaHei UI", 10)).grid(
        row=row, column=0, sticky="w", padx=(12, 8), pady=2)
    tk.Label(frame, text=value_text, anchor="w", justify="left",
             font=("Microsoft YaHei UI", 10, "bold")).grid(
        row=row, column=1, sticky="w", padx=(8, 12), pady=2)


def _add_sep(frame, row, text):
    """Add a section separator label."""
    tk.Label(frame, text=text, anchor="w",
             font=("Microsoft YaHei UI", 10, "bold"),
             fg="#2563EB").grid(
        row=row, column=0, columnspan=2, sticky="w", padx=12, pady=(8, 2))


def _add_lr(frame, row, data, key, unit="", decimals=3):
    """Display a nested left/right parameter. Returns number of rows used."""
    entry = data.get(key, {})
    if isinstance(entry, dict):
        left, right = entry.get("left"), entry.get("right")
    else:
        left = right = entry
    label_cn = _KEY_CN.get(key, key)
    _add_row(frame, row,     f"{label_cn} 左 (Left)",  format_val(left, decimals, unit))
    _add_row(frame, row + 1, f"{label_cn} 右 (Right)", format_val(right, decimals, unit))
    return 2


# ============================================================
# Public API
# ============================================================
def clear_results(tab_basic, tab_step, tab_phase):
    """Remove all widgets from the three result tabs."""
    for tab in [tab_basic, tab_step, tab_phase]:
        for w in tab.winfo_children():
            w.destroy()


def display_results(tab_basic, tab_step, tab_phase, data):
    """Populate the three result tabs from analysis data dict."""
    bp = data.get("basicParameters", {})
    sp = data.get("stepParameters", {})
    pp = data.get("phaseParameters", {})

    has_turns = bp.get("turnSteps", 0) > 0 or bp.get("turnDuration", 0) > 0

    next_row = _display_basic(tab_basic, bp, has_turns)
    _display_step(tab_step, sp)
    _display_phase(tab_phase, pp)

    # Meta info at bottom of basic tab
    version = data.get("version", "?")
    duration = data.get("sessionDuration", 0)
    ts = data.get("timestamp", 0)
    meta_text = f"版本 (Version) {version}  |  会话时长 (Duration) {format_val(duration, 1, 's')}  |  时间戳 (Timestamp) {ts}"
    tk.Label(tab_basic, text=meta_text,
             font=("Microsoft YaHei UI", 8), fg="#888888",
             anchor="w").grid(row=next_row, column=0, columnspan=2,
                              sticky="w", padx=12, pady=(12, 4))


# ============================================================
# Internal: per-tab layout
# ============================================================
def _display_basic(frame, bp, has_turns):
    r = [0]
    def row(inc=1):
        v = r[0]; r[0] += inc; return v

    _add_sep(frame, row(), "─ 速度 ─")
    _add_lr(frame, row(), bp, "walkingSpeed", "m/s", 3); row(2)

    _add_sep(frame, row(), "─ 步态周期 ─")
    _add_row(frame, row(), "步态周期 (Gait Cycle)", format_val(bp.get("gaitCycle"), 3, "s"))
    _add_lr(frame, row(), bp, "cadence", "steps/min", 1); row(2)
    _add_lr(frame, row(), bp, "strideTime", "s", 3); row(2)
    _add_row(frame, row(), "总步数 (Total Steps)", format_val(bp.get("totalSteps"), 0))

    _add_sep(frame, row(), "─ 转身 (Turn) ─")
    if has_turns:
        _add_row(frame, row(), "转身步数 (Turn Steps)", format_val(bp.get("turnSteps"), 0))
        _add_row(frame, row(), "转身时长 (Turn Duration)", format_val(bp.get("turnDuration"), 3, "s"))
    else:
        _add_row(frame, row(), "转身数据", "无腰部传感器 (S1)，已跳过")

    frame.grid_columnconfigure(0, minsize=240)
    return r[0]


def _display_step(frame, sp):
    r = [0]
    def row(inc=1):
        v = r[0]; r[0] += inc; return v

    _add_sep(frame, row(), "─ 步长 / 跨步 ─")
    _add_lr(frame, row(), sp, "stepLength", "m", 3); row(2)
    _add_row(frame, row(), "步长偏差 (Step Length Deviation)", format_val(sp.get("stepLengthDeviation"), 3, "m"))
    _add_row(frame, row(), "跨步长 (Stride Length)", format_val(sp.get("strideLength"), 3, "m"))

    _add_sep(frame, row(), "─ 步态时间 ─")
    _add_lr(frame, row(), sp, "stepTime", "s", 3); row(2)

    _add_sep(frame, row(), "─ 有效步数 ─")
    _add_lr(frame, row(), sp, "validSteps", "", 0); row(2)

    _add_sep(frame, row(), "─ 抬脚高度 ─")
    _add_lr(frame, row(), sp, "footLiftHeight", "m", 3); row(2)

    frame.grid_columnconfigure(0, minsize=240)


def _display_phase(frame, pp):
    r = [0]
    def row(inc=1):
        v = r[0]; r[0] += inc; return v

    _add_sep(frame, row(), "─ 相位参数 ─")

    sp = pp.get("supportPhase", {})
    sp_l = sp.get("left", 0) if isinstance(sp, dict) else sp
    sp_r = sp.get("right", 0) if isinstance(sp, dict) else sp
    sp_l_val = sp_l * 100 if isinstance(sp_l, (int, float)) and 0 < sp_l < 1 else sp_l
    sp_r_val = sp_r * 100 if isinstance(sp_r, (int, float)) and 0 < sp_r < 1 else sp_r
    _add_row(frame, row(), "支撑相 左 (Support Phase Left)", format_val(sp_l_val, 1, "%"))
    _add_row(frame, row(), "支撑相 右 (Support Phase Right)", format_val(sp_r_val, 1, "%"))

    sw = pp.get("swingPhase", {})
    sw_l = sw.get("left", 0) if isinstance(sw, dict) else sw
    sw_r = sw.get("right", 0) if isinstance(sw, dict) else sw
    sw_l_val = sw_l * 100 if isinstance(sw_l, (int, float)) and 0 < sw_l < 1 else sw_l
    sw_r_val = sw_r * 100 if isinstance(sw_r, (int, float)) and 0 < sw_r < 1 else sw_r
    _add_row(frame, row(), "摆动相 左 (Swing Phase Left)", format_val(sw_l_val, 1, "%"))
    _add_row(frame, row(), "摆动相 右 (Swing Phase Right)", format_val(sw_r_val, 1, "%"))

    dsp = pp.get("doubleSupportPhase", 0)
    dsp_val = dsp * 100 if isinstance(dsp, (int, float)) and 0 < dsp < 1 else dsp
    _add_row(frame, row(), "双支撑相 (Double Support Phase)", format_val(dsp_val, 1, "%"))

    frame.grid_columnconfigure(0, minsize=240)


def _display_joint_angles(frame, jp):
    """Display hip / knee / ankle angles."""
    r = [0]
    def row(inc=1):
        v = r[0]; r[0] += inc; return v

    # ── Hip ──
    hip = jp.get("hip", {}) if isinstance(jp, dict) else {}
    _add_sep(frame, row(), "─ 髋关节 (Hip) ─")
    _add_lr(frame, row(), hip, "flexion", "°", 1); row(2)
    _add_lr(frame, row(), hip, "abduction", "°", 1); row(2)
    _add_lr(frame, row(), hip, "rotation", "°", 1); row(2)

    # ── Knee ──
    knee = jp.get("knee", {}) if isinstance(jp, dict) else {}
    _add_sep(frame, row(), "─ 膝关节 (Knee) ─")
    _add_lr(frame, row(), knee, "flexion", "°", 1); row(2)
    _add_lr(frame, row(), knee, "abduction", "°", 1); row(2)
    _add_lr(frame, row(), knee, "rotation", "°", 1); row(2)

    # ── Ankle ──
    ankle = jp.get("ankle", {}) if isinstance(jp, dict) else {}
    _add_sep(frame, row(), "─ 踝关节 (Ankle) ─")
    _add_lr(frame, row(), ankle, "flexion", "°", 1); row(2)
    _add_lr(frame, row(), ankle, "abduction", "°", 1); row(2)
    _add_lr(frame, row(), ankle, "rotation", "°", 1); row(2)

    frame.grid_columnconfigure(0, minsize=240)
    return r[0]
