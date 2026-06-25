# -*- coding: utf-8 -*-
"""
主应用类 — 步态分析系统 UI
Main GaitAnalysisApp with state machine, session management, and queue polling.

Layout: left-right split with title bar and status bar.
Left  (320px): Connection | Data Collection | Gait Analysis | Joint Angle | History
Right (expand): 6 Summary Cards (2 rows) | Result Notebook (4 tabs)
"""

import os, queue, threading, time
from datetime import datetime
from enum import Enum

import tkinter as tk
from tkinter import ttk, messagebox, filedialog

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

from . import (DEFAULT_BAUD, WORK_MODES, VALID_ALIASES, MODE_ALIASES,
                get_base_dir, JOINT_OPTIONS, CALIB_MODES, CALIB_STATES)
from .threads import DataCollectionThread, GaitAnalysisThread, JointAngleThread
from .dialogs import scan_csv_folder, load_alias_map, open_mapping_dialog
from .display import clear_results, display_results


# ============================================================
# Style Constants
# ============================================================
# ── Colors ──
BACKGROUND  = "#F3F6FA"
CARD_BG     = "#FFFFFF"
PRIMARY     = "#2563EB"
PRIMARY_HOVER = "#1D4ED8"
SUCCESS     = "#22C55E"
SUCCESS_HOVER = "#16A34A"
SUCCESS_MUTED = "#BBF7D0"   # inactive state
WARNING     = "#F59E0B"
DANGER      = "#EF4444"
DANGER_HOVER = "#DC2626"
DANGER_MUTED = "#FECACA"    # inactive state
PRIMARY_MUTED = "#BFDBFE"   # inactive state
TEXT_MAIN    = "#111827"
TEXT_SECONDARY = "#6B7280"
BORDER       = "#E5E7EB"
DISABLED_BG  = "#E5E7EB"
DISABLED_FG  = "#9CA3AF"

# ── Badge Colors ──
BADGE_GREEN_BG   = "#DCFCE7"
BADGE_GREEN_TEXT = "#166534"
BADGE_RED_BG     = "#FEE2E2"
BADGE_RED_TEXT   = "#991B1B"
BADGE_ORANGE_BG  = "#FFF7ED"
BADGE_ORANGE_TEXT = "#9A3412"
BADGE_BLUE_BG    = "#DBEAFE"
BADGE_BLUE_TEXT  = "#1E40AF"
BADGE_GRAY_BG    = "#F3F4F6"
BADGE_GRAY_TEXT  = "#374151"

# ── Fonts ──
FONT_TITLE      = ("Microsoft YaHei UI", 15, "bold")
FONT_SUBTITLE   = ("Microsoft YaHei UI", 9)
FONT_SECTION    = ("Microsoft YaHei UI", 11, "bold")
FONT_BODY       = ("Microsoft YaHei UI", 10)
FONT_BODY_BOLD  = ("Microsoft YaHei UI", 10, "bold")
FONT_SMALL      = ("Microsoft YaHei UI", 8)
FONT_MONO       = ("Consolas", 9)
FONT_MONO_SM    = ("Consolas", 8)
FONT_VALUE      = ("Microsoft YaHei UI", 22, "bold")
FONT_VALUE_UNIT = ("Microsoft YaHei UI", 9)
FONT_BADGE      = ("Microsoft YaHei UI", 9, "bold")

# ── Dimensions ──
LEFT_PANEL_WIDTH = 320


# ============================================================
# State Machine
# ============================================================
class UIState(Enum):
    IDLE       = "idle"
    COLLECTING = "collecting"
    STOPPING   = "stopping"
    ANALYZING  = "analyzing"


# ============================================================
# Main Application
# ============================================================
class GaitAnalysisApp:
    def __init__(self, root):
        self.root = root
        self.root.title("步态分析系统 — Gait Analysis System")
        self.root.geometry("1020x780")
        self.root.minsize(880, 600)
        self.root.configure(bg=BACKGROUND)

        self.queue = queue.Queue(maxsize=200)
        self.collection_thread = None
        self.analysis_thread = None
        self.joint_thread = None          # 关节角度测量线程
        self.state = UIState.IDLE
        self.last_result = None
        self._scanned_devices = {}
        self._watchdog_id = None

        self._session_history = []
        self._last_collection_folder = None

        # ── Joint Angle state ──
        self._joint_calib_state = "uncalibrated"  # uncalibrated | calibrating | calibrated | failed
        self._joint_flexion_var = tk.StringVar(value="--")
        self._joint_abduction_var = tk.StringVar(value="--")
        self._joint_rotation_var = tk.StringVar(value="--")
        self._joint_max_var = tk.StringVar(value="--")
        self._joint_min_var = tk.StringVar(value="--")
        self._joint_rom_var = tk.StringVar(value="--")
        self._joint_history_t = []
        self._joint_history_flexion = []
        self._joint_history_abduction = []
        self._joint_history_rotation = []

        # ── Joint sensor segment labels ──
        self._joint_prox_label_var = tk.StringVar(value="大腿 (Thigh)")
        self._joint_dist_label_var = tk.StringVar(value="小腿 (Shank)")
        self._joint_prox_var = tk.StringVar(value="L4")
        self._joint_dist_var = tk.StringVar(value="L5")

        # ── Joint device mapping ──
        self._joint_prox_device_var = tk.StringVar(value="—")
        self._joint_dist_device_var = tk.StringVar(value="—")
        self._joint_device_map = {}  # {device_id: alias} 缓存

        # ── Summary card StringVars ──
        self.summary_speed_var = tk.StringVar(value="--")
        self.summary_cadence_var = tk.StringVar(value="--")
        self.summary_steps_var = tk.StringVar(value="--")
        self.summary_speed_unit = tk.StringVar(value="m/s")
        self.summary_step_length_var = tk.StringVar(value="--")
        self.summary_stride_time_var = tk.StringVar(value="--")
        self.summary_gait_cycle_var = tk.StringVar(value="--")

        # ── Title bar status ──
        self.title_status_var = tk.StringVar(value="● 就绪")
        self.title_info_var = tk.StringVar(value="")

        self._build_ui()
        self._init_button_states()
        self._poll_queue()

    # ============================================================
    # UI Helpers
    # ============================================================
    def _create_card(self, parent, title=None, collapsible=False,
                     start_collapsed=False, **pack_opts):
        """Create a white card Frame with optional section title.

        Returns (card, content) — always pack child widgets into content.
        When collapsible=True, clicking the title toggles content visibility.
        """
        card = tk.Frame(parent, bg=CARD_BG, highlightbackground=BORDER,
                        highlightthickness=1, bd=0)
        card.pack(fill="x", pady=(0, 8), **pack_opts)

        content = tk.Frame(card, bg=CARD_BG)

        if title and collapsible:
            title_frame = tk.Frame(card, bg=CARD_BG, cursor="hand2")
            title_frame.pack(fill="x")

            indicator = "▶  " if start_collapsed else "▼  "
            title_lbl = tk.Label(title_frame, text=f"{indicator}{title}",
                                 font=FONT_SECTION, bg=CARD_BG, fg=TEXT_MAIN,
                                 anchor="w")
            title_lbl.pack(fill="x", padx=12, pady=(10, 6))

            card._collapsed = start_collapsed

            def _toggle(e=None):
                if card._collapsed:
                    content.pack(fill="x")
                    title_lbl.config(text=f"▼  {title}")
                    card._collapsed = False
                else:
                    content.pack_forget()
                    title_lbl.config(text=f"▶  {title}")
                    card._collapsed = True

            title_lbl.bind("<Button-1>", _toggle)
            title_frame.bind("<Button-1>", _toggle)

            if not start_collapsed:
                content.pack(fill="x")
        elif title:
            tk.Label(card, text=title, font=FONT_SECTION,
                     bg=CARD_BG, fg=TEXT_MAIN, anchor="w").pack(
                fill="x", padx=12, pady=(10, 6))
            content.pack(fill="x")
        else:
            content.pack(fill="x")

        return card, content

    def _make_action_btn(self, parent, text, command, bg_color, hover_color,
                         width=14):
        """Create a colored action button (always normal state; visual state
        is managed by _set_action_btn_state)."""
        btn = tk.Button(parent, text=text, width=width,
                        font=FONT_BODY, bg=bg_color, fg="white",
                        activebackground=hover_color, activeforeground="white",
                        bd=0, padx=12, pady=5,
                        relief="flat", cursor="hand2",
                        command=command)
        return btn

    def _set_action_btn_state(self, btn, active, active_text, inactive_text,
                               active_bg, muted_bg, muted_fg="#6B7280"):
        """Toggle an action button between active (full color) and inactive
        (muted/lighter color). Never uses Tk 'disabled' state to avoid
        the hard-to-read gray-out effect."""
        if active:
            btn.configure(text=active_text, bg=active_bg, fg="white",
                          activebackground=active_bg, cursor="hand2")
        else:
            btn.configure(text=inactive_text, bg=muted_bg, fg=muted_fg,
                          activebackground=muted_bg, cursor="arrow")

    def _make_secondary_btn(self, parent, text, command, width=12, font=None, state="normal"):
        """Create a white bordered button for secondary actions."""
        if font is None:
            font = FONT_BODY
        btn = tk.Button(parent, text=text, width=width,
                        font=font, bg=CARD_BG, fg=TEXT_MAIN,
                        activebackground="#F9FAFB",
                        bd=0, padx=8, pady=3,
                        relief="flat", cursor="hand2",
                        highlightbackground=BORDER, highlightthickness=1,
                        command=command, state=state)
        return btn

    def _make_small_btn(self, parent, text, command, width=10, state="normal"):
        """Create a small secondary button."""
        return self._make_secondary_btn(parent, text, command,
                                        width=width, font=FONT_SMALL, state=state)

    def _create_summary_card(self, parent, label_cn, label_en, value_var, unit_var,
                              accent_color=None):
        """Create a single metric summary card with optional left accent strip."""
        card = tk.Frame(parent, bg=CARD_BG, highlightbackground=BORDER,
                        highlightthickness=1, bd=0)
        card.pack(side="left", padx=(0, 8), pady=4)

        # Left accent strip
        if accent_color:
            accent = tk.Frame(card, bg=accent_color, width=3)
            accent.pack(side="left", fill="y")

        inner = tk.Frame(card, bg=CARD_BG)
        inner.pack(side="left", fill="both", expand=True, padx=12, pady=10)

        tk.Label(inner, text=label_cn, font=FONT_BODY, bg=CARD_BG,
                 fg=TEXT_MAIN, anchor="center").pack()

        val_frame = tk.Frame(inner, bg=CARD_BG)
        val_frame.pack()
        # Use accent_color for value if provided, else PRIMARY
        val_color = accent_color if accent_color else PRIMARY
        tk.Label(val_frame, textvariable=value_var, font=FONT_VALUE,
                 bg=CARD_BG, fg=val_color, anchor="center").pack(side="left")
        tk.Label(val_frame, textvariable=unit_var, font=FONT_VALUE_UNIT,
                 bg=CARD_BG, fg=TEXT_SECONDARY, anchor="s").pack(
            side="left", padx=(2, 0))

        tk.Label(inner, text=label_en, font=FONT_SMALL, bg=CARD_BG,
                 fg=TEXT_SECONDARY, anchor="center").pack()

        return card

    # ============================================================
    # UI Build
    # ============================================================
    def _build_ui(self):
        # ── Title Bar ──
        self._build_title_bar()

        # ── Main content: left-right split ──
        paned = tk.PanedWindow(self.root, orient=tk.HORIZONTAL,
                               bg=BACKGROUND, bd=0, sashwidth=2,
                               sashrelief="flat")
        paned.pack(fill="both", expand=True, padx=8, pady=(0, 8))

        # Left panel
        left_frame = tk.Frame(paned, bg=BACKGROUND, width=LEFT_PANEL_WIDTH,
                              height=100)
        paned.add(left_frame, minsize=280)

        # Right panel
        right_frame = tk.Frame(paned, bg=BACKGROUND)
        paned.add(right_frame)

        # Build left (control) cards
        self._build_connection_card(left_frame)
        self._build_dc_card(left_frame)
        self._build_ga_card(left_frame)
        self._build_joint_card(left_frame)
        self._build_history_card(left_frame)

        # Build right (results) area
        self._build_right_panel(right_frame)

        # ── Status Bar ──
        self._build_status_bar()

    def _build_title_bar(self):
        """Build the top title bar with system name and status badge."""
        title_bar = tk.Frame(self.root, bg=CARD_BG, highlightbackground=BORDER,
                             highlightthickness=1, bd=0)
        title_bar.pack(fill="x", padx=0, pady=0)

        title_inner = tk.Frame(title_bar, bg=CARD_BG)
        title_inner.pack(fill="x", padx=16, pady=(10, 6))

        # Left: title + subtitle
        title_col = tk.Frame(title_inner, bg=CARD_BG)
        title_col.pack(side="left")
        tk.Label(title_col, text="步态分析系统", font=FONT_TITLE,
                 bg=CARD_BG, fg=TEXT_MAIN).pack(anchor="w")
        tk.Label(title_col,
                 text="Wearable Sensor-Based Gait Analysis System",
                 font=FONT_SUBTITLE, bg=CARD_BG,
                 fg=TEXT_SECONDARY).pack(anchor="w")

        # Center: real-time info (rows / devices / progress)
        self.title_info_lbl = tk.Label(
            title_inner, textvariable=self.title_info_var,
            font=FONT_SMALL, bg=CARD_BG, fg=TEXT_SECONDARY)
        self.title_info_lbl.pack(side="right", padx=(0, 12))

        # Right: status badge
        self.title_status_badge = tk.Label(
            title_inner, textvariable=self.title_status_var,
            font=FONT_BADGE, bg=BADGE_GRAY_BG, fg=BADGE_GRAY_TEXT,
            padx=12, pady=4)
        self.title_status_badge.pack(side="right")

    def _build_connection_card(self, parent):
        """Build the unified Connection card (shared serial port + baud rate)."""
        card, content = self._create_card(parent, title="🔌 连接 (Connection)")

        conn_row = tk.Frame(content, bg=CARD_BG)
        conn_row.pack(fill="x", padx=12, pady=(4, 8))

        tk.Label(conn_row, text="串口", font=FONT_BODY, bg=CARD_BG,
                 fg=TEXT_SECONDARY, width=4, anchor="w").pack(side="left")

        available_ports = [p.device for p in serial.tools.list_ports.comports()] if serial else []
        default_port = ""
        if serial:
            for p in serial.tools.list_ports.comports():
                if "CH340" in p.description or "CP210" in p.description or "USB" in p.description:
                    default_port = p.device
                    break
            if not default_port and available_ports:
                default_port = available_ports[0]

        self.port_var = tk.StringVar(value=default_port)
        self.port_cb = ttk.Combobox(conn_row, textvariable=self.port_var,
                                     values=available_ports, state="readonly", width=14,
                                     font=FONT_MONO)
        self.port_cb.pack(side="left", padx=(2, 2))

        def _refresh_ports():
            if not serial:
                return
            ports = [p.device for p in serial.tools.list_ports.comports()]
            self.port_cb["values"] = ports
            if ports and self.port_var.get() not in ports:
                self.port_var.set(ports[0])

        self._make_small_btn(conn_row, "↻", _refresh_ports, width=3).pack(
            side="left", padx=(0, 12))

        tk.Label(conn_row, text="波特率", font=FONT_BODY, bg=CARD_BG,
                 fg=TEXT_SECONDARY, width=5, anchor="w").pack(side="left")
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUD))
        tk.Entry(conn_row, textvariable=self.baud_var, width=8,
                 font=FONT_MONO, bg=CARD_BG, fg=TEXT_MAIN,
                 relief="solid", highlightbackground=BORDER,
                 highlightthickness=1, bd=0).pack(side="left", padx=(2, 0))

    def _build_dc_card(self, parent):
        """Build the Data Collection card."""
        card, content = self._create_card(parent, title="📡 数据采集 (Data Collection)",
                                         collapsible=True)

        # Control buttons
        btn_row = tk.Frame(content, bg=CARD_BG)
        btn_row.pack(fill="x", padx=12, pady=(0, 6))

        self.btn_start = self._make_action_btn(
            btn_row, "▶  开始采集", self._start_collection, SUCCESS, SUCCESS_HOVER)
        self.btn_start.pack(side="left", padx=(0, 6))

        self.btn_stop = self._make_action_btn(
            btn_row, "■  停止采集", self._stop_collection, DANGER, DANGER_HOVER)
        self.btn_stop.pack(side="left", padx=(0, 6))

        self.btn_new_session = self._make_secondary_btn(
            btn_row, "⟳ 新建会话", self._new_session, width=10)
        self.btn_new_session.pack(side="right")

        # Status
        self.dc_status_var = tk.StringVar(value="● 未连接")
        tk.Label(content, textvariable=self.dc_status_var,
                 font=FONT_SMALL, bg=CARD_BG, fg=TEXT_SECONDARY,
                 anchor="w").pack(fill="x", padx=12, pady=(0, 8))

    def _build_joint_card(self, parent):
        """Build the Joint Angle (real-time) card."""
        card, content = self._create_card(parent, title="🦵 实时关节角度 (Joint Angle)",
                                         collapsible=True, start_collapsed=True)

        # ── Joint selection ──
        sel_row = tk.Frame(content, bg=CARD_BG)
        sel_row.pack(fill="x", padx=12, pady=(4, 4))
        tk.Label(sel_row, text="关节", font=FONT_BODY, bg=CARD_BG,
                 fg=TEXT_SECONDARY, width=4, anchor="w").pack(side="left")
        self.joint_var = tk.StringVar(value="left_knee")
        joint_keys = list(JOINT_OPTIONS.keys())
        self.joint_cb = ttk.Combobox(sel_row, textvariable=self.joint_var,
                                     values=joint_keys, state="readonly", width=18,
                                     font=FONT_BODY)
        self.joint_cb.pack(side="left", padx=2)
        # show label instead of key
        def _on_joint_change(*args):
            key = self.joint_var.get()
            info = JOINT_OPTIONS.get(key, {})
            self._joint_prox_label_var.set(f"{info.get('prox_label', '?')}")
            self._joint_dist_label_var.set(f"{info.get('dist_label', '?')}")
            self._joint_prox_var.set(f"{info.get('proximal', '?')}")
            self._joint_dist_var.set(f"{info.get('distal', '?')}")
            # 重置设备 ID 显示并尝试从已保存映射中解析
            self._joint_prox_device_var.set("—")
            self._joint_dist_device_var.set("—")
            self._update_joint_device_display()
        self.joint_var.trace_add("write", _on_joint_change)

        # ── Sensor binding display (segment label + alias + device ID) ──
        # Upper segment row
        prox_row = tk.Frame(content, bg=CARD_BG)
        prox_row.pack(fill="x", padx=12, pady=(2, 2))
        tk.Label(prox_row, textvariable=self._joint_prox_label_var, font=FONT_SMALL, bg=CARD_BG,
                 fg=TEXT_SECONDARY, width=12, anchor="w").pack(side="left")
        tk.Label(prox_row, textvariable=self._joint_prox_var, font=FONT_BODY_BOLD,
                 bg=CARD_BG, fg=PRIMARY).pack(side="left", padx=(2, 4))
        tk.Label(prox_row, text="→", font=FONT_SMALL, bg=CARD_BG,
                 fg=TEXT_SECONDARY).pack(side="left", padx=(0, 4))
        tk.Label(prox_row, textvariable=self._joint_prox_device_var, font=FONT_MONO_SM,
                 bg=CARD_BG, fg=TEXT_MAIN).pack(side="left")

        # Lower segment row
        dist_row = tk.Frame(content, bg=CARD_BG)
        dist_row.pack(fill="x", padx=12, pady=(2, 2))
        tk.Label(dist_row, textvariable=self._joint_dist_label_var, font=FONT_SMALL, bg=CARD_BG,
                 fg=TEXT_SECONDARY, width=12, anchor="w").pack(side="left")
        tk.Label(dist_row, textvariable=self._joint_dist_var, font=FONT_BODY_BOLD,
                 bg=CARD_BG, fg=PRIMARY).pack(side="left", padx=(2, 4))
        tk.Label(dist_row, text="→", font=FONT_SMALL, bg=CARD_BG,
                 fg=TEXT_SECONDARY).pack(side="left", padx=(0, 4))
        tk.Label(dist_row, textvariable=self._joint_dist_device_var, font=FONT_MONO_SM,
                 bg=CARD_BG, fg=TEXT_MAIN).pack(side="left")

        # ── Device mapping config button ──
        cfg_row = tk.Frame(content, bg=CARD_BG)
        cfg_row.pack(fill="x", padx=12, pady=(4, 4))
        self.btn_joint_device_map = self._make_small_btn(
            cfg_row, "⚙ 配置设备映射", self._open_joint_device_mapping, width=18)
        self.btn_joint_device_map.pack(side="left")

        # ── Calibration mode ──
        cal_row = tk.Frame(content, bg=CARD_BG)
        cal_row.pack(fill="x", padx=12, pady=(2, 2))
        tk.Label(cal_row, text="校准", font=FONT_BODY, bg=CARD_BG,
                 fg=TEXT_SECONDARY, width=4, anchor="w").pack(side="left")
        self.calib_mode_var = tk.StringVar(value="lower_body_standing")
        calib_mode_labels = list(CALIB_MODES.values())
        calib_mode_keys = list(CALIB_MODES.keys())
        self.calib_mode_cb = ttk.Combobox(cal_row, textvariable=self.calib_mode_var,
                                          values=calib_mode_keys, state="readonly",
                                          width=16, font=FONT_BODY)
        self.calib_mode_cb.pack(side="left", padx=2)

        # ── Control buttons ──
        btn_row = tk.Frame(content, bg=CARD_BG)
        btn_row.pack(fill="x", padx=12, pady=(4, 2))

        self.btn_joint_start = self._make_action_btn(
            btn_row, "▶  开始测量", self._start_joint, SUCCESS, SUCCESS_HOVER, width=11)
        self.btn_joint_start.pack(side="left", padx=(0, 4))

        self.btn_joint_stop = self._make_action_btn(
            btn_row, "■  停止测量", self._stop_joint, DANGER, DANGER_HOVER, width=11)
        self.btn_joint_stop.pack(side="left", padx=(0, 4))

        self.btn_joint_calib = self._make_secondary_btn(
            btn_row, "○ 开始校准", self._start_joint_calib, width=10)
        self.btn_joint_calib.pack(side="left")

        # ── Calibration status ──
        self.joint_calib_status_var = tk.StringVar(value="未标定")
        tk.Label(content, textvariable=self.joint_calib_status_var,
                 font=FONT_SMALL, bg=CARD_BG, fg=WARNING,
                 anchor="w").pack(fill="x", padx=12, pady=(2, 0))

        # ── Measurement status ──
        self.joint_status_var = tk.StringVar(value="● 未启动")
        tk.Label(content, textvariable=self.joint_status_var,
                 font=FONT_SMALL, bg=CARD_BG, fg=TEXT_SECONDARY,
                 anchor="w").pack(fill="x", padx=12, pady=(0, 6))

    def _build_history_card(self, parent):
        """Build the Recent Sessions card."""
        card, content = self._create_card(parent, title="📋 最近会话 (Recent Sessions)",
                                         collapsible=True, start_collapsed=True)

        list_frame = tk.Frame(content, bg=CARD_BG)
        list_frame.pack(fill="x", padx=12, pady=(2, 2))

        self.hist_listbox = tk.Listbox(list_frame, height=3, font=FONT_MONO_SM,
                                        bg="#FAFBFC", fg=TEXT_MAIN,
                                        selectbackground="#DBEAFE",
                                        selectforeground=TEXT_MAIN,
                                        relief="solid",
                                        highlightbackground=BORDER,
                                        highlightthickness=1,
                                        activestyle="none")
        self.hist_listbox.pack(side="left", fill="x", expand=True)
        # Double-click to fill analysis folder
        self.hist_listbox.bind("<Double-Button-1>", lambda e: self._use_history_folder())

        hist_scroll = ttk.Scrollbar(list_frame, orient="vertical",
                                     command=self.hist_listbox.yview)
        hist_scroll.pack(side="right", fill="y")
        self.hist_listbox.configure(yscrollcommand=hist_scroll.set)

        btn_row = tk.Frame(content, bg=CARD_BG)
        btn_row.pack(fill="x", padx=12, pady=(2, 6))
        self._make_small_btn(btn_row, "填入分析目录", self._use_history_folder,
                            width=12).pack(side="left", padx=(0, 4))
        self._make_small_btn(btn_row, "打开文件夹", self._open_history_folder,
                            width=9).pack(side="left")

        self.hist_status_var = tk.StringVar(value="暂无历史会话")
        tk.Label(content, textvariable=self.hist_status_var,
                 font=FONT_SMALL, bg=CARD_BG, fg=TEXT_SECONDARY,
                 anchor="w").pack(fill="x", padx=12, pady=(0, 6))

    def _build_ga_card(self, parent):
        """Build the Gait Analysis card."""
        card, content = self._create_card(parent, title="🔬 步态分析 (Gait Analysis)",
                                         collapsible=True)

        # Data folder
        folder_row = tk.Frame(content, bg=CARD_BG)
        folder_row.pack(fill="x", padx=12, pady=(4, 4))
        tk.Label(folder_row, text="数据目录", font=FONT_BODY, bg=CARD_BG,
                 fg=TEXT_SECONDARY, width=7, anchor="w").pack(side="left")
        self.folder_var = tk.StringVar()
        tk.Entry(folder_row, textvariable=self.folder_var, width=24,
                 font=FONT_MONO_SM, bg=CARD_BG, fg=TEXT_MAIN,
                 relief="solid", highlightbackground=BORDER,
                 highlightthickness=1, bd=0).pack(
            side="left", padx=2, fill="x", expand=True)
        self._make_small_btn(folder_row, "浏览...", self._browse_folder, width=6).pack(
            side="left", padx=(4, 2))
        self.btn_use_recent = self._make_small_btn(
            folder_row, "←最近", self._use_recent_folder, width=5, state="disabled")
        self.btn_use_recent.pack(side="left")

        # Sensor mapping status
        self.map_frame = tk.Frame(content, bg=CARD_BG)
        self.map_status_var = tk.StringVar()
        self.map_status_lbl = tk.Label(self.map_frame, textvariable=self.map_status_var,
                                        font=FONT_SMALL, bg=CARD_BG, fg=WARNING,
                                        anchor="w")
        self.map_status_lbl.pack(side="left")
        self.btn_map = self._make_small_btn(
            self.map_frame, "配置传感器映射...", self._open_mapping_dialog, width=15)

        # Work mode + Run button
        ctrl_row = tk.Frame(content, bg=CARD_BG)
        ctrl_row.pack(fill="x", padx=12, pady=(2, 8))

        tk.Label(ctrl_row, text="工作模式", font=FONT_BODY, bg=CARD_BG,
                 fg=TEXT_SECONDARY, width=7, anchor="w").pack(side="left")
        self.mode_var = tk.StringVar(value="feet_only")
        self.mode_cb = ttk.Combobox(ctrl_row, textvariable=self.mode_var,
                                     values=WORK_MODES, state="readonly", width=12,
                                     font=FONT_BODY)
        self.mode_cb.pack(side="left", padx=2)

        self.btn_analyze = self._make_action_btn(
            ctrl_row, "▶  运行步态分析", self._run_analysis, PRIMARY, PRIMARY_HOVER,
            width=15)
        self.btn_analyze.pack(side="right")

        # Status
        self.ga_status_var = tk.StringVar(value="就绪 (Idle)")
        tk.Label(content, textvariable=self.ga_status_var,
                 font=FONT_SMALL, bg=CARD_BG, fg=TEXT_SECONDARY,
                 anchor="w").pack(fill="x", padx=12, pady=(0, 8))

    def _build_right_panel(self, parent):
        """Build the right-side results area: summary cards + notebook."""
        # ── Summary Cards Row 1: 4 primary metrics ──
        summary_row1 = tk.Frame(parent, bg=BACKGROUND)
        summary_row1.pack(fill="x", pady=(0, 2))

        self._create_summary_card(summary_row1,
            "步行速度", "Walking Speed",
            self.summary_speed_var, self.summary_speed_unit, accent_color=PRIMARY)

        self._create_summary_card(summary_row1,
            "步频", "Cadence",
            self.summary_cadence_var, tk.StringVar(value="steps/min"), accent_color="#E74C3C")

        self._create_summary_card(summary_row1,
            "步长", "Step Length",
            self.summary_step_length_var, tk.StringVar(value="m"), accent_color="#2ECC71")

        self._create_summary_card(summary_row1,
            "跨步时间", "Stride Time",
            self.summary_stride_time_var, tk.StringVar(value="s"), accent_color="#3498DB")

        # ── Summary Cards Row 2: 2 secondary metrics ──
        summary_row2 = tk.Frame(parent, bg=BACKGROUND)
        summary_row2.pack(fill="x", pady=(0, 6))

        self._create_summary_card(summary_row2,
            "总步数", "Total Steps",
            self.summary_steps_var, tk.StringVar(value="steps"), accent_color=TEXT_MAIN)

        self._create_summary_card(summary_row2,
            "步态周期", "Gait Cycle",
            self.summary_gait_cycle_var, tk.StringVar(value="s"), accent_color=TEXT_MAIN)

        # ── Results Notebook ──
        res_card = tk.Frame(parent, bg=CARD_BG, highlightbackground=BORDER,
                            highlightthickness=1, bd=0)
        res_card.pack(fill="both", expand=True)

        # Toolbar inside card
        res_toolbar = tk.Frame(res_card, bg=CARD_BG)
        res_toolbar.pack(fill="x", padx=8, pady=(6, 0))
        tk.Label(res_toolbar, text="📊 分析结果 (Results)", font=FONT_SECTION,
                 bg=CARD_BG, fg=TEXT_MAIN).pack(side="left")
        self.btn_export_results = self._make_small_btn(
            res_toolbar, "导出", self._export_results, width=6)
        self.btn_export_results.pack(side="right", padx=(0, 4))
        self.btn_clear_results = self._make_small_btn(
            res_toolbar, "清除结果", self._clear_results_btn, width=8)
        self.btn_clear_results.pack(side="right")

        self.notebook = ttk.Notebook(res_card)
        self.notebook.pack(fill="both", expand=True, padx=6, pady=(2, 6))

        self.tab_basic, _ = self._make_scrollable_tab("基本参数")
        self.tab_step, _  = self._make_scrollable_tab("步态参数")
        self.tab_phase, _ = self._make_scrollable_tab("相位参数")
        self.tab_joint, self.tab_joint_outer = self._make_scrollable_tab("关节角度")
        self._build_joint_tab_content()

    def _build_status_bar(self):
        """Build the bottom status bar with progress bar."""
        status_bar = tk.Frame(self.root, bg=CARD_BG, highlightbackground=BORDER,
                              highlightthickness=1, bd=0)
        status_bar.pack(fill="x", side="bottom")

        self.status_bar_var = tk.StringVar(value="就绪 (Ready)")
        self.status_bar_label = tk.Label(
            status_bar, textvariable=self.status_bar_var,
            anchor="w", bg=CARD_BG, fg=TEXT_SECONDARY,
            font=FONT_SMALL, padx=12, pady=3)
        self.status_bar_label.pack(side="left", fill="x", expand=True)

        # Indeterminate progress bar (shown during analysis)
        self.status_progress = ttk.Progressbar(
            status_bar, mode="indeterminate", length=120)
        # Hidden by default; shown during ANALYZING / COLLECTING

    def _build_joint_tab_content(self):
        """Build the joint angle tab: 3-axis numeric display + large 3-panel curve canvas."""
        inner = self.tab_joint

        # ── 数值卡片行：三轴当前值 + ROM ──
        val_card = tk.Frame(inner, bg=CARD_BG, highlightbackground=BORDER,
                            highlightthickness=1, bd=0)
        val_card.pack(fill="x", padx=8, pady=(6, 4))

        axes_info = [
            ("屈曲/伸展\nFlexion", self._joint_flexion_var, "#E74C3C"),
            ("外展/内收\nAbduction", self._joint_abduction_var, "#2ECC71"),
            ("内旋/外旋\nRotation", self._joint_rotation_var, "#3498DB"),
            ("活动范围\nROM", self._joint_rom_var, PRIMARY),
        ]
        for label, var, color in axes_info:
            frm = tk.Frame(val_card, bg=CARD_BG)
            frm.pack(side="left", padx=14, pady=6)
            tk.Label(frm, text=label, font=FONT_SMALL, bg=CARD_BG,
                     fg=TEXT_SECONDARY, justify="center").pack()
            val_row = tk.Frame(frm, bg=CARD_BG)
            val_row.pack()
            tk.Label(val_row, textvariable=var,
                     font=("Consolas", 24, "bold"), bg=CARD_BG, fg=color).pack(side="left")
            tk.Label(val_row, text="°", font=("Consolas", 12), bg=CARD_BG,
                     fg=TEXT_SECONDARY).pack(side="left")

        # ── 三面板曲线画布 ──
        curve_outer = tk.Frame(inner, bg=CARD_BG, highlightbackground=BORDER,
                               highlightthickness=1, bd=0)
        curve_outer.pack(fill="both", expand=True, padx=8, pady=(4, 8))

        self.joint_canvas_flexion = self._make_curve_canvas(curve_outer, "Flexion (屈曲/伸展)")
        self.joint_canvas_abduction = self._make_curve_canvas(curve_outer, "Abduction (外展/内收)")
        self.joint_canvas_rotation = self._make_curve_canvas(curve_outer, "Rotation (内旋/外旋)")

        # 各面板最后绘制的关键帧
        self._joint_last_drawn = {}

    def _make_curve_canvas(self, parent, title):
        """Create a labelled curve canvas panel. Returns the canvas widget."""
        panel = tk.Frame(parent, bg=CARD_BG)
        panel.pack(fill="both", expand=True, padx=2, pady=(2, 1))
        tk.Label(panel, text=title, font=FONT_SMALL, bg=CARD_BG,
                 fg=TEXT_SECONDARY, anchor="w").pack(padx=6, pady=(2, 0))
        canvas = tk.Canvas(panel, bg="#FAFBFC", height=170, highlightthickness=0)
        canvas.pack(fill="both", expand=True, padx=4, pady=(0, 4))
        return canvas

    def _draw_one_curve(self, canvas, t_list, deg_list, color, y_label="°"):
        """Draw one angle curve on a canvas."""
        if not t_list or not deg_list or len(deg_list) < 2:
            canvas.delete("all")
            canvas.create_text(200, 85, text="等待数据...", font=FONT_SMALL, fill=TEXT_SECONDARY)
            return

        w = canvas.winfo_width() or 600
        h = canvas.winfo_height() or 170
        pad_l, pad_r, pad_t, pad_b = 50, 20, 12, 28
        pw = w - pad_l - pad_r
        ph = h - pad_t - pad_b
        if pw < 50 or ph < 20:
            return

        t0 = t_list[0]
        t_rel = [t - t0 for t in t_list]
        vals = deg_list
        y_min = min(vals) - 2
        y_max = max(vals) + 2
        if y_max - y_min < 10:
            mid = (y_max + y_min) / 2
            y_min = mid - 5
            y_max = mid + 5
        t_max = max(t_rel) or 1

        def tx(t):
            return pad_l + (t / t_max) * pw
        def ty(v):
            return pad_t + (1.0 - (v - y_min) / (y_max - y_min)) * ph

        canvas.delete("all")

        # 水平网格
        for i in range(5):
            v = y_min + (y_max - y_min) * i / 4
            y = ty(v)
            canvas.create_line(pad_l, y, w - pad_r, y, fill="#E5E7EB", dash=(2, 3))
            canvas.create_text(pad_l - 4, y, text=f"{v:.0f}", anchor="e",
                               font=("Consolas", 7), fill=TEXT_SECONDARY)
        # 零线
        if y_min < 0 < y_max:
            canvas.create_line(pad_l, ty(0), w - pad_r, ty(0), fill="#9CA3AF", dash=(3, 2))

        # 垂直网格
        for i in range(5):
            t_v = t_max * i / 4
            x = tx(t_v)
            canvas.create_line(x, pad_t, x, h - pad_b, fill="#E5E7EB", dash=(2, 3))
        # X 轴标签
        canvas.create_text(w - pad_r + 4, h - pad_b + 4, text=f"{t_max:.0f}s", anchor="ne",
                           font=("Consolas", 7), fill=TEXT_SECONDARY)

        # 折线
        if len(vals) >= 2:
            pts = []
            for j in range(len(vals)):
                pts.extend([tx(t_rel[j]), ty(vals[j])])
            if len(pts) >= 4:
                canvas.create_line(*pts, fill=color, width=2, smooth=True)

        # 图例
        last_v = vals[-1]
        canvas.create_text(w - pad_r, pad_t + 2, text=f"{last_v:.1f}°", anchor="ne",
                           font=("Consolas", 9, "bold"), fill=color)

    def _redraw_joint_curves(self):
        """Redraw all 3 joint angle curve panels."""
        t = self._joint_history_t
        if not t or len(t) < 2:
            return
        self._draw_one_curve(self.joint_canvas_flexion, t,
                             self._joint_history_flexion, "#E74C3C")
        self._draw_one_curve(self.joint_canvas_abduction, t,
                             self._joint_history_abduction, "#2ECC71")
        self._draw_one_curve(self.joint_canvas_rotation, t,
                             self._joint_history_rotation, "#3498DB")

    def _make_scrollable_tab(self, title):
        outer = ttk.Frame(self.notebook)
        self.notebook.add(outer, text=title)
        canvas = tk.Canvas(outer, highlightthickness=0, bg=CARD_BG)
        scrollbar = ttk.Scrollbar(outer, orient="vertical", command=canvas.yview)
        inner = ttk.Frame(canvas)
        inner.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=inner, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        def _on_mousewheel(event):
            canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")
        canvas.bind_all("<MouseWheel>", _on_mousewheel)
        return inner, outer

    # ============================================================
    # Summary Cards Update
    # ============================================================
    def _update_summary_cards(self, data):
        """Update the 6 summary cards from gait analysis result dict."""
        bp = data.get("basicParameters", {}) if data else {}
        sp = data.get("stepParameters", {}) if data else {}

        def _avg_lr(d, key, default=0):
            """Extract left/right average from a nested dict or scalar."""
            entry = d.get(key, {})
            if isinstance(entry, dict):
                l = entry.get("left", 0) or 0
                r = entry.get("right", 0) or 0
                return (l + r) / 2 if (l or r) else default
            return entry or default

        # Walking speed
        ws_val = _avg_lr(bp, "walkingSpeed")
        # Cadence
        cd_val = _avg_lr(bp, "cadence")
        # Total steps
        ts_val = bp.get("totalSteps", 0) or 0
        # Step length (from stepParameters)
        sl_val = _avg_lr(sp, "stepLength")
        # Stride time (from basicParameters)
        st_val = _avg_lr(bp, "strideTime")
        # Gait cycle
        gc_val = bp.get("gaitCycle", 0) or 0

        self.summary_speed_var.set(f"{ws_val:.2f}")
        self.summary_cadence_var.set(f"{cd_val:.0f}")
        self.summary_steps_var.set(f"{ts_val}")
        self.summary_step_length_var.set(f"{sl_val:.2f}")
        self.summary_stride_time_var.set(f"{st_val:.2f}")
        self.summary_gait_cycle_var.set(f"{gc_val:.2f}")

    def _reset_summary_cards(self):
        """Reset summary cards to placeholder."""
        self.summary_speed_var.set("--")
        self.summary_cadence_var.set("--")
        self.summary_steps_var.set("--")
        self.summary_step_length_var.set("--")
        self.summary_stride_time_var.set("--")
        self.summary_gait_cycle_var.set("--")

    def _update_title_status(self, text, badge_bg, badge_fg):
        """Update title bar status badge text and colors."""
        self.title_status_var.set(text)
        self.title_status_badge.configure(bg=badge_bg, fg=badge_fg)

    def _init_button_states(self):
        """Set initial visual states for action buttons (IDLE defaults)."""
        self._set_action_btn_state(self.btn_start, True,
            "▶  开始采集", "▶  开始采集", SUCCESS, SUCCESS_MUTED)
        self._set_action_btn_state(self.btn_stop, False,
            "■  停止采集", "■  停止采集", DANGER, DANGER_MUTED)
        self._set_action_btn_state(self.btn_analyze, True,
            "▶  运行步态分析", "▶  运行步态分析", PRIMARY, PRIMARY_MUTED)
        self._set_action_btn_state(self.btn_joint_start, True,
            "▶  开始测量", "▶  开始测量", SUCCESS, SUCCESS_MUTED)
        self._set_action_btn_state(self.btn_joint_stop, False,
            "■  停止测量", "■  停止测量", DANGER, DANGER_MUTED)
        self.btn_joint_device_map.config(state="normal")

    # ============================================================
    # State Transitions
    # ============================================================
    def _transition_to(self, new_state: UIState):
        old = self.state
        self.state = new_state

        if new_state == UIState.IDLE:
            self.title_info_var.set("")
            self.status_progress.stop()
            self.status_progress.pack_forget()
            self._set_action_btn_state(self.btn_start, True,
                "▶  开始采集", "▶  开始采集", SUCCESS, SUCCESS_MUTED)
            self._set_action_btn_state(self.btn_stop, False,
                "■  停止采集", "■  停止采集", DANGER, DANGER_MUTED)
            self.btn_new_session.config(state="normal")
            self._set_action_btn_state(self.btn_analyze, True,
                "▶  运行步态分析", "▶  运行步态分析", PRIMARY, PRIMARY_MUTED)
            self.btn_clear_results.config(state="normal")
            self.btn_export_results.config(state="normal")
            self.btn_use_recent.config(
                state="normal" if self._last_collection_folder else "disabled")
            self._set_action_btn_state(self.btn_joint_start, True,
                "▶  开始测量", "▶  开始测量", SUCCESS, SUCCESS_MUTED)
            self._set_action_btn_state(self.btn_joint_stop, False,
                "■  停止测量", "■  停止测量", DANGER, DANGER_MUTED)
            self.btn_joint_device_map.config(state="normal")

        elif new_state == UIState.COLLECTING:
            self._set_action_btn_state(self.btn_start, False,
                "▶  开始采集", "● 采集中", SUCCESS, SUCCESS_MUTED)
            self._set_action_btn_state(self.btn_stop, True,
                "■  停止采集", "■  停止采集", DANGER, DANGER_MUTED)
            self.btn_new_session.config(state="disabled")
            self._set_action_btn_state(self.btn_analyze, False,
                "▶  运行步态分析", "▶  运行步态分析", PRIMARY, PRIMARY_MUTED)
            self.btn_clear_results.config(state="disabled")
            self.btn_use_recent.config(state="disabled")
            self._set_action_btn_state(self.btn_joint_start, False,
                "▶  开始测量", "▶  开始测量", SUCCESS, SUCCESS_MUTED)
            self._set_action_btn_state(self.btn_joint_stop, False,
                "■  停止测量", "■  停止测量", DANGER, DANGER_MUTED)
            self.btn_joint_device_map.config(state="disabled")
            self._update_title_status("● 采集中", BADGE_BLUE_BG, BADGE_BLUE_TEXT)

        elif new_state == UIState.STOPPING:
            self._set_action_btn_state(self.btn_start, False,
                "▶  开始采集", "● 采集中", SUCCESS, SUCCESS_MUTED)
            self._set_action_btn_state(self.btn_stop, False,
                "■  停止采集", "◌ 停止中", DANGER, DANGER_MUTED)
            self.btn_new_session.config(state="disabled")
            self.btn_joint_device_map.config(state="disabled")
            self._update_title_status("◌ 停止中", BADGE_ORANGE_BG, BADGE_ORANGE_TEXT)

        elif new_state == UIState.ANALYZING:
            self._set_action_btn_state(self.btn_start, False,
                "▶  开始采集", "▶  开始采集", SUCCESS, SUCCESS_MUTED)
            self._set_action_btn_state(self.btn_stop, False,
                "■  停止采集", "■  停止采集", DANGER, DANGER_MUTED)
            self.btn_new_session.config(state="disabled")
            self._set_action_btn_state(self.btn_analyze, False,
                "▶  运行步态分析", "⏳ 分析中...", PRIMARY, PRIMARY_MUTED, muted_fg="#1E40AF")
            self.btn_clear_results.config(state="disabled")
            self.btn_export_results.config(state="disabled")
            self.btn_joint_device_map.config(state="disabled")
            self._update_title_status("⏳ 分析中", BADGE_BLUE_BG, BADGE_BLUE_TEXT)
            # Show progress bar
            self.status_progress.pack(side="right", padx=(0, 12), pady=3)
            self.status_progress.start(10)

    # ============================================================
    # Session Management
    # ============================================================
    def _new_session(self):
        """Reset UI for a new collection/analysis round."""
        if self.state == UIState.COLLECTING or self.state == UIState.STOPPING:
            messagebox.showwarning("操作进行中", "请先停止当前采集再新建会话。")
            return

        if self.last_result is not None:
            ok = messagebox.askyesno("确认新建会话",
                                     "当前分析结果将被清除，确定新建会话吗？")
            if not ok:
                return

        self.last_result = None
        self._clear_all_results()
        self._joint_reset_display()
        self._reset_summary_cards()
        self.dc_status_var.set("● 未连接")
        self.ga_status_var.set("就绪 (Idle)")
        self.status_bar_var.set("就绪 (Ready) — 新会话已创建")
        self._transition_to(UIState.IDLE)

    def _clear_results_btn(self):
        """Clear results tab contents without affecting other state."""
        self._clear_all_results()
        self._joint_reset_display()
        self._reset_summary_cards()
        self.last_result = None
        self.ga_status_var.set("就绪 (Idle) — 结果已清除")
        self.status_bar_var.set("结果已清除")

    def _export_results(self):
        """Export analysis results to a JSON text file."""
        if self.last_result is None:
            messagebox.showwarning("无数据", "没有分析结果可导出。")
            return
        filepath = filedialog.asksaveasfilename(
            title="导出分析结果",
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("Text files", "*.txt"),
                       ("All files", "*.*")])
        if not filepath:
            return
        import json
        try:
            with open(filepath, "w", encoding="utf-8") as f:
                json.dump(self.last_result, f, ensure_ascii=False, indent=2)
            self.status_bar_var.set(f"结果已导出 → {os.path.basename(filepath)}")
        except Exception as e:
            messagebox.showerror("导出失败", str(e))

    def _clear_all_results(self):
        clear_results(self.tab_basic, self.tab_step, self.tab_phase)
        self._clear_joint_tab()

    def _add_session_history(self, folder, total_rows, devices):
        """Record a completed collection session in history."""
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        entry = {
            "folder": folder,
            "timestamp": ts,
            "total_rows": total_rows,
            "devices": devices,
        }
        for e in self._session_history:
            if e["folder"] == folder:
                e.update(entry)
                self._refresh_history_list()
                return
        self._session_history.insert(0, entry)
        if len(self._session_history) > 10:
            self._session_history = self._session_history[:10]
        self._refresh_history_list()

    def _refresh_history_list(self):
        self.hist_listbox.delete(0, tk.END)
        if not self._session_history:
            self.hist_status_var.set("暂无历史会话")
        else:
            self.hist_status_var.set(f"共 {len(self._session_history)} 个会话")
            for e in self._session_history:
                folder_name = os.path.basename(e["folder"])
                devices_str = ", ".join(
                    f"{k}:{v}" for k, v in e.get("devices", {}).items())
                label = f"{folder_name}  |  {e['total_rows']}行  |  {devices_str}"
                self.hist_listbox.insert(tk.END, label)

    def _use_history_folder(self):
        sel = self.hist_listbox.curselection()
        if not sel:
            return
        idx = sel[0]
        if idx < len(self._session_history):
            folder = self._session_history[idx]["folder"]
            self.folder_var.set(folder)
            self._scan_and_update_mode(folder)

    def _open_history_folder(self):
        sel = self.hist_listbox.curselection()
        if not sel:
            return
        idx = sel[0]
        if idx < len(self._session_history):
            folder = self._session_history[idx]["folder"]
            os.startfile(folder)

    def _use_recent_folder(self):
        """Set folder_var to the most recent collection folder."""
        if self._last_collection_folder:
            self.folder_var.set(self._last_collection_folder)
            self._scan_and_update_mode(self._last_collection_folder)

    # ============================================================
    # Folder browsing with sensor detection
    # ============================================================
    def _browse_folder(self):
        folder = filedialog.askdirectory(title="选择数据目录 (Select Data Folder)")
        if not folder:
            return
        self.folder_var.set(folder)
        self._scan_and_update_mode(folder)

    def _scan_and_update_mode(self, folder):
        self._scanned_devices = scan_csv_folder(folder)

        if not self._scanned_devices:
            self.map_status_var.set("⚠ 未找到 CSV 文件")
            self.map_status_lbl.configure(fg=WARNING)
            self.map_frame.pack(fill="x", padx=12, pady=(2, 4))
            self.btn_map.pack_forget()
            return

        known = {k: v for k, v in self._scanned_devices.items() if k in VALID_ALIASES}
        unknown = {k: v for k, v in self._scanned_devices.items() if k not in VALID_ALIASES}
        existing_map = load_alias_map(folder)

        resolved = dict(known)
        still_unknown = {}
        for dev_id, fname in unknown.items():
            if dev_id in existing_map:
                resolved[existing_map[dev_id]] = fname
            else:
                still_unknown[dev_id] = fname

        parts = []
        if resolved:
            parts.append(f"✓ 已识别: {', '.join(sorted(resolved.keys()))}")
        if still_unknown:
            parts.append(f"⚠ 未映射设备: {', '.join(sorted(still_unknown.keys()))}")

        if still_unknown:
            self.map_status_var.set("  |  ".join(parts))
            self.map_status_lbl.configure(fg=WARNING)
            self.map_frame.pack(fill="x", padx=12, pady=(2, 4))
            self.btn_map.pack(side="right")
        elif unknown:
            # All resolved via existing map
            self.map_status_var.set("  |  ".join(parts))
            self.map_status_lbl.configure(fg=SUCCESS)
            self.map_frame.pack(fill="x", padx=12, pady=(2, 4))
            self.btn_map.pack_forget()
        else:
            # All known — show green success
            self.map_status_var.set(
                f"✓ 传感器映射完整：{', '.join(sorted(resolved.keys()))}")
            self.map_status_lbl.configure(fg=SUCCESS)
            self.map_frame.pack(fill="x", padx=12, pady=(2, 4))
            self.btn_map.pack_forget()

        # Auto-suggest work mode
        alias_set = set(resolved.keys())
        if alias_set == {"L6", "R6"}:
            self.mode_var.set("feet_only")
        elif alias_set >= {"L6", "R6"} and alias_set <= set(MODE_ALIASES["lower_body"]):
            self.mode_var.set("lower_body")
        elif alias_set <= set(MODE_ALIASES["upper_body"]):
            self.mode_var.set("upper_body")

    def _open_mapping_dialog(self):
        folder = self.folder_var.get().strip()
        if not folder:
            return
        open_mapping_dialog(self.root, folder, self._scanned_devices,
                           on_save_callback=lambda: self._scan_and_update_mode(folder))

    # ============================================================
    # Data Collection Actions
    # ============================================================
    def _start_collection(self):
        if self.state != UIState.IDLE:
            return
        serial_port = self.port_var.get().strip()
        if not serial_port:
            messagebox.showwarning("输入错误", "请选择串口")
            return
        try:
            baud = int(self.baud_var.get().strip())
        except ValueError:
            messagebox.showwarning("输入错误", "波特率必须是整数")
            return

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        script_dir = get_base_dir()
        data_root = os.path.join(os.path.dirname(script_dir), "data")
        output_dir = os.path.join(data_root, f"wt901_data_{ts}")

        self._transition_to(UIState.COLLECTING)
        self.dc_status_var.set(f"● 连接中...  |  {serial_port} @ {baud}")
        self.status_bar_var.set("数据采集中...")

        self.collection_thread = DataCollectionThread(serial_port, baud, output_dir, self.queue)
        self.collection_thread.start()

    def _stop_collection(self):
        if self.state != UIState.COLLECTING:
            return
        self._transition_to(UIState.STOPPING)
        # Stop whichever thread is running
        if self.collection_thread:
            self.collection_thread.stop()
            self.dc_status_var.set("● 正在停止...")
            self.status_bar_var.set("正在停止数据采集...")
        elif self.joint_thread:
            self.joint_thread.stop()
            self.joint_status_var.set("● 正在停止...")
            self.status_bar_var.set("正在停止关节测量...")

        self._watchdog_id = self.root.after(8000, self._force_reset_if_stuck)

    def _force_reset_if_stuck(self):
        """Watchdog: if thread hasn't reported 'done' by now, force state reset."""
        if self.state == UIState.STOPPING:
            if self.collection_thread:
                self.dc_status_var.set("⚠ 停止超时，强制复位")
                self.status_bar_var.set("警告: 采集线程未响应，已强制复位")
                self._on_collection_done({
                    "total_rows": 0,
                    "devices": {},
                    "folder": self._last_collection_folder or "",
                })
            elif self.joint_thread:
                self.joint_status_var.set("⚠ 停止超时，强制复位")
                self.status_bar_var.set("警告: 关节测量线程未响应，已强制复位")
                self._on_joint_done()
        self._watchdog_id = None

    def _on_collection_done(self, data):
        if self._watchdog_id:
            self.root.after_cancel(self._watchdog_id)
            self._watchdog_id = None

        self.collection_thread = None
        total = data.get("total_rows", 0)
        folder = data.get("folder", "")
        devices = data.get("devices", {})

        self._last_collection_folder = folder
        dev_str = ", ".join(f"{k}: {v}" for k, v in devices.items())
        self.dc_status_var.set(f"✓ 采集完成  |  总行数: {total}  |  设备: {dev_str}")
        self.status_bar_var.set(f"数据采集完成 — {total} 行 → {folder}")

        if folder and total > 0:
            self._add_session_history(folder, total, devices)

        self._update_title_status("● 就绪", BADGE_GREEN_BG, BADGE_GREEN_TEXT)
        self._transition_to(UIState.IDLE)

    # ============================================================
    # Joint Angle Actions
    # ============================================================
    def _start_joint(self):
        """Start real-time joint angle measurement."""
        if self.state != UIState.IDLE:
            messagebox.showwarning("操作冲突", "请先停止当前操作（采集/分析）再启动关节测量。")
            return
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("输入错误", "请选择串口")
            return
        try:
            baud = int(self.baud_var.get().strip())
        except ValueError:
            baud = DEFAULT_BAUD

        joint_key = self.joint_var.get()
        info = JOINT_OPTIONS.get(joint_key, JOINT_OPTIONS["left_knee"])
        script_dir = get_base_dir()
        calib_dir = os.path.join(script_dir, "temp")

        self._transition_to(UIState.COLLECTING)
        self.joint_status_var.set(f"● 连接中... {port}")
        self.status_bar_var.set("关节角度测量启动中...")
        self._joint_reset_display()

        # Auto-switch to joint angle tab for live curves
        self.notebook.select(self.tab_joint_outer)
        self._joint_calib_state = "uncalibrated"
        self._update_joint_calib_status()

        self._clear_joint_tab()

        # 加载设备映射并传给线程
        self._load_joint_device_map()
        self._update_joint_device_display()

        self.joint_thread = JointAngleThread(
            port, baud, joint_key, self.queue, calib_dir,
            device_map=self._joint_device_map)
        self.joint_thread.start()

    def _stop_joint(self):
        """Stop joint angle measurement."""
        if self.joint_thread is None:
            return
        self.joint_thread.stop()
        self.joint_status_var.set("● 正在停止...")
        self.status_bar_var.set("正在停止关节测量...")

    def _start_joint_calib(self):
        """Request calibration (thread must be running and alive)."""
        if self.joint_thread is None or not self.joint_thread.is_alive():
            messagebox.showwarning("未启动", "请先启动关节角度测量，等待初始化完成后再校准。")
            self._joint_calib_state = "uncalibrated"
            self._update_joint_calib_status()
            return
        self._joint_calib_state = "calibrating"
        self._update_joint_calib_status()
        mode = self.calib_mode_var.get()
        self.joint_thread.start_calibration(mode)
        self.status_bar_var.set(f"正在标定 ({mode})，请保持静止站立...")

    def _joint_reset_display(self):
        self._joint_flexion_var.set("--")
        self._joint_abduction_var.set("--")
        self._joint_rotation_var.set("--")
        self._joint_max_var.set("--")
        self._joint_min_var.set("--")
        self._joint_rom_var.set("--")
        self._joint_history_t.clear()
        self._joint_history_flexion.clear()
        self._joint_history_abduction.clear()
        self._joint_history_rotation.clear()

    def _update_joint_calib_status(self):
        state = self._joint_calib_state
        label = CALIB_STATES.get(state, state)
        if state == "calibrated":
            color = SUCCESS
        elif state in ("calibrating",):
            color = WARNING
        elif state == "failed":
            color = DANGER
        else:
            color = WARNING
        self.joint_calib_status_var.set(label)
        # Update button state
        if state in ("calibrating",):
            self.btn_joint_calib.config(state="disabled")
        else:
            self.btn_joint_calib.config(state="normal")

    # ── Joint Device Mapping helpers ──
    def _load_joint_device_map(self):
        """从 calib_dir 加载 device_alias_map.json 并缓存。"""
        import os as _os
        from . import get_base_dir as _get_base_dir
        from .dialogs import load_alias_map as _load_alias_map
        script_dir = _get_base_dir()
        calib_dir = _os.path.join(script_dir, "temp")
        self._joint_device_map = _load_alias_map(calib_dir)
        return self._joint_device_map

    def _update_joint_device_display(self):
        """根据 device_map 反查当前关节的 prox/dist 设备 ID 并更新显示。"""
        dm = self._joint_device_map
        prox_alias = self._joint_prox_var.get()
        dist_alias = self._joint_dist_var.get()

        prox_dev = "—"
        for dev_id, alias in dm.items():
            if alias == prox_alias:
                prox_dev = dev_id
                break

        dist_dev = "—"
        for dev_id, alias in dm.items():
            if alias == dist_alias:
                dist_dev = dev_id
                break

        self._joint_prox_device_var.set(prox_dev)
        self._joint_dist_device_var.set(dist_dev)

    def _open_joint_device_mapping(self):
        """打开关节角度设备映射配置对话框。"""
        if self.state != UIState.IDLE:
            messagebox.showwarning("操作冲突",
                                   "请先停止当前测量再配置设备映射。")
            return
        import os as _os
        from . import get_base_dir as _get_base_dir
        from .dialogs import open_joint_device_mapping_dialog as _open_dlg
        script_dir = _get_base_dir()
        calib_dir = _os.path.join(script_dir, "temp")

        def _on_mapping_saved():
            self._load_joint_device_map()
            self._update_joint_device_display()

        _open_dlg(self.root, calib_dir, self.joint_var.get(),
                  on_save_callback=_on_mapping_saved)

    def _clear_joint_tab(self):
        for w in self.tab_joint.winfo_children():
            w.destroy()
        self._build_joint_tab_content()

    def _on_joint_done(self):
        self.joint_thread = None
        self.joint_status_var.set("● 已停止")
        self.status_bar_var.set("关节角度测量已停止")
        self._update_title_status("● 就绪", BADGE_GREEN_BG, BADGE_GREEN_TEXT)
        self._transition_to(UIState.IDLE)

    def _on_joint_angle_data(self, data):
        flex = data.get("flexion_deg", 0)
        abd = data.get("abduction_deg", 0)
        rot = data.get("rotation_deg", 0)
        self._joint_flexion_var.set(f"{flex:.1f}")
        self._joint_abduction_var.set(f"{abd:.1f}")
        self._joint_rotation_var.set(f"{rot:.1f}")
        self._joint_max_var.set(f"{data.get('max_deg', 0):.1f}")
        self._joint_min_var.set(f"{data.get('min_deg', 0):.1f}")
        self._joint_rom_var.set(f"{data.get('rom_deg', 0):.1f}")

        # Update curve histories (3 axes)
        self._joint_history_t = data.get("history_t", [])
        self._joint_history_flexion = data.get("history_flexion", [])
        self._joint_history_abduction = data.get("history_abduction", [])
        self._joint_history_rotation = data.get("history_rotation", [])

        # 重绘所有曲线
        self._redraw_joint_curves()

        # Status update
        cal = "✓" if data.get("calibrated") else "✗"
        init = "✓" if data.get("initialized") else "⏳"
        self.joint_status_var.set(f"● 测量中  |  校准:{cal}  初始化:{init}  "
                                  f"|  屈伸:{flex:.1f}°  外展:{abd:.1f}°  旋转:{rot:.1f}°")

    # ============================================================
    # Gait Analysis Actions
    # ============================================================
    def _run_analysis(self):
        if self.state != UIState.IDLE:
            return
        data_folder = self.folder_var.get().strip()
        if not data_folder:
            messagebox.showwarning("缺少输入", "请先选择数据目录 (Data Folder)")
            return
        if not os.path.isdir(data_folder):
            messagebox.showerror("路径错误", f"目录不存在:\n{data_folder}")
            return

        known = {k for k in self._scanned_devices if k in VALID_ALIASES}
        existing_map = load_alias_map(data_folder)
        all_resolved = set(known)
        for dev_id in self._scanned_devices:
            if dev_id not in VALID_ALIASES and dev_id in existing_map:
                all_resolved.add(existing_map[dev_id])

        mode = self.mode_var.get()
        required = set(MODE_ALIASES.get(mode, []))
        if mode == "feet_only":
            needed = {"L6", "R6"}
        else:
            needed = required

        if not needed.issubset(all_resolved):
            missing = needed - all_resolved
            msg = (f"当前工作模式 '{mode}' 需要传感器: {', '.join(sorted(needed))}\n"
                   f"缺失: {', '.join(sorted(missing))}\n\n"
                   f"请点击「配置传感器映射」将设备 ID 映射到对应的身体别名。")
            messagebox.showwarning("传感器缺失", msg)
            return

        self._transition_to(UIState.ANALYZING)
        self.ga_status_var.set("运行中... 请稍候")
        self.status_bar_var.set("正在执行步态分析...")
        self._clear_all_results()
        self._reset_summary_cards()

        script_dir = get_base_dir()
        self.analysis_thread = GaitAnalysisThread(
            data_folder, mode, self.queue, script_dir)
        self.analysis_thread.start()

    def _on_gait_done(self, data):
        self.analysis_thread = None
        elapsed = data.get("elapsed", 0)
        self.ga_status_var.set(f"✓ {data.get('message', '完成')}")
        self.status_bar_var.set(f"步态分析完成 — 耗时 {elapsed:.1f}s")
        self.last_result = data.get("result", {})
        display_results(self.tab_basic, self.tab_step, self.tab_phase,
                        self.last_result)
        self._update_summary_cards(self.last_result)
        self._update_title_status("✓ 分析完成", BADGE_GREEN_BG, BADGE_GREEN_TEXT)
        self._transition_to(UIState.IDLE)

    def _on_gait_error(self, data):
        self.analysis_thread = None
        msg = data.get("message", "未知错误")
        elapsed = data.get("elapsed", 0)
        self.ga_status_var.set(f"✗ 失败: {msg}")
        self.status_bar_var.set(f"步态分析失败 — {elapsed:.1f}s")
        self._update_title_status("✗ 错误", BADGE_RED_BG, BADGE_RED_TEXT)
        self._transition_to(UIState.IDLE)
        tb = data.get("traceback", "")
        detail = f"{msg}\n\n{tb}" if tb else msg
        messagebox.showerror("分析错误", detail)

    # ============================================================
    # Queue Polling
    # ============================================================
    def _poll_queue(self):
        try:
            while True:
                msg = self.queue.get_nowait()
                mt = msg.get("type")

                if mt == "status":
                    state = msg.get("state", "")
                    if state == "connected":
                        self.dc_status_var.set(f"● 已连接  |  {msg.get('message', '')}")
                    elif state == "connecting":
                        self.dc_status_var.set(f"◌ {msg.get('message', '')}")
                    elif state == "disconnected":
                        self.dc_status_var.set(f"✗ {msg.get('message', '')}")
                        self._update_title_status("✗ 断开", BADGE_RED_BG, BADGE_RED_TEXT)
                    elif state == "stopping":
                        self.dc_status_var.set(f"◌ {msg.get('message', '')}")
                    self.status_bar_var.set(msg.get("message", ""))

                elif mt == "data":
                    total = msg.get("total_rows", 0)
                    devices = msg.get("devices", {})
                    dev_str = ", ".join(f"{k}: {v}" for k, v in devices.items())
                    self.dc_status_var.set(f"● 采集中  |  总行数: {total}  |  {dev_str}")
                    self.status_bar_var.set(f"数据采集: {total} 行")
                    # Update title bar info
                    dev_count = len(devices)
                    self.title_info_var.set(f"{total} 行 | {dev_count} 设备")

                elif mt == "done":
                    self._on_collection_done(msg)

                elif mt == "error":
                    self.dc_status_var.set(f"✗ {msg.get('message', '错误')}")
                    self.status_bar_var.set("数据采集出错")

                elif mt == "gait_status":
                    self.ga_status_var.set(f"⏳ {msg.get('message', '运行中...')}")

                elif mt == "gait_done":
                    self._on_gait_done(msg)

                elif mt == "gait_error":
                    self._on_gait_error(msg)

                # ── Joint Angle messages ──
                elif mt == "joint_status":
                    state = msg.get("state", "")
                    if state == "connected":
                        self.joint_status_var.set(f"● 已连接  |  {msg.get('message', '')}")
                    elif state == "calibrated":
                        self._joint_calib_state = "calibrated"
                        self._update_joint_calib_status()
                    elif state == "uncalibrated":
                        self._joint_calib_state = "uncalibrated"
                        self._update_joint_calib_status()
                    elif state == "disconnected":
                        self.joint_status_var.set(f"✗ {msg.get('message', '')}")
                        self._on_joint_done()
                    elif state == "device_detected":
                        # 新设备上线 — 刷新映射显示
                        self._load_joint_device_map()
                        self._update_joint_device_display()
                    elif state == "identified":
                        # 传感器已识别 — 用结构化字段直接更新设备 ID
                        prox_id = msg.get("proximal_id", "")
                        dist_id = msg.get("distal_id", "")
                        if prox_id:
                            self._joint_prox_device_var.set(prox_id)
                        if dist_id:
                            self._joint_dist_device_var.set(dist_id)
                    elif state == "devices":
                        # 周期性设备状态 — 刷新显示
                        self._update_joint_device_display()
                    self.status_bar_var.set(msg.get("message", ""))

                elif mt == "joint_angle":
                    self._on_joint_angle_data(msg)

                elif mt == "joint_calib_status":
                    self.joint_status_var.set(f"◌ {msg.get('message', '')}")

                elif mt == "joint_calib_done":
                    self._joint_calib_state = "calibrated"
                    self._update_joint_calib_status()
                    self.status_bar_var.set(msg.get("message", "标定完成"))
                    self.joint_status_var.set(f"✓ 标定完成 — {msg.get('joint', '')}")

                elif mt == "joint_calib_error":
                    self._joint_calib_state = "failed"
                    self._update_joint_calib_status()
                    self.status_bar_var.set(f"标定失败: {msg.get('message', '')}")

                elif mt == "joint_error":
                    self.joint_status_var.set(f"✗ {msg.get('message', '')}")
                    self.status_bar_var.set("关节角度测量错误")
                    tb = msg.get("traceback", "")
                    detail = f"{msg.get('message', '')}\n\n{tb}" if tb else msg.get('message', '')
                    self._on_joint_done()
                    # 弹窗显示详细错误
                    messagebox.showerror("关节测量错误", detail)

        except queue.Empty:
            pass
        self.root.after(100, self._poll_queue)

    def on_close(self):
        if self.state == UIState.COLLECTING and self.collection_thread:
            self.collection_thread.stop()
            self.collection_thread = None
        if self.joint_thread:
            self.joint_thread.stop()
            self.joint_thread = None
        self.root.destroy()
