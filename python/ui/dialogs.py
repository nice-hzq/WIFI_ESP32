# -*- coding: utf-8 -*-
"""
对话框 — 传感器映射配置
Dialogs for sensor alias mapping.
"""

import os, json, csv
import tkinter as tk
from tkinter import ttk, messagebox

from . import VALID_ALIASES


# ============================================================
# Helpers (moved from ui_app.py)
# ============================================================
def scan_csv_folder(folder):
    """
    Scan a folder for CSV files and return a dict:
        {device_id_or_alias: csv_filename}
    Tries to detect alias from filename suffix first,
    then falls back to reading device_id from CSV content.
    """
    result = {}
    if not os.path.isdir(folder):
        return result
    for fname in os.listdir(folder):
        if not fname.lower().endswith(".csv"):
            continue
        fpath = os.path.join(folder, fname)
        base = os.path.splitext(fname)[0]
        suffix = base.split("_")[-1].strip()
        if suffix in VALID_ALIASES:
            result[suffix] = fname
        else:
            try:
                with open(fpath, "r", encoding="utf-8") as f:
                    reader = csv.reader(f)
                    next(reader, [])
                    for row in reader:
                        if row and row[0].strip():
                            dev_id = row[0].strip()
                            result[dev_id] = fname
                            break
            except Exception:
                result[base] = fname
    return result


def load_alias_map(folder):
    """Load device_alias_map.json if it exists in the folder."""
    map_path = os.path.join(folder, "device_alias_map.json")
    if os.path.isfile(map_path):
        try:
            with open(map_path, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            pass
    return {}


def save_alias_map(folder, mapping):
    """Save device_alias_map.json to the folder."""
    map_path = os.path.join(folder, "device_alias_map.json")
    with open(map_path, "w", encoding="utf-8") as f:
        json.dump(mapping, f, ensure_ascii=False, indent=2)


# ============================================================
# Mapping Dialog
# ============================================================
def open_mapping_dialog(parent, folder, scanned_devices, on_save_callback=None):
    """
    Open a dialog for mapping device IDs to sensor aliases.

    Args:
        parent: Tk root or toplevel
        folder: data folder path (for loading/saving device_alias_map.json)
        scanned_devices: {device_id: filename} dict
        on_save_callback: called after mapping is saved, for UI to re-scan
    """
    # Find unknown device IDs
    known = {k: v for k, v in scanned_devices.items() if k in VALID_ALIASES}
    existing_map = load_alias_map(folder)
    unknown = {}
    for dev_id, fname in scanned_devices.items():
        if dev_id not in VALID_ALIASES and dev_id not in existing_map:
            unknown[dev_id] = fname

    if not unknown:
        messagebox.showinfo("传感器映射", "所有设备已识别，无需额外映射。")
        return

    dialog = tk.Toplevel(parent)
    dialog.title("传感器映射 — Device → Alias Mapping")
    dialog.geometry("520x380")
    dialog.resizable(False, False)
    dialog.transient(parent)
    dialog.grab_set()
    dialog.configure(bg="#FFFFFF")

    tk.Label(dialog, text="将设备 ID 映射到身体传感器别名",
             font=("Microsoft YaHei UI", 12, "bold"),
             bg="#FFFFFF", fg="#111827").pack(pady=(16, 4))
    tk.Label(dialog, text="请为每个设备选择对应的身体位置：",
             font=("Microsoft YaHei UI", 9),
             bg="#FFFFFF", fg="#6B7280").pack(pady=(0, 12))

    body = tk.Frame(dialog, bg="#FFFFFF")
    body.pack(fill="both", expand=True, padx=20, pady=4)

    alias_vars = {}
    for i, (dev_id, fname) in enumerate(sorted(unknown.items())):
        row = tk.Frame(body, bg="#FFFFFF")
        row.pack(fill="x", pady=4)
        tk.Label(row, text=f"设备: {dev_id}", width=28, anchor="w",
                 font=("Consolas", 9), bg="#FFFFFF", fg="#111827").pack(side="left")
        tk.Label(row, text=f"文件: {fname}", width=24, anchor="w",
                 font=("Consolas", 8), bg="#FFFFFF", fg="#9CA3AF").pack(
            side="left", padx=8)
        var = tk.StringVar(value="")
        cb = ttk.Combobox(row, textvariable=var, values=VALID_ALIASES,
                         state="readonly", width=6, font=("Microsoft YaHei UI", 9))
        cb.pack(side="right", padx=4)
        alias_vars[dev_id] = var

    def _save_mapping():
        new_map = {}
        for dev_id, var in alias_vars.items():
            alias = var.get().strip()
            if not alias:
                messagebox.showwarning("不完整", f"请为设备 {dev_id} 选择别名")
                return
            new_map[dev_id] = alias

        full_map = load_alias_map(folder)
        full_map.update(new_map)
        save_alias_map(folder, full_map)
        messagebox.showinfo("保存成功",
                            f"已保存 {len(new_map)} 条映射到 device_alias_map.json")
        dialog.destroy()
        if on_save_callback:
            on_save_callback()

    btn_row = tk.Frame(dialog, bg="#FFFFFF")
    btn_row.pack(pady=(12, 16))
    tk.Button(btn_row, text="保存映射", width=12, font=("Microsoft YaHei UI", 10),
              bg="#2563EB", fg="white", bd=0, padx=10, pady=4,
              activebackground="#1D4ED8", activeforeground="white",
              relief="flat", cursor="hand2",
              command=_save_mapping).pack(side="left", padx=4)
    tk.Button(btn_row, text="取消", width=8, font=("Microsoft YaHei UI", 10),
              bg="#FFFFFF", fg="#111827", bd=0, padx=10, pady=4,
              activebackground="#F9FAFB",
              relief="flat", cursor="hand2",
              highlightbackground="#E5E7EB", highlightthickness=1,
              command=dialog.destroy).pack(side="left", padx=4)


# ============================================================
# Joint Device Mapping Dialog (用于关节角度设备映射)
# ============================================================
def open_joint_device_mapping_dialog(parent, calib_dir, current_joint_key,
                                     on_save_callback=None):
    """
    打开关节角度设备映射配置对话框。

    与 open_mapping_dialog（CSV 扫描驱动）不同：
      - 此对话框用于实时串口场景，用户手动输入设备 ID (WT****)
      - 映射持久化到 calib_dir/device_alias_map.json

    Args:
        parent: Tk root or toplevel
        calib_dir: 存放 device_alias_map.json 的目录
        current_joint_key: 当前选中的关节 key，用于提示
        on_save_callback: 保存后回调（UI 刷新设备显示）
    """
    from . import JOINT_OPTIONS

    existing_map = load_alias_map(calib_dir)

    # 当前关节信息
    joint_info = JOINT_OPTIONS.get(current_joint_key, {})
    joint_label = joint_info.get("label", current_joint_key)
    prox_alias = joint_info.get("proximal", "?")
    dist_alias = joint_info.get("distal", "?")

    dialog = tk.Toplevel(parent)
    dialog.title("关节设备映射 — Joint Device Mapping")
    dialog.geometry("600x480")
    dialog.resizable(True, True)
    dialog.minsize(500, 380)
    dialog.transient(parent)
    dialog.grab_set()
    dialog.configure(bg="#FFFFFF")

    # ── Header ──
    tk.Label(dialog, text="将设备 ID (WT****) 映射到身体传感器别名",
             font=("Microsoft YaHei UI", 12, "bold"),
             bg="#FFFFFF", fg="#111827").pack(pady=(16, 4))
    tk.Label(dialog,
             text=f"当前关节: {joint_label}  |  近端={prox_alias}  远端={dist_alias}",
             font=("Microsoft YaHei UI", 9, "bold"),
             bg="#FFFFFF", fg="#2563EB").pack(pady=(0, 4))
    tk.Label(dialog, text="输入设备 ID（传感器外壳上的 WT 编号），然后选择对应的身体位置。",
             font=("Microsoft YaHei UI", 9),
             bg="#FFFFFF", fg="#6B7280").pack(pady=(0, 12))

    # ── Scrollable table area ──
    canvas = tk.Canvas(dialog, bg="#FFFFFF", highlightthickness=0)
    scrollbar = ttk.Scrollbar(dialog, orient="vertical", command=canvas.yview)
    table_frame = tk.Frame(canvas, bg="#FFFFFF")

    table_frame.bind("<Configure>",
                     lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
    canvas.create_window((0, 0), window=table_frame, anchor="nw")
    canvas.configure(yscrollcommand=scrollbar.set)

    canvas.pack(side="left", fill="both", expand=True, padx=(20, 0), pady=4)
    scrollbar.pack(side="right", fill="y", padx=(0, 20), pady=4)

    # 鼠标滚轮支持
    def _on_mousewheel(event):
        canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")
    canvas.bind_all("<MouseWheel>", _on_mousewheel)

    # ── Table header ──
    hdr = tk.Frame(table_frame, bg="#F3F6FA")
    hdr.pack(fill="x", pady=(0, 2))
    tk.Label(hdr, text="设备 ID (WT****)", width=26, anchor="w",
             font=("Microsoft YaHei UI", 9, "bold"), bg="#F3F6FA", fg="#374151").pack(
        side="left", padx=8, pady=4)
    tk.Label(hdr, text="传感器别名", width=14, anchor="w",
             font=("Microsoft YaHei UI", 9, "bold"), bg="#F3F6FA", fg="#374151").pack(
        side="left", padx=8, pady=4)
    tk.Label(hdr, text="操作", width=8, anchor="w",
             font=("Microsoft YaHei UI", 9, "bold"), bg="#F3F6FA", fg="#374151").pack(
        side="left", padx=8, pady=4)

    # ── Row storage ──
    _rows = []  # list of {"dev_entry": ..., "alias_var": ..., "row_frame": ...}

    def _add_row(dev_id="", alias_val=""):
        """添加一行设备→别名映射"""
        row = tk.Frame(table_frame, bg="#FFFFFF")
        row.pack(fill="x", pady=2)

        dev_entry = tk.Entry(row, width=24, font=("Consolas", 9),
                             bg="#FAFBFC", fg="#111827",
                             relief="solid", highlightbackground="#D1D5DB",
                             highlightthickness=1, bd=0)
        dev_entry.pack(side="left", padx=8, pady=2)
        if dev_id:
            dev_entry.insert(0, dev_id)

        alias_var = tk.StringVar(value=alias_val)
        alias_cb = ttk.Combobox(row, textvariable=alias_var,
                                values=VALID_ALIASES, state="readonly",
                                width=12, font=("Microsoft YaHei UI", 9))
        alias_cb.pack(side="left", padx=8, pady=2)

        def _delete_row(rf=row):
            rf.destroy()
            _rows[:] = [r for r in _rows if r["row_frame"] is not rf]

        del_btn = tk.Button(row, text="✕ 删除", width=6,
                            font=("Microsoft YaHei UI", 8),
                            bg="#FFFFFF", fg="#EF4444", bd=0,
                            activebackground="#FEE2E2",
                            relief="flat", cursor="hand2",
                            command=_delete_row)
        del_btn.pack(side="left", padx=8, pady=2)

        _rows.append({
            "dev_entry": dev_entry,
            "alias_var": alias_var,
            "row_frame": row,
        })

    # ── Populate existing mappings ──
    if existing_map:
        for dev_id, alias in sorted(existing_map.items()):
            _add_row(dev_id=dev_id, alias_val=alias)
    else:
        # 无已有映射时，预填一行空行方便用户
        _add_row()

    # ── Add button ──
    add_btn_row = tk.Frame(table_frame, bg="#FFFFFF")
    add_btn_row.pack(fill="x", pady=6)
    tk.Button(add_btn_row, text="＋ 添加设备", width=12,
              font=("Microsoft YaHei UI", 9),
              bg="#FFFFFF", fg="#2563EB", bd=0,
              activebackground="#DBEAFE",
              relief="flat", cursor="hand2",
              highlightbackground="#BFDBFE", highlightthickness=1,
              command=lambda: _add_row()).pack(side="left", padx=8)

    # ── Footer: quick-fill for current joint ──
    quick_row = tk.Frame(table_frame, bg="#FFFFFF")
    quick_row.pack(fill="x", pady=(2, 8))
    tk.Label(quick_row, text="快捷填充:", font=("Microsoft YaHei UI", 9),
             bg="#FFFFFF", fg="#6B7280").pack(side="left", padx=(8, 4))

    def _quick_fill_current():
        """在当前空行中预填 proximal/distal aliases（仅填 alias，device_id 留空）"""
        # 找到两行空的或新增两行
        empty_rows = [r for r in _rows
                      if not r["dev_entry"].get().strip()
                      and not r["alias_var"].get().strip()]
        needed = [(prox_alias, "近端"), (dist_alias, "远端")]
        for i, (alias, _role) in enumerate(needed):
            if i < len(empty_rows):
                empty_rows[i]["alias_var"].set(alias)
            else:
                _add_row(alias_val=alias)

    tk.Button(quick_row, text=f"填入 {prox_alias}(近端)+{dist_alias}(远端)",
              font=("Microsoft YaHei UI", 8),
              bg="#F0F9FF", fg="#0369A1", bd=0,
              activebackground="#E0F2FE",
              relief="flat", cursor="hand2",
              command=_quick_fill_current).pack(side="left")

    # ── Footer buttons ──
    btn_row = tk.Frame(dialog, bg="#FFFFFF")
    btn_row.pack(pady=(12, 16))

    def _save_mapping():
        new_map = {}
        errors = []
        seen_aliases = {}

        for r in _rows:
            dev_id = r["dev_entry"].get().strip()
            alias = r["alias_var"].get().strip()

            # 跳过空行
            if not dev_id and not alias:
                continue
            # dev_id 和 alias 必须同时填写
            if dev_id and not alias:
                errors.append(f"设备 {dev_id}: 未选择别名")
                continue
            if alias and not dev_id:
                errors.append(f"别名 {alias}: 未填写设备 ID")
                continue

            new_map[dev_id] = alias

            if alias in seen_aliases:
                errors.append(
                    f"别名 {alias} 被多个设备使用: "
                    f"{seen_aliases[alias]} 和 {dev_id}")
            seen_aliases[alias] = dev_id

        if errors:
            messagebox.showwarning("配置不完整",
                                   "以下问题需要修正:\n\n" + "\n".join(errors))
            return

        save_alias_map(calib_dir, new_map)
        messagebox.showinfo("保存成功",
                            f"已保存 {len(new_map)} 条映射到\n{calib_dir}/device_alias_map.json")
        dialog.destroy()
        if on_save_callback:
            on_save_callback()

    tk.Button(btn_row, text="保存映射", width=12,
              font=("Microsoft YaHei UI", 10),
              bg="#2563EB", fg="white", bd=0, padx=10, pady=4,
              activebackground="#1D4ED8", activeforeground="white",
              relief="flat", cursor="hand2",
              command=_save_mapping).pack(side="left", padx=4)
    tk.Button(btn_row, text="取消", width=8,
              font=("Microsoft YaHei UI", 10),
              bg="#FFFFFF", fg="#111827", bd=0, padx=10, pady=4,
              activebackground="#F9FAFB",
              relief="flat", cursor="hand2",
              highlightbackground="#E5E7EB", highlightthickness=1,
              command=dialog.destroy).pack(side="left", padx=4)
