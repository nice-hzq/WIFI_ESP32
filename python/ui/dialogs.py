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
