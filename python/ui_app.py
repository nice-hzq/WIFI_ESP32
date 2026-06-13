# -*- coding: utf-8 -*-
"""
步态分析系统 — 桌面 UI 入口
Gait Analysis System — Desktop UI Entry Point

Usage (dev):
    cd python && python ui_app.py

Usage (frozen / PyInstaller exe):
    GaitAnalysis.exe
"""

import sys, os

# Detect frozen mode (PyInstaller)
IS_FROZEN = getattr(sys, 'frozen', False)

if IS_FROZEN:
    # Running as exe — the exe directory is our base
    BASE_DIR = os.path.dirname(sys.executable)
    # Add base dir to path for local package imports
    if BASE_DIR not in sys.path:
        sys.path.insert(0, BASE_DIR)
else:
    # Running as script — python/ is our base
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    if BASE_DIR not in sys.path:
        sys.path.insert(0, BASE_DIR)

# Check optional dependency early (bundled in exe, so skip check in frozen)
if not IS_FROZEN:
    try:
        import serial
        import serial.tools.list_ports
    except ImportError:
        print("[ERROR] pyserial is required. Run: pip install pyserial")
        sys.exit(1)

import tkinter as tk
from ui.app import GaitAnalysisApp


def main():
    root = tk.Tk()
    app = GaitAnalysisApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
