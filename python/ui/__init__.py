# -*- coding: utf-8 -*-
"""
步态分析系统 UI 包
Gait Analysis System UI Package
"""

import os, sys


# ============================================================
# Path helper (supports PyInstaller frozen mode)
# ============================================================
def get_base_dir():
    """Return the project root directory (python/ in dev, exe dir in frozen)."""
    if getattr(sys, 'frozen', False):
        # Running as PyInstaller exe
        return os.path.dirname(sys.executable)
    else:
        # Running as script
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# ============================================================
# Shared Constants
# ============================================================
DEFAULT_BAUD = 921600
SERIAL_RETRY_DELAY = 1.0
CSV_HEADER = [
    "device_id", "timestamp",
    "Acc_x", "Acc_y", "Acc_z",
    "Gyr_x", "Gyr_y", "Gyr_z",
    "Angle_x", "Angle_y", "Angle_z",
    "Geo_x", "Geo_y", "Geo_z",
]
WORK_MODES = ["lower_body", "feet_only", "upper_body", "full_body"]

VALID_ALIASES = [
    "H", "T1", "T12",
    "L1", "L2", "L3", "L4", "L5", "L6",
    "R1", "R2", "R3", "R4", "R5", "R6",
    "S1",
]

MODE_ALIASES = {
    "feet_only":   ["L6", "R6"],
    "lower_body":  ["S1", "L4", "L5", "L6", "R4", "R5", "R6"],
    "upper_body":  ["H", "T1", "T12", "L1", "L2", "L3", "R1", "R2", "R3"],
    "full_body":   ["H", "T1", "T12", "L1", "L2", "L3", "R1", "R2", "R3",
                    "S1", "L4", "L5", "L6", "R4", "R5", "R6"],
}
