# -*- mode: python ; coding: utf-8 -*-
# ============================================================
# GaitAnalysis — PyInstaller spec file
# ============================================================

from PyInstaller.utils.hooks import collect_data_files, collect_submodules

block_cipher = None

# ── Collect ahrs WMM data files (WMM.COF etc.) ──
ahrs_datas = collect_data_files('ahrs', includes=['utils/WMM*/*'])

# ── Collect scipy hidden imports (dynamic sub-modules) ──
#     scipy._lib.array_api_compat.* is imported dynamically at runtime
#     and PyInstaller cannot detect it via static analysis.
scipy_hidden = (
    collect_submodules('scipy._lib.array_api_compat')
    + collect_submodules('scipy._lib.array_api_compat.numpy')
)

a = Analysis(
    ['python\\ui_app.py'],
    pathex=[],
    binaries=[],
    datas=ahrs_datas,
    hiddenimports=[
        'serial', 'serial.tools.list_ports',
        'numpy', 'scipy', 'matplotlib', 'ahrs',
        'PIL', 'PIL.Image', 'PIL.ImageTk',
        'core', 'core.config', 'core.math_utils', 'core.quaternion',
        'gait', 'gait.gait_pipeline', 'gait.event_detection',
        'gait.temporal_metrics', 'gait.distance_metrics',
        'gait.metrics_builder', 'gait.turn_detection',
        'gait.count_phase', 'gait.tool', 'gait.distance_new',
        'sensor', 'sensor.data_reader',
        'report', 'report.gait_models',
        'orientation', 'orientation.euler_angles',
        'orientation.quaternion_manager', 'orientation.quaternions',
        'json', 'csv', 'queue',
    ] + scipy_hidden,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)
pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.zipfiles,
    a.datas,
    [],
    name='GaitAnalysis',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
