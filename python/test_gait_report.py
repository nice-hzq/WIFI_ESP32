# -*- coding: utf-8 -*-
"""测试步态分析报告输出 — 模拟 C# 调用完整管线"""
import sys, os

# 确保当前目录在 path 中
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core import config

# ---- 1. 配置路径 ----
config.tempDir = os.path.join(os.path.dirname(__file__), "temp")
config.originalDir = os.path.join(os.path.dirname(__file__), "Data","步态评估数据","2")
config.curveDir = os.path.join(os.path.dirname(__file__), "output")
config.WORK_MODE = "lower_body"
config.fs = 100

os.makedirs(config.curveDir, exist_ok=True)

print("=" * 60)
print("步态分析报告测试")
print(f"  tempDir     = {config.tempDir}")
print(f"  originalDir = {config.originalDir}")
print(f"  curveDir    = {config.curveDir}")
print(f"  WORK_MODE   = {config.WORK_MODE}")
print("=" * 60)

# ---- 2. 加载并校准传感器数据 ----
from sensor.data_reader import load_calibrated_filtered_arrays

print("\n加载传感器原始数据...")
try:
    arrs = load_calibrated_filtered_arrays(
        window_size=5,
        aliases=["S1", "L4", "L5", "L6", "R4", "R5", "R6"]
    )
    arr_S1, arr_L4, arr_L5, arr_L6, arr_R4, arr_R5, arr_R6 = arrs
    print(f"  数据加载成功:")
    for name, arr in zip(["S1", "L4", "L5", "L6", "R4", "R5", "R6"], arrs):
        if arr is not None:
            print(f"    {name}: {arr.shape}")
        else:
            print(f"    {name}: None!")
except Exception as e:
    print(f"  数据加载失败: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

import numpy as np


# ============================================================
# 绘图工具：绘制 gyro_x 信号 + HS/TO/MS 检测点
# ============================================================
def plot_gait_events(left_gyr_x, right_gyr_x, res_gyro, fs, title, save_path=None):
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches

    left = res_gyro["left"]
    right = res_gyro["right"]

    sig_L = left.get("sig", left_gyr_x)
    sig_R = right.get("sig", right_gyr_x)
    t_L = np.arange(len(sig_L)) / fs
    t_R = np.arange(len(sig_R)) / fs

    fig, (ax0, ax1) = plt.subplots(2, 1, figsize=(16, 8))

    def _draw(ax, t, sig, side, label):
        ax.plot(t, sig, lw=0.6, color="#333333")
        hs = np.asarray(side.get("HS_idx", []), int)
        to = np.asarray(side.get("TO_idx", []), int)
        ms = np.asarray(side.get("MS_idx", []), int)

        for idx in ms:
            if 0 <= idx < len(t):
                ax.axvline(t[idx], color="blue", alpha=0.25, lw=0.8)
        for idx in to:
            if 0 <= idx < len(t):
                ax.axvline(t[idx], color="green", alpha=0.45, lw=1.0)
        for idx in hs:
            if 0 <= idx < len(t):
                ax.axvline(t[idx], color="red", alpha=0.50, lw=1.2)

        valid_hs = hs[hs < len(t)]
        valid_to = to[to < len(t)]
        valid_ms = ms[ms < len(t)]
        if len(valid_hs):
            ax.plot(t[valid_hs], sig[valid_hs], 'r^', ms=5)
        if len(valid_to):
            ax.plot(t[valid_to], sig[valid_to], 'go', ms=5)
        if len(valid_ms):
            ax.plot(t[valid_ms], sig[valid_ms], 'b*', ms=4)

        ax.set_title(f"{label}  (HS={len(hs)}, TO={len(to)}, MS={len(ms)})", fontsize=12)
        ax.set_ylabel("Angular Velocity (rad/s)")
        ax.grid(True, alpha=0.3)
        ax.legend(handles=[
            mpatches.Patch(color="red", alpha=0.5, label=f"HS ({len(hs)})"),
            mpatches.Patch(color="green", alpha=0.5, label=f"TO ({len(to)})"),
            mpatches.Patch(color="blue", alpha=0.3, label=f"MS ({len(ms)})"),
        ], loc="upper right", fontsize=8)

    _draw(ax0, t_L, sig_L, left,  "Left Foot (L6)")
    _draw(ax1, t_R, sig_R, right, "Right Foot (R6)")
    ax1.set_xlabel("Time (s)")
    fig.suptitle(title, fontsize=14, fontweight="bold")
    plt.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=200, bbox_inches="tight")
        print(f"[plot] 图像已保存: {save_path}")
    else:
        plt.show()
    plt.close(fig)


# ---- 4. 步态事件检测调试 (HS/TO) ----
print("\n" + "=" * 60)
print("【调试】步态事件检测 — HS / TO 点")
print("=" * 60)

from gait.event_detection import gait_to_hs_from_filtered_gyro_x, compute_gait_phase_metrics_v2

fs = config.fs
res_gyro = gait_to_hs_from_filtered_gyro_x(
    left_gyr_x=arr_L6[:, 3],
    right_gyr_x=arr_R6[:, 3],
    fs=fs,
    input_unit='deg',
    return_debug=True,
)

for side_name, side in [("左脚", res_gyro["left"]), ("右脚", res_gyro["right"])]:
    print(f"\n--- {side_name} ---")
    print(f"  HS 检测点数: {len(side['HS_idx'])}")
    print(f"  TO 检测点数: {len(side['TO_idx'])}")
    print(f"  MS 检测点数: {len(side['MS_idx'])}")
    if len(side['HS_idx']) > 0:
        print(f"  HS_idx (前10): {side['HS_idx'][:10].tolist()}")
    if len(side['TO_idx']) > 0:
        print(f"  TO_idx (前10): {side['TO_idx'][:10].tolist()}")
    if len(side['MS_idx']) > 0:
        print(f"  MS_idx (前10): {side['MS_idx'][:10].tolist()}")

    ranges = side['ranges']
    print(f"  有效周期数 (ranges): {len(ranges)}")
    if ranges:
        print(f"  ranges 前5组 (HS1, TO, HS2): {ranges[:5]}")

    cyc = side['cycle_s']
    sta = side['stance_s']
    swi = side['swing_s']
    if len(cyc) > 0:
        print(f"  周期时间(s)  mean={cyc.mean():.3f}  std={cyc.std():.3f}  values={np.round(cyc, 3).tolist()}")
    if len(sta) > 0:
        print(f"  支撑相(s)    mean={sta.mean():.3f}  std={sta.std():.3f}")
    if len(swi) > 0:
        print(f"  摆动相(s)    mean={swi.mean():.3f}  std={swi.std():.3f}")

# 相位详情 DataFrame
phase_dfs = compute_gait_phase_metrics_v2(res_gyro, fs=fs)
for side_name, df in [("左脚", phase_dfs["left"]), ("右脚", phase_dfs["right"])]:
    if df is not None and len(df) > 0:
        print(f"\n--- {side_name} 相位详情 (前5周期) ---")
        print(df.head(5).to_string(index=False))

# 绘制 HS/TO 事件检测图
plot_gait_events(
    left_gyr_x=arr_L6[:, 3],
    right_gyr_x=arr_R6[:, 3],
    res_gyro=res_gyro,
    fs=fs,
    title="Gait Event Detection — HS / TO / MS",
    save_path=os.path.join(config.curveDir, "gait_events_debug.png"),
)

# ---- 5. 空间指标调试 (步长/步宽/抬脚高度) ----
print("\n" + "=" * 60)
print("【调试】空间指标 — 步长 / 步宽 / 抬脚高度")
print("=" * 60)

from gait.distance_metrics import run_dual_detector_spatial_average_pipeline

try:
    spatial_result = run_dual_detector_spatial_average_pipeline(
        arr_L6, arr_R6, fs=fs
    )

    for tag, key in [("偏大检测器", "metrics_big"), ("偏小检测器", "metrics_small"), ("平均", "metrics_avg")]:
        m = spatial_result[key]
        print(f"\n--- {tag} ---")
        print(f"  步长 left  (m): {m['stepLength']['left']}")
        print(f"  步长 right (m): {m['stepLength']['right']}")
        print(f"  步长偏差   (m): {m['stepLengthDeviation']}")
        print(f"  跨步长     (m): {m['strideLength']}")
        print(f"  步宽       (m): {m['stepWidth']}")
        print(f"  行走距离   (m): {m['walkingDistance']}")
        print(f"  行走速度 left  (m/s): {m['walkingSpeed']['left']}")
        print(f"  行走速度 right (m/s): {m['walkingSpeed']['right']}")
        print(f"  抬脚高度 left  (m): {m['footLiftHeight']['left']}")
        print(f"  抬脚高度 right (m): {m['footLiftHeight']['right']}")

        dbg = m.get('_debug', {})
        if dbg:
            print(f"  [逐周期列表]")
            for dk in ['stride_L_all', 'stride_R_all', 'step_L_all', 'step_R_all',
                       'step_widths_all', 'lift_L_all', 'lift_R_all']:
                vals = dbg.get(dk, [])
                if vals:
                    print(f"    {dk}: {np.round(vals, 3).tolist()}")

except Exception as e:
    print(f"  空间指标计算失败: {e}")
    import traceback
    traceback.print_exc()

# ---- 6. 运行完整步态分析管线 (最终 JSON) ----
print("\n" + "=" * 60)
print("【主流程】运行完整步态分析管线")
print("=" * 60)

from gait.gait_pipeline import run_gait_pipeline

try:
    result = run_gait_pipeline(fs=100, return_json=True)
    print(f"  步态分析完成")
    if isinstance(result, str):
        print(f"  最终 JSON 总长: {len(result)} 字符")
        print(f"\n  前 500 字符预览:")
        print(result)
    else:
        print(f"  返回类型: {type(result).__name__}")
except Exception as e:
    print(f"  步态分析失败: {e}")
    import traceback
    traceback.print_exc()

print(f"\n{'=' * 60}")
print("测试完成。")
print(f"{'=' * 60}")
