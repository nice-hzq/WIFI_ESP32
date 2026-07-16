# gait_pipeline.py - 完整的步态分析管线
import sys, os

# 确保 python/ 在 sys.path 中（兼容直接运行和 import）
_cur_dir = os.path.dirname(os.path.abspath(__file__))
_root_dir = os.path.dirname(_cur_dir)  # python/
if _root_dir not in sys.path:
    sys.path.insert(0, _root_dir)

import shutil
from core import config
from gait.distance_metrics import run_dual_detector_spatial_average_pipeline
from gait.tool import safe_float, safe_div
from sensor.data_reader import load_calibrated_filtered_arrays

from gait.event_detection import gait_to_hs_from_filtered_gyro_x
from gait.temporal_metrics import compute_gait_metrics_json
from gait.metrics_builder import build_gait_analysis_result

import time, json, math
import numpy as np
from gait.count_phase import _mahony_quats_from_array, detect_turn_segments_waist, gait_from_feet_gyro_x, count_steps_during_turns
from report.gait_models import *


def run_gait_pipeline(
    *,
    fs: int = 50,
    window_size: int = 5,
    gyro_unit_for_files: str = 'deg',
    use_mag: bool = True,

    # —— 传感器别名（可仅指定双脚）——
    sensor_aliases: list = None,   # None=从 WORK_MODE 自动选择; 例: ["L6","R6"]

    # —— 腰部转身检测参数 ——
    no_turn: bool = False,         # True=强制跳过转身检测（无双脚以外传感器时自动 True）
    angle_win_s: float = 1.0,
    angle_thr_on: float = 35.0,
    angle_thr_off: float = 20.0,
    yawrate_on: float = 35.0,
    yawrate_off: float = 20.0,
    min_turn_dur_s: float = 0.30,
    min_total_angle_deg: float = 40.0,
    off_hold_s: float = 0.15,
    bridge_gap_s: float = 0.40,
    med_win_s: float = 0.12,
    lp_cut_hz: float = 2.5,

    # —— 腰部在转身片段内计步（陀螺模长）——
    turn_step_axis: str = 'norm',
    turn_hp_cut: float = 0.7,
    turn_lp_cut: float = 6.0,
    turn_smooth_win_s: float = 0.08,
    turn_min_step_dist_s: float = 0.35,
    turn_k_height: float = 1.2,
    turn_k_prom: float = 1.5,

    # —— 脚端事件参数 ——
    feet_input_unit: str = 'deg',
    feet_return_debug: bool = False,

    # —— 输出控制 ——
    return_json: bool = True,     # True: 返回 gait_result.to_json()
    return_dict: bool = False     # True: 返回 gait_result.to_dict()
):
    """
    步态分析管线。

    支持模式:
      - 完整下肢 (lower_body): S1+L4+L5+L6+R4+R5+R6 — 含转身检测
      - 简易双足 (feet_only):   L6+R6 — 仅步态事件+空间参数，无转身
      - AUTO: 当 S1 为 None 时自动跳过转身检测

    返回：
      - return_json=True  -> JSON字符串
      - return_dict=True  -> dict
      - 否则返回 GaitAnalysisResult 对象
    """

    import time

    # =====================================================
    # 1) 读取 + 校准 + 滤波
    # =====================================================
    if sensor_aliases is None:
        # 从 WORK_MODE 自动选择
        mode = (config.WORK_MODE or "").lower()
        if mode == "feet_only":
            sensor_aliases = ["L6", "R6"]
        elif mode == "lower_body":
            sensor_aliases = ["S1", "L4", "L5", "L6", "R4", "R5", "R6"]
        elif mode == "upper_body":
            sensor_aliases = ["H", "T1", "T12", "L1", "L2", "L3", "R1", "R2", "R3"]
        else:
            sensor_aliases = ["H", "T1", "T12", "L1", "L2", "L3", "R1", "R2", "R3",
                              "S1", "L4", "L5", "L6", "R4", "R5", "R6"]

    arrs = load_calibrated_filtered_arrays(
        window_size=5,
        columns=["Acc_x", "Acc_y", "Acc_z", "Gyr_x", "Gyr_y", "Gyr_z", "Geo_x", "Geo_y", "Geo_z"],
        aliases=sensor_aliases)

    # 建立 alias -> array 的查找表
    arr_map = dict(zip(sensor_aliases, arrs))

    # =====================================================
    # ★ 保存传感器姿态曲线到 output 目录
    # =====================================================
    if config.curveDir:
        try:
            # 姿态曲线是可选输出。旧版本仓库可能不包含该绘图模块，
            # 因此必须延迟导入，不能让可选功能阻断整个步态分析。
            from output.attitude_curves import save_attitude_curves
            # 过滤掉 None 的数据
            valid_data = {k: v for k, v in arr_map.items() if v is not None}
            save_attitude_curves(
                valid_data,
                fs=fs,
                output_dir=config.curveDir,
                kp=1.5, ki=0.05,
                use_mag=use_mag,
                gyro_unit=gyro_unit_for_files,
            )
        except Exception as e:
            print(f"[WARN] 姿态曲线保存失败: {e}")

    arr_S1 = arr_map.get("S1", None)
    arr_L6 = arr_map.get("L6", None)
    arr_R6 = arr_map.get("R6", None)

    # 验证双脚传感器必须存在
    if arr_L6 is None or arr_R6 is None:
        raise ValueError(
            f"双脚传感器 L6/R6 数据缺失。"
            f"可用传感器: {[a for a, v in arr_map.items() if v is not None]}"
        )

    # 自动判断是否有腰部数据
    has_waist = (arr_S1 is not None and len(arr_S1) > 0)
    do_turns = has_waist and not no_turn

    # =====================================================
    # 2) 后背姿态 + 转身检测（仅当有腰部传感器时）
    # =====================================================
    if do_turns:
        quats_back, eulers_back, yaw_back_cont = _mahony_quats_from_array(
            arr_S1,
            fs=fs,
            kp=1.5,
            ki=0.05,
            use_mag=use_mag,
            normalize_with_quat=True,
            ref_index=0,
            gyro_unit=gyro_unit_for_files,
            yaw_max_rate_dps=180.0
        )

        turn_windows_s, dbg, avg_turn_time = detect_turn_segments_waist(
            yaw_back_cont,
            fs=fs,
            angle_win_s=angle_win_s,
            angle_thr_on=angle_thr_on,
            angle_thr_off=angle_thr_off,
            yawrate_on=yawrate_on,
            yawrate_off=yawrate_off,
            min_turn_dur_s=min_turn_dur_s,
            min_total_angle_deg=min_total_angle_deg,
            off_hold_s=off_hold_s,
            bridge_gap_s=bridge_gap_s,
            med_win_s=med_win_s,
            lp_cut_hz=lp_cut_hz
        )
    else:
        turn_windows_s = []
        avg_turn_time = 0.0
        print("[INFO] 无腰部传感器 (S1)，跳过转身检测。")

    # =====================================================
    # 3) 左右脚事件检测
    # =====================================================
    res_gyro = gait_to_hs_from_filtered_gyro_x(
        left_gyr_x=arr_L6[:, 3],
        right_gyr_x=arr_R6[:, 3],
        fs=fs,
        input_unit=feet_input_unit,
        hs_method="zero_cross",
        return_debug=True
    )

    # =====================================================
    # 4) 空间参数：双检测器平均
    # =====================================================
    # 注意：run_dual_detector_spatial_average_pipeline(arr2=右脚, arr3=左脚)
    fusion_result = run_dual_detector_spatial_average_pipeline(
        arr_R6, arr_L6, fs=fs
    )

    # =====================================================
    # 5) 时间 / 相位参数
    # =====================================================
    metrics = compute_gait_metrics_json(res_gyro, fs=fs)

    # =====================================================
    # 6) 估计 sessionDuration
    # =====================================================
    if has_waist:
        session_duration = len(arr_S1) / float(fs)
    else:
        session_duration = max(len(arr_L6), len(arr_R6)) / float(fs)

    # =====================================================
    # 7) 组装为 gait_models.py 大 JSON
    # =====================================================
    gait_result = build_gait_analysis_result(
        fusion_result=fusion_result,
        metrics=metrics,
        timestamp=int(time.time() * 1000),
        session_duration=session_duration
    )

    # =====================================================
    # 8) 补入转身相关结果（仅当有腰部传感器时）
    # =====================================================
    if do_turns:
        turn_step_counts = count_steps_during_turns(
            arr_S1, fs, turn_windows_s,
            axis=turn_step_axis,
            hp_cut=turn_hp_cut,
            lp_cut=turn_lp_cut,
            smooth_win_s=turn_smooth_win_s,
            min_step_dist_s=turn_min_step_dist_s,
            k_height=turn_k_height,
            k_prom=turn_k_prom,
            return_debug=False
        )
        gait_result.basicParameters.turnSteps = int(sum(turn_step_counts.get("turn_steps", [])))
        gait_result.basicParameters.turnDuration = float(avg_turn_time or 0.0)
    else:
        gait_result.basicParameters.turnSteps = 0
        gait_result.basicParameters.turnDuration = 0.0

    # =====================================================
    # 9) 输出
    # =====================================================
    if return_dict:
        return gait_result.to_dict()

    if return_json:
        return gait_result.to_json()

    return gait_result


def clear_directory(directory_path):
    """Clears all files in the given directory."""
    if os.path.exists(directory_path) and os.path.isdir(directory_path):
        for file_name in os.listdir(directory_path):
            file_path = os.path.join(directory_path, file_name)
            try:
                if os.path.isfile(file_path):
                    os.remove(file_path)
                elif os.path.isdir(file_path):
                    shutil.rmtree(file_path)
            except Exception as e:
                print(f"Error while deleting file {file_path}: {e}")
    else:
        print(f"Directory {directory_path} does not exist.")


def create_gait_result():
    """创建并返回 GaitAnalysisResult 对象"""
    clear_directory(config.curveDir.replace("\\", "/"))
    result = run_gait_pipeline(fs=config.fs, return_json=False, return_dict=False)
    return result


def get_gait_analysis_object():
    """返回 GaitAnalysisResult 对象（供 C# 直接调用）"""
    return create_gait_result()


def print_gait_summary():
    """返回 JSON 字符串"""
    result = create_gait_result()
    return result.to_json()


if __name__ == "__main__":
    config.tempDir = os.path.join(_root_dir, "temp")
    config.originalDir = os.path.join(_root_dir, "Data", "步态评估数据", "2")
    config.curveDir = os.path.join(_root_dir, "output")
    config.WORK_MODE = "lower_body"
    config.fs = 50
    os.makedirs(config.curveDir, exist_ok=True)

    print("=" * 60)
    print("Gait Analysis Pipeline")
    print(f"  tempDir     = {config.tempDir}")
    print(f"  originalDir = {config.originalDir}")
    print(f"  curveDir    = {config.curveDir}")
    print(f"  WORK_MODE   = {config.WORK_MODE}")
    print("=" * 60)

    try:
        result_json = print_gait_summary()
        print(result_json)
        print(f"\n[OK] Done. Output files in: {config.curveDir}")
    except KeyboardInterrupt:
        print("\n[中断] 用户取消了运算 (KeyboardInterrupt)")
