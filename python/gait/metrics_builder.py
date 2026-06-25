import time
from gait.event_detection import gait_to_hs_from_filtered_gyro_x
from report.gait_models import GaitAnalysisResult
from gait.temporal_metrics import compute_gait_metrics_json
from sensor.data_reader import load_calibrated_filtered_arrays


# 先确保你已经:
# from gait_models import GaitAnalysisResult
def build_gait_analysis_result(fusion_result, metrics, timestamp=None, session_duration=0.0):
    """
    将
      1) fusion_result["metrics_avg"] 里的空间/距离参数
      2) metrics 里的时间/相位参数
    填入 gait_models.py 定义的大 JSON 结构中。
    没有的字段保持默认 0。

    参数
    ----
    fusion_result : dict
        run_dual_detector_spatial_average_pipeline(...) 的返回结果
    metrics : dict
        compute_gait_metrics_json(...) 的返回结果
    timestamp : int or None
        时间戳，毫秒。None 时自动用当前时间
    session_duration : float
        本次会话持续时间，单位秒
    """

    if timestamp is None:
        timestamp = int(time.time() * 1000)

    # 取平均后的空间参数
    spatial = fusion_result.get("metrics_avg", {})
    stride_length = fusion_result['metrics_avg']['strideLength']

    # 创建总结果对象
    result = GaitAnalysisResult()
    result.timestamp = int(timestamp)
    result.sessionDuration = float(session_duration)


    # =====================================================
    # 1) basicParameters
    # =====================================================
    result.basicParameters.stepWidth = float(spatial.get("stepWidth", 0.0) or 0.0)

    # result.basicParameters.cadence.left = float(
    #     metrics.get("cadence", {}).get("left", 0.0) or 0.0
    # )
    # result.basicParameters.cadence.right = float(
    #     metrics.get("cadence", {}).get("right", 0.0) or 0.0
    # )


    result.basicParameters.gaitCycle = float(metrics.get("gaitCycle", 0.0) or 0.0)

    # result.basicParameters.walkingSpeed.left = float(
    #     spatial.get("walkingSpeed", {}).get("left", 0.0) or 0.0
    # )
    # result.basicParameters.walkingSpeed.right = float(
    #     spatial.get("walkingSpeed", {}).get("right", 0.0) or 0.0
    # )




    #
    # if session_duration and session_duration > 0:
    #     walking_speed = float(result.basicParameters.walkingDistance / session_duration)
    # else:
    #     walking_speed = 0.0
    #
    # result.basicParameters.walkingSpeed.left = walking_speed
    # result.basicParameters.walkingSpeed.right = walking_speed



    # result.basicParameters.walkingDistance = float(
    #     spatial.get("walkingDistance", 0.0) or 0.0
    # )


    # totalSteps：用 validSteps 左右之和，或直接按 HS 数量估计
    left_valid_steps = len(spatial.get("_debug", {}).get("step_L_all", []))
    right_valid_steps = len(spatial.get("_debug", {}).get("step_R_all", []))
    totall_steps = left_valid_steps + right_valid_steps
    total_valid_time = (totall_steps*result.basicParameters.gaitCycle)/2


    result.basicParameters.cadence.left = float(60.0 * totall_steps / total_valid_time) if total_valid_time > 0 else 0.0
    result.basicParameters.cadence.right = float(60.0 * totall_steps / total_valid_time) if total_valid_time > 0 else 0.0

    # 安全取 max — 步数列表可能为空
    step_L_all = spatial.get("_debug", {}).get("step_L_all", [])
    step_R_all = spatial.get("_debug", {}).get("step_R_all", [])
    if step_L_all or step_R_all:
        all_step_max = max(
            max(step_L_all) if step_L_all else 0.0,
            max(step_R_all) if step_R_all else 0.0,
        )
    else:
        all_step_max = 0.0

    # print("asuhdjasdlkadaksd",(all_step_max*totall_steps)/2)

    result.basicParameters.totalSteps = int(left_valid_steps + right_valid_steps)

    dist_max_step = (all_step_max * totall_steps) / 2.0
    dist_small = float(fusion_result["metrics_small"]["walkingDistance"])
    dist_big = float(fusion_result["metrics_big"]["walkingDistance"])


    walking_distance, distance_debug = fuse_distance_to_fixed_6m(
        dist_max_step=dist_max_step,
        dist_small=dist_small,
        dist_big=dist_big,
        invalid_return=None
    )

    result.basicParameters.totalSteps = int(left_valid_steps + right_valid_steps)
    result.basicParameters.walkingDistance = walking_distance

    # print("distance_debug =", distance_debug)


    result.basicParameters.strideTime.left = float(
        metrics.get("strideTime", {}).get("left", 0.0) or 0.0
    )
    result.basicParameters.strideTime.right = float(
        metrics.get("strideTime", {}).get("right", 0.0) or 0.0
    )

    # turnSteps, turnDuration 当前没有，保持默认 0
    result.basicParameters.turnSteps = int(metrics.get("turnSteps", 0) or 0)
    result.basicParameters.turnDuration = float(metrics.get("turnDuration", 0.0) or 0.0)

    # =====================================================
    # 2) stepParameters
    # =====================================================
    result.stepParameters.stepLength.left = float(
        spatial.get("stepLength", {}).get("left", 0.0) or 0.0
    )
    result.stepParameters.stepLength.right = float(
        spatial.get("stepLength", {}).get("right", 0.0) or 0.0
    )

    result.stepParameters.stepLengthDeviation = float(
        spatial.get("stepLengthDeviation", 0.0) or 0.0
    )



    result.stepParameters.strideLength = float(
        spatial.get("strideLength", 0.0) or 0.0
    )

    result.stepParameters.stepTime.left = float(
        metrics.get("stepTime", {}).get("left", 0.0) or 0.0
    )
    result.stepParameters.stepTime.right = float(
        metrics.get("stepTime", {}).get("right", 0.0) or 0.0
    )


    # 优先使用空间管线中逐跨步计算的步速（更准确，左右独立）
    speed_left = spatial.get("walkingSpeed", {}).get("left")
    speed_right = spatial.get("walkingSpeed", {}).get("right")
    if speed_left is None or speed_left == 0:
        # 退化为总距离 / 总时间
        speed_left = walking_distance / session_duration if session_duration > 0 else None
    if speed_right is None or speed_right == 0:
        speed_right = walking_distance / session_duration if session_duration > 0 else None
    result.basicParameters.walkingSpeed.left = speed_left
    result.basicParameters.walkingSpeed.right = speed_right

    # validSteps
    validSteps_left = int(right_valid_steps)
    validSteps_right = int(left_valid_steps)


    result.stepParameters.validSteps.right = validSteps_left + validSteps_right
    result.stepParameters.validSteps.left = validSteps_left + validSteps_right

    result.stepParameters.footLiftHeight.left = float(
        spatial.get("footLiftHeight", {}).get("left", 0.0) or 0.0
    )
    result.stepParameters.footLiftHeight.right = float(
        spatial.get("footLiftHeight", {}).get("right", 0.0) or 0.0
    )

    # =====================================================
    # 3) phaseParameters
    # =====================================================
    result.phaseParameters.doubleSupportPhase = float(
        metrics.get("doubleSupportPhase", 0.0) or 0.0
    )

    result.phaseParameters.supportPhase.left = float(
        metrics.get("supportPhase", {}).get("left", 0.0) or 0.0
    )
    result.phaseParameters.supportPhase.right = float(
        metrics.get("supportPhase", {}).get("right", 0.0) or 0.0
    )

    result.phaseParameters.swingPhase.left = float(
        metrics.get("swingPhase", {}).get("left", 0.0) or 0.0
    )
    result.phaseParameters.swingPhase.right = float(
        metrics.get("swingPhase", {}).get("right", 0.0) or 0.0
    )

    return result


def fuse_distance_to_fixed_6m(
    *,
    dist_max_step,
    dist_small,
    dist_big,
    invalid_return=None,
    valid_min=4.5,
    valid_max=15.5,
    random_min=5.9,
    random_max=6.1
):
    """
    融合三种距离估计值，用于 6 米步行测试。

    规则：
    1. 先提取三个距离估计值中的有效值；
    2. 计算有效距离的均值；
    3. 如果均值在 5~7 m 范围内，说明距离估计基本接近 6 m，
       则在 5.9~6.1 m 之间随机生成一个距离；
    4. 如果均值不在 5~7 m 范围内，说明距离估计偏差较大，
       则直接返回有效距离的均值；
    5. 如果没有有效值，则返回 invalid_return；
       若 invalid_return=None，则返回 0.0。
    """

    import numpy as np

    values = [dist_max_step, dist_small, dist_big]

    valid = []
    for v in values:
        if v is None:
            continue

        try:
            v = float(v)
        except (TypeError, ValueError):
            continue

        if np.isfinite(v) and v > 0:
            valid.append(v)

    if not valid:
        fused = 0.0 if invalid_return is None else invalid_return
        return fused, {
            "method": "none",
            "values": values,
            "valid_values": valid,
            "reason": "no_valid_distance"
        }

    valid_arr = np.array(valid, dtype=float)

    mean_val = float(np.mean(valid_arr))
    median_val = float(np.median(valid_arr))
    value_range = float(np.max(valid_arr) - np.min(valid_arr))

    # 如果均值在 5~7 m 之间，用有效值中位数（确定性，避免随机导致不可复现）
    if valid_min <= mean_val <= valid_max:
        fused = median_val
        method = "median_in_6m_range"
    else:
        fused = median_val
        method = "median_out_of_valid_range"

    return fused, {
        "method": method,
        "values": values,
        "valid_values": valid,
        "mean": mean_val,
        "median": median_val,
        "range": value_range,
        "valid_min": valid_min,
        "valid_max": valid_max,
        "random_min": random_min,
        "random_max": random_max,
        "fused": fused
    }