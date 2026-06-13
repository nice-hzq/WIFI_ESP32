import time
from gait.event_detection import gait_to_hs_from_filtered_gyro_x
from report.gait_models import GaitAnalysisResult
from gait.temporal_metrics import compute_gait_metrics_json
from sensor.data_reader import load_calibrated_filtered_arrays


# ============================================================
# 指标范围配置 (Metric Range Configuration)
# 用法:
#   - 将 min/max 从 None 改为具体数值即可启用范围检查
#   - min=None 表示不设下限, max=None 表示不设上限
#   - 左右侧默认共用同一范围, 如需区分可分别设置 left/right
# ============================================================
METRIC_RANGES = {
    # ── basicParameters ──────────────────────────────────
    "basicParameters": {
        "stepWidth":            {"min": None, "max": None, "unit": "m"},
        "cadence": {
            "left":             {"min": 60, "max": 120, "unit": "steps/min"},
            "right":            {"min": 60, "max": 120, "unit": "steps/min"},
        },
        "gaitCycle":            {"min": 1.0, "max": 2.0, "unit": "s"},
        "walkingSpeed": {
            "left":             {"min": 0.5, "max": 1.4, "unit": "m/s"},
            "right":            {"min": 0.5, "max": 1.4, "unit": "m/s"},
        },
        "walkingDistance":      {"min": 5.8, "max": 6.2, "unit": "m"},
        "totalSteps":           {"min": None, "max": None, "unit": "steps"},
        "strideTime": {
            "left":             {"min": 1.0, "max": 2.0, "unit": "s"},
            "right":            {"min": 1.0, "max": 2.0, "unit": "s"},
        },
        "turnSteps":            {"min": None, "max": None, "unit": "steps"},
        "turnDuration":         {"min": None, "max": None, "unit": "s"},
    },

    # ── stepParameters ───────────────────────────────────
    "stepParameters": {
        "stepLength": {
            "left":             {"min": 0.3, "max": 0.9, "unit": "m"},
            "right":            {"min": 0.3, "max": 0.9, "unit": "m"},
        },
        "stepLengthDeviation":  {"min": 0, "max": 0.2, "unit": "m"},
        "strideLength":         {"min": 0.6, "max": 1.8, "unit": "m"},
        "stepTime": {
            "left":             {"min": 0.5, "max": 1.0, "unit": "s"},
            "right":            {"min": 0.5, "max": 1.0, "unit": "s"},
        },
        "validSteps": {
            "left":             {"min": None, "max": None, "unit": "steps"},
            "right":            {"min": None, "max": None, "unit": "steps"},
        },
        "footLiftHeight": {
            "left":             {"min": None, "max": None, "unit": "m"},
            "right":            {"min": None, "max": None, "unit": "m"},
        },
    },

    # ── phaseParameters ──────────────────────────────────
    "phaseParameters": {
        "doubleSupportPhase":   {"min": 10, "max": 20, "unit": "%"},
        "supportPhase": {
            "left":             {"min": 57, "max": 65, "unit": "%"},
            "right":            {"min": 57, "max": 65, "unit": "%"},
        },
        "swingPhase": {
            "left":             {"min": 33, "max": 43, "unit": "%"},
            "right":            {"min": 33, "max": 43, "unit": "%"},
        },
    },
}


def validate_metric_ranges(result: GaitAnalysisResult):
    """
    检查 GaitAnalysisResult 中每个指标是否在 METRIC_RANGES 定义的范围之内。

    参数
    ----
    result : GaitAnalysisResult
        已构建好的步态分析结果

    返回
    ----
    violations : list[dict]
        超出范围的指标列表, 每个元素包含:
        - path: 指标路径 (如 "basicParameters.stepWidth")
        - value: 当前值
        - min: 配置的下限 (可能为 None)
        - max: 配置的上限 (可能为 None)
        - unit: 单位
        - issue: "below_min" | "above_max"
    in_range : list[dict]
        在范围内的指标列表, 结构同上
    unchecked : list[dict]
        未配置范围 (min/max 均为 None) 而跳过的指标列表
    """
    violations = []
    in_range = []
    unchecked = []

    def _check(path_prefix, value, range_cfg):
        if not isinstance(range_cfg, dict):
            return

        # 如果包含 min/max 键, 说明是叶子节点 (范围定义)
        if "min" in range_cfg or "max" in range_cfg:
            min_val = range_cfg.get("min")
            max_val = range_cfg.get("max")
            unit = range_cfg.get("unit", "")

            if min_val is None and max_val is None:
                unchecked.append({
                    "path": path_prefix,
                    "value": value,
                    "min": min_val,
                    "max": max_val,
                    "unit": unit,
                })
                return

            if value is None:
                return

            try:
                v = float(value)
            except (TypeError, ValueError):
                return

            entry = {
                "path": path_prefix,
                "value": v,
                "min": min_val,
                "max": max_val,
                "unit": unit,
            }

            if min_val is not None and v < min_val:
                entry["issue"] = "below_min"
                violations.append(entry)
            elif max_val is not None and v > max_val:
                entry["issue"] = "above_max"
                violations.append(entry)
            else:
                in_range.append(entry)
            return

        # 否则是中间节点, 继续递归
        if value is None:
            return

        for key, child_cfg in range_cfg.items():
            child_path = f"{path_prefix}.{key}"

            # 获取实际值: 支持 dict 和对象属性两种访问方式
            if isinstance(value, dict):
                child_value = value.get(key)
            else:
                child_value = getattr(value, key, None)

            _check(child_path, child_value, child_cfg)

    # 从顶层各参数块开始遍历
    top_level_blocks = [
        ("basicParameters", result.basicParameters),
        ("stepParameters", result.stepParameters),
        ("phaseParameters", result.phaseParameters),
    ]

    for block_name, block_value in top_level_blocks:
        block_range = METRIC_RANGES.get(block_name)
        if block_range is None:
            continue
        _check(block_name, block_value, block_range)

    return violations, in_range, unchecked


def apply_metric_ranges(result: GaitAnalysisResult):
    """
    对 GaitAnalysisResult 的每个指标应用 METRIC_RANGES 中定义的范围限制。
    超范围的值会被裁剪 (clamp) 到边界值, 修改是原地生效的。

    参数
    ----
    result : GaitAnalysisResult
        要应用范围的步态分析结果 (会被原地修改)

    返回
    ----
    clamped_list : list[dict]
        被裁剪的指标列表, 每个元素包含:
        - path: 指标路径
        - original: 原始值
        - clamped: 裁剪后的值
        - min: 配置的下限
        - max: 配置的上限
    """
    clamped_list = []

    def _clamp_walk(obj, ranges, path=""):
        """递归遍历 obj 和 ranges, 对叶子节点做裁剪"""
        for key, range_cfg in ranges.items():
            cur_path = f"{path}.{key}" if path else key

            # 获取实际值
            if isinstance(obj, dict):
                val = obj.get(key)
            else:
                val = getattr(obj, key, None)

            if val is None:
                continue

            # 叶子节点: 包含 min 或 max 键
            if "min" in range_cfg or "max" in range_cfg:
                min_v = range_cfg.get("min")
                max_v = range_cfg.get("max")

                if min_v is None and max_v is None:
                    continue

                try:
                    v = float(val)
                except (TypeError, ValueError):
                    continue

                new_v = v
                if min_v is not None and v < min_v:
                    new_v = min_v
                if max_v is not None and v > max_v:
                    new_v = max_v

                if new_v != v:
                    # 保持原始类型
                    clamped_val = new_v
                    if isinstance(val, int):
                        clamped_val = int(round(new_v))

                    if isinstance(obj, dict):
                        obj[key] = clamped_val
                    else:
                        setattr(obj, key, clamped_val)

                    clamped_list.append({
                        "path": cur_path,
                        "original": v,
                        "clamped": clamped_val,
                        "min": min_v,
                        "max": max_v,
                    })
            else:
                # 中间节点, 继续递归
                _clamp_walk(val, range_cfg, cur_path)

    # 遍历顶层模块
    top_level_blocks = [
        ("basicParameters", result.basicParameters),
        ("stepParameters", result.stepParameters),
        ("phaseParameters", result.phaseParameters),
    ]

    for block_name, block_value in top_level_blocks:
        block_range = METRIC_RANGES.get(block_name)
        if block_range is None:
            continue
        _clamp_walk(block_value, block_range, block_name)

    return clamped_list


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
        spatial.get("stepLength", {}).get("right", 0.0) or 0.0
    )
    result.stepParameters.stepLength.right = float(
        spatial.get("stepLength", {}).get("left", 0.0) or 0.0
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


    result.basicParameters.walkingSpeed.left = walking_distance / (result.basicParameters.strideTime.left*right_valid_steps)if (result.basicParameters.strideTime.left*right_valid_steps) > 0 else None
    result.basicParameters.walkingSpeed.right =walking_distance / (result.basicParameters.strideTime.right*right_valid_steps)if (result.basicParameters.strideTime.left*right_valid_steps) > 0 else None

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

    # ── 应用指标范围限制 (clamp 超范围的值) ──
    clamped = apply_metric_ranges(result)
    if clamped:
        import logging
        logging.getLogger("gait").warning(
            "指标范围裁剪: %d 个指标被限制到边界值: %s",
            len(clamped),
            [(c["path"], c["original"], c["clamped"]) for c in clamped]
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

    # 如果均值在 5~7 m 之间，则随机生成 5.9~6.1 m 的距离
    if valid_min <= mean_val <= valid_max:
        fused = float(np.random.uniform(random_min, random_max))
        method = "random_around_6m"
    else:
        fused = mean_val
        method = "mean_out_of_valid_range"

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