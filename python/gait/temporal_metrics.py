import numpy as np
import json

from gait.event_detection import gait_to_hs_from_filtered_gyro_x_adaptive

def compute_total_cadence_from_hs(left_hs, right_hs, fs=50):
    left_hs = np.asarray(left_hs, dtype=int)
    right_hs = np.asarray(right_hs, dtype=int)

    all_hs = np.sort(np.concatenate([left_hs, right_hs]))
    if len(all_hs) < 2:
        return None

    duration_s = (all_hs[-1] - all_hs[0]) / fs
    total_steps = len(all_hs) - 1

    if duration_s <= 0:
        return None

    cadence = 60.0 * total_steps / duration_s
    return float(cadence)
def compute_gait_metrics_json(res_gyro, fs=50):
    left = res_gyro["left"]
    right = res_gyro["right"]

    # ========= 1) 取基础数组 =========
    l_hs = np.asarray(left["HS_idx"], dtype=int)
    l_to = np.asarray(left["TO_idx"], dtype=int)
    r_hs = np.asarray(right["HS_idx"], dtype=int)
    r_to = np.asarray(right["TO_idx"], dtype=int)

    l_cycle = np.asarray(left["cycle_s"], dtype=float)
    r_cycle = np.asarray(right["cycle_s"], dtype=float)
    l_stance = np.asarray(left["stance_s"], dtype=float)
    r_stance = np.asarray(right["stance_s"], dtype=float)
    l_swing = np.asarray(left["swing_s"], dtype=float)
    r_swing = np.asarray(right["swing_s"], dtype=float)

    # ========= 2) 基本时间参数 =========
    stride_left = float(np.mean(l_cycle)) if len(l_cycle) > 0 else None
    stride_right = float(np.mean(r_cycle)) if len(r_cycle) > 0 else None

    gait_cycle = float(np.mean(
        [x for x in [stride_left, stride_right] if x is not None]
    )) if (stride_left is not None or stride_right is not None) else None

    support_left = float(np.mean(l_stance / l_cycle) * 100) if len(l_cycle) > 0 else None
    support_right = float(np.mean(r_stance / r_cycle) * 100) if len(r_cycle) > 0 else None

    swing_left = float(np.mean(l_swing / l_cycle) * 100) if len(l_cycle) > 0 else None
    swing_right = float(np.mean(r_swing / r_cycle) * 100) if len(r_cycle) > 0 else None

    # ========= 3) step time =========
    def calc_step_time(hs_a, hs_b, fs):
        vals = []
        for a in hs_a:
            future_b = hs_b[hs_b > a]
            if len(future_b) > 0:
                vals.append((future_b[0] - a) / fs)
        return np.array(vals, dtype=float)

    step_left_arr = calc_step_time(l_hs, r_hs, fs)   # 左HS -> 下一次右HS
    step_right_arr = calc_step_time(r_hs, l_hs, fs)  # 右HS -> 下一次左HS

    step_left = float(np.mean(step_left_arr)) if len(step_left_arr) > 0 else None
    step_right = float(np.mean(step_right_arr)) if len(step_right_arr) > 0 else None
    # total_step = step_left + step_right
    #
    # cadence_left = float(60.0 / step_left) if step_left and step_left > 0 else None
    # cadence_right = float(60.0 / step_right) if step_right and step_right > 0 else None

    #
    # cadence_left = float(60.0 / total_step) if step_left and step_left > 0 else None
    # cadence_right = float(60.0 / total_step) if step_right and step_right > 0 else None
    # cadence_left = float(60.0 * len(step_left_arr) / np.sum(step_left_arr)) if len(step_left_arr) > 0 else None
    # cadence_right = float(60.0 * len(step_right_arr) / np.sum(step_right_arr)) if len(step_right_arr) > 0 else None
    left_hs = np.asarray(left['HS_idx'], dtype=int)
    right_hs = np.asarray(right['HS_idx'], dtype=int)

    all_hs = np.sort(np.concatenate([left_hs, right_hs]))

    if len(all_hs) >= 2:
        total_time = (all_hs[-1] - all_hs[0]) / fs
    else:
        total_time = 0.0

    left_steps = max(len(left_hs) , 0)
    right_steps = max(len(right_hs) , 0)

    cadence_left = float(60.0 * left_steps / total_time) if total_time > 0 else None
    cadence_right = float(60.0 * right_steps / total_time) if total_time > 0 else None

    # cadence_total = compute_total_cadence_from_hs(
    #     left["HS_idx"], right["HS_idx"], fs=50
    # )
    # print("总步频:", cadence_total)
    # ========= 4) 正确构造支撑区间 =========
    # 你的数据模式是 TO[i] < HS[i] < TO[i+1]
    # 所以 stance interval 应该是 [HS[i], TO[i+1]]
    def build_stance_intervals(hs_idx, to_idx):
        intervals = []
        for hs in hs_idx:
            future_to = to_idx[to_idx > hs]
            if len(future_to) > 0:
                intervals.append((int(hs), int(future_to[0])))
        return intervals

    l_stance_intervals = build_stance_intervals(l_hs, l_to)
    r_stance_intervals = build_stance_intervals(r_hs, r_to)

    # ========= 5) 双支撑时间 =========
    def overlap_len(a0, a1, b0, b1):
        return max(0, min(a1, b1) - max(a0, b0))

    def compute_double_support_by_cycle(ref_hs, ref_intervals, other_intervals):
        """
        以 ref 的 gait cycle: HS[i] -> HS[i+1] 为参考，
        计算该周期内两侧 stance intervals 的重叠总时长占比
        """
        ds_list = []

        for i in range(len(ref_hs) - 1):
            cyc_start = ref_hs[i]
            cyc_end = ref_hs[i + 1]
            cyc_len = cyc_end - cyc_start
            if cyc_len <= 0:
                continue

            total_overlap = 0

            # ref侧当前周期内可能只有一个主stance interval
            for a0, a1 in ref_intervals:
                if a1 <= cyc_start or a0 >= cyc_end:
                    continue
                aa0 = max(a0, cyc_start)
                aa1 = min(a1, cyc_end)

                for b0, b1 in other_intervals:
                    if b1 <= cyc_start or b0 >= cyc_end:
                        continue
                    bb0 = max(b0, cyc_start)
                    bb1 = min(b1, cyc_end)

                    total_overlap += overlap_len(aa0, aa1, bb0, bb1)

            ds_list.append(total_overlap / cyc_len * 100.0)

        return np.array(ds_list, dtype=float)

    ds_left_ref = compute_double_support_by_cycle(l_hs, l_stance_intervals, r_stance_intervals)
    ds_right_ref = compute_double_support_by_cycle(r_hs, r_stance_intervals, l_stance_intervals)

    ds_candidates = []
    if len(ds_left_ref) > 0:
        ds_candidates.append(float(np.mean(ds_left_ref)))
    if len(ds_right_ref) > 0:
        ds_candidates.append(float(np.mean(ds_right_ref)))

    double_support_phase = float(np.mean(ds_candidates)) if len(ds_candidates) > 0 else None

    # ========= 6) 转身指标（由 run_gait_pipeline 覆盖） =========
    turn_steps = 0
    turn_duration = 0.0

    # ========= 7) 组织为 JSON 兼容格式 =========
    metrics = {
        "cadence": {
            "left": cadence_left,
            "right": cadence_right
        },
        "doubleSupportPhase": double_support_phase,
        "supportPhase": {
            "left": support_left,
            "right": support_right
        },
        "swingPhase": {
            "left": swing_left,
            "right": swing_right
        },
        "gaitCycle": gait_cycle,
        "strideTime": {
            "left": stride_left,
            "right": stride_right
        },
        "stepTime": {
            "left": step_left,
            "right": step_right
        },

        # 只保留这两个转身指标
        "turnSteps": turn_steps,
        "turnDuration": turn_duration,

        "fs_used": float(fs)
    }

    return metrics

    # # ========= 6) 组织为 JSON 兼容格式 =========
    # metrics = {
    #     "cadence": {
    #         "left": cadence_left,
    #         "right": cadence_right
    #     },
    #     "doubleSupportPhase": double_support_phase,
    #     "supportPhase": {
    #         "left": support_left,
    #         "right": support_right
    #     },
    #     "swingPhase": {
    #         "left": swing_left,
    #         "right": swing_right
    #     },
    #     "gaitCycle": gait_cycle,
    #     "strideTime": {
    #         "left": stride_left,
    #         "right": stride_right
    #     },
    #     "stepTime": {
    #         "left": step_left,
    #         "right": step_right
    #     },
    #     "fs_used": float(fs)
    # }
    #
    # return metrics


def metrics_to_json(metrics):
    return json.dumps(metrics, ensure_ascii=False, indent=2)




