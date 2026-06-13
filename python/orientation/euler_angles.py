# src/orientation/euler_angles.py

import numpy as np

from core.quaternion import quat_normalize, wrap_to_pi


def quat_wxyz_to_euler_zyx(q: np.ndarray, degrees: bool = False):
    """
    四元数 (w,x,y,z) -> 欧拉角 (roll, pitch, yaw)
    旋转顺序：Z-Y-X（yaw-pitch-roll）

    q: (..., 4)
    return: (..., 3) [roll, pitch, yaw]
    默认返回 rad
    """
    q = np.asarray(q, float)

    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]

    # roll (x-axis)
    sinr_cosp = 2.0 * (w*x + y*z)
    cosr_cosp = 1.0 - 2.0 * (x*x + y*y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis)
    sinp = 2.0 * (w*y - z*x)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)

    # yaw (z-axis)
    siny_cosp = 2.0 * (w*z + x*y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    euler = np.stack([roll, pitch, yaw], axis=-1)

    if degrees:
        euler = np.rad2deg(euler)

    return euler


def extract_flexion_angle_from_quat(
    q_rel: np.ndarray,
    flexion_axis: int = 0
) -> np.ndarray:
    """
    从相对四元数中提取屈曲/伸展角
    q_rel: (N, 4), 格式 [w, x, y, z]
    返回单位: rad，并限制到 [-pi, pi]
    """
    q_rel = quat_normalize(q_rel)

    euler = quat_wxyz_to_euler_zyx(q_rel, degrees=False)
    ang = euler[..., flexion_axis]

    ang = wrap_to_pi(ang)

    return ang
# arr1, arr2, arr3 = load_calibrated_filtered_arrays(
#         window_size=5,
#         columns=["Acc_x","Acc_y","Acc_z","Gyr_x","Gyr_y","Gyr_z","Geo_x","Geo_y","Geo_z"],
#         aliases=["S1", "R6", "L6"]
#     )
#
# mgr = QuaternionManager(fs=100)
#
# # 步态用：脚部建议先关磁力计（抗干扰更稳）
# mgr.add_node("L_foot", use_mag=False, kp= 1, ki = 0.005)
# mgr.add_node("R_foot", use_mag=False, kp= 1, ki = 0.005)
#
# # 腰部如果你想要航向更稳定，可以开磁力计（可选）
# mgr.add_node("Waist", use_mag=False, kp=0.6, ki=1e-5)
#
# Q = mgr.run_batch({
#     "L_foot": arr3,
#     "R_foot": arr2,
#     "Waist":  arr1,
# })
#
# quat_L_wxyz = Q["L_foot"]
# quat_R_wxyz = Q["R_foot"]
# quat_W_wxyz = Q["Waist"]
#
#
# euler_L = quat_wxyz_to_euler_zyx(quat_L_wxyz)
# euler_R = quat_wxyz_to_euler_zyx(quat_R_wxyz)
# euler_W = quat_wxyz_to_euler_zyx(quat_W_wxyz)
#
#
# import numpy as np
# import matplotlib.pyplot as plt
#
#
# def plot_euler_three_sensors(
#     euler_L: np.ndarray,
#     euler_R: np.ndarray,
#     euler_W: np.ndarray,
#     fs: float,
#     *,
#     units: str = "deg",   # "deg" or "rad"
#     sensor_names=("Left Foot", "Right Foot", "Waist"),
#     figsize=(14, 10),
# ):
#     """
#     绘制三个传感器的欧拉角曲线（ZYX: roll, pitch, yaw）
#
#     Parameters
#     ----------
#     euler_L, euler_R, euler_W : (N,3)
#         欧拉角 [roll, pitch, yaw]
#     fs : float
#         采样率
#     units : str
#         角度单位标注
#     """
#
#     eulers = [euler_L, euler_R, euler_W]
#     labels = ["Roll", "Pitch", "Yaw"]
#
#     N = min(e.shape[0] for e in eulers)
#     t = np.arange(N) / fs
#
#     fig, axes = plt.subplots(
#         nrows=3, ncols=3, figsize=figsize, sharex=True
#     )
#
#     for col, (euler, sname) in enumerate(zip(eulers, sensor_names)):
#         for row in range(3):
#             ax = axes[row, col]
#             ax.plot(t, euler[:N, row], linewidth=1.2)
#             ax.set_ylabel(f"{labels[row]} ({units})")
#             ax.grid(True, linestyle="--", alpha=0.4)
#
#             if row == 0:
#                 ax.set_title(sname)
#
#             if row == 2:
#                 ax.set_xlabel("Time (s)")
#
#     plt.tight_layout()
#     plt.show()
# plot_euler_three_sensors(
#     euler_L,
#     euler_R,
#     euler_W,
#     fs=100,
# )
