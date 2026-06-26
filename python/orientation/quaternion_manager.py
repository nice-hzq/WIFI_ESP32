import numpy as np
from core.quaternion import (quat_normalize, quat_conj, quat_inv,
                              quat_mul, quat_rotate, quat_to_rotmat,
                              average_quaternions, quat_relative,
                              quat_to_euler)
from ahrs.filters import Mahony
from ahrs.common.orientation import acc2q, am2q
G0 = 9.80665

def rotate_acc_to_world(acc_s, quat_wxyz, *, direction="world<-sensor", acc_unit="g"):
    """
    acc_s: (N,3) 传感器系加速度
    quat_wxyz: (N,4)
    direction:
      - "world<-sensor": a_w = q ⊗ a_s ⊗ q*
      - "sensor<-world": a_w = q* ⊗ a_s ⊗ q
    """
    acc = np.asarray(acc_s, float).copy()
    if acc_unit.lower() in ["g", "grav"]:
        acc = acc * G0

    Q = np.asarray(quat_wxyz, float)
    N = acc.shape[0]
    acc_w = np.zeros_like(acc, dtype=float)

    for i in range(N):
        q = Q[i]
        if direction == "world<-sensor":
            q_use = q
        else:
            q_use = quat_conj(q)

        acc_w[i] = quat_rotate(q_use, acc[i])

    return acc_w

def remove_gravity_and_static_bias(acc_world, fs=50, n_init=200):
    """
    acc_world: (N,3) 已经旋到 world 的加速度 (m/s^2)
    返回:
      acc_lin_world: 去重力 + 去静止段偏置后的线加速度
      g_vec: 使用的重力向量
      bias: 静止段线加速度均值偏置
    """
    acc_world = np.asarray(acc_world, float)
    mu = acc_world[:n_init].mean(axis=0)

    # 重力方向：按 mu_z 的符号决定 +g 或 -g
    g_sign = 1.0 if mu[2] >= 0 else -1.0
    g_vec = np.array([0.0, 0.0, g_sign*G0], float)

    acc_lin = acc_world - g_vec

    # 静止段线加速度应为 0，做偏置校正
    bias = acc_lin[:n_init].mean(axis=0)
    acc_lin = acc_lin - bias

    return acc_lin, g_vec, bias

def plot_acc_world_check(arr9, quat_wxyz, fs=50, *, direction, acc_unit="g", sec=4.0):
    import matplotlib.pyplot as plt

    acc_s = arr9[:, 0:3]
    acc_w = rotate_acc_to_world(acc_s, quat_wxyz, direction=direction, acc_unit=acc_unit)

    N = int(sec * fs)
    t = np.arange(N)/fs

    # 打印静止段均值
    mu = acc_w[300:N].mean(axis=0)
    print(f"\n[{direction}] acc_world mean (first {sec:.1f}s):", mu,
          "| xy_mag:", np.linalg.norm(mu[:2]),
          "| |mean|:", np.linalg.norm(mu))

    # 画 sensor 三轴 vs world 三轴（静止段）
    fig, axs = plt.subplots(2, 1, sharex=True, figsize=(12,6))

    axs[0].plot(t, acc_s[:N,0], label="ax_s")
    axs[0].plot(t, acc_s[:N,1], label="ay_s")
    axs[0].plot(t, acc_s[:N,2], label="az_s")
    axs[0].set_title("Sensor-frame acc (static segment)")
    axs[0].grid(True); axs[0].legend()

    axs[1].plot(t, acc_w[:N,0], label="ax_w")
    axs[1].plot(t, acc_w[:N,1], label="ay_w")
    axs[1].plot(t, acc_w[:N,2], label="az_w")
    axs[1].axhline( G0, ls="--")
    axs[1].axhline(-G0, ls="--")
    axs[1].set_title(f"World-frame acc (static) | {direction}")
    axs[1].grid(True); axs[1].legend()
    axs[1].set_xlabel("Time (s)")

    plt.tight_layout()
    plt.show()

    return acc_w



def init_from_static(
    imu9: np.ndarray,
    fs: float,
    *,
    n_first: int = 200,
    n_init: int = 400,
    acc_unit: str = "g",
    gyr_unit: str = "deg",
    use_mag: bool = False,
):
    """
    用前 n_init 个样本的静止段初始化：
    - q0：由 acc（可选 mag）给出
    - gyro_bias：静止段陀螺均值
    """
    imu9 = np.asarray(imu9, float)
    a = imu9[n_first:n_init, 0:3].copy()
    w = imu9[n_first:n_init, 3:6].copy()
    m = imu9[n_first:n_init, 6:9].copy()

    # 单位
    if acc_unit.lower() in ["g", "grav"]:
        a = a * G0
    if gyr_unit.lower().startswith("deg"):
        w = np.deg2rad(w)

    a_mean = a.mean(axis=0)
    w_bias = w.mean(axis=0)

    # q0：仅靠 acc 得到俯仰/横滚；如果 use_mag=True 则带 yaw
    if use_mag:
        m_mean = m.mean(axis=0)
        q0 = am2q(a_mean, m_mean)   # wxyz
    else:
        q0 = acc2q(a_mean)          # wxyz

    q0 = np.asarray(q0, float)
    q0 = q0 / (np.linalg.norm(q0) + 1e-12)

    return q0, w_bias

def is_static_segment(imu9, fs, n=200, gyr_thr_dps=5.0, acc_std_thr_g=0.03):
    a = imu9[:n,0:3]
    w = imu9[:n,3:6]
    acc_norm = np.linalg.norm(a, axis=1)
    gyr_norm = np.linalg.norm(w, axis=1)

    ok1 = np.median(gyr_norm) < gyr_thr_dps   # deg/s（你的原始单位若是deg）
    ok2 = np.std(acc_norm) < acc_std_thr_g    # g
    return bool(ok1 and ok2)

class MahonyOrientationNode:
    """
    管理一个传感器节点的姿态（四元数序列 + 在线更新）
    - 支持 IMU 模式（acc+gyr）和 MARG 模式（acc+gyr+mag）
    - 输出四元数格式：wxyz
    """
    def __init__(
        self,
        name: str,
        fs: float,
        *,
        use_mag: bool = False,
        acc_unit: str = "g",     # "g" or "mps2"
        gyr_unit: str = "deg",   # "deg" or "rad"
        kp: float = 0.5,
        ki: float = 0.1,         # 零偏在线估计 — 原 0.05 过小导致收敛极慢
        q0: np.ndarray | None = None,   # ✅ 设为可选
        gyro_bias: np.ndarray| None = None,
    ):
        self.name = name
        self.fs = float(fs)
        self.use_mag = bool(use_mag)
        self.acc_unit = acc_unit
        self.gyr_unit = gyr_unit
        self.kp = float(kp)
        self.ki = float(ki) if float(ki) > 0.0 else 1e-6
        self.gyro_bias = np.zeros(3) if gyro_bias is None else np.asarray(gyro_bias, float)

        self.filter = Mahony(frequency=self.fs, k_P=self.kp, k_I=self.ki,
                            b0=self.gyro_bias.copy())

        if q0 is None:
            self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        else:
            self.q = np.asarray(q0, dtype=float)
            self.q = self.q / (np.linalg.norm(self.q) + 1e-12)

        # 历史缓存（按需开）
        self.Q = None  # (N,4)

    def _prep_units(self, acc3, gyr3, mag3=None):
        acc3 = np.asarray(acc3, float)
        gyr3 = np.asarray(gyr3, float)

        # acc -> m/s^2
        if self.acc_unit.lower() in ["g", "grav"]:
            acc3 = acc3 * G0

        # gyr -> rad/s
        if self.gyr_unit.lower().startswith("deg"):
            gyr3 = np.deg2rad(gyr3)

        if mag3 is not None:
            mag3 = np.asarray(mag3, float)

        return acc3, gyr3, mag3

    def update_one(self, acc3, gyr3, mag3=None) -> np.ndarray:
        """
        更新一帧，返回当前四元数 wxyz
        """
        acc3, gyr3, mag3 = self._prep_units(acc3, gyr3, mag3)

        # 注意：陀螺零偏由 Mahony 滤波器内部 b 估计处理（已通过 b0 注入初始值）
        # 不在此处做外部减法，避免与 filter.b 叠加导致双重扣除

        if self.use_mag and mag3 is not None:
            q_new = self.filter.updateMARG(self.q, gyr=gyr3, acc=acc3, mag=mag3)
        else:
            q_new = self.filter.updateIMU(self.q, gyr=gyr3, acc=acc3)

        q_new = np.asarray(q_new, float)
        q_new = q_new / (np.linalg.norm(q_new) + 1e-12)

        self.q = q_new
        return self.q

    def run_batch(self, imu9: np.ndarray) -> np.ndarray:
        """
        imu9: (N,9) = [acc(3), gyr(3), mag(3)]
        返回 Q: (N,4) wxyz
        """
        imu9 = np.asarray(imu9, float)
        if imu9.ndim != 2 or imu9.shape[1] != 9:
            raise ValueError(f"{self.name}: imu9 must be (N,9)")

        N = imu9.shape[0]
        Q = np.zeros((N, 4), dtype=float)

        for i in range(N):
            acc3 = imu9[i, 0:3]
            gyr3 = imu9[i, 3:6]
            mag3 = imu9[i, 6:9] if self.use_mag else None
            Q[i] = self.update_one(acc3, gyr3, mag3)

        self.Q = Q
        return Q

    def init_from_static(
            self,
            imu9: np.ndarray,
            *,
            n_first: int = 200,
            n_init: int = 400,
            estimate_gyro_bias: bool = False,
    ):
        """
        使用一段静止 IMU 数据初始化姿态
        - 设置 self.q（传感器 → 世界）
        - （可选）设置 self.gyro_bias

        imu9: (N,9) = [acc, gyr, mag]
        """
        imu9 = np.asarray(imu9, float)

        a = imu9[n_first:n_init, 0:3]
        w = imu9[n_first:n_init, 3:6]
        m = imu9[n_first:n_init, 6:9] if self.use_mag else None

        # ===== 单位处理 =====
        if self.acc_unit.lower() in ["g", "grav"]:
            a = a * G0
        if self.gyr_unit.lower().startswith("deg"):
            w = np.deg2rad(w)

        a_mean = a.mean(axis=0)

        # ===== 姿态初始化 =====
        if self.use_mag and m is not None:
            m_mean = m.mean(axis=0)
            q0 = am2q(a_mean, m_mean)
        else:
            q0 = acc2q(a_mean)

        q0 = np.asarray(q0, float)
        self.q = q0 / (np.linalg.norm(q0) + 1e-12)

        # ===== 陀螺零偏（可选）=====
        if estimate_gyro_bias:
            self.gyro_bias = w.mean(axis=0)
            # 同步注入 Mahony 滤波器内部零偏估计，加速收敛
            self.filter.b = self.gyro_bias.copy()

        return self.q


class QuaternionManager:
    """
    管理多个节点（左脚/右脚/腰部...）
    - 每个节点一个 MahonyOrientationNode
    - 统一批处理、取结果
    """
    def __init__(self, fs: float):
        self.fs = float(fs)
        self.nodes: dict[str, MahonyOrientationNode] = {}

    def add_existing_node(self, node: MahonyOrientationNode):
        """
        注册一个已经初始化好的 MahonyOrientationNode
        """
        if node.name in self.nodes:
            raise ValueError(f"Node '{node.name}' already exists.")
        self.nodes[node.name] = node
    def add_node(
        self,
        name: str,
        *,
        use_mag: bool = False,
        acc_unit: str = "g",
        gyr_unit: str = "deg",
        kp: float = 1.0,
        ki: float = 0.001,
        q0: np.ndarray | None = None,
    ):
        if name in self.nodes:
            raise ValueError(f"Node '{name}' already exists.")
        self.nodes[name] = MahonyOrientationNode(
            name=name, fs=self.fs,
            use_mag=use_mag, acc_unit=acc_unit, gyr_unit=gyr_unit,
            kp=kp, ki=ki, q0=q0
        )

    def run_batch(self, data_by_node: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
        """
        data_by_node: {name: imu9(N,9)}
        return: {name: quat_wxyz(N,4)}
        """
        out = {}
        for name, imu9 in data_by_node.items():
            if name not in self.nodes:
                raise ValueError(f"Node '{name}' not registered in manager.")
            out[name] = self.nodes[name].run_batch(imu9)
        return out

    def get_quat(self, name: str) -> np.ndarray:
        return self.nodes[name].Q

    def get_last(self, name: str) -> np.ndarray:
        return self.nodes[name].q


#
# arr1, arr2, arr3 = load_calibrated_filtered_arrays(
#         window_size=5,
#         columns=["Acc_x","Acc_y","Acc_z","Gyr_x","Gyr_y","Gyr_z","Geo_x","Geo_y","Geo_z"],
#         aliases=["S1", "R6", "L6"]
#     )
# # q0_L, wb_L = init_from_static(arr3, fs=50,n_first=200, n_init=400, acc_unit="g", gyr_unit="deg", use_mag=False)
# #
# # mgr = QuaternionManager(fs=50)
# #
# # # 步态用：脚部建议先关磁力计（抗干扰更稳）
# # mgr.add_node("L_foot", use_mag=False, kp=0.8, ki=1e-5,q0=q0_L)
# #
# # mgr.add_node("R_foot", use_mag=False, kp=0.8, ki=1e-5)
# #
# # # 腰部如果你想要航向更稳定，可以开磁力计（可选）
# # mgr.add_node("Waist", use_mag=True, kp=0.6, ki=1e-5)
# #
# # Q = mgr.run_batch({
# #     "L_foot": arr3,
# #     "R_foot": arr2,
# #     "Waist":  arr1,
# # })
# #
# # quat_L_wxyz = Q["L_foot"]
# # quat_R_wxyz = Q["R_foot"]
# # quat_W_wxyz = Q["Waist"]
# #
# # print(quat_L_wxyz.shape, quat_R_wxyz.shape, quat_W_wxyz.shape)  # (N,4)
# #
# # norms = np.linalg.norm(quat_W_wxyz, axis=1)
# # print(norms.min(), norms.max())
#
#
# mgr = QuaternionManager(fs=50)
# node = MahonyOrientationNode(
#     name="L_foot",
#     fs=50,
#     use_mag=False,
#     acc_unit="g",
#     gyr_unit="deg",
#     kp=0.8,
#     ki=1e-5,
# )
#
# node.init_from_static(imu9=arr3)
#
# mgr.add_existing_node(node)
#
# Q = mgr.run_batch({"L_foot": arr3})
#
# quat_L_wxyz = Q["L_foot"]
#
#
# # 看两种方向哪个对
# acc_w1 = plot_acc_world_check(arr3, quat_L_wxyz, fs=50, direction="world<-sensor", acc_unit="g", sec=4.0)
# acc_w2 = plot_acc_world_check(arr3, quat_L_wxyz, fs=50, direction="sensor<-world", acc_unit="g", sec=4.0)
#
# acc_w = rotate_acc_to_world(arr3[:,0:3], quat_L_wxyz, direction="world<-sensor", acc_unit="g")
#
#
# acc_lin_w, g_vec, bias = remove_gravity_and_static_bias(acc_w, fs=50, n_init=200)
# print("g_vec:", g_vec, "bias:", bias)
