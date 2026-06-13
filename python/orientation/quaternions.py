# Backward-compatible re-exports. The module was renamed to quaternion_manager.
# quat_* helpers live in core.quaternion.

from core.quaternion import (
    quat_normalize,
    quat_conj,
    quat_inv,
    quat_mul,
    quat_rotate,
    quat_to_rotmat,
    quat_relative,
    average_quaternions,
    quat_align_hemisphere,
    enforce_quat_continuity,
    quat_to_euler,
    euler_to_quat,
    wrap_to_pi,
    wrap_to_180,
    R_to_quat,
)

from orientation.quaternion_manager import (
    QuaternionManager,
    MahonyOrientationNode,
    rotate_acc_to_world,
    remove_gravity_and_static_bias,
    init_from_static,
)
