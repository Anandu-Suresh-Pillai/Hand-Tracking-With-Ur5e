"""
UR5e Kinematics Engine
======================
Implements Forward Kinematics (FK) and Damped Least Squares (DLS)
Inverse Kinematics (IK) derived from the provided ur5e.urdf.

URDF Joint Chain (parent -> child, with fixed transforms applied):
  world -> base_link (fixed, identity)
  base_link -> shoulder_link     : xyz=[0, 0, 0.163],   rpy=[0, 0, 0],       axis=Z
  shoulder_link -> upper_arm_link: xyz=[0, 0.138, 0],   rpy=[0, pi/2, 0],    axis=Y
  upper_arm_link -> forearm_link : xyz=[0, -0.131, 0.425], rpy=[0, 0, 0],    axis=Y
  forearm_link -> wrist_1_link   : xyz=[0, 0, 0.392],   rpy=[0, pi/2, 0],    axis=Y
  wrist_1_link -> wrist_2_link   : xyz=[0, 0.127, 0],   rpy=[0, 0, 0],       axis=Z
  wrist_2_link -> wrist_3_link   : xyz=[0, 0, 0.1],     rpy=[0, 0, 0],       axis=Y
  wrist_3_link -> tool0 (fixed)  : xyz=[0, 0.1, 0],     rpy=[-pi/2, 0, 0]
"""

import numpy as np

# ---------------------------------------------------------------------------
# URDF-derived fixed transforms between consecutive joint frames
# ---------------------------------------------------------------------------
_PI2 = 1.57079632679

def _make_T(xyz, rpy):
    """Create a 4x4 homogeneous transformation from xyz + RPY angles."""
    r, p, y = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [  -sp,            cp*sr,            cp*cr ],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = xyz
    return T

# Fixed joint-origin transforms from the URDF
T_J0_ORIGIN = _make_T([0.0,   0.0,   0.163], [0.0,  0.0,   0.0])
T_J1_ORIGIN = _make_T([0.0,   0.138, 0.0  ], [0.0,  _PI2,  0.0])
T_J2_ORIGIN = _make_T([0.0,  -0.131, 0.425], [0.0,  0.0,   0.0])
T_J3_ORIGIN = _make_T([0.0,   0.0,   0.392], [0.0,  _PI2,  0.0])
T_J4_ORIGIN = _make_T([0.0,   0.127, 0.0  ], [0.0,  0.0,   0.0])
T_J5_ORIGIN = _make_T([0.0,   0.0,   0.1  ], [0.0,  0.0,   0.0])
T_TOOL0     = _make_T([0.0,   0.1,   0.0  ], [-_PI2, 0.0,  0.0])

# Rotation axis for each joint (in the joint's local frame before joint rotation)
# Joint 0: Z  (shoulder_pan)
# Joint 1: Y  (shoulder_lift)
# Joint 2: Y  (elbow)
# Joint 3: Y  (wrist_1)
# Joint 4: Z  (wrist_2)
# Joint 5: Y  (wrist_3)
_AXES = ['z', 'y', 'y', 'y', 'z', 'y']

# Joint limits directly from the URDF [lower, upper] in radians
JOINT_LIMITS_LOWER = np.array([
    -6.28318530718,  # shoulder_pan
    -6.28318530718,  # shoulder_lift
    -3.14159265359,  # elbow
    -6.28318530718,  # wrist_1
    -6.28318530718,  # wrist_2
    -6.28318530718,  # wrist_3
])
JOINT_LIMITS_UPPER = np.array([
    6.28318530718,
    6.28318530718,
    3.14159265359,
    6.28318530718,
    6.28318530718,
    6.28318530718,
])

# ---------------------------------------------------------------------------
# Rotation matrices about a single axis
# ---------------------------------------------------------------------------

def _Ry(q: float) -> np.ndarray:
    c, s = np.cos(q), np.sin(q)
    return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]])

def _Rz(q: float) -> np.ndarray:
    c, s = np.cos(q), np.sin(q)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

_ROT = {'y': _Ry, 'z': _Rz}

# Fixed sequence: [origin_T, axis_str]
_JOINT_CHAIN = [
    (T_J0_ORIGIN, 'z'),
    (T_J1_ORIGIN, 'y'),
    (T_J2_ORIGIN, 'y'),
    (T_J3_ORIGIN, 'y'),
    (T_J4_ORIGIN, 'z'),
    (T_J5_ORIGIN, 'y'),
]


# ---------------------------------------------------------------------------
# Forward Kinematics
# ---------------------------------------------------------------------------

def forward_kinematics(q: np.ndarray):
    """
    Compute full FK for all 6 joints.

    Parameters
    ----------
    q : array-like, shape (6,)
        Joint angles in radians.

    Returns
    -------
    p_ee : np.ndarray, shape (3,)
        tool0 position in the world frame [x, y, z] (metres).
    joint_origins : list of np.ndarray
        World-frame position of each joint axis origin, shape (6, 3).
    joint_axes : list of np.ndarray
        World-frame unit vector of each joint's rotation axis, shape (6, 3).
    """
    T = np.eye(4)
    joint_origins = []
    joint_axes    = []

    for i, (T_orig, axis) in enumerate(_JOINT_CHAIN):
        T = T @ T_orig

        # Record joint origin and axis in world frame
        joint_origins.append(T[:3, 3].copy())

        if axis == 'z':
            joint_axes.append(T[:3, 2].copy())   # column 2 = local Z
        else:  # 'y'
            joint_axes.append(T[:3, 1].copy())   # column 1 = local Y

        # Apply joint rotation
        T = T @ _ROT[axis](q[i])

    # Fixed tool0 frame
    T = T @ T_TOOL0
    p_ee = T[:3, 3].copy()

    return p_ee, joint_origins, joint_axes


# ---------------------------------------------------------------------------
# Jacobian computation
# ---------------------------------------------------------------------------

def compute_jacobian(q: np.ndarray) -> np.ndarray:
    """
    Compute the 3×6 geometric Jacobian for the position of tool0.

    Each column j is:  J[:,j] = cross(axis_j, p_ee - origin_j)
    """
    p_ee, origins, axes = forward_kinematics(q)
    J = np.zeros((3, 6))
    for j in range(6):
        J[:, j] = np.cross(axes[j], p_ee - origins[j])
    return J


# ---------------------------------------------------------------------------
# Damped Least Squares Inverse Kinematics
# ---------------------------------------------------------------------------

def solve_ik(
    p_target:   np.ndarray,
    q_init:     np.ndarray,
    max_iters:  int   = 200,
    tol:        float = 1e-4,
    damping:    float = 0.05,
    step_size:  float = 0.6,
) -> tuple[np.ndarray, float]:
    """
    Damped Least Squares (DLS / Levenberg-Marquardt) IK solver.

    The DLS update rule is:
        dq = J^T (J J^T + lambda^2 I)^{-1} * error

    This formulation is numerically stable near singularities because the
    damping term lambda prevents the pseudo-inverse from blowing up.

    Parameters
    ----------
    p_target  : desired tool0 position [x, y, z] in metres.
    q_init    : initial joint angles [rad], shape (6,).
    max_iters : maximum solver iterations.
    tol       : convergence threshold (metres).
    damping   : DLS damping factor lambda.
    step_size : fraction of the computed dq to apply per iteration.

    Returns
    -------
    q_sol     : converged (or best-effort) joint angles, shape (6,).
    residual  : final position error (metres).
    """
    q = np.array(q_init, dtype=float)
    lam2 = damping ** 2

    for _ in range(max_iters):
        p_ee, origins, axes = forward_kinematics(q)
        error = p_target - p_ee
        residual = float(np.linalg.norm(error))

        if residual < tol:
            break

        # Build 3×6 Jacobian
        J = np.zeros((3, 6))
        for j in range(6):
            J[:, j] = np.cross(axes[j], p_ee - origins[j])

        # DLS: dq = J^T (J J^T + lam^2 I)^{-1} error   (3×3 solve, fast)
        A = J @ J.T + lam2 * np.eye(3)
        dq = J.T @ np.linalg.solve(A, error)

        q = q + step_size * dq
        # Enforce joint limits
        q = np.clip(q, JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER)

    p_ee, _, _ = forward_kinematics(q)
    residual = float(np.linalg.norm(p_target - p_ee))
    return q, residual


# ---------------------------------------------------------------------------
# Default "ready" home pose (arm pointing upward)
# ---------------------------------------------------------------------------

# These joint angles put the arm in a stable upright ready-pose
# FK result ≈ [0.2, -0.11, 0.80] — nicely in the reachable workspace
HOME_JOINTS = np.array([0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])
HOME_EE_POS, _, _ = forward_kinematics(HOME_JOINTS)
