"""Pure math helpers for the MyCobot 280 arm.

Kept free of ``pymycobot`` so unit tests don't need the hardware library.
Motion primitives (actually talking to the arm) live in
``arm_control_node.py``.

Frames:

* ``base_link``  — robot base (X fwd, Y left, Z up), metres.
* arm            — MyCobot native frame, millimetres, Z up. The arm's
                   native ``+X_arm`` axis points in whatever direction the
                   physical base is facing. :data:`DEFAULT_MOUNT_YAW_DEG`
                   captures the yaw (CCW, degrees) from ``base_link``'s +X
                   to the arm's +X axis.

                   On the LIMO Cobot the arm is bolted 90 degrees CCW, so
                   ``+X_arm`` points to the robot's left (+y_base). That
                   gives the legacy mapping ``X_arm = +y_base,
                   Y_arm = -x_base, Z_arm = +z_base`` as a special case of
                   the general rotation implemented here.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple


# Default physical mount rotation: arm +X points to robot's left (90 deg CCW).
DEFAULT_MOUNT_YAW_DEG: float = 90.0


# ---- coordinate mapping -----------------------------------------------------


def base_to_arm_mm(
    x_m: float,
    y_m: float,
    z_m: float,
    mount_yaw_deg: float = DEFAULT_MOUNT_YAW_DEG,
) -> Tuple[float, float, float]:
    """Map a point from ``base_link`` (metres) to arm frame (millimetres).

    ``mount_yaw_deg`` is the CCW yaw from ``base_link``'s +X to the arm's
    native +X axis. We rotate the base-frame point by ``-mount_yaw_deg``
    (i.e. into the arm frame) and then convert to millimetres.

    With ``mount_yaw_deg = 90`` this reduces to the legacy mapping
    ``X_arm = +y_base, Y_arm = -x_base, Z_arm = +z_base`` used by the
    original ``pick_cube_blue_node``.
    """
    x_mm = float(x_m) * 1000.0
    y_mm = float(y_m) * 1000.0
    z_mm = float(z_m) * 1000.0
    yaw = math.radians(float(mount_yaw_deg))
    c = math.cos(-yaw)
    s = math.sin(-yaw)
    x_arm = c * x_mm - s * y_mm
    y_arm = s * x_mm + c * y_mm
    return x_arm, y_arm, z_mm


# ---- workspace clamp --------------------------------------------------------


@dataclass(frozen=True)
class GraspLimits:
    """Safe cube for MyCobot Cartesian moves, in millimetres.

    Applies to the *final* pose. ``pre_dy`` / ``pre_dz`` are pre-grasp
    offsets that must also stay inside the cube.
    """

    coord_lim_mm: float = 280.0
    pre_dy_mm: float = 30.0
    pre_dz_mm: float = 5.0


def clamp_grasp_coords_mm(
    x: float, y: float, z: float, limits: GraspLimits = GraspLimits()
) -> Tuple[float, float, float, bool]:
    """Clamp a target (and its pre-grasp offset) into the safe arm cube.

    Returns ``(x', y', z', clipped)`` where ``clipped`` is True if any axis
    was modified.
    """
    lim = limits.coord_lim_mm
    dy = limits.pre_dy_mm
    dz = limits.pre_dz_mm
    y_hi = min(lim, lim - dy)
    y_lo = max(-lim, -lim - dy)
    z_hi = min(lim, lim - dz)
    z_lo = max(-lim, -lim - dz)
    cx = max(-lim, min(lim, float(x)))
    cy = max(y_lo, min(y_hi, float(y)))
    cz = max(z_lo, min(z_hi, float(z)))
    clipped = (cx != float(x)) or (cy != float(y)) or (cz != float(z))
    return cx, cy, cz, clipped


# ---- canonical poses -------------------------------------------------------


# Copied verbatim from the old ``pick_cube_blue_node`` so the node can call
# into a single source of truth.
GRASP_RXRYRZ: Tuple[float, float, float] = (-110.0, 45.0, 165.0)


def default_home_angles(
    mount_yaw_deg: float = DEFAULT_MOUNT_YAW_DEG,
) -> Tuple[float, float, float, float, float, float]:
    """Home pose with joint 1 rotated to undo the physical mount yaw.

    At joint 1 = ``-mount_yaw_deg`` the arm's native +X axis points along
    ``base_link``'s +X — i.e. the gripper faces forward from the robot's
    point of view, even though the arm itself is mounted rotated.
    """
    return (-float(mount_yaw_deg), 0.0, 0.0, 0.0, 0.0, 0.0)


# Backwards-compat constant; equals ``default_home_angles(90)`` for the
# LIMO Cobot's 90 deg CCW mount. Code that needs a mount-aware default
# should call :func:`default_home_angles` instead.
HOME_ANGLES: Tuple[float, float, float, float, float, float] = (
    default_home_angles()
)


def default_place_angles(
    mount_yaw_deg: float = DEFAULT_MOUNT_YAW_DEG,
) -> Tuple[float, float, float, float, float, float]:
    """Drop pose rotated to keep the cube going over the same physical tray.

    The legacy place pose put joint 1 at +60 deg — i.e. 60 deg CCW in the
    arm's native frame, which on the default (90 deg) mount corresponds
    to 150 deg CCW in ``base_link`` (60 + 90). We preserve that absolute
    direction by keeping joint 1 at ``(60 - mount_yaw_deg)`` so changing
    the mount doesn't silently change where the cube lands.
    """
    return (60.0 - float(mount_yaw_deg), -30.0, -60.0, 90.0, 0.0, 45.0)


PLACE_ANGLES: Tuple[float, float, float, float, float, float] = (
    default_place_angles()
)
