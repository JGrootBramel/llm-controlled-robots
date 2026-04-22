"""Pure math helpers for the MyCobot 280 arm.

Kept free of ``pymycobot`` so unit tests don't need the hardware library.
Motion primitives (actually talking to the arm) live in
``arm_control_node.py``.

Frame convention (matches the original ``pick_cube_blue_node``):

* ``base_link``  — robot base (X fwd, Y left, Z up), metres.
* arm           — MyCobot natively expects millimetres, with
                   ``X_arm = +y_base``, ``Y_arm = -x_base``, ``Z_arm = +z_base``.

That is the only non-trivial thing worth encapsulating.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


# ---- coordinate mapping -----------------------------------------------------


def base_to_arm_mm(x_m: float, y_m: float, z_m: float) -> Tuple[float, float, float]:
    """Map a point from ``base_link`` (metres) to arm frame (millimetres).

    X_arm = +y_base, Y_arm = -x_base, Z_arm = +z_base.
    """
    x_mm = float(x_m) * 1000.0
    y_mm = float(y_m) * 1000.0
    z_mm = float(z_m) * 1000.0
    return y_mm, -x_mm, z_mm


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
HOME_ANGLES: Tuple[float, float, float, float, float, float] = (
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
)
# Left-side tray drop pose (end of the place maneuver moves the cube over
# the side container on the LIMO Cobot).
PLACE_ANGLES: Tuple[float, float, float, float, float, float] = (
    60.0,
    -30.0,
    -60.0,
    90.0,
    0.0,
    45.0,
)
