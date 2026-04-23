"""Unit tests for ``_helpers.mycobot_helpers``."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

pytestmark = pytest.mark.unit

_HELPERS = (
    Path(__file__).resolve().parents[2]
    / "catkin_ws"
    / "src"
    / "limo_rosa_bridge"
    / "scripts"
)
if str(_HELPERS) not in sys.path:
    sys.path.insert(0, str(_HELPERS))

from _helpers import mycobot_helpers as mch  # noqa: E402


def test_base_to_arm_mm_units_and_axes():
    x, y, z = mch.base_to_arm_mm(0.1, 0.2, 0.3)
    # With the default 90 deg CCW mount: X_arm = +y_base, Y_arm = -x_base,
    # Z_arm = +z_base. Tolerance absorbs the sin/cos rounding.
    assert x == pytest.approx(200.0, abs=1e-9)
    assert y == pytest.approx(-100.0, abs=1e-9)
    assert z == pytest.approx(300.0, abs=1e-9)


def test_base_to_arm_mm_zero_mount_is_identity_in_mm():
    # mount_yaw_deg=0 means arm +X == base_link +X, so only units change.
    x, y, z = mch.base_to_arm_mm(0.1, 0.2, 0.3, mount_yaw_deg=0.0)
    assert x == pytest.approx(100.0, abs=1e-9)
    assert y == pytest.approx(200.0, abs=1e-9)
    assert z == pytest.approx(300.0, abs=1e-9)


def test_base_to_arm_mm_forward_cube_maps_to_minus_y_on_default_mount():
    # A cube 0.3 m in front of the robot lands at (0, -300, 0) in arm frame
    # when the arm is bolted 90 deg CCW. That's why pre_dy is applied on +Y.
    x, y, z = mch.base_to_arm_mm(0.3, 0.0, 0.0)
    assert x == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(-300.0, abs=1e-9)
    assert z == pytest.approx(0.0, abs=1e-9)


def test_default_home_angles_rotate_joint1_to_cancel_mount():
    # Joint 1 compensates the physical mount so the gripper faces forward.
    assert mch.default_home_angles(90.0)[0] == pytest.approx(-90.0)
    assert mch.default_home_angles(0.0)[0] == pytest.approx(0.0)
    assert mch.default_home_angles(-45.0)[0] == pytest.approx(45.0)
    # Other joints stay at 0.
    assert mch.default_home_angles(90.0)[1:] == (0.0, 0.0, 0.0, 0.0, 0.0)


def test_clamp_grasp_coords_inside_cube_unchanged():
    lim = mch.GraspLimits()
    out = mch.clamp_grasp_coords_mm(100.0, 50.0, 10.0, lim)
    assert out[0] == 100.0 and out[1] == 50.0 and out[2] == 10.0
    assert out[3] is False


def test_clamp_grasp_coords_outside_reports_clipped():
    lim = mch.GraspLimits(coord_lim_mm=280.0, pre_dy_mm=30.0, pre_dz_mm=5.0)
    out = mch.clamp_grasp_coords_mm(500.0, -500.0, 500.0, lim)
    assert out[3] is True
    assert out[0] == 280.0  # clipped to +coord_lim
    # Bounds are the intersection of [-lim, lim] and [-lim-dy, lim-dy];
    # the resulting low side is -lim because max(-lim, -lim-dy) == -lim.
    assert out[1] == -280.0
    # z_hi = min(lim, lim - dz) = lim - dz = 275
    assert out[2] == 275.0


def test_canonical_poses_shape():
    assert len(mch.HOME_ANGLES) == 6
    assert len(mch.PLACE_ANGLES) == 6
    assert len(mch.GRASP_RXRYRZ) == 3
