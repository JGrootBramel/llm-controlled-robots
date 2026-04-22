"""Unit tests for ``_helpers.projection`` (no ROS/TF)."""

from __future__ import annotations

import math
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

from _helpers import projection as proj  # noqa: E402


def test_pixel_to_camera_center_is_on_optical_axis():
    x, y, z = proj.pixel_to_camera(u=320, v=240, z=1.5, fx=500, fy=500, cx=320, cy=240)
    assert math.isclose(x, 0.0, abs_tol=1e-9)
    assert math.isclose(y, 0.0, abs_tol=1e-9)
    assert math.isclose(z, 1.5, abs_tol=1e-9)


def test_pixel_to_camera_off_axis():
    x, y, z = proj.pixel_to_camera(u=420, v=140, z=2.0, fx=500, fy=500, cx=320, cy=240)
    # (100, -100) px at Z=2.0 m, fx=fy=500 -> (0.4, -0.4, 2.0)
    assert math.isclose(x, 0.4, abs_tol=1e-6)
    assert math.isclose(y, -0.4, abs_tol=1e-6)
    assert math.isclose(z, 2.0, abs_tol=1e-6)


def test_camera_pixel_to_frame_uses_transform_callback():
    def fake_tf(point, src, tgt, stamp):
        assert src == "camera"
        assert tgt == "base_link"
        x, y, z = point
        return (z, -x, -y)  # ROS optical -> base_link convention

    out = proj.camera_pixel_to_frame(
        u=320, v=240, z=1.2, fx=500, fy=500, cx=320, cy=240,
        source_frame="camera", target_frame="base_link",
        transform_point_fn=fake_tf,
    )
    assert out == (1.2, 0.0, 0.0)


def test_camera_pixel_to_frame_handles_tf_failure():
    def boom(*_a, **_kw):
        raise RuntimeError("TF lookup failed")

    out = proj.camera_pixel_to_frame(
        u=0, v=0, z=1.0, fx=500, fy=500, cx=320, cy=240,
        source_frame="camera", target_frame="map",
        transform_point_fn=boom,
    )
    assert out is None
