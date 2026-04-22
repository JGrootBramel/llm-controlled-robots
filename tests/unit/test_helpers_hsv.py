"""Unit tests for ``limo_rosa_bridge/scripts/_helpers/hsv.py``.

Pure NumPy/OpenCV logic; no ROS required.
"""

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

cv2 = pytest.importorskip("cv2")
np = pytest.importorskip("numpy")

from _helpers import hsv as hsv_helpers  # noqa: E402


def _solid_bgr(h: int, w: int, bgr):
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:, :] = bgr
    return img


def test_red_mask_fires_on_red_patch():
    # Pure red in BGR = (0, 0, 255). After BGR->HSV we should land in the
    # low-H band.
    bgr = _solid_bgr(40, 40, (0, 0, 255))
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = hsv_helpers.red_mask(hsv)
    # Full frame should be masked.
    assert mask.shape == (40, 40)
    assert int(mask.sum()) >= 40 * 40 * 255 * 0.9


def test_red_mask_rejects_blue():
    bgr = _solid_bgr(40, 40, (255, 0, 0))  # pure blue
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = hsv_helpers.red_mask(hsv)
    assert int(mask.sum()) == 0


def test_largest_blob_centroid_size_gates():
    mask = np.zeros((100, 100), dtype=np.uint8)
    # 10x10 blob (area 100) — below default min_area.
    mask[10:20, 10:20] = 255
    assert hsv_helpers.largest_blob_centroid(mask, min_area=200, max_area=3500) is None
    # 20x20 blob (area 400) — within default bounds.
    mask[:] = 0
    mask[30:50, 30:50] = 255
    got = hsv_helpers.largest_blob_centroid(mask, min_area=200, max_area=3500)
    assert got is not None
    u, v, area = got
    assert 35 <= u <= 45 and 35 <= v <= 45
    assert 350 <= area <= 450


def test_color_to_depth_pixels_downscale():
    ud, vd = hsv_helpers.color_to_depth_pixels(640, 480, 640, 480, 320, 240)
    assert ud == 319 and vd == 239  # clamped to bounds


def test_median_depth_patch_rejects_invalid():
    depth = np.full((20, 20), np.nan, dtype=np.float32)
    assert hsv_helpers.median_depth_patch(depth, 10, 10, patch_px=5) is None
    depth[:, :] = 1.25
    got = hsv_helpers.median_depth_patch(depth, 10, 10, patch_px=5)
    assert got is not None
    assert abs(got - 1.25) < 1e-6
