"""HSV color-blob helpers extracted from the old ``pick_cube_blue_node``.

Pure NumPy / OpenCV. No ROS, no TF.

Usage example::

    mask = red_mask(hsv_image)
    u, v, area = largest_blob_centroid(mask, min_area=200, max_area=3500)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

try:
    import cv2  # type: ignore
except Exception:  # pragma: no cover - cv2 optional on test hosts
    cv2 = None  # type: ignore


# Default HSV thresholds for STRICT red-only detection.
# Red wraps around H=0/180 in OpenCV so we need two bands.
# These tighter bounds intentionally reject orange/brown shades by requiring
# very strong saturation and brightness.
DEFAULT_RED_H1 = (0, 7)
DEFAULT_RED_H2 = (173, 180)
DEFAULT_RED_S_LOW = 140
DEFAULT_RED_V_LOW = 90

DEFAULT_BLUE_H = (105, 135)
DEFAULT_BLUE_S_LOW = 70
DEFAULT_BLUE_V_LOW = 40


@dataclass(frozen=True)
class RedThresholds:
    """HSV bounds for the red colour (two bands because H wraps)."""

    h1_low: int = DEFAULT_RED_H1[0]
    h1_high: int = DEFAULT_RED_H1[1]
    h2_low: int = DEFAULT_RED_H2[0]
    h2_high: int = DEFAULT_RED_H2[1]
    s_low: int = DEFAULT_RED_S_LOW
    v_low: int = DEFAULT_RED_V_LOW


@dataclass(frozen=True)
class BlueThresholds:
    """HSV bounds for the blue colour."""

    h_low: int = DEFAULT_BLUE_H[0]
    h_high: int = DEFAULT_BLUE_H[1]
    s_low: int = DEFAULT_BLUE_S_LOW
    v_low: int = DEFAULT_BLUE_V_LOW


def red_mask(hsv: np.ndarray, th: Optional[RedThresholds] = None) -> np.ndarray:
    """Return a uint8 mask (255 inside red hue bands, 0 otherwise)."""
    if cv2 is None:
        raise RuntimeError("red_mask requires OpenCV")
    t = th or RedThresholds()
    low1 = np.array([t.h1_low, t.s_low, t.v_low], dtype=np.uint8)
    high1 = np.array([t.h1_high, 255, 255], dtype=np.uint8)
    low2 = np.array([t.h2_low, t.s_low, t.v_low], dtype=np.uint8)
    high2 = np.array([t.h2_high, 255, 255], dtype=np.uint8)
    m1 = cv2.inRange(hsv, low1, high1)
    m2 = cv2.inRange(hsv, low2, high2)
    return cv2.bitwise_or(m1, m2)


def blue_mask(hsv: np.ndarray, th: Optional[BlueThresholds] = None) -> np.ndarray:
    """Return a uint8 mask (255 inside blue hue band, 0 otherwise)."""
    if cv2 is None:
        raise RuntimeError("blue_mask requires OpenCV")
    t = th or BlueThresholds()
    low = np.array([t.h_low, t.s_low, t.v_low], dtype=np.uint8)
    high = np.array([t.h_high, 255, 255], dtype=np.uint8)
    return cv2.inRange(hsv, low, high)


def largest_blob_centroid(
    mask: np.ndarray,
    min_area: int = 200,
    max_area: int = 3500,
) -> Optional[Tuple[int, int, float]]:
    """Pick the largest contour whose area is in ``[min_area, max_area]``.

    Returns ``(u, v, area)`` or ``None``.

    The upper cap rejects large red background regions (walls, sofa) while
    the lower cap rejects noisy specks. This is identical to the
    filter-by-area logic from the old ``pick_cube_blue_node``.
    """
    if cv2 is None:
        raise RuntimeError("largest_blob_centroid requires OpenCV")
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    for c in sorted(cnts, key=cv2.contourArea, reverse=True):
        a = float(cv2.contourArea(c))
        if a > max_area:
            continue
        if a < min_area:
            continue
        m = cv2.moments(c)
        if m["m00"] <= 0:
            continue
        u = int(m["m10"] / m["m00"])
        v = int(m["m01"] / m["m00"])
        return u, v, a
    return None


def color_to_depth_pixels(
    u_col: int, v_col: int, rgb_w: int, rgb_h: int, depth_w: int, depth_h: int
) -> Tuple[int, int]:
    """Map a colour-image pixel to depth-image indices when sizes differ.

    Using the colour ``(u, v)`` directly on a differently-sized depth image
    was sampling the wrong rows on the LIMO Astra Pro (640x480 colour vs
    640x400 depth).
    """
    if rgb_w == depth_w and rgb_h == depth_h:
        return int(u_col), int(v_col)
    ud = int(round(u_col * depth_w / float(rgb_w)))
    vd = int(round(v_col * depth_h / float(rgb_h)))
    ud = max(0, min(depth_w - 1, ud))
    vd = max(0, min(depth_h - 1, vd))
    return ud, vd


def median_depth_patch(
    depth: np.ndarray,
    u: int,
    v: int,
    patch_px: int = 9,
    depth_min: float = 0.1,
    depth_max: float = 4.5,
) -> Optional[float]:
    """Return the median depth in a square window around ``(u, v)``.

    The window is clipped to image bounds. Only finite values in
    ``(depth_min, depth_max)`` contribute. Returns ``None`` if too few
    valid pixels (< 10) are available.
    """
    win = int(patch_px)
    if win % 2 == 0:
        win += 1
    r = win // 2
    h, w = depth.shape[:2]
    x0, x1 = max(0, u - r), min(w, u + r + 1)
    y0, y1 = max(0, v - r), min(h, v + r + 1)
    patch = depth[y0:y1, x0:x1]
    valid = np.isfinite(patch) & (patch > depth_min) & (patch < depth_max)
    vals = patch[valid]
    if vals.size < 10:
        return None
    return float(np.median(vals))
