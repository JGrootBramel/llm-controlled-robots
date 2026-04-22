"""Unit tests for ``_helpers.standoff``."""

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

from _helpers import standoff  # noqa: E402


def test_zero_offset_is_behind_target_along_base_yaw():
    candidates = standoff.standoff_candidates(
        tx=1.0, ty=0.0, base_yaw=0.0, distance_m=0.5,
        arc_deg=10.0, max_arc_deg=0.0,
    )
    assert len(candidates) == 1
    gx, gy, yaw = candidates[0]
    assert math.isclose(gx, 0.5, abs_tol=1e-6)
    assert math.isclose(gy, 0.0, abs_tol=1e-6)
    # yaw should face the target (pointing from robot to target).
    assert math.isclose(yaw, 0.0, abs_tol=1e-6)


def test_candidates_span_configured_arc():
    candidates = standoff.standoff_candidates(
        tx=0.0, ty=0.0, base_yaw=0.0, distance_m=1.0,
        arc_deg=10.0, max_arc_deg=20.0,
    )
    # center + two pairs of +/- offsets = 5
    assert len(candidates) == 5
    # Every candidate is 1 m from origin.
    for gx, gy, _ in candidates:
        assert math.isclose(math.hypot(gx, gy), 1.0, abs_tol=1e-6)


def test_candidates_yaw_points_at_target():
    candidates = standoff.standoff_candidates(
        tx=2.0, ty=1.0, base_yaw=0.3, distance_m=0.6,
        arc_deg=15.0, max_arc_deg=30.0,
    )
    for gx, gy, yaw in candidates:
        expected = math.atan2(1.0 - gy, 2.0 - gx)
        assert math.isclose(yaw, expected, abs_tol=1e-6)


def test_iter_matches_list():
    args = dict(tx=0.0, ty=0.0, base_yaw=0.0, distance_m=0.5,
                arc_deg=10.0, max_arc_deg=20.0)
    as_list = standoff.standoff_candidates(**args)
    as_iter = list(standoff.iter_standoff_candidates(**args))
    assert as_list == as_iter
